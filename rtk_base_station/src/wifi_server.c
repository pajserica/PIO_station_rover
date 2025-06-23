#include "wifi_server.h"
#include "esp_now_base.h"
#include "esp_log.h"
#include "esp_system.h"
#include <string.h>

static const char *TAG = "WIFI_SERVER";
static httpd_handle_t server = NULL;

// Simple HTML template for status page
static const char* html_template = 
"<!DOCTYPE html>"
"<html>"
"<head>"
"    <title>RTK Base Station</title>"
"    <meta name='viewport' content='width=device-width, initial-scale=1'>"
"    <style>"
"        body { font-family: Arial; margin: 20px; background: #f0f0f0; }"
"        .container { max-width: 600px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }"
"        .status { background: #e8f5e8; padding: 10px; border-radius: 5px; margin: 10px 0; }"
"        .error { background: #ffe8e8; }"
"        h1 { color: #333; text-align: center; }"
"        .refresh { text-align: center; margin: 20px 0; }"
"        button { background: #4CAF50; color: white; padding: 10px 20px; border: none; border-radius: 5px; cursor: pointer; }"
"    "</style>"
"    <script>"
"        function refreshStatus() { location.reload(); }"
"        setInterval(refreshStatus, 5000);"  // Auto refresh every 5 seconds
"    "</script>"
"</head>"
"<body>"
"    <div class='container'>"
"        <h1>🛰️ RTK Base Station</h1>"
"        <div class='status'>"
"            <h3>Status</h3>"
"            <p><strong>Sistem:</strong> Aktivan</p>"
"            <p><strong>RTCM paketi poslani:</strong> %lu</p>"
"            <p><strong>ESP-NOW greške:</strong> %lu</p>"
"            <p><strong>Uptime:</strong> %lu sekundi</p>"
"        </div>"
"        <div class='refresh'>"
"            <button onclick='refreshStatus()'>🔄 Osveži</button>"
"        </div>"
"    </div>"
"</body>"
"</html>";

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Client connected. MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5]);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Client disconnected. MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5]);
    }
}

esp_err_t wifi_server_init(void)
{
    ESP_LOGI(TAG, "Initializing WiFi Access Point...");
    
    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    
    // WiFi configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,
                                             ESP_EVENT_ANY_ID,
                                             &wifi_event_handler,
                                             NULL));
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = WIFI_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    
    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi AP started. SSID: %s, Password: %s", WIFI_SSID, WIFI_PASS);
    
    // Start HTTP server
    return start_webserver();
}

esp_err_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = HTTP_SERVER_PORT;
    config.lru_purge_enable = true;
    
    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    
    if (httpd_start(&server, &config) == ESP_OK) {
        // Register URI handlers
        httpd_uri_t root_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root_uri);
        
        httpd_uri_t status_uri = {
            .uri = "/status",
            .method = HTTP_GET,
            .handler = status_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &status_uri);
        
        ESP_LOGI(TAG, "HTTP server started successfully");
        return ESP_OK;
    }
    
    ESP_LOGE(TAG, "Failed to start HTTP server");
    return ESP_FAIL;
}

esp_err_t root_handler(httpd_req_t *req)
{
    base_status_t status = {0};
    
    // Try to get latest status from queue (non-blocking)
    if (xQueuePeek(status_queue, &status, 0) != pdTRUE) {
        ESP_LOGD(TAG, "No status data available, using defaults");
    }
    
    // Get system uptime
    uint32_t uptime = esp_timer_get_time() / 1000000;  // Convert to seconds
    
    // Prepare response
    char* response = malloc(2048);
    if (response == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    snprintf(response, 2048, html_template, 
             status.rtcm_packets_sent,
             status.esp_now_errors,
             uptime);
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, response, strlen(response));
    
    free(response);
    return ESP_OK;
}

esp_err_t status_handler(httpd_req_t *req)
{
    base_status_t status = {0};
    
    // Try to get latest status from queue (non-blocking)
    if (xQueuePeek(status_queue, &status, 0) != pdTRUE) {
        ESP_LOGD(TAG, "No status data available");
    }
    
    // Create JSON response
    char json_response[512];
    snprintf(json_response, sizeof(json_response),
             "{"
             "\"rtcm_packets_sent\":%lu,"
             "\"esp_now_errors\":%lu,"
             "\"satellites\":%d,"
             "\"uptime\":%lu,"
             "\"system_status\":\"active\""
             "}",
             status.rtcm_packets_sent,
             status.esp_now_errors,
             status.satellites_count,
             esp_timer_get_time() / 1000000);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, strlen(json_response));
    
    return ESP_OK;
}

esp_err_t stop_webserver(void)
{
    if (server) {
        return httpd_stop(server);
    }
    return ESP_OK;
}