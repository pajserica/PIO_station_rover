#include "esp_now_base.h"
#include "esp_log.h"
#include "esp_crc.h"
#include <string.h>

static const char *TAG = "ESP_NOW_BASE";

// Rover MAC address - UPDATE THIS WITH ACTUAL ROVER MAC
uint8_t rover_mac[6] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};

static uint32_t packets_sent = 0;
static uint32_t send_errors = 0;

// ESP-NOW send callback
void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        packets_sent++;
        ESP_LOGD(TAG, "RTCM packet sent successfully. Total: %lu", packets_sent);
    } else {
        send_errors++;
        ESP_LOGW(TAG, "Failed to send RTCM packet. Errors: %lu", send_errors);
    }
    
    // Update base status
    base_status_t base_status = {0};
    base_status.rtcm_packets_sent = packets_sent;
    base_status.esp_now_errors = send_errors;
    
    if (xQueueSend(status_queue, &base_status, 0) != pdTRUE) {
        ESP_LOGD(TAG, "Status queue full, skipping update");
    }
}

esp_err_t esp_now_base_init(void)
{
    ESP_LOGI(TAG, "Initializing ESP-NOW for base station...");
    
    // Initialize WiFi in STA mode for ESP-NOW
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Set WiFi channel (must match rover)
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    
    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(esp_now_send_cb));
    
    // Add rover as peer
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, rover_mac, 6);
    peer_info.channel = 1;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;  // No encryption for simplicity
    
    esp_err_t ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add rover peer: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "ESP-NOW initialized successfully");
    ESP_LOGI(TAG, "Rover MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
             rover_mac[0], rover_mac[1], rover_mac[2], 
             rover_mac[3], rover_mac[4], rover_mac[5]);
    
    return ESP_OK;
}

esp_err_t send_rtcm_to_rover(const uint8_t* data, uint16_t len)
{
    if (data == NULL || len == 0 || len > 250) {
        ESP_LOGE(TAG, "Invalid RTCM data parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = esp_now_send(rover_mac, data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send RTCM data: %s", esp_err_to_name(ret));
        send_errors++;
    }
    
    return ret;
}