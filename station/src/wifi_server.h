#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_event.h"

// WiFi Access Point configuration
#define WIFI_SSID           "RTK_Base_Station"
#define WIFI_PASS           "rtk12345"
#define WIFI_CHANNEL        1
#define WIFI_MAX_CONN       4

// HTTP server configuration
#define HTTP_SERVER_PORT    80

// External queue handle
extern QueueHandle_t status_queue;

// Function prototypes
esp_err_t wifi_server_init(void);
esp_err_t start_webserver(void);
esp_err_t stop_webserver(void);

// HTTP handlers
esp_err_t root_handler(httpd_req_t *req);
esp_err_t status_handler(httpd_req_t *req);

#endif // WIFI_SERVER_H