#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_now_base.h"
#include "uart_gnss.h"
#include "wifi_server.h"

static const char *TAG = "RTK_BASE";

// Global queues for inter-task communication
QueueHandle_t rtcm_queue;
QueueHandle_t status_queue;

void app_main(void)
{
    ESP_LOGI(TAG, "RTK Base Station Starting...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create queues
    rtcm_queue = xQueueCreate(10, 256);  // RTCM data queue
    status_queue = xQueueCreate(5, sizeof(base_status_t));  // Status queue
    
    if (rtcm_queue == NULL || status_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queues");
        return;
    }

    // Initialize components
    ESP_ERROR_CHECK(uart_gnss_init());
    ESP_ERROR_CHECK(esp_now_base_init());
    ESP_ERROR_CHECK(wifi_server_init());

    ESP_LOGI(TAG, "All components initialized successfully");
    ESP_LOGI(TAG, "Base station is ready to receive GNSS data and serve clients");
    
    // Main loop - just monitor system status
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "Base station running...");
    }
}