#include "uart_gnss.h"
#include "esp_now_base.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "UART_GNSS";

esp_err_t uart_gnss_init(void)
{
    ESP_LOGI(TAG, "Initializing UART for LC29HBS GNSS module...");
    
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_RTS_PIN, UART_CTS_PIN));
    
    // Create UART reading task
    BaseType_t ret = xTaskCreatePinnedToCore(
        uart_gnss_task,
        "uart_gnss_task",
        4096,
        NULL,
        5,  // High priority for real-time GNSS data
        NULL,
        1   // Pin to core 1
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART GNSS task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "UART GNSS initialized successfully");
    return ESP_OK;
}

void uart_gnss_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    uint8_t rtcm_buffer[RTCM_MAX_SIZE];
    uint16_t rtcm_index = 0;
    bool rtcm_receiving = false;
    uint16_t expected_rtcm_length = 0;
    
    ESP_LOGI(TAG, "UART GNSS task started");
    
    while (1) {
        // Read data from UART
        int len = uart_read_bytes(UART_PORT_NUM, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            // Process each byte for RTCM messages
            for (int i = 0; i < len; i++) {
                uint8_t byte = data[i];
                
                if (!rtcm_receiving) {
                    // Look for RTCM preamble (0xD3)
                    if (byte == RTCM_PREAMBLE) {
                        rtcm_buffer[0] = byte;
                        rtcm_index = 1;
                        rtcm_receiving = true;
                        ESP_LOGD(TAG, "RTCM message start detected");
                    }
                } else {
                    // Collecting RTCM message
                    if (rtcm_index < RTCM_MAX_SIZE) {
                        rtcm_buffer[rtcm_index++] = byte;
                        
                        // After 3 bytes, we can determine the message length
                        if (rtcm_index == 3) {
                            expected_rtcm_length = extract_rtcm_length(rtcm_buffer);
                            ESP_LOGD(TAG, "RTCM message length: %d", expected_rtcm_length);
                        }
                        
                        // Check if we have received the complete message
                        if (rtcm_index >= 6 && rtcm_index >= (expected_rtcm_length + 6)) {
                            // Complete RTCM message received
                            ESP_LOGI(TAG, "Complete RTCM message received, length: %d", rtcm_index);
                            
                            // Send RTCM data to rover via ESP-NOW
                            if (send_rtcm_to_rover(rtcm_buffer, rtcm_index) == ESP_OK) {
                                ESP_LOGD(TAG, "RTCM data sent to rover");
                            }
                            
                            // Reset for next message
                            rtcm_receiving = false;
                            rtcm_index = 0;
                            expected_rtcm_length = 0;
                        }
                    } else {
                        // Buffer overflow, reset
                        ESP_LOGW(TAG, "RTCM buffer overflow, resetting");
                        rtcm_receiving = false;
                        rtcm_index = 0;
                        expected_rtcm_length = 0;
                    }
                }
            }
        }
        
        // Small delay to prevent watchdog issues
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    free(data);
    vTaskDelete(NULL);
}

bool is_rtcm_message(const uint8_t* data, uint16_t len)
{
    if (len < 3) return false;
    
    // Check RTCM preamble
    if (data[0] != RTCM_PREAMBLE) return false;
    
    // Basic RTCM format validation
    uint16_t length = extract_rtcm_length(data);
    return (length > 0 && length < 1024);
}

uint16_t extract_rtcm_length(const uint8_t* data)
{
    if (data == NULL) return 0;
    
    // RTCM length is in bytes 1-2 (after preamble)
    // Length is 10 bits: 6 bits from byte 1 + 4 bits from byte 2
    uint16_t length = ((data[1] & 0x03) << 8) | data[2];
    return length;
}