#ifndef ESP_NOW_BASE_H
#define ESP_NOW_BASE_H

#include "esp_now.h"
#include "esp_wifi.h"

// Rover MAC address (you'll need to update this with actual rover MAC)
extern uint8_t rover_mac[6];

// RTCM data structure
typedef struct {
    uint8_t data[256];
    uint16_t length;
    uint32_t timestamp;
} rtcm_packet_t;

// Base station status
typedef struct {
    uint8_t satellites_count;
    uint8_t fix_quality;
    float latitude;
    float longitude;
    uint32_t rtcm_packets_sent;
    uint32_t esp_now_errors;
} base_status_t;

// External queue handle
extern QueueHandle_t rtcm_queue;
extern QueueHandle_t status_queue;

// Function prototypes
esp_err_t esp_now_base_init(void);
esp_err_t send_rtcm_to_rover(const uint8_t* data, uint16_t len);
void esp_now_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

#endif // ESP_NOW_BASE_H