#ifndef UART_GNSS_H
#define UART_GNSS_H

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

// UART configuration for LC29HBS
#define UART_PORT_NUM      UART_NUM_2
#define UART_BAUD_RATE     115200
#define UART_TX_PIN        17
#define UART_RX_PIN        16
#define UART_RTS_PIN       UART_PIN_NO_CHANGE
#define UART_CTS_PIN       UART_PIN_NO_CHANGE

#define BUF_SIZE           1024
#define RTCM_MAX_SIZE      256

// RTCM message detection
#define RTCM_PREAMBLE      0xD3

// External queue handle
extern QueueHandle_t rtcm_queue;

// Function prototypes
esp_err_t uart_gnss_init(void);
void uart_gnss_task(void *pvParameters);
bool is_rtcm_message(const uint8_t* data, uint16_t len);
uint16_t extract_rtcm_length(const uint8_t* data);

#endif // UART_GNSS_H