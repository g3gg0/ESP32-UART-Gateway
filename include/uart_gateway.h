#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"


/* UART Gateway default configuration */
#define UART_DEFAULT_BAUD 115200

#define UART_DEFAULT_LED_GPIO 8
#define UART_DEFAULT_TX_GPIO 20
#define UART_DEFAULT_RX_GPIO 21
#define UART_DEFAULT_RESET_GPIO 0xFF
#define UART_DEFAULT_CONTROL_GPIO 0xFF

/* Stream buffer sizes - 32 KiB each direction */
#define STREAM_BUFFER_SIZE (32 * 1024)


/* Configuration magic: 16-byte pattern for identifying config packets */
#define UART_CONFIG_MAGIC_SIZE 8
extern const uint8_t uart_config_magic[UART_CONFIG_MAGIC_SIZE];

/* Configuration packet structure - packed for binary compatibility */
typedef struct __attribute__((packed))
{
    uint8_t magic[UART_CONFIG_MAGIC_SIZE];
    uint32_t baud_rate;
    uint8_t tx_gpio;
    uint8_t rx_gpio;
    uint8_t reset_gpio;
    uint8_t control_gpio;
    uint8_t led_gpio;
    uint8_t padding[3];
    uint8_t inv_magic[UART_CONFIG_MAGIC_SIZE];
} uart_config_packet_t;

#define UART_CONFIG_PACKET_SIZE sizeof(uart_config_packet_t)

/* Control packet magic: 16-byte pattern for identifying control packets */
#define UART_CONTROL_MAGIC_SIZE 8
extern const uint8_t uart_control_magic[UART_CONTROL_MAGIC_SIZE];

/* Maximum control command string length */
#define UART_CONTROL_CMD_SIZE 16

/* Control packet structure - packed for binary compatibility */
typedef struct __attribute__((packed))
{
    uint8_t magic[UART_CONTROL_MAGIC_SIZE];
    char command[UART_CONTROL_CMD_SIZE];
    uint8_t inv_magic[UART_CONTROL_MAGIC_SIZE];
} uart_control_packet_t;

#define UART_CONTROL_PACKET_SIZE sizeof(uart_control_packet_t)

/* Log message packet magic: 16-byte pattern for identifying log packets */
#define UART_LOGMSG_MAGIC_SIZE 8
extern const uint8_t uart_logmsg_magic[UART_LOGMSG_MAGIC_SIZE];

/* Maximum log message length in bytes */
#define UART_LOGMSG_MAX_LEN 256

/* Log message packet header (payload follows) */
typedef struct __attribute__((packed))
{
    uint8_t magic[UART_LOGMSG_MAGIC_SIZE];
    uint32_t length;
    uint8_t inv_magic[UART_LOGMSG_MAGIC_SIZE];
} uart_logmsg_header_t;

#define UART_LOGMSG_HEADER_SIZE sizeof(uart_logmsg_header_t)

typedef struct
{
    uint32_t baud_rate;
    uint8_t tx_gpio;
    uint8_t rx_gpio;
    uint8_t reset_gpio;
    uint8_t control_gpio;
    uint8_t led_gpio;
} uartgw_config_t;

/* Callback function type for configuration change notifications */
typedef void (*uart_config_callback_t)(const uartgw_config_t *config);

/* Initialize UART gateway with default configuration */
void uart_gateway_init(const uartgw_config_t *config);

/* Register callback for configuration changes */
void uart_gateway_register_callback(uart_config_callback_t callback);

/* Configure UART with new parameters */
esp_err_t uart_gateway_configure(const uartgw_config_t *config);

/* Get current UART configuration */
void uart_gateway_get_config(uartgw_config_t *config);

/* Process received data buffer, checking for configuration packets */
void uart_gateway_process_data(const uint8_t *data, size_t length);

/* Send data to UART TX pin */
esp_err_t uart_gateway_send(const uint8_t *data, size_t length);

/* Receive data from UART RX pin */
int uart_gateway_receive(uint8_t *buffer, size_t max_length);

/* Check if UART is configured and ready */
bool uart_gateway_is_ready(void);

/* Deinitialize UART */
void uart_gateway_deinit(void);

/* Save configuration to NVS */
esp_err_t uart_gateway_save_config(void);

/* Load configuration from NVS */
esp_err_t uart_gateway_load_config(uartgw_config_t *loaded_config);

/* Build a configuration response packet */
void uart_gateway_build_response_packet(uint8_t *packet, size_t *packet_len);

/* Queue data from CDC to UART */
esp_err_t uart_gateway_queue_cdc_data(const uint8_t *data, size_t length);

/* Receive data from CDC queue (intended for UART) */
int uart_gateway_receive_cdc_queue(uint8_t *buffer, size_t max_length);

/* Queue data from UART to CDC */
esp_err_t uart_gateway_queue_uart_data(const uint8_t *data, size_t length);

/* Receive data from UART queue (intended for CDC) */
int uart_gateway_receive_uart_queue(uint8_t *buffer, size_t max_length);

/* Send formatted log message to CDC as LOGMSG packet */
void send_message(const char *fmt, ...);

/* Start gateway tasks and USB CDC; call after uart_gateway_init */
void uart_gateway_start(void);

/* Stop gateway tasks and USB CDC */
void uart_gateway_stop(void);
