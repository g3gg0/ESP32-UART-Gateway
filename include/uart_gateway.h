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

/* Extended mode activation magic: a valid packet with type 0x000A and 8-byte payload */
#define UART_EXTMODE_MAGIC_PAYLOAD "UARTGWEX"
#define UART_EXTMODE_MAGIC_SIZE 12 /* 4 bytes header + 8 bytes payload */

/* Configuration packet structure - packed for binary compatibility */
/* No magic fields - uses packet header in extended mode */
typedef struct __attribute__((packed))
{
    uint32_t baud_rate;
    uint8_t tx_gpio;
    uint8_t rx_gpio;
    uint8_t reset_gpio;
    uint8_t control_gpio;
    uint8_t led_gpio;
    uint8_t padding[2];
    uint8_t extended_mode;
} uart_config_packet_t;

#define UART_CONFIG_PACKET_SIZE sizeof(uart_config_packet_t)

/* Extended mode packet types */
typedef enum
{
    UART_PACKET_TYPE_DATA = 0x00,    /* Normal serial data */
    UART_PACKET_TYPE_CONFIG = 0x01,  /* USB config struct */
    UART_PACKET_TYPE_CONTROL = 0x02, /* Control commands (B:1234, R:0) */
    UART_PACKET_TYPE_LOG = 0x03,     /* Log messages from ESP32 */
    UART_PACKET_TYPE_SWD = 0x04,     /* SWD commands */
    UART_PACKET_TYPE_EXTMODE = 0x0A  /* Extended mode activation packet */
} uart_packet_type_t;

/* Extended mode packet header */
typedef struct __attribute__((packed))
{
    uint16_t length; /* Total length including length and type field (minimum 4) */
    uint16_t type;   /* Packet type (see uart_packet_type_t) */
} uart_packet_header_t;

#define PTR_BEHIND(ptr) ((void *)(((intptr_t)(ptr)) + sizeof(uart_packet_header_t)))

#define UART_PACKET_HEADER_SIZE sizeof(uart_packet_header_t)

/* Maximum control command string length */
#define UART_CONTROL_CMD_SIZE 16

/* Maximum log message length in bytes */
#define UART_LOGMSG_MAX_LEN 256

typedef struct
{
    uint32_t baud_rate;
    uint8_t tx_gpio;
    uint8_t rx_gpio;
    uint8_t reset_gpio;
    uint8_t control_gpio;
    uint8_t led_gpio;
    uint8_t extended_mode;
} uartgw_config_t;

typedef struct
{
    uart_port_t uart_num;
    uartgw_config_t current_config;
    bool is_configured;
    bool is_initializing;
    uint8_t config_buffer[UART_CONFIG_PACKET_SIZE];
    size_t config_buffer_pos;
    StreamBufferHandle_t cdc_to_uart_buffer;
    QueueHandle_t uart_to_cdc_buffer;
} uart_gateway_ctx_t;


/* ===== SWD UART sub-protocol (inside UART_PACKET_TYPE_SWD payload) =====
 *
 * Payload begins with swd_uart_req_hdr_t (magic + op + flags + seq).
 * Firmware always enqueues a binary response packet with op|0x80.
 */

#define SWD_UART_MAGIC 0xCAFEu

typedef enum
{
    SWD_UART_OP_DETECT_PINS = 0x01,
    SWD_UART_OP_DEINIT = 0x02,
    SWD_UART_OP_TRANSFER = 0x03,
    SWD_UART_OP_AP_READ = 0x10,
    SWD_UART_OP_AP_READ_SINGLE = 0x11,
    SWD_UART_OP_AP_WRITE = 0x12,
} swd_uart_op_t;

typedef enum
{
    SWD_UART_STATUS_OK = 0x00,
    SWD_UART_STATUS_BAD_LEN = 0x01,
    SWD_UART_STATUS_BAD_OP = 0x02,
    SWD_UART_STATUS_NOT_INITIALIZED = 0x03,
    SWD_UART_STATUS_PARITY = 0x04,
    SWD_UART_STATUS_INTERNAL = 0x7F,
} swd_uart_status_t;

typedef enum
{
    SWD_UART_FLAG_VERBOSE_LOG = 0x01,
} swd_uart_flags_t;

typedef struct __attribute__((packed))
{
    uint16_t magic; /* SWD_UART_MAGIC */
    uint8_t op;
    uint8_t flags;
    uint16_t seq;
    uint16_t reserved;
} swd_uart_req_hdr_t;

typedef struct __attribute__((packed))
{
    uint16_t magic; /* SWD_UART_MAGIC */
    uint8_t op;     /* request op | 0x80 */
    uint8_t status; /* swd_uart_status_t */
    uint16_t seq;
    uint8_t swdio_gpio; /* final active SWDIO GPIO or 0xFF */
    uint8_t swclk_gpio; /* final active SWCLK GPIO or 0xFF */
    uint8_t ack;        /* SWD ACK (1/2/4) or 8 for parity mismatch, 0 if N/A */
    uint8_t reserved;
    uint16_t data_len;
} swd_uart_rsp_hdr_t;

typedef struct __attribute__((packed))
{
    uint32_t dpidr;
    uint32_t targetid;
    uint8_t dpidr_ok;
    uint8_t targetid_ok;
    uint8_t detected_device;
    uint8_t reserved;
} swd_uart_detect_rsp_t;

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
void uart_gateway_build_response_packet(uart_config_packet_t *packet);

/* Queue data from CDC to UART */
esp_err_t uart_gateway_queue_cdc_data(const uint8_t *data, size_t length);

/* Receive data from CDC queue (intended for UART) */
int uart_gateway_receive_cdc_queue(uint8_t *buffer, size_t max_length);

/* Queue data from UART to CDC */
esp_err_t queue_packet(uart_packet_header_t *packet);

/* Receive data from UART queue (intended for CDC) */
uart_packet_header_t *unqueue_packet(void);

/* Send formatted log message to CDC as LOGMSG packet */
void send_message(const char *fmt, ...);

/* Start gateway tasks and USB CDC; call after uart_gateway_init */
void uart_gateway_start(void);

/* Stop gateway tasks and USB CDC */
void uart_gateway_stop(void);
