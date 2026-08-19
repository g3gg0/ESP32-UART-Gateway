#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum
{
    CANONEF_INVERT_NONE = 0,
    CANONEF_INVERT_DCL = (1U << 0),
    CANONEF_INVERT_DLC = (1U << 1),
    CANONEF_INVERT_LCLK_OUT = (1U << 2),
    CANONEF_INVERT_LCLK_IN = (1U << 3),
} canonef_inversion_t;

typedef enum
{
    CANONEF_CONFIG_FLAG_ACK_DURATION = (1U << 0),
    CANONEF_CONFIG_FLAG_IGNORE_ACK = (1U << 1),
} canonef_config_flags_t;

typedef enum
{
    CANONEF_STATUS_OK = 0x00,
    CANONEF_STATUS_BAD_LEN = 0x01,
    CANONEF_STATUS_INVALID_ARG = 0x02,
    CANONEF_STATUS_NOT_INITIALIZED = 0x03,
    CANONEF_STATUS_ACK_TIMEOUT = 0x04,
    CANONEF_STATUS_TRANSFER_FAILED = 0x05,
    CANONEF_STATUS_NO_MEMORY = 0x06,
    CANONEF_STATUS_ACK_TOO_LONG = 0x07,
    CANONEF_STATUS_INTERNAL = 0x7F,
} canonef_status_t;

typedef struct __attribute__((packed))
{
    uint8_t dcl_gpio;
    uint8_t dlc_gpio;
    uint8_t lclk_out_gpio;
    uint8_t lclk_in_gpio;
    uint8_t inversion_mask;
    uint8_t flags;
    uint16_t reserved;
    uint32_t clock_hz;
} canonef_config_packet_t;

typedef struct __attribute__((packed))
{
    uint8_t status;
    canonef_config_packet_t config;
} canonef_config_response_t;

void canonef_stop_session(void);
esp_err_t canonef_handle_config_packet(const uint8_t *payload, size_t payload_len);
esp_err_t canonef_handle_xfer_packet(const uint8_t *payload, size_t payload_len);
esp_err_t canonef_handle_reset_packet(const uint8_t *payload, size_t payload_len);