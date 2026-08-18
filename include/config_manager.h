#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define WIFI_SSID_MAX_LEN 64
#define WIFI_PASSWORD_MAX_LEN 64
#define DEVICE_NAME_MAX_LEN 32

typedef struct
{
    uint32_t uart_baud_rate;
    uint8_t uart_tx_gpio;
    uint8_t uart_rx_gpio;
    uint8_t uart_reset_gpio;
    uint8_t uart_control_gpio;
    uint8_t uart_led_gpio;

    uint32_t can_default_baud;

    uint32_t swd_default_io_mask;
    uint32_t swd_clock_delay_us;
    uint32_t swd_idle_bits;

    char device_name[DEVICE_NAME_MAX_LEN];

    char wifi_sta_ssid[WIFI_SSID_MAX_LEN];
    char wifi_sta_password[WIFI_PASSWORD_MAX_LEN];
    char wifi_ap_ssid[WIFI_SSID_MAX_LEN];
    char wifi_ap_password[WIFI_PASSWORD_MAX_LEN];
    uint8_t wifi_ap_channel;
    uint8_t wifi_ap_max_connections;
    uint8_t wifi_sta_timeout_s;
} device_config_t;

typedef enum
{
    CONFIG_FIELD_U8 = 0,
    CONFIG_FIELD_U32 = 1,
    CONFIG_FIELD_STR = 2,
} config_field_type_t;

typedef struct
{
    const char *group;
    const char *label;
    const char *nvs_namespace;
    const char *nvs_key;
    config_field_type_t type;
    size_t offset;
    size_t str_max_len;
    uint32_t min_value;
    uint32_t max_value;
} config_field_desc_t;

const device_config_t *config_manager_get(void);
device_config_t *config_manager_get_mutable(void);

const config_field_desc_t *config_manager_get_fields(size_t *count);

esp_err_t config_manager_init(void);
esp_err_t config_manager_save_all(void);
esp_err_t config_manager_load_all(void);
