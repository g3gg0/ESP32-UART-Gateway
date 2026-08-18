#include <string.h>

#include "config_manager.h"
#include "uart_gateway.h"
#include "sdkconfig.h"
#include "nvs.h"
#include "esp_err.h"

#ifndef CONFIG_GW_UART_DEFAULT_BAUD
#define CONFIG_GW_UART_DEFAULT_BAUD 115200
#endif
#ifndef CONFIG_GW_UART_DEFAULT_TX_GPIO
#define CONFIG_GW_UART_DEFAULT_TX_GPIO 20
#endif
#ifndef CONFIG_GW_UART_DEFAULT_RX_GPIO
#define CONFIG_GW_UART_DEFAULT_RX_GPIO 21
#endif
#ifndef CONFIG_GW_UART_DEFAULT_RESET_GPIO
#define CONFIG_GW_UART_DEFAULT_RESET_GPIO 255
#endif
#ifndef CONFIG_GW_UART_DEFAULT_CONTROL_GPIO
#define CONFIG_GW_UART_DEFAULT_CONTROL_GPIO 255
#endif
#ifndef CONFIG_GW_UART_DEFAULT_LED_GPIO
#define CONFIG_GW_UART_DEFAULT_LED_GPIO 8
#endif
#ifndef CONFIG_GW_CAN_DEFAULT_BAUD
#define CONFIG_GW_CAN_DEFAULT_BAUD 500000
#endif
#ifndef CONFIG_GW_SWD_DEFAULT_CLOCK_DELAY
#define CONFIG_GW_SWD_DEFAULT_CLOCK_DELAY 0
#endif
#ifndef CONFIG_GW_SWD_DEFAULT_IDLE_BITS
#define CONFIG_GW_SWD_DEFAULT_IDLE_BITS 8
#endif
#ifndef CONFIG_GW_WIFI_AP_SSID
#define CONFIG_GW_WIFI_AP_SSID "ESP32C3-GW"
#endif
#ifndef CONFIG_GW_WIFI_AP_PASSWORD
#define CONFIG_GW_WIFI_AP_PASSWORD ""
#endif
#ifndef CONFIG_GW_WIFI_AP_CHANNEL
#define CONFIG_GW_WIFI_AP_CHANNEL 1
#endif
#ifndef CONFIG_GW_WIFI_AP_MAX_CONN
#define CONFIG_GW_WIFI_AP_MAX_CONN 4
#endif
#ifndef CONFIG_GW_WIFI_STA_TIMEOUT_S
#define CONFIG_GW_WIFI_STA_TIMEOUT_S 12
#endif
#ifndef CONFIG_GW_DEVICE_NAME
#define CONFIG_GW_DEVICE_NAME "esp32c3-gw"
#endif

#define FIELD_OFFSET(_field) offsetof(device_config_t, _field)

static device_config_t g_cfg = {
    .uart_baud_rate = CONFIG_GW_UART_DEFAULT_BAUD,
    .uart_tx_gpio = (uint8_t)CONFIG_GW_UART_DEFAULT_TX_GPIO,
    .uart_rx_gpio = (uint8_t)CONFIG_GW_UART_DEFAULT_RX_GPIO,
    .uart_reset_gpio = (uint8_t)CONFIG_GW_UART_DEFAULT_RESET_GPIO,
    .uart_control_gpio = (uint8_t)CONFIG_GW_UART_DEFAULT_CONTROL_GPIO,
    .uart_led_gpio = (uint8_t)CONFIG_GW_UART_DEFAULT_LED_GPIO,

    .can_default_baud = CONFIG_GW_CAN_DEFAULT_BAUD,

    .swd_default_io_mask = 0x003007FF,
    .swd_clock_delay_us = CONFIG_GW_SWD_DEFAULT_CLOCK_DELAY,
    .swd_idle_bits = CONFIG_GW_SWD_DEFAULT_IDLE_BITS,

    .device_name = CONFIG_GW_DEVICE_NAME,

    .wifi_sta_ssid = "",
    .wifi_sta_password = "",
    .wifi_ap_ssid = CONFIG_GW_WIFI_AP_SSID,
    .wifi_ap_password = CONFIG_GW_WIFI_AP_PASSWORD,
    .wifi_ap_channel = (uint8_t)CONFIG_GW_WIFI_AP_CHANNEL,
    .wifi_ap_max_connections = (uint8_t)CONFIG_GW_WIFI_AP_MAX_CONN,
    .wifi_sta_timeout_s = (uint8_t)CONFIG_GW_WIFI_STA_TIMEOUT_S,
};

static const config_field_desc_t g_fields[] = {
    { "UART", "Baud Rate", "uart_config", "baud_rate", CONFIG_FIELD_U32, FIELD_OFFSET(uart_baud_rate), 0, 300, 1000000 },
    { "UART", "TX GPIO", "uart_config", "tx_gpio", CONFIG_FIELD_U8, FIELD_OFFSET(uart_tx_gpio), 0, 0, 43 },
    { "UART", "RX GPIO", "uart_config", "rx_gpio", CONFIG_FIELD_U8, FIELD_OFFSET(uart_rx_gpio), 0, 0, 43 },
    { "UART", "RESET GPIO", "uart_config", "reset_gpio", CONFIG_FIELD_U8, FIELD_OFFSET(uart_reset_gpio), 0, 0, 255 },
    { "UART", "CONTROL GPIO", "uart_config", "control_gpio", CONFIG_FIELD_U8, FIELD_OFFSET(uart_control_gpio), 0, 0, 255 },
    { "UART", "LED GPIO", "uart_config", "led_gpio", CONFIG_FIELD_U8, FIELD_OFFSET(uart_led_gpio), 0, 0, 255 },

    { "CAN", "Default Baud", "can_config", "baud", CONFIG_FIELD_U32, FIELD_OFFSET(can_default_baud), 0, 25000, 1000000 },

    { "SWD", "Default IO Mask", "swd_config", "io_mask", CONFIG_FIELD_U32, FIELD_OFFSET(swd_default_io_mask), 0, 0, 0xFFFFFFFF },
    { "SWD", "Clock Delay (us)", "swd_config", "clock_delay", CONFIG_FIELD_U32, FIELD_OFFSET(swd_clock_delay_us), 0, 0, 1000 },
    { "SWD", "Idle Bits", "swd_config", "idle_bits", CONFIG_FIELD_U32, FIELD_OFFSET(swd_idle_bits), 0, 1, 64 },

    { "Network", "Device Name (DHCP/mDNS)", "wifi_config", "device_name", CONFIG_FIELD_STR, FIELD_OFFSET(device_name), DEVICE_NAME_MAX_LEN, 0, 0 },

    { "WiFi STA", "SSID", "wifi_config", "sta_ssid", CONFIG_FIELD_STR, FIELD_OFFSET(wifi_sta_ssid), WIFI_SSID_MAX_LEN, 0, 0 },
    { "WiFi STA", "Password", "wifi_config", "sta_password", CONFIG_FIELD_STR, FIELD_OFFSET(wifi_sta_password), WIFI_PASSWORD_MAX_LEN, 0, 0 },
    { "WiFi AP", "SSID", "wifi_config", "ap_ssid", CONFIG_FIELD_STR, FIELD_OFFSET(wifi_ap_ssid), WIFI_SSID_MAX_LEN, 0, 0 },
    { "WiFi AP", "Password", "wifi_config", "ap_password", CONFIG_FIELD_STR, FIELD_OFFSET(wifi_ap_password), WIFI_PASSWORD_MAX_LEN, 0, 0 },
    { "WiFi AP", "Channel", "wifi_config", "ap_channel", CONFIG_FIELD_U8, FIELD_OFFSET(wifi_ap_channel), 0, 1, 13 },
    { "WiFi AP", "Max Connections", "wifi_config", "ap_max_conn", CONFIG_FIELD_U8, FIELD_OFFSET(wifi_ap_max_connections), 0, 1, 8 },
    { "WiFi STA", "Connect Timeout (s)", "wifi_config", "sta_timeout_s", CONFIG_FIELD_U8, FIELD_OFFSET(wifi_sta_timeout_s), 0, 3, 60 },
};

static void *field_ptr(const config_field_desc_t *field)
{
    return (void *)(((uint8_t *)&g_cfg) + field->offset);
}

static esp_err_t load_field(nvs_handle_t nvs_handle, const config_field_desc_t *field)
{
    esp_err_t err = ESP_OK;
    void *ptr = field_ptr(field);

    if (field->type == CONFIG_FIELD_U8)
    {
        uint8_t value = *(uint8_t *)ptr;
        err = nvs_get_u8(nvs_handle, field->nvs_key, &value);
        if (err == ESP_OK)
        {
            *(uint8_t *)ptr = value;
        }
    }
    else if (field->type == CONFIG_FIELD_U32)
    {
        uint32_t value = *(uint32_t *)ptr;
        err = nvs_get_u32(nvs_handle, field->nvs_key, &value);
        if (err == ESP_OK)
        {
            *(uint32_t *)ptr = value;
        }
    }
    else
    {
        size_t required_len = field->str_max_len;
        err = nvs_get_str(nvs_handle, field->nvs_key, (char *)ptr, &required_len);
    }

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        return ESP_OK;
    }

    return err;
}

static esp_err_t save_field(nvs_handle_t nvs_handle, const config_field_desc_t *field)
{
    void *ptr = field_ptr(field);

    if (field->type == CONFIG_FIELD_U8)
    {
        uint32_t value = *(uint8_t *)ptr;
        if (value < field->min_value || value > field->max_value)
        {
            return ESP_ERR_INVALID_ARG;
        }
        return nvs_set_u8(nvs_handle, field->nvs_key, *(uint8_t *)ptr);
    }

    if (field->type == CONFIG_FIELD_U32)
    {
        uint32_t value = *(uint32_t *)ptr;
        if (value < field->min_value || value > field->max_value)
        {
            return ESP_ERR_INVALID_ARG;
        }
        return nvs_set_u32(nvs_handle, field->nvs_key, value);
    }

    ((char *)ptr)[field->str_max_len - 1] = '\0';
    return nvs_set_str(nvs_handle, field->nvs_key, (const char *)ptr);
}

esp_err_t config_manager_load_all(void)
{
    const char *current_ns = NULL;
    nvs_handle_t nvs_handle = 0;
    esp_err_t status = ESP_OK;

    for (size_t i = 0; i < (sizeof(g_fields) / sizeof(g_fields[0])); i++)
    {
        const config_field_desc_t *field = &g_fields[i];

        if (current_ns == NULL || strcmp(current_ns, field->nvs_namespace) != 0)
        {
            if (current_ns != NULL)
            {
                nvs_close(nvs_handle);
            }

            esp_err_t open_err = nvs_open(field->nvs_namespace, NVS_READONLY, &nvs_handle);
            if (open_err != ESP_OK)
            {
                current_ns = NULL;
                if (open_err != ESP_ERR_NVS_NOT_FOUND)
                {
                    send_message("CFG: nvs_open(%s) failed: %s", field->nvs_namespace, esp_err_to_name(open_err));
                }
                continue;
            }

            current_ns = field->nvs_namespace;
        }

        esp_err_t err = load_field(nvs_handle, field);
        if (err != ESP_OK)
        {
            send_message("CFG: load %s/%s failed: %s", field->nvs_namespace, field->nvs_key, esp_err_to_name(err));
            status = err;
        }
    }

    if (current_ns != NULL)
    {
        nvs_close(nvs_handle);
    }

    return status;
}

esp_err_t config_manager_save_all(void)
{
    const char *current_ns = NULL;
    nvs_handle_t nvs_handle = 0;
    esp_err_t status = ESP_OK;

    for (size_t i = 0; i < (sizeof(g_fields) / sizeof(g_fields[0])); i++)
    {
        const config_field_desc_t *field = &g_fields[i];

        if (current_ns == NULL || strcmp(current_ns, field->nvs_namespace) != 0)
        {
            if (current_ns != NULL)
            {
                esp_err_t c_err = nvs_commit(nvs_handle);
                if (c_err != ESP_OK)
                {
                    status = c_err;
                }
                nvs_close(nvs_handle);
            }

            esp_err_t open_err = nvs_open(field->nvs_namespace, NVS_READWRITE, &nvs_handle);
            if (open_err != ESP_OK)
            {
                status = open_err;
                current_ns = NULL;
                continue;
            }

            current_ns = field->nvs_namespace;
        }

        esp_err_t err = save_field(nvs_handle, field);
        if (err != ESP_OK)
        {
            send_message("CFG: save %s/%s failed: %s", field->nvs_namespace, field->nvs_key, esp_err_to_name(err));
            status = err;
        }
    }

    if (current_ns != NULL)
    {
        esp_err_t c_err = nvs_commit(nvs_handle);
        if (c_err != ESP_OK)
        {
            status = c_err;
        }
        nvs_close(nvs_handle);
    }

    return status;
}

esp_err_t config_manager_init(void)
{
    return config_manager_load_all();
}

const config_field_desc_t *config_manager_get_fields(size_t *count)
{
    if (count)
    {
        *count = sizeof(g_fields) / sizeof(g_fields[0]);
    }
    return g_fields;
}

const device_config_t *config_manager_get(void)
{
    return &g_cfg;
}

device_config_t *config_manager_get_mutable(void)
{
    return &g_cfg;
}
