#include "led.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_gateway.h"

#define TAG "LED"

/* LED blink rates in Hz */
#define LED_IDLE_FREQ 1
#define LED_ACTIVE_FREQ 15

/* Duration to keep active blink rate after UART activity (in milliseconds) */
#define ACTIVITY_TIMEOUT_MS 100

static volatile TickType_t last_activity_tick = 0;
static TaskHandle_t led_task_handle = NULL;

extern uartgw_config_t config;

static void led_task(void *param)
{
    int level = 0;

    while (true)
    {
        TickType_t current_tick = xTaskGetTickCount();
        TickType_t last_tick = last_activity_tick;

        /* Determine current blink frequency */
        uint32_t freq;
        if ((current_tick - last_tick) < pdMS_TO_TICKS(ACTIVITY_TIMEOUT_MS))
        {
            freq = LED_ACTIVE_FREQ;
        }
        else
        {
            freq = LED_IDLE_FREQ;
        }

        /* Period in milliseconds */
        uint32_t period_ms = 1000 / freq;
        uint32_t on_time_ms = period_ms / 2;

        if (config.led_gpio != 0xFF)
        {
            /* LED on */
            gpio_set_level(config.led_gpio, level);
        }
        /* Sleep with interruption by task notification */
        xTaskNotifyWait(0, 0xFFFFFFFF, NULL, pdMS_TO_TICKS(on_time_ms));

        level ^= 1; /* Toggle LED level */
    }
}

esp_err_t led_init()
{
    /* Initialize state */
    last_activity_tick = 0;

    /* Create LED task */
    if (xTaskCreate(led_task, "led_task", 2048, NULL, 5, &led_task_handle) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create LED task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "LED initialized");
    return ESP_OK;
}

void led_signal_uart_activity(void)
{
    last_activity_tick = xTaskGetTickCount();
    /* Interrupt the LED task's sleep to apply new frequency immediately */
    if (led_task_handle != NULL)
    {
        xTaskNotify(led_task_handle, 0, eNoAction);
    }
}
