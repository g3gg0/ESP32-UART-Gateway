#include "led.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "uart_gateway.h"

#define TAG "LED"

/* LED blink rates in Hz */
#define LED_IDLE_FREQ 1
#define LED_ACTIVE_FREQ 15

/* LED PWM configuration */
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL LEDC_CHANNEL_0
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_FREQUENCY 5000

/* Idle fade range (percent) */
#define LED_IDLE_MIN_PCT 10
#define LED_IDLE_MAX_PCT 90
#define LED_IDLE_FADE_STEP_PCT 4

/* Duration to keep active blink rate after UART activity (in milliseconds) */
#define ACTIVITY_TIMEOUT_MS 100

static volatile TickType_t last_activity_tick = 0;
static TaskHandle_t led_task_handle = NULL;
static volatile bool led_task_should_stop = false;

static uartgw_config_t config;

static void led_task(void *param)
{
    (void)param;
    int level = 0;
    int idle_dir = 1;
    int idle_pct = LED_IDLE_MIN_PCT;
    const int duty_max = (1 << LEDC_DUTY_RES) - 1;

    if (config.led_gpio != 0xFF)
    {
        int duty = (idle_pct * duty_max) / 100;
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    }

    while (true)
    {
        if (led_task_should_stop)
        {
            break;
        }

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

        if (config.led_gpio == 0xFF)
        {
            xTaskNotifyWait(0, 0xFFFFFFFF, NULL, pdMS_TO_TICKS(50));
            continue;
        }

        if (freq == LED_ACTIVE_FREQ)
        {
            /* Active blink using PWM duty 0/100% */
            int duty = level ? duty_max : 0;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

            /* Period in milliseconds */
            uint32_t period_ms = 1000 / LED_ACTIVE_FREQ;
            uint32_t on_time_ms = period_ms / 2;

            /* Sleep with interruption by task notification */
            xTaskNotifyWait(0, 0xFFFFFFFF, NULL, pdMS_TO_TICKS(on_time_ms));

            if (led_task_should_stop)
            {
                break;
            }

            level ^= 1;
        }
        else
        {
            /* Idle fade between 20% and 80% */
            idle_pct += (idle_dir * LED_IDLE_FADE_STEP_PCT);
            if (idle_pct >= LED_IDLE_MAX_PCT)
            {
                idle_pct = LED_IDLE_MAX_PCT;
                idle_dir = -1;
            }
            else if (idle_pct <= LED_IDLE_MIN_PCT)
            {
                idle_pct = LED_IDLE_MIN_PCT;
                idle_dir = 1;
            }

            int duty = (idle_pct * duty_max) / 100;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

            /* Step delay for ~1 Hz breathing cycle */
            xTaskNotifyWait(0, 0xFFFFFFFF, NULL, pdMS_TO_TICKS(30));

            if (led_task_should_stop)
            {
                break;
            }
        }
    }

    if (config.led_gpio != 0xFF)
    {
        ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);
    }

    led_task_handle = NULL;
    vTaskDelete(NULL);
}

esp_err_t led_init(const uartgw_config_t *cfg)
{
    if (cfg == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    led_task_should_stop = false;

    /* Initialize state */
    last_activity_tick = 0;

    if (cfg->led_gpio != 0xFF)
    {
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_MODE,
            .timer_num = LEDC_TIMER,
            .duty_resolution = LEDC_DUTY_RES,
            .freq_hz = LEDC_FREQUENCY,
            .clk_cfg = LEDC_AUTO_CLK,
        };

        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_MODE,
            .channel = LEDC_CHANNEL,
            .timer_sel = LEDC_TIMER,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = cfg->led_gpio,
            .duty = 0,
            .hpoint = 0,
        };

        ledc_timer_config(&ledc_timer);
        ledc_channel_config(&ledc_channel);
    }

    config = *cfg;

    /* Create LED task */
    if (xTaskCreate(led_task, "led_task", 2048, NULL, 5, &led_task_handle) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create LED task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "LED initialized");
    return ESP_OK;
}

void led_stop(void)
{
    if (led_task_handle == NULL)
    {
        return;
    }

    led_task_should_stop = true;
    xTaskNotify(led_task_handle, 0, eNoAction);

    for (int i = 0; i < 20; i++)
    {
        if (led_task_handle == NULL)
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (config.led_gpio != 0xFF)
    {
        ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);
        gpio_reset_pin(config.led_gpio);
    }
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
