#include "led_status.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "pins.h"

static const char *TAG = "led";

/* XIAO ESP32-C6 onboard user LED is active-low (drive LOW to light).
 * See pins.h for why the onboard LED is used instead of the board's D1. */
#define IDL0_LED_ACTIVE_LEVEL  0

static TimerHandle_t s_blink_timer = NULL;
static bool s_blink_phase = false;

static void apply_level(bool on)
{
    gpio_set_level(IDL0_PIN_LED, on ? IDL0_LED_ACTIVE_LEVEL : !IDL0_LED_ACTIVE_LEVEL);
}

static void blink_cb(TimerHandle_t t)
{
    (void)t;
    s_blink_phase = !s_blink_phase;
    apply_level(s_blink_phase);
}

bool idl0_led_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << IDL0_PIN_LED,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (gpio_config(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed for LED pin");
        return false;
    }
    apply_level(false);

    s_blink_timer = xTimerCreate("idl0_led", pdMS_TO_TICKS(500), pdTRUE, NULL, blink_cb);
    if (s_blink_timer == NULL) {
        ESP_LOGE(TAG, "xTimerCreate failed");
        return false;
    }
    return true;
}

void idl0_led_set(idl0_led_pattern_t pattern)
{
    xTimerStop(s_blink_timer, 0);
    s_blink_phase = false;

    switch (pattern) {
        case IDL0_LED_OFF:
            apply_level(false);
            break;
        case IDL0_LED_ON:
            apply_level(true);
            break;
        case IDL0_LED_BLINK_SLOW:
            xTimerChangePeriod(s_blink_timer, pdMS_TO_TICKS(500), 0);
            xTimerStart(s_blink_timer, 0);
            break;
        case IDL0_LED_BLINK_FAST:
            xTimerChangePeriod(s_blink_timer, pdMS_TO_TICKS(125), 0);
            xTimerStart(s_blink_timer, 0);
            break;
    }
}
