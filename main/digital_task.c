#include "digital_task.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "idl0_format.h"
#include "writer_task.h"

#define TAG "digital_task"

typedef struct {
    int     channel_idx;       /* index into s_channels */
    int64_t timestamp_us;
} press_event_t;

static QueueHandle_t s_press_queue;
static TaskHandle_t  s_task;
static digital_channel_cfg_t s_channels[DIGITAL_MAX_CHANNELS];
static size_t  s_channel_count;
static uint8_t s_press_counters[DIGITAL_MAX_CHANNELS];
static int64_t s_last_press_us[DIGITAL_MAX_CHANNELS];

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    const int idx = (int)(intptr_t)arg;
    press_event_t ev = {
        .channel_idx = idx,
        .timestamp_us = esp_timer_get_time(),
    };
    BaseType_t hp_task_woken = pdFALSE;
    xQueueSendFromISR(s_press_queue, &ev, &hp_task_woken);
    if (hp_task_woken) portYIELD_FROM_ISR();
}

/* Submit one CHANNEL_SAMPLE record (u8 press counter) via the writer.
 * Builds the framed record on the stack. */
static void submit_press_record(uint8_t channel_id, int64_t ts_us, uint8_t value) {
    /* 3-byte framing + 1 channel_id + 8 timestamp_us + 1 value = 13 bytes max. */
    uint8_t buf[16];
    const size_t n = idl0_format_channel_sample(buf, channel_id, ts_us,
                                                &value, sizeof(value));
    if (n == 0) return;
    (void)idl0_writer_submit(buf, n);
}

static void digital_task_fn(void *arg) {
    (void)arg;
    press_event_t ev;
    while (true) {
        if (xQueueReceive(s_press_queue, &ev, portMAX_DELAY) != pdTRUE) continue;
        if (ev.channel_idx < 0 || (size_t)ev.channel_idx >= s_channel_count) continue;
        const digital_channel_cfg_t *ch = &s_channels[ev.channel_idx];

        /* Software debounce — reject presses within debounce_ms of the last one. */
        const int64_t since_us = ev.timestamp_us - s_last_press_us[ev.channel_idx];
        if (since_us < (int64_t)ch->debounce_ms * 1000) continue;
        s_last_press_us[ev.channel_idx] = ev.timestamp_us;

        /* Monotonic press counter (u8 — wraps; press order still derivable). */
        s_press_counters[ev.channel_idx]++;
        submit_press_record(ch->channel_id, ev.timestamp_us,
                            s_press_counters[ev.channel_idx]);
        ESP_LOGI(TAG, "marker '%s' press #%u (channel %u, t=%lld us)",
                 ch->label, s_press_counters[ev.channel_idx],
                 ch->channel_id, ev.timestamp_us);
    }
}

void digital_task_start(const digital_channel_cfg_t *channels, size_t count) {
    digital_task_stop();
    if (channels == NULL || count == 0) return;

    if (count > DIGITAL_MAX_CHANNELS) count = DIGITAL_MAX_CHANNELS;
    s_channel_count = count;
    memcpy(s_channels, channels, count * sizeof(*channels));
    memset(s_press_counters, 0, sizeof(s_press_counters));
    memset(s_last_press_us, 0, sizeof(s_last_press_us));

    s_press_queue = xQueueCreate(16, sizeof(press_event_t));
    if (s_press_queue == NULL) {
        ESP_LOGE(TAG, "queue create failed");
        return;
    }

    /* Best-effort install — already-installed is fine. */
    gpio_install_isr_service(0);

    for (size_t i = 0; i < count; i++) {
        const digital_channel_cfg_t *ch = &s_channels[i];
        if (!ch->enabled) continue;
        const gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << ch->gpio_pin,
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = ch->active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pull_down_en = ch->active_low ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
            .intr_type    = ch->active_low ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE,
        };
        gpio_config(&cfg);
        gpio_isr_handler_add(ch->gpio_pin, gpio_isr_handler, (void *)(intptr_t)i);
        ESP_LOGI(TAG, "marker '%s' on GPIO %d (active_%s, debounce %u ms)",
                 ch->label, ch->gpio_pin,
                 ch->active_low ? "low" : "high", ch->debounce_ms);
    }

    xTaskCreate(digital_task_fn, "digital_task", 2048, NULL, 5, &s_task);
}

void digital_task_stop(void) {
    for (size_t i = 0; i < s_channel_count; i++) {
        if (s_channels[i].enabled) {
            gpio_isr_handler_remove(s_channels[i].gpio_pin);
        }
    }
    if (s_task != NULL) {
        vTaskDelete(s_task);
        s_task = NULL;
    }
    if (s_press_queue != NULL) {
        vQueueDelete(s_press_queue);
        s_press_queue = NULL;
    }
    s_channel_count = 0;
}
