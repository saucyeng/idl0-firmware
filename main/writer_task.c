/* Single-consumer SD writer — see writer_task.h. */

#include "writer_task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"

#include "sd_logger.h"

static const char *TAG = "writer";

#define WRITER_FLUSH_PERIOD_US (1000 * 1000)   /* durability flush cadence (~1 s) */

static RingbufHandle_t s_rb   = NULL;
static TaskHandle_t    s_task = NULL;

static void writer_task_fn(void *arg)
{
    (void)arg;
    int64_t last_flush_us = esp_timer_get_time();
    for (;;) {
        size_t item_size = 0;
        void *item = xRingbufferReceive(s_rb, &item_size, pdMS_TO_TICKS(100));
        if (item != NULL) {
            if (!idl0_sd_append((const uint8_t *)item, item_size)) {
                ESP_LOGW(TAG, "append failed (%u bytes)", (unsigned)item_size);
            }
            vRingbufferReturnItem(s_rb, item);
        }
        /* Periodic durability flush + fsync. idl0_sd_append fills a 16 KB
         * cluster buffer without flushing (so it doesn't hold the shared SPI
         * bus on every batch); idl0_sd_flush pushes that buffer through AND
         * fsyncs the directory entry, so a power loss costs at most ~1 s of
         * samples and the on-disk file SIZE never freezes mid-session — FATFS
         * otherwise commits the size only at close (§10.2 / §5.3). No-op when
         * no session file is open. */
        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_flush_us >= WRITER_FLUSH_PERIOD_US) {
            idl0_sd_flush();
            last_flush_us = now_us;
        }
    }
}

bool idl0_writer_init(void)
{
    if (s_task != NULL) {
        return true;
    }
    s_rb = xRingbufferCreate(IDL0_WRITER_BUFFER_BYTES, RINGBUF_TYPE_NOSPLIT);
    if (s_rb == NULL) {
        ESP_LOGE(TAG, "ring buffer alloc failed");
        return false;
    }
    BaseType_t ok = xTaskCreate(writer_task_fn, "writer", 4096, NULL, 5, &s_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        return false;
    }
    ESP_LOGI(TAG, "writer task up (%u-byte buffer)", IDL0_WRITER_BUFFER_BYTES);
    return true;
}

bool idl0_writer_submit(const uint8_t *buf, size_t len)
{
    if (s_rb == NULL) {
        return false;
    }
    /* Drop on full — never block the producer. SESSION_END uses a longer
     * timeout via _drain (caller's responsibility). No log here: the IMU task
     * accounts for the drop and reports a count in its 5 s diag line — a
     * per-drop ESP_LOGW floods the console and feeds back into more drops. */
    BaseType_t sent = xRingbufferSend(s_rb, buf, len, 0);
    if (sent != pdTRUE) {
        return false;
    }
    return true;
}

bool idl0_writer_drain(uint32_t timeout_ms)
{
    if (s_rb == NULL) {
        return true;
    }
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        UBaseType_t items = 0;
        vRingbufferGetInfo(s_rb, NULL, NULL, NULL, NULL, &items);
        if (items == 0) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed += 10;
    }
    ESP_LOGW(TAG, "drain timeout (%u ms) — items still queued", (unsigned)timeout_ms);
    return false;
}
