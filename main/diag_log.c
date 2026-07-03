/* On-SD diagnostic log — see diag_log.h. */

#include "diag_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "ble_service.h"   /* idl0_ble_is_suspended */
#include "sd_logger.h"     /* IDL0_SD_MOUNT_POINT, idl0_sd_state / _str */
#include "session.h"       /* idl0_session_is_running */
#include "wifi_server.h"   /* idl0_wifi_state / _str */

static const char *TAG = "diag";

#define DIAG_PATH        IDL0_SD_MOUNT_POINT "/idl0_debug.log"
#define DIAG_PERIOD_MS   15000   /* heap sample cadence */
#define DIAG_MAX_BYTES   (512 * 1024)  /* rotate at 512 KB so it never grows unbounded */
#define DIAG_LINE_MAX    160
/* Event message must leave room for the "t=<u32> EVT " prefix (≤17 chars)
 * inside DIAG_LINE_MAX, so the compose snprintf can't truncate. */
#define DIAG_MSG_MAX     128

static SemaphoreHandle_t s_mutex;
static size_t            s_bytes;   /* approx bytes written since last rotate */

/* Human-readable esp_reset_reason(). A BROWNOUT here across boots = battery sag. */
static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_SW:        return "SW";       /* esp_restart (e.g. config apply / OTA) */
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_BROWNOUT:  return "BROWNOUT"; /* battery / supply sag */
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_EXT:       return "EXT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "OTHER";
    }
}

/* Append one already-formatted line (no newline) under the mutex, rotating the
 * file if it has grown past the cap. No-op without SD / before init. */
static void diag_append(const char *line)
{
    if (s_mutex == NULL || idl0_sd_state() == IDL0_SD_ABSENT) {
        return;
    }
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
        return;
    }
    /* Rotate (truncate) rather than grow unbounded. */
    const char *mode = "a";
    if (s_bytes > DIAG_MAX_BYTES) {
        mode = "w";
        s_bytes = 0;
    }
    FILE *f = fopen(DIAG_PATH, mode);
    if (f != NULL) {
        int n = fprintf(f, "%s\n", line);
        fclose(f);
        if (n > 0) {
            s_bytes += (size_t)n;
        }
    }
    xSemaphoreGive(s_mutex);
}

/* Monotonic uptime in whole seconds for the line prefix. */
static uint32_t uptime_s(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000LL);
}

void idl0_diag_log_event(const char *fmt, ...)
{
    char msg[DIAG_MSG_MAX];
    va_list ap;
    va_start(ap, fmt);
    int m = vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (m < 0) {
        return;
    }
    char line[DIAG_LINE_MAX];
    snprintf(line, sizeof(line), "t=%lu EVT %s", (unsigned long)uptime_s(), msg);
    diag_append(line);
}

static void diag_sample(void)
{
    char line[DIAG_LINE_MAX];
    snprintf(line, sizeof(line),
             "t=%lu heap=%lu min=%lu frag=%lu wifi=%s ble_susp=%d sd=%s",
             (unsigned long)uptime_s(),
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)esp_get_minimum_free_heap_size(),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),
             idl0_wifi_state_str(idl0_wifi_state()),
             idl0_ble_is_suspended() ? 1 : 0,
             idl0_sd_state_str(idl0_sd_state()));
    diag_append(line);
}

static void diag_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(DIAG_PERIOD_MS));
        /* §1: never write while a logging session owns the SD card. */
        if (idl0_session_is_running()) {
            continue;
        }
        diag_sample();
    }
}

void idl0_diag_log_init(void)
{
    if (s_mutex != NULL) {
        return;  /* already inited */
    }
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGW(TAG, "mutex alloc failed — diag log disabled");
        return;
    }

    /* Boot marker. APPEND (not truncate) so a mid-session brownout/crash reset
     * leaves a visible trail across reboots — the size cap bounds the file. */
    char line[DIAG_LINE_MAX];
    snprintf(line, sizeof(line),
             "==== BOOT reason=%s heap=%lu ====",
             reset_reason_str(esp_reset_reason()),
             (unsigned long)esp_get_free_heap_size());
    /* Pre-load the rotate counter so we don't grow past the cap across boots.
     * Cheap stat via fseek to end. */
    if (idl0_sd_state() != IDL0_SD_ABSENT) {
        FILE *f = fopen(DIAG_PATH, "a");
        if (f != NULL) {
            fseek(f, 0, SEEK_END);
            s_bytes = (size_t)ftell(f);
            fclose(f);
        }
    }
    diag_append(line);

    if (xTaskCreate(diag_task, "diag", 3072, NULL, 1, NULL) != pdPASS) {
        ESP_LOGW(TAG, "diag task create failed");
    } else {
        ESP_LOGI(TAG, "diag log started: %s", DIAG_PATH);
    }
}
