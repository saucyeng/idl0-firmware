#include "status.h"

#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ble_service.h"
#include "gps_driver.h"
#include "hrm_task.h"
#include "imu_driver.h"
#include "ota.h"
#include "sd_logger.h"
#include "session.h"
#include "wifi_server.h"

static const char *TAG = "status";

static esp_timer_handle_t s_publish_timer = NULL;

/* Sized for the §7.3 nine-line worst case (six base + OTA + HR + HR_Battery).
 * Each line max ~30 bytes; 256 gives ample headroom and matches the BLE
 * preferred MTU (§6 — CONFIG_BT_NIMBLE_ATT_PREFERRED_MTU=256). Keep in
 * lockstep with s_status in ble_service.c. */
#define IDL0_STATUS_BUF_BYTES 256

void idl0_status_publish(void)
{
    char status[IDL0_STATUS_BUF_BYTES];
    /* TODO(idl0): pull Battery from the battery driver (§10.1) once
     * that subsystem lands. */
    int n = snprintf(status, sizeof(status),
             "WiFi: %s\nLogging: %s\nBattery: 100%%\nSD: %s\nGPS: %s\nIMU: %s\nFirmware: %s",
             idl0_wifi_state_str(idl0_wifi_state()),
             idl0_session_is_running() ? "RUNNING" : "STOPPED",
             idl0_sd_state_str(idl0_sd_state()),
             idl0_gps_state_str(idl0_gps_state()),
             idl0_imu_state_str(idl0_imu_state()),
             esp_app_get_description()->version);
    if (n < 0) return;
    size_t cursor = (size_t)n;

    if (cursor < sizeof(status) && idl0_ota_pending_verify()) {
        int add = snprintf(status + cursor, sizeof(status) - cursor,
                           "\nOTA: PENDING_VERIFY");
        if (add > 0) cursor += (size_t)add;
    }

    /* §7.3 HR line. State mapping:
     *   OFF                          → ABSENT
     *   STREAMING + contact ok       → CONNECTED <bpm>
     *   STREAMING + no contact       → NO_CONTACT <bpm>
     *   SUSPENDED                    → SUSPENDED
     *   SCANNING / CONNECTING / DISCOVERING → SEARCHING */
    const hrm_state_t hr_state = hrm_task_state();
    if (cursor < sizeof(status)) {
        int add = 0;
        switch (hr_state) {
            case HRM_STATE_OFF:
                add = snprintf(status + cursor, sizeof(status) - cursor,
                               "\nHR: ABSENT");
                break;
            case HRM_STATE_STREAMING:
                add = snprintf(status + cursor, sizeof(status) - cursor,
                               "\nHR: %s %u",
                               hrm_task_no_contact() ? "NO_CONTACT" : "CONNECTED",
                               (unsigned)hrm_task_latest_bpm());
                break;
            case HRM_STATE_SUSPENDED:
                add = snprintf(status + cursor, sizeof(status) - cursor,
                               "\nHR: SUSPENDED");
                break;
            case HRM_STATE_SCANNING:
            case HRM_STATE_CONNECTING:
            case HRM_STATE_DISCOVERING:
            default:
                add = snprintf(status + cursor, sizeof(status) - cursor,
                               "\nHR: SEARCHING");
                break;
        }
        if (add > 0) cursor += (size_t)add;
    }

    if (cursor < sizeof(status) && hrm_task_has_battery()) {
        snprintf(status + cursor, sizeof(status) - cursor,
                 "\nHR_Battery: %u%%", (unsigned)hrm_task_battery_pct());
    }

    idl0_ble_publish_status(status);
}

static void publish_timer_cb(void *arg)
{
    (void)arg;
    idl0_status_publish();
}

void idl0_status_publisher_start(void)
{
    if (s_publish_timer != NULL) return;
    const esp_timer_create_args_t args = {
        .callback = publish_timer_cb,
        .arg      = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "status_pub",
        .skip_unhandled_events = true,
    };
    esp_err_t err = esp_timer_create(&args, &s_publish_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create: %s", esp_err_to_name(err));
        return;
    }
    /* 1 Hz — keeps BPM, NO_CONTACT, and any state we don't explicitly
     * hook fresh on the FF04 characteristic at ~1 s lag worst case. */
    err = esp_timer_start_periodic(s_publish_timer, 1000 * 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_start_periodic: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "publisher started at 1 Hz");
}
