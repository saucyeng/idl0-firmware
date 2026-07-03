/* BLE Heart Rate Monitor task.
 *
 * NimBLE central role on the same host instance as the peripheral GATT
 * (§7.5). Scans for the configured address, connects, discovers Heart
 * Rate Service (0x180D), subscribes to Heart Rate Measurement (0x2A37),
 * reads Battery Level (0x2A19) once. Phase 3.3 adds the notification
 * parser; Phase 3.4 wires the state machine.
 */

#include "hrm_task.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"

#include "idl0_format.h"
#include "mode_state.h"
#include "session.h"
#include "status.h"
#include "writer_task.h"

static const char *TAG = "hrm";

typedef enum {
    EV_CONFIG_UPDATED,
    EV_WIFI_BIT_UP,    /* was EV_WIFI_ON */
    EV_WIFI_BIT_DOWN,  /* was EV_WIFI_OFF */
} hrm_event_t;

static QueueHandle_t s_event_queue = NULL;
static TaskHandle_t  s_task        = NULL;
static hrm_config_t  s_config;
static hrm_state_t   s_state       = HRM_STATE_OFF;
static uint8_t       s_latest_bpm  = 0;
static bool          s_no_contact  = false;
static uint8_t       s_battery_pct = 0;
static bool          s_has_battery = false;

static uint16_t      s_conn_handle = 0xFFFF;

/* Standard HRS / Battery Service UUIDs. */
#define HR_SVC_UUID16       0x180D
#define HR_MEAS_CHR_UUID16  0x2A37
#define BAT_SVC_UUID16      0x180F
#define BAT_LVL_CHR_UUID16  0x2A19

static uint16_t s_hr_meas_val_handle = 0;

static void run_state_machine(hrm_event_t ev);
static void start_scan(void);
static int  hrm_gap_event_cb(struct ble_gap_event *event, void *arg);

static void start_service_discovery(uint16_t conn_handle);
/* Phase 3.3 fills this in. */
static void handle_hr_notification(struct os_mbuf *om);

/* Wait briefly for the NimBLE host to finish syncing with the controller.
 * ble_service.c initialises NimBLE; we only call this from the hrm_task
 * context so it's safe to vTaskDelay. */
static bool wait_for_ble_sync(uint32_t max_ms)
{
    const uint32_t step = 50;
    uint32_t waited = 0;
    while (!ble_hs_synced()) {
        if (waited >= max_ms) return false;
        vTaskDelay(pdMS_TO_TICKS(step));
        waited += step;
    }
    return true;
}

static void start_scan(void)
{
    if (!wait_for_ble_sync(2000)) {
        ESP_LOGW(TAG, "BLE host not synced; will retry on next event");
        return;
    }
    /* Active scan so we capture scan-response payloads (some straps tuck
     * the HRS UUID into the response rather than the primary advert).
     * filter_duplicates=1 avoids re-reporting the same advertiser over
     * and over while we're already trying to match. */
    struct ble_gap_disc_params disc_params = {
        .itvl = 0x10,                 /* 10 ms */
        .window = 0x10,
        .filter_policy = 0,
        .limited = 0,
        .passive = 0,
        .filter_duplicates = 1,
    };
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params,
                          hrm_gap_event_cb, NULL);
    ESP_LOGI(TAG, "start_scan: ble_gap_disc rc=%d", rc);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "ble_gap_disc failed rc=%d", rc);
    }
}

static int hrm_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
        case BLE_GAP_EVENT_DISC: {
            if (memcmp(event->disc.addr.val, s_config.address, HRM_ADDRESS_LEN) != 0) {
                return 0;
            }
            ESP_LOGI(TAG, "found target HRM; connecting");
            (void)ble_gap_disc_cancel();
            s_state = HRM_STATE_CONNECTING;
            struct ble_gap_conn_params cp = {
                .scan_itvl = 0x10,
                .scan_window = 0x10,
                .itvl_min = 24,   /* 30 ms — dual-conn safe minimum */
                .itvl_max = 40,   /* 50 ms */
                .latency = 0,
                .supervision_timeout = 0x100,  /* 2.56 s */
                .min_ce_len = 0,
                .max_ce_len = 0,
            };
            int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr,
                                     30000, &cp, hrm_gap_event_cb, NULL);
            if (rc != 0) {
                ESP_LOGW(TAG, "ble_gap_connect rc=%d; rescanning", rc);
                s_state = HRM_STATE_SCANNING;
                start_scan();
            }
            return 0;
        }
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                s_conn_handle = event->connect.conn_handle;
                s_state = HRM_STATE_DISCOVERING;
                start_service_discovery(s_conn_handle);
            } else {
                ESP_LOGW(TAG, "connect failed status=%d; rescanning",
                         event->connect.status);
                s_state = HRM_STATE_SCANNING;
                start_scan();
            }
            return 0;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGW(TAG, "disconnect reason=%d", event->disconnect.reason);
            s_conn_handle = 0xFFFF;
            s_has_battery = false;
            if (s_state != HRM_STATE_SUSPENDED && s_config.enabled) {
                s_state = HRM_STATE_SCANNING;
                start_scan();
            }
            idl0_status_publish();
            return 0;
        case BLE_GAP_EVENT_NOTIFY_RX:
            handle_hr_notification(event->notify_rx.om);
            return 0;
        default:
            return 0;
    }
}

static int on_battery_read(uint16_t conn_handle,
                           const struct ble_gatt_error *err,
                           struct ble_gatt_attr *attr,
                           void *arg)
{
    (void)conn_handle;
    (void)arg;
    if (err == NULL || err->status == 0) {
        if (attr != NULL && attr->om != NULL &&
            OS_MBUF_PKTLEN(attr->om) >= 1) {
            uint8_t pct = 0;
            ble_hs_mbuf_to_flat(attr->om, &pct, 1, NULL);
            s_battery_pct = pct;
            s_has_battery = true;
            ESP_LOGI(TAG, "HRM battery: %u%%", (unsigned)pct);
            idl0_status_publish();
        }
    } else if (err->status != BLE_HS_EDONE) {
        /* EDONE = normal end-of-iteration signal from ble_gattc_read_by_uuid,
         * not a failure. Only log real errors. */
        ESP_LOGW(TAG, "battery read failed status=%d", err->status);
    }
    return 0;
}

static int on_subscribe_complete(uint16_t conn_handle,
                                 const struct ble_gatt_error *err,
                                 struct ble_gatt_attr *attr,
                                 void *arg)
{
    (void)attr;
    (void)arg;
    if (err != NULL && err->status != 0) {
        ESP_LOGW(TAG, "CCCD write failed status=%d", err->status);
        return 0;
    }
    s_state = HRM_STATE_STREAMING;
    ESP_LOGI(TAG, "HR Measurement notifications subscribed");
    idl0_status_publish();

    /* One-shot battery read on connect. We don't poll — §4.5. */
    const ble_uuid_t *bat_chr = BLE_UUID16_DECLARE(BAT_LVL_CHR_UUID16);
    int rc = ble_gattc_read_by_uuid(conn_handle, 1, 0xFFFF, bat_chr,
                                    on_battery_read, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_gattc_read_by_uuid(battery) rc=%d", rc);
    }
    return 0;
}

static int on_hr_meas_chr_disc(uint16_t conn_handle,
                               const struct ble_gatt_error *err,
                               const struct ble_gatt_chr *chr,
                               void *arg)
{
    (void)arg;
    if (err != NULL && err->status != 0 &&
        err->status != BLE_HS_EDONE) {
        /* Polar H10's vendor-specific services confuse NimBLE's discovery
         * iterator and EBADDATA (10) fires near end-of-iteration, AFTER
         * our HR characteristic match was already returned and the CCCD
         * subscribe initiated. Logged at DEBUG so a real strap-wide
         * discovery failure still surfaces if you crank the log level. */
        ESP_LOGD(TAG, "HR Meas chr discovery err=%d", err->status);
        return 0;
    }
    if (chr != NULL) {
        s_hr_meas_val_handle = chr->val_handle;
        /* Subscribe via CCCD write (handle = value_handle + 1 on
         * standards-compliant servers; works for all known HRMs). */
        uint8_t cccd[2] = { 0x01, 0x00 };
        int rc = ble_gattc_write_flat(conn_handle, chr->val_handle + 1,
                                      cccd, sizeof(cccd),
                                      on_subscribe_complete, NULL);
        if (rc != 0) {
            ESP_LOGW(TAG, "CCCD write rc=%d", rc);
        }
    }
    return 0;
}

static void start_service_discovery(uint16_t conn_handle)
{
    const ble_uuid_t *hr_chr = BLE_UUID16_DECLARE(HR_MEAS_CHR_UUID16);
    int rc = ble_gattc_disc_chrs_by_uuid(conn_handle, 1, 0xFFFF, hr_chr,
                                         on_hr_meas_chr_disc, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gattc_disc_chrs_by_uuid(HR_MEAS) rc=%d", rc);
    }
}

/* Submit one CHANNEL_SAMPLE record via the standard writer pipeline.
 * Builds the framed record on the stack — encoders never allocate. */
static void submit_channel_sample(uint8_t channel_id, int64_t timestamp_us,
                                  const void *value, size_t value_len)
{
    /* 3-byte framing + 1 channel_id + 8 timestamp_us + 8 max value = 20. */
    uint8_t buf[24];
    size_t n = idl0_format_channel_sample(buf, channel_id, timestamp_us,
                                          value, value_len);
    if (n == 0) return;
    (void)idl0_writer_submit(buf, n);
}

/* Parse one HR Measurement notification per the BLE HRS spec:
 *
 *   flags:u8
 *     bit 0: HR value format — 0=u8, 1=u16
 *     bit 1: sensor contact status (only meaningful if bit 2 set)
 *     bit 2: sensor contact supported
 *     bit 3: energy expended present (u16, kJ) — skipped
 *     bit 4: RR-interval(s) present
 *   bpm: u8 or u16
 *   [energy_expended: u16]   (if flag bit 3)
 *   [rr_intervals: u16 × N]  (most-recent-first, ticks of 1/1024 s)
 *
 * The most recent RR ended at the notification arrival time; older RRs
 * ended one rr_ticks period earlier each. Each beat becomes a
 * CHANNEL_SAMPLE on channel 23 with timestamp = end-of-RR; the BPM
 * field becomes a CHANNEL_SAMPLE on channel 22.
 *
 * Records are only emitted while a session is running — the channel
 * registry (§5.2) carries entries for 22/23 only when HRM is enabled
 * at session start. */
static void handle_hr_notification(struct os_mbuf *om)
{
    if (om == NULL) return;
    int len = OS_MBUF_PKTLEN(om);
    if (len < 2) return;
    uint8_t buf[32];
    if (len > (int)sizeof(buf)) len = (int)sizeof(buf);
    if (ble_hs_mbuf_to_flat(om, buf, (uint16_t)len, NULL) != 0) return;

    const int64_t now_us = esp_timer_get_time();

    const uint8_t flags = buf[0];
    const bool hr_u16            = (flags & 0x01) != 0;
    const bool contact_supported = (flags & 0x04) != 0;
    const bool contact_ok        = (flags & 0x02) != 0;
    const bool energy_present    = (flags & 0x08) != 0;
    const bool rr_present        = (flags & 0x10) != 0;

    int idx = 1;
    uint16_t bpm = 0;
    if (hr_u16) {
        if (len < idx + 2) return;
        bpm = (uint16_t)(buf[idx] | (buf[idx + 1] << 8));
        idx += 2;
    } else {
        if (len < idx + 1) return;
        bpm = buf[idx++];
    }
    s_latest_bpm = (uint8_t)(bpm > 255 ? 255 : bpm);
    s_no_contact = contact_supported && !contact_ok;

    if (energy_present) {
        if (len < idx + 2) return;
        idx += 2;   /* energy expended u16 — discarded */
    }

    /* Records only flow during an active session. */
    if (!idl0_session_is_running()) return;

    /* HR_BPM (channel 22) — u8 BPM. */
    uint8_t bpm_u8 = s_latest_bpm;
    submit_channel_sample(s_config.hr_channel_id, now_us, &bpm_u8, sizeof(bpm_u8));

    if (!rr_present) return;

    /* Walk RR intervals (most-recent-first). end_us is the moment the
     * RR being processed ended; the next RR ended one period earlier. */
    int64_t end_us = now_us;
    while (idx + 2 <= len) {
        uint16_t rr_ticks = (uint16_t)(buf[idx] | (buf[idx + 1] << 8));
        idx += 2;
        submit_channel_sample(s_config.rr_channel_id, end_us,
                              &rr_ticks, sizeof(rr_ticks));
        end_us -= ((int64_t)rr_ticks * 1000000) / 1024;
    }
}

static void hrm_task_fn(void *arg)
{
    (void)arg;
    bool prev_wifi = (idl0_mode_get_bits() & IDL0_MODE_BIT_WIFI_UP) != 0;
    for (;;) {
        /* Block on the queue for up to 250 ms so EV_CONFIG_UPDATED still
         * arrives promptly, then check the event group for a WIFI_UP
         * bit edge. The 250 ms cadence is the worst-case latency from
         * a WiFi state change to hrm_task reacting. */
        hrm_event_t ev;
        if (xQueueReceive(s_event_queue, &ev, pdMS_TO_TICKS(250)) == pdTRUE) {
            run_state_machine(ev);
        }
        const bool wifi_up = (idl0_mode_get_bits() & IDL0_MODE_BIT_WIFI_UP) != 0;
        if (wifi_up != prev_wifi) {
            run_state_machine(wifi_up ? EV_WIFI_BIT_UP : EV_WIFI_BIT_DOWN);
            prev_wifi = wifi_up;
        }
    }
}

static void run_state_machine(hrm_event_t ev)
{
    switch (ev) {
        case EV_CONFIG_UPDATED:
            /* First-kick path: hrm_task_start sets state=SCANNING and
             * queues this event. Also re-entered if config_updated fires
             * from OFF (future hot-reload). */
            if (s_config.enabled &&
                (s_state == HRM_STATE_SCANNING || s_state == HRM_STATE_OFF)) {
                s_state = HRM_STATE_SCANNING;
                start_scan();
            }
            break;

        case EV_WIFI_BIT_UP:
            ESP_LOGI(TAG, "EV_WIFI_BIT_UP; state=%d", (int)s_state);
            /* §10.4 — drop the HRM link while SoftAP is up. Active
             * connection: terminate it (DISCONNECT event fires; the
             * SUSPENDED check there prevents auto-rescan). Active scan:
             * cancel discovery. Already SUSPENDED: nothing to do. */
            if (s_state == HRM_STATE_STREAMING ||
                s_state == HRM_STATE_CONNECTING ||
                s_state == HRM_STATE_DISCOVERING) {
                s_state = HRM_STATE_SUSPENDED;
                if (s_conn_handle != 0xFFFF) {
                    (void)ble_gap_terminate(s_conn_handle,
                                            BLE_ERR_REM_USER_CONN_TERM);
                }
                idl0_status_publish();
            } else if (s_state == HRM_STATE_SCANNING) {
                s_state = HRM_STATE_SUSPENDED;
                (void)ble_gap_disc_cancel();
                idl0_status_publish();
            }
            break;

        case EV_WIFI_BIT_DOWN:
            ESP_LOGI(TAG, "EV_WIFI_BIT_DOWN; state=%d enabled=%d",
                     (int)s_state, (int)s_config.enabled);
            if (s_state == HRM_STATE_SUSPENDED && s_config.enabled) {
                s_state = HRM_STATE_SCANNING;
                start_scan();
                idl0_status_publish();
            }
            break;
    }
}

void hrm_task_start(const hrm_config_t *config)
{
    if (config == NULL) return;
    memcpy(&s_config, config, sizeof(s_config));
    if (!s_config.enabled) {
        ESP_LOGI(TAG, "HRM disabled in config — task not started");
        s_state = HRM_STATE_OFF;
        return;
    }
    if (s_event_queue == NULL) {
        s_event_queue = xQueueCreate(8, sizeof(hrm_event_t));
        if (s_event_queue == NULL) {
            ESP_LOGE(TAG, "queue create failed");
            return;
        }
    }
    if (s_task == NULL) {
        BaseType_t rc = xTaskCreate(hrm_task_fn, "hrm_task", 4096, NULL, 5, &s_task);
        if (rc != pdPASS) {
            ESP_LOGE(TAG, "task create failed (rc=%d)", (int)rc);
            s_task = NULL;
            return;
        }
    }
    s_state = HRM_STATE_SCANNING;
    hrm_event_t kick = EV_CONFIG_UPDATED;
    (void)xQueueSend(s_event_queue, &kick, 0);
    ESP_LOGI(TAG, "HRM task started; target name=\"%s\"", s_config.name);
}

void hrm_task_stop(void)
{
    if (s_task != NULL) {
        vTaskDelete(s_task);
        s_task = NULL;
    }
    if (s_event_queue != NULL) {
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
    }
    s_state = HRM_STATE_OFF;
}

hrm_state_t hrm_task_state(void)       { return s_state; }
uint8_t     hrm_task_latest_bpm(void)  { return s_latest_bpm; }
bool        hrm_task_no_contact(void)  { return s_no_contact; }
uint8_t     hrm_task_battery_pct(void) { return s_battery_pct; }
bool        hrm_task_has_battery(void) { return s_has_battery; }
