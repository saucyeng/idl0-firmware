/* IDL0 BLE control plane — NimBLE peripheral.
 *
 * Implements ble_service.h. See §7 of docs/IDL0_SPEC.md for the GATT
 * contract. Task 3 scope: host init + advertising. The GATT service
 * itself is added in Task 4.
 */

#include "ble_service.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "device_id.h"
#include "diag_log.h"
#include "idl0_config.h"

static const char *TAG = "ble";

/* 16-bit service UUID: spec's 000000FF-0000-1000-8000-00805F9B34FB is
 * the Bluetooth-base expansion of 0x00FF. */
#define IDL0_BLE_SVC_UUID16   0x00FF

/* Caller's command callback, stored by idl0_ble_init. */
static idl0_ble_command_cb s_cmd_cb = NULL;

/* Address type resolved at sync time, used when advertising. */
static uint8_t s_own_addr_type;

/* Characteristic 16-bit UUIDs. */
#define IDL0_BLE_CHR_CONTROL_UUID16  0xFF03
#define IDL0_BLE_CHR_STATUS_UUID16   0xFF04
#define IDL0_BLE_CHR_CONFIG_RX_UUID16 0xFF05  /* config push reassembly (§7.2) */
#define IDL0_BLE_CHR_CONFIG_TX_UUID16 0xFF06  /* config read-back (§7.2) */

/* Upper bound on a reassembled config blob — matches CONFIG_MAX_BYTES in
 * idl0_config.c / the WiFi POST /config cap. The buffer is heap-allocated on
 * CMD_CONFIG_BEGIN and freed on COMMIT / abort so it costs nothing at idle. */
#define IDL0_BLE_CONFIG_MAX  8192

/* Bytes served per FF06 read. Kept well under the negotiated ATT MTU (256 at
 * connect, the same assumption the app's push chunking relies on) so each read
 * returns in a single ATT_READ_RSP — no Read Blob, one chunk per read(). */
#define IDL0_BLE_CONFIG_TX_CHUNK 200

/* FF05 reassembly state. Written only from the NimBLE host task (the GATT
 * access callbacks all run there), so no extra locking is needed. */
static char  *s_cfg_buf;     /* NULL unless a CONFIG_BEGIN..COMMIT is in flight */
static size_t s_cfg_len;     /* bytes accumulated so far */
static bool   s_cfg_overflow; /* a chunk would have exceeded the cap */

/* FF06 read-back state — snapshot of idl0_config.json served chunk-by-chunk.
 * Same single-task access model as the RX buffer. */
static char  *s_cfg_tx_buf;  /* NULL unless a read-back is in flight */
static size_t s_cfg_tx_len;  /* snapshot length */
static size_t s_cfg_tx_pos;  /* bytes already served */

/* Current connection, or BLE_HS_CONN_HANDLE_NONE when not connected. */
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

/* Value handle of the status characteristic, filled in by the stack at
 * registration; needed to send notifications. */
static uint16_t s_status_val_handle;

/* Latest status string (§7.3). Served on READ and sent on NOTIFY.
 * Sized for the nine §7.3 lines worst case (six base + optional OTA,
 * HR, HR_Battery). The publisher in status.c builds into a matching
 * 256 B buffer — keep both in lockstep. */
static char s_status[256] = "WiFi: OFF\nLogging: STOPPED\nBattery: 100%";

/* True while the §10.4 radio handoff has BLE suspended (WiFi mode owns
 * the radio). Written from the httpd task (/handoff) and whichever task
 * runs idl0_wifi_stop(); read by the NimBLE host task in gap_event_cb. */
static volatile bool s_suspended = false;

static void start_advertising(void);

/* Free the FF05 reassembly buffer and clear its state. Safe to call when no
 * transfer is in flight (idempotent). */
static void config_buf_reset(void)
{
    free(s_cfg_buf);
    s_cfg_buf = NULL;
    s_cfg_len = 0;
    s_cfg_overflow = false;
}

/* CMD_CONFIG_BEGIN (0x07): (re)allocate the reassembly buffer and reset
 * length so the app can stream a fresh config blob to FF05. Returns the ATT
 * ack code for the FF03 write response. */
static uint8_t config_begin(void)
{
    config_buf_reset();
    s_cfg_buf = malloc(IDL0_BLE_CONFIG_MAX);
    if (s_cfg_buf == NULL) {
        ESP_LOGE(TAG, "config begin: buffer alloc failed");
        return IDL0_ACK_BUSY;  /* transient: app may retry */
    }
    idl0_diag_log_event("ble config begin");
    return IDL0_ACK_OK;
}

/* One-shot task: deliver the COMMIT write response, then reboot to apply the
 * new config (read at boot only, §4.2). The 500 ms delay lets the GATT ACK
 * and a final status notify flush so the app confirms the push before the
 * link drops; the app then re-establishes BLE and lands back in idle mode. */
static void config_restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "config committed — rebooting to apply");
    esp_restart();
}

/* CMD_CONFIG_COMMIT (0x08): validate the reassembled blob, atomically write
 * idl0_config.json, and schedule a reboot. Returns the ATT ack for FF03. */
static uint8_t config_commit(void)
{
    if (s_cfg_buf == NULL || s_cfg_len == 0) {
        ESP_LOGW(TAG, "config commit with no buffered data");
        return IDL0_ACK_PRECONDITION;
    }
    if (s_cfg_overflow) {
        ESP_LOGW(TAG, "config commit rejected: blob exceeded %d bytes",
                 IDL0_BLE_CONFIG_MAX);
        config_buf_reset();
        return IDL0_ACK_PRECONDITION;
    }

    idl0_config_write_result_t wr = idl0_config_write_json(s_cfg_buf, s_cfg_len);
    config_buf_reset();
    switch (wr) {
        case IDL0_CONFIG_WRITE_BAD_JSON:
            ESP_LOGW(TAG, "config commit rejected: malformed JSON");
            return IDL0_ACK_PRECONDITION;
        case IDL0_CONFIG_WRITE_IO_ERROR:
            ESP_LOGE(TAG, "config commit failed: SD write error");
            return IDL0_ACK_BUSY;  /* transient: app may retry */
        case IDL0_CONFIG_WRITE_OK:
            break;
    }

    idl0_diag_log_event("ble config commit");
    if (xTaskCreate(config_restart_task, "cfg_restart", 2048, NULL, 5, NULL)
            != pdPASS) {
        /* Could not schedule the deferred reboot — fall back to an immediate
         * restart. The app's reconnect-after-reboot path handles the slightly
         * earlier link drop. */
        ESP_LOGW(TAG, "restart task create failed — restarting now");
        esp_restart();
    }
    return IDL0_ACK_OK;
}

/* Config-RX characteristic (FF05) — write-only. Appends each write to the
 * reassembly buffer opened by CMD_CONFIG_BEGIN. A write before BEGIN, or one
 * that would exceed the cap, is recorded as an overflow/error so the eventual
 * COMMIT refuses cleanly rather than persisting a partial blob. */
static int config_rx_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (s_cfg_buf == NULL) {
        /* No CONFIG_BEGIN — reject so the app re-issues the handshake. */
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len == 0) {
        return 0;  /* empty write — nothing to append */
    }
    if (s_cfg_len + len > IDL0_BLE_CONFIG_MAX) {
        /* Latch overflow; COMMIT will refuse. Don't write past the buffer. */
        s_cfg_overflow = true;
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    int rc = ble_hs_mbuf_to_flat(ctxt->om, s_cfg_buf + s_cfg_len, len, NULL);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    s_cfg_len += len;
    return 0;
}

/* Free the FF06 read-back snapshot and clear its cursor. Idempotent. */
static void config_tx_reset(void)
{
    free(s_cfg_tx_buf);
    s_cfg_tx_buf = NULL;
    s_cfg_tx_len = 0;
    s_cfg_tx_pos = 0;
}

/* CMD_CONFIG_READ_BEGIN (0x09): snapshot idl0_config.json into a heap buffer
 * and reset the FF06 cursor so the app can read it back chunk-by-chunk.
 * Returns the ATT ack for the FF03 write response. */
static uint8_t config_read_begin(void)
{
    config_tx_reset();

    FILE *f = fopen(IDL0_CONFIG_PATH, "rb");
    if (f == NULL) {
        ESP_LOGW(TAG, "config read: %s not found", IDL0_CONFIG_PATH);
        return IDL0_ACK_PRECONDITION;  /* no config to read */
    }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > IDL0_BLE_CONFIG_MAX) {
        fclose(f);
        ESP_LOGW(TAG, "config read: bad size %ld", sz);
        return IDL0_ACK_PRECONDITION;
    }

    s_cfg_tx_buf = malloc((size_t)sz);
    if (s_cfg_tx_buf == NULL) {
        fclose(f);
        return IDL0_ACK_BUSY;  /* transient: app may retry */
    }
    size_t rd = fread(s_cfg_tx_buf, 1, (size_t)sz, f);
    fclose(f);
    if (rd != (size_t)sz) {
        config_tx_reset();
        ESP_LOGE(TAG, "config read: short read %u/%ld", (unsigned)rd, sz);
        return IDL0_ACK_BUSY;
    }
    s_cfg_tx_len = (size_t)sz;
    s_cfg_tx_pos = 0;
    idl0_diag_log_event("ble config read begin");
    return IDL0_ACK_OK;
}

/* Config-TX characteristic (FF06) — read-only. Each read serves the next
 * IDL0_BLE_CONFIG_TX_CHUNK bytes of the snapshot and advances the cursor; an
 * empty read means EOF (cursor at end) or no read-back open. The app reads
 * until it gets an empty chunk, then reassembles + decodes the JSON. */
static int config_tx_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    if (s_cfg_tx_buf == NULL || s_cfg_tx_pos >= s_cfg_tx_len) {
        return 0;  /* empty value: no transfer open, or EOF */
    }

    size_t remaining = s_cfg_tx_len - s_cfg_tx_pos;
    size_t chunk = remaining < IDL0_BLE_CONFIG_TX_CHUNK
                       ? remaining
                       : IDL0_BLE_CONFIG_TX_CHUNK;
    int rc = os_mbuf_append(ctxt->om, s_cfg_tx_buf + s_cfg_tx_pos, chunk);
    if (rc != 0) {
        return BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    s_cfg_tx_pos += chunk;
    return 0;
}

/* Control characteristic (FF03) — write-only. Payload is a single
 * command byte (§7.2). */
static int control_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len != 1) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    uint8_t cmd = 0;
    int rc = ble_hs_mbuf_to_flat(ctxt->om, &cmd, 1, NULL);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (cmd < IDL0_CMD_WIFI_ON || cmd > IDL0_CMD_CONFIG_READ_BEGIN) {
        ESP_LOGW(TAG, "unknown control command 0x%02X", cmd);
        return BLE_ATT_ERR_INVALID_PDU;
    }

    ESP_LOGI(TAG, "control command 0x%02X", cmd);

    /* Config push/read handshakes are handled in the BLE layer (it owns the
     * FF05 / FF06 buffers), not routed to the mode-command dispatcher. */
    if (cmd == IDL0_CMD_CONFIG_BEGIN) {
        return config_begin();
    }
    if (cmd == IDL0_CMD_CONFIG_COMMIT) {
        return config_commit();
    }
    if (cmd == IDL0_CMD_CONFIG_READ_BEGIN) {
        return config_read_begin();
    }

    uint8_t ack = IDL0_ACK_OK;
    if (s_cmd_cb != NULL) {
        ack = s_cmd_cb((idl0_ble_command_t)cmd);
    }
    return ack;
}

/* Status characteristic (FF04) — readable; also pushed via notify. */
static int status_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    int rc = os_mbuf_append(ctxt->om, s_status, strlen(s_status));
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

/* GATT service table: service 0x00FF with FF03 (write) + FF04 (notify). */
static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(IDL0_BLE_SVC_UUID16),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(IDL0_BLE_CHR_CONTROL_UUID16),
                .access_cb = control_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = BLE_UUID16_DECLARE(IDL0_BLE_CHR_STATUS_UUID16),
                .access_cb = status_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_status_val_handle,
            },
            {
                .uuid = BLE_UUID16_DECLARE(IDL0_BLE_CHR_CONFIG_RX_UUID16),
                .access_cb = config_rx_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = BLE_UUID16_DECLARE(IDL0_BLE_CHR_CONFIG_TX_UUID16),
                .access_cb = config_tx_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            { 0 }  /* end of characteristics */
        },
    },
    { 0 }  /* end of services */
};

/* GAP event handler — connect / disconnect / advertising-complete. */
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "connect; status=%d", event->connect.status);
            if (event->connect.status == 0) {
                s_conn_handle = event->connect.conn_handle;
            } else if (!s_suspended) {
                start_advertising();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "disconnect; reason=%d", event->disconnect.reason);
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            /* Drop any half-streamed config blob / read-back snapshot so they
             * can't leak across the disconnect or be committed after a
             * reconnect. */
            config_buf_reset();
            config_tx_reset();
            if (!s_suspended) {
                start_advertising();
            }
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            if (!s_suspended) {
                ESP_LOGI(TAG, "advertising complete; restarting");
                start_advertising();
            }
            break;

        default:
            break;
    }
    return 0;
}

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};
    const char *name = idl0_device_name();
    static const ble_uuid16_t svc_uuid = BLE_UUID16_INIT(IDL0_BLE_SVC_UUID16);

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (const uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    fields.uuids16 = &svc_uuid;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed; rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params = {0};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;   /* undirected connectable */
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;   /* general discoverable */

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed; rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "advertising as %s", name);
}

/* Called when the host and controller have synced. */
static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed; rc=%d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed; rc=%d", rc);
        return;
    }
    start_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset; reason=%d", reason);
}

/* NimBLE host task — runs the host event loop until nimble_port_stop. */
static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

bool idl0_ble_init(idl0_ble_command_cb on_command)
{
    s_cmd_cb = on_command;

    /* The BLE controller stores PHY calibration in NVS. */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    if (nvs_err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed (%s)", esp_err_to_name(nvs_err));
        return false;
    }

    esp_err_t rc = nimble_port_init();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed (%s)", esp_err_to_name(rc));
        return false;
    }

    /* Silence the NimBLE host stack's per-notify INFO chatter
     * ("GATT procedure initiated: notify; att_handle=...") which the 1 Hz
     * status publisher emits on every frame and clogs the serial monitor.
     * WARN keeps genuine NimBLE warnings/errors; our own "ble"/"idl0" tags
     * (including "disconnect; reason=N") are unaffected. */
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int gatt_rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (gatt_rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed; rc=%d", gatt_rc);
        return false;
    }
    gatt_rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (gatt_rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed; rc=%d", gatt_rc);
        return false;
    }

    int name_rc = ble_svc_gap_device_name_set(idl0_device_name());
    if (name_rc != 0) {
        ESP_LOGE(TAG, "device_name_set failed; rc=%d", name_rc);
        return false;
    }

    nimble_port_freertos_init(ble_host_task);
    ESP_LOGI(TAG, "NimBLE host started");
    return true;
}

void idl0_ble_publish_status(const char *status_string)
{
    if (status_string == NULL) {
        return;
    }
    /* Cache for the next READ of the status characteristic. */
    strncpy(s_status, status_string, sizeof(s_status) - 1);
    s_status[sizeof(s_status) - 1] = '\0';

    /* Notify the connected central, if any. */
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(s_status, strlen(s_status));
    if (om == NULL) {
        ESP_LOGE(TAG, "status notify: mbuf alloc failed");
        return;
    }
    int rc = ble_gatts_notify_custom(s_conn_handle, s_status_val_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "status notify failed; rc=%d", rc);
    }
}

void idl0_ble_suspend(void)
{
    if (s_suspended) {
        return;
    }
    s_suspended = true;
    /* Order matters: stop advertising first so the disconnect below
     * cannot race a new inbound connection. Both calls are safe from
     * non-host tasks (NimBLE serializes via the host lock). */
    ble_gap_adv_stop();
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
    /* \302\247 is UTF-8 "§" in octal escapes — hex escapes are greedy in C
     * ("\xA710" would parse as one escape), octal stops at three digits. */
    ESP_LOGI(TAG, "suspended (radio handoff, SPEC \302\24710.4)");
    idl0_diag_log_event("ble suspend");
}

void idl0_ble_resume(void)
{
    if (!s_suspended) {
        return;
    }
    s_suspended = false;
    start_advertising();
    ESP_LOGI(TAG, "resumed - advertising");
    idl0_diag_log_event("ble resume");
}

bool idl0_ble_is_suspended(void)
{
    return s_suspended;
}
