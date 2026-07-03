/* See wifi_server.h. P8 M1: AP bring-up + state only. M2-M5 add the
 * four HTTP endpoints. */

#include "wifi_server.h"

#include <string.h>

#include "esp_app_desc.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "ble_service.h"
#include "device_id.h"
#include "diag_log.h"
#include "status.h"

#include <dirent.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/stat.h>

#include "cJSON.h"

#include "idl0_config.h"
#include "mode_state.h"
#include "ota.h"
#include "sd_logger.h"

static const char *TAG = "wifi";

#define IDL0_WIFI_PASSWORD       "datalogger123"
#define IDL0_WIFI_CHANNEL        1
#define IDL0_WIFI_MAX_CONN       4

static idl0_wifi_state_t s_state    = IDL0_WIFI_OFF;
static esp_netif_t      *s_ap_netif = NULL;
static httpd_handle_t    s_httpd    = NULL;
static bool              s_event_loop_inited = false;
static bool              s_wifi_inited       = false;

/* Last HTTP request time, µs since boot — one input to the §10.4
 * no-activity failsafe. Touched by every endpoint handler; initialised
 * in idl0_wifi_start so the window starts at AP-up. */
static int64_t s_last_http_us = 0;

static void touch_http_activity(void)
{
    s_last_http_us = esp_timer_get_time();
}

idl0_wifi_state_t idl0_wifi_state(void) { return s_state; }

const char *idl0_wifi_state_str(idl0_wifi_state_t state)
{
    switch (state) {
        case IDL0_WIFI_ON:  return "ON";
        case IDL0_WIFI_OFF:
        default:            return "OFF";
    }
}

#define IDL0_SESSIONS_DIR IDL0_SD_MOUNT_POINT "/sessions"

/* Reads ?file=<name> from the query string into `out` (NUL-terminated).
 * Returns true on success. Rejects empty names and names containing
 * '/' or '\' (path-traversal guard — only direct children of
 * /sessions/ are reachable). */
static bool read_file_param(httpd_req_t *req, char *out, size_t out_len)
{
    size_t qlen = httpd_req_get_url_query_len(req) + 1;
    if (qlen <= 1 || qlen > 256) return false;
    char query[256];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return false;
    }
    if (httpd_query_key_value(query, "file", out, out_len) != ESP_OK) {
        return false;
    }
    if (out[0] == '\0') return false;
    for (const char *p = out; *p != '\0'; p++) {
        if (*p == '/' || *p == '\\') return false;
    }
    return true;
}

/* Reads the 16-byte session UUID from the v3 header of [path] and renders it
 * as 32 lowercase hex chars into [out] (33 bytes incl. NUL), matching the
 * app's BinaryParser rendering. The UUID lives at file offset 5 — after the
 * 4-byte "IDL0" magic and 1-byte schema version (§5.1).
 *
 * Returns true on success. On a short read or magic mismatch returns false
 * and the caller omits "session_id"; the app then treats that file as
 * identity-unknown rather than failing. */
static bool read_session_id_hex(const char *path, char out[33])
{
    FILE *f = fopen(path, "rb");
    if (f == NULL) return false;
    uint8_t prefix[21];  /* 4 magic + 1 schema + 16 uuid */
    size_t got = fread(prefix, 1, sizeof(prefix), f);
    fclose(f);
    if (got < sizeof(prefix)) return false;
    if (memcmp(prefix, "IDL0", 4) != 0) return false;

    static const char hex[] = "0123456789abcdef";
    for (int i = 0; i < 16; i++) {
        uint8_t b = prefix[5 + i];
        out[i * 2]     = hex[b >> 4];
        out[i * 2 + 1] = hex[b & 0x0F];
    }
    out[32] = '\0';
    return true;
}

static esp_err_t files_get_handler(httpd_req_t *req)
{
    touch_http_activity();
    cJSON *arr = cJSON_CreateArray();
    if (arr == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    /* TEMP(idl0-debug): bracket /files enumeration so the serial timestamps
     * reveal whether the request arrived and how long header reads take.
     * Remove once the intermittent GET-timeout cause is confirmed. */
    ESP_LOGI(TAG, "GET /files: enumerating");

    DIR *dir = opendir(IDL0_SESSIONS_DIR);
    if (dir != NULL) {
        struct dirent *ent;
        while ((ent = readdir(dir)) != NULL) {
            if (ent->d_name[0] == '.') continue;  /* skip "." / ".." */
            char path[96];
            int n = snprintf(path, sizeof(path), "%s/%s",
                             IDL0_SESSIONS_DIR, ent->d_name);
            if (n < 0 || (size_t)n >= sizeof(path)) continue;
            struct stat st;
            if (stat(path, &st) != 0) continue;

            cJSON *obj = cJSON_CreateObject();
            cJSON_AddStringToObject(obj, "name", ent->d_name);
            cJSON_AddNumberToObject(obj, "size", (double)st.st_size);

            /* session_id lets the app diff device files against its library
             * without downloading them (§6.1). Omitted if the header can't
             * be read; the app then shows the file as identity-unknown. */
            char sid[33];
            if (read_session_id_hex(path, sid)) {
                cJSON_AddStringToObject(obj, "session_id", sid);
            }

            cJSON_AddItemToArray(arr, obj);
        }
        closedir(dir);
    }

    int file_count = cJSON_GetArraySize(arr);
    char *body = cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);
    if (body == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    /* TEMP(idl0-debug): pair with the "enumerating" line above — the gap
     * between the two timestamps is the firmware's per-file SD read cost. */
    ESP_LOGI(TAG, "GET /files: sending %d entries", file_count);

    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    free(body);
    return err;
}

/* 8 KB amortises chunk framing + fread + TCP send overhead. With the
 * tuned lwIP send buffer + window from Task 1b, /download hits the
 * 500 KB/s+ band the C6 AP is capable of. */
#define IDL0_DOWNLOAD_CHUNK 8192

static esp_err_t download_get_handler(httpd_req_t *req)
{
    touch_http_activity();
    char fname[64];
    if (!read_file_param(req, fname, sizeof(fname))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing or invalid 'file'");
        return ESP_FAIL;
    }

    char path[96];
    int n = snprintf(path, sizeof(path), "%s/%s", IDL0_SESSIONS_DIR, fname);
    if (n < 0 || (size_t)n >= sizeof(path)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "name too long");
        return ESP_FAIL;
    }

    struct stat st;
    if (stat(path, &st) != 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no such file");
        return ESP_FAIL;
    }
    long total = (long)st.st_size;
    long start = 0, end = total - 1;
    bool partial = false;

    /* Range header: "bytes=START-END" or "bytes=START-". */
    char range[64];
    if (httpd_req_get_hdr_value_str(req, "Range", range, sizeof(range)) == ESP_OK) {
        long rs = 0, re = -1;
        if (sscanf(range, "bytes=%ld-%ld", &rs, &re) >= 1) {
            if (rs >= 0 && rs < total) {
                start   = rs;
                end     = (re >= 0 && re < total) ? re : total - 1;
                partial = true;
            }
        }
    }

    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "fopen failed");
        return ESP_FAIL;
    }
    if (start > 0) {
        if (fseek(f, start, SEEK_SET) != 0) {
            fclose(f);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "seek failed");
            return ESP_FAIL;
        }
    }

    httpd_resp_set_type(req, "application/octet-stream");
    /* Streamed via httpd_resp_send_chunk → Transfer-Encoding: chunked.
     * We deliberately do NOT set Content-Length (mixing it with chunked
     * is undefined behaviour). For partial responses, Content-Range is
     * still meaningful and the client (Dart `http.StreamedResponse`)
     * accepts a null content length. */
    if (partial) {
        char crange[64];
        snprintf(crange, sizeof(crange), "bytes %ld-%ld/%ld", start, end, total);
        httpd_resp_set_hdr(req, "Content-Range", crange);
        httpd_resp_set_status(req, "206 Partial Content");
    }

    uint8_t buf[IDL0_DOWNLOAD_CHUNK];
    long remaining = end - start + 1;
    while (remaining > 0) {
        size_t want = (remaining > (long)sizeof(buf)) ? sizeof(buf) : (size_t)remaining;
        size_t got = fread(buf, 1, want, f);
        if (got == 0) break;
        if (httpd_resp_send_chunk(req, (const char *)buf, got) != ESP_OK) {
            fclose(f);
            return ESP_FAIL;
        }
        /* A multi-minute download is activity: without this, a transfer
         * longer than the §10.4 idle window would trip the no-HTTP
         * failsafe arm mid-stream (handlers only touch at request start). */
        touch_http_activity();
        remaining -= got;
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);  /* end of chunked stream */
    return ESP_OK;
}

static esp_err_t delete_get_handler(httpd_req_t *req)
{
    touch_http_activity();
    char fname[64];
    if (!read_file_param(req, fname, sizeof(fname))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing or invalid 'file'");
        return ESP_FAIL;
    }
    char path[96];
    int n = snprintf(path, sizeof(path), "%s/%s", IDL0_SESSIONS_DIR, fname);
    if (n < 0 || (size_t)n >= sizeof(path)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "name too long");
        return ESP_FAIL;
    }
    if (remove(path) != 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "remove failed");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "deleted");
    return ESP_OK;
}

#define IDL0_CONFIG_MAX_BYTES 8192

static esp_err_t config_post_handler(httpd_req_t *req)
{
    touch_http_activity();
    int total = req->content_len;
    if (total <= 0 || total > IDL0_CONFIG_MAX_BYTES) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad content length");
        return ESP_FAIL;
    }

    char *body = malloc((size_t)total + 1);
    if (body == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "out of memory");
        return ESP_FAIL;
    }
    int received = 0;
    while (received < total) {
        int n = httpd_req_recv(req, body + received, total - received);
        if (n <= 0) {
            free(body);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            return ESP_FAIL;
        }
        received += n;
    }
    body[total] = '\0';

    /* Validate + atomic-write is shared with the BLE config-push path
     * (FF05 + CMD_CONFIG_COMMIT, §7.2) — see idl0_config_write_json. */
    idl0_config_write_result_t wr = idl0_config_write_json(body, (size_t)total);
    free(body);
    switch (wr) {
        case IDL0_CONFIG_WRITE_BAD_JSON:
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "malformed JSON");
            return ESP_FAIL;
        case IDL0_CONFIG_WRITE_IO_ERROR:
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "write failed");
            return ESP_FAIL;
        case IDL0_CONFIG_WRITE_OK:
            break;
    }
    httpd_resp_sendstr(req, "ok");

    /* idl0_config.json is read at boot only (HRM enable/address set in
     * hrm_task_start, IMU ODR/ranges in idl0_imu_init_one), so reboot to apply
     * the new config in full — no chasing which subsystem hot-reloads. The GPS
     * module is UART-only with no power-enable GPIO (pins.h), so esp_restart()
     * resets the SoC without cutting GPS power: the u-blox keeps its fix across
     * the ~hundreds-of-ms reset. The 500 ms delay lets the 200 "ok" flush so
     * the app confirms the push before the link drops; the app then
     * re-establishes BLE and lands back in idle mode. The device boots into
     * idle (WiFi off, no session) by default. */
    ESP_LOGI(TAG, "/config saved — rebooting to apply");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;  /* unreachable */
}

/* 8 KB chunk amortises recv + esp_ota_write overhead. Same size as
 * /download — fits comfortably in the 16 KB httpd task stack. */
#define IDL0_OTA_CHUNK 8192

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    touch_http_activity();
    int total = req->content_len;
    ESP_LOGI(TAG, "/ota begin, announced size=%d", total);

    idl0_ota_session_t *s = idl0_ota_begin(total > 0 ? (size_t)total : 0);
    if (s == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota_begin failed");
        return ESP_FAIL;
    }

    char *buf = malloc(IDL0_OTA_CHUNK);
    if (buf == NULL) {
        idl0_ota_abort(s);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "alloc failed");
        return ESP_FAIL;
    }

    int received = 0;
    while (total < 0 || received < total) {
        int n = httpd_req_recv(req, buf, IDL0_OTA_CHUNK);
        if (n == 0) break;                          /* EOF (chunked) */
        if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;  /* benign retry */
        if (n < 0) {
            free(buf);
            idl0_ota_abort(s);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            return ESP_FAIL;
        }
        if (!idl0_ota_write(s, buf, (size_t)n)) {
            free(buf);
            idl0_ota_abort(s);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "flash write failed");
            return ESP_FAIL;
        }
        /* A slow OTA upload is activity — same rationale as /download:
         * don't let the no-HTTP failsafe arm fire mid-stream. */
        touch_http_activity();
        received += n;
    }
    free(buf);

    /* If Content-Length was set, the upload must have delivered exactly
     * that many bytes. A short stream means the client closed the
     * socket mid-upload — reject so we don't validate a truncated image
     * and commit it as boot. */
    if (total > 0 && received != total) {
        idl0_ota_abort(s);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "short upload");
        return ESP_FAIL;
    }

    if (!idl0_ota_end(s)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "image validation failed");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "ok\n");
    ESP_LOGI(TAG, "/ota success — rebooting in 500 ms");
    /* Defer the reboot long enough for the 200 response to flush. */
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;  /* unreachable */
}

/* GET /ping — identity + status (SPEC §6.1). The WiFi-mode status feed
 * and the app's identity/liveness probe. Allocation-light, never touches
 * the SD card: idl0_sd_state() is a cached-state accessor (same source
 * the 1 Hz BLE status publisher uses). */
static esp_err_t ping_get_handler(httpd_req_t *req)
{
    touch_http_activity();

    EventBits_t bits = idl0_mode_get_bits();
    const char *mode = (bits & IDL0_MODE_BIT_LOGGING_ACTIVE) ? "recording"
                     : (bits & IDL0_MODE_BIT_WIFI_UP)        ? "wifi"
                                                             : "idle";

    cJSON *obj = cJSON_CreateObject();
    if (obj == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    cJSON_AddStringToObject(obj, "device", idl0_device_name());
    cJSON_AddStringToObject(obj, "fw", esp_app_get_description()->version);
    cJSON_AddNumberToObject(obj, "proto", IDL0_WIFI_PROTO_VERSION);
    /* TODO(idl0): pull Battery from the battery driver (§10.1) once that
     * subsystem lands — mirrors the same TODO in status.c. */
    cJSON_AddNumberToObject(obj, "battery", 100);
    cJSON_AddStringToObject(obj, "sd", idl0_sd_state_str(idl0_sd_state()));
    cJSON_AddStringToObject(obj, "mode", mode);
    cJSON_AddStringToObject(obj, "ble", idl0_ble_is_suspended() ? "off" : "on");

    char *body = cJSON_PrintUnformatted(obj);
    cJSON_Delete(obj);
    if (body == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
    free(body);
    return err;
}

/* POST /handoff — the app confirms its HTTP link is verified; drop BLE
 * so WiFi owns the radio outright (SPEC §10.4). Respond before
 * suspending: the 200 is the contract, and the suspend has no bearing
 * on it. Idempotent — a second call finds BLE already suspended. */
static esp_err_t handoff_post_handler(httpd_req_t *req)
{
    touch_http_activity();
    httpd_resp_sendstr(req, "ok");
    ESP_LOGI(TAG, "/handoff - dropping BLE (radio handoff)");
    idl0_ble_suspend();
    return ESP_OK;
}

/* One-shot timer for the deferred AP teardown requested by POST
 * /wifi_off. Runs in the esp_timer task — NOT the httpd task — because
 * httpd_stop() joins the httpd task and would deadlock if called from a
 * handler. Created lazily, reused across WiFi sessions. */
static esp_timer_handle_t s_wifi_off_timer = NULL;

static void wifi_off_timer_cb(void *arg)
{
    (void)arg;
    idl0_wifi_stop();
    /* BLE is back (idl0_wifi_stop resumes it) — push a status frame so a
     * reconnecting app sees WiFi: OFF immediately. */
    idl0_status_publish();
}

/* POST /wifi_off — exits WiFi mode over HTTP; the normal exit path once
 * BLE is off (SPEC §6.1, §10.4). The 500 ms delay lets the 200 flush
 * before the AP drops (same pattern as /config's reboot). */
static esp_err_t wifi_off_post_handler(httpd_req_t *req)
{
    touch_http_activity();

    if (s_wifi_off_timer == NULL) {
        const esp_timer_create_args_t args = {
            .callback = wifi_off_timer_cb,
            .name     = "wifi_off",
        };
        if (esp_timer_create(&args, &s_wifi_off_timer) != ESP_OK) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "timer create failed");
            return ESP_FAIL;
        }
    }

    httpd_resp_sendstr(req, "ok");
    ESP_LOGI(TAG, "/wifi_off - AP teardown in 500 ms");
    esp_timer_start_once(s_wifi_off_timer, 500 * 1000);
    return ESP_OK;
}

/* §10.4 no-activity failsafe: five minutes with no associated station,
 * or five minutes with no HTTP request, exits WiFi mode autonomously so
 * the device is never stranded in AP mode draining battery. The app's
 * 10 s /ping heartbeat keeps an active link alive indefinitely. */
#define IDL0_WIFI_IDLE_TIMEOUT_US   (5LL * 60 * 1000 * 1000)
#define IDL0_WIFI_FAILSAFE_POLL_MS  10000

static TaskHandle_t s_failsafe_task = NULL;

/* Last time at least one station was associated (or AP-up time). */
static int64_t s_last_sta_us = 0;

static void failsafe_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(IDL0_WIFI_FAILSAFE_POLL_MS));

        wifi_sta_list_t stas = {0};
        if (esp_wifi_ap_get_sta_list(&stas) == ESP_OK && stas.num > 0) {
            s_last_sta_us = esp_timer_get_time();
        }

        int64_t now = esp_timer_get_time();
        bool sta_idle  = (now - s_last_sta_us)  > IDL0_WIFI_IDLE_TIMEOUT_US;
        bool http_idle = (now - s_last_http_us) > IDL0_WIFI_IDLE_TIMEOUT_US;
        if (sta_idle || http_idle) {
            ESP_LOGW(TAG, "no-activity failsafe (%s) - exiting WiFi mode",
                     sta_idle ? "no station for 5 min" : "no HTTP for 5 min");
            /* Clear the handle first so idl0_wifi_stop doesn't try to
             * delete the task it is running on; we self-delete below. */
            s_failsafe_task = NULL;
            idl0_wifi_stop();
            idl0_status_publish();
            vTaskDelete(NULL);
        }
    }
}

static esp_err_t register_endpoints(httpd_handle_t srv)
{
    static const httpd_uri_t files_uri = {
        .uri = "/files", .method = HTTP_GET, .handler = files_get_handler,
    };
    static const httpd_uri_t download_uri = {
        .uri = "/download", .method = HTTP_GET, .handler = download_get_handler,
    };
    static const httpd_uri_t delete_uri = {
        .uri = "/delete", .method = HTTP_GET, .handler = delete_get_handler,
    };
    static const httpd_uri_t config_uri = {
        .uri = "/config", .method = HTTP_POST, .handler = config_post_handler,
    };
    static const httpd_uri_t ota_uri = {
        .uri = "/ota", .method = HTTP_POST, .handler = ota_post_handler,
    };
    static const httpd_uri_t ping_uri = {
        .uri = "/ping", .method = HTTP_GET, .handler = ping_get_handler,
    };
    static const httpd_uri_t handoff_uri = {
        .uri = "/handoff", .method = HTTP_POST, .handler = handoff_post_handler,
    };
    static const httpd_uri_t wifi_off_uri = {
        .uri = "/wifi_off", .method = HTTP_POST, .handler = wifi_off_post_handler,
    };
    esp_err_t err = httpd_register_uri_handler(srv, &files_uri);
    if (err == ESP_OK) err = httpd_register_uri_handler(srv, &download_uri);
    if (err == ESP_OK) err = httpd_register_uri_handler(srv, &delete_uri);
    if (err == ESP_OK) err = httpd_register_uri_handler(srv, &config_uri);
    if (err == ESP_OK) err = httpd_register_uri_handler(srv, &ota_uri);
    if (err == ESP_OK) err = httpd_register_uri_handler(srv, &ping_uri);
    if (err == ESP_OK) err = httpd_register_uri_handler(srv, &handoff_uri);
    if (err == ESP_OK) err = httpd_register_uri_handler(srv, &wifi_off_uri);
    return err;
}

static bool ensure_event_loop(void)
{
    if (s_event_loop_inited) return true;
    esp_err_t err = esp_event_loop_create_default();
    if (err == ESP_ERR_INVALID_STATE) {
        /* Already created elsewhere — fine. */
        s_event_loop_inited = true;
        return true;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_event_loop_create_default: %s", esp_err_to_name(err));
        return false;
    }
    s_event_loop_inited = true;
    return true;
}

static bool ensure_netif(void)
{
    static bool netif_inited = false;
    if (netif_inited) return true;
    esp_err_t err = esp_netif_init();
    if (err == ESP_ERR_INVALID_STATE) {
        netif_inited = true;
        return true;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init: %s", esp_err_to_name(err));
        return false;
    }
    netif_inited = true;
    return true;
}

bool idl0_wifi_start(void)
{
    if (s_state == IDL0_WIFI_ON) {
        return true;
    }
    if (!ensure_netif() || !ensure_event_loop()) {
        return false;
    }
    if (s_ap_netif == NULL) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (s_ap_netif == NULL) {
            ESP_LOGE(TAG, "esp_netif_create_default_wifi_ap failed");
            return false;
        }
    }

    if (!s_wifi_inited) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        esp_err_t err = esp_wifi_init(&cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_wifi_init: %s", esp_err_to_name(err));
            return false;
        }
        s_wifi_inited = true;
    }

    /* Build the AP config. SSID comes from idl0_device_name() — "IDL0-XXXX". */
    const char *ssid = idl0_device_name();
    wifi_config_t wifi_cfg = {0};
    wifi_cfg.ap.ssid_len       = (uint8_t)strnlen(ssid, sizeof(wifi_cfg.ap.ssid));
    wifi_cfg.ap.channel        = IDL0_WIFI_CHANNEL;
    wifi_cfg.ap.max_connection = IDL0_WIFI_MAX_CONN;
    wifi_cfg.ap.authmode       = WIFI_AUTH_WPA2_PSK;
    memcpy(wifi_cfg.ap.ssid, ssid, wifi_cfg.ap.ssid_len);
    strncpy((char *)wifi_cfg.ap.password, IDL0_WIFI_PASSWORD,
            sizeof(wifi_cfg.ap.password) - 1);

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg);
    if (err == ESP_OK) err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AP start failed: %s", esp_err_to_name(err));
        return false;
    }
    /* AP-mode does not sleep; set explicitly for clarity. */
    esp_wifi_set_ps(WIFI_PS_NONE);

    /* HTTP server. Endpoint handlers register in later milestones. */
    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    hcfg.max_uri_handlers = 12;
    /* The default httpd task stack (4 KB) overflows: an 8 KB read
     * buffer in /download + ~256 B of locals + httpd/lwIP call-frame
     * depth needs ~12-14 KB. 16 KB leaves a comfortable margin. */
    hcfg.stack_size = 16384;
    /* OTA uploads ~1.5 MB; even at 200 KB/s that's ~8 s end-to-end,
     * and any single TCP stall under load can pause a chunk for
     * several seconds. The 5 s default trips during normal uploads. */
    hcfg.recv_wait_timeout = 30;
    hcfg.send_wait_timeout = 30;
    err = httpd_start(&s_httpd, &hcfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(err));
        esp_wifi_stop();
        return false;
    }
    esp_err_t reg_err = register_endpoints(s_httpd);
    if (reg_err != ESP_OK) {
        ESP_LOGE(TAG, "register_endpoints: %s", esp_err_to_name(reg_err));
        httpd_stop(s_httpd);
        s_httpd = NULL;
        esp_wifi_stop();
        return false;
    }

    /* Arm the §10.4 no-activity failsafe: both idle windows start at
     * AP-up so a phone that never joins still gets timed out. */
    s_last_http_us = esp_timer_get_time();
    s_last_sta_us  = s_last_http_us;
    if (xTaskCreate(failsafe_task, "wifi_failsafe", 4096, NULL, 3,
                    &s_failsafe_task) != pdPASS) {
        ESP_LOGW(TAG, "failsafe task create failed - no idle timeout");
        s_failsafe_task = NULL;
    }

    s_state = IDL0_WIFI_ON;
    idl0_mode_set_bits(IDL0_MODE_BIT_WIFI_UP);
    ESP_LOGI(TAG, "AP up: SSID=%s, IP=192.168.4.1, HTTP on :80", ssid);
    idl0_diag_log_event("wifi up");
    return true;
}

bool idl0_wifi_stop(void)
{
    ESP_LOGI(TAG, "wifi_stop enter; state=%s", idl0_wifi_state_str(s_state));
    if (s_state == IDL0_WIFI_OFF) {
        return true;
    }
    /* Kill the failsafe poller — unless the failsafe itself is the
     * caller (it cleared the handle and self-deletes after we return). */
    if (s_failsafe_task != NULL &&
        xTaskGetCurrentTaskHandle() != s_failsafe_task) {
        vTaskDelete(s_failsafe_task);
    }
    s_failsafe_task = NULL;
    if (s_httpd != NULL) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
    }
    esp_err_t err = esp_wifi_stop();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_stop: %s", esp_err_to_name(err));
    }
    idl0_mode_clear_bits(IDL0_MODE_BIT_WIFI_UP);
    s_state = IDL0_WIFI_OFF;
    /* §10.4: BLE comes back whenever WiFi mode ends, regardless of which
     * path ended it (BLE CMD_WIFI_OFF pre-handoff, POST /wifi_off, or
     * the no-activity failsafe). No-op if BLE was never suspended. */
    idl0_ble_resume();
    ESP_LOGI(TAG, "AP down");
    idl0_diag_log_event("wifi down");
    return true;
}
