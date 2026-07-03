/* idl0_config.json loader. Reads the JSON from the SD card root,
 * derives the §5.4 IMU channel mask, and computes the §5.1 config CRC32.
 * On any failure, out_config is filled with safe defaults so a session
 * can still start. See docs/IDL0_SPEC.md §8.
 */

#include "idl0_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"

#include "idl0_format.h"
#include "sd_logger.h"

static const char *TAG = "config";

#define CONFIG_MAX_BYTES 8192

/* All-IMU, all-axis mask (bits 0-17). */
#define DEFAULT_IMU_MASK 0x0003FFFFu

static void fill_defaults(idl0_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    snprintf(cfg->bike_name, sizeof(cfg->bike_name), "%s", "Unconfigured");
    cfg->is_full_suspension   = true;
    cfg->imu_count            = 3;
    cfg->imu_sample_rate_hz   = 104;
    cfg->accel_range_g[0]     = 32;
    cfg->accel_range_g[1]     = 32;
    cfg->accel_range_g[2]     = 32;
    cfg->gyro_range_dps[0]    = 2000;
    cfg->gyro_range_dps[1]    = 2000;
    cfg->gyro_range_dps[2]    = 2000;
    cfg->high_performance_mode = true;
    cfg->imu_enabled[0] = cfg->imu_enabled[1] = cfg->imu_enabled[2] = true;
    cfg->gps_sample_rate_hz   = 5;
    cfg->imu_channel_mask     = DEFAULT_IMU_MASK;
    cfg->config_crc32         = 0;
    /* HRM disabled by default; channel IDs match spec §5.2 so status.c
     * and session.c can reference them even with no strap configured. */
    cfg->hrm.hr_channel_id    = 22;
    cfg->hrm.rr_channel_id    = 23;
    /* Analog + digital channel arrays default empty — users add entries
     * via the app's `+ Add channel…` flow as hardware lands. */
    cfg->analog_sample_rate_hz = 100;
    cfg->analog_channel_count  = 0;
    cfg->digital_channel_count = 0;
}

/* Parse `analog.channels[]` from §8 — user-added ADC channels. Empty / missing
 * array is valid (no analog channels). */
static void load_analog(const cJSON *root, idl0_config_t *cfg)
{
    const cJSON *analog = cJSON_GetObjectItemCaseSensitive(root, "analog");
    if (!cJSON_IsObject(analog)) return;

    const cJSON *rate = cJSON_GetObjectItemCaseSensitive(analog, "sample_rate_hz");
    if (cJSON_IsNumber(rate)) {
        cfg->analog_sample_rate_hz = (uint16_t)rate->valueint;
    }

    const cJSON *channels = cJSON_GetObjectItemCaseSensitive(analog, "channels");
    if (!cJSON_IsArray(channels)) return;

    cfg->analog_channel_count = 0;
    const cJSON *entry = NULL;
    cJSON_ArrayForEach(entry, channels) {
        if (!cJSON_IsObject(entry)) continue;
        if (cfg->analog_channel_count >= ANALOG_MAX_CHANNELS) {
            ESP_LOGW(TAG, "analog.channels[] truncated at %d entries", ANALOG_MAX_CHANNELS);
            break;
        }
        analog_channel_t *a = &cfg->analog_channels[cfg->analog_channel_count];
        memset(a, 0, sizeof(*a));

        const cJSON *en   = cJSON_GetObjectItemCaseSensitive(entry, "enabled");
        const cJSON *pin  = cJSON_GetObjectItemCaseSensitive(entry, "adc_pin");
        const cJSON *lbl  = cJSON_GetObjectItemCaseSensitive(entry, "label");
        const cJSON *unit = cJSON_GetObjectItemCaseSensitive(entry, "units");
        const cJSON *scl  = cJSON_GetObjectItemCaseSensitive(entry, "scale");
        const cJSON *off  = cJSON_GetObjectItemCaseSensitive(entry, "offset");

        a->enabled = cJSON_IsTrue(en);
        a->adc_pin = cJSON_IsNumber(pin) ? pin->valueint : 0;
        if (cJSON_IsString(lbl) && lbl->valuestring != NULL) {
            strncpy(a->label, lbl->valuestring, sizeof(a->label) - 1);
        }
        if (cJSON_IsString(unit) && unit->valuestring != NULL) {
            strncpy(a->units, unit->valuestring, sizeof(a->units) - 1);
        }
        a->scale  = cJSON_IsNumber(scl) ? (float)scl->valuedouble : 1.0f;
        a->offset = cJSON_IsNumber(off) ? (float)off->valuedouble : 0.0f;

        cfg->analog_channel_count++;
    }
}

/* Parse `digital.channels[]` from §8 — user-added GPIO inputs. Spec 1 ships
 * `kind=marker` only; other kinds are silently skipped here (forward-compat). */
static void load_digital(const cJSON *root, idl0_config_t *cfg)
{
    const cJSON *digital = cJSON_GetObjectItemCaseSensitive(root, "digital");
    if (!cJSON_IsObject(digital)) return;

    const cJSON *channels = cJSON_GetObjectItemCaseSensitive(digital, "channels");
    if (!cJSON_IsArray(channels)) return;

    cfg->digital_channel_count = 0;
    const cJSON *entry = NULL;
    cJSON_ArrayForEach(entry, channels) {
        if (!cJSON_IsObject(entry)) continue;
        if (cfg->digital_channel_count >= DIGITAL_MAX_CHANNELS) {
            ESP_LOGW(TAG, "digital.channels[] truncated at %d entries", DIGITAL_MAX_CHANNELS);
            break;
        }

        const cJSON *kind = cJSON_GetObjectItemCaseSensitive(entry, "kind");
        const char *kind_str = (cJSON_IsString(kind) && kind->valuestring) ?
                               kind->valuestring : "marker";
        /* Spec 1 implements marker only. */
        if (strcmp(kind_str, "marker") != 0) {
            ESP_LOGW(TAG, "digital.channels[] kind=%s not yet supported, skipped",
                     kind_str);
            continue;
        }

        digital_channel_cfg_t *d = &cfg->digital_channels[cfg->digital_channel_count];
        memset(d, 0, sizeof(*d));

        const cJSON *en    = cJSON_GetObjectItemCaseSensitive(entry, "enabled");
        const cJSON *pin   = cJSON_GetObjectItemCaseSensitive(entry, "gpio_pin");
        const cJSON *act   = cJSON_GetObjectItemCaseSensitive(entry, "active_low");
        const cJSON *db    = cJSON_GetObjectItemCaseSensitive(entry, "debounce_ms");
        const cJSON *lbl   = cJSON_GetObjectItemCaseSensitive(entry, "label");

        d->enabled    = cJSON_IsTrue(en);
        d->gpio_pin   = cJSON_IsNumber(pin) ? pin->valueint : -1;
        d->active_low = cJSON_IsBool(act) ? cJSON_IsTrue(act) : true;
        d->debounce_ms = cJSON_IsNumber(db) ? (uint16_t)db->valueint : 20;
        if (cJSON_IsString(lbl) && lbl->valuestring != NULL) {
            strncpy(d->label, lbl->valuestring, sizeof(d->label) - 1);
        } else {
            snprintf(d->label, sizeof(d->label), "Marker");
        }

        cfg->digital_channel_count++;
    }
}

/* Parses "AA:BB:CC:DD:EE:FF" into a 6-byte address in NimBLE wire order
 * (LSB first). Returns true on success. Lenient on case; rejects strings
 * that don't have exactly 17 characters or a non-hex byte. */
static bool parse_hex_addr_le(const char *s, uint8_t *out)
{
    if (s == NULL) return false;
    size_t len = strlen(s);
    if (len < 17) return false;
    uint8_t big[6];
    for (int i = 0; i < 6; i++) {
        unsigned int b = 0;
        if (sscanf(s + i * 3, "%2x", &b) != 1) return false;
        big[i] = (uint8_t)b;
    }
    /* NimBLE addresses are LSB-first; reverse so memcmp against
     * event->disc.addr.val matches directly. */
    for (int i = 0; i < 6; i++) out[i] = big[5 - i];
    return true;
}

static void load_hrm(const cJSON *root, idl0_config_t *cfg)
{
    /* Default: disabled, channel IDs pinned to spec (§5.2). */
    memset(&cfg->hrm, 0, sizeof(cfg->hrm));
    cfg->hrm.hr_channel_id = 22;
    cfg->hrm.rr_channel_id = 23;

    const cJSON *hrm = cJSON_GetObjectItemCaseSensitive(root, "heart_rate_monitor");
    if (!cJSON_IsObject(hrm)) return;

    const cJSON *en = cJSON_GetObjectItemCaseSensitive(hrm, "enabled");
    cfg->hrm.enabled = cJSON_IsTrue(en);

    const cJSON *addr = cJSON_GetObjectItemCaseSensitive(hrm, "device_address");
    if (cJSON_IsString(addr) && addr->valuestring != NULL) {
        if (!parse_hex_addr_le(addr->valuestring, cfg->hrm.address)) {
            /* Address malformed — keep enabled=false so we don't scan
             * for an unknown peer. */
            cfg->hrm.enabled = false;
        }
    } else if (cfg->hrm.enabled) {
        /* Enabled but no address — can't connect to anything. */
        cfg->hrm.enabled = false;
    }

    const cJSON *name = cJSON_GetObjectItemCaseSensitive(hrm, "device_name");
    if (cJSON_IsString(name) && name->valuestring != NULL) {
        strncpy(cfg->hrm.name, name->valuestring, sizeof(cfg->hrm.name) - 1);
    }
}

/* Sets the 6 mask bits for one IMU from a channels.imuN JSON object. */
static uint32_t parse_imu_axes(const cJSON *imu_obj, int imu_index)
{
    static const char *axis_keys[6] = {
        "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"
    };
    uint32_t bits = 0;
    for (int axis = 0; axis < 6; axis++) {
        const cJSON *v = cJSON_GetObjectItemCaseSensitive(imu_obj, axis_keys[axis]);
        if (cJSON_IsTrue(v)) {
            bits |= (1u << axis);
        }
    }
    return bits << (imu_index * 6);
}

bool idl0_config_load(idl0_config_t *out_config)
{
    fill_defaults(out_config);

    FILE *f = fopen(IDL0_CONFIG_PATH, "rb");
    if (f == NULL) {
        ESP_LOGW(TAG, "%s not found — using defaults", IDL0_CONFIG_PATH);
        return false;
    }

    uint8_t *raw = malloc(CONFIG_MAX_BYTES);
    if (raw == NULL) {
        ESP_LOGE(TAG, "out of memory reading config");
        fclose(f);
        return false;
    }
    size_t len = fread(raw, 1, CONFIG_MAX_BYTES, f);
    fclose(f);

    /* CRC32 over the exact on-disk bytes — before any parsing (§5.1). */
    out_config->config_crc32 = idl0_format_crc32(raw, len);

    cJSON *root = cJSON_ParseWithLength((const char *)raw, len);
    free(raw);
    if (root == NULL) {
        ESP_LOGW(TAG, "config JSON malformed — using defaults");
        return false;
    }

    const cJSON *bike = cJSON_GetObjectItemCaseSensitive(root, "bike_profile");
    if (cJSON_IsObject(bike)) {
        const cJSON *name = cJSON_GetObjectItemCaseSensitive(bike, "name");
        if (cJSON_IsString(name) && name->valuestring != NULL) {
            snprintf(out_config->bike_name, sizeof(out_config->bike_name),
                     "%s", name->valuestring);
        }
        const cJSON *cnt = cJSON_GetObjectItemCaseSensitive(bike, "imu_count");
        if (cJSON_IsNumber(cnt)) {
            int c = cnt->valueint;
            out_config->imu_count = (uint8_t)(c < 1 ? 1 : (c > 3 ? 3 : c));
        }
    }

    const cJSON *imu = cJSON_GetObjectItemCaseSensitive(root, "imu");
    if (cJSON_IsObject(imu)) {
        const cJSON *rate = cJSON_GetObjectItemCaseSensitive(imu, "sample_rate_hz");
        if (cJSON_IsNumber(rate)) {
            out_config->imu_sample_rate_hz = (uint16_t)rate->valueint;
        }

        /* Top-level accel_range_g / gyro_range_dps seed all three slots for
         * back-compat with config files that lack per-IMU sub-blocks. */
        const cJSON *arange_top = cJSON_GetObjectItemCaseSensitive(imu, "accel_range_g");
        if (cJSON_IsNumber(arange_top)) {
            uint8_t v = (uint8_t)arange_top->valueint;
            out_config->accel_range_g[0] = v;
            out_config->accel_range_g[1] = v;
            out_config->accel_range_g[2] = v;
        }
        const cJSON *grange_top = cJSON_GetObjectItemCaseSensitive(imu, "gyro_range_dps");
        if (cJSON_IsNumber(grange_top)) {
            uint16_t v = (uint16_t)grange_top->valueint;
            out_config->gyro_range_dps[0] = v;
            out_config->gyro_range_dps[1] = v;
            out_config->gyro_range_dps[2] = v;
        }

        const cJSON *hp = cJSON_GetObjectItemCaseSensitive(imu, "high_performance_mode");
        if (cJSON_IsBool(hp)) {
            out_config->high_performance_mode = cJSON_IsTrue(hp);
        }
        const cJSON *lp = cJSON_GetObjectItemCaseSensitive(imu, "low_power_mode");
        if (cJSON_IsBool(lp)) {
            out_config->low_power_mode = cJSON_IsTrue(lp);
        }

        /* Per-IMU sub-blocks: imu.imu0, imu.imu1, imu.imu2.
         * Each carries enabled, accel_range_g, gyro_range_dps, and channels.
         * Per-IMU values override the top-level seed parsed above. */
        static const char *imu_keys[3] = { "imu0", "imu1", "imu2" };
        uint32_t mask = 0;
        bool have_per_imu_channels = false;
        for (int i = 0; i < 3; i++) {
            const cJSON *imu_blk =
                cJSON_GetObjectItemCaseSensitive(imu, imu_keys[i]);
            if (!cJSON_IsObject(imu_blk)) continue;

            const cJSON *en = cJSON_GetObjectItemCaseSensitive(imu_blk, "enabled");
            if (cJSON_IsBool(en)) {
                out_config->imu_enabled[i] = cJSON_IsTrue(en);
            }
            const cJSON *ar = cJSON_GetObjectItemCaseSensitive(imu_blk, "accel_range_g");
            if (cJSON_IsNumber(ar)) {
                out_config->accel_range_g[i] = (uint8_t)ar->valueint;
            }
            const cJSON *gr = cJSON_GetObjectItemCaseSensitive(imu_blk, "gyro_range_dps");
            if (cJSON_IsNumber(gr)) {
                out_config->gyro_range_dps[i] = (uint16_t)gr->valueint;
            }
            const cJSON *channels_blk =
                cJSON_GetObjectItemCaseSensitive(imu_blk, "channels");
            if (cJSON_IsObject(channels_blk)) {
                mask |= parse_imu_axes(channels_blk, i);
                have_per_imu_channels = true;
            }
        }
        if (have_per_imu_channels) {
            out_config->imu_channel_mask = mask;
        }

        /* Back-compat: old schema had imu.channels.imuN (flat channels block).
         * Only falls back if no per-IMU sub-blocks provided a channels object. */
        if (!have_per_imu_channels) {
            const cJSON *channels = cJSON_GetObjectItemCaseSensitive(imu, "channels");
            if (cJSON_IsObject(channels)) {
                uint32_t legacy_mask = 0;
                for (int i = 0; i < 3; i++) {
                    const cJSON *obj =
                        cJSON_GetObjectItemCaseSensitive(channels, imu_keys[i]);
                    if (cJSON_IsObject(obj)) {
                        legacy_mask |= parse_imu_axes(obj, i);
                    }
                }
                out_config->imu_channel_mask = legacy_mask;
            }
        }
    }

    load_hrm(root, out_config);
    load_analog(root, out_config);
    load_digital(root, out_config);

    const cJSON *gps = cJSON_GetObjectItemCaseSensitive(root, "gps");
    if (cJSON_IsObject(gps)) {
        const cJSON *rate = cJSON_GetObjectItemCaseSensitive(gps, "sample_rate_hz");
        if (cJSON_IsNumber(rate)) {
            int r = rate->valueint;
            out_config->gps_sample_rate_hz = (uint8_t)(r < 1 ? 1 : (r > 10 ? 10 : r));
        }
        const cJSON *sbas = cJSON_GetObjectItemCaseSensitive(gps, "sbas_enabled");
        if (cJSON_IsBool(sbas)) {
            out_config->sbas_enabled = cJSON_IsTrue(sbas);
        }
    }

    cJSON_Delete(root);
    ESP_LOGI(TAG, "config loaded: bike '%s', %u IMUs, mask 0x%05lX, crc 0x%08lX",
             out_config->bike_name, out_config->imu_count,
             (unsigned long)out_config->imu_channel_mask,
             (unsigned long)out_config->config_crc32);
    return true;
}

bool idl0_config_save(const idl0_config_t *config)
{
    /* Config push is implemented with the WiFi server in P8. The firmware
     * does not write idl0_config.json during P5-P7. */
    (void)config;
    ESP_LOGW(TAG, "idl0_config_save not implemented until P8");
    return false;
}

idl0_config_write_result_t idl0_config_write_json(const char *buf, size_t len)
{
    if (buf == NULL || len == 0 || len > CONFIG_MAX_BYTES) {
        return IDL0_CONFIG_WRITE_BAD_JSON;
    }

    /* Validate it parses as JSON. We don't enforce the §8 schema here — the
     * app's config editor is the gatekeeper for shape; firmware just refuses
     * outright garbage so a corrupt push can't replace a good config. */
    cJSON *root = cJSON_ParseWithLength(buf, len);
    if (root == NULL) {
        return IDL0_CONFIG_WRITE_BAD_JSON;
    }
    cJSON_Delete(root);

    /* Persist verbatim so the on-disk bytes drive config_crc32 (§5.1). Write
     * to a temp path and rename on success so a short or interrupted write
     * can't corrupt the live config. */
    static const char *tmp_path = IDL0_CONFIG_PATH ".tmp";
    FILE *f = fopen(tmp_path, "wb");
    if (f == NULL) {
        return IDL0_CONFIG_WRITE_IO_ERROR;
    }
    size_t written = fwrite(buf, 1, len, f);
    int close_err = fclose(f);
    if (written != len || close_err != 0) {
        remove(tmp_path);
        return IDL0_CONFIG_WRITE_IO_ERROR;
    }

    /* FAT32 (ESP-IDF VFS) rename() does NOT overwrite an existing target — it
     * returns -1. Unlink the live config first so second-and-later pushes
     * don't fail. The unlink may legitimately fail when no live config exists
     * yet (first push of a freshly formatted SD); that's fine — only a rename
     * failure is fatal. */
    remove(IDL0_CONFIG_PATH);
    if (rename(tmp_path, IDL0_CONFIG_PATH) != 0) {
        remove(tmp_path);
        return IDL0_CONFIG_WRITE_IO_ERROR;
    }
    return IDL0_CONFIG_WRITE_OK;
}
