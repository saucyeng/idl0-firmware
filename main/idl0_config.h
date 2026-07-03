/* idl0_config.json loader.
 *
 * Reads the JSON from the SD card root, validates it against §8
 * of the spec, and exposes the parsed struct (§4.2 — config is
 * re-read from SD each boot).
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "digital_task.h"
#include "hrm_task.h"
#include "sd_logger.h"

#define ANALOG_MAX_CHANNELS 8

/* One entry from `analog.channels[]` (§8). Spec-driven user-added ADC channels.
 * `channel_id` is assigned at session start by session.c. */
typedef struct {
    bool    enabled;
    int     adc_pin;          /* GPIO number; mapped to ADC channel by analog reader */
    char    label[20];        /* matches channel registry name field */
    char    units[8];         /* matches channel registry units field */
    float   scale;
    float   offset;
    uint8_t channel_id;       /* assigned at session start */
} analog_channel_t;

/* Absolute path of the JSON config file on the SD card. */
#define IDL0_CONFIG_PATH IDL0_SD_MOUNT_POINT "/idl0_config.json"

typedef struct {
    /* Bike profile */
    char bike_name[32];
    bool is_full_suspension;
    uint8_t imu_count;       /* 1..3 */

    /* IMU global */
    uint16_t imu_sample_rate_hz;
    bool low_power_mode;
    bool high_performance_mode;

    /* Per-IMU full-scale ranges (one entry per IMU index 0/1/2).
     * Valid accel values: 4, 8, 16, 32 (g).
     * Valid gyro values: 125, 250, 500, 1000, 2000 (dps).
     * Defaults: 32 g / 2000 dps for all three IMUs.
     * Config source: imu.imuN.accel_range_g / imu.imuN.gyro_range_dps;
     * top-level imu.accel_range_g seeds all three slots before per-IMU
     * values override (back-compat with configs that lack per-IMU blocks). */
    uint8_t  accel_range_g[3];
    uint16_t gyro_range_dps[3];

    /* Per-IMU enables and orientation are stored separately;
     * full struct grows as bring-up progresses. */
    bool imu_enabled[3];

    /* IMU channel mask (§5.4): bits 0-5 IMU0 accel XYZ / gyro XYZ,
     * 6-11 IMU1, 12-17 IMU2. Derived from the per-axis `channels`
     * object in idl0_config.json. */
    uint32_t imu_channel_mask;

    /* GPS */
    uint8_t gps_sample_rate_hz;  /* 1..10 */
    bool sbas_enabled;

    /* BLE Heart Rate Monitor (§7.5 / §8). hrm.enabled gates everything;
     * a missing `heart_rate_monitor` block in the JSON disables HRM. */
    hrm_config_t hrm;

    /* Analog ADC channels — user-added entries from `analog.channels[]` (§8).
     * `analog_sample_rate_hz` is shared across all entries (the ADC scheduler
     * round-robins between configured pins). Defaults to 100 Hz. */
    uint16_t analog_sample_rate_hz;
    size_t   analog_channel_count;
    analog_channel_t analog_channels[ANALOG_MAX_CHANNELS];

    /* Digital input channels — user-added entries from `digital.channels[]`
     * (§8). Spec 1 ships `kind=marker` only; channel_id assigned at session
     * start. */
    size_t digital_channel_count;
    digital_channel_cfg_t digital_channels[DIGITAL_MAX_CHANNELS];

    /* Computed */
    uint32_t config_crc32;
} idl0_config_t;

/* Load /idl0_config.json from SD. Returns true on success. On failure,
 * out_config is populated with safe defaults. */
bool idl0_config_load(idl0_config_t *out_config);

/* Persist a config struct to /idl0_config.json on SD. */
bool idl0_config_save(const idl0_config_t *config);

/* Result of idl0_config_write_json(). */
typedef enum {
    IDL0_CONFIG_WRITE_OK = 0,    /* validated and persisted */
    IDL0_CONFIG_WRITE_BAD_JSON,  /* buf did not parse as JSON */
    IDL0_CONFIG_WRITE_IO_ERROR,  /* SD open / write / rename failed */
} idl0_config_write_result_t;

/* Validate `buf` (`len` bytes of raw JSON text) and, on success, atomically
 * persist it verbatim to IDL0_CONFIG_PATH on the SD card. The bytes are
 * written unmodified so the on-disk image drives config_crc32 (§5.1).
 *
 * Shared by the WiFi `POST /config` handler and the BLE config-push path
 * (FF05 + CMD_CONFIG_COMMIT, §7.2). Does NOT reboot — config is read at boot
 * only (§4.2), so the caller decides when to esp_restart() to apply.
 *
 * Write goes to a temp path and is renamed over the live file so a short or
 * interrupted write cannot corrupt the active config. */
idl0_config_write_result_t idl0_config_write_json(const char *buf, size_t len);
