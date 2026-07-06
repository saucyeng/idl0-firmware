/* LSM6DSO32 SPI driver — up to IDL0_IMU_COUNT (3) IMUs.
 *
 * Each IMU has its own SPI device handle (with its own CS pin from
 * pins.h), its own ST `stmdev_ctx_t`, and its own pair-builder for
 * (gyro, accel) word ordering coming out of the FIFO. The platform
 * read/write glue is shared and dispatches via the ctx's `handle`
 * which holds a pointer to the per-IMU spi_device_handle_t.
 *
 * Symbol notes (vendored ST driver lsm6dso32x_STdC v2.3.0):
 *   - ODR enums spell the half-step as "417Hz" / "1667Hz" (not 416 / 1666);
 *     the public API still accepts 416 / 1666 from callers and maps them
 *     to the nearest ST enum.
 *   - FIFO continuous-overwrite is LSM6DSO32X_STREAM_MODE.
 *   - Per-word tag is read via lsm6dso32x_fifo_sensor_tag_get; data via
 *     lsm6dso32x_fifo_out_raw_get. Tag constants: LSM6DSO32X_GYRO_NC_TAG,
 *     LSM6DSO32X_XL_NC_TAG.
 *   - The "unread words" count is not a field on lsm6dso32x_fifo_status2_t
 *     (which only exposes the low 2 bits via diff_fifo + the latched
 *     overrun bit). Use lsm6dso32x_fifo_data_level_get for the full
 *     16-bit count, and lsm6dso32x_fifo_status_get for the overrun flag
 *     (`.fifo_ovr_ia`).
 */

#include "imu_driver.h"

#include <string.h>

#include "driver/spi_master.h"
#include "esp_attr.h"          /* WORD_ALIGNED_ATTR for the DMA burst buffers */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lsm6dso32x_reg.h"
#include "pins.h"

static const char *TAG = "imu";

/* SPI clock for the IMU. Datasheet max is 10 MHz. The shared SPI bus
 * now carries only the onboard IMU0 plus the SD card (both on direct
 * PCB traces); the external IMU1/IMU2 moved to I²C via imu_driver_i2c
 * because their level-shifted SPI didn't work — so the bus loading is
 * back to what the board was originally designed for. */
#define IMU_SPI_HZ        (10 * 1000 * 1000)
#define IMU_QUEUE_DEPTH   1
#define IMU_WHO_AM_I      0x6C

/* Max FIFO words read in one SPI burst. 512 words = 256 (gyro,accel) pairs =
 * the full LSM6DSO32X FIFO (3 KB, 7-byte words). 1 + 512*7 = 3585 bytes fits
 * under the bus's 4096-byte max_transfer_sz (set in sd_logger spi_bus_config).
 * The burst replaces the old words*2 per-word transactions — the per-word path
 * spent ~⅔ of every poll cycle in spi_device_transmit overhead.
 */
#define IMU_SPI_BURST_MAX_WORDS 512

/* CS pin per IMU index. Order matches pins.h §3 (IMU0 onboard sprung
 * mass; IMU1 front unsprung; IMU2 rear unsprung). */
static const int s_cs_pin[IDL0_IMU_COUNT] = {
    IDL0_PIN_CS_IMU0,
    IDL0_PIN_CS_IMU1,
    IDL0_PIN_CS_IMU2,
};

/* Per-IMU runtime state. NULL spi handle == slot not yet attempted. */
static spi_device_handle_t s_spi[IDL0_IMU_COUNT];
static stmdev_ctx_t        s_ctx[IDL0_IMU_COUNT];
static idl0_imu_state_t    s_per_state[IDL0_IMU_COUNT];
static bool                s_init_attempted[IDL0_IMU_COUNT];
static bool                s_overrun_flag[IDL0_IMU_COUNT];
static int16_t             s_pending_gyro[IDL0_IMU_COUNT][3];
static bool                s_have_gyro[IDL0_IMU_COUNT];
/* Pair-builder diagnostics, surfaced in the session-end diag line (imu_task.c),
 * read+cleared by idl0_imu_take_pair_diag(). s_gyro_fresh: is the pending gyro
 * newer than the last emitted pair (drives the reuse count). s_reuse: accels
 * that reused a stale gyro on a FIFO phase slip — the samples the lossless
 * pairing recovers (were dropped before). s_unknown: unrecognised FIFO tags —
 * a corruption / unexpected-batch-word signal, 0 on a clean bus. */
static bool                s_gyro_fresh[IDL0_IMU_COUNT];
static uint32_t            s_reuse[IDL0_IMU_COUNT];
static uint32_t            s_unknown[IDL0_IMU_COUNT];
/* Count of unrecognised FIFO tag words seen since the last diag flush — a
 * corruption / unexpected-batch-word signal (0 on a clean bus). Read+cleared
 * by idl0_imu_take_unknown(). */
static uint32_t            s_unknown[IDL0_IMU_COUNT];

/* --- ST driver platform glue -------------------------------------
 *
 * The ST register-access driver expects the `handle` field to be an
 * opaque pointer it passes back unchanged. We store a pointer to the
 * per-IMU spi_device_handle_t there so a single set of glue functions
 * serves all three IMUs. */

static int32_t platform_write(void *handle, uint8_t reg,
                              const uint8_t *bufp, uint16_t len)
{
    spi_device_handle_t *spi = (spi_device_handle_t *)handle;
    uint8_t tx[1 + 16];
    if (len > sizeof(tx) - 1) return -1;
    tx[0] = reg & 0x7F;  /* MSB=0 → write */
    memcpy(tx + 1, bufp, len);
    spi_transaction_t t = {
        .length = (1 + len) * 8,
        .tx_buffer = tx,
    };
    esp_err_t err = spi_device_transmit(*spi, &t);
    return (err == ESP_OK) ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg,
                             uint8_t *bufp, uint16_t len)
{
    spi_device_handle_t *spi = (spi_device_handle_t *)handle;
    uint8_t tx[1 + 32] = {0};
    uint8_t rx[1 + 32] = {0};
    if (len > sizeof(tx) - 1) return -1;
    tx[0] = reg | 0x80;  /* MSB=1 → read */
    spi_transaction_t t = {
        .length = (1 + len) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_transmit(*spi, &t);
    if (err != ESP_OK) return -1;
    memcpy(bufp, rx + 1, len);
    return 0;
}

static void platform_mdelay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* --- Helpers: spec config values → ST driver enums ---------------
 * ST spells the half-steps as 12Hz5 / 417Hz / 1667Hz; the public API
 * accepts the rounded values (12 or 13, 416, 1666) and maps them
 * to the nearest ST enum. Fallback is 104 Hz / max range / high-perf
 * so a bad config still boots. */

static lsm6dso32x_odr_xl_t xl_odr_enum(uint16_t hz)
{
    switch (hz) {
        case 12: case 13: return LSM6DSO32X_XL_ODR_12Hz5;
        case 26:          return LSM6DSO32X_XL_ODR_26Hz;
        case 52:          return LSM6DSO32X_XL_ODR_52Hz;
        case 104:         return LSM6DSO32X_XL_ODR_104Hz;
        case 208:         return LSM6DSO32X_XL_ODR_208Hz;
        case 416: case 417: return LSM6DSO32X_XL_ODR_417Hz;
        case 833:         return LSM6DSO32X_XL_ODR_833Hz;
        case 1666: case 1667: return LSM6DSO32X_XL_ODR_1667Hz;
        default:          return LSM6DSO32X_XL_ODR_104Hz;
    }
}

static lsm6dso32x_odr_g_t gy_odr_enum(uint16_t hz)
{
    switch (hz) {
        case 12: case 13: return LSM6DSO32X_GY_ODR_12Hz5;
        case 26:          return LSM6DSO32X_GY_ODR_26Hz;
        case 52:          return LSM6DSO32X_GY_ODR_52Hz;
        case 104:         return LSM6DSO32X_GY_ODR_104Hz;
        case 208:         return LSM6DSO32X_GY_ODR_208Hz;
        case 416: case 417: return LSM6DSO32X_GY_ODR_417Hz;
        case 833:         return LSM6DSO32X_GY_ODR_833Hz;
        case 1666: case 1667: return LSM6DSO32X_GY_ODR_1667Hz;
        default:          return LSM6DSO32X_GY_ODR_104Hz;
    }
}

static lsm6dso32x_bdr_xl_t xl_bdr_enum(uint16_t hz)
{
    switch (hz) {
        case 12: case 13: return LSM6DSO32X_XL_BATCHED_AT_12Hz5;
        case 26:          return LSM6DSO32X_XL_BATCHED_AT_26Hz;
        case 52:          return LSM6DSO32X_XL_BATCHED_AT_52Hz;
        case 104:         return LSM6DSO32X_XL_BATCHED_AT_104Hz;
        case 208:         return LSM6DSO32X_XL_BATCHED_AT_208Hz;
        case 416: case 417: return LSM6DSO32X_XL_BATCHED_AT_417Hz;
        case 833:         return LSM6DSO32X_XL_BATCHED_AT_833Hz;
        case 1666: case 1667: return LSM6DSO32X_XL_BATCHED_AT_1667Hz;
        default:          return LSM6DSO32X_XL_BATCHED_AT_104Hz;
    }
}

static lsm6dso32x_bdr_gy_t gy_bdr_enum(uint16_t hz)
{
    switch (hz) {
        case 12: case 13: return LSM6DSO32X_GY_BATCHED_AT_12Hz5;
        case 26:          return LSM6DSO32X_GY_BATCHED_AT_26Hz;
        case 52:          return LSM6DSO32X_GY_BATCHED_AT_52Hz;
        case 104:         return LSM6DSO32X_GY_BATCHED_AT_104Hz;
        case 208:         return LSM6DSO32X_GY_BATCHED_AT_208Hz;
        case 416: case 417: return LSM6DSO32X_GY_BATCHED_AT_417Hz;
        case 833:         return LSM6DSO32X_GY_BATCHED_AT_833Hz;
        case 1666: case 1667: return LSM6DSO32X_GY_BATCHED_AT_1667Hz;
        default:          return LSM6DSO32X_GY_BATCHED_AT_104Hz;
    }
}

static lsm6dso32x_fs_xl_t xl_fs_enum(uint8_t g)
{
    switch (g) {
        case 4:  return LSM6DSO32X_4g;
        case 8:  return LSM6DSO32X_8g;
        case 16: return LSM6DSO32X_16g;
        case 32: return LSM6DSO32X_32g;
        default: return LSM6DSO32X_32g;
    }
}

static lsm6dso32x_fs_g_t gy_fs_enum(uint16_t dps)
{
    switch (dps) {
        case 125:  return LSM6DSO32X_125dps;
        case 250:  return LSM6DSO32X_250dps;
        case 500:  return LSM6DSO32X_500dps;
        case 1000: return LSM6DSO32X_1000dps;
        case 2000: return LSM6DSO32X_2000dps;
        default:   return LSM6DSO32X_2000dps;
    }
}

/* --- Diagnostic --------------------------------------------------
 * Dumps a 32-byte burst from register 0x00 plus a write-readback test
 * to a scratch user-offset register. Runs once per init attempt just
 * before the WHO_AM_I check so IMU0 (passing) and IMU1/IMU2 (failing)
 * produce side-by-side dumps for comparison.
 *
 * Interpretation:
 *   - burst byte[0x0F] = 0x6C → chip is alive and SPI addressing works
 *   - all-zero burst, write-readback returns 0x00 not 0xA5 → chip is
 *     receiving commands but its MOSI input is dead (sees 0xFF, reads
 *     register 0x7F into a sea of zero-default offsets)
 *   - burst differs between IMU0 and IMU1 in a structured way → bus
 *     framing problem (clock phase / first-bit misalignment)
 * TODO(idl0): remove once IMU1/IMU2 silence is diagnosed.
 */
static void diag_dump_one(uint8_t imu_index)
{
    if (s_spi[imu_index] == NULL) return;
    stmdev_ctx_t *ctx = &s_ctx[imu_index];

    uint8_t buf[32] = {0};
    int32_t r_burst = lsm6dso32x_read_reg(ctx, 0x00, buf, sizeof(buf));

    uint8_t test_val = 0xA5;
    uint8_t readback = 0xEE;
    int32_t w_rc = lsm6dso32x_write_reg(ctx, 0x73, &test_val, 1);
    int32_t r_rc = lsm6dso32x_read_reg(ctx,  0x73, &readback, 1);
    uint8_t zero = 0;
    lsm6dso32x_write_reg(ctx, 0x73, &zero, 1);   /* restore */

    ESP_LOGW(TAG, "IMU%u DIAG burst 0x00+32 (rc=%ld): "
                  "%02X %02X %02X %02X %02X %02X %02X %02X "
                  "%02X %02X %02X %02X %02X %02X %02X %02X "
                  "%02X %02X %02X %02X %02X %02X %02X %02X "
                  "%02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned)imu_index, (long)r_burst,
             buf[0],  buf[1],  buf[2],  buf[3],  buf[4],  buf[5],  buf[6],  buf[7],
             buf[8],  buf[9],  buf[10], buf[11], buf[12], buf[13], buf[14], buf[15],
             buf[16], buf[17], buf[18], buf[19], buf[20], buf[21], buf[22], buf[23],
             buf[24], buf[25], buf[26], buf[27], buf[28], buf[29], buf[30], buf[31]);
    ESP_LOGW(TAG, "IMU%u DIAG WHO_AM_I byte (burst[0x0F])=0x%02X (expect 0x6C); "
                  "X_OFS_USR write 0xA5 → readback 0x%02X (w_rc=%ld r_rc=%ld)",
             (unsigned)imu_index, buf[0x0F],
             readback, (long)w_rc, (long)r_rc);
}

/* --- Public API -------------------------------------------------- */

const char *idl0_imu_state_str(idl0_imu_state_t state)
{
    switch (state) {
        case IDL0_IMU_OK:      return "OK";
        case IDL0_IMU_PARTIAL: return "PARTIAL";
        case IDL0_IMU_ERROR:   return "ERROR";
        case IDL0_IMU_ABSENT:
        default:               return "ABSENT";
    }
}

idl0_imu_state_t idl0_imu_state(void)
{
    int attempted = 0, ok_count = 0, error = 0;
    for (int i = 0; i < IDL0_IMU_COUNT; i++) {
        if (!s_init_attempted[i]) continue;
        attempted++;
        if (s_per_state[i] == IDL0_IMU_OK)         ok_count++;
        else if (s_per_state[i] == IDL0_IMU_ERROR) error++;
    }
    if (attempted == 0) return IDL0_IMU_ABSENT;
    if (error > 0)      return IDL0_IMU_ERROR;
    if (ok_count == 0)  return IDL0_IMU_ABSENT;
    if (ok_count < attempted) return IDL0_IMU_PARTIAL;
    return IDL0_IMU_OK;
}

bool idl0_imu_is_active(uint8_t imu_index)
{
    if (imu_index >= IDL0_IMU_COUNT) return false;
    return s_per_state[imu_index] == IDL0_IMU_OK;
}

bool idl0_imu_init_one(spi_host_device_t spi_host,
                       uint8_t imu_index,
                       uint16_t sample_rate_hz,
                       uint8_t accel_range_g,
                       uint16_t gyro_range_dps,
                       bool high_performance)
{
    if (imu_index >= IDL0_IMU_COUNT) {
        ESP_LOGE(TAG, "init_one: index %u out of range", (unsigned)imu_index);
        return false;
    }

    s_init_attempted[imu_index] = true;
    s_per_state[imu_index]      = IDL0_IMU_ABSENT;

    spi_device_interface_config_t dev = {
        .clock_speed_hz = IMU_SPI_HZ,
        .mode           = 0,                 /* CPOL=0, CPHA=0 */
        .spics_io_num   = s_cs_pin[imu_index],
        .queue_size     = IMU_QUEUE_DEPTH,
    };
    esp_err_t err = spi_bus_add_device(spi_host, &dev, &s_spi[imu_index]);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IMU%u spi_bus_add_device: %s",
                 (unsigned)imu_index, esp_err_to_name(err));
        return false;
    }

    stmdev_ctx_t *ctx = &s_ctx[imu_index];
    ctx->write_reg = platform_write;
    ctx->read_reg  = platform_read;
    ctx->mdelay    = platform_mdelay;
    ctx->handle    = &s_spi[imu_index];

    diag_dump_one(imu_index);

    uint8_t who = 0;
    if (lsm6dso32x_device_id_get(ctx, &who) != 0 || who != IMU_WHO_AM_I) {
        ESP_LOGW(TAG, "IMU%u WHO_AM_I mismatch: got 0x%02X, expected 0x%02X",
                 (unsigned)imu_index, who, IMU_WHO_AM_I);
        spi_bus_remove_device(s_spi[imu_index]);
        s_spi[imu_index] = NULL;
        return false;
    }

    /* Software reset, then wait for it to clear. */
    lsm6dso32x_reset_set(ctx, PROPERTY_ENABLE);
    uint8_t rst = 1;
    for (int i = 0; i < 10 && rst != 0; i++) {
        lsm6dso32x_reset_get(ctx, &rst);
        platform_mdelay(2);
    }

    /* Block Data Update so register reads don't tear. */
    lsm6dso32x_block_data_update_set(ctx, PROPERTY_ENABLE);

    /* Full-scale + ODR. */
    lsm6dso32x_xl_full_scale_set(ctx, xl_fs_enum(accel_range_g));
    lsm6dso32x_gy_full_scale_set(ctx, gy_fs_enum(gyro_range_dps));
    lsm6dso32x_xl_data_rate_set(ctx, xl_odr_enum(sample_rate_hz));
    lsm6dso32x_gy_data_rate_set(ctx, gy_odr_enum(sample_rate_hz));

    /* Power mode (v2.3.0 enums: LSM6DSO32X_HIGH_PERFORMANCE_MD /
     * LSM6DSO32X_LOW_NORMAL_POWER_MD for accel; LSM6DSO32X_GY_HIGH_PERFORMANCE
     * / LSM6DSO32X_GY_NORMAL for gyro). */
    if (high_performance) {
        lsm6dso32x_xl_power_mode_set(ctx, LSM6DSO32X_HIGH_PERFORMANCE_MD);
        lsm6dso32x_gy_power_mode_set(ctx, LSM6DSO32X_GY_HIGH_PERFORMANCE);
    } else {
        lsm6dso32x_xl_power_mode_set(ctx, LSM6DSO32X_LOW_NORMAL_POWER_MD);
        lsm6dso32x_gy_power_mode_set(ctx, LSM6DSO32X_GY_NORMAL);
    }

    /* FIFO: continuous-overwrite (STREAM), BDR = ODR for both XL and GY. */
    lsm6dso32x_fifo_xl_batch_set(ctx, xl_bdr_enum(sample_rate_hz));
    lsm6dso32x_fifo_gy_batch_set(ctx, gy_bdr_enum(sample_rate_hz));
    lsm6dso32x_fifo_mode_set(ctx, LSM6DSO32X_STREAM_MODE);

    s_have_gyro[imu_index]    = false;
    s_gyro_fresh[imu_index]   = false;
    s_overrun_flag[imu_index] = false;
    s_per_state[imu_index]    = IDL0_IMU_OK;
    ESP_LOGI(TAG, "IMU%u up: ODR=%u Hz, AccelFS=±%ug, GyroFS=±%udps, %s",
             (unsigned)imu_index,
             (unsigned)sample_rate_hz, (unsigned)accel_range_g,
             (unsigned)gyro_range_dps,
             high_performance ? "HIGH_PERFORMANCE" : "LOW_NORMAL");
    return true;
}

uint16_t idl0_imu_fifo_count(uint8_t imu_index)
{
    if (imu_index >= IDL0_IMU_COUNT) return 0;
    if (s_per_state[imu_index] != IDL0_IMU_OK) return 0;

    stmdev_ctx_t *ctx = &s_ctx[imu_index];

    /* Unread-words count: 16-bit, via the dedicated helper that reads
     * FIFO_STATUS1 + the low 2 bits of FIFO_STATUS2. The status2 struct
     * itself only exposes those low 2 bits + flag bits, so we read both. */
    uint16_t level = 0;
    if (lsm6dso32x_fifo_data_level_get(ctx, &level) != 0) {
        s_per_state[imu_index] = IDL0_IMU_ERROR;
        return 0;
    }

    /* Overrun: latched in FIFO_STATUS2.fifo_ovr_ia. */
    lsm6dso32x_fifo_status2_t st = {0};
    if (lsm6dso32x_fifo_status_get(ctx, &st) != 0) {
        s_per_state[imu_index] = IDL0_IMU_ERROR;
        return level;
    }
    if (st.fifo_ovr_ia) {
        s_overrun_flag[imu_index] = true;
    }
    return level;
}

bool idl0_imu_pop_sample(uint8_t imu_index, idl0_imu_sample_t *out)
{
    if (imu_index >= IDL0_IMU_COUNT) return false;
    if (s_per_state[imu_index] != IDL0_IMU_OK) return false;

    stmdev_ctx_t *ctx = &s_ctx[imu_index];

    /* Read the tag of the next FIFO word, then read its 6 data bytes.
     * In v2.3.0 the tag is a separate getter (FIFO_DATA_OUT_TAG); the
     * payload comes from FIFO_DATA_OUT_X_L..Z_H via fifo_out_raw_get. */
    lsm6dso32x_fifo_tag_t tag = (lsm6dso32x_fifo_tag_t)0;
    if (lsm6dso32x_fifo_sensor_tag_get(ctx, &tag) != 0) {
        s_per_state[imu_index] = IDL0_IMU_ERROR;
        return false;
    }
    uint8_t data[6];
    if (lsm6dso32x_fifo_out_raw_get(ctx, data) != 0) {
        s_per_state[imu_index] = IDL0_IMU_ERROR;
        return false;
    }

    if (tag == LSM6DSO32X_GYRO_NC_TAG) {
        memcpy(s_pending_gyro[imu_index], data, sizeof(s_pending_gyro[imu_index]));
        s_have_gyro[imu_index] = true;
        return false;
    }
    if (tag == LSM6DSO32X_XL_NC_TAG) {
        if (!s_have_gyro[imu_index]) {
            /* Lone accel — startup case or post-resync. Drop and wait
             * for the next gyro to re-sync the pair stream. */
            return false;
        }
        memcpy(out->gyro,  s_pending_gyro[imu_index], sizeof(out->gyro));
        memcpy(out->accel, data,                       sizeof(out->accel));
        s_have_gyro[imu_index] = false;
        return true;
    }
    /* Other tags (timestamp, temperature, etc.) — not enabled in our
     * config; skip harmlessly if they appear. */
    return false;
}

bool idl0_imu_check_overrun(uint8_t imu_index)
{
    if (imu_index >= IDL0_IMU_COUNT) return false;
    bool was = s_overrun_flag[imu_index];
    s_overrun_flag[imu_index] = false;
    return was;
}

void idl0_imu_take_pair_diag(uint8_t imu_index, uint32_t *reuse, uint32_t *unknown)
{
    if (imu_index >= IDL0_IMU_COUNT) {
        if (reuse)   *reuse   = 0;
        if (unknown) *unknown = 0;
        return;
    }
    if (reuse)   *reuse   = s_reuse[imu_index];
    if (unknown) *unknown = s_unknown[imu_index];
    s_reuse[imu_index]   = 0;
    s_unknown[imu_index] = 0;
}

/* DMA-capable, word-aligned scratch for the burst read. The IMU task is the
 * sole caller and drains the IMUs one at a time (serial round-robin), so a
 * single shared tx/rx pair is safe — there is no reentrancy. Static, not on the
 * task stack: 2 × ~3.5 KB would blow it. The +4 pads the buffer so the SPI DMA
 * engine, which rounds the rx length up to a 4-byte multiple, can never write
 * past the end at the full 1 + 512×7 = 3585-byte transaction. */
#define IMU_SPI_BURST_BUF_BYTES (1 + IMU_SPI_BURST_MAX_WORDS * 7 + 4)
static WORD_ALIGNED_ATTR uint8_t s_burst_tx[IMU_SPI_BURST_BUF_BYTES];
static WORD_ALIGNED_ATTR uint8_t s_burst_rx[IMU_SPI_BURST_BUF_BYTES];

size_t idl0_imu_drain(uint8_t imu_index, idl0_imu_sample_t *out, size_t max_pairs)
{
    if (imu_index >= IDL0_IMU_COUNT) return 0;
    if (s_per_state[imu_index] != IDL0_IMU_OK) return 0;
    if (max_pairs == 0) return 0;

    /* Unread-words count, and latch the overrun bit, via the existing helper. */
    uint16_t level = idl0_imu_fifo_count(imu_index);
    if (level == 0) return 0;

    size_t words = level;
    if (words > IMU_SPI_BURST_MAX_WORDS) words = IMU_SPI_BURST_MAX_WORDS;
    const size_t nbytes = words * 7;

    /* One full-duplex DMA burst from FIFO_DATA_OUT_TAG (0x78). With IF_INC=1
     * (chip default; the init never clears it) consecutive 7-byte FIFO words
     * stream back-to-back, the address auto-incrementing 0x79..0x7E then
     * wrapping to 0x78 of the next word. MSB=1 on the address byte selects a
     * read. This collapses ~words*2 blocking per-word transactions into one. */
    s_burst_tx[0] = 0x78 | 0x80;
    memset(s_burst_tx + 1, 0, nbytes);
    spi_transaction_t t = {
        .length    = (1 + nbytes) * 8,
        .tx_buffer = s_burst_tx,
        .rx_buffer = s_burst_rx,
    };
    if (spi_device_transmit(s_spi[imu_index], &t) != ESP_OK) {
        s_per_state[imu_index] = IDL0_IMU_ERROR;
        return 0;
    }

    /* Pair gyro→accel words in CPU — pure memory parse, no I/O per word. Same
     * logic and carry state as idl0_imu_pop_sample, mirroring idl0_imu_drain_i2c:
     * a torn pair at the buffer boundary resolves on the next drain via
     * s_pending_gyro/s_have_gyro. Tag = byte[0] >> 3 (low 3 bits are
     * parity+counter, ignored). */
    const uint8_t *buf = s_burst_rx + 1;
    size_t n_pairs = 0;
    for (size_t w = 0; w < words && n_pairs < max_pairs; w++) {
        const uint8_t *word = &buf[w * 7];
        const lsm6dso32x_fifo_tag_t tag = (lsm6dso32x_fifo_tag_t)(word[0] >> 3);
        const uint8_t *data = &word[1];

        if (tag == LSM6DSO32X_GYRO_NC_TAG) {
            memcpy(s_pending_gyro[imu_index], data,
                   sizeof(s_pending_gyro[imu_index]));
            s_have_gyro[imu_index]  = true;
            s_gyro_fresh[imu_index] = true;
        } else if (tag == LSM6DSO32X_XL_NC_TAG) {
            /* Emit one sample per accel, pairing with the most-recent gyro. We
             * deliberately do NOT clear s_have_gyro after a pair: on a FIFO
             * phase slip (G,A,A) the second accel reuses the last gyro (≤1
             * sample / 1.2 ms stale) rather than being dropped — lossless, and
             * it self-resyncs on the next gyro word. Only a never-yet-seen gyro
             * (startup) skips. */
            if (!s_have_gyro[imu_index]) continue;
            if (!s_gyro_fresh[imu_index]) s_reuse[imu_index]++;  /* phase slip */
            s_gyro_fresh[imu_index] = false;
            memcpy(out[n_pairs].gyro,  s_pending_gyro[imu_index],
                   sizeof(out[n_pairs].gyro));
            memcpy(out[n_pairs].accel, data, sizeof(out[n_pairs].accel));
            n_pairs++;
        } else {
            s_unknown[imu_index]++;   /* unrecognised tag — corruption signal */
        }
    }
    return n_pairs;
}
