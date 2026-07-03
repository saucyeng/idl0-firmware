/* TEMP: I²C-bus variant of the IMU driver — see imu_driver_i2c.h for
 * the rationale and removal procedure.
 *
 * This file deliberately duplicates the config sequence and ODR/range
 * enum mapping from imu_driver.c rather than refactoring shared
 * helpers into a third file. The duplication is the cost of keeping
 * the I²C path entirely self-contained so it can be deleted in one
 * piece when the external breakouts are reworked.
 */

#include "imu_driver_i2c.h"

#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lsm6dso32x_reg.h"
#include "pins.h"

static const char *TAG = "imu_i2c";

#define I2C_BUS_PORT       I2C_NUM_0
/* Fast-mode-plus (1 MHz). LSM6DSO32 supports it per datasheet; the
 * prior Saucy prototype (Datalogger v2) ran 3 BMI160s on a similar
 * harness at this clock with no drops. Lower to 400 kHz if signal
 * integrity over the harness ever degrades. */
#define I2C_BUS_HZ         1000000
#define I2C_TIMEOUT_MS     100
#define IMU_WHO_AM_I       0x6C
/* Max FIFO words drained in one burst. 512 words = the full LSM6DSO32X FIFO
 * (256 pairs), so one drain can fully recover after a stall. 512*7 = 3584 bytes
 * lives in a file-static buffer (below), not on the IMU task stack. */
#define IMU_BURST_MAX_WORDS  512

/* Per-IMU I²C device descriptor. ctx->handle points at one of these
 * so the platform glue can recover both the bus handle and the 7-bit
 * device address from the opaque handle pointer. */
typedef struct {
    i2c_master_dev_handle_t dev;
} i2c_imu_t;

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_imu_t      s_i2c[IDL0_IMU_COUNT];
static stmdev_ctx_t   s_ctx[IDL0_IMU_COUNT];
static bool           s_active[IDL0_IMU_COUNT];
static bool           s_overrun_flag[IDL0_IMU_COUNT];
static int16_t        s_pending_gyro[IDL0_IMU_COUNT][3];
static bool           s_have_gyro[IDL0_IMU_COUNT];
/* Pair-builder diagnostics — mirror imu_driver.c; read+cleared by
 * idl0_imu_take_pair_diag_i2c(). s_reuse: lone accels that reused a stale gyro
 * on a FIFO phase slip (samples the lossless pairing recovers). s_unknown:
 * unrecognised FIFO tags (corruption signal, 0 on a clean bus). */
static bool           s_gyro_fresh[IDL0_IMU_COUNT];
static uint32_t       s_reuse[IDL0_IMU_COUNT];
static uint32_t       s_unknown[IDL0_IMU_COUNT];

/* --- ST driver platform glue --------------------------------------
 * I²C addressing doesn't use the MSB read/write trick from SPI — the
 * direction is in the I²C address byte itself, handled by the master
 * driver. We just send `[reg, data...]` for write and `[reg] → [data...]`
 * (repeated-start) for read. */

static int32_t platform_write(void *handle, uint8_t reg,
                              const uint8_t *bufp, uint16_t len)
{
    i2c_imu_t *imu = (i2c_imu_t *)handle;
    uint8_t buf[1 + 32];
    if (len > sizeof(buf) - 1) return -1;
    buf[0] = reg;
    memcpy(buf + 1, bufp, len);
    esp_err_t err = i2c_master_transmit(imu->dev, buf, 1 + len,
                                        I2C_TIMEOUT_MS);
    return (err == ESP_OK) ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg,
                             uint8_t *bufp, uint16_t len)
{
    i2c_imu_t *imu = (i2c_imu_t *)handle;
    esp_err_t err = i2c_master_transmit_receive(imu->dev, &reg, 1,
                                                bufp, len,
                                                I2C_TIMEOUT_MS);
    return (err == ESP_OK) ? 0 : -1;
}

static void platform_mdelay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* --- spec config values → ST driver enums --- duplicated from
 * imu_driver.c. Same comments / fallbacks apply. */

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

/* --- I²C bus init (idempotent, runs once on first IMU init) --- */

static bool i2c_bus_init(void)
{
    if (s_bus != NULL) return true;
    i2c_master_bus_config_t bus_config = {
        .i2c_port          = I2C_BUS_PORT,
        .sda_io_num        = IDL0_PIN_I2C_SDA,
        .scl_io_num        = IDL0_PIN_I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus: %s", esp_err_to_name(err));
        s_bus = NULL;
        return false;
    }
    ESP_LOGI(TAG, "I²C bus up: SDA=GPIO%d SCL=GPIO%d @ %u Hz",
             IDL0_PIN_I2C_SDA, IDL0_PIN_I2C_SCL, (unsigned)I2C_BUS_HZ);
    return true;
}

/* DIAG: same dump as imu_driver.c's diag_dump_one, but over I²C. Lets
 * us compare the IMU0 (SPI) and IMU1 (I²C) burst reads side-by-side
 * in the boot log to confirm the chip is now reachable. Remove with
 * the rest of the module. */
static void diag_dump_one(uint8_t imu_index, uint8_t i2c_addr)
{
    if (!s_active[imu_index]) {
        /* state isn't OK yet; still dump because we want to see what
         * comes back even when WHO_AM_I is about to fail. */
    }
    stmdev_ctx_t *ctx = &s_ctx[imu_index];
    uint8_t buf[32] = {0};
    int32_t r_burst = lsm6dso32x_read_reg(ctx, 0x00, buf, sizeof(buf));
    uint8_t test_val = 0xA5, readback = 0xEE, zero = 0;
    int32_t w_rc = lsm6dso32x_write_reg(ctx, 0x73, &test_val, 1);
    int32_t r_rc = lsm6dso32x_read_reg(ctx,  0x73, &readback, 1);
    lsm6dso32x_write_reg(ctx, 0x73, &zero, 1);
    ESP_LOGW(TAG, "IMU%u(I²C@0x%02X) DIAG burst 0x00+32 (rc=%ld): "
                  "%02X %02X %02X %02X %02X %02X %02X %02X "
                  "%02X %02X %02X %02X %02X %02X %02X %02X "
                  "%02X %02X %02X %02X %02X %02X %02X %02X "
                  "%02X %02X %02X %02X %02X %02X %02X %02X",
             (unsigned)imu_index, i2c_addr, (long)r_burst,
             buf[0],  buf[1],  buf[2],  buf[3],  buf[4],  buf[5],  buf[6],  buf[7],
             buf[8],  buf[9],  buf[10], buf[11], buf[12], buf[13], buf[14], buf[15],
             buf[16], buf[17], buf[18], buf[19], buf[20], buf[21], buf[22], buf[23],
             buf[24], buf[25], buf[26], buf[27], buf[28], buf[29], buf[30], buf[31]);
    ESP_LOGW(TAG, "IMU%u(I²C@0x%02X) DIAG WHO_AM_I (burst[0x0F])=0x%02X (expect 0x6C); "
                  "X_OFS_USR write 0xA5 → readback 0x%02X (w_rc=%ld r_rc=%ld)",
             (unsigned)imu_index, i2c_addr, buf[0x0F],
             readback, (long)w_rc, (long)r_rc);
}

/* --- Public API --- */

bool idl0_imu_init_one_i2c(uint8_t imu_index,
                           uint8_t i2c_addr,
                           uint16_t sample_rate_hz,
                           uint8_t accel_range_g,
                           uint16_t gyro_range_dps,
                           bool high_performance)
{
    if (imu_index >= IDL0_IMU_COUNT) {
        ESP_LOGE(TAG, "init_one_i2c: index %u out of range", (unsigned)imu_index);
        return false;
    }
    if (!i2c_bus_init()) return false;

    s_active[imu_index] = false;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = i2c_addr,
        .scl_speed_hz    = I2C_BUS_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(s_bus, &dev_cfg,
                                              &s_i2c[imu_index].dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IMU%u(I²C@0x%02X) bus_add_device: %s",
                 (unsigned)imu_index, i2c_addr, esp_err_to_name(err));
        return false;
    }

    stmdev_ctx_t *ctx = &s_ctx[imu_index];
    ctx->write_reg = platform_write;
    ctx->read_reg  = platform_read;
    ctx->mdelay    = platform_mdelay;
    ctx->handle    = &s_i2c[imu_index];

    diag_dump_one(imu_index, i2c_addr);

    uint8_t who = 0;
    if (lsm6dso32x_device_id_get(ctx, &who) != 0 || who != IMU_WHO_AM_I) {
        ESP_LOGW(TAG, "IMU%u(I²C@0x%02X) WHO_AM_I mismatch: got 0x%02X, expected 0x%02X",
                 (unsigned)imu_index, i2c_addr, who, IMU_WHO_AM_I);
        i2c_master_bus_rm_device(s_i2c[imu_index].dev);
        s_i2c[imu_index].dev = NULL;
        return false;
    }

    /* Software reset, wait for it to clear. */
    lsm6dso32x_reset_set(ctx, PROPERTY_ENABLE);
    uint8_t rst = 1;
    for (int i = 0; i < 10 && rst != 0; i++) {
        lsm6dso32x_reset_get(ctx, &rst);
        platform_mdelay(2);
    }

    lsm6dso32x_block_data_update_set(ctx, PROPERTY_ENABLE);

    lsm6dso32x_xl_full_scale_set(ctx, xl_fs_enum(accel_range_g));
    lsm6dso32x_gy_full_scale_set(ctx, gy_fs_enum(gyro_range_dps));
    lsm6dso32x_xl_data_rate_set(ctx, xl_odr_enum(sample_rate_hz));
    lsm6dso32x_gy_data_rate_set(ctx, gy_odr_enum(sample_rate_hz));

    if (high_performance) {
        lsm6dso32x_xl_power_mode_set(ctx, LSM6DSO32X_HIGH_PERFORMANCE_MD);
        lsm6dso32x_gy_power_mode_set(ctx, LSM6DSO32X_GY_HIGH_PERFORMANCE);
    } else {
        lsm6dso32x_xl_power_mode_set(ctx, LSM6DSO32X_LOW_NORMAL_POWER_MD);
        lsm6dso32x_gy_power_mode_set(ctx, LSM6DSO32X_GY_NORMAL);
    }

    lsm6dso32x_fifo_xl_batch_set(ctx, xl_bdr_enum(sample_rate_hz));
    lsm6dso32x_fifo_gy_batch_set(ctx, gy_bdr_enum(sample_rate_hz));
    lsm6dso32x_fifo_mode_set(ctx, LSM6DSO32X_STREAM_MODE);

    s_have_gyro[imu_index]    = false;
    s_gyro_fresh[imu_index]   = false;
    s_overrun_flag[imu_index] = false;
    s_active[imu_index]       = true;
    ESP_LOGI(TAG, "IMU%u up on I²C @ 0x%02X: ODR=%u Hz, AccelFS=±%ug, GyroFS=±%udps, %s",
             (unsigned)imu_index, i2c_addr,
             (unsigned)sample_rate_hz, (unsigned)accel_range_g,
             (unsigned)gyro_range_dps,
             high_performance ? "HIGH_PERFORMANCE" : "LOW_NORMAL");
    return true;
}

bool idl0_imu_is_active_i2c(uint8_t imu_index)
{
    if (imu_index >= IDL0_IMU_COUNT) return false;
    return s_active[imu_index];
}

uint16_t idl0_imu_fifo_count_i2c(uint8_t imu_index)
{
    if (imu_index >= IDL0_IMU_COUNT) return 0;
    if (!s_active[imu_index]) return 0;

    stmdev_ctx_t *ctx = &s_ctx[imu_index];

    uint16_t level = 0;
    if (lsm6dso32x_fifo_data_level_get(ctx, &level) != 0) {
        s_active[imu_index] = false;
        return 0;
    }
    lsm6dso32x_fifo_status2_t st = {0};
    if (lsm6dso32x_fifo_status_get(ctx, &st) != 0) {
        s_active[imu_index] = false;
        return level;
    }
    if (st.fifo_ovr_ia) {
        s_overrun_flag[imu_index] = true;
    }
    return level;
}

bool idl0_imu_pop_sample_i2c(uint8_t imu_index, idl0_imu_sample_t *out)
{
    if (imu_index >= IDL0_IMU_COUNT) return false;
    if (!s_active[imu_index]) return false;

    stmdev_ctx_t *ctx = &s_ctx[imu_index];

    lsm6dso32x_fifo_tag_t tag = (lsm6dso32x_fifo_tag_t)0;
    if (lsm6dso32x_fifo_sensor_tag_get(ctx, &tag) != 0) {
        s_active[imu_index] = false;
        return false;
    }
    uint8_t data[6];
    if (lsm6dso32x_fifo_out_raw_get(ctx, data) != 0) {
        s_active[imu_index] = false;
        return false;
    }

    if (tag == LSM6DSO32X_GYRO_NC_TAG) {
        memcpy(s_pending_gyro[imu_index], data, sizeof(s_pending_gyro[imu_index]));
        s_have_gyro[imu_index] = true;
        return false;
    }
    if (tag == LSM6DSO32X_XL_NC_TAG) {
        if (!s_have_gyro[imu_index]) return false;
        memcpy(out->gyro,  s_pending_gyro[imu_index], sizeof(out->gyro));
        memcpy(out->accel, data,                       sizeof(out->accel));
        s_have_gyro[imu_index] = false;
        return true;
    }
    return false;
}

bool idl0_imu_check_overrun_i2c(uint8_t imu_index)
{
    if (imu_index >= IDL0_IMU_COUNT) return false;
    bool was = s_overrun_flag[imu_index];
    s_overrun_flag[imu_index] = false;
    return was;
}

void idl0_imu_take_pair_diag_i2c(uint8_t imu_index, uint32_t *reuse, uint32_t *unknown)
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

size_t idl0_imu_drain_i2c(uint8_t imu_index, idl0_imu_sample_t *out,
                          size_t max_pairs)
{
    if (imu_index >= IDL0_IMU_COUNT) return 0;
    if (!s_active[imu_index]) return 0;
    if (max_pairs == 0) return 0;

    stmdev_ctx_t *ctx = &s_ctx[imu_index];

    /* Step 1: how many FIFO words are unread, and is the overrun bit
     * latched? Two register reads, ~30 µs each at 1 MHz. */
    uint16_t level = 0;
    if (lsm6dso32x_fifo_data_level_get(ctx, &level) != 0) {
        s_active[imu_index] = false;
        return 0;
    }
    lsm6dso32x_fifo_status2_t st = {0};
    if (lsm6dso32x_fifo_status_get(ctx, &st) == 0 && st.fifo_ovr_ia) {
        s_overrun_flag[imu_index] = true;
    }
    if (level == 0) return 0;

    /* Cap to our burst buffer. Anything beyond stays in the chip's
     * FIFO and gets picked up next poll — no data loss as long as we
     * don't fall behind the 512-word chip FIFO depth. */
    size_t words = level;
    if (words > IMU_BURST_MAX_WORDS) words = IMU_BURST_MAX_WORDS;

    /* Step 2: one I²C transaction reading words × 7 bytes starting at
     * FIFO_DATA_OUT_TAG (0x78). With IF_INC=1 (default) the chip
     * streams consecutive FIFO words; when the auto-increment passes
     * 0x7E it wraps to 0x78 of the next word. This is the entire point
     * of the burst path — replaces words×2 separate transactions.
     *
     * 7-byte word layout: [tag, X_L, X_H, Y_L, Y_H, Z_L, Z_H]
     * Tag byte: bits 7:3 = sensor tag, bits 2:0 = parity+counter (ignored).
     *
     * Static (file-scope) buffer: 3584 bytes is too large for the IMU task
     * stack. The IMU task drains IMUs one at a time, so a single shared buffer
     * is safe — no reentrancy. */
    static uint8_t buf[IMU_BURST_MAX_WORDS * 7];
    const size_t nbytes = words * 7;
    if (lsm6dso32x_read_reg(ctx, 0x78, buf, nbytes) != 0) {
        s_active[imu_index] = false;
        return 0;
    }

    /* Step 3: walk the buffer pairing gyro→accel words. Same logic as
     * the per-word pop_sample but no I/O per word — pure memory parse.
     * Carries the pending gyro across calls via s_pending_gyro/s_have_gyro
     * so a torn pair at the buffer boundary resolves on the next drain. */
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
            /* Emit one sample per accel, pairing with the most-recent gyro (see
             * imu_driver.c): do NOT clear s_have_gyro after a pair, so a lone
             * accel on a FIFO phase slip reuses the last gyro (≤1 sample stale)
             * instead of being dropped — lossless, self-resyncs on the next
             * gyro. Only a never-yet-seen gyro (startup) skips. */
            if (!s_have_gyro[imu_index]) continue;
            if (!s_gyro_fresh[imu_index]) s_reuse[imu_index]++;  /* phase slip */
            s_gyro_fresh[imu_index] = false;
            memcpy(out[n_pairs].gyro,  s_pending_gyro[imu_index],
                   sizeof(out[n_pairs].gyro));
            memcpy(out[n_pairs].accel, data,
                   sizeof(out[n_pairs].accel));
            n_pairs++;
        } else {
            s_unknown[imu_index]++;   /* unrecognised tag — corruption signal */
        }
    }
    return n_pairs;
}
