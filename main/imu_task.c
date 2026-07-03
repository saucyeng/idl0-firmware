/* IMU polling task — drains all enabled IMUs and emits IMU_SAMPLE
 * records (§5.5) via the shared writer. Three IMUs (§3) are polled
 * sequentially every IMU_POLL_MS; each IMU's read instant is taken
 * just before its own drain so per-sample timestamps are accurate
 * per-instance (no cross-IMU skew).
 */

#include "imu_task.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"

#include "diag_log.h"
#include "idl0_config.h"
#include "idl0_format.h"
#include "imu_driver.h"
#include "imu_driver_i2c.h"   /* TEMP: I²C path for external IMUs */
#include "mode_state.h"
#include "session.h"
#include "writer_task.h"

#define IMU_POLL_MS            20    /* drain every 20 ms — keeps FIFO occupancy
                                      * ~17 pairs at 833 Hz (was 50 ms → ~127
                                      * pairs ≈ FIFO capacity = drops). */
#define IMU_DRAIN_MAX_PAIRS    256   /* full LSM6DSO32X FIFO (256 pairs). Lets one
                                      * drain fully recover after a stall instead
                                      * of leaving residual to overflow. */
#define IMU_DIAG_LOG_PERIOD_MS 5000  /* periodic serial "drained N pairs" log */

static const char *TAG = "imu_task";

/* TEMP: per-IMU bus selection. IMU0 is the onboard LSM6DSO32 wired
 * direct to the C6 — stays on SPI via [imu_driver.c]. IMU1 and IMU2
 * are Adafruit 4692 breakouts whose passive level shifter blocks
 * push-pull SPI; they run on I²C via [imu_driver_i2c.c]. Remove this
 * table, the dispatch helpers below, and the I²C branch in
 * idl0_imu_task_start() once the external boards are reworked. */
typedef enum { IDL0_BUS_SPI, IDL0_BUS_I2C } idl0_imu_bus_t;
static const idl0_imu_bus_t s_imu_bus[IDL0_IMU_COUNT] = {
    IDL0_BUS_SPI,   /* IMU0 — onboard, direct SPI */
    IDL0_BUS_I2C,   /* IMU1 — external 4692, I²C @ 0x6A (AD0 jumper open) */
    IDL0_BUS_I2C,   /* IMU2 — external 4692, I²C @ 0x6B (close AD0 jumper) */
};
static const uint8_t s_imu_i2c_addr[IDL0_IMU_COUNT] = {
    0,      /* IMU0 not on I²C */
    0x6A,   /* IMU1 — board ships with AD0 open */
    0x6B,   /* IMU2 — requires AD0 solder jumper closed on this board */
};

static inline bool imu_is_active(uint8_t i) {
    return s_imu_bus[i] == IDL0_BUS_I2C
        ? idl0_imu_is_active_i2c(i)
        : idl0_imu_is_active(i);
}
/* Drain is bus-dispatched inline in drain_one() (idl0_imu_drain /
 * idl0_imu_drain_i2c); the per-word fifo_count/pop_sample helpers are gone. */
static inline bool imu_check_overrun(uint8_t i) {
    return s_imu_bus[i] == IDL0_BUS_I2C
        ? idl0_imu_check_overrun_i2c(i)
        : idl0_imu_check_overrun(i);
}
static inline void imu_take_pair_diag(uint8_t i, uint32_t *reuse, uint32_t *unknown) {
    if (s_imu_bus[i] == IDL0_BUS_I2C) {
        idl0_imu_take_pair_diag_i2c(i, reuse, unknown);
    } else {
        idl0_imu_take_pair_diag(i, reuse, unknown);
    }
}

static TaskHandle_t s_task         = NULL;
static uint16_t     s_odr_hz       = 104;
static uint32_t     s_channel_mask = 0;

/* Per-IMU counters for the periodic diagnostic log. Reset on each log emit. */
static uint32_t s_diag_pairs[IDL0_IMU_COUNT];
static uint32_t s_diag_overruns[IDL0_IMU_COUNT];
static uint32_t s_diag_writer_drops;   /* batches the writer rejected this window */
static int64_t  s_diag_last_log_us;

/* Last timestamp emitted per IMU — used to keep timestamps strictly monotonic
 * (§5.5 back-counting from a jittery per-drain esp_timer snapshot can otherwise
 * step ≤2 ms backwards at a drain boundary on the first-drained IMU). */
static int64_t s_last_ts[IDL0_IMU_COUNT] = { INT64_MIN, INT64_MIN, INT64_MIN };

/* Session drain instrumentation. Accumulated in RAM across a logging session
 * and emitted to the on-SD diag log (idl0_debug.log) by idl0_imu_diag_flush()
 * AFTER the session file closes — never written mid-session (§1: raw capture
 * only). Attributes the per-drain cost (bus read vs total) so the field log can
 * show where the cycle goes and confirm the burst path. */
static uint32_t s_m_drains[IDL0_IMU_COUNT];
static uint64_t s_m_read_sum_us[IDL0_IMU_COUNT];   /* idl0_imu_drain* read time */
static uint32_t s_m_read_max_us[IDL0_IMU_COUNT];
static uint64_t s_m_total_sum_us[IDL0_IMU_COUNT];  /* full drain_one time */
static uint32_t s_m_total_max_us[IDL0_IMU_COUNT];
static uint32_t s_m_pairs[IDL0_IMU_COUNT];
static uint32_t s_m_overruns[IDL0_IMU_COUNT];
static uint32_t s_m_cycle_max_us;                  /* slowest full poll cycle */
static uint32_t s_m_writer_drops;

/* Drain scratch — static, shared, single-IMU-at-a-time (the round-robin is
 * serial, so there's no reentrancy). At 256 pairs these are 3 KB + 6 KB;
 * static keeps them off the task stack. `s_batch` holds one IMU_SAMPLE record
 * per pair (24 = max record size). drain_one() emits records to the writer
 * when a session is running, and drains for FIFO hygiene otherwise. */
static idl0_imu_sample_t s_pairs[IMU_DRAIN_MAX_PAIRS];
static uint8_t           s_batch[IMU_DRAIN_MAX_PAIRS * 24];

static void drain_one(uint8_t imu_index, int64_t period_us)
{
    if (!imu_is_active(imu_index)) return;

    /* Snapshot the read instant before draining so all per-sample
     * timestamps anchor on the same t_read for this IMU. */
    const int64_t t_read_us = esp_timer_get_time();

    /* Both buses now use a single burst transaction (SPI via DMA, I²C via one
     * multi-byte read) — the per-word SPI loop was removed: its ~2 blocking
     * transactions/word dominated the poll cycle (see the firmware drop
     * root-cause doc). */
    const size_t n_pairs = (s_imu_bus[imu_index] == IDL0_BUS_I2C)
        ? idl0_imu_drain_i2c(imu_index, s_pairs, IMU_DRAIN_MAX_PAIRS)
        : idl0_imu_drain(imu_index, s_pairs, IMU_DRAIN_MAX_PAIRS);

    const int64_t t_after_read = esp_timer_get_time();

    if (imu_check_overrun(imu_index)) {
        s_diag_overruns[imu_index]++;
        s_m_overruns[imu_index]++;
    }

    s_diag_pairs[imu_index] += n_pairs;

    /* Per-drain instrumentation (RAM only; flushed to the diag log at session
     * end). Counts every drain so the read-vs-total split is visible. */
    const uint32_t read_us = (uint32_t)(t_after_read - t_read_us);
    s_m_drains[imu_index]++;
    s_m_pairs[imu_index] += n_pairs;
    s_m_read_sum_us[imu_index] += read_us;
    if (read_us > s_m_read_max_us[imu_index]) s_m_read_max_us[imu_index] = read_us;

    if (n_pairs == 0) goto account_total;
    if (!idl0_session_is_running()) goto account_total;  /* drained for hygiene */

    /* Walk-back timestamps per §5.5: sample 0 is oldest, sample N-1 is newest
     * = t_read_us. Records are formatted into one batch buffer and pushed to
     * the writer as a single ring-buffer item per drain (per-record submits
     * would flood the writer). A per-IMU monotonic clamp keeps timestamps
     * strictly increasing across drain boundaries. */
    size_t batch_len = 0;
    for (size_t i = 0; i < n_pairs; i++) {
        int64_t ts = t_read_us - (int64_t)(n_pairs - 1 - i) * period_us;
        if (ts <= s_last_ts[imu_index]) {
            ts = s_last_ts[imu_index] + 1;   /* clamp: never step backwards */
        }
        s_last_ts[imu_index] = ts;
        const int16_t axes[6] = {
            s_pairs[i].accel[0], s_pairs[i].accel[1], s_pairs[i].accel[2],
            s_pairs[i].gyro[0],  s_pairs[i].gyro[1],  s_pairs[i].gyro[2],
        };
        batch_len += idl0_format_imu_sample(
            s_batch + batch_len, imu_index, ts, s_channel_mask, axes);
    }
    if (batch_len > 0 && !idl0_writer_submit(s_batch, batch_len)) {
        /* Ring buffer full — count it; do NOT log per drop. A per-batch
         * ESP_LOGW here floods the UART console (~5 ms/line, blocking) on the
         * IMU task, which delays the next drain and starves the writer — a
         * feedback loop that turns a brief backlog into seconds of drops. The
         * count surfaces in the 5 s diag line and the session-end summary. */
        s_m_writer_drops++;
        s_diag_writer_drops++;
    }

account_total:;
    const uint32_t total_us = (uint32_t)(esp_timer_get_time() - t_read_us);
    s_m_total_sum_us[imu_index] += total_us;
    if (total_us > s_m_total_max_us[imu_index]) s_m_total_max_us[imu_index] = total_us;
}

/* Zero the session drain accumulators. Called by the session layer at start. */
void idl0_imu_diag_reset(void)
{
    memset(s_m_drains,        0, sizeof(s_m_drains));
    memset(s_m_read_sum_us,   0, sizeof(s_m_read_sum_us));
    memset(s_m_read_max_us,   0, sizeof(s_m_read_max_us));
    memset(s_m_total_sum_us,  0, sizeof(s_m_total_sum_us));
    memset(s_m_total_max_us,  0, sizeof(s_m_total_max_us));
    memset(s_m_pairs,         0, sizeof(s_m_pairs));
    memset(s_m_overruns,      0, sizeof(s_m_overruns));
    s_m_cycle_max_us = 0;
    s_m_writer_drops = 0;
    /* Discard pair-builder diagnostics accumulated between sessions so the
     * session-end line reflects only this session. */
    for (uint8_t i = 0; i < IDL0_IMU_COUNT; i++) {
        imu_take_pair_diag(i, NULL, NULL);
    }
}

/* Emit the accumulated drain stats to the on-SD diag log. Called by the session
 * layer at stop, AFTER the session file is closed, so it never contends with
 * session SD writes (§1). One line per active IMU + one cycle/summary line. */
void idl0_imu_diag_flush(void)
{
    for (uint8_t i = 0; i < IDL0_IMU_COUNT; i++) {
        if (s_m_drains[i] == 0) continue;
        uint32_t reuse = 0, unknown = 0;
        imu_take_pair_diag(i, &reuse, &unknown);
        idl0_diag_log_event(
            "IMU%u %s drains=%lu pairs=%lu ovr=%lu reuse=%lu unk=%lu read_us=%lu/%lu tot_us=%lu/%lu",
            (unsigned)i, s_imu_bus[i] == IDL0_BUS_I2C ? "i2c" : "spi",
            (unsigned long)s_m_drains[i], (unsigned long)s_m_pairs[i],
            (unsigned long)s_m_overruns[i],
            (unsigned long)reuse, (unsigned long)unknown,
            (unsigned long)(s_m_read_sum_us[i] / s_m_drains[i]),
            (unsigned long)s_m_read_max_us[i],
            (unsigned long)(s_m_total_sum_us[i] / s_m_drains[i]),
            (unsigned long)s_m_total_max_us[i]);
    }
    idl0_diag_log_event("IMU cycle_max_us=%lu writer_drops=%lu poll_ms=%d cap=%d",
                        (unsigned long)s_m_cycle_max_us,
                        (unsigned long)s_m_writer_drops,
                        IMU_POLL_MS, IMU_DRAIN_MAX_PAIRS);
}

static void emit_diag_log(int64_t now_us)
{
    /* Show per-IMU pair counts + overruns since last log. Lets us
     * see at a glance whether each IMU's FIFO is filling (and at
     * roughly the expected rate vs ODR). */
    ESP_LOGI(TAG, "drained over last %u ms — "
                  "IMU0: %lu pairs ovr=%lu | "
                  "IMU1: %lu pairs ovr=%lu | "
                  "IMU2: %lu pairs ovr=%lu | writer_drops=%lu",
             (unsigned)IMU_DIAG_LOG_PERIOD_MS,
             (unsigned long)s_diag_pairs[0],    (unsigned long)s_diag_overruns[0],
             (unsigned long)s_diag_pairs[1],    (unsigned long)s_diag_overruns[1],
             (unsigned long)s_diag_pairs[2],    (unsigned long)s_diag_overruns[2],
             (unsigned long)s_diag_writer_drops);
    for (int i = 0; i < IDL0_IMU_COUNT; i++) {
        s_diag_pairs[i]    = 0;
        s_diag_overruns[i] = 0;
    }
    s_diag_writer_drops = 0;
    s_diag_last_log_us = now_us;
}

static void imu_task_fn(void *arg)
{
    (void)arg;

    /* Pre-compute the inter-sample period in µs from the configured ODR. */
    const int64_t period_us = (s_odr_hz > 0) ? (1000000LL / s_odr_hz) : 10000LL;

    s_diag_last_log_us = esp_timer_get_time();
    bool prev_wifi = (idl0_mode_get_bits() & IDL0_MODE_BIT_WIFI_UP) != 0;
    int64_t prev_cycle_us = 0;   /* start of the previous drain pass */

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(IMU_POLL_MS));

        /* §10.4 — while the SoftAP is up the WiFi/logging mutex guarantees no
         * session, so there is no consumer for IMU data. Stop draining: IMU0
         * shares the SPI2 bus with the SD card, so its per-cycle reads (plus
         * the IMU1/2 I²C bursts) steal bus-lock time and CPU from the /download
         * fread loop. Suspending lets the transfer have the bus. FIFOs simply
         * overflow while suspended; the first drain after WIFI_OFF discards the
         * stale backlog — exactly the existing no-session "hygiene" path. */
        const bool wifi_up = (idl0_mode_get_bits() & IDL0_MODE_BIT_WIFI_UP) != 0;
        if (wifi_up != prev_wifi) {
            ESP_LOGI(TAG, "%s IMU sampling (SoftAP %s)",
                     wifi_up ? "suspending" : "resuming",
                     wifi_up ? "up" : "down");
            prev_wifi = wifi_up;
            /* Re-anchor the diag window so the resume log isn't skewed by the
             * suspended gap. */
            if (!wifi_up) s_diag_last_log_us = esp_timer_get_time();
        }
        if (wifi_up) continue;

        const int64_t cycle_start_us = esp_timer_get_time();
        for (uint8_t i = 0; i < IDL0_IMU_COUNT; i++) {
            drain_one(i, period_us);
        }

        /* Track the slowest full poll cycle (vTaskDelay + the three drains) for
         * the session diag summary — this is the FIFO's unattended window. */
        if (prev_cycle_us != 0 && idl0_session_is_running()) {
            const uint32_t cyc = (uint32_t)(cycle_start_us - prev_cycle_us);
            if (cyc > s_m_cycle_max_us) s_m_cycle_max_us = cyc;
        }
        prev_cycle_us = cycle_start_us;

        const int64_t now_us = esp_timer_get_time();
        if (now_us - s_diag_last_log_us >= (int64_t)IMU_DIAG_LOG_PERIOD_MS * 1000) {
            emit_diag_log(now_us);
        }
    }
}

bool idl0_imu_task_start(void)
{
    if (s_task != NULL) {
        return true;
    }
    idl0_config_t cfg;
    idl0_config_load(&cfg);  /* defaults on failure */

    /* Respect the config's per-IMU enable flags (from idl0_config.json). The
     * temporary all-on DIAG override was removed: with IMU1/IMU2 disabled in the
     * config they are not initialised (no I²C bring-up) and not polled — the
     * round-robin's imu_is_active() check short-circuits them. */
    ESP_LOGI(TAG, "imu_enabled = { %d, %d, %d } (from config)",
             cfg.imu_enabled[0], cfg.imu_enabled[1], cfg.imu_enabled[2]);

    s_odr_hz       = cfg.imu_sample_rate_hz;
    s_channel_mask = cfg.imu_channel_mask;

    /* Initialise every IMU the config has enabled. Each call is
     * independent — a missing or unresponsive IMU does not block
     * its siblings, the aggregate state just reflects PARTIAL.
     * Bus selection per s_imu_bus[] above (TEMP). */
    int up = 0;
    for (uint8_t i = 0; i < IDL0_IMU_COUNT; i++) {
        if (!cfg.imu_enabled[i]) continue;
        bool ok;
        if (s_imu_bus[i] == IDL0_BUS_I2C) {
            ok = idl0_imu_init_one_i2c(i, s_imu_i2c_addr[i],
                                       cfg.imu_sample_rate_hz,
                                       cfg.accel_range_g[i],
                                       cfg.gyro_range_dps[i],
                                       cfg.high_performance_mode);
        } else {
            ok = idl0_imu_init_one(SPI2_HOST, i,
                                   cfg.imu_sample_rate_hz,
                                   cfg.accel_range_g[i],
                                   cfg.gyro_range_dps[i],
                                   cfg.high_performance_mode);
        }
        if (ok) up++;
    }
    if (up == 0) {
        ESP_LOGE(TAG, "no IMUs initialised");
        return false;
    }

    /* Priority 5 — equal to the SD writer. With the burst drain + 256-pair cap
     * the chip FIFO has ~300 ms of headroom (field logs show ovr=0), so the
     * drain does NOT need to outrank the writer; the binding constraint is now
     * the writer keeping the 64 KB ring drained. Equal priority time-slices the
     * two so neither starves (an earlier prio-6 inversion starved the writer at
     * session start → "buffer full" drops). The task sleeps IMU_POLL_MS between
     * short bursts, leaving the core to the writer and BLE host.
     *
     * 6 KB stack: the big drain/batch buffers are file-static, so the frame is
     * small; 6 KB keeps comfortable headroom. */
    BaseType_t ok = xTaskCreate(imu_task_fn, "imu", 6144, NULL, 5, &s_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        s_task = NULL;
        return false;
    }
    ESP_LOGI(TAG, "IMU task up: %d/%d IMUs active, ODR %u Hz, mask 0x%05lX",
             up, IDL0_IMU_COUNT,
             (unsigned)s_odr_hz, (unsigned long)s_channel_mask);
    return true;
}
