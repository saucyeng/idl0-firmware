/* LSM6DSO32 SPI driver — up to three IMUs on shared SPI2_HOST.
 *
 * The shared SPI bus is initialised by sd_logger; this module adds
 * each IMU as a separate device with its own chip-select (CS pins per
 * pins.h) and `stmdev_ctx_t`. The vendored ST register-access driver
 * (firmware/components/lsm6dso32x_STdC/) provides the chip-level API.
 *
 * Spec contracts: §3 (hardware), §4 (firmware), §5.4 / §5.5 (IMU
 * channel mask + IMU_SAMPLE record), §8 (per-IMU ODR / range / mode).
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"

/* Number of physical IMU slots. Matches §3 (IMU0 onboard sprung mass,
 * IMU1 front unsprung, IMU2 rear unsprung) and the channel-mask layout
 * in §5.4 (bits 0-17 = 3 × 6 axes). */
#define IDL0_IMU_COUNT 3

/* IMU subsystem state, mapped 1:1 onto the §7.3 IMU: status token.
 *
 * Aggregate across all configured IMUs:
 *   ABSENT  — no IMU was even attempted (init never called)
 *   OK      — every attempted IMU answered WHO_AM_I and configured cleanly
 *   PARTIAL — some attempted IMUs are up, others failed (e.g. unpopulated)
 *   ERROR   — at least one IMU had a read/write failure after init
 */
typedef enum {
    IDL0_IMU_ABSENT = 0,
    IDL0_IMU_OK,
    IDL0_IMU_PARTIAL,
    IDL0_IMU_ERROR,
} idl0_imu_state_t;

/* One drained sample, ready to be timestamped + framed by imu_task. */
typedef struct {
    int16_t accel[3];  /* X, Y, Z — raw int16 LSB, no scaling */
    int16_t gyro[3];   /* X, Y, Z — raw int16 LSB */
} idl0_imu_sample_t;

/* Aggregate state — see enum doc above. */
idl0_imu_state_t idl0_imu_state(void);
const char *idl0_imu_state_str(idl0_imu_state_t state);

/* Returns true if the given IMU initialised successfully and is currently
 * usable. False for unpopulated slots, init failures, or post-init errors.
 * Bounds-checks imu_index. */
bool idl0_imu_is_active(uint8_t imu_index);

/* Initialise one IMU (by index 0..IDL0_IMU_COUNT-1) on the shared SPI
 * host: add the device with its CS pin (per pins.h), verify WHO_AM_I
 * (0x6C), configure accel + gyro at the given ODR / range / mode,
 * enable FIFO continuous + BDR equal to ODR.
 *
 * `sample_rate_hz`: one of 12 / 13 / 26 / 52 / 104 / 208 / 416 / 833 / 1666.
 * `accel_range_g`:  4, 8, 16, or 32.
 * `gyro_range_dps`: 125, 250, 500, 1000, or 2000.
 * `high_performance`: true → high-performance mode on both accel and gyro.
 *
 * Returns true on success (this IMU's per-instance state → OK).
 * On failure leaves the per-instance state at ABSENT (chip absent /
 * miswired / WHO_AM_I wrong) or ERROR (later read/write failure).
 * Independent of other IMUs — failure here does not affect siblings. */
bool idl0_imu_init_one(spi_host_device_t spi_host,
                       uint8_t imu_index,
                       uint16_t sample_rate_hz,
                       uint8_t accel_range_g,
                       uint16_t gyro_range_dps,
                       bool high_performance);

/* Per-IMU FIFO_STATUS unread-words count (0..512). Returns 0 if this
 * IMU is not active. */
uint16_t idl0_imu_fifo_count(uint8_t imu_index);

/* Pop one tagged FIFO word from the given IMU and append to its
 * pair-builder. If the word completes a (gyro, accel) pair, fills
 * `*out` and returns true; otherwise returns false. Pairs self-resync
 * on any tag-order inversion. Pure SPI I/O — no scaling.
 *
 * Superseded by idl0_imu_drain() for the polling path; kept for the
 * single-word use cases and tests. */
bool idl0_imu_pop_sample(uint8_t imu_index, idl0_imu_sample_t *out);

/* Burst-drain up to `max_pairs` (gyro, accel) pairs in a single DMA SPI
 * transaction (the FIFO streams from 0x78 with IF_INC auto-increment). The
 * SPI-bus analogue of idl0_imu_drain_i2c(): one transaction replaces the
 * per-word tag+data reads, removing the per-transaction overhead that made the
 * SPI drain dominate the poll cycle. Latches the overrun flag and carries the
 * pending gyro across calls. Returns the number of pairs written to `out`. */
size_t idl0_imu_drain(uint8_t imu_index, idl0_imu_sample_t *out, size_t max_pairs);

/* Was this IMU's FIFO overrun bit set since the last call? Clears on
 * read. Used by imu_task to log overruns (§5.5: not written to file). */
bool idl0_imu_check_overrun(uint8_t imu_index);

/* Read + clear this IMU's pair-builder diagnostics since the last call:
 * `reuse` = accels that reused a stale gyro on a FIFO phase slip (the samples
 * the lossless pairing recovers); `unknown` = unrecognised FIFO tags (a
 * corruption signal, 0 on a clean bus). Either out-pointer may be NULL. */
void idl0_imu_take_pair_diag(uint8_t imu_index, uint32_t *reuse, uint32_t *unknown);
