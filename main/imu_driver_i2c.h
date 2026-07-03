/* TEMP: I²C-bus variant of the IMU driver — for the external Adafruit
 * 4692 breakouts only.
 *
 * The 4692 has a passive bidirectional MOSFET level shifter on its
 * SDA/SCL pads, designed for I²C use. It blocks push-pull SPI because
 * rising edges propagate only via the slow drain-side pull-up, eating
 * the setup-time margin for MOSI vs SCK. I²C avoids the issue because
 * it's open-drain by protocol — exactly what the shifter expects.
 *
 * IMU0 (onboard, no level shifter in path) stays on SPI via
 * [imu_driver.c]. IMU1 and IMU2 run on I²C via this module. Both paths
 * use the same vendored ST register-level driver; the only difference
 * is the platform read/write glue.
 *
 * Removal: delete this header + imu_driver_i2c.c, drop the SRCS entry
 * from main/CMakeLists.txt, and revert the bus-dispatch additions in
 * imu_task.c (search for `IDL0_BUS_I2C`). The SPI driver in
 * imu_driver.c is not touched by this module.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "imu_driver.h"   /* for idl0_imu_sample_t and IDL0_IMU_COUNT */

/* Initialise one IMU on the shared I²C bus at the given 7-bit address.
 * Allocates the bus on first call. Verifies WHO_AM_I (0x6C). Configures
 * accel + gyro at the requested ODR / range / mode, then enables FIFO
 * continuous + BDR equal to ODR. Mirrors idl0_imu_init_one() but over
 * I²C instead of SPI.
 *
 * Returns true on success. On failure the per-instance state is left
 * inactive so other IMUs are unaffected. */
bool idl0_imu_init_one_i2c(uint8_t imu_index,
                           uint8_t i2c_addr,
                           uint16_t sample_rate_hz,
                           uint8_t accel_range_g,
                           uint16_t gyro_range_dps,
                           bool high_performance);

/* Per-IMU runtime queries — mirror the SPI versions in imu_driver.h
 * but operate on I²C-side state. imu_task.c routes calls to the
 * appropriate variant via its bus-dispatch table. */
bool     idl0_imu_is_active_i2c(uint8_t imu_index);
uint16_t idl0_imu_fifo_count_i2c(uint8_t imu_index);
bool     idl0_imu_pop_sample_i2c(uint8_t imu_index, idl0_imu_sample_t *out);
bool     idl0_imu_check_overrun_i2c(uint8_t imu_index);
/* Read + clear the I²C pair-builder diagnostics — see idl0_imu_take_pair_diag()
 * in imu_driver.h. Either out-pointer may be NULL. */
void     idl0_imu_take_pair_diag_i2c(uint8_t imu_index, uint32_t *reuse, uint32_t *unknown);

/* Burst-drain N FIFO pairs in a single I²C transaction. The chip
 * exposes its FIFO output at registers 0x78..0x7E (1 tag + 6 data
 * bytes per word); with IF_INC=1 a multi-byte read from 0x78 streams
 * consecutive FIFO words back-to-back. One transaction with all the
 * setup/teardown overhead replaces N×2 per-word transactions and is
 * what lets the prototype keep up with 3 IMUs at 800 Hz over I²C.
 *
 * Returns the number of (gyro, accel) pairs written to `out` (≤ max_pairs).
 * Updates the per-IMU overrun flag and pending-gyro carry state. */
size_t idl0_imu_drain_i2c(uint8_t imu_index, idl0_imu_sample_t *out,
                          size_t max_pairs);
