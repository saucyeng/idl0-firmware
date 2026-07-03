/* IMU polling task.
 *
 * Periodically reads FIFO_STATUS, drains all available FIFO words via
 * the IMU driver, builds (gyro, accel) sample pairs, assigns per-
 * sample timestamps walking back from the read instant at the nominal
 * ODR cadence (§5.5), and submits one IMU_SAMPLE record per pair into
 * the writer pipeline when a session is running.
 *
 * Outside a session the task still drains the FIFO so it doesn't
 * overflow (the §7.3 status string surfaces overruns when they occur
 * — §5.5: not written to the file). */

#pragma once

#include <stdbool.h>

/* Brings up IMU0 from config and starts the polling task. Idempotent. */
bool idl0_imu_task_start(void);

/* Drain-instrumentation hooks for the session layer. The IMU task accumulates
 * per-drain timing in RAM during a session; the session layer zeroes it at
 * start and, at stop (after the session file closes), flushes a summary to the
 * on-SD diag log — so nothing is written mid-session (§1: raw capture only). */
void idl0_imu_diag_reset(void);
void idl0_imu_diag_flush(void);
