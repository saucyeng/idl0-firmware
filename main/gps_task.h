/* GPS reader task.
 *
 * Reads bytes from the GPS UART, accumulates them into NMEA lines,
 * feeds them to the gps_driver integrator, and submits GPS_FIX records
 * (§5.6) into writer_task. On first fix, anchors the session's wall
 * clock and renames the session file via session.c. Watches for lost
 * lock (no valid RMC for >5 s) and pushes FIX → NOFIX in that case.
 */

#pragma once

#include <stdbool.h>

/* Brings up the UART (via idl0_gps_init) and starts the reader task.
 * Idempotent. */
bool idl0_gps_task_start(void);
