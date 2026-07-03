/* Single-consumer SD writer.
 *
 * Sensor tasks (gps_task, imu_task, …) submit records into the ring
 * buffer; one writer_task drains it and calls idl0_sd_append. Only one
 * task ever touches the open SD file — no per-record SD mutex needed
 * outside sd_logger itself.
 *
 * Buffer absorbs SD-write latency transients without dropping records. 64 KB
 * is ~3 s of one IMU at 833 Hz (~20 KB/s) or ~1 s of three — enough to ride out
 * the fresh-file FAT-allocation slowness at session start that was overflowing
 * the old 16 KB ring in the first few seconds.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define IDL0_WRITER_BUFFER_BYTES (64 * 1024)

/* Create the ring buffer and start the writer task. Idempotent. */
bool idl0_writer_init(void);

/* Submit a fully-framed record. Drops on backpressure (returns false)
 * — sensor producers are expected to log+continue on drop. */
bool idl0_writer_submit(const uint8_t *buf, size_t len);

/* Block up to `timeout_ms` for the buffer to drain. Returns true if
 * empty, false on timeout. */
bool idl0_writer_drain(uint32_t timeout_ms);
