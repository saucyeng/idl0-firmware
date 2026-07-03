/* Session lifecycle: start/stop orchestration, UUID generation,
 * header build, SESSION_END flushing.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ble_service.h"

bool idl0_session_start(void);
bool idl0_session_stop(void);
bool idl0_session_is_running(void);

/* Called from idl0_ble_init callback. */
uint8_t idl0_session_on_ble_command(idl0_ble_command_t cmd);

/* Called by gps_task on the first valid fix received during a running
 * session: anchors session_start_utc and renames the temp file to
 * YYYY-MM-DD_HH-MM-SS.idl0. Idempotent within a session. */
void idl0_session_on_first_fix(int64_t gps_epoch_ms, int64_t device_timestamp_us);
