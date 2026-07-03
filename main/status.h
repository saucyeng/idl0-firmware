/* §7.3 BLE status-string publisher.
 *
 * Each subsystem (sd, gps, imu, session) exposes its current state via
 * its own accessor; this module composes the multi-line string and
 * pushes it to the FF04 characteristic. Call after any state change.
 */

#pragma once

void idl0_status_publish(void);

/* Starts a 1 Hz periodic publish so the connected BLE central sees live
 * BPM / HR state / battery without relying on every subsystem to fire
 * an on-change publish. Idempotent; safe to call once at boot. */
void idl0_status_publisher_start(void);
