/* OTA streaming write + app-level rollback helpers.
 *
 * Lifecycle: idl0_ota_begin() picks the inactive OTA slot and opens an
 * esp_ota_handle. Repeated idl0_ota_write() calls stream data into it.
 * idl0_ota_end() validates the image's embedded SHA-256 and sets the
 * boot partition. The caller (HTTP handler) then reboots.
 *
 * Rollback: after reboot the new image is in PENDING_VERIFY. The app
 * must send CMD_OTA_CONFIRM (which calls idl0_ota_mark_valid) to
 * commit. If the device reboots before that, the bootloader rolls back.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef struct idl0_ota_session idl0_ota_session_t;

/* Initialise the pending-verify latch by reading the running partition
 * state. Call once from app_main before anyone else queries it. */
void idl0_ota_init(void);

/* True if the currently-running image is in ESP_OTA_IMG_PENDING_VERIFY.
 * Latched on idl0_ota_init() so callers don't have to hit NVS. */
bool idl0_ota_pending_verify(void);

/* Open a write session into the inactive OTA partition.
 * `expected_size` is the announced image size in bytes (from
 * Content-Length), or 0 if unknown. Returns NULL on failure. */
idl0_ota_session_t *idl0_ota_begin(size_t expected_size);

/* Append `len` bytes to the in-progress write. Returns false on flash
 * write error or if no session is open. */
bool idl0_ota_write(idl0_ota_session_t *s, const void *data, size_t len);

/* Finalise the write. Validates the image's embedded SHA-256 and sets
 * the boot partition. Returns false (and aborts the session) on a
 * validation or set-boot failure. Frees the session on success or
 * failure — caller must not touch the handle after this. */
bool idl0_ota_end(idl0_ota_session_t *s);

/* Abort an in-progress session without setting boot. Safe to call on
 * NULL. Frees the session. */
void idl0_ota_abort(idl0_ota_session_t *s);

/* Commit the running image (cancels pending rollback). Returns false
 * if not in PENDING_VERIFY (no-op in that case). */
bool idl0_ota_mark_valid(void);
