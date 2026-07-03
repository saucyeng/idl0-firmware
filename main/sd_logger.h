/* SD card SPI / FATFS mount and append-only log writer.
 *
 * Owns the mutex protecting the open session file. Public API is
 * intentionally thin: open, append, close, free-space query.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"

/* FATFS mount point for the SD card. */
#define IDL0_SD_MOUNT_POINT "/sdcard"

/* SD subsystem state, mapped 1:1 onto the §7.3 `SD:` status token. */
typedef enum {
    IDL0_SD_ABSENT = 0,  /* no card, or mount failed */
    IDL0_SD_OK,          /* mounted, free space >= IDL0_SD_MIN_FREE_MB */
    IDL0_SD_FULL,        /* mounted, free space  < IDL0_SD_MIN_FREE_MB */
    IDL0_SD_ERROR,       /* mounted, but a file write has failed */
} idl0_sd_state_t;

#define IDL0_SD_MIN_FREE_MB 200

bool idl0_sd_init(spi_host_device_t spi_host);

/* Current SD state. Valid before and after idl0_sd_init(). */
idl0_sd_state_t idl0_sd_state(void);

/* "OK" | "FULL" | "ERROR" | "ABSENT" — the §7.3 SD: token. */
const char *idl0_sd_state_str(idl0_sd_state_t state);

/* Open /sessions/tmp_<boot_ms>.idl0. Caller renames on first GPS fix. */
bool idl0_sd_open_session(int64_t boot_ms);

bool idl0_sd_append(const uint8_t *buf, size_t len);

/* Flush buffered data to the card and fsync the directory entry — commits the
 * file SIZE + FAT, not just the data. idl0_sd_append no longer flushes per call
 * (it fills a 16 KB cluster buffer); the writer task calls this on a ~1 Hz
 * cadence, so an unexpected power loss costs at most ~1 s of samples AND the
 * on-disk file size never lags the data by more than that. Without the fsync,
 * FATFS would commit the size only at close, freezing it mid-session (the data
 * lands on the card but the file reads as truncated). Returns true on success.
 * No-op (false) if no session file is open. */
bool idl0_sd_flush(void);

/* Rename the currently open file to /sessions/YYYY-MM-DD_HH-MM-SS.idl0
 * using the supplied UTC seconds-since-epoch. */
bool idl0_sd_rename_with_timestamp(int64_t utc_seconds);

bool idl0_sd_close_session(void);

/* Free space in MB on the FAT volume. */
uint32_t idl0_sd_free_mb(void);
