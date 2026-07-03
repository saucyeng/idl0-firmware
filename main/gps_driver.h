/* u-blox MAX-M10S GPS driver — NMEA parser + UART RX.
 *
 * P6 hardware limitation: the ESP→module TX trace is open on the
 * current board revision, so the firmware cannot send UBX commands.
 * The module runs at factory defaults: 9600 baud, RMC/GGA at 1 Hz
 * (we ignore the other default sentences). When TX is rewired, the
 * UBX init burst is added behind idl0_gps_init() — TODO(idl0).
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The rate the module actually emits at, used by the file-header
 * gps_sample_rate_hz field (§5.1). With TX broken we cannot reconfigure
 * the module, so this is the factory default. */
#define IDL0_GPS_ACTUAL_RATE_HZ 1

/* Parsed GPS fix — fields mirror §5.6 GPS_FIX record. */
typedef struct {
    int64_t gps_epoch_ms;
    int64_t device_timestamp_us;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int16_t altitude_x10;
    uint16_t speed_x100;
    uint16_t heading_x100;
    uint8_t fix_quality;
    uint8_t satellites;
} idl0_gps_fix_t;

/* GPS subsystem state, mapped 1:1 onto the §7.3 GPS: status token. */
typedef enum {
    IDL0_GPS_ABSENT = 0,  /* gps_init not called or failed */
    IDL0_GPS_NOFIX,       /* UART up, no valid RMC received yet */
    IDL0_GPS_FIX,         /* at least one RMC with status='A' seen */
} idl0_gps_state_t;

/* Initialise UART RX (TX pin not wired). Returns true on success. */
bool idl0_gps_init(void);

idl0_gps_state_t idl0_gps_state(void);
const char *idl0_gps_state_str(idl0_gps_state_t state);

/* Internal hook used by gps_task's lost-lock watchdog. Forces the
 * driver's state-machine to the given state without touching the
 * integrator's pending fix struct. Do not call from anywhere else —
 * normal transitions happen inside idl0_gps_feed_line. */
void idl0_gps_force_state(idl0_gps_state_t state);

/* --- NMEA parser — exposed for host-test --- */

/* Verifies a "$...*HH" NMEA line's XOR checksum. Returns true on match.
 * The trailing CR/LF (if any) must already be stripped. */
bool idl0_nmea_verify_checksum(const char *line);

/* Parses an RMC sentence into the provided fix. Returns true if the
 * sentence is well-formed, checksum passes, and status='A' (active).
 * Fields filled: gps_epoch_ms, latitude_e7, longitude_e7, speed_x100,
 * heading_x100. Other fields are left untouched. */
bool idl0_nmea_parse_rmc(const char *line, idl0_gps_fix_t *fix);

/* Parses a GGA sentence. Fields filled: altitude_x10, fix_quality,
 * satellites. lat/lon are present in GGA but RMC is canonical — we
 * ignore them here. Returns true on a well-formed, checksum-valid GGA. */
bool idl0_nmea_parse_gga(const char *line, idl0_gps_fix_t *fix);

/* Feed one received NMEA line (no trailing CR/LF) to the integrator.
 * If the line completes a fix (RMC arrives with the latest GGA already
 * folded in), out_fix is populated and the function returns true. The
 * device_timestamp_us field is set from esp_timer_get_time() at the
 * moment the RMC is processed. */
bool idl0_gps_feed_line(const char *line, idl0_gps_fix_t *out_fix);
