/* On-SD diagnostic log (idl0_debug.log).
 *
 * A lightweight, append-only diagnostics file written to the SD card root to
 * help diagnose long-session WiFi/BLE/heap degradation that only shows up over
 * time on real hardware (where a serial monitor isn't attached). It records:
 *   - a BOOT marker per power-up, including esp_reset_reason() — a
 *     ESP_RST_BROWNOUT line is the tell-tale of battery sag; a clean
 *     ESP_RST_POWERON/SW rules it out.
 *   - periodic heap samples (free / min-ever / largest-free-block) so a slow
 *     heap or fragmentation leak shows up as a declining trend.
 *   - explicit event lines (WiFi up/down, BLE suspend/resume) so heap drops
 *     can be correlated with radio cycles.
 *
 * Non-interference guarantees (§1: during a logging session the firmware does
 * raw capture only): the periodic sampler SKIPS while a session is running, so
 * it never contends with session SD writes. Writes are serialised by a mutex,
 * the file is size-capped (rotated, not grown unbounded), and the whole thing
 * is a no-op when no SD card is mounted.
 */

#pragma once

/* Start the periodic diagnostic sampler task and append the boot marker.
 * Safe to call once from app_main after idl0_sd_init(). No-op without SD. */
void idl0_diag_log_init(void);

/* Append one timestamped event line, e.g. idl0_diag_log_event("wifi up").
 * Safe from any task; serialised internally. No-op without SD / before init. */
void idl0_diag_log_event(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
