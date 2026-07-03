/* Debounced GPIO marker reader.
 *
 * Reads `config.digital.channels[]` (§8) entries with kind=`marker` and
 * registers a GPIO ISR per pin. Each accepted (post-debounce) press is
 * timestamped at the ISR and written to the standard writer pipeline as
 * a CHANNEL_SAMPLE (§5.7) on the channel id assigned by session.c at
 * session start. Value = monotonic press counter (u8, wraps at 255).
 *
 * Spec 1 ships marker only — DigitalKind.level / .pwm are reserved in the
 * schema but not yet implemented here.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DIGITAL_MAX_CHANNELS 4

typedef struct {
    bool     enabled;
    int      gpio_pin;
    bool     active_low;
    uint16_t debounce_ms;
    char     label[20];        /* matches channel registry name field */
    uint8_t  channel_id;       /* assigned at session start */
} digital_channel_cfg_t;

/* Start the GPIO ISR-backed marker reader. Idempotent — calling again
 * with a new config replaces the active channel set. Pass NULL/0 to stop. */
void digital_task_start(const digital_channel_cfg_t *channels, size_t count);

/* Stop the task and detach all installed ISRs. */
void digital_task_stop(void);
