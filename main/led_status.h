/* Status LED helper.
 *
 * Wraps the one onboard LED (IDL0_PIN_LED, see pins.h — active-high).
 * Patterns are 1-byte enums; the caller picks one and the module
 * manages timing internally via a FreeRTOS software timer.
 */

#pragma once

#include <stdbool.h>

typedef enum {
    IDL0_LED_OFF,
    IDL0_LED_ON,
    IDL0_LED_BLINK_SLOW,   /* 1 Hz, 50% duty — idle */
    IDL0_LED_BLINK_FAST,   /* 4 Hz, 50% duty — logging */
} idl0_led_pattern_t;

bool idl0_led_init(void);
void idl0_led_set(idl0_led_pattern_t pattern);
