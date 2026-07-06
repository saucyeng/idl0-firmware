/* Single source of truth for "what mode is the firmware in".
 * Producers: wifi_server.c (WIFI_UP bit), session.c (LOGGING_ACTIVE bit).
 * Subscribers wait on bit changes via xEventGroupWaitBits / xEventGroupGetBits;
 * no per-subsystem callbacks.
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define IDL0_MODE_BIT_WIFI_UP         BIT0
#define IDL0_MODE_BIT_LOGGING_ACTIVE  BIT1

/* Returns the global mode event group, lazily creating it on first call.
 * Safe to call from any task. Never returns NULL after first call. */
EventGroupHandle_t idl0_mode_event_group(void);

/* Atomically sets the given bits in the global mode event group. */
void idl0_mode_set_bits(EventBits_t bits);

/* Atomically clears the given bits in the global mode event group. */
void idl0_mode_clear_bits(EventBits_t bits);

/* Returns a snapshot of the global mode event group's current bits. */
EventBits_t idl0_mode_get_bits(void);
