/* Device identity derived from the ESP32-C6 eFuse MAC.
 *
 * §3.6: the advertised BLE name and WiFi SSID are `IDL0-XXXX`, where
 * XXXX is the uppercase hex of the last two MAC bytes. This module
 * reads the MAC once at startup and exposes the formatted name.
 *
 * The 6-byte device ID used in the binary log header (§5.1) is exposed
 * by idl0_device_id_bytes() for the SD writer.
 */

#pragma once

#include <stdint.h>

/* Reads the eFuse MAC and formats the cached identifiers. Call once
 * early in app_main, before idl0_ble_init. */
void idl0_device_id_init(void);

/* Returns the advertised device name, e.g. "IDL0-B2C3".
 * Valid only after idl0_device_id_init(). Stable pointer to static
 * storage — safe to hold. */
const char *idl0_device_name(void);

/* Returns a pointer to the 6-byte device ID (the eFuse MAC). Stable
 * static storage. Valid only after idl0_device_id_init(). */
const uint8_t *idl0_device_id_bytes(void);
