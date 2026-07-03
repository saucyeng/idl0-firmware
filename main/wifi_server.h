/* WiFi AP mode + HTTP server for on-demand file transfer and config push.
 *
 * SSID derived from the MAC per §3.6 (idl0_device_name → "IDL0-XXXX").
 * Password is the §6 shared default until per-device passwords land
 * (§6 TODO #16). See §6.1 for endpoint contracts. WiFi is off by
 * default; CMD_WIFI_ON (§7.2) starts the AP + HTTP server, CMD_WIFI_OFF
 * stops them.
 */

#pragma once

#include <stdbool.h>

/* WiFi control-protocol version reported by GET /ping (SPEC §6.1).
 * Bump on breaking changes to the HTTP control plane; the app refuses
 * operations on a major mismatch. */
#define IDL0_WIFI_PROTO_VERSION 1

/* WiFi subsystem state, mapped 1:1 onto the §7.3 WiFi: status token. */
typedef enum {
    IDL0_WIFI_OFF = 0,  /* AP not running */
    IDL0_WIFI_ON,       /* AP up + HTTP server listening */
} idl0_wifi_state_t;

idl0_wifi_state_t idl0_wifi_state(void);
const char *idl0_wifi_state_str(idl0_wifi_state_t state);

/* Bring up the AP and start the HTTP server. Idempotent — repeated
 * calls while already up return true without restarting the radio. */
bool idl0_wifi_start(void);

/* Stop the HTTP server and the AP. Idempotent. */
bool idl0_wifi_stop(void);
