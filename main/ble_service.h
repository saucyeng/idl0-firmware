/* NimBLE GATT service for the IDL0 device control plane.
 *
 * Service FF, FF03 (write+rsp, control), FF04 (notify, status).
 * See §7 of docs/IDL0_SPEC.md for the wire contract.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    IDL0_CMD_WIFI_ON        = 0x01,
    IDL0_CMD_WIFI_OFF       = 0x02,
    IDL0_CMD_START_LOGGING  = 0x03,
    IDL0_CMD_STOP_LOGGING   = 0x04,
    IDL0_CMD_CALIBRATE_IMU  = 0x05,
    IDL0_CMD_OTA_CONFIRM    = 0x06,
    /* BLE config push (§7.2). CONFIG_BEGIN resets the FF05 reassembly
     * buffer; the app then streams the JSON in chunks to FF05; CONFIG_COMMIT
     * validates + atomically writes idl0_config.json and reboots to apply
     * (config is read at boot only, §4.2). These are handled inside the BLE
     * layer, not routed to the command callback. */
    IDL0_CMD_CONFIG_BEGIN   = 0x07,
    IDL0_CMD_CONFIG_COMMIT  = 0x08,
    /* BLE config read-back (§7.2). Snapshots idl0_config.json into a TX buffer
     * and resets the FF06 read cursor; the app then reads FF06 repeatedly,
     * each read serving the next chunk until an empty read signals EOF. Used
     * to round-trip / verify a push. Handled in the BLE layer. */
    IDL0_CMD_CONFIG_READ_BEGIN = 0x09,
} idl0_ble_command_t;

/* Returns the ATT result code that the GATT write response will carry:
 *   0x00 — success (command accepted and dispatched)
 *   0x03 — WRITE_NOT_PERMITTED (mutex / precondition refusal)
 *   0x80 — IDL0_ACK_BUSY (reserved)
 *   0x81 — IDL0_ACK_PRECONDITION (reserved)
 *   0x82 — IDL0_ACK_NOT_IMPLEMENTED (reserved)
 */
typedef uint8_t (*idl0_ble_command_cb)(idl0_ble_command_t cmd);

#define IDL0_ACK_OK               0x00
#define IDL0_ACK_MUTEX_REFUSED    0x03  /* BLE: WRITE_NOT_PERMITTED */
#define IDL0_ACK_BUSY             0x80  /* reserved */
#define IDL0_ACK_PRECONDITION     0x81  /* reserved */
#define IDL0_ACK_NOT_IMPLEMENTED  0x82  /* reserved */

bool idl0_ble_init(idl0_ble_command_cb on_command);

/* Push a status update to the FF04 characteristic. String is built
 * by the caller per §7.3 (newline-delimited: WiFi:, Logging:, Battery:). */
void idl0_ble_publish_status(const char *status_string);

/* §10.4 radio handoff. While suspended, the peripheral neither advertises
 * nor accepts connections — WiFi owns the radio. The NimBLE host keeps
 * running; only the GAP layer is quiesced. idl0_wifi_stop() resumes. */
void idl0_ble_suspend(void);

/* Re-starts advertising after a suspend. Safe to call when not
 * suspended (no-op). Called from idl0_wifi_stop() on every WiFi-mode
 * exit path (BLE command, POST /wifi_off, no-activity failsafe). */
void idl0_ble_resume(void);

/* True while the §10.4 radio handoff has BLE suspended. */
bool idl0_ble_is_suspended(void);
