/* BLE Heart Rate Monitor task (NimBLE central role).
 *
 * Connects to a standards-compliant HRM strap (Heart Rate Service 0x180D)
 * identified by `heart_rate_monitor.device_address` in idl0_config.json
 * (§8). Subscribes to the Heart Rate Measurement characteristic and reads
 * the Battery Level characteristic once on connect; each notification is
 * decoded into channel 22 (HR_BPM) and channel 23 (HR_RR) CHANNEL_SAMPLE
 * records via the standard writer pipeline.
 *
 * See IDL0_SPEC.md §5.2, §7.5, §10.4 (spec location: docs/README.md).
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HRM_ADDRESS_LEN 6

typedef struct {
    bool    enabled;
    /* BLE address stored in NimBLE wire order — LSB first. Parsing
     * `AA:BB:CC:DD:EE:FF` yields address[0]=0xFF, address[5]=0xAA so
     * memcmp against `event->disc.addr.val` works directly. */
    uint8_t address[HRM_ADDRESS_LEN];
    char    name[24];                    /* for log lines, optional */
    uint8_t hr_channel_id;               /* fixed at 22 (§5.2) */
    uint8_t rr_channel_id;               /* fixed at 23 (§5.2) */
} hrm_config_t;

typedef enum {
    HRM_STATE_OFF,
    HRM_STATE_SCANNING,
    HRM_STATE_CONNECTING,
    HRM_STATE_DISCOVERING,
    HRM_STATE_STREAMING,
    HRM_STATE_SUSPENDED,        /* WiFi on; HRM dropped intentionally (§10.4). */
} hrm_state_t;

/* Snapshot the config into the task and start the FreeRTOS task + event
 * queue. No-op if `config->enabled` is false. Safe to call once at boot
 * after idl0_config_load. */
void hrm_task_start(const hrm_config_t *config);

/* Stop the task and release its queue. Currently called only on shutdown;
 * config hot-reload would call this then hrm_task_start again. */
void hrm_task_stop(void);

/* Accessors for status.c (§7.3 HR / HR_Battery lines). */
hrm_state_t hrm_task_state(void);
uint8_t     hrm_task_latest_bpm(void);
bool        hrm_task_no_contact(void);
uint8_t     hrm_task_battery_pct(void);
bool        hrm_task_has_battery(void);
