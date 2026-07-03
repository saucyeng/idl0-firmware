/* IDL0 firmware entry point.
 *
 * P4 milestone: BLE control plane up. Boots, brings up the status LED
 * and the NimBLE GATT service, and dispatches received commands to a
 * handler that logs them and reflects logging state on the LED. Real
 * command behaviour (SD, sensors) arrives in P5+.
 */

#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_service.h"
#include "device_id.h"
#include "diag_log.h"
#include "driver/gpio.h"
#include "gps_task.h"
#include "hrm_task.h"
#include "idl0_config.h"
#include "imu_task.h"
#include "led_status.h"
#include "ota.h"
#include "pins.h"
#include "sd_logger.h"
#include "writer_task.h"
#include "status.h"
#include "session.h"
#include "wifi_server.h"

static const char *TAG = "idl0";

/* Invoked from the BLE host task when a control command is written. */
static uint8_t on_ble_command(idl0_ble_command_t cmd)
{
    uint8_t ack = IDL0_ACK_OK;
    switch (cmd) {
        case IDL0_CMD_START_LOGGING:
            ESP_LOGI(TAG, "command: START_LOGGING");
            ack = idl0_session_on_ble_command(cmd);
            break;
        case IDL0_CMD_STOP_LOGGING:
            ESP_LOGI(TAG, "command: STOP_LOGGING");
            ack = idl0_session_on_ble_command(cmd);
            break;
        case IDL0_CMD_WIFI_ON:
            ESP_LOGI(TAG, "command: WIFI_ON");
            if (idl0_session_is_running()) {
                ESP_LOGW(TAG, "WIFI_ON refused: session running (mutex \302\2477.2)");
                ack = IDL0_ACK_MUTEX_REFUSED;
                break;
            }
            idl0_wifi_start();
            break;
        case IDL0_CMD_WIFI_OFF:
            ESP_LOGI(TAG, "command: WIFI_OFF");
            idl0_wifi_stop();
            break;
        case IDL0_CMD_CALIBRATE_IMU:
            ESP_LOGI(TAG, "command: CALIBRATE_IMU (no-op until IMU bring-up)");
            break;
        case IDL0_CMD_OTA_CONFIRM:
            ESP_LOGI(TAG, "command: OTA_CONFIRM");
            if (!idl0_ota_mark_valid()) {
                ESP_LOGW(TAG, "OTA_CONFIRM with no pending-verify image");
            }
            break;
        case IDL0_CMD_CONFIG_BEGIN:
        case IDL0_CMD_CONFIG_COMMIT:
        case IDL0_CMD_CONFIG_READ_BEGIN:
            /* Config push/read handshakes are handled entirely in the BLE
             * layer (it owns the FF05/FF06 buffers) and never routed here.
             * Defensive: if one ever reaches the dispatcher, refuse cleanly. */
            ESP_LOGW(TAG, "config command 0x%02X reached dispatcher", cmd);
            ack = IDL0_ACK_NOT_IMPLEMENTED;
            break;
    }
    idl0_status_publish();
    return ack;
}

void app_main(void)
{
    ESP_LOGI(TAG, "IDL0 firmware boot");

    idl0_device_id_init();

    /* Latch the running OTA partition's pending-verify state before any
     * other subsystem starts. Do NOT auto-mark-valid — the app must
     * send CMD_OTA_CONFIRM after it has verified the device is healthy.
     * If we never get that confirmation and reboot, the bootloader
     * rolls back to the previous slot. */
    idl0_ota_init();

    if (!idl0_led_init()) {
        ESP_LOGE(TAG, "LED init failed; halting");
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    idl0_led_set(IDL0_LED_BLINK_SLOW);

    /* Idle every SPI peripheral's CS line BEFORE bringing up the shared
     * SPI bus. Without this, IMU1 / IMU2 CS pins (GPIO 21 / 22) sit
     * high-Z at reset and the LSM6DSO32 chips read their floating CS as
     * asserted, then drive MISO during SD card init — corrupting the
     * SCR read and failing the mount with ESP_ERR_INVALID_RESPONSE.
     * IMU0's CS is later re-claimed by the spi_master driver via
     * spics_io_num; explicit pre-idling is harmless. */
    static const gpio_num_t cs_pins[] = {
        IDL0_PIN_CS_IMU0, IDL0_PIN_CS_IMU1, IDL0_PIN_CS_IMU2,
    };
    for (size_t i = 0; i < sizeof(cs_pins) / sizeof(cs_pins[0]); i++) {
        gpio_reset_pin(cs_pins[i]);
        gpio_set_direction(cs_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(cs_pins[i], 1);   /* deasserted */
    }

    if (!idl0_sd_init(SPI2_HOST)) {
        ESP_LOGW(TAG, "SD card not available at boot");
    }

    if (!idl0_writer_init()) {
        ESP_LOGE(TAG, "writer init failed; logging will not work");
    }

    if (!idl0_imu_task_start()) {
        ESP_LOGW(TAG, "IMU unavailable — sessions will record without IMU");
    }

    if (!idl0_gps_task_start()) {
        ESP_LOGW(TAG, "GPS unavailable — sessions will record without GPS");
    }

    if (!idl0_ble_init(on_ble_command)) {
        ESP_LOGE(TAG, "BLE init failed; LED only");
    } else {
        ESP_LOGI(TAG, "BLE control plane up as %s", idl0_device_name());
        idl0_status_publish();
        idl0_status_publisher_start();
    }

    /* HRM task picks up the saved address from idl0_config.json. NimBLE
     * is already initialised by idl0_ble_init above; the central role
     * uses the same host instance. No-op if HRM is disabled. */
    {
        idl0_config_t cfg;
        idl0_config_load(&cfg);
        hrm_task_start(&cfg.hrm);
    }

    /* On-SD diagnostics (heap trend + brownout-reset trail) to chase the
     * long-session WiFi degradation. No-op without SD; skips itself during a
     * logging session so it never contends with session writes. */
    idl0_diag_log_init();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
