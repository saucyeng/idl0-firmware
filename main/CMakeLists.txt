# IDL0 firmware main component.
# Each module is added as a SRCS entry as it lands.

idf_component_register(
    SRCS "main.c"
         "led_status.c"
         "device_id.c"
         "ble_service.c"
         "sd_logger.c"
         "idl0_format.c"
         "idl0_config.c"
         "session.c"
         "writer_task.c"
         "status.c"
         "gps_driver.c"
         "gps_task.c"
         "imu_driver.c"
         "imu_driver_i2c.c"
         "imu_task.c"
         "wifi_server.c"
         "ota.c"
         "hrm_task.c"
         "digital_task.c"
         "mode_state.c"
         "diag_log.c"
    INCLUDE_DIRS "."
    REQUIRES
        # SD card writer
        fatfs sdmmc esp_driver_sdspi
        # GPIO (LED, buttons) + UART (GPS)
        esp_driver_gpio esp_driver_uart
        # TEMP: I²C master driver for external IMUs (imu_driver_i2c.c)
        esp_driver_i2c
        # esp_timer_get_time (session), esp_random / esp_mac (device_id)
        esp_timer esp_hw_support
        # NimBLE (ble_service.c) and its NVS dependency
        bt nvs_flash
        # FreeRTOS ring buffer (writer_task.c — freertos/ringbuf.h)
        esp_ringbuf
        # Vendored ST LSM6DSO32 driver (firmware/components/lsm6dso32x_STdC, pinned to v2.3.0)
        lsm6dso32x_STdC
        # WiFi AP + HTTP server (wifi_server.c)
        esp_wifi esp_netif esp_event esp_http_server
        # OTA write + boot partition control (ota.c)
        app_update
)
