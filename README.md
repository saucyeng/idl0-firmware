# idl0 — firmware

ESP32-C6 firmware for the IDL0 mountain-bike data logger. During a logging
session it does raw sensor capture only — sensor bytes straight to the SD card,
no processing. Everything else (BLE/WiFi control, OTA update, calibration, file
transfer) runs outside the logging path.

Part of the three-repo IDL0 system: this firmware, the [idl-rs](https://github.com/saucyeng/idl-rs)
processing engine, and the companion app.

## Build & flash (ESP-IDF v6.0.1, target esp32c6)

```
idf.py set-target esp32c6
idf.py -p /dev/ttyACM0 flash monitor    # first flash over USB
```

The **first** flash must be over USB — it lays down the bootloader, partition
table, and first slot. Every update after that ships over-the-air: tag a release
(`git tag v1.5.0 && git push --tags`) and the app installs it from GitHub Releases.

## License

GPL-3.0-or-later. See [LICENSE](LICENSE). (GPLv3, not v2, for compatibility with
ESP-IDF's Apache-2.0.) Vendored components under `components/` and
`managed_components/` keep their own licenses. Contributions require the CLA.
