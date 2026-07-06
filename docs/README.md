# docs/ — spec pointer

This repo hosts no local copy of the IDL0 spec. That is a deliberate,
monospec decision made at the firmware/app repo split: one spec, one
source of truth, in one place.

**Canonical spec:**
https://github.com/saucyeng/idl0-app/blob/main/docs/IDL0_SPEC.md

Spec changes are made in `idl0-app`, never here. If a firmware task needs
a spec change, propose it as a PR against `idl0-app` first (see this
repo's `CLAUDE.md` §1).

## Section map — what this firmware implements

Source comments in this repo cite spec sections as `IDL0_SPEC.md §N`
(numbers only, no local path) and point back here for the spec's
location. Use this table to jump from a section number to the
subsystem that implements it:

| Spec section | Subsystem | Where |
|---|---|---|
| §1 | System philosophy — zero processing while logging | applies repo-wide; see `CLAUDE.md` §2 |
| §4 | Firmware architecture | `CMakeLists.txt`, overall `main/` layout |
| §4.6 | Partitions, OTA, rollback | `partitions.csv`, `sdkconfig.defaults` (bootloader rollback options) |
| §5 | Binary log format (`.idl0`) | `main/idl0_format.c`, `main/idl0_format.h` |
| §5.1 | Config CRC32 | `main/idl0_config.c` |
| §5.2 | HRM channel records | `main/hrm_task.h` |
| §5.4 | IMU channel mask | `main/idl0_config.c` |
| §6 | WiFi endpoints | `main/wifi_server.c` |
| §6.1 | `POST /ota` | `main/wifi_server.c` |
| §7 | BLE GATT contract | `main/ble_service.c`, `main/ble_service.h` |
| §7.2 | BLE commands / acks (`IDL0_ACK_*`) | `main/ble_service.h` |
| §7.3 | BLE status text | `main/status.c` |
| §7.5 | HRM over BLE | `main/hrm_task.h` |
| §8 | Configuration schema (`idl0_config.json`) | `main/idl0_config.c` |
| §10 | Device behavior | `main/sd_logger.c` (§10.2 SD layout), `main/hrm_task.h` (§10.4) |

If a source comment's `§N` and this table disagree, the spec (linked
above) is canonical — file an issue/PR rather than trusting either
local copy blindly.

## Release process

The git tag is the version of record:

1. Push a tag matching `v*` (e.g. `git tag v1.5.0 && git push --tags`).
2. `.github/workflows/firmware-release.yml` builds the firmware
   (ESP-IDF v6.0.1, target `esp32c6`) and publishes a GitHub Release
   with the `.bin` asset and its `.sha256` checksum sidecar.
3. The tag (leading `v` stripped) is written to `version.txt` at build
   time, which the ESP-IDF build bakes into `esp_app_desc_t.version` —
   surfaced to the app via `GET /ping`'s `fw` field and the BLE §7.3
   `Firmware: %s` status line.
4. A tag containing a `-` (e.g. `v1.5.0-beta.1`) publishes as a GitHub
   **prerelease**; a plain `vX.Y.Z` tag publishes as a stable release.

See this repo's `CLAUDE.md` §3 for the full versioning/OTA contract,
including the bootloader-level Kconfig caveat (rollback options require
a USB re-flash and never take effect over OTA).
