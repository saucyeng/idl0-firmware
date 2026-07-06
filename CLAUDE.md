# CLAUDE.md — IDL0 Firmware Standing Orders

This repo builds the ESP32-C6 firmware only. It has no local spec copy —
read `docs/README.md` first, then follow it to the relevant spec section(s)
before touching code.

---

## 1. Before You Start

1. Read `docs/README.md` (this repo) — it maps spec sections to firmware
   subsystems.
2. Follow it to the relevant section(s) of the canonical spec,
   `docs/IDL0_SPEC.md` in **idl0-app**:
   https://github.com/saucyeng/idl0-app/blob/main/docs/IDL0_SPEC.md
3. The spec is canonical there. **Spec changes are made in idl0-app, never
   here.** If a firmware task needs a spec change, propose it as a PR against
   idl0-app first (or flag it and stop).
4. Every task declares a spec disposition out loud before writing code, same
   discipline as idl0-app's CLAUDE.md §9:
   - **spec-first** — architectural change, new binary format field, new BLE
     command/ack, new Kconfig behavior. Propose the spec text (as a diff
     against idl0-app's SPEC) before implementing.
   - **spec-during** — additive within an existing surface; note the spec
     section that already covers it.
   - **no spec change needed** — bug fix, refactor, internal cleanup. Say
     this explicitly; silence is not the same as "no spec change needed."
5. Ambiguous field type, edge-case behavior, or layer placement (device vs.
   app)? Stop and ask. Do not infer and proceed — a wrong assumption on this
   firmware costs a USB re-flash to fix at best, a bricked bootloader at
   worst.

---

## 2. Hard Constraints

- **Zero processing while logging.** During an active logging session:
  sensor bytes go straight to the SD card and nothing else runs on the
  logging path. Analysis DSP — filtering, integration, FFT — never runs on
  this device, in any mode. That work is idl-rs (app repo). Non-logging
  activity (boot, calibration, file transfer, config, OTA) may compute as
  needed.
- **Toolchain is fixed:** ESP-IDF v6.0.1, target `esp32c6`. Don't upgrade
  either without a spec-first task (CI pins the same version — see
  `.github/workflows/firmware-release.yml`).
- **`sdkconfig.defaults` is what ships.** `sdkconfig` is gitignored and
  per-developer only; CI builds from a fresh checkout, so it only ever sees
  `sdkconfig.defaults`. Any Kconfig option with a behavioral effect —
  BLE/WiFi buffer sizes, partition table, rollback, stack sizes — belongs in
  `sdkconfig.defaults` with a comment explaining *why*, or it silently
  doesn't ship. `sdkconfig`/`sdkconfig.old` in your working tree are
  scratch; don't rely on anything only present there.

---

## 3. Versioning & Release

- The git tag is the version of record. Pushing tag `v1.5.0` triggers
  `.github/workflows/firmware-release.yml`, which strips the leading `v`
  and writes `version.txt` → baked by the ESP-IDF build into
  `esp_app_desc_t.version` → surfaced to the app via `GET /ping`'s `fw`
  field (`main/wifi_server.c`) and the BLE §7.3 `Firmware: %s` status line
  (`main/status.c`).
- Tags containing a `-` (e.g. `v1.5.0-beta.1`) publish as a GitHub
  **prerelease** — the beta channel. Plain `vX.Y.Z` tags are stable
  releases.
- A local build has no `version.txt`; the reported version falls back to a
  git-describe string. Don't hand-edit `version.txt` — it's CI-generated
  and only exists in the release build (untracked in git; note it is NOT
  listed in `.gitignore`).
- First flash is always USB (`idf.py -p <port> flash monitor` — lays down
  bootloader, partition table, first OTA slot). Every subsequent update
  ships OTA via a tagged GitHub Release; see README.md's build/flash
  section for the exact commands.
- **Bootloader-level Kconfig options never take effect over OTA** — e.g.
  `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`. Changing one requires a one-time
  USB re-flash on every device already in the field; call this out
  explicitly in the PR/task notes when you touch one.

---

## 4. Host Tests

`test/` holds host-compiled unit tests for the pure-C99 parsing code (no
ESP-IDF dependency). Run them whenever you touch `idl0_format.c` or
GPS/NMEA parsing:

```
cd test
cc -std=c99 -I../main test_idl0_format.c ../main/idl0_format.c -o test_idl0_format
./test_idl0_format
```

`test_gps_nmea.c` documents expected byte-level NMEA parsing behavior but
does **not** link yet — `gps_driver.c` still mixes the pure parser with
UART/log calls. It's inspection-verified only until the parser is extracted
to `gps_nmea.c` (a follow-up task); once that lands, the build line is:

```
cc -std=c99 -I../main test_gps_nmea.c ../main/gps_nmea.c -o test_gps_nmea
```

Don't add ESP-IDF includes to anything under `test/` or to code these tests
depend on — that's what keeps them host-buildable.

---

## 5. Documentation Artifacts

`CHANGELOG.md` and `TASKS.md` live in **idl0-app**, not here — log
meaningful firmware changes there (per idl0-app CLAUDE.md §10). This repo's
own `README.md` changes only for audience-facing changes (build/flash
commands, supported target, license). Don't create a changelog or task
list in this repo.

---

## 6. Comment & Code Conventions

Mirror the app repo's conventions:

- **Units are mandatory** on every numeric value — a bare `200` is wrong,
  `200 // mV, ADC full-scale` is right.
- **`// TODO(idl0): description`** — never a bare `// TODO`.
- **Typed error handling** — return/propagate a specific error code, never
  swallow a failure silently. BLE command handling returns one of the
  `IDL0_ACK_*` codes (`main/ble_service.h`); when adding or touching an ack
  path, reference the `IDL0_ACK_*` name in the comment, not just its hex
  value (e.g. `IDL0_ACK_PRECONDITION`, not `0x81`).
- Document *why*, not just *what* — see `sdkconfig.defaults` for the house
  style of comment-per-option.

---

## 7. Key Files

| File | Purpose |
|------|---------|
| `docs/README.md` | Pointer into the canonical spec (idl0-app) + §-map for this repo. |
| `README.md` | Build & flash commands, license/CLA summary. |
| `CLA.md` | Contributor license agreement (applies repo-wide). |
| `sdkconfig.defaults` | What actually ships — see §2. |
| `.github/workflows/firmware-release.yml` | Tag → build → version.txt → GitHub Release pipeline (see §3). |
| `test/` | Host-compiled unit tests for pure-C99 parsing code (see §4). |
