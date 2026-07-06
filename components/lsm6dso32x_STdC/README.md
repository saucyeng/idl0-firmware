# lsm6dso32x_STdC (vendored)

Source: https://github.com/STMicroelectronics/lsm6dso32x-pid
Tag pulled: v2.3.0 (Feb 2026)
License: BSD-3-Clause (see ./LICENSE)

This is the platform-independent C driver provided by STMicroelectronics.
The ESP-IDF SPI adapter (`platform_read` / `platform_write` / `platform_delay`
function pointers required by `stmdev_ctx_t`) lives in `../../main/imu_driver.c`
— not here — so this folder stays a pristine copy of upstream and
can be re-synced cleanly.

When updating: re-run the `curl` commands used to vendor this driver and check the diff before committing.
