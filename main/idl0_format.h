/* IDL0 v3 binary format encoders.
 *
 * Pure byte-builders for the on-disk format. No I/O — the caller
 * (sd_logger / session) decides where bytes go.
 *
 * See IDL0_SPEC.md §5 for the byte-level contract (spec location: docs/README.md).
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

/* Schema version emitted in the file header. */
#define IDL0_SCHEMA_VERSION 3

/* Size of one channel registry entry in bytes (§5.2):
 * channel_id(1) + data_type(1) + sample_rate_hz(2) + scale(4) + offset(4)
 * + name(20) + units(8) = 40. */
#define IDL0_REGISTRY_ENTRY_BYTES 40

/* One entry in the channel registry passed to idl0_format_header().
 * Fields are written to the file in the order they appear here, matching
 * the §5.2 table. */
typedef struct {
    uint8_t  channel_id;
    uint8_t  data_type;       /* 0=u8 1=u16 2=u32 3=i8 4=i16 5=i32 6=f32 7=f64 */
    uint16_t sample_rate_hz;  /* 0 = event-driven */
    float    scale;           /* physical = stored × scale + offset */
    float    offset;
    uint8_t  name[20];        /* null-terminated ASCII */
    uint8_t  units[8];        /* null-terminated ASCII */
} idl0_channel_registry_entry_t;

/* Header construction. Returns bytes written; out_buf must be at least
 * 48 + IDL0_REGISTRY_ENTRY_BYTES * channel_count + 4 bytes. */
size_t idl0_format_header(uint8_t *out_buf,
                          size_t out_len,
                          const uint8_t session_uuid[16],
                          const uint8_t device_id[6],
                          int64_t session_start_utc_ms,
                          uint32_t config_crc32,
                          uint32_t imu_channel_mask,
                          uint8_t imu_count,
                          uint16_t imu_sample_rate_hz,
                          uint8_t gps_sample_rate_hz,
                          uint8_t channel_registry_count,
                          const idl0_channel_registry_entry_t *channel_registry);

/* IMU_SAMPLE (0x01) — variable stride per channel mask.
 * out_buf must be at least 24 bytes (3 framing + 21 payload max). */
size_t idl0_format_imu_sample(uint8_t *out_buf,
                              uint8_t imu_index,
                              int64_t timestamp_us,
                              uint32_t imu_channel_mask,
                              const int16_t axes[6]);  /* indexed by mask bit position 0..5 */

/* GPS_FIX (0x02) — always 35 bytes total (3 framing + 32 payload). */
size_t idl0_format_gps_fix(uint8_t *out_buf,
                           int64_t gps_epoch_ms,
                           int64_t device_timestamp_us,
                           int32_t lat_e7,
                           int32_t lon_e7,
                           int16_t alt_x10,
                           uint16_t speed_x100,
                           uint16_t heading_x100,
                           uint8_t fix_quality,
                           uint8_t satellites);

/* CHANNEL_SAMPLE (0x03). value_len is 1..8 bytes per registry data_type. */
size_t idl0_format_channel_sample(uint8_t *out_buf,
                                  uint8_t channel_id,
                                  int64_t timestamp_us,
                                  const void *value,
                                  size_t value_len);

/* SESSION_END (0xFF) — always 3 bytes total. */
size_t idl0_format_session_end(uint8_t *out_buf);

/* CRC-32/ISO-HDLC over arbitrary bytes (for the header config_crc32 field).
 * Portable software implementation (poly 0xEDB88320, reflected, init/xorout
 * 0xFFFFFFFF) — bit-identical to esp_rom_crc32_le and Dart's crclib Crc32.
 * Kept dependency-free so idl0_format.c stays host-testable. */
uint32_t idl0_format_crc32(const uint8_t *buf, size_t len);
