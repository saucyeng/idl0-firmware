/* IDL0 v3 binary format encoders — pure C99, no ESP-IDF dependency.
 *
 * All multi-byte integers are little-endian (§5). The encoders write
 * bytes out explicitly, which is correct on any host — asserted by the
 * host test.
 *
 * See IDL0_SPEC.md §5 for the byte-level contract (spec location: docs/README.md).
 */

#include "idl0_format.h"

#include <string.h>

uint32_t idl0_format_crc32(const uint8_t *buf, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return ~crc;
}

/* Append helpers — advance *pos as they write. Caller guarantees room. */
static void put_u8(uint8_t *b, size_t *pos, uint8_t v)   { b[(*pos)++] = v; }
static void put_u16(uint8_t *b, size_t *pos, uint16_t v) {
    b[(*pos)++] = (uint8_t)(v & 0xFF);
    b[(*pos)++] = (uint8_t)(v >> 8);
}
static void put_u32(uint8_t *b, size_t *pos, uint32_t v) {
    for (int i = 0; i < 4; i++) b[(*pos)++] = (uint8_t)(v >> (8 * i));
}
static void put_i64(uint8_t *b, size_t *pos, int64_t v) {
    uint64_t u = (uint64_t)v;
    for (int i = 0; i < 8; i++) b[(*pos)++] = (uint8_t)(u >> (8 * i));
}
/* Write a float as 4 little-endian bytes via type-punning through uint32_t.
 * Avoids strict-aliasing UB by using memcpy (optimised away by any modern
 * compiler at -O1+). */
static void put_f32(uint8_t *b, size_t *pos, float v) {
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    put_u32(b, pos, bits);
}

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
                          const idl0_channel_registry_entry_t *channel_registry)
{
    /* Fixed header = 48 bytes; each registry entry = 40 bytes (§5.2); end
     * marker = 4 bytes.  48 + N*40 + 4. */
    size_t need = 48 + (size_t)channel_registry_count * IDL0_REGISTRY_ENTRY_BYTES + 4;
    if (out_len < need) {
        return 0;
    }
    size_t pos = 0;
    memcpy(out_buf, "IDL0", 4);              pos += 4;
    put_u8(out_buf, &pos, IDL0_SCHEMA_VERSION); /* schema version = 3 */
    memcpy(out_buf + pos, session_uuid, 16); pos += 16;
    memcpy(out_buf + pos, device_id, 6);     pos += 6;
    put_i64(out_buf, &pos, session_start_utc_ms);
    put_u32(out_buf, &pos, config_crc32);
    put_u32(out_buf, &pos, imu_channel_mask);
    put_u8(out_buf, &pos, imu_count);
    put_u16(out_buf, &pos, imu_sample_rate_hz);
    put_u8(out_buf, &pos, gps_sample_rate_hz);
    put_u8(out_buf, &pos, channel_registry_count);
    /* Write each registry entry as 40 bytes in §5.2 field order:
     * channel_id(1), data_type(1), sample_rate_hz(2), scale(4), offset(4),
     * name(20), units(8). */
    for (uint8_t k = 0; k < channel_registry_count; k++) {
        const idl0_channel_registry_entry_t *e = &channel_registry[k];
        put_u8(out_buf,  &pos, e->channel_id);
        put_u8(out_buf,  &pos, e->data_type);
        put_u16(out_buf, &pos, e->sample_rate_hz);
        put_f32(out_buf, &pos, e->scale);
        put_f32(out_buf, &pos, e->offset);
        memcpy(out_buf + pos, e->name,  20); pos += 20;
        memcpy(out_buf + pos, e->units,  8); pos +=  8;
    }
    put_u32(out_buf, &pos, 0xDEADBEEFu);     /* end marker */
    return pos;
}

size_t idl0_format_imu_sample(uint8_t *out_buf,
                              uint8_t imu_index,
                              int64_t timestamp_us,
                              uint32_t imu_channel_mask,
                              const int16_t axes[6])
{
    /* The six axis bits for this IMU live at mask offset imu_index*6. */
    uint32_t imu_bits = (imu_channel_mask >> (imu_index * 6)) & 0x3Fu;

    size_t pos = 3;  /* leave room for the 3-byte framing header */
    put_u8(out_buf, &pos, imu_index);
    put_i64(out_buf, &pos, timestamp_us);
    for (int axis = 0; axis < 6; axis++) {
        if (imu_bits & (1u << axis)) {
            put_u16(out_buf, &pos, (uint16_t)axes[axis]);
        }
    }
    size_t payload_len = pos - 3;
    out_buf[0] = 0x01;
    out_buf[1] = (uint8_t)(payload_len & 0xFF);
    out_buf[2] = (uint8_t)(payload_len >> 8);
    return pos;
}

size_t idl0_format_gps_fix(uint8_t *out_buf,
                           int64_t gps_epoch_ms,
                           int64_t device_timestamp_us,
                           int32_t lat_e7,
                           int32_t lon_e7,
                           int16_t alt_x10,
                           uint16_t speed_x100,
                           uint16_t heading_x100,
                           uint8_t fix_quality,
                           uint8_t satellites)
{
    size_t pos = 3;  /* framing header filled in last */
    put_i64(out_buf, &pos, gps_epoch_ms);
    put_i64(out_buf, &pos, device_timestamp_us);
    put_u32(out_buf, &pos, (uint32_t)lat_e7);
    put_u32(out_buf, &pos, (uint32_t)lon_e7);
    put_u16(out_buf, &pos, (uint16_t)alt_x10);
    put_u16(out_buf, &pos, speed_x100);
    put_u16(out_buf, &pos, heading_x100);
    put_u8(out_buf, &pos, fix_quality);
    put_u8(out_buf, &pos, satellites);
    out_buf[0] = 0x02;
    out_buf[1] = 32;   /* payload is always 32 bytes */
    out_buf[2] = 0;
    return pos;        /* 35 */
}

size_t idl0_format_channel_sample(uint8_t *out_buf,
                                  uint8_t channel_id,
                                  int64_t timestamp_us,
                                  const void *value,
                                  size_t value_len)
{
    size_t pos = 3;
    put_u8(out_buf, &pos, channel_id);
    put_i64(out_buf, &pos, timestamp_us);
    memcpy(out_buf + pos, value, value_len);  /* value is already LE on-wire */
    pos += value_len;
    size_t payload_len = pos - 3;
    out_buf[0] = 0x03;
    out_buf[1] = (uint8_t)(payload_len & 0xFF);
    out_buf[2] = (uint8_t)(payload_len >> 8);
    return pos;
}

size_t idl0_format_session_end(uint8_t *out_buf)
{
    out_buf[0] = 0xFF;  /* SESSION_END */
    out_buf[1] = 0x00;  /* payload_len low  */
    out_buf[2] = 0x00;  /* payload_len high */
    return 3;
}
