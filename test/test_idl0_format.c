/* Host-compiled unit test for idl0_format.c.
 *
 * Build & run (from firmware/test/):
 *   cc -std=c99 -I../main test_idl0_format.c ../main/idl0_format.c -o test_idl0_format
 *   ./test_idl0_format
 *
 * idl0_format.c has no ESP-IDF dependency, so any host C compiler works.
 * Tests cover schema v3 (40-byte registry entries, schema byte = 3).
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "idl0_format.h"

static int g_checks = 0;
#define CHECK(cond) do { g_checks++; if (!(cond)) { \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); return 1; } } while (0)

/* Read a little-endian u16 from a buffer. */
static uint16_t rd_u16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/* Read a little-endian u32 from a buffer. */
static uint32_t rd_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Read a little-endian f32 (IEEE 754) from a buffer. */
static float rd_f32(const uint8_t *p) {
    uint32_t bits = rd_u32(p);
    float v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

/* Read a little-endian i16. */
static int16_t rd_i16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int test_crc32(void) {
    /* CRC-32/ISO-HDLC of ASCII "123456789" is the standard 0xCBF43926. */
    const uint8_t check[] = "123456789";
    CHECK(idl0_format_crc32(check, 9) == 0xCBF43926u);
    CHECK(idl0_format_crc32((const uint8_t *)"", 0) == 0x00000000u);
    return 0;
}

static int test_session_end(void) {
    uint8_t buf[8] = {0xAA, 0xAA, 0xAA, 0xAA};
    size_t n = idl0_format_session_end(buf);
    CHECK(n == 3);
    CHECK(buf[0] == 0xFF);
    CHECK(buf[1] == 0x00 && buf[2] == 0x00);
    return 0;
}

static int test_header(void) {
    uint8_t uuid[16];
    uint8_t devid[6];
    for (int i = 0; i < 16; i++) uuid[i]  = (uint8_t)(0x10 + i);
    for (int i = 0; i < 6;  i++) devid[i] = (uint8_t)(0xA0 + i);

    uint8_t buf[128];
    size_t n = idl0_format_header(buf, sizeof(buf), uuid, devid,
                                  0x1122334455667788LL, 0xDEADC0DEu,
                                  0x0003FFFFu, 3, 104, 5, 0, NULL);

    /* Fixed part = 4+1+16+6+8+4+4+1+2+1+1 = 48, + 0 registry + 4 marker. */
    CHECK(n == 52);
    CHECK(memcmp(buf, "IDL0", 4) == 0);
    CHECK(buf[4] == 3);                              /* schema version = 3 */
    CHECK(memcmp(buf + 5, uuid, 16) == 0);           /* session UUID   */
    CHECK(memcmp(buf + 21, devid, 6) == 0);          /* device ID      */
    CHECK(rd_u32(buf + 35) == 0xDEADC0DEu);          /* config CRC32   */
    CHECK(rd_u32(buf + 39) == 0x0003FFFFu);          /* IMU mask       */
    CHECK(buf[43] == 3);                             /* IMU count      */
    CHECK(buf[47] == 0);                             /* registry count */
    CHECK(rd_u32(buf + 48) == 0xDEADBEEFu);          /* end marker     */
    return 0;
}

static int test_header_registry(void) {
    /* Build a header with two 40-byte registry entries and verify:
     * - total size = 48 + 2*40 + 4 = 132 bytes
     * - schema byte = 3
     * - each entry laid out in §5.2 order:
     *   channel_id(1), data_type(1), sample_rate_hz(2), scale(4), offset(4),
     *   name(20), units(8) = 40 bytes per entry.
     * Entry 0: IMU0_AccelX, i16, 104 Hz, scale=32/32768, offset=0, "g"
     * Entry 1: IMU0_GyroX,  i16, 104 Hz, scale=2000/32768, offset=0, "dps" */
    uint8_t uuid[16] = {0};
    uint8_t devid[6] = {0};

    idl0_channel_registry_entry_t reg[2];
    memset(reg, 0, sizeof(reg));

    reg[0].channel_id     = 0;
    reg[0].data_type      = 4;   /* i16 */
    reg[0].sample_rate_hz = 104;
    reg[0].scale          = 32.0f / 32768.0f;
    reg[0].offset         = 0.0f;
    strncpy((char *)reg[0].name,  "IMU0_AccelX", sizeof(reg[0].name)  - 1);
    strncpy((char *)reg[0].units, "g",           sizeof(reg[0].units) - 1);

    reg[1].channel_id     = 3;
    reg[1].data_type      = 4;   /* i16 */
    reg[1].sample_rate_hz = 104;
    reg[1].scale          = 2000.0f / 32768.0f;
    reg[1].offset         = 0.0f;
    strncpy((char *)reg[1].name,  "IMU0_GyroX", sizeof(reg[1].name)  - 1);
    strncpy((char *)reg[1].units, "dps",        sizeof(reg[1].units) - 1);

    /* 48 + 2*40 + 4 = 132 */
    uint8_t buf[200];
    size_t n = idl0_format_header(buf, sizeof(buf), uuid, devid,
                                  0LL, 0u, 0x0003FFFFu, 3, 104, 5, 2, reg);
    CHECK(n == 132);
    CHECK(buf[4] == 3);   /* schema version */
    CHECK(buf[47] == 2);  /* registry count */

    /* Entry 0 starts at byte 48. */
    const uint8_t *e0 = buf + 48;
    CHECK(e0[0] == 0);                                 /* channel_id */
    CHECK(e0[1] == 4);                                 /* data_type = i16 */
    CHECK(rd_u16(e0 + 2) == 104);                      /* sample_rate_hz */
    /* scale = 32/32768 = 0.0009765625; offset = 0. */
    CHECK(rd_f32(e0 + 4) == 32.0f / 32768.0f);        /* scale */
    CHECK(rd_f32(e0 + 8) == 0.0f);                    /* offset */
    CHECK(strncmp((const char *)(e0 + 12), "IMU0_AccelX", 11) == 0); /* name */
    CHECK((e0 + 12)[11] == 0);                         /* null-terminated */
    CHECK(strncmp((const char *)(e0 + 32), "g", 1) == 0); /* units */

    /* Entry 1 starts at byte 48 + 40 = 88. */
    const uint8_t *e1 = buf + 88;
    CHECK(e1[0] == 3);                                 /* channel_id */
    CHECK(e1[1] == 4);                                 /* data_type = i16 */
    CHECK(rd_u16(e1 + 2) == 104);                      /* sample_rate_hz */
    CHECK(rd_f32(e1 + 4) == 2000.0f / 32768.0f);      /* scale */
    CHECK(rd_f32(e1 + 8) == 0.0f);                    /* offset */
    CHECK(strncmp((const char *)(e1 + 12), "IMU0_GyroX", 10) == 0); /* name */
    CHECK(strncmp((const char *)(e1 + 32), "dps", 3) == 0);         /* units */

    /* End marker at byte 128. */
    CHECK(rd_u32(buf + 128) == 0xDEADBEEFu);
    return 0;
}

static int test_imu_sample(void) {
    int16_t axes[6] = { 100, -200, 300, -400, 500, -600 };
    uint8_t buf[24];

    /* IMU0, mask 0x07 = accel XYZ only. Payload = 1 + 8 + 3*2 = 15. */
    size_t n = idl0_format_imu_sample(buf, 0, 0x0123456789ABCDEFLL, 0x07u, axes);
    CHECK(n == 18);                          /* 3 framing + 15 payload */
    CHECK(buf[0] == 0x01);
    CHECK(((uint16_t)buf[1] | ((uint16_t)buf[2] << 8)) == 15);
    CHECK(buf[3] == 0);                      /* imu_index */
    CHECK(rd_i16(buf + 12) == 100);          /* accel_x at 3+1+8 */
    CHECK(rd_i16(buf + 14) == -200);         /* accel_y */
    CHECK(rd_i16(buf + 16) == 300);          /* accel_z */

    /* IMU1 (mask bits 6-11), only gyro_x enabled -> bit 9 set. */
    n = idl0_format_imu_sample(buf, 1, 0LL, 0x200u, axes);
    CHECK(n == 12);                          /* 3 + 1 + 8 + 2 */
    CHECK(buf[3] == 1);                      /* imu_index */
    CHECK(rd_i16(buf + 12) == -400);         /* axes[3] = gyro_x */
    return 0;
}

static int test_gps_fix(void) {
    uint8_t buf[40];
    size_t n = idl0_format_gps_fix(buf,
                                   1700000000000LL,  /* gps_epoch_ms */
                                   42000000LL,       /* device_timestamp_us */
                                   515000000,        /* lat e7 */
                                   -1000000,         /* lon e7 */
                                   1234,             /* alt x10 */
                                   8800,             /* speed x100 */
                                   18000,            /* heading x100 */
                                   1, 9);            /* fix_quality, sats */
    CHECK(n == 35);                                  /* 3 framing + 32 payload */
    CHECK(buf[0] == 0x02);
    CHECK(((uint16_t)buf[1] | ((uint16_t)buf[2] << 8)) == 32);
    CHECK(rd_u32(buf + 19) == (uint32_t)515000000);  /* lat at 3+8+8 */
    CHECK((int32_t)rd_u32(buf + 23) == -1000000);    /* lon */
    CHECK(rd_i16(buf + 27) == 1234);                 /* alt */
    CHECK(((uint16_t)buf[29] | ((uint16_t)buf[30] << 8)) == 8800);   /* speed */
    CHECK(((uint16_t)buf[31] | ((uint16_t)buf[32] << 8)) == 18000);  /* heading */
    CHECK(buf[33] == 1);                             /* fix_quality */
    CHECK(buf[34] == 9);                             /* satellites  */
    return 0;
}

static int test_channel_sample(void) {
    uint8_t buf[24];
    uint32_t value = 0x11223344u;
    /* channel_id 0, u32 value -> payload = 1 + 8 + 4 = 13. */
    size_t n = idl0_format_channel_sample(buf, 0, 0x0102030405060708LL,
                                          &value, sizeof(value));
    CHECK(n == 16);
    CHECK(buf[0] == 0x03);
    CHECK(((uint16_t)buf[1] | ((uint16_t)buf[2] << 8)) == 13);
    CHECK(buf[3] == 0);                       /* channel_id */
    CHECK(rd_u32(buf + 12) == 0x11223344u);   /* value at 3+1+8 */
    return 0;
}

int main(void) {
    /* The encoders assume a little-endian host. */
    uint32_t probe = 1;
    assert(*(const uint8_t *)&probe == 1 && "little-endian host required");

    if (test_crc32())             return 1;
    if (test_session_end())       return 1;
    if (test_header())            return 1;
    if (test_header_registry())   return 1;
    if (test_imu_sample())        return 1;
    if (test_gps_fix())           return 1;
    if (test_channel_sample())    return 1;

    printf("PASS — %d checks\n", g_checks);
    return 0;
}
