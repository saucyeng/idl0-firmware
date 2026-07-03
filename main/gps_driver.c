/* GPS driver — see gps_driver.h.
 *
 * P6 milestone 2: state stub only. The NMEA parser (M3) and UART task
 * (M4) populate the rest. */

#include "gps_driver.h"

#include <ctype.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_timer.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "pins.h"

static idl0_gps_state_t s_state = IDL0_GPS_ABSENT;

idl0_gps_state_t idl0_gps_state(void) { return s_state; }

void idl0_gps_force_state(idl0_gps_state_t state) { s_state = state; }

const char *idl0_gps_state_str(idl0_gps_state_t state)
{
    switch (state) {
        case IDL0_GPS_FIX:    return "FIX";
        case IDL0_GPS_NOFIX:  return "NOFIX";
        case IDL0_GPS_ABSENT:
        default:              return "ABSENT";
    }
}

/* --- Helpers (pure C; no ESP-IDF references) ---------------------- */

/* Days from 1970-01-01 for the given proleptic Gregorian date.
 * Howard Hinnant's days_from_civil — handles all valid GPS dates. */
static int64_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153u * (m > 2 ? m - 3 : m + 9) + 2u) / 5u + d - 1u;
    const unsigned doe = yoe * 365u + yoe/4u - yoe/100u + doy;
    return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

/* Convert "DDMM.mmmm" (lat) or "DDDMM.mmmm" (lon) + hemisphere char
 * into degrees × 1e7. Returns 0 on a blank/invalid field. */
static int32_t parse_lat_lon(const char *dmm, char hemi)
{
    if (dmm == NULL || *dmm == '\0') return 0;
    double v = atof(dmm);
    double deg_whole = (double)((int)v / 100);          /* truncate to whole degrees */
    double minutes   = v - deg_whole * 100.0;
    double deg       = deg_whole + minutes / 60.0;
    if (hemi == 'S' || hemi == 'W') deg = -deg;
    /* Round-to-nearest, away from zero. */
    double scaled = deg * 1e7;
    return (int32_t)(scaled >= 0 ? scaled + 0.5 : scaled - 0.5);
}

/* Token splitter — replaces ',' with '\0' in `buf` and writes pointers
 * into `out` for up to `max` tokens. Returns the token count. The '*'
 * before the checksum is also treated as a terminator. */
static size_t split_csv(char *buf, char *out[], size_t max)
{
    size_t n = 0;
    out[n++] = buf;
    for (char *p = buf; *p != '\0' && n < max; p++) {
        if (*p == ',' || *p == '*') {
            *p = '\0';
            out[n++] = p + 1;
        }
    }
    return n;
}

bool idl0_nmea_verify_checksum(const char *line)
{
    if (line == NULL || line[0] != '$') return false;
    uint8_t sum = 0;
    const char *p = line + 1;
    while (*p != '\0' && *p != '*') {
        sum ^= (uint8_t)*p;
        p++;
    }
    if (*p != '*' || !isxdigit((unsigned char)p[1]) || !isxdigit((unsigned char)p[2])) {
        return false;
    }
    char hex[3] = { p[1], p[2], '\0' };
    return (uint8_t)strtol(hex, NULL, 16) == sum;
}

bool idl0_nmea_parse_rmc(const char *line, idl0_gps_fix_t *fix)
{
    if (!idl0_nmea_verify_checksum(line)) return false;
    /* Talker prefix may be GP, GN, GL, GA — match on the message id only. */
    if (strlen(line) < 6 || strncmp(line + 3, "RMC", 3) != 0) return false;

    char buf[96];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char *t[16] = {0};
    size_t n = split_csv(buf, t, 16);
    if (n < 10) return false;

    /* RMC field order:
     *  [0]=$xxRMC  [1]=hhmmss.ss  [2]=status (A/V)
     *  [3]=lat     [4]=N/S        [5]=lon       [6]=E/W
     *  [7]=speed (knots)  [8]=course (deg true) [9]=ddmmyy */
    if (t[2][0] != 'A') return false;  /* void fix */

    /* Time hhmmss.ss → hh, mm, ss, ms */
    const char *tm = t[1];
    if (strlen(tm) < 6) return false;
    int hh = (tm[0]-'0')*10 + (tm[1]-'0');
    int mm = (tm[2]-'0')*10 + (tm[3]-'0');
    int ss = (tm[4]-'0')*10 + (tm[5]-'0');
    int ms = 0;
    if (tm[6] == '.') {
        for (int i = 0; i < 3; i++) {
            char c = tm[7 + i];
            ms = ms * 10 + (isdigit((unsigned char)c) ? (c - '0') : 0);
        }
    }

    /* Date ddmmyy → d, m, y (2000-relative). */
    const char *dt = t[9];
    if (strlen(dt) < 6) return false;
    int dd = (dt[0]-'0')*10 + (dt[1]-'0');
    int mo = (dt[2]-'0')*10 + (dt[3]-'0');
    int yy = (dt[4]-'0')*10 + (dt[5]-'0');
    int year = 2000 + yy;

    int64_t days = days_from_civil(year, (unsigned)mo, (unsigned)dd);
    int64_t secs = days * 86400 + (int64_t)hh * 3600 + (int64_t)mm * 60 + ss;
    fix->gps_epoch_ms = secs * 1000 + ms;

    fix->latitude_e7  = parse_lat_lon(t[3], t[4][0]);
    fix->longitude_e7 = parse_lat_lon(t[5], t[6][0]);

    double knots = (t[7][0] != '\0') ? atof(t[7]) : 0.0;
    double kmh   = knots * 1.852;
    double kmh_x100 = kmh * 100.0 + 0.5;
    fix->speed_x100 = (kmh_x100 < 0.0)        ? 0
                    : (kmh_x100 > 65535.0)    ? 65535
                    : (uint16_t)kmh_x100;

    double course = (t[8][0] != '\0') ? atof(t[8]) : 0.0;
    double course_x100 = course * 100.0 + 0.5;
    fix->heading_x100 = (course_x100 < 0.0)     ? 0
                      : (course_x100 > 65535.0) ? 65535
                      : (uint16_t)course_x100;

    return true;
}

bool idl0_nmea_parse_gga(const char *line, idl0_gps_fix_t *fix)
{
    if (!idl0_nmea_verify_checksum(line)) return false;
    if (strlen(line) < 6 || strncmp(line + 3, "GGA", 3) != 0) return false;

    char buf[96];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char *t[16] = {0};
    size_t n = split_csv(buf, t, 16);
    if (n < 10) return false;

    /* GGA fields:
     *  [0]=$xxGGA  [1]=hhmmss.ss  [2]=lat [3]=N/S
     *  [4]=lon     [5]=E/W        [6]=fix_quality (0..9)
     *  [7]=satellites [8]=hdop    [9]=altitude (m) [10]=altitude unit */

    int fq = atoi(t[6]);
    fix->fix_quality = (fq < 0) ? 0 : (fq > 255 ? 255 : (uint8_t)fq);

    int sats = atoi(t[7]);
    fix->satellites = (sats < 0) ? 0 : (sats > 255 ? 255 : (uint8_t)sats);

    double alt = (t[9][0] != '\0') ? atof(t[9]) : 0.0;
    double alt_x10 = alt * 10.0 + (alt >= 0.0 ? 0.5 : -0.5);
    fix->altitude_x10 = (alt_x10 < -32768.0) ? -32768
                      : (alt_x10 >  32767.0) ?  32767
                      : (int16_t)alt_x10;

    return true;
}

/* Integrator — RMC drives emit; latest GGA is folded in. */
static idl0_gps_fix_t s_pending = {0};
static bool           s_gga_seen = false;

bool idl0_gps_feed_line(const char *line, idl0_gps_fix_t *out_fix)
{
    if (line == NULL || line[0] != '$') return false;

    /* GGA: update the rolling altitude/quality/sats. */
    if (strlen(line) >= 6 && strncmp(line + 3, "GGA", 3) == 0) {
        if (idl0_nmea_parse_gga(line, &s_pending)) {
            s_gga_seen = true;
        }
        return false;
    }

    /* RMC: emit if checksum + status='A'; carry forward latest GGA. */
    if (strlen(line) >= 6 && strncmp(line + 3, "RMC", 3) == 0) {
        if (!idl0_nmea_parse_rmc(line, &s_pending)) return false;
        s_pending.device_timestamp_us = esp_timer_get_time();
        if (!s_gga_seen) {
            /* GGA hasn't arrived yet — emit a fix with altitude/quality/sats=0. */
            s_pending.altitude_x10 = 0;
            s_pending.fix_quality  = 0;
            s_pending.satellites   = 0;
        }
        if (s_state == IDL0_GPS_NOFIX) s_state = IDL0_GPS_FIX;
        *out_fix = s_pending;
        return true;
    }

    return false;  /* unhandled sentence — silently ignored */
}

#define IDL0_GPS_UART       UART_NUM_1
#define IDL0_GPS_BAUD       9600
#define IDL0_GPS_RX_BUF     2048

static const char *TAG = "gps";

bool idl0_gps_init(void)
{
    const uart_config_t cfg = {
        .baud_rate  = IDL0_GPS_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(IDL0_GPS_UART, IDL0_GPS_RX_BUF, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(err));
        s_state = IDL0_GPS_ABSENT;
        return false;
    }
    err = uart_param_config(IDL0_GPS_UART, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(err));
        s_state = IDL0_GPS_ABSENT;
        return false;
    }
    /* TX pin not wired on this board revision — pass UART_PIN_NO_CHANGE
     * so the driver leaves TX unmapped. TODO(idl0): when the TX trace
     * is reworked, set TX to IDL0_PIN_GPS_TX and emit UBX-CFG-* here. */
    err = uart_set_pin(IDL0_GPS_UART,
                       UART_PIN_NO_CHANGE,   /* tx */
                       IDL0_PIN_GPS_RX,      /* rx */
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(err));
        s_state = IDL0_GPS_ABSENT;
        return false;
    }
    s_state = IDL0_GPS_NOFIX;
    ESP_LOGI(TAG, "GPS UART up on RX=%d @ %d baud (TX unwired)",
             IDL0_PIN_GPS_RX, IDL0_GPS_BAUD);
    return true;
}
