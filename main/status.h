// Thread-safe telemetry snapshot shared between sensor/radio loop and GET /status.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    char     device[8];
    char     peer[8];
    uint32_t uptime_s;
    int      wifi_clients;

    // Radio link
    bool     radio_ok;
    bool     link_up;
    int      last_heard_s;      // -1 if peer never heard
    int      ping_ms;           // -1 if no ping answer yet
    int      loss_pct;
    uint32_t tx;
    uint32_t rx;

    // Battery & charger
    bool     batt_ok;
    int      batt_percent;
    float    batt_volts;
    bool     batt_charging;

    // DS18B20 battery temperature
    bool     ds18b20_ok;
    float    batt_temp_c;

    // ESP32-C3 internal chip temperature
    bool     chip_temp_ok;
    float    chip_temp_c;

    // BMP280 barometer
    bool     bmp280_ok;
    float    pressure_hpa;
    int      altitude_m;
    char     pressure_trend[12]; // "rising", "steady", "falling"

    // Active warnings
    bool     warn_battery_low;
    bool     warn_battery_hot;
    bool     warn_storm;
} status_t;

void status_init(const char *device, const char *peer);
void status_set(const status_t *in);
void status_get(status_t *out);
