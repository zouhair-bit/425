// Wi-Fi update mode (ota.c)
#pragma once
// pct: 0..100 upload, 101 = done/rebooting, -1 = waiting, -2 = error
typedef void (*ota_status_cb_t)(int pct, const char *msg);
void ota_start(const char *ssid, const char *pass, ota_status_cb_t cb);
void ota_mark_good(void);
