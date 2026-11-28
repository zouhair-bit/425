// Wi-Fi OTA update endpoints (ota.c), usable both mounted on the shared chat HTTP
// server (/ota) and in button-held recovery update mode (/).
#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

// pct: 0..100 upload, 101 = done/rebooting, -1 = waiting, -2 = error
typedef void (*ota_status_cb_t)(int pct, const char *msg);

esp_err_t ota_register(httpd_handle_t srv, const char *page_uri, ota_status_cb_t cb);
void      ota_start(const char *ssid, const char *pass, ota_status_cb_t cb);
void      ota_mark_good(void);
