// Single owner of ESP32-C3 Wi-Fi SoftAP (shared by normal chat mode and OTA update mode).
#pragma once

#include <stdbool.h>
#include "esp_err.h"

typedef void (*wifi_ap_evt_cb_t)(bool joined, int total_clients);

esp_err_t wifi_ap_start(const char *ssid, const char *pass);
void      wifi_ap_stop(void);
int       wifi_ap_clients(void);
void      wifi_ap_set_evt_cb(wifi_ap_evt_cb_t cb);
