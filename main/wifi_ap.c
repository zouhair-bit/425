// Wi-Fi SoftAP for TREELINE handhelds (ECHO-LINK / DELTA-LINK).
// Owns esp_netif, default event loop, and Wi-Fi driver so chat and OTA never collide.
#include <stdio.h>
#include "tlog.h"
#include <string.h>
#include <stdatomic.h>
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "wifi_ap.h"

#define WIFI_AP_CHANNEL       6
#define WIFI_AP_MAX_STATIONS  4
#define WIFI_AP_TX_POWER_QDBM 34   // 8.5 dBm: ESP32-C3 SuperMini antenna fix

static bool             s_started;
static bool             s_inited;
static atomic_int       s_clients;
static wifi_ap_evt_cb_t s_evt_cb;

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)data;
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        int n = atomic_fetch_add(&s_clients, 1) + 1;
        printf("WIFI: station joined (%d connected)\n", n);
        if (s_evt_cb) s_evt_cb(true, n);
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        int prev = atomic_fetch_sub(&s_clients, 1);
        int n = prev > 0 ? prev - 1 : 0;
        if (prev <= 0) atomic_store(&s_clients, 0);
        printf("WIFI: station left (%d connected)\n", n);
        if (s_evt_cb) s_evt_cb(false, n);
    }
}

void wifi_ap_set_evt_cb(wifi_ap_evt_cb_t cb)
{
    s_evt_cb = cb;
}

esp_err_t wifi_ap_start(const char *ssid, const char *pass)
{
    if (s_started) return ESP_OK;

    if (!s_inited) {
        esp_err_t e = esp_netif_init();
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
        e = esp_event_loop_create_default();
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
        esp_netif_create_default_wifi_ap();   // default IP 192.168.4.1/24
        wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
        e = esp_wifi_init(&ic);
        if (e != ESP_OK) return e;
        e = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL);
        if (e != ESP_OK) return e;
        s_inited = true;
    }

    wifi_config_t wc = { 0 };
    snprintf((char *)wc.ap.ssid, sizeof(wc.ap.ssid), "%s", ssid);
    wc.ap.ssid_len = (uint8_t)strlen((const char *)wc.ap.ssid);
    snprintf((char *)wc.ap.password, sizeof(wc.ap.password), "%s", pass);
    wc.ap.channel        = WIFI_AP_CHANNEL;
    wc.ap.max_connection = WIFI_AP_MAX_STATIONS;
    wc.ap.authmode       = strlen(pass) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    esp_err_t e = esp_wifi_set_mode(WIFI_MODE_AP);
    if (e != ESP_OK) return e;
    e = esp_wifi_set_config(WIFI_IF_AP, &wc);
    if (e != ESP_OK) return e;
    e = esp_wifi_start();
    if (e != ESP_OK) return e;

    // Required immediately after esp_wifi_start() on ESP32-C3 SuperMini
    esp_err_t pe = esp_wifi_set_max_tx_power(WIFI_AP_TX_POWER_QDBM);
    int8_t pw = 0;
    esp_wifi_get_max_tx_power(&pw);
    atomic_store(&s_clients, 0);
    s_started = true;
    printf("WIFI: AP '%s' up at 192.168.4.1 (max %d clients, TX %.2f dBm: %s)\n",
           ssid, WIFI_AP_MAX_STATIONS, pw / 4.0f, esp_err_to_name(pe));
    return ESP_OK;
}

void wifi_ap_stop(void)
{
    if (!s_started) return;
    esp_wifi_stop();
    atomic_store(&s_clients, 0);
    s_started = false;
    printf("WIFI: AP stopped\n");
}

int wifi_ap_clients(void)
{
    if (!s_started) return -1;
    int n = atomic_load(&s_clients);
    return n < 0 ? 0 : n;
}
