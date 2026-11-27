// Wi-Fi update mode: the handheld becomes a Wi-Fi access point with a small web page.
//   http://192.168.4.1/      upload page (pick build\treeline_ui.bin, press Update)
//   http://192.168.4.1/log   console log (same text as the USB serial monitor)
// The new image goes into the other OTA slot; the old one keeps running until it is
// verified, and the bootloader rolls back if the new one never confirms itself.
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "tlog.h"
#include "ota.h"

#define OTA_TX_POWER_QDBM 34   // 0.25 dBm units: 34 = 8.5 dBm (range 8..84)

static ota_status_cb_t status_cb;
static int64_t last_cb_us;

static void status(int pct, const char *msg) { if (status_cb) status_cb(pct, msg); }

static const char PAGE[] =
"<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'>"
"<title>TREELINE update</title><style>"
"body{background:#05080c;color:#4cb8ff;font:15px monospace;max-width:520px;margin:24px auto;padding:0 14px}"
"h1{font-size:20px;border-bottom:1px dashed #4cb8ff;padding-bottom:6px}"
"input,button{font:inherit;color:#4cb8ff;background:#0b1520;border:1px solid #4cb8ff;padding:8px;margin:6px 0;width:100%}"
"button{cursor:pointer}#bar{height:14px;border:1px solid #4cb8ff;margin:10px 0}#fill{height:100%;width:0;background:#4cb8ff}"
"a{color:#4cb8ff}</style></head><body><h1>TREELINE // firmware update</h1>"
"<p>running: <b>%VER%</b></p>"
"<p>Pick <b>build\\treeline_ui.bin</b> from the project folder:</p>"
"<input type=file id=f accept='.bin'><button onclick=go()>UPDATE</button>"
"<div id=bar><div id=fill></div></div><div id=msg>&nbsp;</div>"
"<p><a href=/log>console log</a></p><script>"
"function go(){var f=document.getElementById('f').files[0];if(!f){msg.textContent='pick a .bin file first';return}"
"var x=new XMLHttpRequest();x.open('POST','/update');"
"x.upload.onprogress=function(e){if(e.lengthComputable){var p=Math.round(100*e.loaded/e.total);fill.style.width=p+'%';msg.textContent='uploading '+p+'%'}};"
"x.onload=function(){msg.textContent=x.responseText};x.onerror=function(){msg.textContent='connection lost (if it reached 100% the device is rebooting)'};"
"x.send(f)}</script></body></html>";

static esp_err_t page_get(httpd_req_t *req)
{
    const esp_app_desc_t *d = esp_app_get_description();
    char ver[64];
    snprintf(ver, sizeof(ver), "%s %s %s", d->project_name, d->date, d->time);
    const char *mark = strstr(PAGE, "%VER%");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send_chunk(req, PAGE, mark - PAGE);
    httpd_resp_send_chunk(req, ver, strlen(ver));
    httpd_resp_send_chunk(req, mark + 5, strlen(mark + 5));
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t log_get(httpd_req_t *req)
{
    static char buf[6200];
    size_t n = tl_log_copy(buf, sizeof(buf));
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Refresh", "3");
    return httpd_resp_send(req, buf, n);
}

static void reboot_task(void *arg) { vTaskDelay(pdMS_TO_TICKS(1500)); esp_restart(); }

static esp_err_t update_post(httpd_req_t *req)
{
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no OTA slot (partition table)"); return ESP_FAIL; }
    int total = req->content_len, got = 0;
    if (total <= 0 || total > (int)part->size) { httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad file size"); return ESP_FAIL; }
    printf("OTA: receiving %d bytes into %s\n", total, part->label);
    status(0, "receiving");
    esp_ota_handle_t h;
    esp_err_t err = esp_ota_begin(part, OTA_SIZE_UNKNOWN, &h);
    if (err != ESP_OK) { status(-2, "flash erase failed"); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err)); return ESP_FAIL; }
    static char buf[2048];
    while (got < total) {
        int n = httpd_req_recv(req, buf, sizeof(buf) < (size_t)(total - got) ? sizeof(buf) : (size_t)(total - got));
        if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (n <= 0) { esp_ota_abort(h); status(-2, "upload interrupted"); printf("OTA: upload interrupted\n"); return ESP_FAIL; }
        if ((err = esp_ota_write(h, buf, n)) != ESP_OK) {
            esp_ota_abort(h); status(-2, "not a valid image");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "write failed: not a valid ESP32-C3 image?");
            printf("OTA: write failed %s\n", esp_err_to_name(err));
            return ESP_FAIL;
        }
        got += n;
        int64_t now = esp_timer_get_time();
        if (now - last_cb_us > 150000 || got == total) { last_cb_us = now; status((int)(100LL * got / total), "receiving"); }
    }
    status(100, "verifying");
    if ((err = esp_ota_end(h)) != ESP_OK) {
        status(-2, "image check failed");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "image check failed: wrong file?");
        printf("OTA: image check failed %s\n", esp_err_to_name(err));
        return ESP_FAIL;
    }
    if ((err = esp_ota_set_boot_partition(part)) != ESP_OK) {
        status(-2, "could not switch slot");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_FAIL;
    }
    printf("OTA: done, booting %s\n", part->label);
    status(101, "rebooting");
    httpd_resp_sendstr(req, "OK - new firmware written, device is rebooting");
    xTaskCreate(reboot_task, "reboot", 2048, NULL, 5, NULL);
    return ESP_OK;
}

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (id == WIFI_EVENT_AP_STACONNECTED) { printf("OTA: a device joined the Wi-Fi\n"); status(-1, "laptop connected"); }
    if (id == WIFI_EVENT_AP_STADISCONNECTED) printf("OTA: a device left the Wi-Fi\n");
}

void ota_start(const char *ssid, const char *pass, ota_status_cb_t cb)
{
    status_cb = cb;
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL));
    wifi_config_t wc = { 0 };
    strncpy((char *)wc.ap.ssid, ssid, sizeof(wc.ap.ssid) - 1);
    wc.ap.ssid_len = strlen(ssid);
    strncpy((char *)wc.ap.password, pass, sizeof(wc.ap.password) - 1);
    wc.ap.channel = 6;
    wc.ap.max_connection = 2;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    // C3 SuperMini: the antenna match is poor and at full power (~20 dBm) the AP is often
    // not seen at all. 8.5 dBm is plenty for a laptop next to the handheld.
    esp_err_t pe = esp_wifi_set_max_tx_power(OTA_TX_POWER_QDBM);
    int8_t pw = 0; esp_wifi_get_max_tx_power(&pw);
    printf("OTA: Wi-Fi TX power %.2f dBm (%s)\n", pw / 4.0, esp_err_to_name(pe));

    httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
    hc.recv_wait_timeout = 15;
    hc.stack_size = 6144;
    httpd_handle_t srv;
    ESP_ERROR_CHECK(httpd_start(&srv, &hc));
    const httpd_uri_t u1 = { .uri = "/", .method = HTTP_GET, .handler = page_get };
    const httpd_uri_t u2 = { .uri = "/log", .method = HTTP_GET, .handler = log_get };
    const httpd_uri_t u3 = { .uri = "/update", .method = HTTP_POST, .handler = update_post };
    httpd_register_uri_handler(srv, &u1);
    httpd_register_uri_handler(srv, &u2);
    httpd_register_uri_handler(srv, &u3);
    printf("OTA: Wi-Fi '%s' (password '%s') up, open http://192.168.4.1\n", ssid, pass);
    status(-1, "waiting for upload");
}

// Call once the firmware has started properly: cancels the automatic rollback.
void ota_mark_good(void)
{
    esp_ota_img_states_t st;
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_err_t e = esp_ota_get_state_partition(run, &st);
    if (e == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        printf("OTA: new firmware confirmed good (%s)\n", run->label);
    } else {
        // 0=new 1=pending 2=valid 3=invalid 4=aborted, -1=undefined (USB flash, no otadata entry)
        printf("OTA: running %s, state %d (nothing to confirm)\n", run->label, e == ESP_OK ? (int)st : -1);
    }
}
