// Wi-Fi OTA firmware update handlers (see ota.h).
//   GET  /ota (or / in update mode)   upload page
//   GET  /log                         RAM console log (same text as USB serial monitor)
//   POST /update                      streams new .bin into the inactive OTA partition
#include <stdio.h>
#include "tlog.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "wifi_ap.h"
#include "ota.h"

static ota_status_cb_t status_cb;
static int64_t last_cb_us;

static void status(int pct, const char *msg)
{
    if (status_cb) status_cb(pct, msg);
}

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
"<p><a href=/log>console log</a> &middot; <a href=/>chat</a></p><script>"
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
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send_chunk(req, PAGE, mark - PAGE);
    httpd_resp_send_chunk(req, ver, strlen(ver));
    httpd_resp_send_chunk(req, mark + 5, strlen(mark + 5));
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t log_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "Refresh", "3");

    uint32_t pos = 0, end_pos = 0;
    tl_log_range(&pos, &end_pos);

    char chunk[512];
    while ((int32_t)(end_pos - pos) > 0) {
        // tl_log_copy_slice locks, copies <= 512 B, and releases mutex before send_chunk
        size_t n = tl_log_copy_slice(&pos, end_pos, chunk, sizeof(chunk));
        if (n == 0) break;
        esp_err_t err = httpd_resp_send_chunk(req, chunk, (ssize_t)n);
        if (err != ESP_OK) return err;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

static void reboot_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
}

static esp_err_t update_post(httpd_req_t *req)
{
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no OTA slot (partition table)");
        return ESP_FAIL;
    }
    int total = req->content_len, got = 0;
    if (total <= 0 || total > (int)part->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad file size");
        return ESP_FAIL;
    }
    printf("OTA: receiving %d bytes into %s\n", total, part->label);
    status(0, "receiving");
    esp_ota_handle_t h;
    esp_err_t err = esp_ota_begin(part, OTA_SIZE_UNKNOWN, &h);
    if (err != ESP_OK) {
        status(-2, "flash erase failed");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
        return ESP_FAIL;
    }
    static char buf[2048];
    while (got < total) {
        size_t chunk = sizeof(buf) < (size_t)(total - got) ? sizeof(buf) : (size_t)(total - got);
        int n = httpd_req_recv(req, buf, chunk);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (n <= 0) {
            esp_ota_abort(h);
            status(-2, "upload interrupted");
            printf("OTA: upload interrupted\n");
            return ESP_FAIL;
        }
        if ((err = esp_ota_write(h, buf, n)) != ESP_OK) {
            esp_ota_abort(h);
            status(-2, "not a valid image");
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "write failed: not a valid ESP32-C3 image?");
            printf("OTA: write failed %s\n", esp_err_to_name(err));
            return ESP_FAIL;
        }
        got += n;
        int64_t now = esp_timer_get_time();
        if (now - last_cb_us > 150000 || got == total) {
            last_cb_us = now;
            status((int)(100LL * got / total), "receiving");
        }
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

esp_err_t ota_register(httpd_handle_t srv, const char *page_uri, ota_status_cb_t cb)
{
    status_cb = cb;
    const httpd_uri_t u1 = { .uri = page_uri,  .method = HTTP_GET,  .handler = page_get };
    const httpd_uri_t u2 = { .uri = "/log",    .method = HTTP_GET,  .handler = log_get };
    const httpd_uri_t u3 = { .uri = "/update", .method = HTTP_POST, .handler = update_post };
    esp_err_t e = httpd_register_uri_handler(srv, &u1);
    if (e != ESP_OK) return e;
    e = httpd_register_uri_handler(srv, &u2);
    if (e != ESP_OK) return e;
    return httpd_register_uri_handler(srv, &u3);
}

static void on_ota_wifi_evt(bool joined, int total)
{
    (void)total;
    if (joined) status(-1, "laptop connected");
}

// Standalone button-held recovery update mode: shares wifi_ap + registers OTA at "/".
void ota_start(const char *ssid, const char *pass, ota_status_cb_t cb)
{
    status_cb = cb;
    wifi_ap_set_evt_cb(on_ota_wifi_evt);
    ESP_ERROR_CHECK(wifi_ap_start(ssid, pass));

    httpd_config_t hc = HTTPD_DEFAULT_CONFIG();
    hc.recv_wait_timeout = 15;
    hc.stack_size        = 8192;
    hc.max_uri_handlers  = 8;
    hc.lru_purge_enable  = true;
    httpd_handle_t srv = NULL;
    ESP_ERROR_CHECK(httpd_start(&srv, &hc));
    ESP_ERROR_CHECK(ota_register(srv, "/", cb));
    printf("OTA: recovery mode on '%s' (pass '%s'), open http://192.168.4.1\n", ssid, pass);
    status(-1, "waiting for upload");
}

void ota_mark_good(void)
{
    esp_ota_img_states_t st;
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_err_t e = esp_ota_get_state_partition(run, &st);
    if (e == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        printf("OTA: new firmware confirmed good (%s)\n", run->label);
    } else {
        printf("OTA: running %s, state %d (nothing to confirm)\n", run->label, e == ESP_OK ? (int)st : -1);
    }
}
