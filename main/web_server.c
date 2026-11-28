// TREELINE HTTP server & REST API (coexists with OTA at /ota, /log, /update).
//
// Endpoints:
//   GET  /                  Embedded chat web app (main/web/index.html)
//   POST /time              Syncs Unix epoch clock
//   POST /send              Validates UTF-8 message, stores as "sending", queues to radio
//   GET  /messages?since=N  Streams JSON messages with rev > N + last_rev
//   GET  /status            Returns sensor, battery, radio link, warnings & errors JSON
//   POST /test/bad_crc      Demo helper: transmits a frame with corrupted CRC
//   GET  /ota, /log, POST /update   OTA endpoints registered via ota_register()
#include <stdio.h>
#include "tlog.h"
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "app_queues.h"
#include "msg_store.h"
#include "status.h"
#include "link.h"
#include "ota.h"
#include "web_server.h"

extern const char _binary_index_html_start[];
extern const char _binary_index_html_end[];

#define MAX_SEND_BODY  1024
#define MAX_TIME_BODY  256

static httpd_handle_t s_srv;

// ---------------------------------------------------------------- helpers
static esp_err_t send_json_status(httpd_req_t *req, const char *http_status, const char *json)
{
    httpd_resp_set_status(req, http_status);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

// Reads full HTTP body into buf (NUL-terminated). Rejects oversized/empty bodies before reading.
static int recv_body(httpd_req_t *req, char *buf, size_t max_len)
{
    int total = req->content_len;
    if (total <= 0 || (size_t)total > max_len) return -1;

    int got = 0;
    int timeouts = 0;
    while (got < total) {
        int n = httpd_req_recv(req, buf + got, (size_t)(total - got));
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            if (++timeouts >= 5) return -1;
            continue;
        }
        if (n <= 0) return -1;
        got += n;
        timeouts = 0;
    }
    buf[got] = '\0';
    return got;
}

static bool is_blank(const char *s)
{
    for (; *s; s++) {
        if ((unsigned char)*s > ' ' || !isspace((unsigned char)*s)) return false;
    }
    return true;
}

static double round1(float v) { return round((double)v * 10.0) / 10.0; }
static double round2(float v) { return round((double)v * 100.0) / 100.0; }

// ---------------------------------------------------------------- GET /
static esp_err_t index_get(httpd_req_t *req)
{
    size_t len = (size_t)(_binary_index_html_end - _binary_index_html_start);
    // EMBED_TXTFILES appends a trailing NUL byte; strip it before sending
    if (len > 0 && _binary_index_html_start[len - 1] == '\0') len--;
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, _binary_index_html_start, (ssize_t)len);
}

// ---------------------------------------------------------------- POST /time
static esp_err_t time_post(httpd_req_t *req)
{
    char body[MAX_TIME_BODY + 1];
    if (recv_body(req, body, MAX_TIME_BODY) < 0) {
        return send_json_status(req, "400 Bad Request", "{\"error\":\"bad_json\"}");
    }

    cJSON *root = cJSON_Parse(body);
    if (!root || !cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return send_json_status(req, "400 Bad Request", "{\"error\":\"bad_json\"}");
    }

    const cJSON *ep = cJSON_GetObjectItemCaseSensitive(root, "epoch");
    if (!cJSON_IsNumber(ep) || ep->valuedouble < 1000000000.0) {
        cJSON_Delete(root);
        return send_json_status(req, "400 Bad Request", "{\"error\":\"bad_json\"}");
    }

    int64_t epoch = (int64_t)ep->valuedouble;
    cJSON_Delete(root);

    msg_store_set_time(epoch);
    return send_json_status(req, "200 OK", "{\"ok\":true}");
}

// ---------------------------------------------------------------- POST /send
static esp_err_t send_post(httpd_req_t *req)
{
    char body[MAX_SEND_BODY + 1];
    if (recv_body(req, body, MAX_SEND_BODY) < 0) {
        return send_json_status(req, "400 Bad Request", "{\"error\":\"bad_json\"}");
    }

    cJSON *root = cJSON_Parse(body);
    if (!root || !cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return send_json_status(req, "400 Bad Request", "{\"error\":\"bad_json\"}");
    }

    const cJSON *jt = cJSON_GetObjectItemCaseSensitive(root, "text");
    if (!cJSON_IsString(jt) || jt->valuestring == NULL) {
        cJSON_Delete(root);
        return send_json_status(req, "400 Bad Request", "{\"error\":\"bad_json\"}");
    }

    const char *txt = jt->valuestring;
    size_t len = strlen(txt);
    if (len == 0 || is_blank(txt)) {
        cJSON_Delete(root);
        return send_json_status(req, "400 Bad Request", "{\"error\":\"empty\"}");
    }
    // Check UTF-8 length strictly in bytes (never truncate)
    if (len > MSG_MAX_BYTES) {
        cJSON_Delete(root);
        return send_json_status(req, "400 Bad Request", "{\"error\":\"too_long\"}");
    }

    // Never block on radio: reject with 503 immediately if TX queue is full
    if (!q_to_radio || uxQueueSpacesAvailable(q_to_radio) == 0) {
        cJSON_Delete(root);
        return send_json_status(req, "503 Service Unavailable", "{\"error\":\"busy\"}");
    }

    out_msg_t out = { 0 };
    out.len = (uint16_t)len;
    memcpy(out.text, txt, len);
    out.text[len] = '\0';
    cJSON_Delete(root);

    uint32_t id = msg_store_add_out(out.text, out.len);
    out.id = id;

    if (xQueueSend(q_to_radio, &out, 0) != pdTRUE) {
        msg_store_set_status(id, MSG_STATUS_FAILED);
        return send_json_status(req, "503 Service Unavailable", "{\"error\":\"busy\"}");
    }

    char resp[64];
    snprintf(resp, sizeof(resp), "{\"id\":%lu,\"status\":\"sending\"}", (unsigned long)id);
    return send_json_status(req, "200 OK", resp);
}

// ---------------------------------------------------------------- GET /messages?since=<rev>
#define MSG_STREAM_BATCH 5

static esp_err_t messages_get(httpd_req_t *req)
{
    uint32_t since = 0;
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[24];
        if (httpd_query_key_value(query, "since", val, sizeof(val)) == ESP_OK) {
            unsigned long parsed = strtoul(val, NULL, 10);
            since = (uint32_t)parsed;
        }
    }

    // Freeze the upper rev bound for this response so anything arriving mid-stream
    // has rev > snap_rev and is guaranteed to be picked up on the next poll.
    uint32_t snap_rev = msg_store_last_rev();

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    char hdr[64];
    int hlen = snprintf(hdr, sizeof(hdr), "{\"last_rev\":%lu,\"messages\":[", (unsigned long)snap_rev);
    esp_err_t err = httpd_resp_send_chunk(req, hdr, hlen);
    if (err != ESP_OK) return err;

    // Small static batch buffer (~1 KB instead of 10 KB) + never hold mutex across send_chunk
    static msg_t batch[MSG_STREAM_BATCH];
    uint32_t after_id = 0;
    bool first = true;

    if (snap_rev > since) {
        for (;;) {
            int count = msg_store_batch(since, snap_rev, after_id, batch, MSG_STREAM_BATCH);
            if (count <= 0) break;

            for (int i = 0; i < count; i++) {
                const msg_t *m = &batch[i];
                after_id = m->id;

                cJSON *obj = cJSON_CreateObject();
                if (!obj) {
                    httpd_resp_send_chunk(req, NULL, 0);
                    return ESP_ERR_NO_MEM;
                }
                cJSON_AddNumberToObject(obj, "id",     (double)m->id);
                cJSON_AddNumberToObject(obj, "rev",    (double)m->rev);
                cJSON_AddStringToObject(obj, "from",   m->from);
                cJSON_AddStringToObject(obj, "dir",    msg_dir_str(m->dir));
                cJSON_AddNumberToObject(obj, "time",   (double)m->time);
                cJSON_AddStringToObject(obj, "status", msg_status_str(m->status));
                cJSON_AddStringToObject(obj, "text",   m->text);

                char *js = cJSON_PrintUnformatted(obj);
                cJSON_Delete(obj);
                if (!js) {
                    httpd_resp_send_chunk(req, NULL, 0);
                    return ESP_ERR_NO_MEM;
                }

                if (!first) err = httpd_resp_send_chunk(req, ",", 1);
                if (err == ESP_OK) err = httpd_resp_send_chunk(req, js, (ssize_t)strlen(js));
                free(js);
                first = false;
                if (err != ESP_OK) return err;
            }

            if (count < MSG_STREAM_BATCH) break;
        }
    }

    err = httpd_resp_send_chunk(req, "]}", 2);
    if (err != ESP_OK) return err;
    return httpd_resp_send_chunk(req, NULL, 0);
}

// ---------------------------------------------------------------- GET /status
static esp_err_t status_get_handler(httpd_req_t *req)
{
    status_t st;
    status_get(&st);

    cJSON *root = cJSON_CreateObject();
    if (!root) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");

    cJSON_AddStringToObject(root, "device",       st.device);
    cJSON_AddStringToObject(root, "peer",         st.peer);
    cJSON_AddNumberToObject(root, "uptime_s",     (double)st.uptime_s);
    cJSON_AddNumberToObject(root, "wifi_clients", st.wifi_clients >= 0 ? st.wifi_clients : 0);

    // link object
    cJSON *link = cJSON_AddObjectToObject(root, "link");
    if (link) {
        cJSON_AddBoolToObject(link, "up", st.link_up);
        if (st.last_heard_s >= 0) cJSON_AddNumberToObject(link, "last_heard_s", st.last_heard_s);
        else                      cJSON_AddNullToObject(link,   "last_heard_s");
        if (st.ping_ms >= 0)      cJSON_AddNumberToObject(link, "ping_ms", st.ping_ms);
        else                      cJSON_AddNullToObject(link,   "ping_ms");
        cJSON_AddNumberToObject(link, "loss_pct", st.loss_pct);
        cJSON_AddNumberToObject(link, "tx",       (double)st.tx);
        cJSON_AddNumberToObject(link, "rx",       (double)st.rx);
    }

    // battery object
    cJSON *batt = cJSON_AddObjectToObject(root, "battery");
    if (batt) {
        if (st.batt_ok) {
            cJSON_AddNumberToObject(batt, "percent", st.batt_percent);
            cJSON_AddNumberToObject(batt, "volts",   round2(st.batt_volts));
        } else {
            cJSON_AddNullToObject(batt, "percent");
            cJSON_AddNullToObject(batt, "volts");
        }
        cJSON_AddBoolToObject(batt, "charging", st.batt_charging);
        if (st.ds18b20_ok) cJSON_AddNumberToObject(batt, "temp_c", round1(st.batt_temp_c));
        else               cJSON_AddNullToObject(batt,   "temp_c");
    }

    // chip temp
    if (st.chip_temp_ok) cJSON_AddNumberToObject(root, "chip_temp_c", round1(st.chip_temp_c));
    else                 cJSON_AddNullToObject(root,   "chip_temp_c");

    // barometer
    if (st.bmp280_ok) {
        cJSON_AddNumberToObject(root, "pressure_hpa",   round1(st.pressure_hpa));
        cJSON_AddNumberToObject(root, "altitude_m",     st.altitude_m);
        cJSON_AddStringToObject(root, "pressure_trend", st.pressure_trend);
    } else {
        cJSON_AddNullToObject(root, "pressure_hpa");
        cJSON_AddNullToObject(root, "altitude_m");
        cJSON_AddNullToObject(root, "pressure_trend");
    }

    // warnings[]
    cJSON *warnings = cJSON_AddArrayToObject(root, "warnings");
    if (warnings) {
        if (st.warn_battery_low) cJSON_AddItemToArray(warnings, cJSON_CreateString("battery_low"));
        if (st.warn_battery_hot) cJSON_AddItemToArray(warnings, cJSON_CreateString("battery_hot"));
        if (st.warn_storm)       cJSON_AddItemToArray(warnings, cJSON_CreateString("storm"));
    }

    // errors[] (DS18B20 is intentionally absent on PCB v3 -> temp_c stays null, not an error)
    cJSON *errors = cJSON_AddArrayToObject(root, "errors");
    if (errors) {
        if (!st.bmp280_ok)    cJSON_AddItemToArray(errors, cJSON_CreateString("bmp280"));
        if (!st.radio_ok)     cJSON_AddItemToArray(errors, cJSON_CreateString("hc12"));
        if (!st.chip_temp_ok) cJSON_AddItemToArray(errors, cJSON_CreateString("chip_temp"));
    }

    char *js = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!js) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");

    esp_err_t err = send_json_status(req, "200 OK", js);
    free(js);
    return err;
}

// ---------------------------------------------------------------- POST /test/bad_crc
static esp_err_t bad_crc_post(httpd_req_t *req)
{
    if (!link_send_bad_crc_test("CORRUPTED_CRC_TEST")) {
        return send_json_status(req, "503 Service Unavailable", "{\"error\":\"no_radio\"}");
    }
    return send_json_status(req, "200 OK", "{\"ok\":true}");
}

// ---------------------------------------------------------------- lifecycle
esp_err_t web_server_start(ota_status_cb_t ota_cb)
{
    if (s_srv) return ESP_OK;

    httpd_config_t hc    = HTTPD_DEFAULT_CONFIG();
    hc.stack_size        = 8192;
    hc.max_uri_handlers  = 12;
    hc.recv_wait_timeout = 15;
    hc.lru_purge_enable  = true;

    esp_err_t err = httpd_start(&s_srv, &hc);
    if (err != ESP_OK) return err;

    static const httpd_uri_t routes[] = {
        { .uri = "/",             .method = HTTP_GET,  .handler = index_get },
        { .uri = "/time",         .method = HTTP_POST, .handler = time_post },
        { .uri = "/send",         .method = HTTP_POST, .handler = send_post },
        { .uri = "/messages",     .method = HTTP_GET,  .handler = messages_get },
        { .uri = "/status",       .method = HTTP_GET,  .handler = status_get_handler },
        { .uri = "/test/bad_crc", .method = HTTP_POST, .handler = bad_crc_post },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        err = httpd_register_uri_handler(s_srv, &routes[i]);
        if (err != ESP_OK) return err;
    }

    // Mount OTA alongside chat at /ota (plus /log and /update)
    err = ota_register(s_srv, "/ota", ota_cb);
    if (err != ESP_OK) return err;

    printf("WEB: server up at http://192.168.4.1/ (chat + /ota + /log)\n");
    return ESP_OK;
}

void web_server_stop(void)
{
    if (!s_srv) return;
    httpd_stop(s_srv);
    s_srv = NULL;
    printf("WEB: server stopped\n");
}
