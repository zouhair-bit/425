// Thread-safe 50-message RAM ring buffer (see msg_store.h).
#include <stdio.h>
#include "tlog.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "msg_store.h"

static SemaphoreHandle_t s_mu;
static msg_t    s_ring[MSG_STORE_CAP];
static int      s_head;        // index of oldest message in ring
static int      s_count;       // valid entries (0..MSG_STORE_CAP)
static uint32_t s_next_id = 1;
static uint32_t s_rev;

static char     s_local_name[8] = "ECHO";
static char     s_peer_name[8]  = "DELTA";

static bool     s_time_synced;
static int64_t  s_epoch_base;
static int64_t  s_uptime_base_s;

static int64_t now_locked(void)
{
    if (!s_time_synced) return 0;
    int64_t up_s = esp_timer_get_time() / 1000000LL;
    return s_epoch_base + (up_s - s_uptime_base_s);
}

void msg_store_init(const char *local_name, const char *peer_name)
{
    if (!s_mu) s_mu = xSemaphoreCreateMutex();
    xSemaphoreTake(s_mu, portMAX_DELAY);
    memset(s_ring, 0, sizeof(s_ring));
    s_head = 0;
    s_count = 0;
    s_next_id = 1;
    s_rev = 0;
    if (local_name && *local_name) snprintf(s_local_name, sizeof(s_local_name), "%s", local_name);
    if (peer_name && *peer_name)   snprintf(s_peer_name, sizeof(s_peer_name), "%s", peer_name);
    xSemaphoreGive(s_mu);
}

static uint32_t add_locked(const char *from, msg_dir_t dir, msg_status_t status,
                           const char *text, size_t len)
{
    if (len > MSG_MAX_BYTES) len = MSG_MAX_BYTES;
    int idx;
    if (s_count < MSG_STORE_CAP) {
        idx = (s_head + s_count) % MSG_STORE_CAP;
        s_count++;
    } else {
        // Evict oldest entry cleanly while keeping chronological order
        idx = s_head;
        s_head = (s_head + 1) % MSG_STORE_CAP;
    }

    msg_t *m = &s_ring[idx];
    m->id     = s_next_id++;
    m->rev    = ++s_rev;
    m->time   = now_locked();
    m->dir    = dir;
    m->status = status;
    m->len    = (uint16_t)len;
    snprintf(m->from, sizeof(m->from), "%s", from);
    memcpy(m->text, text, len);
    m->text[len] = '\0';
    return m->id;
}

uint32_t msg_store_add_out(const char *text, size_t len)
{
    xSemaphoreTake(s_mu, portMAX_DELAY);
    uint32_t id = add_locked(s_local_name, MSG_DIR_OUT, MSG_STATUS_SENDING, text, len);
    xSemaphoreGive(s_mu);
    return id;
}

uint32_t msg_store_add_in(const char *text, size_t len)
{
    xSemaphoreTake(s_mu, portMAX_DELAY);
    uint32_t id = add_locked(s_peer_name, MSG_DIR_IN, MSG_STATUS_RECEIVED, text, len);
    xSemaphoreGive(s_mu);
    return id;
}

void msg_store_set_status(uint32_t id, msg_status_t status)
{
    xSemaphoreTake(s_mu, portMAX_DELAY);
    for (int i = 0; i < s_count; i++) {
        msg_t *m = &s_ring[(s_head + i) % MSG_STORE_CAP];
        if (m->id == id) {
            if (m->status != status) {
                m->status = status;
                m->rev = ++s_rev;
            }
            break;
        }
    }
    xSemaphoreGive(s_mu);
}

int msg_store_since(uint32_t since_rev, msg_t *out, int max_out, uint32_t *out_last_rev)
{
    int n = 0;
    xSemaphoreTake(s_mu, portMAX_DELAY);
    for (int i = 0; i < s_count && n < max_out; i++) {
        const msg_t *m = &s_ring[(s_head + i) % MSG_STORE_CAP];
        if (m->rev > since_rev) {
            out[n++] = *m;   // copy struct out; never expose internal pointers
        }
    }
    if (out_last_rev) *out_last_rev = s_rev;
    xSemaphoreGive(s_mu);
    return n;
}

int msg_store_batch(uint32_t since_rev, uint32_t max_rev, uint32_t after_id,
                    msg_t *out, int max_out)
{
    int n = 0;
    xSemaphoreTake(s_mu, portMAX_DELAY);
    for (int i = 0; i < s_count && n < max_out; i++) {
        const msg_t *m = &s_ring[(s_head + i) % MSG_STORE_CAP];
        if (m->id > after_id && m->rev > since_rev && m->rev <= max_rev) {
            out[n++] = *m;   // copy struct out; mutex released before any network I/O
        }
    }
    xSemaphoreGive(s_mu);
    return n;
}

uint32_t msg_store_last_rev(void)
{
    xSemaphoreTake(s_mu, portMAX_DELAY);
    uint32_t r = s_rev;
    xSemaphoreGive(s_mu);
    return r;
}

void msg_store_set_time(int64_t epoch_sec)
{
    if (epoch_sec <= 0) return;
    xSemaphoreTake(s_mu, portMAX_DELAY);
    s_epoch_base    = epoch_sec;
    s_uptime_base_s = esp_timer_get_time() / 1000000LL;
    s_time_synced   = true;
    xSemaphoreGive(s_mu);
}

int64_t msg_store_now(void)
{
    xSemaphoreTake(s_mu, portMAX_DELAY);
    int64_t t = now_locked();
    xSemaphoreGive(s_mu);
    return t;
}

const char *msg_status_str(msg_status_t status)
{
    switch (status) {
        case MSG_STATUS_SENDING:   return "sending";
        case MSG_STATUS_DELIVERED: return "delivered";
        case MSG_STATUS_FAILED:    return "failed";
        case MSG_STATUS_RECEIVED:  return "received";
        default:                   return "failed";
    }
}

const char *msg_dir_str(msg_dir_t dir)
{
    return dir == MSG_DIR_OUT ? "out" : "in";
}
