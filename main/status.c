// Thread-safe telemetry snapshot (see status.h).
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "status.h"

static SemaphoreHandle_t s_mu;
static status_t s_cur;

void status_init(const char *device, const char *peer)
{
    if (!s_mu) s_mu = xSemaphoreCreateMutex();
    xSemaphoreTake(s_mu, portMAX_DELAY);
    memset(&s_cur, 0, sizeof(s_cur));
    if (device) snprintf(s_cur.device, sizeof(s_cur.device), "%s", device);
    if (peer)   snprintf(s_cur.peer,   sizeof(s_cur.peer),   "%s", peer);
    s_cur.last_heard_s = -1;
    s_cur.ping_ms      = -1;
    snprintf(s_cur.pressure_trend, sizeof(s_cur.pressure_trend), "steady");
    xSemaphoreGive(s_mu);
}

void status_set(const status_t *in)
{
    if (!s_mu || !in) return;
    xSemaphoreTake(s_mu, portMAX_DELAY);
    s_cur = *in;
    xSemaphoreGive(s_mu);
}

void status_get(status_t *out)
{
    if (!out) return;
    if (!s_mu) { memset(out, 0, sizeof(*out)); return; }
    xSemaphoreTake(s_mu, portMAX_DELAY);
    *out = s_cur;
    xSemaphoreGive(s_mu);
}
