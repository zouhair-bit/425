// RAM copy of the console log (see tlog.h)
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#define TL_SIZE 6144
static char ring[TL_SIZE];
static size_t head, used;
static uint32_t total_written;
static SemaphoreHandle_t mu;

static void append(const char *s, size_t n)
{
    if (!mu) mu = xSemaphoreCreateMutex();
    xSemaphoreTake(mu, portMAX_DELAY);
    for (size_t i = 0; i < n; i++) {
        ring[head] = s[i];
        head = (head + 1) % TL_SIZE;
        if (used < TL_SIZE) used++;
        total_written++;
    }
    xSemaphoreGive(mu);
}

static int tl_vprintf(const char *fmt, va_list ap)
{
    char buf[256];
    va_list cp;
    va_copy(cp, ap);
    int n = vsnprintf(buf, sizeof(buf), fmt, cp);
    va_end(cp);
    if (n > 0) append(buf, n < (int)sizeof(buf) ? (size_t)n : sizeof(buf) - 1);
    return vprintf(fmt, ap);                  // still goes to the USB serial monitor
}

int tl_printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int n = tl_vprintf(fmt, ap);
    va_end(ap);
    return n;
}

size_t tl_log_copy(char *out, size_t n)
{
    if (!mu || !n) return 0;
    xSemaphoreTake(mu, portMAX_DELAY);
    size_t len = used < n - 1 ? used : n - 1;
    size_t start = (head + TL_SIZE - len) % TL_SIZE;
    for (size_t i = 0; i < len; i++) out[i] = ring[(start + i) % TL_SIZE];
    out[len] = 0;
    xSemaphoreGive(mu);
    return len;
}

void tl_log_range(uint32_t *start_pos, uint32_t *end_pos)
{
    if (!mu) { *start_pos = *end_pos = 0; return; }
    xSemaphoreTake(mu, portMAX_DELAY);
    *end_pos = total_written;
    *start_pos = total_written - (uint32_t)used;
    xSemaphoreGive(mu);
}

// Copies up to max_len bytes starting at *pos (up to end_pos) and advances *pos.
// Mutex is acquired and released per slice so callers never hold it across network I/O.
size_t tl_log_copy_slice(uint32_t *pos, uint32_t end_pos, char *out, size_t max_len)
{
    if (!mu || !out || !max_len) return 0;
    xSemaphoreTake(mu, portMAX_DELAY);
    uint32_t oldest = total_written - (uint32_t)used;
    if ((int32_t)(*pos - oldest) < 0) *pos = oldest;
    if ((int32_t)(end_pos - total_written) > 0) end_pos = total_written;
    int32_t avail = (int32_t)(end_pos - *pos);
    size_t n = 0;
    if (avail > 0) {
        n = (size_t)avail < max_len ? (size_t)avail : max_len;
        uint32_t offset_from_oldest = *pos - oldest;
        size_t ring_start = (head + TL_SIZE - used + offset_from_oldest) % TL_SIZE;
        for (size_t i = 0; i < n; i++) out[i] = ring[(ring_start + i) % TL_SIZE];
        *pos += (uint32_t)n;
    }
    xSemaphoreGive(mu);
    return n;
}

void tl_log_hook_esp_log(void) { esp_log_set_vprintf(tl_vprintf); }
