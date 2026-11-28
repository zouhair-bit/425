// Console log that is also kept in RAM, so it can be read over Wi-Fi (http://192.168.4.1/log).
// Including this after <stdio.h> sends every printf() in that file through tl_printf().
#pragma once
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>

int tl_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
size_t tl_log_copy(char *out, size_t n);   // oldest -> newest, returns length
void tl_log_range(uint32_t *start_pos, uint32_t *end_pos);
size_t tl_log_copy_slice(uint32_t *pos, uint32_t end_pos, char *out, size_t max_len);
void tl_log_hook_esp_log(void);            // also capture ESP_LOGx output

#define printf(...) tl_printf(__VA_ARGS__)
