// HC-12 single-module test: talks to the module in AT command mode (SET low).
// Nothing is transmitted over the air.
//
// TX/RX wiring differs between builds of the hardware:
//   carrier PCB: GPIO10 -> J5 pin 4 (HC-12 TXD), GPIO20 -> J5 pin 3 (HC-12 RXD)  = "PCB" map
//   breadboard:  GPIO10 -> HC-12 RXD, GPIO20 -> HC-12 TXD                         = "straight" map
// The test tries both, keeps the one the module answers on and remembers it in NVS, so a
// board only ever tries the wrong one once (two pins briefly driving each other for ~2 ms).
#include <stdio.h>
#include "tlog.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_random.h"
#include "nvs.h"

#define HC12_UART      UART_NUM_1
#define HC12_SET_GPIO  6    // LOW = AT command mode, HIGH = normal radio mode
#define HC12_PIN_A     10
#define HC12_PIN_B     20
#define HC12_NVS_NS    "treeline"
#define HC12_NVS_KEY   "hc12_map"

enum { MAP_PCB = 0, MAP_STRAIGHT = 1 };
static const char *map_name[] = { "PCB (C3 TX GPIO20, RX GPIO10)", "straight (C3 TX GPIO10, RX GPIO20)" };

static bool uart_ready;
static int cur_map = -1;

static int map_tx(int m) { return m == MAP_PCB ? HC12_PIN_B : HC12_PIN_A; }
static int map_rx(int m) { return m == MAP_PCB ? HC12_PIN_A : HC12_PIN_B; }

static void set_map(int m)
{
    // the UART TX signal stays routed to an old pin unless that pin is released first
    gpio_reset_pin(HC12_PIN_A);
    gpio_reset_pin(HC12_PIN_B);
    ESP_ERROR_CHECK(uart_set_pin(HC12_UART, map_tx(m), map_rx(m), UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    cur_map = m;
    vTaskDelay(pdMS_TO_TICKS(20));
}

static int map_load(void)
{
    nvs_handle_t h;
    uint8_t v = 0xFF;
    if (nvs_open(HC12_NVS_NS, NVS_READONLY, &h) == ESP_OK) { nvs_get_u8(h, HC12_NVS_KEY, &v); nvs_close(h); }
    return v <= MAP_STRAIGHT ? v : -1;
}

static void map_save(int m)
{
    nvs_handle_t h;
    esp_err_t e = nvs_open(HC12_NVS_NS, NVS_READWRITE, &h);
    if (e == ESP_OK) { e = nvs_set_u8(h, HC12_NVS_KEY, (uint8_t)m); if (e == ESP_OK) e = nvs_commit(h); nvs_close(h); }
    printf("HC12: wiring saved as %s (%s)\n", map_name[m], esp_err_to_name(e));
}

static void set_baud(int baud)
{
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(HC12_UART, &cfg);
    vTaskDelay(pdMS_TO_TICKS(20));
}

// Send a command, collect the reply until 150 ms of silence (max ~700 ms).
int hc12_cmd(const char *cmd, char *resp, size_t resp_len)
{
    uart_flush_input(HC12_UART);
    uart_write_bytes(HC12_UART, cmd, strlen(cmd));
    int total = 0, idle_ms = 0, elapsed = 0;
    while (elapsed < 700 && idle_ms < 150 && total < (int)resp_len - 1) {
        int n = uart_read_bytes(HC12_UART, (uint8_t *)resp + total, resp_len - 1 - total, pdMS_TO_TICKS(50));
        if (n > 0) { total += n; idle_ms = 0; } else { idle_ms += 50; }
        elapsed += 50;
    }
    resp[total] = 0;
    for (int i = 0; i < total; i++) {            // make it printable on one line
        if (resp[i] == '\r' || resp[i] == '\n') resp[i] = ' ';
        else if (resp[i] < 0x20 || resp[i] > 0x7E) resp[i] = '.';
    }
    return total;
}

// Give SET a fresh HIGH->LOW edge, then send "AT". If the module is busy passing on a
// radio packet from the other unit at that moment it can miss the switch to command
// mode, so callers retry; the random wait makes a repeat collision unlikely.
bool hc12_try_at(char *resp, size_t resp_len)
{
    gpio_set_level(HC12_SET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(60 + esp_random() % 150));
    gpio_set_level(HC12_SET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));             // datasheet: >40 ms after SET goes low
    int n = hc12_cmd("AT", resp, resp_len);
    return n > 0 && strstr(resp, "OK");
}

void hc12_test(void)
{
    printf("--- HC-12 test: SET=GPIO%d, UART on GPIO%d/GPIO%d (TX/RX order found below) ---\n",
           HC12_SET_GPIO, HC12_PIN_A, HC12_PIN_B);

    // 1. Power/wiring: a powered HC-12 holds its TXD high and has a pull-up on RXD, so with
    //    the C3 pull-downs on, both UART pins read 1 whichever way round they are wired.
    gpio_config_t chk = {
        .pin_bit_mask = (1ULL << HC12_PIN_A) | (1ULL << HC12_PIN_B) | (1ULL << HC12_SET_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&chk);
    vTaskDelay(pdMS_TO_TICKS(5));
    int a = gpio_get_level(HC12_PIN_A), b = gpio_get_level(HC12_PIN_B), set_lvl = gpio_get_level(HC12_SET_GPIO);
    printf("HC12 pins with C3 pull-downs: GPIO%d=%d GPIO%d=%d SET GPIO%d=%d -> %s\n",
           HC12_PIN_A, a, HC12_PIN_B, b, HC12_SET_GPIO, set_lvl,
           a && b ? "HC-12 powered and both UART wires connected" :
           !a && !b ? "both LOW: HC-12 has no power or is not connected" : "one UART wire is not connected");

    // 2. Enter command mode
    gpio_reset_pin(HC12_SET_GPIO);
    gpio_set_direction(HC12_SET_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(HC12_SET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));             // datasheet: >40 ms after SET goes low

    if (!uart_ready) {
        ESP_ERROR_CHECK(uart_driver_install(HC12_UART, 1024, 0, 0, NULL, 0));
        uart_ready = true;
    }

    // 3. Find the TX/RX order at the factory 9600 baud: remembered order first, then the other.
    //    If SET might not be connected the module could be in radio mode and would transmit
    //    what we send: then only one "AT" per order and no baud search.
    char resp[200];
    bool found = false;
    int stored = map_load();
    int first = stored >= 0 ? stored : MAP_PCB;        // PCB v3 default: C3 TX=GPIO20, RX=GPIO10
    int found_baud = 9600;
    set_baud(9600);
    for (int k = 0; k < 2 && !found; k++) {
        int m = k == 0 ? first : !first;
        set_map(m);
        int tries = !set_lvl ? 1 : k == 0 ? 4 : 2;
        for (int t = 0; t < tries && !found; t++) {
            found = hc12_try_at(resp, sizeof(resp));
            if (!found && resp[0]) printf("HC12 %s: got '%s' (garbled)\n", map_name[m], resp);
        }
        if (!found) printf("HC12: no reply at 9600 baud with %s\n", map_name[m]);
    }
    // 4. Not at 9600: the module may have been set to another rate. Search with the first order.
    static const int bauds[] = { 19200, 38400, 57600, 115200, 4800, 2400, 1200 };
    if (!found && set_lvl) {
        set_map(first);
        for (int i = 0; i < (int)(sizeof(bauds) / sizeof(bauds[0])) && !found; i++) {
            set_baud(bauds[i]);
            found = hc12_try_at(resp, sizeof(resp));
            if (found) found_baud = bauds[i];
            else if (resp[0]) printf("HC12 %d baud: got '%s' (garbled, wrong baud)\n", bauds[i], resp);
        }
    }

    if (found) {
        printf("HC12 AT    -> '%s'  (module answers at %d baud, wiring %s)\n", resp, found_baud, map_name[cur_map]);
        if (cur_map != stored) map_save(cur_map);
        hc12_cmd("AT+V", resp, sizeof(resp));
        printf("HC12 AT+V  -> '%s'\n", resp);
        hc12_cmd("AT+RX", resp, sizeof(resp));
        printf("HC12 AT+RX -> '%s'\n", resp);
        if (!set_lvl) printf("HC12: SET works (module entered command mode); its pin just has no strong pull-up\n");
    } else {
        printf("HC12: NO REPLY with either TX/RX order%s. Check: SET on GPIO%d, HC-12 VCC/GND, "
               "UART wires on GPIO%d and GPIO%d\n", set_lvl ? " at any baud rate" : " at 9600 baud",
               HC12_SET_GPIO, HC12_PIN_A, HC12_PIN_B);
        set_baud(9600);
        set_map(first);                          // link.c keeps retrying with this order
        // (pin-map probe removed: it re-configured GPIO0-6, which are the button, CHRG and battery pins)
    }

    // 5. Back to normal mode (SET high). Module stays idle: nothing is sent.
    gpio_set_level(HC12_SET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    printf("------------------------\n");
}
