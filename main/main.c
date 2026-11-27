// TREELINE handheld firmware: OLED interface (ui.c, ported from the mockup) driven by real data.
//
//  UI task      ~30 fps: button edges -> ui_frame() -> SH1106 over I2C
//  button task  2 ms polling of GPIO5, reports edges within a few ms (lockout debounce)
//  app_main     init (radio, sensors) in the background while the title screen shows,
//               then a 50 ms loop: ping answers every tick, sensors/radio/battery once a second
//
// Pins (carrier PCB): SDA 7, SCL 9, button 5 (to GND), CHRG 3 (charger 5V via R3 10k, HIGH = charging),
// battery divider 1, HC-12 SET 6, UART on 10/20 (TX/RX order detected per board, see hc12.c).
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/temperature_sensor.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_idf_version.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "ui.h"
#include "link.h"
#include "ota.h"
#include "tlog.h"

#define I2C_SDA_GPIO    7
#define I2C_SCL_GPIO    9
#define I2C_HZ_FAST     400000   // ~30 fps on the OLED
#define I2C_HZ_SAFE     100000   // fallback if the bus is not happy at 400 kHz
#define OLED_ADDR       0x3C
#define BMP_ADDR_LO     0x76
#define BMP_ADDR_HI     0x77
#define OLED_COL_OFS    2        // SH1106 has 132 columns, panel starts at column 2
#define OLED_ROTATE_180 1        // both units: PCB mounts the screen upside down
#define PERFBOARD_MAC5  0x30     // MAC byte 5 of the unit named DELTA (was on the perfboard)
#define BTN_GPIO        5
#define CHRG_GPIO       3
#define CHRG_ACTIVE     1        // wired to the charger 5V (IN+): voltage present = charging; pulled down otherwise
#define BAT_ADC_CHAN    ADC_CHANNEL_1   // GPIO1
#define BAT_DIVIDER     2.0f
#define OTA_PASS        "treeline"   // Wi-Fi password in update mode (8+ characters)
#define FRAME_MS        25       // 40 fps target; the I2C send (~22 ms at 400 kHz) sets the real limit
#define HOME_ALT_M        19.0f  // true altitude (m) where the board first gets a settled reading (AUB)
#define ALT_SHOW_RELATIVE 1      // OLED ALT row: 1 = height above home (0 there), 0 = calibrated altitude
#define BARO_NVS_NS       "treeline"

static const char *TAG = "treeline";
static SemaphoreHandle_t ui_lock;
static SemaphoreHandle_t i2c_lock;     // OLED frames and BMP280 reads share the bus
static QueueHandle_t btn_q;
typedef struct { bool down; uint32_t ms; } btn_ev_t;

static uint32_t ms_now(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
#define LOCK()   xSemaphoreTake(ui_lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(ui_lock)
#define I2C_LOCK()   xSemaphoreTake(i2c_lock, portMAX_DELAY)
#define I2C_UNLOCK() xSemaphoreGive(i2c_lock)

// ---------------------------------------------------------------- I2C + SH1106
static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t oled, bmp;
static bool oled_ok, bmp_ok, oled_rot180;
static uint8_t bmp_addr;
static uint32_t i2c_hz = I2C_HZ_FAST;
static uint8_t pagebuf[2][8][128];   // double buffer: draw one while the other is sent
static uint8_t sent[8][128];          // what the panel shows now (skip unchanged pages)
static bool sent_valid;

static esp_err_t oled_cmds(const uint8_t *cmds, size_t n)
{
    uint8_t buf[32] = { 0x00 };
    memcpy(buf + 1, cmds, n);
    return i2c_master_transmit(oled, buf, n + 1, 50);
}

static esp_err_t oled_init(void)
{
    uint8_t c[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0xAD, 0x8B,
        0xA1, 0xC8,                     // segment remap / COM scan (index 10, 11)
        0xDA, 0x12, 0x81, 0xCF, 0xD9, 0x22, 0xDB, 0x35, 0xA4, 0xA6, 0xAF,
    };
    if (oled_rot180) { c[10] = 0xA0; c[11] = 0xC0; }
    return oled_cmds(c, sizeof(c));
}

static esp_err_t oled_flush(const uint8_t (*pages)[128])
{
    uint8_t buf[129];
    buf[0] = 0x40;
    for (int p = 0; p < 8; p++) {
        if (sent_valid && memcmp(sent[p], pages[p], 128) == 0) continue;   // page unchanged: skip it
        const uint8_t pos[] = { 0xB0 | p, 0x00 | (OLED_COL_OFS & 0x0F), 0x10 | (OLED_COL_OFS >> 4) };
        esp_err_t err = oled_cmds(pos, sizeof(pos));
        if (err != ESP_OK) { sent_valid = false; return err; }
        memcpy(buf + 1, pages[p], 128);
        err = i2c_master_transmit(oled, buf, sizeof(buf), 50);
        if (err != ESP_OK) { sent_valid = false; return err; }
        memcpy(sent[p], pages[p], 128);
    }
    sent_valid = true;
    return ESP_OK;
}

static void pack_frame(uint8_t (*pages)[128])   // 1 byte per pixel -> SH1106 pages (bit 0 = top row)
{
    const uint8_t *fb = ui_fb();
    for (int p = 0; p < 8; p++) {
        const uint8_t *r0 = fb + p * 8 * 128;
        for (int x = 0; x < 128; x++) {
            uint8_t b = 0;
            for (int r = 0; r < 8; r++) b |= (r0[r * 128 + x] ? 1 : 0) << r;
            pages[p][x] = b;
        }
    }
}

static void add_dev(uint8_t addr, i2c_master_dev_handle_t *h)
{
    if (*h) { i2c_master_bus_rm_device(*h); *h = NULL; }
    i2c_device_config_t cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addr, .scl_speed_hz = i2c_hz };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &cfg, h));
}

static bool oled_bringup(void)
{
    if (i2c_master_probe(bus, OLED_ADDR, 50) != ESP_OK) return false;
    add_dev(OLED_ADDR, &oled);
    if (oled_init() == ESP_OK) return true;
    i2c_hz = I2C_HZ_SAFE;                            // retry slower
    add_dev(OLED_ADDR, &oled);
    return oled_init() == ESP_OK;
}

// ---------------------------------------------------------------- BMP280 (Bosch reference compensation)
static struct { uint16_t T1; int16_t T2, T3; uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9; } cal;
static esp_err_t bmp_read(uint8_t reg, uint8_t *d, size_t n) { return i2c_master_transmit_receive(bmp, &reg, 1, d, n, 50); }
static esp_err_t bmp_write(uint8_t reg, uint8_t v) { uint8_t b[2] = { reg, v }; return i2c_master_transmit(bmp, b, 2, 50); }

static char bmp_err[28];

static bool bmp_try(uint8_t addr)
{
    if (bmp) { i2c_master_bus_rm_device(bmp); bmp = NULL; }
    // the BMP280 stays at 100 kHz like the test firmware that found it; only the OLED needs speed
    i2c_device_config_t cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addr, .scl_speed_hz = 100000 };
    if (i2c_master_bus_add_device(bus, &cfg, &bmp) != ESP_OK) return false;
    uint8_t id = 0;
    esp_err_t err = bmp_read(0xD0, &id, 1);
    printf("BMP280 @0x%02X: chip id %s 0x%02X\n", addr, esp_err_to_name(err), id);
    if (err != ESP_OK) { snprintf(bmp_err, sizeof(bmp_err), "BMP 0x%02X: %s", addr, err == ESP_ERR_TIMEOUT ? "timeout" : "no ACK"); return false; }
    if (id != 0x58 && id != 0x60 && id != 0x56 && id != 0x57) { snprintf(bmp_err, sizeof(bmp_err), "BMP 0x%02X: id 0x%02X?", addr, id); return false; }
    bmp_addr = addr;
    return true;
}

static bool bmp_init(void)
{
    bmp_addr = 0;
    if (!bmp_try(BMP_ADDR_HI)) {
        char e77[28]; snprintf(e77, sizeof(e77), "%s", bmp_err);   // 0x77 is the PCB address: report that one
        if (!bmp_try(BMP_ADDR_LO)) {
            snprintf(bmp_err, sizeof(bmp_err), "%s", e77);
            if (bmp) { i2c_master_bus_rm_device(bmp); bmp = NULL; }
            return false;
        }
    }
    uint8_t c[24];
    esp_err_t err = bmp_read(0x88, c, sizeof(c));
    if (err != ESP_OK) { snprintf(bmp_err, sizeof(bmp_err), "BMP calib %s", esp_err_to_name(err)); printf("%s\n", bmp_err); return false; }
#define U16(i) (uint16_t)(c[i] | (c[i + 1] << 8))
    cal.T1 = U16(0); cal.T2 = U16(2); cal.T3 = U16(4); cal.P1 = U16(6); cal.P2 = U16(8); cal.P3 = U16(10);
    cal.P4 = U16(12); cal.P5 = U16(14); cal.P6 = U16(16); cal.P7 = U16(18); cal.P8 = U16(20); cal.P9 = U16(22);
#undef U16
    if (cal.T1 == 0 || cal.P1 == 0) { snprintf(bmp_err, sizeof(bmp_err), "BMP calib empty"); printf("%s\n", bmp_err); return false; }
    if ((err = bmp_write(0xF5, 0x90)) != ESP_OK || (err = bmp_write(0xF4, 0x57)) != ESP_OK) {
        snprintf(bmp_err, sizeof(bmp_err), "BMP config %s", esp_err_to_name(err)); printf("%s\n", bmp_err);
        return false;
    }
    bmp_err[0] = 0;
    return true;
}

// Free a bus that a sensor may still be holding (e.g. reset in the middle of a read):
// 9 clocks then a STOP, done on plain GPIO before the I2C driver takes the pins.
static void i2c_bus_recover(void)
{
    gpio_config_t io = { .pin_bit_mask = (1ULL << I2C_SDA_GPIO) | (1ULL << I2C_SCL_GPIO),
                         .mode = GPIO_MODE_INPUT_OUTPUT_OD, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&io);
    gpio_set_level(I2C_SDA_GPIO, 1);
    for (int i = 0; i < 9; i++) {
        gpio_set_level(I2C_SCL_GPIO, 0); esp_rom_delay_us(5);
        gpio_set_level(I2C_SCL_GPIO, 1); esp_rom_delay_us(5);
    }
    gpio_set_level(I2C_SDA_GPIO, 0); esp_rom_delay_us(5);   // STOP: SDA rises while SCL is high
    gpio_set_level(I2C_SCL_GPIO, 1); esp_rom_delay_us(5);
    gpio_set_level(I2C_SDA_GPIO, 1); esp_rom_delay_us(5);
    printf("I2C lines after recovery: SDA=%d SCL=%d\n", gpio_get_level(I2C_SDA_GPIO), gpio_get_level(I2C_SCL_GPIO));
    gpio_reset_pin(I2C_SDA_GPIO);
    gpio_reset_pin(I2C_SCL_GPIO);
}

static bool bmp_measure(float *tc, float *hpa)
{
    uint8_t d[6];
    if (bmp_read(0xF7, d, 6) != ESP_OK) return false;
    int32_t adc_P = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4), adc_T = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4);
    if (adc_T == 0x80000 || adc_P == 0x80000) return false;
    int32_t v1 = ((((adc_T >> 3) - ((int32_t)cal.T1 << 1))) * cal.T2) >> 11;
    int32_t v2 = (((((adc_T >> 4) - (int32_t)cal.T1) * ((adc_T >> 4) - (int32_t)cal.T1)) >> 12) * cal.T3) >> 14;
    int32_t t_fine = v1 + v2;
    *tc = ((t_fine * 5 + 128) >> 8) / 100.0f;
    int64_t p1 = (int64_t)t_fine - 128000, p2 = p1 * p1 * cal.P6;
    p2 += (p1 * cal.P5) << 17; p2 += ((int64_t)cal.P4) << 35;
    p1 = ((p1 * p1 * cal.P3) >> 8) + ((p1 * cal.P2) << 12);
    p1 = ((((int64_t)1) << 47) + p1) * cal.P1 >> 33;
    if (p1 == 0) return false;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - p2) * 3125) / p1;
    p1 = (((int64_t)cal.P9) * (p >> 13) * (p >> 13)) >> 25;
    p2 = (((int64_t)cal.P8) * p) >> 19;
    p = ((p + p1 + p2) >> 8) + (((int64_t)cal.P7) << 4);
    *hpa = (p / 256.0f) / 100.0f;
    return true;
}

// ---------------------------------------------------------------- altitude calibration
// Sea-level pressure (p0) is worked out once from a settled reading taken at HOME_ALT_M and kept
// in NVS; it is redone if HOME_ALT_M changes. Weather moves p0 by up to +-20 hPa (~ +-170 m),
// so the reading drifts over days.
static float baro_p0 = 1013.25f;   // standard atmosphere until calibrated
static bool baro_cal;

static float alt_from(float hpa, float p0) { return 44330.0f * (1.0f - powf(hpa / p0, 0.1903f)); }

static void baro_load(void)
{
    nvs_handle_t h;
    if (nvs_open(BARO_NVS_NS, NVS_READONLY, &h) != ESP_OK) { printf("BARO: no calibration stored yet\n"); return; }
    int32_t p0_mhpa = 0, home_cm = 0;   // p0 in 1/1000 hPa
    bool ok = nvs_get_i32(h, "p0", &p0_mhpa) == ESP_OK && nvs_get_i32(h, "home_cm", &home_cm) == ESP_OK;
    nvs_close(h);
    if (!ok) { printf("BARO: no calibration stored yet\n"); return; }
    if (home_cm != (int32_t)lroundf(HOME_ALT_M * 100)) {
        printf("BARO: HOME_ALT_M changed (stored %.2f m, now %.2f m): recalibrating\n", home_cm / 100.0f, HOME_ALT_M);
        return;
    }
    if (p0_mhpa < 900000 || p0_mhpa > 1100000) { printf("BARO: stored p0 %ld out of range, recalibrating\n", (long)p0_mhpa); return; }
    baro_p0 = p0_mhpa / 1000.0f;
    baro_cal = true;
    printf("BARO: sea-level pressure %.2f hPa from NVS (home %.1f m)\n", baro_p0, HOME_ALT_M);
}

static void baro_calibrate(float hpa)
{
    baro_p0 = hpa / powf(1.0f - HOME_ALT_M / 44330.0f, 1.0f / 0.1903f);
    baro_cal = true;
    nvs_handle_t h;
    esp_err_t e = nvs_open(BARO_NVS_NS, NVS_READWRITE, &h);
    if (e == ESP_OK) {
        e = nvs_set_i32(h, "p0", (int32_t)lroundf(baro_p0 * 1000));
        if (e == ESP_OK) e = nvs_set_i32(h, "home_cm", (int32_t)lroundf(HOME_ALT_M * 100));
        if (e == ESP_OK) e = nvs_commit(h);
        nvs_close(h);
    }
    printf("BARO: calibrated at %.1f m: %.2f hPa -> sea level %.2f hPa (%s)\n",
           HOME_ALT_M, hpa, baro_p0, e == ESP_OK ? "saved" : esp_err_to_name(e));
}

// ---------------------------------------------------------------- battery
static adc_oneshot_unit_handle_t bat_adc;
static adc_cali_handle_t bat_cali;

static void bat_init(void)
{
    adc_oneshot_unit_init_cfg_t u = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&u, &bat_adc));
    adc_oneshot_chan_cfg_t c = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(bat_adc, BAT_ADC_CHAN, &c));
    adc_cali_curve_fitting_config_t k = { .unit_id = ADC_UNIT_1, .chan = BAT_ADC_CHAN, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    if (adc_cali_create_scheme_curve_fitting(&k, &bat_cali) != ESP_OK) bat_cali = NULL;
}

static int bat_pin_mv;   // last averaged voltage at GPIO1 itself (before the divider factor)

static float bat_volts(void)
{
    int sum = 0;
    for (int i = 0; i < 16; i++) {
        int raw, mv;
        if (adc_oneshot_read(bat_adc, BAT_ADC_CHAN, &raw) != ESP_OK) return -1;
        sum += (bat_cali && adc_cali_raw_to_voltage(bat_cali, raw, &mv) == ESP_OK) ? mv : raw * 2500 / 4095;
    }
    bat_pin_mv = sum / 16;
    return bat_pin_mv * BAT_DIVIDER / 1000.0f;
}

// CHRG input: pulled toward "not charging" so an idle or unconnected line never shows charging
static void chrg_pin_setup(void)
{
    gpio_config_t io = { .pin_bit_mask = 1ULL << CHRG_GPIO, .mode = GPIO_MODE_INPUT,
                         .pull_up_en = CHRG_ACTIVE ? GPIO_PULLUP_DISABLE : GPIO_PULLUP_ENABLE,
                         .pull_down_en = CHRG_ACTIVE ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE };
    gpio_config(&io);
}
static bool chrg_on(void) { return gpio_get_level(CHRG_GPIO) == CHRG_ACTIVE; }

static int adc_mv(adc_channel_t ch, adc_cali_handle_t cali)
{
    int sum = 0;
    for (int i = 0; i < 16; i++) {
        int raw, mv;
        if (adc_oneshot_read(bat_adc, ch, &raw) != ESP_OK) return -1;
        sum += (cali && adc_cali_raw_to_voltage(cali, raw, &mv) == ESP_OK) ? mv : raw * 2500 / 4095;
    }
    return sum / 16;
}

// Update mode only: wiring check for the battery divider (GPIO1) and CHRG (GPIO3), done by
// switching the C3's own ~45k pulls in. A floating GPIO1 follows them (~0 / ~3 V); on the
// 100k/100k divider with a live battery the pull-down reading stays near 1 V. CHRG is also read
// as a voltage with the pull-down on: high = charging (red LED on), low = not charging.
static void bat_pin_diag(void)
{
    static adc_cali_handle_t chg_cali;
    static bool chg_cali_tried;
    chrg_pin_setup();
    vTaskDelay(pdMS_TO_TICKS(5));
    int chg_lvl = gpio_get_level(CHRG_GPIO);
    adc_oneshot_chan_cfg_t c = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    adc_oneshot_config_channel(bat_adc, ADC_CHANNEL_3, &c);   // GPIO3 to analog (drops its pull)
    if (!chg_cali_tried) {
        chg_cali_tried = true;
        adc_cali_curve_fitting_config_t k = { .unit_id = ADC_UNIT_1, .chan = ADC_CHANNEL_3, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
        if (adc_cali_create_scheme_curve_fitting(&k, &chg_cali) != ESP_OK) chg_cali = NULL;
    }
    if (CHRG_ACTIVE) gpio_pulldown_en(CHRG_GPIO); else gpio_pullup_en(CHRG_GPIO);   // as the digital read
    vTaskDelay(pdMS_TO_TICKS(20));
    int chg_mv = adc_mv(ADC_CHANNEL_3, chg_cali);
    chrg_pin_setup();                                           // back to a digital input

    const gpio_num_t b = GPIO_NUM_1;
    gpio_pullup_dis(b); gpio_pulldown_dis(b); vTaskDelay(pdMS_TO_TICKS(50));
    int none = adc_mv(BAT_ADC_CHAN, bat_cali);
    gpio_pullup_en(b);  vTaskDelay(pdMS_TO_TICKS(50));
    int up = adc_mv(BAT_ADC_CHAN, bat_cali);
    gpio_pullup_dis(b); gpio_pulldown_en(b); vTaskDelay(pdMS_TO_TICKS(50));
    int down = adc_mv(BAT_ADC_CHAN, bat_cali);
    gpio_pulldown_dis(b); vTaskDelay(pdMS_TO_TICKS(50));

    printf("DIAG GPIO1: no pull %d mV, pull-up %d mV, pull-down %d mV -> %s\n", none, up, down,
           down < 150 && up > 2400 ? "follows the pulls: FLOATING or VSW at 0 V" :
           down > 600 ? "driven by the divider" : "unclear");
    printf("DIAG CHRG GPIO3: digital %d, %d mV with pull -> %s\n", chg_lvl, chg_mv,
           chg_mv > 1650 ? "high: charging" : chg_mv < 400 ? "low: not charging" : "part-way: unclear");
}

// One line for the serial monitor and the /log page: what the divider sees and how it is read.
static void bat_log(float v)
{
    printf("BATT: GPIO1 %d mV x%.1f = VSW %.2f V -> %s, CHRG GPIO%d=%d%s\n", bat_pin_mv, BAT_DIVIDER, v,
           v > 4.45f ? "USB level" : v > 2.5f ? "battery" : "nothing on VSW (switch off / wire link / divider?)",
           CHRG_GPIO, gpio_get_level(CHRG_GPIO), chrg_on() ? " (charging)" : "");
}

static int bat_percent(float v)
{
    static const struct { float v; int pct; } k[] = {
        {4.20f, 100}, {4.15f, 95}, {4.11f, 90}, {4.08f, 85}, {4.02f, 80}, {3.98f, 75}, {3.95f, 70}, {3.91f, 65},
        {3.87f, 60}, {3.85f, 55}, {3.84f, 50}, {3.82f, 45}, {3.80f, 40}, {3.79f, 35}, {3.77f, 30}, {3.75f, 25},
        {3.73f, 20}, {3.71f, 15}, {3.69f, 10}, {3.61f, 5}, {3.27f, 0},
    };
    const int n = sizeof(k) / sizeof(k[0]);
    if (v >= k[0].v) return 100;
    if (v <= k[n - 1].v) return 0;
    for (int i = 1; i < n; i++) if (v >= k[i].v) {
        float f = (v - k[i].v) / (k[i - 1].v - k[i].v);
        return (int)(k[i].pct + f * (k[i - 1].pct - k[i].pct) + 0.5f);
    }
    return 0;
}

// ---------------------------------------------------------------- button: report edges within a few ms
static void button_pin_setup(void)
{
    gpio_config_t io = { .pin_bit_mask = 1ULL << BTN_GPIO, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&io);
}

static void button_task(void *arg)
{
    button_pin_setup();
    vTaskDelay(pdMS_TO_TICKS(10));
    int state = gpio_get_level(BTN_GPIO);
    uint32_t last_change = 0;
    for (;;) {
        int raw = gpio_get_level(BTN_GPIO);
        uint32_t now = ms_now();
        // act on the first edge, then ignore contact bounce for 25 ms
        if (raw != state && now - last_change >= 25) {
            state = raw; last_change = now;
            btn_ev_t ev = { .down = (raw == 0), .ms = now };
            xQueueSend(btn_q, &ev, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(2));   // poll every 2 ms
    }
}

// ---------------------------------------------------------------- UI task
// ---- the I2C send runs in its own task, so the next frame is drawn while this one goes out
static SemaphoreHandle_t flush_go, flush_done;
static volatile int front;
static volatile uint32_t stat_flush_us, stat_frames;

static void flush_task(void *arg)
{
    uint32_t retry_at = 0;
    for (;;) {
        xSemaphoreTake(flush_go, portMAX_DELAY);
        int64_t t0 = esp_timer_get_time();
        I2C_LOCK();
        if (oled_ok) {
            if (oled_flush(pagebuf[front]) != ESP_OK) { oled_ok = false; retry_at = ms_now() + 1000; ESP_LOGW(TAG, "OLED write failed"); }
        } else if (ms_now() > retry_at) {
            oled_ok = oled_bringup();
            sent_valid = false;
            retry_at = ms_now() + 2000;
        }
        I2C_UNLOCK();
        stat_flush_us += (uint32_t)(esp_timer_get_time() - t0);
        xSemaphoreGive(flush_done);
    }
}

static void ui_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    int back = 0;
    uint32_t stat_render_us = 0;
    int64_t stat_t0 = esp_timer_get_time();
    for (;;) {
        btn_ev_t ev;
        int64_t t0 = esp_timer_get_time();
        LOCK();
        while (xQueueReceive(btn_q, &ev, 0) == pdTRUE) ui_button(ev.down, ev.ms);
        ui_frame(ms_now());
        pack_frame(pagebuf[back]);
        UNLOCK();
        stat_render_us += (uint32_t)(esp_timer_get_time() - t0);
        xSemaphoreTake(flush_done, portMAX_DELAY);    // previous frame fully sent
        front = back; back ^= 1;
        xSemaphoreGive(flush_go);                     // send this one while the next is drawn
        stat_frames++;
        if (esp_timer_get_time() - stat_t0 >= 10000000) {   // every 10 s: frame rate report
            float s = (esp_timer_get_time() - stat_t0) / 1e6f;
            printf("UI: %.1f fps, draw %.1f ms, send %.1f ms per frame\n", stat_frames / s,
                   stat_render_us / 1000.0f / stat_frames, stat_flush_us / 1000.0f / stat_frames);
            stat_frames = 0; stat_render_us = 0; stat_flush_us = 0; stat_t0 = esp_timer_get_time();
        }
        // steady pacing; always yield at least 1 ms so the sensor/radio loop gets the CPU
        TickType_t next = wake + pdMS_TO_TICKS(FRAME_MS), nowt = xTaskGetTickCount();
        if ((int32_t)(next - nowt) <= 0 || uxQueueMessagesWaiting(btn_q)) { vTaskDelay(1); wake = xTaskGetTickCount(); }
        else xTaskDelayUntil(&wake, pdMS_TO_TICKS(FRAME_MS));
    }
}

// ---------------------------------------------------------------- radio hooks
static char peer_name[8];
void ui_port_ping(void) { link_ping(); }
// Long press in update mode reboots while the button is still held, which would land straight
// back in update mode. This RTC word survives esp_restart() (not a power-off) and makes the next
// boot ignore the button once.
#define SKIP_UPD_MAGIC 0x5EB0071Du
static RTC_NOINIT_ATTR uint32_t skip_update_flag;
void ui_port_reboot(void) { printf("rebooting to normal mode\n"); skip_update_flag = SKIP_UPD_MAGIC; esp_restart(); }
static void on_ota_status(int pct, const char *msg) { LOCK(); ui_update_status(pct, msg); UNLOCK(); }   // called from ui_frame() with the lock held
static void on_ping_rx(void)                // HC-12 receive task, after answering the peer's ping
{
    LOCK();
    ui_ping_rx(peer_name);
    UNLOCK();
    printf("PING from %s answered\n", peer_name);
}
static void on_message(const char *text)    // HC-12 receive task
{
    LOCK();
    ui_message(peer_name, text);
    UNLOCK();
    printf("MSG from %s: %s\n", peer_name, text);
}

// ---------------------------------------------------------------- main
void app_main(void)
{
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    bool unit_delta = mac[5] == PERFBOARD_MAC5;
    oled_rot180 = OLED_ROTATE_180;   // both units on the PCB now: same screen orientation
    const char *me = unit_delta ? "DELTA" : "ECHO";
    snprintf(peer_name, sizeof(peer_name), "%s", unit_delta ? "ECHO" : "DELTA");

    tl_log_hook_esp_log();
    {
        static const char *rr[] = { "unknown", "power-on", "external pin", "software", "panic", "interrupt watchdog",
                                    "task watchdog", "other watchdog", "deep sleep", "brownout", "SDIO", "USB", "JTAG",
                                    "efuse", "power glitch", "CPU lockup" };
        esp_reset_reason_t r = esp_reset_reason();
        printf("TREELINE %s: reset reason %s (%d)\n", me, (unsigned)r < sizeof(rr) / sizeof(rr[0]) ? rr[r] : "?", (int)r);
    }
    bool skip_update = skip_update_flag == SKIP_UPD_MAGIC && esp_reset_reason() == ESP_RST_SW;
    skip_update_flag = 0;
    // Button held while switching on (~0.3 s) -> Wi-Fi update mode
    gpio_config_t bio = { .pin_bit_mask = 1ULL << BTN_GPIO, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&bio);
    vTaskDelay(pdMS_TO_TICKS(20));
    bool update_mode = !skip_update;
    if (skip_update) printf("TREELINE %s: left update mode, button ignored this boot\n", me);
    for (int i = 0; i < 10 && update_mode; i++) { if (gpio_get_level(BTN_GPIO)) update_mode = false; vTaskDelay(pdMS_TO_TICKS(30)); }

    ui_lock = xSemaphoreCreateMutex();
    i2c_lock = xSemaphoreCreateMutex();
    btn_q = xQueueCreate(16, sizeof(btn_ev_t));
    ui_init("TREELINE");
    ui_data_t *S = ui_data();
    snprintf(S->me, sizeof(S->me), "%s", me);
    snprintf(S->peer, sizeof(S->peer), "%s", peer_name);
    snprintf(S->os_ver, sizeof(S->os_ver), "ESP-IDF %d.%d", ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR);
    snprintf(S->boot_line[0], 24, "HC-12 433.4MHz CH001");
    snprintf(S->boot_line[1], 24, "SH1106 128x64 I2C");
    snprintf(S->boot_line[2], 24, "BMP280 ...");
    snprintf(S->boot_line[3], 24, "BUTTON GPIO%d", BTN_GPIO);
    snprintf(S->boot_line[4], 24, "BATTERY ADC GPIO1");
    snprintf(S->boot_line[5], 24, "CHARGER CHRG GPIO%d", CHRG_GPIO);
    snprintf(S->boot_line[6], 24, "peer %s ...", peer_name);

    // screen first, so the title shows while everything else starts
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    i2c_master_bus_config_t bc = {
        .i2c_port = -1, .sda_io_num = I2C_SDA_GPIO, .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true,
    };
    i2c_bus_recover();
    ESP_ERROR_CHECK(i2c_new_master_bus(&bc, &bus));
    oled_ok = oled_bringup();
    bmp_ok = bmp_init();                             // before the UI task starts using the bus
    printf("TREELINE %s: OLED %s at %lu Hz, BMP280 %s", me, oled_ok ? "ok" : "MISSING", (unsigned long)i2c_hz, bmp_ok ? "ok" : "NOT FOUND");
    if (bmp_ok) printf(" at 0x%02X", bmp_addr);
    printf("\n");
    xTaskCreate(button_task, "btn", 2048, NULL, 7, NULL);
    flush_go = xSemaphoreCreateBinary();
    flush_done = xSemaphoreCreateBinary();
    xSemaphoreGive(flush_done);
    xTaskCreate(flush_task, "oled", 3072, NULL, 3, NULL);
    xTaskCreate(ui_task, "ui", 6144, NULL, 3, NULL);
    vTaskPrioritySet(NULL, 4);   // this loop is light (sleeps 50 ms) but must never be starved by the UI

    if (update_mode) {
        char ssid[24];
        snprintf(ssid, sizeof(ssid), "TREELINE-%s", me);
        printf("TREELINE %s: button held at power-on -> Wi-Fi update mode\n", me);
        LOCK(); ui_update_mode(ssid, OTA_PASS, "192.168.4.1"); UNLOCK();
        ota_start(ssid, OTA_PASS, on_ota_status);
        // battery check that works on battery alone: read it at http://192.168.4.1/log
        chrg_pin_setup();
        bat_init();
        for (;;) { bat_log(bat_volts()); bat_pin_diag(); vTaskDelay(pdMS_TO_TICKS(5000)); }   // web server + UI do the rest
    }

    // the rest starts behind the title screen
    esp_err_t ne = nvs_flash_init();
    if (ne == ESP_ERR_NVS_NO_FREE_PAGES || ne == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); ne = nvs_flash_init(); }
    if (ne == ESP_OK) baro_load(); else printf("NVS init failed: %s (altitude not calibrated)\n", esp_err_to_name(ne));
    chrg_pin_setup();
    temperature_sensor_handle_t tsens = NULL;
    temperature_sensor_config_t tc = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
    ESP_ERROR_CHECK(temperature_sensor_install(&tc, &tsens));
    ESP_ERROR_CHECK(temperature_sensor_enable(tsens));
    bat_init();
    hc12_test();
    // nothing else may own these pins: make sure the button and CHRG still have their pulls
    button_pin_setup();
    chrg_pin_setup();
    link_set_msg_cb(on_message);
    link_set_ping_cb(on_ping_rx);
    bool radio = link_start();
    // give the peer a moment to show up for the self-test line
    for (int i = 0; i < 30; i++) { link_stats_t st; link_stats(&st); if (st.peer_seen) break; if (i % 10 == 0) link_send(); vTaskDelay(pdMS_TO_TICKS(100)); }
    link_stats_t st0; link_stats(&st0);
    float v0 = bat_volts();

    LOCK();
    S->radio_ok = radio;
    S->boot_ok[0] = radio;
    S->boot_ok[1] = oled_ok;
    if (bmp_ok) snprintf(S->boot_line[2], 24, "BMP280 @0x%02X", bmp_addr); else snprintf(S->boot_line[2], 24, "%.23s", bmp_err);
    S->boot_ok[2] = bmp_ok;
    S->boot_ok[3] = gpio_get_level(BTN_GPIO) == 1;      // idles HIGH when released
    S->boot_ok[4] = v0 > 2.5f;
    if (v0 > 4.45f) snprintf(S->boot_line[4], 24, "BATTERY: USB power");
    else if (v0 <= 2.5f) snprintf(S->boot_line[4], 24, "BATTERY not found");
    S->boot_ok[5] = true;
    snprintf(S->boot_line[6], 24, st0.peer_seen ? "peer %s found" : "peer %s not heard", peer_name);
    S->boot_ok[6] = st0.peer_seen;
    S->boot_ready = true;
    UNLOCK();
    ota_mark_good();   // started fine: keep this firmware (otherwise the bootloader rolls back)
    printf("TREELINE %s: radio %s, BMP %s, battery %.2f V, peer %s\n", me, radio ? "ok" : "FAIL",
           bmp_ok ? "ok" : "missing", v0, st0.peer_seen ? "found" : "not heard");

    bool charging = chrg_on(), chg_cand = charging, warned_low = false;
    LOCK(); S->charging = charging; UNLOCK();
    int64_t next_sec = esp_timer_get_time(), next_hist = 0, next_hb = 0;
    float v_avg = -1;         // smoothed battery voltage (-1 = no battery reading yet)
    float v_last = v0;        // last battery reading taken while the radio was quiet
    int pct_shown = -1;       // battery % on screen, changed only by real moves
    int low_s = 0;            // seconds in a row the battery has read <= 10 %
    float settle[5];          // last pressures: calibrate once they agree (sensor filter settled)
    int nsettle = 0;
    for (;;) {
        int rtt;
        if (link_ping_poll(&rtt)) {
            LOCK(); ui_ping_result(true, rtt); UNLOCK();
            printf("PING %s: %d ms\n", peer_name, rtt);
        }
        int64_t now = esp_timer_get_time();
        if (now >= next_hb) {                           // heartbeat ~1 s, jittered so the units rarely collide
            link_send();
            next_hb = now + 900000 + esp_random() % 400000;
        }
        if (now >= next_sec) {
            next_sec += 1000000;
            float t = 0, p = 0, chip = 0;
            I2C_LOCK();
            if (!bmp_ok && (now / 1000000) % 5 == 0) {          // not found yet: look again every 5 s
                bmp_ok = bmp_init();
                if (bmp_ok) printf("BMP280 found at 0x%02X\n", bmp_addr);
            }
            bool have_p = bmp_ok && bmp_measure(&t, &p);
            if (bmp_ok && !have_p && (now / 1000000) % 10 == 0) printf("BMP280: no reading yet\n");
            I2C_UNLOCK();
            if (have_p && !baro_cal && ne == ESP_OK) {
                settle[nsettle % 5] = p; nsettle++;
                if (nsettle >= 5) {
                    float lo = settle[0], hi = settle[0];
                    for (int i = 1; i < 5; i++) { lo = fminf(lo, settle[i]); hi = fmaxf(hi, settle[i]); }
                    if (hi - lo < 0.10f) baro_calibrate(p);
                }
            }
            float alt_abs = have_p ? alt_from(p, baro_p0) : 0;
            if (have_p && (now / 1000000) % 30 == 0)
                printf("BARO: %.2f hPa, altitude %.1f m, above home %+.1f m%s\n", p, alt_abs, alt_abs - HOME_ALT_M,
                       baro_cal ? "" : " (not calibrated yet)");
            temperature_sensor_get_celsius(tsens, &chip);
            // the HC-12 TX burst pulls the battery down (and its RF gets into the divider):
            // read only when the radio has been quiet for 150 ms, else reuse the last reading
            if (esp_timer_get_time() - link_last_tx_us() > 150000) v_last = bat_volts();
            float v = v_last;
            if ((now / 1000000) % 10 == 0) { bat_log(v); link_print(); }
            int lvl = chrg_on();
            link_stats_t st; link_stats(&st);

            LOCK();
            S->uptime = (uint32_t)(now / 1000000);
            S->heap_free = esp_get_free_heap_size();
            S->tCpu = chip;
            S->press_ok = have_p;
            snprintf(S->baro_err, sizeof(S->baro_err), "%s", bmp_ok ? (have_p ? "" : "BMP: no reading yet") : bmp_err);
            if (have_p) {
                S->press = p; S->tAir = t;
                S->alt = ALT_SHOW_RELATIVE ? alt_abs - HOME_ALT_M : alt_abs;
                if (now >= next_hist) {                 // one point every 5 min, 3 h shown
                    if (S->nhist == UI_HIST) { memmove(S->hist, S->hist + 1, sizeof(float) * (UI_HIST - 1)); S->nhist--; }
                    S->hist[S->nhist++] = p;
                    next_hist = now + 300LL * 1000000;
                }
            }
            S->usb_power = v > 4.45f;
            S->batt_ok = v > 2.5f && !S->usb_power;
            // The Li-ion curve is flat mid-range (3.80 V = 40 %, 3.84 V = 50 %), so ADC noise and the
            // dip while the radio transmits made the % jump. Average over ~10 s, then only move the
            // shown % by 2+ points, and while discharging only let it rise by 4+.
            if (S->batt_ok) {
                v_avg = v_avg < 0 ? v : v_avg + 0.1f * (v - v_avg);
                int p = bat_percent(v_avg), d = p - pct_shown;
                if (pct_shown < 0 || d <= -2 || d >= (charging ? 2 : 4)) pct_shown = p;
            } else { v_avg = -1; pct_shown = -1; }
            S->battV = S->batt_ok ? v_avg : v;
            S->batt = S->batt_ok ? pct_shown : 0;
            // CHRG must read the same twice in a row before it counts (it flickers with no battery)
            if (lvl == chg_cand && lvl != charging) { charging = lvl; ui_set_charging(charging); }
            chg_cand = lvl;
            // low only after 30 s in a row at <= 10 %, so one bad reading cannot raise the warning
            low_s = (S->batt_ok && !charging && S->batt <= 10) ? low_s + 1 : 0;
            S->low = low_s >= 30;
            if (S->low && !warned_low) { warned_low = true; ui_warn(0); printf("BATT: low battery warning at %d %% (%.2f V)\n", S->batt, S->battV); }
            if (!S->low && S->batt > 15) warned_low = false;
            S->radio_ok = st.ok;
            S->linkUp = st.peer_seen && st.heard_s >= 0 && st.heard_s < 6;
            S->seen = st.peer_seen ? (int)st.heard_s : -1;
            S->tx = (int)st.tx; S->rx = (int)st.rx; S->loss = (int)(st.loss + 0.5f);
            UNLOCK();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
