// TREELINE handheld firmware: OLED interface, sensors, HC-12 radio link,
// Wi-Fi SoftAP, chat web server, and OTA updates (carrier PCB v3).
//
//  UI task       ~30 fps: button edges & incoming chat -> ui_frame() -> SH1106 over I2C
//  button task   GPIO5 interrupt + esp_timer_get_time() software debounce (short/double/long)
//  router task   routes radio events (DELIVERED/FAILED/RECEIVED) -> msg_store + OLED queue
//  link tasks    HC-12 RX parser + TX retry task (0xAA 0x55 framing, CRC-16, ACK, 3 retries)
//  app_main      init + 50 ms main loop (heartbeat, auto-ping, sensors, battery, shutdown)
//
// Carrier PCB v3 pinout (ESP32-C3 SuperMini):
//   I2C (OLED 0x3C + BMP280 0x77): SDA = GPIO7, SCL = GPIO9 (GPIO8 unconnected)
//   Push button (to GND):          GPIO5 (GPIO ISR + software debounce, active LOW)
//   Battery ADC (100k/100k div):   GPIO1 (ADC1_CH1, reads VSW / 2 after power switch)
//   Charger sense (5V IN+ via 10k):GPIO3 (HIGH = charging, internal pull-down enabled)
//   HC-12 radio (9600 baud):       SET = GPIO6, TX = GPIO20 (-> RXD), RX = GPIO10 (<- TXD)
//   DS18B20:                       not installed on PCB v3 (battery.temp_c = null, OLED = --)
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
#include "esp_sleep.h"
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
#include "app_queues.h"
#include "wifi_ap.h"
#include "msg_store.h"
#include "router.h"
#include "status.h"
#include "web_server.h"

// Carrier PCB v3 pinout
#define I2C_SDA_GPIO       GPIO_NUM_7
#define I2C_SCL_GPIO       GPIO_NUM_9
#define I2C_HZ_FAST        400000
#define I2C_HZ_SAFE        100000
#define OLED_ADDR          0x3C
#define BMP_ADDR_HI        0x77          // PCB v3: CSB and SDO tied to 3V3 -> 0x77
#define BMP_ADDR_LO        0x76          // fallback address
#define OLED_COL_OFS       2
#define OLED_ROTATE_180    1             // both ECHO and DELTA screens mounted 180 deg
#define PERFBOARD_MAC5     0x30          // unit 1 (DELTA)

#define BTN_GPIO           GPIO_NUM_5    // active LOW push button (wake / ping / sleep)
#define BTN_DEBOUNCE_US    25000LL       // 25 ms software debounce via esp_timer_get_time()
#define CHRG_GPIO          GPIO_NUM_3    // charger 5V IN+ via 10k: HIGH = charging
#define CHRG_ACTIVE        1             // HIGH = charging (pulled down when unplugged)
#define BAT_ADC_CHAN       ADC_CHANNEL_1 // GPIO1 (100k/100k divider, VSW / 2)
#define BAT_DIVIDER        2.0f

#define WIFI_PASS          "treeline"    // WPA2 SoftAP password (8+ chars, shared normal & OTA)
#define FRAME_MS           25
#define HOME_ALT_M         19.0f         // calibration altitude (m)
#define ALT_SHOW_RELATIVE  1
#define BARO_NVS_NS        "treeline"

#define BATT_SHUTDOWN_V    3.50f         // clean shutdown below 3.5 V (3 consecutive readings)
#define BATT_MIN_VALID_V   2.50f         // below this = no battery / floating divider
#define STORM_DROP_3H_HPA  3.0f          // rapid 3h pressure drop threshold for "storm"
#define STORM_DROP_10M_HPA 1.5f          // rapid 10m pressure drop threshold for "storm"

static const char *TAG = "treeline";
static SemaphoreHandle_t ui_lock;
static SemaphoreHandle_t i2c_lock;
static QueueHandle_t     btn_q;
static TaskHandle_t      btn_task_handle;
typedef struct { bool down; uint32_t ms; } btn_ev_t;

static uint32_t ms_now(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
#define LOCK()       xSemaphoreTake(ui_lock, portMAX_DELAY)
#define UNLOCK()     xSemaphoreGive(ui_lock)
#define I2C_LOCK()   xSemaphoreTake(i2c_lock, portMAX_DELAY)
#define I2C_UNLOCK() xSemaphoreGive(i2c_lock)

// ---------------------------------------------------------------- I2C + SH1106
static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t oled, bmp;
static bool     oled_ok, bmp_ok, oled_rot180;
static uint8_t  bmp_addr;
static uint32_t i2c_hz = I2C_HZ_FAST;
static uint8_t  pagebuf[2][8][128];
static uint8_t  sent[8][128];
static bool     sent_valid;

static esp_err_t oled_cmds(const uint8_t *cmds, size_t n)
{
    if (!oled) return ESP_ERR_INVALID_STATE;
    uint8_t buf[32] = { 0x00 };
    memcpy(buf + 1, cmds, n);
    return i2c_master_transmit(oled, buf, n + 1, 50);
}

static esp_err_t oled_init(void)
{
    uint8_t c[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0xAD, 0x8B,
        0xA1, 0xC8,
        0xDA, 0x12, 0x81, 0xCF, 0xD9, 0x22, 0xDB, 0x35, 0xA4, 0xA6, 0xAF,
    };
    if (oled_rot180) { c[10] = 0xA0; c[11] = 0xC0; }
    return oled_cmds(c, sizeof(c));
}

static void oled_power_off(void)
{
    I2C_LOCK();
    if (oled_ok && oled) {
        const uint8_t off_cmd = 0xAE;
        oled_cmds(&off_cmd, 1);
    }
    I2C_UNLOCK();
}

static esp_err_t oled_flush(const uint8_t (*pages)[128])
{
    if (!oled) return ESP_ERR_INVALID_STATE;
    uint8_t buf[129];
    buf[0] = 0x40;
    for (int p = 0; p < 8; p++) {
        if (sent_valid && memcmp(sent[p], pages[p], 128) == 0) continue;
        const uint8_t pos[] = { (uint8_t)(0xB0 | p), (uint8_t)(0x00 | (OLED_COL_OFS & 0x0F)), (uint8_t)(0x10 | (OLED_COL_OFS >> 4)) };
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

static void pack_frame(uint8_t (*pages)[128])
{
    const uint8_t *fb = ui_fb();
    for (int p = 0; p < 8; p++) {
        const uint8_t *r0 = fb + p * 8 * 128;
        for (int x = 0; x < 128; x++) {
            uint8_t b = 0;
            for (int r = 0; r < 8; r++) b |= (uint8_t)((r0[r * 128 + x] ? 1 : 0) << r);
            pages[p][x] = b;
        }
    }
}

static bool add_dev(uint8_t addr, i2c_master_dev_handle_t *h)
{
    if (!bus) return false;
    if (*h) { i2c_master_bus_rm_device(*h); *h = NULL; }
    i2c_device_config_t cfg = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addr, .scl_speed_hz = i2c_hz };
    return i2c_master_bus_add_device(bus, &cfg, h) == ESP_OK;
}

static bool oled_bringup(void)
{
    if (!bus) return false;
    if (i2c_master_probe(bus, OLED_ADDR, 50) != ESP_OK) return false;
    if (!add_dev(OLED_ADDR, &oled)) return false;
    if (oled_init() == ESP_OK) return true;
    i2c_hz = I2C_HZ_SAFE;
    if (!add_dev(OLED_ADDR, &oled)) return false;
    return oled_init() == ESP_OK;
}

// ---------------------------------------------------------------- BMP280 (0x77 primary, 0x76 fallback)
static struct { uint16_t T1; int16_t T2, T3; uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9; } cal;
static esp_err_t bmp_read(uint8_t reg, uint8_t *d, size_t n) { return i2c_master_transmit_receive(bmp, &reg, 1, d, n, 50); }
static esp_err_t bmp_write(uint8_t reg, uint8_t v) { uint8_t b[2] = { reg, v }; return i2c_master_transmit(bmp, b, 2, 50); }

static char bmp_err[28];

static bool bmp_try(uint8_t addr)
{
    if (!bus) { snprintf(bmp_err, sizeof(bmp_err), "I2C bus offline"); return false; }
    if (bmp) { i2c_master_bus_rm_device(bmp); bmp = NULL; }
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
    if (!bus) { snprintf(bmp_err, sizeof(bmp_err), "I2C bus offline"); return false; }
    // PCB v3 ties CSB and SDO to 3V3 -> probe 0x77 first, fall back to 0x76
    if (!bmp_try(BMP_ADDR_HI)) {
        char e77[28]; snprintf(e77, sizeof(e77), "%s", bmp_err);
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
    cal.T1 = U16(0); cal.T2 = (int16_t)U16(2); cal.T3 = (int16_t)U16(4); cal.P1 = U16(6);
    cal.P2 = (int16_t)U16(8); cal.P3 = (int16_t)U16(10); cal.P4 = (int16_t)U16(12);
    cal.P5 = (int16_t)U16(14); cal.P6 = (int16_t)U16(16); cal.P7 = (int16_t)U16(18);
    cal.P8 = (int16_t)U16(20); cal.P9 = (int16_t)U16(22);
#undef U16
    if (cal.T1 == 0 || cal.P1 == 0) { snprintf(bmp_err, sizeof(bmp_err), "BMP calib empty"); printf("%s\n", bmp_err); return false; }
    if ((err = bmp_write(0xF5, 0x90)) != ESP_OK || (err = bmp_write(0xF4, 0x57)) != ESP_OK) {
        snprintf(bmp_err, sizeof(bmp_err), "BMP config %s", esp_err_to_name(err)); printf("%s\n", bmp_err);
        return false;
    }
    bmp_err[0] = 0;
    return true;
}

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
    gpio_set_level(I2C_SDA_GPIO, 0); esp_rom_delay_us(5);
    gpio_set_level(I2C_SCL_GPIO, 1); esp_rom_delay_us(5);
    gpio_set_level(I2C_SDA_GPIO, 1); esp_rom_delay_us(5);
    printf("I2C lines after recovery: SDA=%d SCL=%d\n", gpio_get_level(I2C_SDA_GPIO), gpio_get_level(I2C_SCL_GPIO));
    gpio_reset_pin(I2C_SDA_GPIO);
    gpio_reset_pin(I2C_SCL_GPIO);
}

static bool bmp_measure(float *tc, float *hpa)
{
    uint8_t d[6];
    if (!bmp || bmp_read(0xF7, d, 6) != ESP_OK) return false;
    int32_t adc_P = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4);
    int32_t adc_T = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4);
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
    float val = (p / 256.0f) / 100.0f;
    if (val < 300.0f || val > 1200.0f) return false;
    *hpa = val;
    return true;
}

// ---------------------------------------------------------------- altitude calibration
static float baro_p0 = 1013.25f;
static bool  baro_cal;

static float alt_from(float hpa, float p0) { return 44330.0f * (1.0f - powf(hpa / p0, 0.1903f)); }

static void baro_load(void)
{
    nvs_handle_t h;
    if (nvs_open(BARO_NVS_NS, NVS_READONLY, &h) != ESP_OK) { printf("BARO: no calibration stored yet\n"); return; }
    int32_t p0_mhpa = 0, home_cm = 0;
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

// ---------------------------------------------------------------- battery & charger
static adc_oneshot_unit_handle_t bat_adc;
static adc_cali_handle_t         bat_cali;

static void bat_init(void)
{
    adc_oneshot_unit_init_cfg_t u = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&u, &bat_adc));
    adc_oneshot_chan_cfg_t c = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(bat_adc, BAT_ADC_CHAN, &c));
    adc_cali_curve_fitting_config_t k = { .unit_id = ADC_UNIT_1, .chan = BAT_ADC_CHAN, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    if (adc_cali_create_scheme_curve_fitting(&k, &bat_cali) != ESP_OK) bat_cali = NULL;
}

static int bat_pin_mv;

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

// CHRG input (PCB v3: 5V IN+ via 10k, active HIGH): internal pull-down keeps idle line LOW
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

static void bat_pin_diag(void)
{
    static adc_cali_handle_t chg_cali;
    static bool chg_cali_tried;
    chrg_pin_setup();
    vTaskDelay(pdMS_TO_TICKS(5));
    int chg_lvl = gpio_get_level(CHRG_GPIO);
    adc_oneshot_chan_cfg_t c = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    adc_oneshot_config_channel(bat_adc, ADC_CHANNEL_3, &c);
    if (!chg_cali_tried) {
        chg_cali_tried = true;
        adc_cali_curve_fitting_config_t k = { .unit_id = ADC_UNIT_1, .chan = ADC_CHANNEL_3, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
        if (adc_cali_create_scheme_curve_fitting(&k, &chg_cali) != ESP_OK) chg_cali = NULL;
    }
    if (CHRG_ACTIVE) gpio_pulldown_en(CHRG_GPIO); else gpio_pullup_en(CHRG_GPIO);
    vTaskDelay(pdMS_TO_TICKS(20));
    int chg_mv = adc_mv(ADC_CHANNEL_3, chg_cali);
    chrg_pin_setup();

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
           chg_mv > 1650 ? "high: charging" :
           chg_mv < 400  ? "low: not charging" : "part-way: unclear");
}

static void bat_log(float v)
{
    printf("BATT: GPIO1 %d mV x%.1f = VSW %.2f V -> %s, CHRG GPIO%d=%d%s\n", bat_pin_mv, BAT_DIVIDER, v,
           v > 4.45f ? "USB level" : v > BATT_MIN_VALID_V ? "battery" : "nothing on VSW",
           (int)CHRG_GPIO, gpio_get_level(CHRG_GPIO), chrg_on() ? " (charging)" : "");
}

static int bat_percent(float v)
{
    static const struct { float v; int pct; } k[] = {
        {4.20f, 100}, {4.15f, 95}, {4.11f, 90}, {4.08f, 85}, {4.02f, 80}, {3.98f, 75}, {3.95f, 70}, {3.91f, 65},
        {3.87f, 60}, {3.85f, 55}, {3.84f, 50}, {3.82f, 45}, {3.80f, 40}, {3.79f, 35}, {3.77f, 30}, {3.75f, 25},
        {3.73f, 20}, {3.71f, 15}, {3.69f, 10}, {3.61f, 5}, {3.27f, 0},
    };
    const int n = (int)(sizeof(k) / sizeof(k[0]));
    if (v >= k[0].v) return 100;
    if (v <= k[n - 1].v) return 0;
    for (int i = 1; i < n; i++) if (v >= k[i].v) {
        float f = (v - k[i].v) / (k[i - 1].v - k[i].v);
        return (int)(k[i].pct + f * (k[i - 1].pct - k[i].pct) + 0.5f);
    }
    return 0;
}

// ---------------------------------------------------------------- button: GPIO5 ISR + esp_timer debounce
static void IRAM_ATTR btn_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t hpw = pdFALSE;
    if (btn_task_handle) {
        vTaskNotifyGiveFromISR(btn_task_handle, &hpw);
        portYIELD_FROM_ISR(hpw);
    }
}

static void button_pin_setup(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << BTN_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io);
}

static void button_task(void *arg)
{
    (void)arg;
    btn_task_handle = xTaskGetCurrentTaskHandle();
    button_pin_setup();
    esp_err_t ie = gpio_install_isr_service(0);
    if (ie != ESP_OK && ie != ESP_ERR_INVALID_STATE) ESP_LOGW(TAG, "gpio_install_isr_service: %s", esp_err_to_name(ie));
    gpio_isr_handler_add(BTN_GPIO, btn_isr_handler, NULL);

    vTaskDelay(pdMS_TO_TICKS(10));
    int state = gpio_get_level(BTN_GPIO);
    int64_t last_change_us = 0;
    bool recheck = false;

    for (;;) {
        // Sleep with 0% CPU until GPIO5 edge interrupt fires (or 25 ms post-lockout recheck)
        ulTaskNotifyTake(pdTRUE, recheck ? pdMS_TO_TICKS(25) : portMAX_DELAY);

        int64_t now_us = esp_timer_get_time();
        int raw = gpio_get_level(BTN_GPIO);
        if (raw != state) {
            if (now_us - last_change_us >= BTN_DEBOUNCE_US) {
                state = raw;
                last_change_us = now_us;
                btn_ev_t ev = { .down = (raw == 0), .ms = (uint32_t)(now_us / 1000) };
                xQueueSend(btn_q, &ev, 0);
                recheck = true;
            } else {
                recheck = true;
            }
        } else {
            recheck = false;
        }
    }
}

// ---------------------------------------------------------------- UI task
static SemaphoreHandle_t flush_go, flush_done;
static volatile int front;
static volatile uint32_t stat_flush_us, stat_frames;

static void flush_task(void *arg)
{
    (void)arg;
    uint32_t retry_at = 0;
    for (;;) {
        xSemaphoreTake(flush_go, portMAX_DELAY);
        int64_t t0 = esp_timer_get_time();
        I2C_LOCK();
        if (oled_ok) {
            if (oled_flush(pagebuf[front]) != ESP_OK) { oled_ok = false; retry_at = ms_now() + 1000; ESP_LOGW(TAG, "OLED write failed"); }
        } else if (bus && ms_now() > retry_at) {
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
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    int back = 0;
    uint32_t stat_render_us = 0;
    int64_t stat_t0 = esp_timer_get_time();
    for (;;) {
        btn_ev_t ev;
        display_msg_t dm;
        int64_t t0 = esp_timer_get_time();
        LOCK();
        while (xQueueReceive(btn_q, &ev, 0) == pdTRUE) ui_button(ev.down, ev.ms);
        if (q_to_display) {
            while (xQueueReceive(q_to_display, &dm, 0) == pdTRUE) ui_message(dm.from, dm.text);
        }
        ui_frame(ms_now());
        pack_frame(pagebuf[back]);
        UNLOCK();
        stat_render_us += (uint32_t)(esp_timer_get_time() - t0);
        xSemaphoreTake(flush_done, portMAX_DELAY);
        front = back; back ^= 1;
        xSemaphoreGive(flush_go);
        stat_frames++;
        if (esp_timer_get_time() - stat_t0 >= 10000000) {
            float s = (esp_timer_get_time() - stat_t0) / 1e6f;
            printf("UI: %.1f fps, draw %.1f ms, send %.1f ms, heap %lu B\n",
                   stat_frames / s,
                   stat_render_us / 1000.0f / stat_frames,
                   stat_flush_us / 1000.0f / stat_frames,
                   (unsigned long)esp_get_free_heap_size());
            stat_frames = 0; stat_render_us = 0; stat_flush_us = 0; stat_t0 = esp_timer_get_time();
        }
        TickType_t next = wake + pdMS_TO_TICKS(FRAME_MS), nowt = xTaskGetTickCount();
        if ((int32_t)(next - nowt) <= 0 || uxQueueMessagesWaiting(btn_q)) { vTaskDelay(1); wake = xTaskGetTickCount(); }
        else xTaskDelayUntil(&wake, pdMS_TO_TICKS(FRAME_MS));
    }
}

// ---------------------------------------------------------------- UI hooks & shutdown
static char peer_name[8];
void ui_port_ping(void) { link_ping(); }

#define SKIP_UPD_MAGIC 0x5EB0071Du
static RTC_NOINIT_ATTR uint32_t skip_update_flag;
void ui_port_reboot(void) { printf("rebooting to normal mode\n"); skip_update_flag = SKIP_UPD_MAGIC; esp_restart(); }

static void on_ota_status(int pct, const char *msg)
{
    LOCK();
    ui_update_status(pct, msg);
    UNLOCK();
}

static void clean_low_battery_shutdown(float v)
{
    printf("POWER: battery %.2f V < %.2f V for 3 readings -> clean shutdown\n", v, BATT_SHUTDOWN_V);
    LOCK();
    ui_warn(4);   // "SHUTTING DOWN"
    UNLOCK();
    vTaskDelay(pdMS_TO_TICKS(2000));

    link_sleep();
    oled_power_off();
    web_server_stop();
    wifi_ap_stop();

    // Configure GPIO5 push button (active LOW) as deep-sleep wakeup source
    gpio_isr_handler_remove(BTN_GPIO);
    gpio_config_t wake_io = {
        .pin_bit_mask = 1ULL << BTN_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&wake_io);
    esp_sleep_enable_gpio_wakeup_on_hp_periph_powerdown(1ULL << BTN_GPIO, ESP_GPIO_WAKEUP_GPIO_LOW);
    printf("POWER: entering deep sleep (press GPIO%d button to wake)\n", (int)BTN_GPIO);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_deep_sleep_start();
}

// ---------------------------------------------------------------- main
void app_main(void)
{
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    bool unit_delta = mac[5] == PERFBOARD_MAC5;
    oled_rot180 = (OLED_ROTATE_180 != 0);   // both ECHO and DELTA rotated 180 deg
    const char *me = unit_delta ? "DELTA" : "ECHO";
    snprintf(peer_name, sizeof(peer_name), "%s", unit_delta ? "ECHO" : "DELTA");
    char ap_ssid[24];
    snprintf(ap_ssid, sizeof(ap_ssid), "%s-LINK", me);   // ECHO-LINK / DELTA-LINK

    tl_log_hook_esp_log();
    {
        static const char *rr[] = { "unknown", "power-on", "external pin", "software", "panic", "interrupt watchdog",
                                    "task watchdog", "other watchdog", "deep sleep", "brownout", "SDIO", "USB", "JTAG",
                                    "efuse", "power glitch", "CPU lockup" };
        esp_reset_reason_t r = esp_reset_reason();
        printf("TREELINE %s: reset reason %s (%d)\n", me, (unsigned)r < sizeof(rr) / sizeof(rr[0]) ? rr[r] : "?", (int)r);
    }

    // Single NVS initialization shared by baro calibration, hc12 pin map, and Wi-Fi
    esp_err_t ne = nvs_flash_init();
    if (ne == ESP_ERR_NVS_NO_FREE_PAGES || ne == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ne = nvs_flash_init();
    }
    if (ne != ESP_OK) printf("NVS init failed: %s\n", esp_err_to_name(ne));

    bool skip_update = skip_update_flag == SKIP_UPD_MAGIC && esp_reset_reason() == ESP_RST_SW;
    skip_update_flag = 0;

    // Button held while switching on (~0.3 s) -> Wi-Fi recovery update mode
    gpio_config_t bio = { .pin_bit_mask = 1ULL << BTN_GPIO, .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&bio);
    vTaskDelay(pdMS_TO_TICKS(20));
    bool update_mode = !skip_update;
    if (skip_update) printf("TREELINE %s: left update mode, button ignored this boot\n", me);
    for (int i = 0; i < 10 && update_mode; i++) {
        if (gpio_get_level(BTN_GPIO)) update_mode = false;
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    ui_lock  = xSemaphoreCreateMutex();
    i2c_lock = xSemaphoreCreateMutex();
    btn_q    = xQueueCreate(16, sizeof(btn_ev_t));
    ui_init("TREELINE");
    ui_data_t *S = ui_data();
    snprintf(S->me, sizeof(S->me), "%s", me);
    snprintf(S->peer, sizeof(S->peer), "%s", peer_name);
    snprintf(S->wifi_ssid, sizeof(S->wifi_ssid), "%s", ap_ssid);
    snprintf(S->wifi_pass, sizeof(S->wifi_pass), "%s", WIFI_PASS);
    snprintf(S->wifi_ip, sizeof(S->wifi_ip), "192.168.4.1");
    snprintf(S->os_ver, sizeof(S->os_ver), "ESP-IDF %d.%d", ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR);
    snprintf(S->boot_line[0], 24, "HC-12 433.4MHz CH001");
    snprintf(S->boot_line[1], 24, "SH1106 128x64 I2C");
    snprintf(S->boot_line[2], 24, "BMP280 ...");
    snprintf(S->boot_line[3], 24, "BUTTON GPIO%d ISR", (int)BTN_GPIO);
    snprintf(S->boot_line[4], 24, "BATTERY ADC GPIO1");
    snprintf(S->boot_line[5], 24, "CHARGER CHRG GPIO%d", (int)CHRG_GPIO);
    snprintf(S->boot_line[6], 24, "peer %s ...", peer_name);

    // Best-effort I2C + OLED bring-up before update_mode (no ESP_ERROR_CHECK, no BMP280 yet)
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
    i2c_master_bus_config_t bc = {
        .i2c_port = -1, .sda_io_num = I2C_SDA_GPIO, .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true,
    };
    i2c_bus_recover();
    esp_err_t i2ce = i2c_new_master_bus(&bc, &bus);
    if (i2ce == ESP_OK) {
        oled_ok = oled_bringup();
    } else {
        bus = NULL;
        oled_ok = false;
        printf("I2C: bus init failed (%s) - continuing without OLED\n", esp_err_to_name(i2ce));
    }
    printf("TREELINE %s: OLED %s at %lu Hz\n", me, oled_ok ? "ok" : "MISSING", (unsigned long)i2c_hz);

    xTaskCreate(button_task, "btn", 2560, NULL, 7, NULL);
    flush_go   = xSemaphoreCreateBinary();
    flush_done = xSemaphoreCreateBinary();
    xSemaphoreGive(flush_done);
    xTaskCreate(flush_task, "oled", 3072, NULL, 3, NULL);
    xTaskCreate(ui_task,    "ui",   6144, NULL, 3, NULL);
    vTaskPrioritySet(NULL, 4);

    // Recovery OTA mode runs BEFORE BMP280, HC-12, or chat tasks so peripheral faults can't lock out OTA
    if (update_mode) {
        printf("TREELINE %s: button held at power-on -> Wi-Fi update mode (%s)\n", me, ap_ssid);
        LOCK(); ui_update_mode(ap_ssid, WIFI_PASS, "192.168.4.1"); UNLOCK();
        ota_start(ap_ssid, WIFI_PASS, on_ota_status);
        chrg_pin_setup();
        bat_init();
        for (;;) {
            bat_log(bat_volts());
            bat_pin_diag();
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }

    // Normal mode initialization (runs behind boot screen)
    I2C_LOCK();
    bmp_ok = bmp_init();
    I2C_UNLOCK();
    printf("TREELINE %s: BMP280 %s", me, bmp_ok ? "ok" : "NOT FOUND");
    if (bmp_ok) printf(" at 0x%02X", bmp_addr);
    printf("\n");

    if (ne == ESP_OK) baro_load();
    chrg_pin_setup();

    temperature_sensor_handle_t tsens = NULL;
    bool chip_tsens_ready = false;
    temperature_sensor_config_t tc = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 100);
    if (temperature_sensor_install(&tc, &tsens) == ESP_OK && temperature_sensor_enable(tsens) == ESP_OK) {
        chip_tsens_ready = true;
    } else {
        printf("CHIP TEMP: internal sensor init failed\n");
    }

    bat_init();

    // Chat store, telemetry snapshot, router queues, Wi-Fi SoftAP, and shared HTTP server
    msg_store_init(me, peer_name);
    status_init(me, peer_name);
    router_init(peer_name);

    esp_err_t we = wifi_ap_start(ap_ssid, WIFI_PASS);
    esp_err_t se = ESP_FAIL;
    if (we == ESP_OK) {
        se = web_server_start(NULL);
        if (se != ESP_OK) printf("WEB: start failed: %s\n", esp_err_to_name(se));
    } else {
        printf("WIFI: AP start failed: %s\n", esp_err_to_name(we));
    }

    // HC-12 radio bring-up
    hc12_test();
    button_pin_setup();
    chrg_pin_setup();
    bool radio = link_start();
    for (int i = 0; i < 30; i++) {
        link_stats_t st; link_stats(&st);
        if (st.peer_seen) break;
        if (i % 10 == 0) link_send();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    link_stats_t st0; link_stats(&st0);
    float v0 = bat_volts();

    LOCK();
    S->radio_ok = radio;
    S->boot_ok[0] = radio;
    S->boot_ok[1] = oled_ok;
    if (bmp_ok) snprintf(S->boot_line[2], 24, "BMP280 @0x%02X", bmp_addr);
    else snprintf(S->boot_line[2], 24, "%.23s", bmp_err[0] ? bmp_err : "BMP280 missing");
    S->boot_ok[2] = bmp_ok;
    S->boot_ok[3] = gpio_get_level(BTN_GPIO) == 1;
    S->boot_ok[4] = v0 > BATT_MIN_VALID_V;
    if (v0 > 4.45f) snprintf(S->boot_line[4], 24, "BATTERY: USB power");
    else if (v0 <= BATT_MIN_VALID_V) snprintf(S->boot_line[4], 24, "BATTERY not found");
    S->boot_ok[5] = true;
    snprintf(S->boot_line[6], 24, st0.peer_seen ? "peer %s found" : "peer %s not heard", peer_name);
    S->boot_ok[6] = st0.peer_seen;
    S->boot_ready = true;
    UNLOCK();

    // Only confirm OTA image if Wi-Fi AP and HTTP server are up; otherwise bootloader rolls back
    if (we == ESP_OK && se == ESP_OK) {
        ota_mark_good();
    } else {
        printf("OTA: NOT confirming image (wifi=%s, web=%s) - will roll back on reboot\n",
               esp_err_to_name(we), esp_err_to_name(se));
    }

    printf("TREELINE %s: AP=%s radio=%s BMP=%s batt=%.2fV peer=%s\n",
           me, ap_ssid, radio ? "ok" : "FAIL", bmp_ok ? "ok" : "missing",
           v0, st0.peer_seen ? "found" : "not heard");

    bool charging = chrg_on(), chg_cand = charging;
    bool warned_low = false, warned_storm = false;
    int  low_shut_cnt = 0;
    int  bmp_fail_cnt = 0;

    // 1-minute pressure history (last 11 minutes) for rapid storm-drop detection
    float p_1m[12] = { 0 };
    int   p_1m_cnt = 0;
    int64_t next_1m = 0;

    LOCK(); S->charging = charging; UNLOCK();
    int64_t next_sec = esp_timer_get_time(), next_hist = 0, next_hb = 0, next_ping = next_sec + 15000000LL;
    float settle[5];
    int nsettle = 0;

    for (;;) {
        int rtt;
        if (link_ping_poll(&rtt)) {
            LOCK(); ui_ping_result(true, rtt); UNLOCK();
            printf("PING %s: %d ms\n", peer_name, rtt);
        }

        int64_t now = esp_timer_get_time();
        if (now >= next_hb) {
            link_send();
            next_hb = now + 900000 + esp_random() % 400000;
        }
        if (now >= next_ping) {
            link_stats_t pst; link_stats(&pst);
            if (pst.ok && pst.peer_seen) link_ping();
            next_ping = now + 30000000LL;   // keep /status ping_ms fresh every 30 s
        }

        if (now >= next_sec) {
            next_sec += 1000000;
            float t = 0.0f, p = 0.0f, chip = 0.0f;

            // BMP280 read with hot-unplug detection & 5 s automatic re-probe
            I2C_LOCK();
            if (!bmp_ok && bus && (now / 1000000) % 5 == 0) {
                bmp_ok = bmp_init();
                if (bmp_ok) { bmp_fail_cnt = 0; printf("BMP280 found at 0x%02X\n", bmp_addr); }
            }
            bool have_p = bmp_ok && bmp_measure(&t, &p);
            if (bmp_ok) {
                if (have_p) {
                    bmp_fail_cnt = 0;
                } else if (++bmp_fail_cnt >= 3) {
                    printf("BMP280: 3 consecutive read failures -> marking offline\n");
                    bmp_ok = false;
                    if (bmp) { i2c_master_bus_rm_device(bmp); bmp = NULL; }
                    snprintf(bmp_err, sizeof(bmp_err), "BMP280 unplugged");
                }
            }
            I2C_UNLOCK();

            if (have_p && !baro_cal && ne == ESP_OK) {
                settle[nsettle % 5] = p; nsettle++;
                if (nsettle >= 5) {
                    float lo = settle[0], hi = settle[0];
                    for (int i = 1; i < 5; i++) { lo = fminf(lo, settle[i]); hi = fmaxf(hi, settle[i]); }
                    if (hi - lo < 0.10f) baro_calibrate(p);
                }
            }

            float alt_abs = have_p ? alt_from(p, baro_p0) : 0.0f;
            if (have_p && (now / 1000000) % 30 == 0) {
                printf("BARO: %.2f hPa, altitude %.1f m, above home %+.1f m%s\n",
                       p, alt_abs, alt_abs - HOME_ALT_M, baro_cal ? "" : " (uncalibrated)");
            }

            // Record 1-minute pressure samples for rapid storm drop check
            if (have_p && now >= next_1m) {
                if (p_1m_cnt == 12) { memmove(p_1m, p_1m + 1, sizeof(float) * 11); p_1m_cnt = 11; }
                p_1m[p_1m_cnt++] = p;
                next_1m = now + 60LL * 1000000;
            }

            bool chip_ok = chip_tsens_ready && (temperature_sensor_get_celsius(tsens, &chip) == ESP_OK);

            float v = bat_volts();
            if ((now / 1000000) % 10 == 0) { bat_log(v); link_print(); }
            bool lvl = chrg_on();
            link_stats_t st; link_stats(&st);
            int wclients = wifi_ap_clients();

            // Compute pressure trend & storm flag
            const char *p_trend = "steady";
            bool storm_now = false;

            LOCK();
            S->uptime       = (uint32_t)(now / 1000000);
            S->heap_free    = esp_get_free_heap_size();
            S->wifiDev      = wclients;
            S->chip_temp_ok = chip_ok;
            S->tCpu         = chip_ok ? chip : 0.0f;
            S->batt_temp_ok = false;   // DS18B20 not installed on PCB v3
            S->tBatt        = 0.0f;
            S->press_ok     = have_p;
            snprintf(S->baro_err, sizeof(S->baro_err), "%s", bmp_ok ? (have_p ? "" : "BMP: no reading yet") : bmp_err);

            if (have_p) {
                S->press = p; S->tAir = t;
                S->alt   = ALT_SHOW_RELATIVE ? alt_abs - HOME_ALT_M : alt_abs;
                if (now >= next_hist) {
                    if (S->nhist == UI_HIST) { memmove(S->hist, S->hist + 1, sizeof(float) * (UI_HIST - 1)); S->nhist--; }
                    S->hist[S->nhist++] = p;
                    next_hist = now + 300LL * 1000000;
                }
                float d3h = (S->nhist >= 2) ? (S->hist[S->nhist - 1] - S->hist[0]) : 0.0f;
                float d10m = (p_1m_cnt >= 2) ? (p - p_1m[0]) : 0.0f;
                if (d3h <= -1.0f || d10m <= -0.8f) p_trend = "falling";
                else if (d3h >= 1.0f || d10m >= 0.8f) p_trend = "rising";
                storm_now = (d3h <= -STORM_DROP_3H_HPA) || (d10m <= -STORM_DROP_10M_HPA);
            }

            S->usb_power = v > 4.45f;
            S->batt_ok   = v > BATT_MIN_VALID_V && !S->usb_power;
            S->battV     = v;
            S->batt      = S->batt_ok ? bat_percent(v) : 0;

            // Debounce CHRG over two consecutive 1 s samples
            if (lvl == chg_cand && lvl != charging) { charging = lvl; ui_set_charging(charging); }
            chg_cand = lvl;

            // Warnings
            S->low = S->batt_ok && !charging && S->batt <= 10;
            if (S->low && !warned_low) { warned_low = true; ui_warn(0); }
            if (!S->low && S->batt > 15) warned_low = false;

            if (storm_now && !warned_storm) { warned_storm = true; ui_warn(3); }
            if (!storm_now) warned_storm = false;

            S->radio_ok = st.ok;
            S->linkUp   = st.ok && st.peer_seen && st.heard_s >= 0 && st.heard_s < 6;
            S->seen     = st.peer_seen ? (int)st.heard_s : -1;
            S->tx       = (int)st.tx;
            S->rx       = (int)st.rx;
            S->loss     = (int)(st.loss + 0.5f);
            if (st.ping_ms >= 0) S->ping = st.ping_ms;

            // Publish thread-safe snapshot for GET /status
            status_t snap = { 0 };
            snprintf(snap.device, sizeof(snap.device), "%s", me);
            snprintf(snap.peer,   sizeof(snap.peer),   "%s", peer_name);
            snap.uptime_s         = S->uptime;
            snap.wifi_clients     = wclients;
            snap.radio_ok         = st.ok;
            snap.link_up          = st.ok && st.peer_seen && st.heard_s >= 0.0f && st.heard_s <= 30.0f;
            snap.last_heard_s     = st.peer_seen ? (int)(st.heard_s + 0.5f) : -1;
            snap.ping_ms          = st.ping_ms;
            snap.loss_pct         = S->loss;
            snap.tx               = st.tx;
            snap.rx               = st.rx;
            snap.batt_ok          = S->batt_ok;
            snap.batt_percent     = S->batt;
            snap.batt_volts       = S->battV;
            snap.batt_charging    = charging;
            snap.ds18b20_ok       = false;   // not installed on PCB v3 -> JSON null
            snap.batt_temp_c      = 0.0f;
            snap.chip_temp_ok     = chip_ok;
            snap.chip_temp_c      = chip;
            snap.bmp280_ok        = have_p;
            snap.pressure_hpa     = p;
            snap.altitude_m       = have_p ? (int)lroundf(alt_abs) : 0;
            snprintf(snap.pressure_trend, sizeof(snap.pressure_trend), "%s", p_trend);
            snap.warn_battery_low = S->low;
            snap.warn_battery_hot = false;
            snap.warn_storm       = storm_now;

            bool trigger_shutdown = false;
            if (S->batt_ok && !charging && v > BATT_MIN_VALID_V && v < BATT_SHUTDOWN_V) {
                if (++low_shut_cnt >= 3) trigger_shutdown = true;
            } else {
                low_shut_cnt = 0;
            }
            UNLOCK();

            status_set(&snap);

            if (trigger_shutdown) {
                clean_low_battery_shutdown(v);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
