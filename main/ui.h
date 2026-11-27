// TREELINE OLED interface: C port of the "TREELINE · OLED interface" mockup.
// Pure C, no ESP-IDF dependencies: the firmware feeds data and button edges in,
// calls ui_frame() ~30 times a second and sends ui_fb() to the SH1106.
// Everything here runs in ONE task (the UI task). Other tasks hand data over through
// the firmware's mutex, see main.c.
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define UI_W 128
#define UI_H 64
#define UI_HIST 37          // barometer history points (3 h, one every 5 min)
#define UI_MSGS 6

typedef struct {
    char from[8];
    char text[96];
    uint32_t rx_up;         // uptime (s) when it arrived
} ui_msg_t;

typedef struct {
    // battery / power
    int   batt;             // 0..100 %
    float battV;
    bool  batt_ok;          // a battery voltage was measured
    bool  usb_power;        // VSW is at USB level (no battery reading)
    bool  low;
    bool  charging;
    // radio
    bool  radio_ok;         // HC-12 answered
    bool  linkUp;           // peer heard recently
    int   seen;             // s since the peer was last heard (-1 = never)
    int   tx, rx, loss;     // packets, loss %
    int   ping;             // last round trip ms (-1 = none yet)
    int   unread;
    int   wifiDev;          // -1 = Wi-Fi off
    // sensors
    bool  press_ok;
    char  baro_err[28];     // why the BMP280 is not working (shown on screen)
    float press, alt, tAir, tCpu;
    float hist[UI_HIST];
    int   nhist;            // valid points in hist (newest last)
    // system
    uint32_t uptime;
    uint32_t heap_free;
    char  os_ver[16];       // e.g. "ESP-IDF 6.1"
    // inbox, newest first
    ui_msg_t msgs[UI_MSGS];
    int nmsgs;
    // names
    char me[8], peer[8];
    // boot self-test log: 7 lines, ok flags
    char boot_line[7][24];
    bool boot_ok[7];
    bool boot_ready;        // firmware finished init; the log may finish
} ui_data_t;

void ui_init(const char *title);
ui_data_t *ui_data(void);                    // edit under the firmware's lock
void ui_button(bool down, uint32_t now_ms);  // debounced edges, UI task only
void ui_frame(uint32_t now_ms);              // render one frame
const uint8_t *ui_fb(void);                  // 128*64 bytes, 1 = lit

// events (call under the firmware's lock)
void ui_message(const char *from, const char *text);
void ui_set_charging(bool on);
void ui_warn(int kind);                      // 0 = low battery, 1 = USB-C with switch ON
void ui_ping_result(bool ok, int rtt_ms);
void ui_ping_rx(const char *from);           // the peer pinged us (we answered): short toast

// Wi-Fi update mode screen
void ui_update_mode(const char *ssid, const char *pass, const char *ip);
void ui_update_status(int pct, const char *msg);   // 0..100, 101 = rebooting, -1 = waiting, -2 = error

// hooks the firmware implements
void ui_port_ping(void);     // send a ping to the peer (double press)
void ui_port_reboot(void);   // long press in update mode

