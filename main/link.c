// Two-unit HC-12 radio link (TREELINE).
// Heartbeat: "PING,<id>,<seq>,<rx>,<recent>" about once a second (link quality).
// Ping:      "Q,<id>,<n>"  -> the other unit answers at once with "A,<id>,<n>" (round trip time).
// Message:   "M,<id>,<text>" (shown on the receiver's screen).
// Each unit sends "PING,<id>,<seq>,<rx>,<recent>\n" about once a second, where <rx> is
// how many of the other unit's packets it has received in total and <recent> how many
// of the other unit's last 20. So one unit on the PC can report both directions.
// NOTE: the HC-12 remembers the power setting after power-off.
#include <stdio.h>
#include "tlog.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "link.h"

#define HC12_UART      UART_NUM_1
#define HC12_SET_GPIO  6

int hc12_cmd(const char *cmd, char *resp, size_t resp_len);   // hc12.c
bool hc12_try_at(char *resp, size_t resp_len);                 // hc12.c

// Radio power adapts to the range. The peer reports in every heartbeat how many of our last
// 20 packets it heard: that is how well we are getting through.
//   heard >= 19/20 for 20 s       -> one step down (the window has refilled at the new power)
//   heard <= 16/20                -> one step up (at most every 4 s), no step down for 60 s
//   peer silent 6 s / not found   -> maximum, to get the link back
// Steps: P1 -1, P2 +2, P3 +5, P4 +8, P5 +11, P6 +14, P7 +17, P8 +20 dBm.
#define LINK_PWR_MIN        4               // floor: P4 = +8 dBm (the old fixed level)
#define LINK_PWR_MAX        8               // +20 dBm. 433 MHz limits are often ~+10 dBm (P4/P5): check yours
#define LINK_DOWN_HOLD_US   (20 * 1000000LL)
#define LINK_UP_HOLD_US     (4 * 1000000LL)
#define LINK_NO_DOWN_US     (60 * 1000000LL)  // after a step up
#define LINK_LOST_US        (6 * 1000000LL)
#define LINK_WINDOW      20                 // "recent" = the peer's last 20 packets
#define LINK_RETRY_US    (10 * 1000000LL)   // if the HC-12 did not answer, try again every 10 s

static const int pwr_dbm[9] = { 0, -1, 2, 5, 8, 11, 14, 17, 20 };
static int pwr = LINK_PWR_MAX;              // start loud: the peer has not been found yet
static int64_t pwr_changed_us, pwr_up_us;
// AT commands and radio traffic share the UART: while at_busy the receive task parks
// (so it does not eat the module's reply) and nothing is transmitted.
static volatile bool at_busy, rx_parked;

// Every radio transmission goes through here, so the battery reading can avoid the TX burst.
static volatile int64_t last_tx_us;
static void radio_tx(const char *pkt, int len)
{
    uart_write_bytes(HC12_UART, pkt, len);
    last_tx_us = esp_timer_get_time();
}
int64_t link_last_tx_us(void) { return last_tx_us; }

static bool link_ok;
static TaskHandle_t rx_handle;
static int64_t last_try_us;
static uint32_t start_attempts;
static char my_id[3], peer_id[3] = "--";
static volatile uint32_t tx_count, rx_count, peer_first_seq, peer_last_seq, bad_lines;
static volatile uint32_t peer_got_ours, peer_got_recent;   // as reported by the peer
static volatile uint32_t recent_mask;      // bit 0 = peer's latest seq, bit n = latest seq - n
static volatile bool peer_seen;
static volatile int64_t last_rx_us;
static volatile int64_t last_heard_us;      // any valid packet from the peer
static volatile uint32_t ping_seq;
static volatile int64_t ping_sent_us;
static volatile int ping_rtt_ms;
static volatile bool ping_answered;
static void (*msg_cb)(const char *text);

void link_set_msg_cb(void (*cb)(const char *text)) { msg_cb = cb; }
static void (*ping_cb)(void);
void link_set_ping_cb(void (*cb)(void)) { ping_cb = cb; }

static void handle_line(const char *line)
{
    char id[3];
    unsigned long n;
    if (line[0] == 'Q' && sscanf(line, "Q,%2[0-9A-F],%lu", id, &n) == 2) {
        if (strcmp(id, my_id) == 0) return;
        char pkt[24];
        int len = snprintf(pkt, sizeof(pkt), "A,%s,%lu\n", my_id, n);
        radio_tx(pkt, len);     // answer straight away
        last_heard_us = esp_timer_get_time();
        if (ping_cb) ping_cb();                     // after the answer, so the reply is not delayed
        return;
    }
    if (line[0] == 'A' && sscanf(line, "A,%2[0-9A-F],%lu", id, &n) == 2) {
        if (strcmp(id, my_id) == 0) return;
        last_heard_us = esp_timer_get_time();
        if (n == ping_seq && !ping_answered && ping_sent_us) {
            ping_rtt_ms = (int)((esp_timer_get_time() - ping_sent_us) / 1000);
            ping_answered = true;
        }
        return;
    }
    if (line[0] == 'M' && line[1] == ',' && sscanf(line, "M,%2[0-9A-F],", id) == 1) {
        if (strcmp(id, my_id) == 0) return;
        last_heard_us = esp_timer_get_time();
        const char *text = line + 5;
        if (msg_cb && *text) msg_cb(text);
        return;
    }
    unsigned long seq, their_rx, their_recent = 0;
    int fields = sscanf(line, "PING,%2[0-9A-F],%lu,%lu,%lu", id, &seq, &their_rx, &their_recent);
    if (fields < 3 || strcmp(id, my_id) == 0) {
        bad_lines++;
        return;
    }
    if (!peer_seen || seq <= peer_last_seq) {      // first contact, or the peer restarted
        peer_first_seq = seq;
        rx_count = 0;
        recent_mask = 0;
        strcpy(peer_id, id);
        peer_seen = true;
    } else {
        uint32_t gap = seq - peer_last_seq;
        recent_mask = gap >= 32 ? 0 : recent_mask << gap;
    }
    recent_mask |= 1;
    rx_count++;
    peer_last_seq = seq;
    peer_got_ours = their_rx;
    peer_got_recent = their_recent;
    last_rx_us = esp_timer_get_time();
    last_heard_us = last_rx_us;
}

// Of the peer's last up-to-20 packets, how many arrived. Packets the peer sent after the
// last one we heard are counted as missing, using about 1 s per packet.
static void recent_in(uint32_t *got, uint32_t *of)
{
    if (!peer_seen) { *got = *of = 0; return; }
    int64_t age_us = esp_timer_get_time() - last_rx_us;   // peer sends every ~1.0-1.4 s
    uint32_t silent = age_us > 1600000 ? (uint32_t)((age_us - 500000) / 1100000) : 0;
    uint32_t mask = silent >= 32 ? 0 : recent_mask << silent;
    uint32_t span = peer_last_seq - peer_first_seq + 1 + silent;
    *of = span < LINK_WINDOW ? span : LINK_WINDOW;
    *got = __builtin_popcount(mask & ((1u << *of) - 1));
}

static void rx_task(void *arg)
{
    char line[128];
    int len = 0;
    uint8_t c;
    while (true) {
        if (at_busy) { rx_parked = true; len = 0; vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        rx_parked = false;
        if (uart_read_bytes(HC12_UART, &c, 1, pdMS_TO_TICKS(50)) != 1) continue;
        if (c == '\n') {
            line[len] = 0;
            if (len) handle_line(line);
            len = 0;
        } else if (c != '\r') {
            if (len < (int)sizeof(line) - 1) line[len++] = c;
            else len = 0;                           // overlong garbage: drop it
        }
    }
}

// Call after hc12_test() has found the module at 9600 baud.
bool link_start(void)
{
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    snprintf(my_id, sizeof(my_id), "%02X", mac[5]);

    char resp[120];
    start_attempts++;
    vTaskDelay(pdMS_TO_TICKS(300));                 // let the module finish leaving the previous AT session
    bool ok = false;
    for (int attempt = 0; attempt < 5 && !ok; attempt++) {
        ok = hc12_try_at(resp, sizeof(resp));       // fresh SET edge each try
        if (!ok) printf("LINK: HC-12 no reply (try %d/5)%s%s\n", attempt + 1, resp[0] ? ", got: " : "", resp);
    }
    if (ok) {
        char cmd[8];
        snprintf(cmd, sizeof(cmd), "AT+P%d", pwr);
        hc12_cmd(cmd, resp, sizeof(resp));           printf("LINK %s   -> '%s'\n", cmd, resp);
        pwr_changed_us = esp_timer_get_time();
        hc12_cmd("AT+C001", resp, sizeof(resp));      printf("LINK AT+C001 -> '%s'\n", resp);
        hc12_cmd("AT+RX", resp, sizeof(resp));        printf("LINK AT+RX   -> '%s'\n", resp);
    }
    gpio_set_level(HC12_SET_GPIO, 1);               // back to radio mode
    vTaskDelay(pdMS_TO_TICKS(150));
    uart_flush_input(HC12_UART);
    last_try_us = esp_timer_get_time();

    if (!ok) {
        printf("LINK: HC-12 not answering, will retry in 10 s\n");
        return link_ok = false;
    }
    if (!rx_handle) xTaskCreate(rx_task, "hc12_rx", 3072, NULL, 6, &rx_handle);
    printf("LINK: started (after %lu start attempt%s), my id %s, power P%d (%+d dBm, adaptive), channel 001\n",
           (unsigned long)start_attempts, start_attempts == 1 ? "" : "s", my_id, pwr, pwr_dbm[pwr]);
    return link_ok = true;
}

// Switch the HC-12 to power level p (about 0.3 s with the radio deaf). Called from link_send().
static bool set_power(int p)
{
    at_busy = true;
    for (int i = 0; i < 30 && !rx_parked; i++) vTaskDelay(pdMS_TO_TICKS(10));
    char cmd[8], resp[48];
    snprintf(cmd, sizeof(cmd), "AT+P%d", p);
    gpio_set_level(HC12_SET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(80));                  // datasheet: >40 ms after SET goes low
    hc12_cmd(cmd, resp, sizeof(resp));
    bool ok = strstr(resp, "OK+P") != NULL;
    gpio_set_level(HC12_SET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100));                 // back in radio mode after ~80 ms
    uart_flush_input(HC12_UART);
    at_busy = false;
    if (!ok) printf("LINK: power change to P%d failed ('%s'), will retry\n", p, resp);
    return ok;
}

static void adapt_power(void)
{
    int64_t now = esp_timer_get_time();
    int target = pwr;
    const char *why = NULL;
    if (!peer_seen || now - last_rx_us > LINK_LOST_US) {
        target = LINK_PWR_MAX; why = peer_seen ? "peer lost" : "looking for peer";
    } else if (tx_count >= LINK_WINDOW) {           // the peer's report covers a full window
        if (peer_got_recent <= 16 && now - pwr_changed_us > LINK_UP_HOLD_US) {
            target = pwr + 1; why = "weak";
        } else if (peer_got_recent >= 19 && now - pwr_changed_us > LINK_DOWN_HOLD_US && now - pwr_up_us > LINK_NO_DOWN_US) {
            target = pwr - 1; why = "strong";
        }
    }
    if (target > LINK_PWR_MAX) target = LINK_PWR_MAX;
    if (target < LINK_PWR_MIN) target = LINK_PWR_MIN;
    if (target == pwr) return;
    int old = pwr;
    if (!set_power(target)) { pwr_changed_us = now; return; }   // try again after the hold time
    pwr = target;
    pwr_changed_us = esp_timer_get_time();
    if (target > old) pwr_up_us = pwr_changed_us;
    printf("LINK: power P%d -> P%d (%+d dBm): %s, peer heard %lu/%d\n", old, pwr, pwr_dbm[pwr], why,
           (unsigned long)peer_got_recent, LINK_WINDOW);
}

void link_send(void)
{
    if (!link_ok) {
        if (esp_timer_get_time() - last_try_us > LINK_RETRY_US) link_start();
        return;
    }
    adapt_power();
    uint32_t got, of;
    recent_in(&got, &of);
    char pkt[48];
    int n = snprintf(pkt, sizeof(pkt), "PING,%s,%lu,%lu,%lu\n", my_id,
                     (unsigned long)++tx_count, (unsigned long)rx_count, (unsigned long)got);
    radio_tx(pkt, n);
}

static float loss_pct(void)
{
    uint32_t expected = peer_seen ? peer_last_seq - peer_first_seq + 1 : 0;
    return expected ? 100.0f * (expected - rx_count) / expected : 0.0f;
}

void link_print(void)
{
    if (!link_ok) return;
    uint32_t got, of;
    recent_in(&got, &of);
    uint32_t out_of = tx_count < LINK_WINDOW ? tx_count : LINK_WINDOW;
    float age = peer_seen ? (esp_timer_get_time() - last_rx_us) / 1e6f : -1;
    printf("LINK me=%s P%d %+ddBm | IN from %s: last20 %lu/%lu, total rx=%lu loss=%.1f%%, last %.1fs ago"
           " | OUT %s heard last20 %lu/%lu, total %lu of my %lu | bad=%lu\n",
           my_id, pwr, pwr_dbm[pwr], peer_id, (unsigned long)got, (unsigned long)of,
           (unsigned long)rx_count, loss_pct(), age,
           peer_id, (unsigned long)peer_got_recent, (unsigned long)out_of,
           (unsigned long)peer_got_ours, (unsigned long)tx_count, (unsigned long)bad_lines);
}

// Four 21-character OLED lines
void link_lines(char out[4][32])
{
    if (!link_ok) {
        snprintf(out[0], 32, "RADIO: HC-12 FAIL");
        snprintf(out[1], 32, "retrying, try %lu", (unsigned long)start_attempts);
        out[2][0] = out[3][0] = 0;
        return;
    }
    snprintf(out[0], 32, "LINK me %s P%d CH1", my_id, pwr);
    if (!peer_seen) {
        snprintf(out[1], 32, "TX %lu", (unsigned long)tx_count);
        snprintf(out[2], 32, "waiting for peer...");
        out[3][0] = 0;
        return;
    }
    uint32_t got, of;
    recent_in(&got, &of);
    uint32_t out_of = tx_count < LINK_WINDOW ? tx_count : LINK_WINDOW;
    float age = (esp_timer_get_time() - last_rx_us) / 1e6f;
    snprintf(out[1], 32, "TX %lu  RX %lu", (unsigned long)tx_count, (unsigned long)rx_count);
    snprintf(out[2], 32, "IN  %lu/%lu %s%.0fs", (unsigned long)got, (unsigned long)of,
             age > 5 ? "LOST " : "", age);
    snprintf(out[3], 32, "OUT %lu/%lu (%s hears)", (unsigned long)peer_got_recent,
             (unsigned long)out_of, peer_id);
}

// ---------------------------------------------------------------- TREELINE additions

// Round-trip ping to the peer (double press). Answer arrives in link_ping_poll().
void link_ping(void)
{
    if (!link_ok || at_busy) return;                // mid power change: this ping is skipped
    ping_answered = false;
    ping_seq++;
    char pkt[24];
    int len = snprintf(pkt, sizeof(pkt), "Q,%s,%lu\n", my_id, (unsigned long)ping_seq);
    ping_sent_us = esp_timer_get_time();
    radio_tx(pkt, len);
}

bool link_ping_poll(int *rtt_ms)
{
    if (!ping_answered) return false;
    ping_answered = false;
    *rtt_ms = ping_rtt_ms;
    return true;
}

void link_send_text(const char *text)
{
    if (!link_ok) return;
    for (int i = 0; i < 60 && at_busy; i++) vTaskDelay(pdMS_TO_TICKS(10));   // wait out a power change
    char pkt[110];
    int len = snprintf(pkt, sizeof(pkt), "M,%s,%.100s\n", my_id, text);
    radio_tx(pkt, len);
}


void link_stats(link_stats_t *st)
{
    st->ok = link_ok;
    st->peer_seen = peer_seen;
    st->heard_s = peer_seen ? (esp_timer_get_time() - last_heard_us) / 1e6f : -1;
    st->tx = tx_count;
    st->rx = rx_count;
    st->loss = loss_pct();
}
