// Two-unit HC-12 radio link (TREELINE).
//
// Coexisting protocols on UART1 (9600 baud):
//   ASCII lines (\n-terminated):
//     PING,<id>,<seq>,<rx>,<recent>   1 Hz heartbeat for link quality stats
//     Q,<id>,<n> -> A,<id>,<n>        Round-trip ping measurement
//   Binary frames (non-ASCII 0xAA 0x55 sync):
//     [0xAA][0x55][LEN][TYPE][SRC][SEQ][TEXT:0..160][CRC16_HI][CRC16_LO]
//     TYPE: 'M' (chat message, LEN 1..160), 'K' (ACK, LEN 0)
//     CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) covers LEN..TEXT.
//
// Outgoing messages arrive on q_to_radio; radio_tx_task transmits with 1 initial send
// + up to 3 retries (~5 s total) timed by esp_timer, then posts RADIO_DELIVERED or
// RADIO_FAILED to q_from_radio. Valid incoming 'M' frames are ACKed immediately,
// deduplicated by (SRC, SEQ), and posted as RADIO_RECEIVED to q_from_radio.
#include <stdio.h>
#include "tlog.h"
#include <string.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "app_queues.h"
#include "link.h"

#define HC12_UART          UART_NUM_1
#define HC12_SET_GPIO      6

#define LINK_POWER_CMD     "AT+P4"
#define LINK_POWER_LABEL   "P4"
#define LINK_WINDOW        20
#define LINK_RETRY_US      (10 * 1000000LL)  // re-probe HC-12 every 10 s if absent

#define FRAME_SYNC0        0xAA
#define FRAME_SYNC1        0x55
#define FRAME_TYPE_MSG     'M'
#define FRAME_TYPE_ACK     'K'
#define FRAME_MAX_RETRIES  3                 // 1 initial send + 3 retries = 4 attempts
#define FRAME_ACK_BASE_US  1150000LL         // ~1.15-1.35 s per attempt (~5 s total)
#define FRAME_ACK_JITTER   200000U
#define FRAME_BYTE_TMO_US  500000LL          // inter-byte timeout prevents parser wedge

int  hc12_cmd(const char *cmd, char *resp, size_t resp_len);   // hc12.c
bool hc12_try_at(char *resp, size_t resp_len);                 // hc12.c

static SemaphoreHandle_t uart_tx_lock;
static TaskHandle_t      rx_handle;
static TaskHandle_t      tx_handle;
static esp_timer_handle_t ack_timer;

static volatile bool     link_ok;
static int64_t           last_try_us;
static uint32_t          start_attempts;
static uint8_t           my_mac5;
static char              my_id[3], peer_id[3] = "--";

// Heartbeat & link quality stats
static volatile uint32_t tx_count, rx_count, peer_first_seq, peer_last_seq, bad_lines;
static volatile uint32_t peer_got_ours, peer_got_recent;
static volatile uint32_t recent_mask;
static volatile bool     peer_seen;
static volatile int64_t  last_rx_us;
static volatile int64_t  last_heard_us;

// Round-trip ping state
static volatile uint32_t ping_seq;
static volatile int64_t  ping_sent_us;
static volatile int      ping_rtt_ms = -1;
static volatile bool     ping_answered;

// Reliable message TX/ACK state
static atomic_bool       tx_in_flight;
static volatile bool     ack_waiting;
static volatile bool     ack_received;
static volatile uint8_t  ack_expect_seq;
static uint8_t           tx_seq;

// RX deduplication
static bool              have_last_rx;
static uint8_t           last_rx_src;
static uint8_t           last_rx_seq;

// ---------------------------------------------------------------- CRC-16/CCITT-FALSE
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void uart_send_locked(const void *data, size_t len)
{
    if (!uart_tx_lock) return;
    xSemaphoreTake(uart_tx_lock, portMAX_DELAY);
    if (link_ok) uart_write_bytes(HC12_UART, data, len);
    xSemaphoreGive(uart_tx_lock);
}

static size_t build_frame(uint8_t *out, uint8_t type, uint8_t src, uint8_t seq,
                          const uint8_t *payload, uint8_t len, uint16_t crc_xor)
{
    out[0] = FRAME_SYNC0;
    out[1] = FRAME_SYNC1;
    out[2] = len;
    out[3] = type;
    out[4] = src;
    out[5] = seq;
    if (len > 0 && payload) memcpy(out + 6, payload, len);
    uint16_t crc = crc16_ccitt(out + 2, 4 + len) ^ crc_xor;
    out[6 + len] = (uint8_t)(crc >> 8);
    out[7 + len] = (uint8_t)(crc & 0xFF);
    return 8 + len;
}

// ---------------------------------------------------------------- ACK timer callback
static void on_ack_timeout(void *arg)
{
    (void)arg;
    if (tx_handle) xTaskNotifyGive(tx_handle);
}

// ---------------------------------------------------------------- ASCII line handler
static void handle_ascii_line(const char *line)
{
    char id[3];
    unsigned long n;

    if (line[0] == 'Q' && sscanf(line, "Q,%2[0-9A-F],%lu", id, &n) == 2) {
        if (strcmp(id, my_id) == 0) return;
        char pkt[24];
        int len = snprintf(pkt, sizeof(pkt), "A,%s,%lu\n", my_id, n);
        uart_send_locked(pkt, (size_t)len);
        last_heard_us = esp_timer_get_time();
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

    unsigned long seq, their_rx, their_recent = 0;
    int fields = sscanf(line, "PING,%2[0-9A-F],%lu,%lu,%lu", id, &seq, &their_rx, &their_recent);
    if (fields < 3 || strcmp(id, my_id) == 0) {
        bad_lines++;
        return;
    }
    if (!peer_seen || seq <= peer_last_seq) {
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

// ---------------------------------------------------------------- Binary frame handler
static void handle_bin_frame(const uint8_t *body, uint8_t len)
{
    // body[0]=LEN, body[1]=TYPE, body[2]=SRC, body[3]=SEQ, body[4..4+len-1]=TEXT, body[4+len..5+len]=CRC
    uint8_t  type   = body[1];
    uint8_t  src    = body[2];
    uint8_t  seq    = body[3];
    uint16_t rx_crc = ((uint16_t)body[4 + len] << 8) | body[5 + len];
    uint16_t calc   = crc16_ccitt(body, 4 + len);

    if (rx_crc != calc) {
        bad_lines++;
        printf("LINK: CRC MISMATCH (type=0x%02X len=%u seq=%u got=0x%04X exp=0x%04X) - dropped\n",
               type, len, seq, rx_crc, calc);
        return;
    }

    if (src == my_mac5) return;   // ignore own echo

    last_heard_us = esp_timer_get_time();
    if (!peer_seen) {
        snprintf(peer_id, sizeof(peer_id), "%02X", src);
        peer_seen = true;
    }

    if (type == FRAME_TYPE_ACK) {
        if (ack_waiting && seq == ack_expect_seq) {
            ack_received = true;
            if (ack_timer) esp_timer_stop(ack_timer);
            if (tx_handle) xTaskNotifyGive(tx_handle);
        }
        return;
    }

    if (type == FRAME_TYPE_MSG && len > 0 && len <= MSG_MAX_BYTES) {
        // ACK immediately so sender knows it arrived
        uint8_t ack_pkt[8];
        size_t ack_len = build_frame(ack_pkt, FRAME_TYPE_ACK, my_mac5, seq, NULL, 0, 0);
        uart_send_locked(ack_pkt, ack_len);

        // Suppress duplicate delivery if our previous ACK was lost
        if (have_last_rx && last_rx_src == src && last_rx_seq == seq) {
            printf("LINK: duplicate msg seq=%u from %02X (re-ACKed)\n", seq, src);
            return;
        }
        have_last_rx = true;
        last_rx_src  = src;
        last_rx_seq  = seq;

        if (q_from_radio) {
            radio_evt_t evt = { 0 };
            evt.type = RADIO_RECEIVED;
            evt.len  = len;
            memcpy(evt.text, body + 4, len);
            evt.text[len] = '\0';
            if (xQueueSend(q_from_radio, &evt, pdMS_TO_TICKS(100)) != pdTRUE) {
                printf("LINK: q_from_radio full, dropped RX msg\n");
            }
        }
    }
}

// ---------------------------------------------------------------- RX task (ASCII + binary)
typedef enum {
    RX_IDLE = 0,
    RX_ASCII,
    RX_BIN_SYNC2,
    RX_BIN_HDR,
    RX_BIN_BODY,
} rx_state_t;

static void rx_task(void *arg)
{
    (void)arg;
    rx_state_t st = RX_IDLE;
    char    line[128];
    int     line_len = 0;

    // body holds [LEN, TYPE, SRC, SEQ, TEXT[0..160], CRC_HI, CRC_LO] -> max 166 bytes
    uint8_t bin_buf[4 + MSG_MAX_BYTES + 2];
    int     bin_pos = 0;
    int     bin_need = 0;
    uint8_t bin_len = 0;
    int64_t bin_last_us = 0;

    uint8_t c;
    for (;;) {
        int got = uart_read_bytes(HC12_UART, &c, 1, pdMS_TO_TICKS(50));
        int64_t now = esp_timer_get_time();

        // Inter-byte timeout resets binary parser if a frame is cut short or corrupted
        if (st >= RX_BIN_SYNC2 && (now - bin_last_us) > FRAME_BYTE_TMO_US) {
            bad_lines++;
            st = RX_IDLE;
            bin_pos = 0;
        }
        if (got != 1) continue;

        switch (st) {
            case RX_IDLE:
                if (c == FRAME_SYNC0) {
                    st = RX_BIN_SYNC2;
                    bin_last_us = now;
                } else if (c >= 0x20 && c <= 0x7E) {
                    line[0] = (char)c;
                    line_len = 1;
                    st = RX_ASCII;
                }
                break;

            case RX_ASCII:
                if (c == '\n') {
                    line[line_len] = '\0';
                    if (line_len > 0) handle_ascii_line(line);
                    line_len = 0;
                    st = RX_IDLE;
                } else if (c == FRAME_SYNC0) {
                    // Interrupted ASCII line followed by binary sync
                    line_len = 0;
                    st = RX_BIN_SYNC2;
                    bin_last_us = now;
                } else if (c != '\r') {
                    if (line_len < (int)sizeof(line) - 1) line[line_len++] = (char)c;
                    else { line_len = 0; st = RX_IDLE; }
                }
                break;

            case RX_BIN_SYNC2:
                bin_last_us = now;
                if (c == FRAME_SYNC1) {
                    st = RX_BIN_HDR;
                    bin_pos = 0;
                } else if (c == FRAME_SYNC0) {
                    // Stay waiting for 0x55
                } else {
                    bad_lines++;
                    st = RX_IDLE;
                }
                break;

            case RX_BIN_HDR:
                bin_last_us = now;
                bin_buf[bin_pos++] = c;
                if (bin_pos == 1) {
                    // Sanity check LEN immediately
                    if (c > MSG_MAX_BYTES) {
                        bad_lines++;
                        st = RX_IDLE;
                    } else {
                        bin_len = c;
                    }
                } else if (bin_pos == 2) {
                    // Sanity check TYPE and LEN/TYPE consistency
                    bool valid_type = (c == FRAME_TYPE_MSG && bin_len > 0) ||
                                      (c == FRAME_TYPE_ACK && bin_len == 0);
                    if (!valid_type) {
                        bad_lines++;
                        st = RX_IDLE;
                    }
                } else if (bin_pos == 4) {
                    // Header complete (LEN, TYPE, SRC, SEQ); now read TEXT + 2 CRC bytes
                    bin_need = 4 + bin_len + 2;
                    st = RX_BIN_BODY;
                }
                break;

            case RX_BIN_BODY:
                bin_last_us = now;
                bin_buf[bin_pos++] = c;
                if (bin_pos >= bin_need) {
                    handle_bin_frame(bin_buf, bin_len);
                    st = RX_IDLE;
                    bin_pos = 0;
                }
                break;
        }
    }
}

// ---------------------------------------------------------------- TX + retry + re-probe task
static bool probe_and_config_hc12(void)
{
    xSemaphoreTake(uart_tx_lock, portMAX_DELAY);
    char resp[120];
    start_attempts++;
    vTaskDelay(pdMS_TO_TICKS(300));
    bool ok = false;
    for (int attempt = 0; attempt < 5 && !ok; attempt++) {
        ok = hc12_try_at(resp, sizeof(resp));
        if (!ok) printf("LINK: HC-12 no reply (try %d/5)%s%s\n",
                        attempt + 1, resp[0] ? ", got: " : "", resp);
    }
    if (ok) {
        hc12_cmd(LINK_POWER_CMD, resp, sizeof(resp)); printf("LINK %s   -> '%s'\n", LINK_POWER_CMD, resp);
        hc12_cmd("AT+C001", resp, sizeof(resp));      printf("LINK AT+C001 -> '%s'\n", resp);
        hc12_cmd("AT+RX", resp, sizeof(resp));        printf("LINK AT+RX   -> '%s'\n", resp);
    }
    gpio_set_level(HC12_SET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
    uart_flush_input(HC12_UART);
    last_try_us = esp_timer_get_time();
    link_ok = ok;
    xSemaphoreGive(uart_tx_lock);

    if (!ok) {
        printf("LINK: HC-12 not answering, will retry in 10 s\n");
    } else {
        printf("LINK: started (after %lu start attempt%s), my id %s, power %s, channel 001\n",
               (unsigned long)start_attempts, start_attempts == 1 ? "" : "s", my_id, LINK_POWER_LABEL);
    }
    return ok;
}

static void tx_task(void *arg)
{
    (void)arg;
    out_msg_t msg;
    uint8_t frame[8 + MSG_MAX_BYTES];

    for (;;) {
        if (!q_to_radio || xQueueReceive(q_to_radio, &msg, pdMS_TO_TICKS(500)) != pdTRUE) {
            // Re-probe missing HC-12 from here so app_main sensor/UI loop never stalls
            if (!link_ok && (esp_timer_get_time() - last_try_us) > LINK_RETRY_US) {
                if (probe_and_config_hc12() && !rx_handle) {
                    xTaskCreate(rx_task, "hc12_rx", 4096, NULL, 6, &rx_handle);
                }
            }
            continue;
        }

        if (!link_ok || msg.len == 0 || msg.len > MSG_MAX_BYTES) {
            if (q_from_radio) {
                radio_evt_t evt = { .type = RADIO_FAILED, .id = msg.id };
                xQueueSend(q_from_radio, &evt, pdMS_TO_TICKS(100));
            }
            continue;
        }

        uint8_t seq = ++tx_seq;
        size_t flen = build_frame(frame, FRAME_TYPE_MSG, my_mac5, seq,
                                  (const uint8_t *)msg.text, (uint8_t)msg.len, 0);

        atomic_store(&tx_in_flight, true);
        bool delivered = false;

        for (int attempt = 0; attempt <= FRAME_MAX_RETRIES; attempt++) {
            ack_received   = false;
            ack_expect_seq = seq;
            ack_waiting    = true;
            ulTaskNotifyTake(pdTRUE, 0);   // clear any stale notification

            uart_send_locked(frame, flen);

            uint64_t tmo_us = (uint64_t)(FRAME_ACK_BASE_US + (esp_random() % FRAME_ACK_JITTER));
            esp_timer_stop(ack_timer);
            esp_timer_start_once(ack_timer, tmo_us);

            // Wait for either RX ACK notification or esp_timer timeout notification
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
            esp_timer_stop(ack_timer);
            ack_waiting = false;

            if (ack_received) {
                delivered = true;
                break;
            }
            printf("LINK: msg #%lu seq=%u no ACK (attempt %d/%d)\n",
                   (unsigned long)msg.id, seq, attempt + 1, FRAME_MAX_RETRIES + 1);
        }

        atomic_store(&tx_in_flight, false);

        if (q_from_radio) {
            radio_evt_t evt = {
                .type = delivered ? RADIO_DELIVERED : RADIO_FAILED,
                .id   = msg.id,
            };
            xQueueSend(q_from_radio, &evt, pdMS_TO_TICKS(100));
        }
    }
}

// ---------------------------------------------------------------- public API
static void recent_in(uint32_t *got, uint32_t *of)
{
    if (!peer_seen) { *got = *of = 0; return; }
    int64_t age_us = esp_timer_get_time() - last_rx_us;
    uint32_t silent = age_us > 1600000 ? (uint32_t)((age_us - 500000) / 1100000) : 0;
    uint32_t mask = silent >= 32 ? 0 : recent_mask << silent;
    uint32_t span = peer_last_seq - peer_first_seq + 1 + silent;
    *of = span < LINK_WINDOW ? span : LINK_WINDOW;
    *got = __builtin_popcount(mask & ((1u << *of) - 1));
}

bool link_start(void)
{
    uint8_t mac[6];
    esp_efuse_mac_get_default(mac);
    my_mac5 = mac[5];
    snprintf(my_id, sizeof(my_id), "%02X", my_mac5);
    tx_seq = (uint8_t)(esp_random() & 0xFF);

    if (!uart_tx_lock) uart_tx_lock = xSemaphoreCreateMutex();
    if (!ack_timer) {
        esp_timer_create_args_t ta = {
            .callback = on_ack_timeout,
            .name     = "hc12_ack",
        };
        ESP_ERROR_CHECK(esp_timer_create(&ta, &ack_timer));
    }

    bool ok = probe_and_config_hc12();

    if (ok && !rx_handle) {
        xTaskCreate(rx_task, "hc12_rx", 4096, NULL, 6, &rx_handle);
    }
    if (!tx_handle) {
        xTaskCreate(tx_task, "hc12_tx", 4096, NULL, 5, &tx_handle);
    }
    return ok;
}

void link_send(void)
{
    // Skip heartbeats if radio is absent or while a chat message is awaiting ACK
    if (!link_ok || atomic_load(&tx_in_flight)) return;
    uint32_t got, of;
    recent_in(&got, &of);
    char pkt[48];
    int n = snprintf(pkt, sizeof(pkt), "PING,%s,%lu,%lu,%lu\n", my_id,
                     (unsigned long)++tx_count, (unsigned long)rx_count, (unsigned long)got);
    uart_send_locked(pkt, (size_t)n);
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
    printf("LINK me=%s %s | IN from %s: last20 %lu/%lu, total rx=%lu loss=%.1f%%, last %.1fs ago"
           " | OUT %s heard last20 %lu/%lu, total %lu of my %lu | bad=%lu\n",
           my_id, LINK_POWER_LABEL, peer_id, (unsigned long)got, (unsigned long)of,
           (unsigned long)rx_count, loss_pct(), age,
           peer_id, (unsigned long)peer_got_recent, (unsigned long)out_of,
           (unsigned long)peer_got_ours, (unsigned long)tx_count, (unsigned long)bad_lines);
}

void link_ping(void)
{
    if (!link_ok || atomic_load(&tx_in_flight)) return;
    ping_answered = false;
    ping_seq++;
    char pkt[24];
    int len = snprintf(pkt, sizeof(pkt), "Q,%s,%lu\n", my_id, (unsigned long)ping_seq);
    ping_sent_us = esp_timer_get_time();
    uart_send_locked(pkt, (size_t)len);
}

bool link_ping_poll(int *rtt_ms)
{
    if (!ping_answered) return false;
    ping_answered = false;
    *rtt_ms = ping_rtt_ms;
    return true;
}

void link_stats(link_stats_t *st)
{
    st->ok         = link_ok;
    st->peer_seen  = peer_seen;
    st->heard_s    = peer_seen ? (esp_timer_get_time() - last_heard_us) / 1e6f : -1.0f;
    st->ping_ms    = ping_rtt_ms;
    st->tx         = tx_count;
    st->rx         = rx_count;
    st->loss       = loss_pct();
    st->bad_frames = bad_lines;
}

void link_sleep(void)
{
    if (!uart_tx_lock || !link_ok) return;
    xSemaphoreTake(uart_tx_lock, portMAX_DELAY);
    link_ok = false;
    char resp[32];
    gpio_set_level(HC12_SET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    hc12_cmd("AT+SLEEP", resp, sizeof(resp));
    printf("LINK: AT+SLEEP -> '%s'\n", resp);
    gpio_set_level(HC12_SET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    xSemaphoreGive(uart_tx_lock);
}

bool link_send_bad_crc_test(const char *text)
{
    if (!link_ok || !text) return false;
    size_t len = strlen(text);
    if (len == 0 || len > MSG_MAX_BYTES) return false;
    uint8_t frame[8 + MSG_MAX_BYTES];
    // Flip all CRC bits so receiver must reject it on CRC check
    size_t flen = build_frame(frame, FRAME_TYPE_MSG, my_mac5, ++tx_seq,
                              (const uint8_t *)text, (uint8_t)len, 0xFFFF);
    uart_send_locked(frame, flen);
    printf("LINK: sent corrupted-CRC test frame (%u B)\n", (unsigned)len);
    return true;
}
