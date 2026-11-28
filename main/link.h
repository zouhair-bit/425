// HC-12 radio link (link.c / hc12.c)
// ASCII lines:   heartbeat ("PING,...") and round-trip ping ("Q,..." / "A,...")
// Binary frames: chat messages ('M') and ACKs ('K') with 0xAA 0x55 sync + CRC-16 + 3 retries.
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool     ok;
    bool     peer_seen;
    float    heard_s;     // seconds since any valid packet from peer (-1 = never)
    int      ping_ms;     // last measured round-trip ping ms (-1 = none yet)
    uint32_t tx;
    uint32_t rx;
    float    loss;        // % of peer heartbeats missed
    uint32_t bad_frames;  // corrupted/CRC-failed frames + malformed lines
} link_stats_t;

void hc12_test(void);                       // hc12.c: finds module, configures UART
bool link_start(void);                      // configures HC-12 and starts RX/TX tasks
void link_send(void);                       // sends 1 Hz heartbeat (skipped while TX busy)
void link_print(void);
void link_ping(void);
bool link_ping_poll(int *rtt_ms);
void link_stats(link_stats_t *st);
void link_sleep(void);                      // AT+SLEEP for low-battery shutdown
int64_t link_last_tx_us(void);              // esp_timer time the last radio transmission ends
bool link_send_bad_crc_test(const char *text);  // demo helper: sends intentionally bad CRC frame
