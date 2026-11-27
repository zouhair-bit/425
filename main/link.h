// HC-12 radio link (link.c / hc12.c)
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool ok, peer_seen;
    float heard_s;          // s since any packet from the peer (-1 = never)
    uint32_t tx, rx;
    float loss;             // % of the peer's heartbeats that did not arrive
} link_stats_t;

void hc12_test(void);                       // hc12.c: finds the module, sets up the UART
bool link_start(void);
void link_send(void);                       // heartbeat, call about once a second
void link_print(void);
void link_ping(void);
bool link_ping_poll(int *rtt_ms);
void link_send_text(const char *text);
void link_stats(link_stats_t *st);
void link_set_msg_cb(void (*cb)(const char *text));
void link_set_ping_cb(void (*cb)(void));
int64_t link_last_tx_us(void);              // esp_timer time of the last radio transmission    // called (receive task) after answering a peer's ping
