// Shared FreeRTOS queues and message types connecting web_server, link (radio),
// router, and OLED UI.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define MSG_MAX_BYTES 160   // max UTF-8 payload bytes per radio packet

// Outgoing message: web_server -> link TX task
typedef struct {
    uint32_t id;                      // msg_store id, echoed back on DELIVERED/FAILED
    uint16_t len;                     // UTF-8 byte length (1..MSG_MAX_BYTES)
    char     text[MSG_MAX_BYTES + 1]; // NUL-terminated UTF-8 payload
} out_msg_t;

typedef enum {
    RADIO_DELIVERED = 0,
    RADIO_FAILED,
    RADIO_RECEIVED,
} radio_evt_type_t;

// Radio event: link TX/RX tasks -> router task
typedef struct {
    radio_evt_type_t type;
    uint32_t id;                      // valid for RADIO_DELIVERED / RADIO_FAILED
    uint16_t len;                     // valid for RADIO_RECEIVED
    char     text[MSG_MAX_BYTES + 1]; // NUL-terminated UTF-8 payload for RADIO_RECEIVED
} radio_evt_t;

// Display notification: router task -> UI task
typedef struct {
    char from[8];
    char text[MSG_MAX_BYTES + 1];
} display_msg_t;

extern QueueHandle_t q_to_radio;      // web_server sends (0 timeout), link TX task receives
extern QueueHandle_t q_from_radio;    // link TX/RX send, router task receives
extern QueueHandle_t q_to_display;    // router sends incoming messages, UI task receives
