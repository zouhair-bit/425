// Router task connecting link (q_from_radio) -> msg_store + OLED (q_to_display).
#include <stdio.h>
#include "tlog.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "app_queues.h"
#include "msg_store.h"
#include "router.h"

#define Q_TO_RADIO_DEPTH    5
#define Q_FROM_RADIO_DEPTH  10
#define Q_TO_DISPLAY_DEPTH  6

QueueHandle_t q_to_radio;
QueueHandle_t q_from_radio;
QueueHandle_t q_to_display;

static char s_peer[8] = "DELTA";

static void router_task(void *arg)
{
    (void)arg;
    radio_evt_t evt;
    for (;;) {
        if (xQueueReceive(q_from_radio, &evt, portMAX_DELAY) != pdTRUE) continue;

        switch (evt.type) {
            case RADIO_DELIVERED:
                msg_store_set_status(evt.id, MSG_STATUS_DELIVERED);
                printf("ROUTER: msg #%lu delivered\n", (unsigned long)evt.id);
                break;

            case RADIO_FAILED:
                msg_store_set_status(evt.id, MSG_STATUS_FAILED);
                printf("ROUTER: msg #%lu failed\n", (unsigned long)evt.id);
                break;

            case RADIO_RECEIVED: {
                uint32_t id = msg_store_add_in(evt.text, evt.len);
                printf("ROUTER: rx msg #%lu from %s (%u B)\n",
                       (unsigned long)id, s_peer, (unsigned)evt.len);
                display_msg_t dm = { 0 };
                snprintf(dm.from, sizeof(dm.from), "%s", s_peer);
                memcpy(dm.text, evt.text, evt.len <= MSG_MAX_BYTES ? evt.len : MSG_MAX_BYTES);
                dm.text[evt.len <= MSG_MAX_BYTES ? evt.len : MSG_MAX_BYTES] = '\0';
                // Non-blocking send so display backpressure can never stall the router
                if (xQueueSend(q_to_display, &dm, 0) != pdTRUE) {
                    display_msg_t drop;
                    xQueueReceive(q_to_display, &drop, 0);
                    xQueueSend(q_to_display, &dm, 0);
                }
                break;
            }
        }
    }
}

void router_init(const char *peer_name)
{
    if (peer_name && *peer_name) snprintf(s_peer, sizeof(s_peer), "%s", peer_name);
    if (!q_to_radio)   q_to_radio   = xQueueCreate(Q_TO_RADIO_DEPTH,   sizeof(out_msg_t));
    if (!q_from_radio) q_from_radio = xQueueCreate(Q_FROM_RADIO_DEPTH, sizeof(radio_evt_t));
    if (!q_to_display) q_to_display = xQueueCreate(Q_TO_DISPLAY_DEPTH, sizeof(display_msg_t));
    xTaskCreate(router_task, "router", 3072, NULL, 4, NULL);
}
