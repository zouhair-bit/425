// Thread-safe 50-message RAM ring buffer with monotonic id and rev counters
// and Unix epoch time sync.
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "app_queues.h"

#define MSG_STORE_CAP 50

typedef enum {
    MSG_DIR_OUT = 0,
    MSG_DIR_IN,
} msg_dir_t;

typedef enum {
    MSG_STATUS_SENDING = 0,
    MSG_STATUS_DELIVERED,
    MSG_STATUS_FAILED,
    MSG_STATUS_RECEIVED,
} msg_status_t;

typedef struct {
    uint32_t     id;
    uint32_t     rev;
    int64_t      time;       // Unix seconds (0 if clock not yet synced)
    char         from[8];    // "ECHO" or "DELTA"
    msg_dir_t    dir;
    msg_status_t status;
    uint16_t     len;
    char         text[MSG_MAX_BYTES + 1];
} msg_t;

void     msg_store_init(const char *local_name, const char *peer_name);
uint32_t msg_store_add_out(const char *text, size_t len);
uint32_t msg_store_add_in(const char *text, size_t len);
void     msg_store_set_status(uint32_t id, msg_status_t status);

// Copies every message with rev > since_rev in chronological (oldest->newest) order.
// If out_last_rev != NULL, writes the current rev captured under the same mutex lock.
int      msg_store_since(uint32_t since_rev, msg_t *out, int max_out, uint32_t *out_last_rev);

// Copies up to max_out messages with id > after_id and since_rev < rev <= max_rev
// in chronological order (for lock-free chunked HTTP streaming without a full-ring copy).
int      msg_store_batch(uint32_t since_rev, uint32_t max_rev, uint32_t after_id,
                         msg_t *out, int max_out);
uint32_t msg_store_last_rev(void);

// Time synchronization from browser POST /time
void     msg_store_set_time(int64_t epoch_sec);
int64_t  msg_store_now(void);

const char *msg_status_str(msg_status_t status);
const char *msg_dir_str(msg_dir_t dir);
