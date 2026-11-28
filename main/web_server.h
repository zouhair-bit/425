// Shared HTTP server for TREELINE chat web app, REST API, and /ota endpoints.
#pragma once

#include "esp_err.h"
#include "ota.h"

esp_err_t web_server_start(ota_status_cb_t ota_cb);
void      web_server_stop(void);
