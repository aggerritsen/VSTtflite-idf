#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Start HTTP server (call once after WiFi is up)
void http_server_start(void);

// Update the last captured JPEG frame (called by camera task)
void httpd_update_last_frame(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
