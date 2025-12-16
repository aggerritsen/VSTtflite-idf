#pragma once

#include "esp_err.h"
#include <string>

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// UART / power
// -----------------------------------------------------------------------------
esp_err_t modem_init_uart(void);
void      modem_deinit_uart(void);

esp_err_t modem_power_on(void);   // no-op on AXP2101 boards
bool      wait_for_modem(void);

// -----------------------------------------------------------------------------
// Time
// -----------------------------------------------------------------------------
bool modem_get_timestamp(std::string &ts_compact,
                         std::string &ts_iso8601);

#ifdef __cplusplus
}
#endif
