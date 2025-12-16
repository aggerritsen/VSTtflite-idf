#ifndef WIFI_H
#define WIFI_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Wi-Fi in Access Point (AP) mode
 *
 * Creates an AP with a fixed IP (192.168.4.1).
 * Safe to call once during boot before starting HTTPD.
 */
void wifi_ap_start(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_H
