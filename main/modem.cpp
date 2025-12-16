// File: modem.cpp
// SIM7070 / SIM7080 / SIM7090 modem support
// ESP32-S3 + AXP2101 (auto-boot boards)

#include "modem.h"

#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ctime>
#include <sys/time.h>


#include <string>
#include <vector>
#include <algorithm>

static const char *TAG = "MODEM";

// -----------------------------------------------------------------------------
// UART configuration
// -----------------------------------------------------------------------------

#define MODEM_UART_NUM 		UART_NUM_1
#define MODEM_UART_BAUD 	115200
#define MODEM_UART_TX_PIN 	5
#define MODEM_UART_RX_PIN 	4
#define MODEM_UART_BUF_SIZE 4096

#define AT_POLL_INTERVAL_MS 500
#define AT_TOTAL_TIMEOUT_MS 30000

// -----------------------------------------------------------------------------
// Utilities
// -----------------------------------------------------------------------------

static std::string trim(const std::string &s)
{
	size_t start = s.find_first_not_of(" \r\n\t");
	if (start == std::string::npos) return "";
	size_t end = s.find_last_not_of(" \r\n\t");
	return s.substr(start, end - start + 1);
}

static bool is_plausible_year(int year)
{
	return year >= 2020 && year <= 2099;
}

// -----------------------------------------------------------------------------
// Low-level line reader
// -----------------------------------------------------------------------------

static esp_err_t modem_read_line(std::string &out, int timeout_ms)
{
	out.clear();
	uint8_t ch;
	int64_t t0 = esp_timer_get_time() / 1000;

	while (true) {
		int n = uart_read_bytes(MODEM_UART_NUM, &ch, 1, pdMS_TO_TICKS(10));
		if (n == 1) {
			if (ch == '\n') {
				out = trim(out);
				if (!out.empty()) return ESP_OK;
			} else if (ch != '\r') {
				out.push_back((char)ch);
			}
		}

		if ((esp_timer_get_time() / 1000) - t0 > timeout_ms) {
			return ESP_ERR_TIMEOUT;
		}
	}
}

// -----------------------------------------------------------------------------
// AT command helpers
// -----------------------------------------------------------------------------

static esp_err_t modem_send_cmd_internal(
	const char *cmd,
	std::vector<std::string> &resp,
	int timeout_ms)
{
	resp.clear();

	std::string full = cmd ? cmd : "";
	if (!full.empty() && full.find("\r\n") == std::string::npos) {
		full += "\r\n";
	}

	uart_flush(MODEM_UART_NUM);

	if (!full.empty()) {
		ESP_LOGI(TAG, "AT CMD: %s", cmd);
		uart_write_bytes(MODEM_UART_NUM, full.data(), full.size());
	}

	int64_t t0 = esp_timer_get_time() / 1000;

	while (true) {
		std::string line;
		if (modem_read_line(line, 200) == ESP_OK) {
			ESP_LOGI(TAG, "AT RSP: %s", line.c_str());
			if (line == "OK") 	return ESP_OK;
			if (line == "ERROR") return ESP_FAIL;
			resp.push_back(line);
		}

		if ((esp_timer_get_time() / 1000) - t0 > timeout_ms) {
			return ESP_ERR_TIMEOUT;
		}
	}
}

esp_err_t modem_send_cmd(const char *cmd,
						 std::vector<std::string> &resp,
						 int timeout_ms)
{
	return modem_send_cmd_internal(cmd, resp, timeout_ms);
}

esp_err_t modem_send_cmd_expect_ok(const char *cmd, int timeout_ms)
{
	std::vector<std::string> dummy;
	return modem_send_cmd_internal(cmd, dummy, timeout_ms);
}

// -----------------------------------------------------------------------------
// Network registration helper (NEW, internal only)
// -----------------------------------------------------------------------------

static void wait_for_network_registration(uint32_t timeout_ms)
{
	int64_t start = esp_timer_get_time() / 1000;
	std::vector<std::string> resp;

	while ((esp_timer_get_time() / 1000) - start < timeout_ms) {

		if (modem_send_cmd("AT+CEREG?", resp, 2000) == ESP_OK) {
			for (auto &l : resp) {
				if (l.find(",1") != std::string::npos ||
					l.find(",5") != std::string::npos) {
					ESP_LOGI(TAG, "Network registered (CEREG)");
					return;
				}
			}
		}

		if (modem_send_cmd("AT+CREG?", resp, 2000) == ESP_OK) {
			for (auto &l : resp) {
				if (l.find(",1") != std::string::npos ||
					l.find(",5") != std::string::npos) {
					ESP_LOGI(TAG, "Network registered (CREG)");
					return;
				}
			}
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	ESP_LOGW(TAG, "Network registration timeout");
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

extern "C" esp_err_t modem_init_uart(void)
{
	ESP_LOGI(TAG, "Initializing modem UART...");

	uart_config_t cfg = {};
	cfg.baud_rate = MODEM_UART_BAUD;
	cfg.data_bits = UART_DATA_8_BITS;
	cfg.parity 	= UART_PARITY_DISABLE;
	cfg.stop_bits = UART_STOP_BITS_1;
	cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	cfg.source_clk = UART_SCLK_DEFAULT;

	ESP_ERROR_CHECK(uart_param_config(MODEM_UART_NUM, &cfg));
	ESP_ERROR_CHECK(uart_set_pin(
		MODEM_UART_NUM,
		MODEM_UART_TX_PIN,
		MODEM_UART_RX_PIN,
		UART_PIN_NO_CHANGE,
		UART_PIN_NO_CHANGE
	));
	ESP_ERROR_CHECK(uart_driver_install(
		MODEM_UART_NUM,
		MODEM_UART_BUF_SIZE,
		MODEM_UART_BUF_SIZE,
		0,
		nullptr,
		0
	));

	return ESP_OK;
}

extern "C" bool wait_for_modem(void)
{
	ESP_LOGI(TAG, "Waiting for modem AT readiness...");

	int64_t start = esp_timer_get_time() / 1000;

	while ((esp_timer_get_time() / 1000) - start < AT_TOTAL_TIMEOUT_MS) {

		uart_write_bytes(MODEM_UART_NUM, "AT\r\n", 4);

		std::string line;
		if (modem_read_line(line, 500) == ESP_OK && line == "OK") {

			modem_send_cmd_expect_ok("AT+CLTS=1", 2000);
			modem_send_cmd_expect_ok("AT+CTZR=1", 2000);

			// NEW: wait for network before trusting time
			wait_for_network_registration(60000);

			return true;
		}

		vTaskDelay(pdMS_TO_TICKS(AT_POLL_INTERVAL_MS));
	}

	return false;
}

extern "C" bool modem_get_timestamp(std::string &ts_compact,
									std::string &ts_iso8601)
{
	std::vector<std::string> resp;

	for (int attempt = 0; attempt < 10; attempt++) {

		if (modem_send_cmd("AT+CCLK?", resp, 5000) != ESP_OK) {
			vTaskDelay(pdMS_TO_TICKS(1000));
			continue;
		}

		for (auto &line : resp) {
			if (line.find("+CCLK:") != 0) continue;

			auto p1 = line.find('"');
			auto p2 = line.find('"', p1 + 1);
			if (p1 == std::string::npos || p2 == std::string::npos) continue;

			std::string b = line.substr(p1 + 1, p2 - p1 - 1);
			if (b.size() < 20) continue;

			int yy = std::stoi(b.substr(0, 2));
			int year = 2000 + yy;
			if (!is_plausible_year(year)) {
				ESP_LOGW(TAG, "Ignoring implausible modem time: %s", b.c_str());
				continue;
			}

			// unchanged compact format
			ts_compact =
				std::to_string(year) +
				b.substr(3,2) +
				b.substr(6,2) + "_" +
				b.substr(9,2) +
				b.substr(12,2) +
				b.substr(15,2);

			// timezone-aware ISO8601
			char tz_sign = b[17];
			int tz_q = std::stoi(b.substr(18, 2));
			int tz_min = tz_q * 15;

			char tz_buf[16];
			snprintf(tz_buf, sizeof(tz_buf),
					 "%c%02d:%02d",
					 tz_sign,
					 tz_min / 60,
					 tz_min % 60);

			ts_iso8601 =
				std::to_string(year) + "-" +
				b.substr(3,2) + "-" +
				b.substr(6,2) + "T" +
				b.substr(9,2) + ":" +
				b.substr(12,2) + ":" +
				b.substr(15,2) +
				tz_buf;

			return true;
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	return false;
}

