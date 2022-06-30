#include "log.hpp"

#include "config.hpp"
#include "flash.hpp"
#include "scheduler.hpp"
#include "util.hpp"

#include <Arduino.h>
#include <EEPROM.h>

#include <cassert>

#define LOG_BUF_SIZE (PRELOG_MS / KALMAN_PERIOD)
#define LOG_WRITE_BUF_SIZE (sizeof(LogMessage) * LOG_BUF_SIZE)
#define LOG_MAX_FLIGHT_MESSAGES (FLIGHT_FLASH_FLIGHT_SIZE / sizeof(LogMessage))
#define LOG_MAX_FLIGHT_TIME (LOG_MAX_FLIGHT_MESSAGES / (1000 / KALMAN_PERIOD))

static_assert(LOG_MAX_FLIGHT_TIME > 5 * 60, "Insufficient space available for flight logs");

static void log_step();
static void log_print_flight(size_t flight);
static void log_print_msg(const LogMessage &msg);

RingBuffer<LogMessage, LOG_BUF_SIZE> log_buf;
RingBuffer<uint8_t, LOG_WRITE_BUF_SIZE> write_buf;

static bool write_enabled = false;
static size_t current_page = 0;
static size_t written_pages = 0;
static uint8_t flight_num;
static bool current_block_erased = false;

void log_setup()
{
	flash_setup();

	flight_num = EEPROM.read(EEPROM_FLIGHT) % FLIGHT_FLASH_FLIGHTS;
	current_page = FLIGHT_FLASH_FLIGHT_PAGES * flight_num;

	scheduler_add(TaskId::LogFlush, Task(log_step, 100'000L, 30'000L));
}

void log_start()
{
	write_enabled = true;
	// Flight started, advance to next flight.
	EEPROM.write(EEPROM_FLIGHT, wrapping_add(flight_num, 1, FLIGHT_FLASH_FLIGHTS));
	// Run one step to move records from the log buffer to the write buffer
	// and start the first erase operation.
	log_step();
}

void log_stop()
{
	write_enabled = false;
}

static void log_step()
{
	// Don't do anything if we haven't started flight yet
	// or if we've run out of storage space.
	if (!write_enabled || written_pages >= FLIGHT_FLASH_FLIGHT_PAGES) {
		return;
	}

	// Move messages from log buffer to write buffer
	LogMessage temp;
	auto temp_bytes = reinterpret_cast<uint8_t *>(&temp);
	while (write_buf.available() >= sizeof(LogMessage)) {
		if (!log_buf.pop(&temp)) {
			break;
		}
		bool ok = write_buf.push(temp_bytes, sizeof(temp), false);
		assert(ok);
	}

	// Write messages from write buffer
	uint8_t page[FLIGHT_FLASH_PAGE_SIZE];
	while (written_pages < FLIGHT_FLASH_FLIGHT_PAGES && !flash_busy()) {
		// Erase block if we've moved into a new one
		if (!current_block_erased && current_page % FLIGHT_FLASH_PAGES_PER_BLOCK == 0) {
			flash_erase(current_page);
			current_block_erased = true;
			break;
		}
		// Clear erased flag
		current_block_erased = false;

		if (!write_buf.pop(page, FLIGHT_FLASH_PAGE_SIZE)) {
			break;
		}

		flash_write(current_page, page);

		++current_page;
		++written_pages;
	}
}

void log_add(const LogMessage &data)
{
	if (!log_buf.push(data, true) && write_enabled) {
		Serial.println("Log buffer overflow!");
	}
}

void log_print_all()
{
	if (write_enabled) {
		Serial.println("Cannot read while in flight!");
		return;
	}

	uint8_t first_flight = EEPROM.read(EEPROM_FLIGHT);

	for (size_t flight_i = 0; flight_i < FLIGHT_FLASH_FLIGHTS; ++flight_i) {
		uint8_t flight = wrapping_add(first_flight, flight_i, FLIGHT_FLASH_FLIGHTS);
		log_print_flight(flight);
		Serial.println("---");
	}
}

static void log_print_flight(size_t flight)
{
	uint8_t page[FLIGHT_FLASH_PAGE_SIZE];
	LogMessage msg;
	RingBuffer<uint8_t, LOG_WRITE_BUF_SIZE> read_buf;
	size_t flight_addr = FLIGHT_FLASH_FLIGHT_PAGES * flight;
	uint32_t last_time = 0;

	for (size_t page_i = 0; page_i < FLIGHT_FLASH_FLIGHT_PAGES; ++page_i) {
		flash_read(flight_addr + page_i, page);

		if (!read_buf.push(page, FLIGHT_FLASH_PAGE_SIZE, false)) {
			Serial.println("Read buffer error.");
			break;
		}

		bool first_msg = true;
		while (read_buf.pop(reinterpret_cast<uint8_t *>(&msg), sizeof(LogMessage))) {
			// Flight ends if difference between timestamps is too large,
			// there is no difference, or the checksum doesn't match.
			if ((!first_msg && (msg.time_ms - last_time > 1000 ||
						msg.time_ms == last_time)) ||
					struct_checksum(msg) != msg.checksum) {
				return;
			}

			log_print_msg(msg);

			last_time = msg.time_ms;
			first_msg = false;
		}
	}
}

static void log_print_msg(const LogMessage &msg)
{
	Serial.print(msg.time_ms);
	Serial.print(',');
	// Position
	Serial.print(msg.state(0));
	Serial.print(',');
	// Velocity
	Serial.print(msg.state(1));
	Serial.print(',');
	// Acceleration
	Serial.print(msg.state(2));
	Serial.print(',');
	Serial.print(msg.temp);
	Serial.print(',');
	Serial.print(msg.pressure);
	Serial.print(',');
	Serial.print(msg.altitude);
	Serial.print(',');
	Serial.print(msg.accel_x);
	Serial.print(',');
	Serial.print(msg.accel_y);
	Serial.print(',');
	Serial.print(msg.accel_z);
	Serial.print(',');
	Serial.print(msg.lat, 6);
	Serial.print(',');
	Serial.print(msg.lon, 6);
	Serial.print(',');
	Serial.print(msg.gps_alt);
	Serial.print(',');
	Serial.print(msg.batt_v);
	Serial.print(',');
	Serial.print(msg.sys_v);
	Serial.println();
}
