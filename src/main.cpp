#include "accel.hpp"
#include "baro.hpp"
#include "config.hpp"
#include "kalman.hpp"
#include "log.hpp"
#include "scheduler.hpp"
#include "util.hpp"

#include <SPI.h>
#include <Wire.h>

#include <cmath>

struct ChannelStatus {
	uint32_t fire_time;
	bool firing;
};

// Prototypes
void command_step();
void blink_step();
void print_step();

#ifdef KALMAN_GAINS
static KalmanFilter kf(KALMAN_PERIOD / 1000.0f, {KALMAN_GAINS});
#else
static KalmanFilter kf(KALMAN_PERIOD / 1000.0f,
		ALTITUDE_SIGMA, ACCELERATION_SIGMA, MODEL_SIGMA);
#endif

void setup()
{

#ifdef PIN_LAUNCH
	pinMode(PIN_LAUNCH, OUTPUT);
	digitalWrite(PIN_LAUNCH, LOW);
#endif

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	analogReadResolution(12);  // Enable full resolution

	Serial.begin(9'600);

	Serial.println(F("Flight Computer " __DATE__ " " __TIME__));

	Wire.begin();

	SPI.begin();

	baro_setup();
	accel_setup();
#if LOG_ENABLE
	log_setup();
#endif

	scheduler_add(TaskId::Command, Task(command_step, 100'000L, 10));
	scheduler_add(TaskId::Print, Task(print_step, 3'000'000L, 3000));
	scheduler_add(TaskId::Blink, Task(blink_step, (KALMAN_PERIOD / 2) * 1000L, 20));
}

void loop()
{
	uint32_t wait_time = schedule();
	if (wait_time > 4) {
		delayMicroseconds(wait_time - 4);
	}
}

void command_step()
{
	switch (Serial.read()) {
	case 'r':
		log_print_all();
		break;
	default:
		// Serial.println("Unrecognized command.");
		break;
	}
}

void blink_step()
{
	static bool on = false;
	digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
	on = !on;
}

void print_step()
{
	accel_print();
	baro_print();
}
