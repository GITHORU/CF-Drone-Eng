// Motor output through MOSFETs
// For ESCs, set pwmStop, pwmMin and pwmMax to pulse widths in microseconds and lower pwmFrequency (to 400)

#include "util.h"
#include "board_config.h"

float motors[4]; // normalized motor thrusts in range [0..1]

// Motor pins (MOTOR_REAR_LEFT=0, MOTOR_REAR_RIGHT=1, MOTOR_FRONT_RIGHT=2, MOTOR_FRONT_LEFT=3)
int motorPins[4] = BOARD_MOTOR_PINS; // RL, RR, FR, FL

int pwmFrequency = 25000; // ESP32-C3 LEDC default clock is the 40 MHz XTAL; 10-bit max is 39062 Hz.
                          // ESP32 APB is 80 MHz; 10-bit max is 78125 Hz.
                          // 25 kHz divides cleanly on both clocks, with margin.
int pwmResolution = 10;
int pwmStop = 0;
int pwmMin = 0;
int pwmMax = -1; // -1 = duty-cycle mode (MOSFET direct drive). For an ESC, set the max pulse width in microseconds.

// Motors array indexes:
const int MOTOR_REAR_LEFT = 0;
const int MOTOR_REAR_RIGHT = 1;
const int MOTOR_FRONT_RIGHT = 2;
const int MOTOR_FRONT_LEFT = 3;

void setupMotors() {
	print("Setup Motors\n");

	// Detach every pin first (clears a previous LEDC channel on re-entry), then drive low so motors cannot spin.
	for (int i = 0; i < 4; i++) {
		ledcDetach(motorPins[i]);         // first call with nothing attached returns false and is harmless
		pinMode(motorPins[i], OUTPUT);
		digitalWrite(motorPins[i], LOW);
	}

	// configure pins
	for (int i = 0; i < 4; i++) {
		bool ok = ledcAttach(motorPins[i], pwmFrequency, pwmResolution);
		if (ok) {
			double actual = ledcChangeFrequency(motorPins[i], pwmFrequency, pwmResolution);
			if (actual > 0) pwmFrequency = (int)round(actual); // keep the double so the rounded Hz does not lose precision
		}
		print("  motor%d pin=%d ledcAttach=%s\n", i, motorPins[i], ok ? "OK" : "FAIL");
	}

	sendMotors();
	print("Motors initialized\n");
}

int getDutyCycle(float value) {
	value = constrain(value, 0, 1);
	if (pwmMax >= 0) { // pulse-width mode (ESC)
		float pwm = mapf(value, 0, 1, pwmMin, pwmMax);
		if (value == 0) pwm = pwmStop;
		float duty = mapf(pwm, 0, 1000000 / pwmFrequency, 0, (1 << pwmResolution) - 1);
		return round(duty);
	} else { // duty-cycle mode (MOSFET direct drive)
		return round(value * ((1 << pwmResolution) - 1));
	}
}

void sendMotors() {
	for (int i = 0; i < 4; i++) {
		ledcWrite(motorPins[i], getDutyCycle(motors[i]));
	}
}

bool motorsActive() {
	return motors[0] != 0 || motors[1] != 0 || motors[2] != 0 || motors[3] != 0;
}

void testMotor(int n) {
	print("Testing motor %d\n", n);
	motors[n] = 0.3;
	delay(50); // ESP32 may need to wait until the end of the current cycle to change duty https://github.com/espressif/arduino-esp32/issues/5306
	sendMotors();
	pause(3);
	motors[n] = 0;
	sendMotors();
	print("Done\n");
}
