// Onboard LED

#include "board_config.h"

#if BOARD_LED_ENABLED

#define BLINK_PERIOD      500000  // slow blink: 500 ms half-period -> 1 Hz
#define BLINK_FAST_PERIOD  62500  // fast blink: 62.5 ms half-period -> 8 Hz


// ---- external state ----
extern bool armed;
extern float t;
extern float controlTime;
extern float rcLossTimeout;
extern float thrustTarget;    // control.ino
extern float batteryVoltage;  // battery.ino
extern bool isInverted; // safety.ino
#if WEB_RC_ENABLED
extern bool webRCEnabled;
extern bool useWebRC;
bool isUsingWebRC();
#endif

void setupLED() {
	pinMode(BOARD_LED_PIN, OUTPUT);
	digitalWrite(BOARD_LED_PIN, BOARD_LED_INVERTED ? HIGH : LOW); // start off
}

void setLED(bool on) {
	static bool state = false;
	if (on == state) {
		return; // don't call digitalWrite if the state is the same
	}
	digitalWrite(BOARD_LED_PIN, (on ^ BOARD_LED_INVERTED) ? HIGH : LOW);
	state = on;
}

void blinkLED() {
	setLED(micros() / BLINK_PERIOD % 2);
}

// Battery LED: threshold depends on flight state.
// In flight (thrustTarget >= 0.15) use L2 (VBAT_LOW_THRESHOLD). L1 does not apply in flight.
// Disarmed or idle-armed uses L1 (VBAT_WARN_THRESHOLD).
bool batteryAlertActive() {
	if (batteryVoltage <= VBAT_ABSENT_THRESHOLD) return false;
	bool flying = armed && thrustTarget >= 0.15f;
	if (flying) return batteryVoltage < VBAT_LOW_THRESHOLD;   // L2: in flight
	else        return batteryVoltage < VBAT_WARN_THRESHOLD;   // L1: disarmed or idle
}

// Any alert: low battery, RC loss, or inverted
bool ledAlertActive() {
	if (isInverted) return true;

	// RC loss (SBUS / CRSF), only while armed
	if (controlTime != 0 && armed && (t - controlTime > rcLossTimeout)) return true;

#if WEB_RC_ENABLED
	// Web RC was active and has now timed out
	if (webRCEnabled && useWebRC && !isUsingWebRC()) return true;
#endif

	if (batteryAlertActive()) return true;

	return false;
}

// Called from the main loop. LED pattern follows flight state.
void updateLED() {
	if (!armed) {
		if (batteryAlertActive()) {
			setLED(micros() / BLINK_FAST_PERIOD % 2); // low battery before arm: fast blink
		} else {
			setLED(false); // idle: off
		}
		return;
	}
	if (ledAlertActive()) {
		setLED(micros() / BLINK_FAST_PERIOD % 2); // any alert: 8 Hz
	} else {
		setLED(micros() / BLINK_PERIOD % 2); // armed, normal: 1 Hz
	}
}

#else // BOARD_LED_ENABLED == 0 (no onboard LED)

void setupLED() {}
void setLED(bool on) {}
void updateLED() {}

#endif // BOARD_LED_ENABLED
