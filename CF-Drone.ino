// Main program
// Build with Arduino IDE 2.3.x or newer. Arduino 1.8.19 is not supported.
// Requires the ESP32 core by Espressif Systems, plus dependencies such as MAVLink.

#include "vector.h"
#include "quaternion.h"
#include "util.h"
#include "board_config.h"

// Wi-Fi and the web remote are set per chip in board_config.h.
// To override, #undef here and #define again.
#define WIFI_ENABLED    BOARD_WIFI_ENABLED
#define WEB_RC_ENABLED  BOARD_WEB_RC_ENABLED

float t = NAN; // current step time, seconds
float dt; // time since the previous step, seconds
float controlRoll, controlPitch, controlYaw, controlThrottle; // pilot input, range [-1, 1]
float controlMode = NAN;
Vector gyro; // gyroscope
Vector acc; // accelerometer, m/s^2
Vector rates; // filtered angular rate, rad/s
Quaternion attitude; // estimated attitude (quaternion)
bool landed; // landed and stationary

void setup() {
	Serial.begin(115200);
	disableBrownOut(); // ignore brownout reset so a motor-start voltage dip does not reboot the board
	print("CHERGLOW (琛光) CF-Drone Flight Controller\n"); // brand line required by LICENSE article 2
	print("Initialization started\n");
	setupParameters(); // load parameters from flash (defaults if nothing is stored)
	setupLED();
	setupMotors(); // motor output (PWM/DShot)
	setLED(true); // LED on while initializing
#if WIFI_ENABLED
	setupWiFi(); // Wi-Fi for the web remote and MAVLink
#endif
#if WEB_RC_ENABLED
	setupWebRC();
#endif
	setupIMU();
	setupRC(); // RC receiver (SBUS / CRSF)
	setLED(false); // LED off when init is done
	print("Initialization complete\n");
	print("================================\n");
}

void loop() {
	readIMU(); // raw gyro/accel, calibration and axis rotation
	step(); // update t, dt, and the loop-rate counter
	readRC();
#if WEB_RC_ENABLED
	readWebRC();
	processConsoleCommandQueue(); // run web console commands on the main loop, not inside the HTTP callback
#endif
	estimate(); // attitude estimate (complementary filter)
	updateBatteryVoltage();
	control(); // attitude PID, writes motor commands
	sendMotors(); // write motor outputs (PWM/DShot)
	handleInput(); // serial / web console
#if WIFI_ENABLED
	processMavlink();
#endif
	logData();
	syncParameters(); // delayed flash write so parameters are not erased on every change
	updateLED();
}
