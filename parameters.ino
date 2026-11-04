// Parameters stored in flash

#include <Preferences.h>
#include <nvs_flash.h> // nvs_flash_init()/nvs_flash_erase(): erase and rebuild NVS when Preferences.begin() fails
#include "util.h"

extern float channelZero[16];
extern float channelMax[16];
extern float rollChannel, pitchChannel, throttleChannel, yawChannel, modeChannel;
extern int wifiMode, udpLocalPort, udpRemotePort;
extern int motorPins[4];
extern int pwmFrequency, pwmResolution, pwmStop, pwmMin, pwmMax;
extern float motThrMin;
extern float motThrMax;
extern int mavlinkSysId;
extern Rate telemetrySlow, telemetryFast;
extern float rcLossTimeout, descendTime;
extern int flightModes[3];
extern Vector accBias, accScale;
extern Vector imuRotation;
extern LowPassFilter<Vector> gyroBiasFilter;
extern int rcRxPin;
extern int rcProtocol;   // RC protocol: 0=SBUS, 1=CRSF (ELRS)
extern int rcTxPin;      // telemetry TX pin, -1 = unused
extern int rcBaud;       // UART baud rate
extern float trimRoll, trimPitch;         // software trim, defined in control.ino
extern float levelGateThreshold;         // stick gate threshold, defined in estimate.ino
extern float levelBiasGain;              // Mahony I gain, defined in estimate.ino
extern Vector levelGyroBias;             // Mahony virtual gyro bias, defined in estimate.ino

Preferences storage;

struct Parameter {
	const char *name; // max length is 15 (Preferences key limit)
	bool integer;
	union { float *f; int *i; };
	float cache; // what's stored in flash
	void (*callback)(); // called after parameter change
	Parameter(const char *name, float *variable, void (*callback)() = nullptr) : name(name), integer(false), f(variable), cache(0), callback(callback) {};
	Parameter(const char *name, int *variable, void (*callback)() = nullptr) : name(name), integer(true), i(variable), cache(0), callback(callback) {};
	float getValue() const { return integer ? *i : *f; }
	void setValue(const float value) { if (integer) *i = (int)value; else *f = value; }
};

Parameter parameters[] = {
	// ===== Control (inner rate PID) =====
	// The rate loop commands body rotation speed. It is the innermost loop.
	// P: proportional gain. Higher is faster; too high oscillates.
	// I: integral gain. Removes steady error (motor mismatch). Too high winds up.
	// D: derivative gain. Damps overshoot. Too high amplifies noise.
	// WU (windup): integral clamp so the I term cannot grow without limit.
	{"CTL_R_RATE_P",  &rollRatePID.p},      // roll rate P (rad/s -> torque)
	{"CTL_R_RATE_I",  &rollRatePID.i},      // roll rate I
	{"CTL_R_RATE_D",  &rollRatePID.d},      // roll rate D
	{"CTL_R_RATE_WU", &rollRatePID.windup}, // roll rate integral limit (+/- windup)
	{"CTL_P_RATE_P",  &pitchRatePID.p},     // pitch rate P
	{"CTL_P_RATE_I",  &pitchRatePID.i},     // pitch rate I
	{"CTL_P_RATE_D",  &pitchRatePID.d},     // pitch rate D
	{"CTL_P_RATE_WU", &pitchRatePID.windup},// pitch rate integral limit
	{"CTL_Y_RATE_P",  &yawRatePID.p},       // yaw rate P (yaw inertia is small, so this is usually larger)
	{"CTL_Y_RATE_I",  &yawRatePID.i},       // yaw rate I
	{"CTL_Y_RATE_D",  &yawRatePID.d},       // yaw rate D

	// ===== Control (outer angle PID) =====
	// The angle loop turns an attitude error into a rate command for the inner loop.
	// The I term can cancel a fixed hover offset from mechanical asymmetry. Set WU or it will not stay bounded.
	{"CTL_R_P",  &rollPID.p},      // roll angle P (rad error -> rad/s rate command)
	{"CTL_R_I",  &rollPID.i},      // roll angle I, fixed-offset compensation (compiled default is 0)
	{"CTL_R_D",  &rollPID.d},      // roll angle D (usually 0)
	{"CTL_R_WU", &rollPID.windup}, // roll angle integral limit (rad/s). Required when I is not 0
	{"CTL_P_P",  &pitchPID.p},     // pitch angle P
	{"CTL_P_I",  &pitchPID.i},     // pitch angle I (compiled default is 0)
	{"CTL_P_D",  &pitchPID.d},     // pitch angle D (usually 0)
	{"CTL_P_WU", &pitchPID.windup},// pitch angle integral limit (rad/s)
	{"CTL_Y_P",  &yawPID.p},       // yaw angle P

	// ===== Control (limits) =====
	{"CTL_P_RATE_MAX", &maxRate.y}, // max pitch rate (rad/s). Stick commands above this are clipped
	{"CTL_R_RATE_MAX", &maxRate.x}, // max roll rate (rad/s)
	{"CTL_Y_RATE_MAX", &maxRate.z}, // max yaw rate (rad/s)
	{"CTL_TILT_MAX",   &tiltMax},   // max tilt in STAB (rad). The angle loop is limited past this

	// ===== Control (software trim) =====
	// Cancels a hover drift from CG offset or uneven motor thrust, without a mechanical change.
	{"CTL_TRIM_ROLL",  &trimRoll},  // roll trim (rad). Positive rolls right. Step about 0.005 rad
	{"CTL_TRIM_PITCH", &trimPitch}, // pitch trim (rad). Positive pitches nose up. Step about 0.005 rad

	// ===== Control (flight modes) =====
	// The RC mode channel (RC_MODE) maps three switch positions to mode ids:
	// 0=RAW (direct), 1=ACRO, 2=STAB, 3=ALTHOLD (barometer altitude hold), 4=AUTO
	{"CTL_FLT_MODE_0", &flightModes[0]}, // mode for switch position 0
	{"CTL_FLT_MODE_1", &flightModes[1]}, // mode for switch position 1
	{"CTL_FLT_MODE_2", &flightModes[2]}, // mode for switch position 2

	// ===== IMU (mounting rotation) =====
	// If the IMU axes do not match the body frame, set the rotation here (rad).
	{"IMU_ROT_ROLL",  &imuRotation.x}, // IMU rotation about body X / roll (rad)
	{"IMU_ROT_PITCH", &imuRotation.y}, // IMU rotation about body Y / pitch (rad)
	{"IMU_ROT_YAW",   &imuRotation.z}, // IMU rotation about body Z / yaw (rad)

	// ===== IMU (gyro bias filter) =====
	// Static gyro bias is estimated at startup with an exponential moving average.
	// Smaller alpha filters harder (slower). Larger alpha converges faster and passes more vibration.
	{"IMU_GYRO_BIAS_A", &gyroBiasFilter.alpha}, // gyro-bias low-pass alpha (0..1)

	// ===== IMU (accelerometer calibration) =====
	// Run a six-face accelerometer calibration after install. Results are stored here.
	// acc_corrected = (acc_raw - bias) * scale
	{"IMU_ACC_BIAS_X", &accBias.x},  // accelerometer X bias (m/s^2)
	{"IMU_ACC_BIAS_Y", &accBias.y},  // accelerometer Y bias (m/s^2)
	{"IMU_ACC_BIAS_Z", &accBias.z},  // accelerometer Z bias (m/s^2)
	{"IMU_ACC_SCALE_X", &accScale.x}, // accelerometer X scale (dimensionless, ideal 1.0)
	{"IMU_ACC_SCALE_Y", &accScale.y}, // accelerometer Y scale
	{"IMU_ACC_SCALE_Z", &accScale.z}, // accelerometer Z scale

	// ===== Attitude estimate =====
	// Complementary filter: gyro integration for fast motion, accelerometer for slow gravity correction.
	{"EST_ACC_WEIGHT",   &accWeight},          // accelerometer attitude weight (0..1). Higher corrects more and is more sensitive to vibration
	{"EST_RATES_LPF_A",  &ratesFilter.alpha},  // rate low-pass alpha (0..1). Smaller lowers the cutoff (~40 Hz at 0.2)
	{"EST_LVL_GATE_THR", &levelGateThreshold}, // stick gate (0..1). Past this deflection applyLevel fades to 0
	{"EST_LVL_BIAS_GAIN",&levelBiasGain},      // Mahony I gain into the virtual gyro bias. Larger converges faster (~30 s at 0.00002)

	// ===== Motors (pins) =====
	// A change calls setupMotors() immediately and rebinds the LEDC channels. No reboot.
	{"MOT_PIN_FL", &motorPins[MOTOR_FRONT_LEFT],  setupMotors}, // front-left motor GPIO
	{"MOT_PIN_FR", &motorPins[MOTOR_FRONT_RIGHT], setupMotors}, // front-right motor GPIO
	{"MOT_PIN_RL", &motorPins[MOTOR_REAR_LEFT],   setupMotors}, // rear-left motor GPIO
	{"MOT_PIN_RR", &motorPins[MOTOR_REAR_RIGHT],  setupMotors}, // rear-right motor GPIO

	// ===== Motors (PWM) =====
	// MOSFET direct drive: pwmMax=-1, pwmStop/pwmMin=0, pure duty cycle.
	// ESC: pwmMax is the max pulse (us), pwmMin the min pulse, pwmFrequency down to 400 Hz.
	{"MOT_PWM_FREQ", &pwmFrequency, setupMotors}, // PWM frequency (Hz). 25000 for MOSFET, 400 for ESC
	{"MOT_PWM_RES",  &pwmResolution, setupMotors},// PWM resolution (bits). LEDC top is 2^n-1. 10 is typical
	{"MOT_PWM_STOP", &pwmStop},                   // PWM at stop. 0 in duty mode, idle pulse (us) for an ESC
	{"MOT_PWM_MIN",  &pwmMin},                    // minimum PWM. 0 in duty mode, min pulse (us) for an ESC
	{"MOT_PWM_MAX",  &pwmMax},                    // maximum PWM. -1 = duty mode (MOSFET), else ESC max pulse (us)

	// ===== Motors (thrust map) =====
	// Linear map from normalized throttle (0..1) onto motor output, leaving headroom for attitude correction.
	{"MOT_THR_MIN", &motThrMin}, // thrust at the bottom of the stick (0..1). Keep at least 0.05 to avoid stall
	{"MOT_THR_MAX", &motThrMax}, // thrust at full stick (0..1). Keep at most 0.9 so attitude correction has room

	// ===== RC (channel calibration) =====
	// Raw RC is scaled to a normalized value (-1..1 or 0..1).
	// channelZero[n]: stick center raw value. channelMax[n]: full-travel raw value.
	// normalized = (raw - zero) / max
	{"RC_ZERO_0", &channelZero[0]}, // channel 0 center
	{"RC_ZERO_1", &channelZero[1]}, // channel 1 center
	{"RC_ZERO_2", &channelZero[2]}, // channel 2 center
	{"RC_ZERO_3", &channelZero[3]}, // channel 3 center
	{"RC_ZERO_4", &channelZero[4]}, // channel 4 center
	{"RC_ZERO_5", &channelZero[5]}, // channel 5 center
	{"RC_ZERO_6", &channelZero[6]}, // channel 6 center
	{"RC_ZERO_7", &channelZero[7]}, // channel 7 center
	{"RC_MAX_0",  &channelMax[0]},  // channel 0 travel
	{"RC_MAX_1",  &channelMax[1]},  // channel 1 travel
	{"RC_MAX_2",  &channelMax[2]},  // channel 2 travel
	{"RC_MAX_3",  &channelMax[3]},  // channel 3 travel
	{"RC_MAX_4",  &channelMax[4]},  // channel 4 travel
	{"RC_MAX_5",  &channelMax[5]},  // channel 5 travel
	{"RC_MAX_6",  &channelMax[6]},  // channel 6 travel
	{"RC_MAX_7",  &channelMax[7]},  // channel 7 travel

	// ===== RC (function mapping) =====
	// Physical channel index for each axis (0-based integer).
	{"RC_ROLL",     &rollChannel},     // roll channel index
	{"RC_PITCH",    &pitchChannel},    // pitch channel index
	{"RC_THROTTLE", &throttleChannel}, // throttle channel index
	{"RC_YAW",      &yawChannel},      // yaw channel index
	{"RC_MODE",     &modeChannel},     // flight-mode switch channel (3 positions)

	// ===== RC (UART) =====
	// All four call setupRC, so the UART is reopened immediately. No reboot.
	{"RC_RX_PIN",   &rcRxPin,    setupRC}, // receiver data GPIO (UART RX)
	{"RC_PROTOCOL", &rcProtocol, setupRC}, // 0=SBUS (inverted), 1=CRSF/ELRS (idle-high, 420000 baud)
	{"RC_TX_PIN",   &rcTxPin,    setupRC}, // telemetry GPIO (UART TX). -1 = no telemetry
	{"RC_BAUD",     &rcBaud,     setupRC}, // RC UART baud. SBUS=100000, CRSF=420000

	// ===== WiFi =====
	// wifi.ino: 0=off, 1=AP (own hotspot), 2=STA (join an existing network)
	{"WIFI_MODE",     &wifiMode},      // Wi-Fi mode: 0=off, 1=AP, 2=STA
	{"WIFI_LOC_PORT", &udpLocalPort},  // local UDP port (ground station sends here)
	{"WIFI_REM_PORT", &udpRemotePort}, // remote UDP port (flight controller sends here)

	// ===== MAVLink telemetry =====
	{"MAV_SYS_ID",    &mavlinkSysId},       // MAVLink system id (1..254). Must be unique per vehicle
	{"MAV_RATE_SLOW", &telemetrySlow.rate}, // slow telemetry rate (Hz): heartbeat, battery, and similar
	{"MAV_RATE_FAST", &telemetryFast.rate}, // fast telemetry rate (Hz): attitude, rates, and similar

	// ===== Failsafe =====
	{"SF_RC_LOSS_TIME",  &rcLossTimeout}, // RC loss timeout (s). After this, auto-descent starts
	{"SF_DESCEND_TIME",  &descendTime},   // seconds to ramp thrust to zero. Smaller lands faster
};

void setupParameters() {
	print("Setup parameters\n");
	if (!storage.begin("flix", false)) {
		// Preferences.begin() returns false when nvs_flash_init() fails (partition change, NVS version mismatch, or a corrupt store after power loss).
		// Erase and retry, which is the ESP-IDF recovery path. Otherwise every parameter silently fails to persist.
		print("[NVS] Preferences.begin failed, erasing the NVS partition and retrying...\n");
		nvs_flash_erase();
		esp_err_t err = nvs_flash_init();
		print("[NVS] nvs_flash_init retry result: err=%d\n", err);
		if (!storage.begin("flix", false)) {
			print("[NVS] Second init still failed. Parameters will not persist. Check the partition table and the hardware.\n");
		} else {
			print("[NVS] Erase and rebuild succeeded. Parameters restart from code defaults (old data is gone; run ca/cr again).\n");
		}
	}
	// Read parameters from storage
	for (auto &parameter : parameters) {
		if (!storage.isKey(parameter.name)) {
			storage.putFloat(parameter.name, parameter.getValue()); // store default value
		}
		float stored = storage.getFloat(parameter.name, parameter.getValue());
		// Integer parameter read back as NaN (damaged flash): restore the code default and rewrite it
		if (parameter.integer && !isfinite(stored)) {
			stored = (float)parameter.getValue();
			storage.putFloat(parameter.name, stored);
		}
		parameter.setValue(stored);
		parameter.cache = parameter.getValue();
	}
}

int parametersCount() {
	return sizeof(parameters) / sizeof(parameters[0]);
}

const char *getParameterName(int index) {
	if (index < 0 || index >= parametersCount()) return "";
	return parameters[index].name;
}

float getParameter(int index) {
	if (index < 0 || index >= parametersCount()) return NAN;
	return parameters[index].getValue();
}

float getParameter(const char *name) {
	for (auto &parameter : parameters) {
		if (strcasecmp(parameter.name, name) == 0) {
			return parameter.getValue();
		}
	}
	return NAN;
}

bool setParameter(const char *name, const float value) {
	for (auto &parameter : parameters) {
		if (strcasecmp(parameter.name, name) == 0) {
			if (parameter.integer && !isfinite(value)) return false; // can't set integer to NaN or Inf
			parameter.setValue(value);
			if (parameter.callback) parameter.callback();
			return true;
		}
	}
	return false;
}

void syncParameters() {
	static Rate rate(1);
	if (!rate) return; // sync once per second
	if (motorsActive()) return; // don't use flash while flying, it may cause a delay

	for (auto &parameter : parameters) {
		if (parameter.getValue() == parameter.cache) continue;
		if (isnan(parameter.getValue()) && isnan(parameter.cache)) continue; // handle NAN != NAN
		size_t written = storage.putFloat(parameter.name, parameter.getValue());
		if (written != sizeof(float)) continue; // write failed: keep the old cache so the next 1 Hz pass retries
		parameter.cache = parameter.getValue();
	}
}

void printParameters() {
	for (auto &parameter : parameters) {
		print("%s = %g\n", parameter.name, parameter.getValue());
	}
}

void resetParameters() {
	storage.clear();
	ESP.restart();
}
