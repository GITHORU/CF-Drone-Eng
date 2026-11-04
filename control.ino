// Flight control

#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "lpf.h"
#include "util.h"

#if WEB_RC_ENABLED
#include "control.h"
#endif

// Tuned for a 118 mm micro quad
// ============== Rate loop (inner) ==============
#define PITCHRATE_P 0.06 // higher P for a faster response
#define PITCHRATE_I 0.1 // moderate I to cover motor mismatch
#define PITCHRATE_D 0.001 // small D to damp oscillation
#define PITCHRATE_I_LIM 0.3 // integral clamp
#define ROLLRATE_P PITCHRATE_P // roll uses the same gains as pitch
#define ROLLRATE_I PITCHRATE_I 
#define ROLLRATE_D PITCHRATE_D
#define ROLLRATE_I_LIM PITCHRATE_I_LIM
#define YAWRATE_P 0.3 // yaw needs a higher P (less inertia)
#define YAWRATE_I 0.01 // moderate I
#define YAWRATE_D 0.01 // small D
#define YAWRATE_I_LIM 0.3
// ============== Angle loop (outer) ==============
#define ROLL_P 6 // higher P for a fast response
#define ROLL_I 0 // outer-loop I
#define ROLL_D 0 // the angle loop usually needs no D
#define ROLL_I_LIM radians(5.0f) // outer roll integral limit (rad/s), about 5 deg/s, so it cannot run away at low throttle or on a mode change
#define PITCH_P ROLL_P // roll and pitch share gains
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D
#define PITCH_I_LIM ROLL_I_LIM
#define YAW_P 3 // yaw responds a bit more slowly

// ============== Limits ==============
#define PITCHRATE_MAX radians(360) // high rate limit, 360 deg/s
#define ROLLRATE_MAX radians(360)
#define YAWRATE_MAX radians(300) // yaw rate limit is a bit lower
#define TILT_MAX radians(30) // max tilt 30 deg
#define ALTHOLD_HOVER_THRUST 0.5f   // hover thrust placeholder until a barometer exists
#define ARM_THROTTLE_LIMIT   0.05f  // max throttle (0..1) that still allows arming, 5%
#define RATES_D_LPF_ALPHA 0.2 // cutoff frequency ~ 40 Hz

float motThrMin = 0.10f;  // thrust at the bottom of the stick, parameter MOT_THR_MIN
float motThrMax = 0.9f;   // thrust at full stick, parameter MOT_THR_MAX; 10% left for attitude correction

const int RAW = 0, ACRO = 1, STAB = 2, ALTHOLD = 3, AUTO = 4; // flight modes
int mode = STAB;
bool armed = false;
int flightModes[] = {STAB, STAB, STAB}; // flight mode for each of the 3 RC switch positions, parameters CTL_FLT_MODE_0/1/2

#if WEB_RC_ENABLED
extern uint16_t webRCButtons;
#endif

PID rollRatePID(ROLLRATE_P, ROLLRATE_I, ROLLRATE_D, ROLLRATE_I_LIM, RATES_D_LPF_ALPHA);
PID pitchRatePID(PITCHRATE_P, PITCHRATE_I, PITCHRATE_D, PITCHRATE_I_LIM, RATES_D_LPF_ALPHA);
PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);
PID rollPID(ROLL_P, ROLL_I, ROLL_D, ROLL_I_LIM);
PID pitchPID(PITCH_P, PITCH_I, PITCH_D, PITCH_I_LIM);
PID yawPID(YAW_P, 0, 0);
Vector maxRate(ROLLRATE_MAX, PITCHRATE_MAX, YAWRATE_MAX);
float tiltMax = TILT_MAX;

Quaternion attitudeTarget;
Vector ratesTarget;
Vector ratesExtra; // feedforward rates
Vector torqueTarget;
float thrustTarget;

// ============== Software trim ==============
// Cancels a fixed drift from mechanical asymmetry (CG offset, motor or prop mismatch, IMU tilt).
// Unit: radians.
//
// Tune from the serial or web console. No reflash:
//   p <name> <value>
//   Step about 0.005 rad (~0.3 deg) until it holds with the sticks centered.
//   Values are stored in flash and survive power-off.
//
// Drift seen with sticks centered:
//   drifts left  -> p CTL_TRIM_ROLL  +0.01  (positive commands a right tilt)
//   drifts right -> p CTL_TRIM_ROLL  -0.01  (negative commands a left tilt)
//   drifts forward -> p CTL_TRIM_PITCH -0.01  (negative commands nose up)
//   drifts back    -> p CTL_TRIM_PITCH +0.01  (positive commands nose down)
//
// Battery level and payload shift the trim slightly. Retune when that happens.
float trimRoll  = 0.0f; // roll trim (rad), parameter CTL_TRIM_ROLL
float trimPitch = 0.0f; // pitch trim (rad), parameter CTL_TRIM_PITCH

extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;
extern float motors[4];
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;
extern float batteryVoltage;  // battery.ino

void control() {
	interpretControls();
#if WEB_RC_ENABLED
	interpretWebRC();
#endif
	failsafe();
	controlAttitude();
	controlRates();
	controlTorque();
}

void interpretControls() {
	if (controlMode < 0.25) mode = flightModes[0];
	else if (controlMode <= 0.75) mode = flightModes[1];
	else if (controlMode > 0.75) mode = flightModes[2];

	if (mode == AUTO) return; // pilot is not effective in AUTO mode

#if WEB_RC_ENABLED
	if (!isUsingWebRC()) { // stick arm gesture only while the web remote is idle
#endif
	extern bool imuOK;
	static bool armWarnNotified = false;  // print the arming block once, not every loop
	if (controlThrottle < 0.05 && controlYaw > 0.95) { // arm gesture
		if (!imuOK) {
			// IMU fault: refuse to arm
			if (!armWarnNotified) {
				print("IMU fault, arming blocked\n");
#if WEB_RC_ENABLED
				setWebRCWarn("IMU fault, arming blocked");
#endif
				armWarnNotified = true;
			}
		} else if (batteryVoltage > VBAT_ABSENT_THRESHOLD && batteryVoltage < VBAT_WARN_THRESHOLD) {
			// L1 or below: refuse to arm, and say so once
			if (!armWarnNotified) {
				print("Low battery (%.2fV), arming blocked\n", batteryVoltage);
#if WEB_RC_ENABLED
				char warnBuf[64];
				snprintf(warnBuf, sizeof(warnBuf), "Low battery (%.2fV), arming blocked", batteryVoltage);
				setWebRCWarn(warnBuf);
#endif
				armWarnNotified = true;
			}
		} else {
			armed = true;
			armWarnNotified = false; // reset after a successful arm
		}
	}
	if (controlThrottle < 0.05 && controlYaw < -0.95) armed = false; // disarm gesture
#if WEB_RC_ENABLED
	}
#endif

	if (abs(controlYaw) < 0.1) controlYaw = 0; // yaw dead zone

	if (controlThrottle < 0.05f) {
		thrustTarget = 0.0f;   // bottom deadzone -> idle, PID does not run
	} else {
		thrustTarget = mapf(controlThrottle, 0.05f, 1.0f, motThrMin, motThrMax);
	}

	if (mode == STAB) {
		float yawTarget = attitudeTarget.getYaw();
		if (!armed || invalid(yawTarget) || controlYaw != 0) yawTarget = attitude.getYaw(); // reset yaw target
		// trimRoll/trimPitch are added to the stick command to cancel a fixed drift.
		// See the variable comments above, or: p CTL_TRIM_ROLL / p CTL_TRIM_PITCH
		attitudeTarget = Quaternion::fromEuler(Vector(controlRoll * tiltMax + trimRoll, controlPitch * tiltMax + trimPitch, yawTarget));
		ratesExtra = Vector(0, 0, -controlYaw * maxRate.z); // positive yaw stick means clockwise rotation in FLU
	}

	if (mode == ACRO) {
		attitudeTarget.invalidate(); // skip attitude control
		ratesTarget.x = controlRoll * maxRate.x;
		ratesTarget.y = controlPitch * maxRate.y;
		ratesTarget.z = -controlYaw * maxRate.z; // positive yaw stick means clockwise rotation in FLU
	}

	if (mode == RAW) { // direct torque control
		attitudeTarget.invalidate(); // skip attitude control
		ratesTarget.invalidate(); // skip rate control
		torqueTarget = Vector(controlRoll, controlPitch, -controlYaw) * 0.1;
	}
}

void controlAttitude() {
	if (!armed || attitudeTarget.invalid() || thrustTarget < motThrMin) return; // skip attitude control

	const Vector up(0, 0, 1);
	Vector upActual = Quaternion::rotateVector(up, attitude);
	Vector upTarget = Quaternion::rotateVector(up, attitudeTarget);

	Vector error = Vector::rotationVectorBetween(upTarget, upActual);

	ratesTarget.x = rollPID.update(error.x) + ratesExtra.x;
	ratesTarget.y = pitchPID.update(error.y) + ratesExtra.y;

	float yawError = wrapAngle(attitudeTarget.getYaw() - attitude.getYaw());
	ratesTarget.z = yawPID.update(yawError) + ratesExtra.z;
}


void controlRates() {
	if (!armed || ratesTarget.invalid() || thrustTarget < motThrMin) return; // skip rates control

	Vector error = ratesTarget - rates;

	// Calculate desired torque, where 0 - no torque, 1 - maximum possible torque
	torqueTarget.x = rollRatePID.update(error.x);
	torqueTarget.y = pitchRatePID.update(error.y);
	torqueTarget.z = yawRatePID.update(error.z);
}

void controlTorque() {
	if (!torqueTarget.valid()) return; // skip torque control

	if (!armed) {
		memset(motors, 0, sizeof(motors)); // stop motors if disarmed
		return;
	}

	if (thrustTarget < motThrMin) {
		for (int i = 0; i < 4; i++) motors[i] = motThrMin; // idle thrust
		return;
	}

	motors[MOTOR_FRONT_LEFT] = thrustTarget + torqueTarget.x - torqueTarget.y + torqueTarget.z;
	motors[MOTOR_FRONT_RIGHT] = thrustTarget - torqueTarget.x - torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_LEFT] = thrustTarget + torqueTarget.x + torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_RIGHT] = thrustTarget - torqueTarget.x + torqueTarget.y + torqueTarget.z;

	desaturate(motors[MOTOR_FRONT_LEFT], motors[MOTOR_FRONT_RIGHT], motors[MOTOR_REAR_LEFT], motors[MOTOR_REAR_RIGHT]);

	motors[0] = constrain(motors[0], 0, 1);
	motors[1] = constrain(motors[1], 0, 1);
	motors[2] = constrain(motors[2], 0, 1);
	motors[3] = constrain(motors[3], 0, 1);
}

void desaturate(float& a, float& b, float& c, float& d) {
	// avg is about thrustTarget (torque terms sum to zero). Keep avg, scale the torque deviation down.
	float avg = (a + b + c + d) * 0.25f;
	float maxVal = max(max(a, b), max(c, d));
	float minVal = min(min(a, b), min(c, d));

	float scale = 1.0f;
	if (maxVal > 1.0f && maxVal > avg) {
		scale = min(scale, (1.0f - avg) / (maxVal - avg));
	}
	if (minVal < 0.0f && avg > minVal) {
		scale = min(scale, avg / (avg - minVal));
	}

	if (scale < 1.0f) {
		a = avg + (a - avg) * scale;
		b = avg + (b - avg) * scale;
		c = avg + (c - avg) * scale;
		d = avg + (d - avg) * scale;
	}
}

const char* getModeName() {
	switch (mode) {
		case RAW:     return "RAW";
		case ACRO:    return "ACRO";
		case STAB:    return "STAB";
		case ALTHOLD: return "ALTHOLD";
		case AUTO:    return "AUTO";
		default:      return "UNKNOWN";
	}
}

#if WEB_RC_ENABLED
// Web remote buttons: arm, disarm, and mode only
void interpretWebRC() {
	if (!isUsingWebRC()) return;

	// Rising edge, so a held button does not retrigger every control cycle
	static uint16_t lastWebRCButtons = 0;
	uint16_t risingEdge = webRCButtons & ~lastWebRCButtons;
	lastWebRCButtons = webRCButtons;

	// Log arm state changes
	static bool lastArmedState = false;
	if (armed != lastArmedState) {
		print(armed ? "Web RC: armed\n" : "Web RC: disarmed\n");
		lastArmedState = armed;
	}

	// Button 0: arm (rising edge)
	if (risingEdge & 0x0001) {
		extern bool imuOK;
		extern char webRCWarnMsg[];
		if (!imuOK) {
			setWebRCWarn("IMU fault, arming blocked");
		} else if (batteryVoltage > VBAT_ABSENT_THRESHOLD && batteryVoltage < VBAT_WARN_THRESHOLD) {
			// Low battery: refuse to arm (same rule as the RC path in interpretControls())
			char warnBuf[64];
			snprintf(warnBuf, sizeof(warnBuf), "Low battery (%.2fV), arming blocked", batteryVoltage);
			setWebRCWarn(warnBuf);
		} else if (controlThrottle > ARM_THROTTLE_LIMIT) {
			setWebRCWarn("Throttle too high to arm");
		} else {
			armed = true;
			webRCWarnMsg[0] = '\0'; // armed, clear any previous warning
		}
	}

	// Button 1: disarm (rising edge)
	if (risingEdge & 0x0002) {
		armed = false;
	}

	// Button 2: emergency stop (rising edge)
	if (risingEdge & 0x0004) {
		armed = false;
		thrustTarget = 0.0f;
	}

	// Button 6: STAB mode (rising edge)
	if (risingEdge & 0x0040) {
		mode = STAB;
	}

	// Button 7: ACRO mode (rising edge)
	if (risingEdge & 0x0080) {
		mode = ACRO;
	}

	// Button 8: ALTHOLD (rising edge). Not implemented, so fall back to STAB and keep the web mode cycle moving.
	if (risingEdge & 0x0100) {
		mode = STAB;
		setWebRCWarn("Alt-hold unavailable, switched to STAB");
	}

	// Mode-change log
	static int lastMode = STAB;
	if (mode != lastMode) {
		print("Web RC: mode %s\n", getModeName());
		lastMode = mode;
	}
}
#endif
