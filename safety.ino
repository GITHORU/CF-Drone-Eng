// Fail-safe functions

bool isInverted = false;  // body is inverted (body-Z cosine < INVERTED_COS_THRESHOLD)

float rcLossTimeout = 1;        // RC loss timeout (s), parameter SF_RC_LOSS_TIME
float descendTime = 10;         // time to ramp thrust to zero (s), parameter SF_DESCEND_TIME
#define WEB_RC_LOSS_TIMEOUT_MS 8000UL  // web remote loss timeout (ms); must exceed the 2000 ms heartbeat

// Inverted-vehicle cutoff
#define INVERTED_COS_THRESHOLD -0.7f   // cos(134 deg). Tilt past 134 deg counts as inverted (margin for gyro drift)
#define INVERTED_TIMEOUT        1.5f   // stay inverted this long (s) and motors stop

extern float controlTime;
extern float controlRoll, controlPitch, controlThrottle, controlYaw;

#if WEB_RC_ENABLED
extern bool webRCEnabled;
extern bool useWebRC;
extern unsigned long webRCLastUpdate;
bool isUsingWebRC();
#endif

extern bool armed;
extern int mode;
extern float dt;
extern float thrustTarget;
extern float t;
extern float batteryVoltage;  // battery.ino
extern Quaternion attitudeTarget;
extern Quaternion attitude;
extern Vector ratesExtra;      // control.ino
extern PID rollRatePID, pitchRatePID, yawRatePID;  // control.ino
extern PID rollPID, pitchPID, yawPID;              // control.ino
extern float motThrMin;        // control.ino

void failsafe() {
	rcLossFailsafe();
#if WEB_RC_ENABLED
	webRCLossFailsafe();
#endif
	autoFailsafe();
	invertedFailsafe();
	batteryFailsafe();
}

// RC loss failsafe
void rcLossFailsafe() {
	if (controlTime == 0) return; // no RC at all
	if (!armed) return;
#if WEB_RC_ENABLED
	if (isUsingWebRC()) return; // web RC has its own timeout (webRCLossFailsafe)
#endif
	if (t - controlTime > rcLossTimeout) {
		descend();
	}
}

// Smooth descend on RC lost
void descend() {
	if (mode != AUTO) {
		// First entry: hold current yaw, force level, clear rate feedforward, reset PID integrators
		float currentYaw = attitude.getYaw();
		attitudeTarget = Quaternion::fromEuler(Vector(0, 0, currentYaw));
		ratesExtra = Vector(0, 0, 0);
		rollRatePID.reset();
		pitchRatePID.reset();
		yawRatePID.reset();
		rollPID.reset();
		pitchPID.reset();
		yawPID.reset();
		// Cap thrust at the hover value so a high-throttle trigger does not stretch the descent
		if (thrustTarget > ALTHOLD_HOVER_THRUST) thrustTarget = ALTHOLD_HOVER_THRUST;
		mode = AUTO;
	}
	// Track actual yaw every frame so the yaw PID does not wind up during the descent
	float currentYaw = attitude.getYaw();
	attitudeTarget = Quaternion::fromEuler(Vector(0, 0, currentYaw));

	thrustTarget -= dt / descendTime;
	if (thrustTarget < 0) {
		thrustTarget = 0;
		armed = false;
	}
}

// Allow pilot to interrupt automatic flight
void autoFailsafe() {
	static float roll, pitch, yaw, throttle;
	
	// control* already merges SBUS, MAVLink, and web RC, so check it directly
	if (roll != controlRoll || pitch != controlPitch || yaw != controlYaw || abs(throttle - controlThrottle) > 0.05) {
		if (mode == AUTO) mode = STAB;
	}
	
	roll = controlRoll;
	pitch = controlPitch;
	yaw = controlYaw;
	throttle = controlThrottle;
}

#if WEB_RC_ENABLED
// Web remote link-loss protection
void webRCLossFailsafe() {
	if (!webRCEnabled || !useWebRC) return;
	if (!armed) return;

	// Compare milliseconds directly. Dividing to seconds can hide up to 1 s.
	if (millis() - webRCLastUpdate > WEB_RC_LOSS_TIMEOUT_MS) {
		print("Web RC link lost, starting descent\n");
		descend();
		webRCEnabled = false;
		useWebRC = false;
	}
}
#endif

// Inverted cutoff: if body Z stays past INVERTED_COS_THRESHOLD for INVERTED_TIMEOUT, motors stop
void invertedFailsafe() {
	if (!armed) {
		isInverted = false;
		return;
	}

	// Body Z in the world frame: about +1 upright, about -1 inverted
	Vector worldUp = Quaternion::rotateVector(Vector(0, 0, 1), attitude);

	static float invertedStartTime = 0;
	if (worldUp.z < INVERTED_COS_THRESHOLD) {
		isInverted = true;
		if (invertedStartTime == 0) invertedStartTime = t;
		if (t - invertedStartTime > INVERTED_TIMEOUT) {
			armed = false;
			thrustTarget = 0.0f;
			invertedStartTime = 0;
			print("Inverted cutoff: motors stopped\n");
		}
	} else {
		isInverted = false;
		invertedStartTime = 0;
	}
}

// Battery protection. Thresholds are the VBAT_* macros in battery.ino.
// L1 (VBAT_WARN_THRESHOLD): block arm while disarmed (control.ino), auto-disarm at idle
// L2 (VBAT_LOW_THRESHOLD): in flight, LED fast-blink only
// L3 (VBAT_CRITICAL_THRESHOLD): in flight, auto-land via descend(); rate is SF_DESCEND_TIME
void batteryFailsafe() {
	static bool l3Latched = false;
	static float lowSince = 0.0f;
	static float l3LastNotify = 0.0f;

	if (batteryVoltage < VBAT_ABSENT_THRESHOLD) return; // no battery connected, ignore
	if (!armed) {
		l3Latched = false;
		lowSince = 0.0f;
		l3LastNotify = 0.0f;
		return;
	}

	// Once L3 trips, keep descending until disarm so the threshold does not chatter
	if (l3Latched) {
		if (t - l3LastNotify >= 2.0f) {
			print("Critical battery (%.2fV), still landing, thrust %.0f%%\n",
			      batteryVoltage, thrustTarget * 100.0f);
#if WEB_RC_ENABLED
			char warnBuf[64];
			snprintf(warnBuf, sizeof(warnBuf),
			         "Critical battery (%.2fV), landing, thrust %.0f%%",
			         batteryVoltage, thrustTarget * 100.0f);
			setWebRCWarn(warnBuf);
#endif
			l3LastNotify = t;
		}
		descend();
		return;
	}

	bool flying = thrustTarget >= BATTERY_FLYING_THRUST_MIN;

	bool actionCondition = false;
	bool criticalAction = false;

	// In flight only L3 takes action. L2 stays an LED warning.
	if (flying) {
		if (batteryVoltage < VBAT_CRITICAL_THRESHOLD) {
			actionCondition = true;
			criticalAction = true;
		}
	} else {
		// Armed and idle: L1 auto-disarms
		if (batteryVoltage < VBAT_WARN_THRESHOLD) {
			actionCondition = true;
		}
	}

	if (!actionCondition) {
		lowSince = 0.0f;
		return;
	}

	if (lowSince == 0.0f) lowSince = t;
	if (t - lowSince < BATTERY_ACTION_DEBOUNCE_TIME) return;

	lowSince = 0.0f;
	if (criticalAction) {
		l3Latched = true;
		l3LastNotify = 0.0f;
		print("Critical battery (%.2fV), auto-landing\n", batteryVoltage);
#if WEB_RC_ENABLED
		char warnBuf[64];
		snprintf(warnBuf, sizeof(warnBuf),
		         "Critical battery (%.2fV), auto-landing",
		         batteryVoltage);
		setWebRCWarn(warnBuf);
#endif
		descend();
		return;
	}

	armed = false;
	thrustTarget = 0.0f;
	print("Low battery (%.2fV), auto-disarmed\n", batteryVoltage);
#if WEB_RC_ENABLED
	char warnBuf[64];
	snprintf(warnBuf, sizeof(warnBuf), "Low battery (%.2fV), auto-disarmed", batteryVoltage);
	setWebRCWarn(warnBuf);
#endif
}
