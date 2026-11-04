// Attitude estimate from gyro and accelerometer

#include "quaternion.h"
#include "vector.h"
#include "lpf.h"
#include "util.h"

float accWeight = 0.003;

// ============== Level-correction P term ==============
float levelWeight = 0;  // level-correction P weight (off: cannot tell gyro temperature drift from a mechanical offset, and the term then fights the trim)
float levelMaxTilt = radians(30); // rad, level correction fades out at this tilt (matches TILT_MAX)
// Stick gate.
// When stick deflection (max of roll and pitch, 0..1) exceeds this threshold,
// applyLevel fades linearly to 0 so it does not couple into the PID integrator and drift the hover after the stick is released.
// Tuning:
//   - smaller: gate is more sensitive (a small stick input stops the correction) and hover damping is weaker
//   - larger: applyLevel keeps interfering longer while the stick is deflected
//   - useful range: 0.2 (sensitive) to 0.4 (loose). Default 0.2 (weight is 0 at 20% stick)
//   - live tune: p EST_LVL_GATE_THR <value>
float levelGateThreshold = 0.2f; // stick gate threshold, parameter EST_LVL_GATE_THR

// ============== Mahony-style level-correction I term ==============
// levelBiasGain: integrator gain from gravity error into a virtual gyro bias (I term).
// The bias is subtracted from the gyro in applyGyro(). It does not write the attitude quaternion,
// so the PID loop does not see the correction and does not oscillate against the inner-loop I term.
// Larger converges faster and can over-integrate. About 30 s to steady state.
// Parameter: EST_LVL_BIAS_GAIN
float levelBiasGain = 0;  // Mahony I gain (off: cannot tell gyro temperature drift from a mechanical offset)
Vector levelGyroBias(0, 0, 0); // virtual gyro bias estimated by applyLevel(), rad/s

extern float controlRoll, controlPitch; // pilot sticks, defined in CF-Drone.ino
LowPassFilter<Vector> ratesFilter(0.2); // cutoff frequency ~ 40 Hz

void estimate() {
	applyGyro();
	applyAcc();
	applyLevel();
}

void applyGyro() {
	// Subtract the Mahony I-term virtual bias, then filter and integrate.
	// Long-term level correction arrives as a bias, not as a direct attitude kick, so the PID does not see it.
	rates = ratesFilter.update(gyro - levelGyroBias);

	// apply rates to attitude
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(rates * dt));
}

void applyAcc() {
	// test should we apply accelerometer gravity correction
	float accNorm = acc.norm();
	landed = !motorsActive() && abs(accNorm - ONE_G) < ONE_G * 0.1f;

	if (!landed) return;

	// calculate accelerometer correction
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	Vector correction = Vector::rotationVectorBetween(acc, up) * accWeight;

	// apply correction
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(correction));
}

// Level correction (Mahony style: P term writes attitude, I term integrates a virtual gyro bias).
//   - P term (levelWeight): a small attitude correction for a fast response. Its magnitude is already halved to reduce kicks.
//   - I term (levelBiasGain): gravity error integrates into levelGyroBias and is removed from the gyro in applyGyro().
//     That path bypasses the PID error, which removes the coupling oscillation caused by writing attitude directly.
void applyLevel() {
	if (landed) {
		levelGyroBias = Vector(0, 0, 0); // clear the bias on the ground so the next takeoff learns again
		return;
	}

	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	float tilt = acos(constrain(up.z, -1.0f, 1.0f));

	// Skip when nearly level: as tilt approaches 0 the cross product is dominated by vibration and points at random.
	if (tilt < radians(0.1f)) return;

	// P weight: fades as tilt approaches levelMaxTilt
	float dynamicWeight = levelWeight * constrain(1.0f - tilt / levelMaxTilt, 0.0f, 1.0f);

	// ---- stick gate ----
	float stickDeflection = max(abs(controlRoll), abs(controlPitch));
	float stickGate = constrain(1.0f - stickDeflection / levelGateThreshold, 0.0f, 1.0f);
	dynamicWeight *= stickGate;
	// ------------------------------------

	// Gravity reference error: rotation vector between body Z and world Z
	Vector error = Vector::rotationVectorBetween(Vector(0, 0, 1), up);

	// P term: small direct attitude correction, kept for a fast response to drift
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(error * dynamicWeight));

	// I term: integrate into the virtual gyro bias (Mahony style).
	// While the stick is out (stickGate=0) integration pauses, so a deliberate maneuver does not train the bias.
	levelGyroBias += error * (levelBiasGain * stickGate);

	// Limit: equivalent to 3 deg/s max, so the bias cannot run away
	float biasNorm = levelGyroBias.norm();
	const float biasLimit = radians(3.0f);
	if (biasNorm > biasLimit) levelGyroBias = levelGyroBias * (biasLimit / biasNorm);
}
