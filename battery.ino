// Battery voltage monitoring

#include "board_config.h"

#define VBAT_ADC_PIN       BOARD_VBAT_ADC_PIN     // battery voltage sense pin
#define VBAT_ADC_SAMPLES   BOARD_VBAT_ADC_SAMPLES  // ADC samples: C3=8 (less blocking) / ESP32 and S3=16 (less noise)
#define VBAT_DIVIDER       (43.0f / 33.0f)  // divider: 10k top + 33k bottom, Vbat = Vadc * 43/33
#define VBAT_WARN_THRESHOLD  -999.0f  // L1 disabled for now: arming is not blocked by battery voltage
#define VBAT_LOW_THRESHOLD      3.3f  // L2: in-flight low-battery warning (V)
#define VBAT_CRITICAL_THRESHOLD -999.0f // L3 disabled for now: no auto-land on low voltage
#define VBAT_ABSENT_THRESHOLD   0.5f  // below this, treat as no battery and ignore alarms

#define BATTERY_FLYING_THRUST_MIN    0.15f  // thrust at or above this counts as flying; L1 does not apply
#define BATTERY_ACTION_DEBOUNCE_TIME  0.9f  // low-voltage must hold this long before action (s)

extern float t;
float batteryVoltage = 0.0f;  // latest voltage, refreshed by updateBatteryVoltage()

void updateBatteryVoltage() {
	static float lastCheck = 0;
	if (t - lastCheck < 0.5f) return;  // sample every 0.5 s
	lastCheck = t;
	batteryVoltage = readBatteryVoltage();
}

float readBatteryVoltage() {
	long sum = 0;
	for (int i = 0; i < VBAT_ADC_SAMPLES; i++) sum += analogReadMilliVolts(VBAT_ADC_PIN);
	return (sum / (float)VBAT_ADC_SAMPLES / 1000.0f) * VBAT_DIVIDER;
}
