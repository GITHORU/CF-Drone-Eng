// Implementation of command line interface 

#include "pid.h"
#include "vector.h"
#include "util.h"
#include "lpf.h"

extern LowPassFilter<Vector> gyroBiasFilter;

#if WEB_RC_ENABLED
extern bool webConsoleEnabled;
extern void webLog(const char* msg);
#endif

extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;
extern const int RAW, ACRO, STAB, AUTO;
extern float t, dt, loopRate;
extern float controlTime;
extern uint16_t channels[16];
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;
extern float motors[4];
extern int mode;
extern bool armed;

const char* motd =
"CLI. Type a command and press Enter:\n"
"help - this menu\n"
"p - print every parameter\n"
"p <name> - print one parameter\n"
"p <name> <value> - set a parameter\n"
"p MOT_PIN_FL 14 - example: front-left motor pin 14\n"
"preset - erase stored parameters and reboot\n"
"mfr, mfl, mrr, mrl - spin one motor (bypasses the mixer). REMOVE THE PROPS.\n"
"ca - calibrate the accelerometer\n"
"ps - print pitch/roll/yaw\n"
"cr - calibrate the RC radio\n"
"rc - print RC channels\n"
"wifi - print Wi-Fi info\n"
"ap <ssid> <password> - set AP SSID and password\n"
"sta <ssid> <password> - set STA SSID and password\n"
"raw/stab/acro/auto - set the flight mode\n"
"arm - arm\n"
"disarm - disarm\n"
"psq - print the attitude quaternion\n"
"imu - print IMU data\n"
"time - print time and loop rate\n"
"mot - print motor outputs\n"
"sys - print system info\n"
"log [dump] - print the log\n"
"reboot - reboot\n"
"reset - reset the attitude estimate\n";

void print(const char* format, ...) {
	// A fixed buffer plus vsnprintf silently truncates long text (the boot menu, for example) and can drop the newline,
	// so the next print sticks to the same line. Format into a small stack buffer first. If that is too small,
	// allocate the exact size on the heap and format again.
	char stackBuf[512];
	va_list args, argsCopy;
	va_start(args, format);
	va_copy(argsCopy, args);
	int needed = vsnprintf(stackBuf, sizeof(stackBuf), format, argsCopy);
	va_end(argsCopy);
	if (needed < 0) needed = 0; // encoding error: avoid a negative size in the allocation below

	char *buf = stackBuf;
	bool heapAllocated = false;
	if (needed >= (int)sizeof(stackBuf)) {
		buf = (char*)malloc(needed + 1);
		if (buf) {
			vsnprintf(buf, needed + 1, format, args);
			heapAllocated = true;
		} else {
			buf = stackBuf; // out of memory: keep the truncated stack buffer
		}
	}
	va_end(args);

	Serial.print(buf);
#if WIFI_ENABLED
	mavlinkPrint(buf);
#endif
#if WEB_RC_ENABLED
	if (webConsoleEnabled) webLog(buf);
#endif
	if (heapAllocated) free(buf);
}

void pause(float duration) {
	float start = t;
	while (t - start < duration) {
		readIMU(); // keep the IMU and attitude updating during long commands (ca/cr), or the printed values freeze
		step();
		estimate();
		handleInput();
#if WIFI_ENABLED
		processMavlink();
#endif
#if WEB_RC_ENABLED
		readWebRC(); // keep the HTTP server answering during long commands (ca/cr)
#endif
		delay(50);
	}
}

void doCommand(String str, bool echo = false) {
	// parse command
	String command, arg0, arg1;
	splitString(str, command, arg0, arg1);
	if (command.isEmpty()) return;

	// echo command
	if (echo) {
		print("> %s\n", str.c_str());
	}

	command.toLowerCase();

	// execute command
	if (command == "help" || command == "motd") {
		print("%s\n", motd);
	} else if (command == "p" && arg0 == "") {
		printParameters();
	} else if (command == "p" && arg0 != "" && arg1 == "") {
		print("%s = %g\n", arg0.c_str(), getParameter(arg0.c_str()));
	} else if (command == "p") {
		bool success = setParameter(arg0.c_str(), arg1.toFloat());
		if (success) {
			print("%s = %g\n", arg0.c_str(), getParameter(arg0.c_str()));
		} else {
			print("Parameter not found: %s\n", arg0.c_str());
		}
	} else if (command == "preset") {
		resetParameters();
	} else if (command == "time") {
		print("Time: %f\n", t);
		print("Loop rate: %.0f\n", loopRate);
		print("dt: %f\n", dt);
	} else if (command == "ps") {
		Vector a = attitude.toEuler();
		print("roll: %f pitch: %f yaw: %f\n", degrees(a.x), degrees(a.y), degrees(a.z));
	} else if (command == "psq") {
		print("qw: %f qx: %f qy: %f qz: %f\n", attitude.w, attitude.x, attitude.y, attitude.z);
	} else if (command == "imu") {
		printIMUInfo();
		printIMUCalibration();
		print("landed: %d\n", landed);
	} else if (command == "arm") {
		extern bool imuOK;
		if (!imuOK) { print("IMU fault, arming blocked\n"); }
		else armed = true;
	} else if (command == "disarm") {
		armed = false;
	} else if (command == "raw") {
		mode = RAW;
	} else if (command == "stab") {
		mode = STAB;
	} else if (command == "acro") {
		mode = ACRO;
	} else if (command == "auto") {
		mode = AUTO;
	} else if (command == "rc") {
		print("channels: ");
		for (int i = 0; i < 16; i++) {
			print("%u ", channels[i]);
		}
		print("\nroll: %g pitch: %g yaw: %g throttle: %g mode: %g\n",
			controlRoll, controlPitch, controlYaw, controlThrottle, controlMode);
		print("time: %.1f\n", controlTime);
		print("mode: %s\n", getModeName());
		print("armed: %d\n", armed);
	} else if (command == "wifi") {
#if WIFI_ENABLED
		printWiFiInfo();
#endif
	} else if (command == "ap") {
#if WIFI_ENABLED
		configWiFi(true, arg0.c_str(), arg1.c_str());
#endif
	} else if (command == "sta") {
#if WIFI_ENABLED
		configWiFi(false, arg0.c_str(), arg1.c_str());
#endif
	} else if (command == "mot") {
		print("front-right %g front-left %g rear-right %g rear-left %g\n",
			motors[MOTOR_FRONT_RIGHT], motors[MOTOR_FRONT_LEFT], motors[MOTOR_REAR_RIGHT], motors[MOTOR_REAR_LEFT]);
	} else if (command == "log") {
		printLogHeader();
		if (arg0 == "dump") printLogData();
	} else if (command == "cr") {
		calibrateRC();
	} else if (command == "ca") {
		calibrateAccel();
	} else if (command == "mfr") {
		testMotor(MOTOR_FRONT_RIGHT);
	} else if (command == "mfl") {
		testMotor(MOTOR_FRONT_LEFT);
	} else if (command == "mrr") {
		testMotor(MOTOR_REAR_RIGHT);
	} else if (command == "mrl") {
		testMotor(MOTOR_REAR_LEFT);
	} else if (command == "sys") {
#ifdef ESP32
		print("Chip: %s\n", ESP.getChipModel());
		print("Temperature: %.1f °C\n", temperatureRead());
		print("Free heap: %d\n", ESP.getFreeHeap());
		// Print tasks table
		print("Num  Task                Stack  Prio  Core  CPU%%\n");
		int taskCount = uxTaskGetNumberOfTasks();
		TaskStatus_t *systemState = new TaskStatus_t[taskCount];
		uint32_t totalRunTime;
		uxTaskGetSystemState(systemState, taskCount, &totalRunTime);
		for (int i = 0; i < taskCount; i++) {
			String core = systemState[i].xCoreID == tskNO_AFFINITY ? "*" : String(systemState[i].xCoreID);
			int cpuPercentage = systemState[i].ulRunTimeCounter / (totalRunTime / 100);
			print("%-5d%-20s%-7d%-6d%-6s%d\n",systemState[i].xTaskNumber, systemState[i].pcTaskName,
				systemState[i].usStackHighWaterMark, systemState[i].uxCurrentPriority, core.c_str(), cpuPercentage);
		}
		delete[] systemState;
#endif
	} else if (command == "reset") {
		attitude = Quaternion();
		gyroBiasFilter.reset();
	} else if (command == "reboot") {
		ESP.restart();
	} else {
		print("Invalid command: %s\n", command.c_str());
	}
}

void handleInput() {
	static bool showMotd = true;
	static String input;

	if (showMotd) {
		print("%s\n", motd);
		showMotd = false;
	}

	while (Serial.available()) {
		char c = Serial.read();
		if (c == '\n') {
			doCommand(input);
			input.clear();
		} else {
			input += c;
		}
	}
}
