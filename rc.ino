// RC receiver
// Protocols: CRSF/ELRS (default) and SBUS
// Switch with RC_PROTOCOL: 0=SBUS, 1=CRSF (ELRS)

#include "drivers/SBUS.h"
#include "util.h"
#include "board_config.h"

// ---- Protocol settings (board defaults, overridable by parameters) ----
int rcProtocol = BOARD_RC_PROTOCOL; // 0=SBUS, 1=CRSF (ELRS)
int rcBaud     = BOARD_RC_BAUD;     // UART baud rate
int rcTxPin    = BOARD_RC_TX_PIN;   // telemetry TX pin, -1 = unused

// ---- SBUS object (used when rcProtocol==0) ----
SBUS rc(BOARD_RC_SERIAL);

int rcRxPin = BOARD_RC_RX_PIN; // RC RX pin. -1 disables RC. Parameter RC_RX_PIN.
uint16_t channels[16]; // raw rc channels
float controlTime; // time of the last controls update
float channelZero[16]; // per-channel center / minimum calibration
float channelMax[16]; // per-channel full-travel calibration

// Channels mapping (using float to store in parameters):
float rollChannel = NAN, pitchChannel = NAN, throttleChannel = NAN, yawChannel = NAN, modeChannel = NAN;

// ============================================================
// CRSF parser (used only when rcProtocol==1)
// ============================================================
#define CRSF_MAX_FRAME_SIZE 64
static uint8_t crsfBuf[CRSF_MAX_FRAME_SIZE];
static int     crsfBufLen = 0;

// CRC8 table (poly 0xD5), covers TYPE + PAYLOAD
static uint8_t crsf_crc8(const uint8_t *buf, int len) {
	static const uint8_t crc8tab[256] = {
		0x00,0xD5,0x7F,0xAA,0xFE,0x2B,0x81,0x54, 0x29,0xFC,0x56,0x83,0xD7,0x02,0xA8,0x7D,
		0x52,0x87,0x2D,0xF8,0xAC,0x79,0xD3,0x06, 0x7B,0xAE,0x04,0xD1,0x85,0x50,0xFA,0x2F,
		0xA4,0x71,0xDB,0x0E,0x5A,0x8F,0x25,0xF0, 0x8D,0x58,0xF2,0x27,0x73,0xA6,0x0C,0xD9,
		0xF6,0x23,0x89,0x5C,0x08,0xDD,0x77,0xA2, 0xDF,0x0A,0xA0,0x75,0x21,0xF4,0x5E,0x8B,
		0x9D,0x48,0xE2,0x37,0x63,0xB6,0x1C,0xC9, 0xB4,0x61,0xCB,0x1E,0x4A,0x9F,0x35,0xE0,
		0xCF,0x1A,0xB0,0x65,0x31,0xE4,0x4E,0x9B, 0xE6,0x33,0x99,0x4C,0x18,0xCD,0x67,0xB2,
		0x39,0xEC,0x46,0x93,0xC7,0x12,0xB8,0x6D, 0x10,0xC5,0x6F,0xBA,0xEE,0x3B,0x91,0x44,
		0x6B,0xBE,0x14,0xC1,0x95,0x40,0xEA,0x3F, 0x42,0x97,0x3D,0xE8,0xBC,0x69,0xC3,0x16,
		0xEF,0x3A,0x90,0x45,0x11,0xC4,0x6E,0xBB, 0xC6,0x13,0xB9,0x6C,0x38,0xED,0x47,0x92,
		0xBD,0x68,0xC2,0x17,0x43,0x96,0x3C,0xE9, 0x94,0x41,0xEB,0x3E,0x6A,0xBF,0x15,0xC0,
		0x4B,0x9E,0x34,0xE1,0xB5,0x60,0xCA,0x1F, 0x62,0xB7,0x1D,0xC8,0x9C,0x49,0xE3,0x36,
		0x19,0xCC,0x66,0xB3,0xE7,0x32,0x98,0x4D, 0x30,0xE5,0x4F,0x9A,0xCE,0x1B,0xB1,0x64,
		0x72,0xA7,0x0D,0xD8,0x8C,0x59,0xF3,0x26, 0x5B,0x8E,0x24,0xF1,0xA5,0x70,0xDA,0x0F,
		0x20,0xF5,0x5F,0x8A,0xDE,0x0B,0xA1,0x74, 0x09,0xDC,0x76,0xA3,0xF7,0x22,0x88,0x5D,
		0xD6,0x03,0xA9,0x7C,0x28,0xFD,0x57,0x82, 0xFF,0x2A,0x80,0x55,0x01,0xD4,0x7E,0xAB,
		0x84,0x51,0xFB,0x2E,0x7A,0xAF,0x05,0xD0, 0xAD,0x78,0xD2,0x07,0x53,0x86,0x2C,0xF9
	};
	uint8_t crc = 0;
	for (int i = 0; i < len; i++) crc = crc8tab[crc ^ buf[i]];
	return crc;
}

// Parse CRSF frames in the buffer. Returns true when a channel frame was unpacked.
static bool parseCRSFBuffer() {
	while (crsfBufLen >= 4) {
		// CRSF address: 0xC8=flight controller, 0x00=broadcast, 0xEE=transmitter
		if (crsfBuf[0] != 0xC8 && crsfBuf[0] != 0x00 && crsfBuf[0] != 0xEE) {
			memmove(crsfBuf, crsfBuf + 1, --crsfBufLen);
			continue;
		}
		uint8_t frameLen = crsfBuf[1]; // TYPE + PAYLOAD + CRC length
		if (frameLen < 2 || frameLen > 62) {
			memmove(crsfBuf, crsfBuf + 1, --crsfBufLen);
			continue;
		}
		int totalLen = 2 + frameLen; // ADDR + LEN + (TYPE+PAYLOAD+CRC)
		if (crsfBufLen < totalLen) break; // wait for the rest of the frame

		// CRC over TYPE + PAYLOAD, not ADDR, LEN, or the CRC byte itself
		uint8_t calcCrc  = crsf_crc8(&crsfBuf[2], frameLen - 1);
		uint8_t frameCrc = crsfBuf[2 + frameLen - 1];
		if (calcCrc != frameCrc) {
			memmove(crsfBuf, crsfBuf + 1, --crsfBufLen);
			continue;
		}

		// RC Channels Packed (type 0x16, payload = 22 bytes, frameLen = 24)
		if (crsfBuf[2] == 0x16 && frameLen == 24) {
			const uint8_t *d = &crsfBuf[3]; // start of payload
			channels[ 0] = ((uint16_t)d[ 0]       | (uint16_t)d[ 1] << 8) & 0x7FF;
			channels[ 1] = ((uint16_t)d[ 1] >>  3 | (uint16_t)d[ 2] << 5) & 0x7FF;
			channels[ 2] = ((uint16_t)d[ 2] >>  6 | (uint16_t)d[ 3] << 2 | (uint16_t)d[ 4] << 10) & 0x7FF;
			channels[ 3] = ((uint16_t)d[ 4] >>  1 | (uint16_t)d[ 5] << 7) & 0x7FF;
			channels[ 4] = ((uint16_t)d[ 5] >>  4 | (uint16_t)d[ 6] << 4) & 0x7FF;
			channels[ 5] = ((uint16_t)d[ 6] >>  7 | (uint16_t)d[ 7] << 1 | (uint16_t)d[ 8] <<  9) & 0x7FF;
			channels[ 6] = ((uint16_t)d[ 8] >>  2 | (uint16_t)d[ 9] << 6) & 0x7FF;
			channels[ 7] = ((uint16_t)d[ 9] >>  5 | (uint16_t)d[10] << 3) & 0x7FF;
			channels[ 8] = ((uint16_t)d[11]        | (uint16_t)d[12] << 8) & 0x7FF;
			channels[ 9] = ((uint16_t)d[12] >>  3 | (uint16_t)d[13] << 5) & 0x7FF;
			channels[10] = ((uint16_t)d[13] >>  6 | (uint16_t)d[14] << 2 | (uint16_t)d[15] << 10) & 0x7FF;
			channels[11] = ((uint16_t)d[15] >>  1 | (uint16_t)d[16] << 7) & 0x7FF;
			channels[12] = ((uint16_t)d[16] >>  4 | (uint16_t)d[17] << 4) & 0x7FF;
			channels[13] = ((uint16_t)d[17] >>  7 | (uint16_t)d[18] << 1 | (uint16_t)d[19] <<  9) & 0x7FF;
			channels[14] = ((uint16_t)d[19] >>  2 | (uint16_t)d[20] << 6) & 0x7FF;
			channels[15] = ((uint16_t)d[20] >>  5 | (uint16_t)d[21] << 3) & 0x7FF;
			memmove(crsfBuf, crsfBuf + totalLen, crsfBufLen -= totalLen);
			return true;
		}

		// Other frame types (telemetry, device info): skip
		memmove(crsfBuf, crsfBuf + totalLen, crsfBufLen -= totalLen);
	}
	return false;
}

// If flash has no user calibration, write the protocol defaults
static void initDefaultRCCalibration() {
	bool anySet = false;
	for (int i = 0; i < 8; i++) {
		if (channelMax[i] != 0) { anySet = true; break; }
	}
	if (!anySet) {
		// CRSF: 172..1811; SBUS: 0..2047
		uint16_t zeroVal = (rcProtocol == 1) ? 172  : 0;
		uint16_t maxVal  = (rcProtocol == 1) ? 1811 : 2047;
		for (int i = 0; i < 16; i++) channelZero[i] = zeroVal;
		for (int i = 0; i < 16; i++) channelMax[i]  = maxVal;
	}
}

// ============================================================
// Public API
// ============================================================

void setupRC() {
	if (rcRxPin < 0) return; // no pin configured, skip RC init
	print("Setup RC protocol=%d\n", rcProtocol);
	if (rcProtocol == 1) {
		// CRSF / ELRS: standard levels, not inverted, 420000 baud
		BOARD_RC_SERIAL.begin(rcBaud, SERIAL_8N1, rcRxPin, rcTxPin);
	} else {
		// SBUS: 100000 baud, inverted (handled inside the SBUS library)
		rc.begin(rcRxPin);
	}
	initDefaultRCCalibration();
}

bool readRC() {
	if (rcRxPin < 0) return false; // RC disabled

	if (rcProtocol == 1) {
		// CRSF path: pull UART bytes into the buffer and try to parse a full frame
		while (BOARD_RC_SERIAL.available() &&
		       crsfBufLen < (int)sizeof(crsfBuf)) {
			crsfBuf[crsfBufLen++] = BOARD_RC_SERIAL.read();
		}
		if (parseCRSFBuffer()) {
			normalizeRC();
			controlTime = t;
			return true;
		}
		return false;
	} else {
		// SBUS path (unchanged)
		if (rc.read()) {
			SBUSData data = rc.data();
			for (int i = 0; i < 16; i++) channels[i] = data.ch[i];
			normalizeRC();
			controlTime = t;
			return true;
		}
		return false;
	}
}

void normalizeRC() {
	float controls[16];
	for (int i = 0; i < 16; i++) {
		controls[i] = mapf(channels[i], channelZero[i], channelMax[i], 0, 1);
	}
	// Update control values
	controlRoll = rollChannel >= 0 ? controls[(int)rollChannel] : NAN;
	controlPitch = pitchChannel >= 0 ? controls[(int)pitchChannel] : NAN;
	controlYaw = yawChannel >= 0 ? controls[(int)yawChannel] : NAN;
	controlThrottle = throttleChannel >= 0 ? controls[(int)throttleChannel] : NAN;
	controlMode = modeChannel >= 0 ? controls[(int)modeChannel] : NAN;
}

void calibrateRC() {
	uint16_t zero[16];
	uint16_t center[16];
	uint16_t max[16];
	print("1/8 RC calibration. Center every stick and set every switch to its default. Starts in 8 seconds.\n");
	pause(8);
	calibrateRCChannel(NULL, zero, zero, "2/8 Reference. Left stick down, right stick centered. Hold.\n...     ...\n...     .o.\n.o.     ...\n");
	calibrateRCChannel(NULL, center, center, "3/8 Reference. Left stick centered, right stick centered. Hold.\n...     ...\n.o.     .o.\n...     ...\n");
	calibrateRCChannel(&throttleChannel, zero, max, "4/8 Throttle. Left stick full up (max throttle), right stick centered. Hold.\n.o.     ...\n...     .o.\n...     ...\n");
	calibrateRCChannel(&yawChannel, center, max, "5/8 Yaw. Left stick full right (yaw right), right stick centered. Hold.\n...     ...\n..o     .o.\n...     ...\n");
	calibrateRCChannel(&pitchChannel, zero, max, "6/8 Pitch. Left stick full down, right stick full up (pitch forward). Hold.\n...     .o.\n...     ...\n.o.     ...\n");
	calibrateRCChannel(&rollChannel, zero, max, "7/8 Roll. Left stick full down, right stick full right (roll right). Hold.\n...     ...\n...     ..o\n.o.     ...\n");
	calibrateRCChannel(&modeChannel, zero, max, "8/8 Mode. Set the mode switch to its highest position. Hold.\n");
	printRCCalibration();
}

// Step 1: start pose
//   center every stick
//   every switch at its default (usually disarmed / middle)
//   hold still
// Step 2: base stick move
//   left stick down, right stick centered, hold
// Step 3: sticks centered
//   both sticks centered, hold
// Step 4: throttle channel
//   left stick full up (max throttle), right stick centered, hold
//   the throttle channel is detected from that move
// Step 5: yaw channel
//   left stick full right (yaw right), right stick centered, hold
// Step 6: pitch channel
//   left stick full down, right stick full up (pitch forward), hold
// Step 7: roll channel
//   left stick full down, right stick full right (roll right), hold
// Step 8: mode channel
//   return the arm switch to disarmed first (safety, not part of the calibration)
//   then set the mode switch to its highest position and hold
//   the mode channel is detected from that move

void calibrateRCChannel(float *channel, uint16_t in[16], uint16_t out[16], const char *str) {
	print("%s", str);
	pause(8);
	for (int i = 0; i < 30; i++) readRC(); // try update 30 times max
	memcpy(out, channels, sizeof(channels));

	if (channel == NULL) {
		print("  -> reference recorded\n"); // center steps only store a baseline; no channel is identified
		return;
	}

	// Find channel that changed the most between in and out
	int ch = -1, diff = 0;
	for (int i = 0; i < 16; i++) {
		if (abs(out[i] - in[i]) > diff) {
			ch = i;
			diff = abs(out[i] - in[i]);
		}
	}
	if (ch >= 0 && diff > 10) { // difference threshold is 10
		*channel = ch;
		channelZero[ch] = in[ch];
		channelMax[ch] = out[ch];
		print("  -> channel %d (change %d, zero=%u max=%u)\n", ch, diff, in[ch], out[ch]);
	} else {
		*channel = NAN;
		print("  -> not detected. Largest change was %d (need > 10). Move the stick or switch as prompted, and confirm the link with the rc command.\n", diff);
	}
}

void printRCCalibration() {
	print("Control   Ch     Zero   Max\n");
	print("Roll      %-7g%-7g%-7g\n", rollChannel, rollChannel >= 0 ? channelZero[(int)rollChannel] : NAN, rollChannel >= 0 ? channelMax[(int)rollChannel] : NAN);
	print("Pitch     %-7g%-7g%-7g\n", pitchChannel, pitchChannel >= 0 ? channelZero[(int)pitchChannel] : NAN, pitchChannel >= 0 ? channelMax[(int)pitchChannel] : NAN);
	print("Yaw       %-7g%-7g%-7g\n", yawChannel, yawChannel >= 0 ? channelZero[(int)yawChannel] : NAN, yawChannel >= 0 ? channelMax[(int)yawChannel] : NAN);
	print("Throttle  %-7g%-7g%-7g\n", throttleChannel, throttleChannel >= 0 ? channelZero[(int)throttleChannel] : NAN, throttleChannel >= 0 ? channelMax[(int)throttleChannel] : NAN);
	print("Mode      %-7g%-7g%-7g\n", modeChannel, modeChannel >= 0 ? channelZero[(int)modeChannel] : NAN, modeChannel >= 0 ? channelMax[(int)modeChannel] : NAN);
}
