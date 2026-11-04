// Web remote server

#if WEB_RC_ENABLED

#include <WebServer.h>
#include <WiFi.h>
#include "web_rc_html.h"
#include "board_config.h"

// Shared pilot inputs. The web protocol writes the same variables as SBUS and MAVLink.
extern float t;
extern float controlTime;
extern float controlRoll, controlPitch, controlYaw, controlThrottle, controlMode;

// ==================== constants ====================
#define WEB_RC_TIMEOUT_MS    10000          // link down if no packet arrives within this many ms; must exceed the heartbeat
// VBAT_ADC_PIN / VBAT_ADC_SAMPLES / VBAT_DIVIDER live in battery.ino

// ==================== link flags ====================
bool webRCEnabled    = false;  // a web RC link is currently valid (updated every frame in readWebRC())
bool useWebRC        = false;  // web RC is the active pilot input (kept equal to webRCEnabled)
bool webRCUpdated    = false;  // at least one stick packet has arrived (false until the first one)
bool webConsoleEnabled = false; // web debug console: POST /console/enable turns it on, then print() is copied into the buffer
char webRCWarnMsg[64] = ""; // warning for the page; cleared after it is sent once

// ==================== last request, used to build the response ====================
static int lastProcType        = 0;   // 1=stick 2=button 4=heartbeat
static int lastProcButtonIdx   = -1;  // button index 0..15, or -1
static int lastProcButtonState = -1;  // 1=pressed, 0=released, or -1

// ==================== last stick values, for the status endpoint ====================
// Units: throttle 0..100 (%), attitude axes +/-30 (deg), button 1=down 0=up
float webRCRoll     = 0.0f;
float webRCPitch    = 0.0f;
float webRCYaw      = 0.0f;
float webRCThrottle = 0.0f;
uint16_t webRCButtons    = 0;       // 16-bit button mask: bit0=arm bit1=disarm bit2=e-stop bit6=STAB bit7=ACRO bit8=ALTHOLD
unsigned long webRCLastUpdate = 0;  // millis() of the last packet

// ==================== sensitivity ====================
// output = processed angle * scale / STICK_MAX, written into control* ([-1,1])
// smaller scale = softer; larger = more sensitive
float webRCThrottleScale = 1.0f;    // throttle cap. 1.0=100%, 0.8=80% max
float webRCStickScale    = 0.85f;   // roll/pitch sensitivity, useful range 0.5..1.0
float webRCYawScale      = 0.68f;   // yaw sensitivity. Yaw inertia is small, so this stays lower. Range 0.3..0.8

// ==================== deadzone ====================
// Normalized 0..1. Inside the deadzone the output is 0.
// Larger = wider center and fewer false inputs. Smaller = more sensitive, easier to drift.
float stickDeadzone    = 0.06f;     // roll, pitch, and yaw, 6%
float throttleDeadzone = 0.06f;     // low-throttle deadzone. Below this the output is 0%.

// ==================== input ranges (keep in sync with the page) ====================
static const float THROTTLE_MIN = 0.0f;    // throttle output minimum (%)
static const float THROTTLE_MAX = 100.0f;  // throttle output maximum (%)
static const float STICK_MAX    = 30.0f;   // full-stick attitude (deg). A change must match TILT_MAX
static const float RAW_MAX      = 100.0f;  // page stick full scale. The page JS always sends +/-100

// ==================== runtime state ====================
static float lastValidThrottle = 0.0f;     // last accepted throttle, used when a sample is rejected
static float lastValidRoll     = 0.0f;     // last accepted roll
static float lastValidPitch    = 0.0f;     // last accepted pitch
static float lastValidYaw      = 0.0f;     // last accepted yaw
static unsigned long lastDataErrorTime = 0; // last bad-sample time, rate-limits the error log

// ==================== web server ====================
WebServer webRCServer(80);          // port 80, so the URL needs no port

// ------ Old PCBs printed :8080. Redirect 301 to port 80. Delete once those boards are gone. ------------
// Heap pointer, constructed in setupWebRC(). BSS cost is 4 bytes.
// ESP32-C3 has no old boards. The redirect is compiled out so a single core does not spin for 200 ms or hold an extra socket.
#ifndef CONFIG_IDF_TARGET_ESP32C3
static WiFiServer* redirectServer8080 = nullptr;
#endif
// ------ Old PCBs printed :8080. Redirect 301 to port 80. Delete once those boards are gone. ------------

// ==================== console log buffer ====================
#define CONSOLE_LINES    BOARD_CONSOLE_LINES    // lines: C3=20 (~7 KB saved) / ESP32 and S3=50
#define CONSOLE_LINE_LEN BOARD_CONSOLE_LINE_LEN  // chars per line: C3=160 / ESP32 and S3=240
static char consoleBuf[CONSOLE_LINES][CONSOLE_LINE_LEN];
static int  consoleTail   = 0;
static int  consoleFilled = 0;
static int  consoleTotal  = 0;   // monotonic line count, used as the incremental cursor

#define CONSOLE_CMD_QUEUE_SIZE 4
#define CONSOLE_CMD_LEN 64
static char consoleCmdQueue[CONSOLE_CMD_QUEUE_SIZE][CONSOLE_CMD_LEN];
static int  consoleCmdHead  = 0;
static int  consoleCmdTail  = 0;
static int  consoleCmdCount = 0;

bool enqueueConsoleCmd(const char* cmd) {
    if (!cmd || !*cmd) return false;
    if (consoleCmdCount >= CONSOLE_CMD_QUEUE_SIZE) return false;
    strncpy(consoleCmdQueue[consoleCmdTail], cmd, CONSOLE_CMD_LEN - 1);
    consoleCmdQueue[consoleCmdTail][CONSOLE_CMD_LEN - 1] = '\0';
    consoleCmdTail = (consoleCmdTail + 1) % CONSOLE_CMD_QUEUE_SIZE;
    consoleCmdCount++;
    return true;
}

void processConsoleCommandQueue() {
    if (consoleCmdCount <= 0) return;

    String cmd = consoleCmdQueue[consoleCmdHead];
    consoleCmdHead = (consoleCmdHead + 1) % CONSOLE_CMD_QUEUE_SIZE;
    consoleCmdCount--;
    doCommand(cmd, false);
}

void webLog(const char* msg) {
    // Split on \n so a newline does not land inside the JSON string
    const char* start = msg;
    while (*start) {
        const char* end = strchr(start, '\n');
        int len = end ? (int)(end - start) : (int)strlen(start);
        if (len > 0) {
            int copy = (len < CONSOLE_LINE_LEN - 1) ? len : (CONSOLE_LINE_LEN - 1);
            strncpy(consoleBuf[consoleTail], start, copy);
            consoleBuf[consoleTail][copy] = '\0';
            consoleTail = (consoleTail + 1) % CONSOLE_LINES;
            if (consoleFilled < CONSOLE_LINES) consoleFilled++;
            consoleTotal++;
        }
        if (!end) break;
        start = end + 1;
    }
}

// ==================== sticks ====================

// Deadzone in normalized space. Input and output are both [-1, 1].
// Outside the deadzone the remainder is scaled back to full range, so full stick is still full output.
float applyDeadzone(float norm, float deadzone) {
    if (fabsf(norm) < deadzone) return 0.0f;
    float sign = (norm > 0.0f) ? 1.0f : -1.0f;
    return sign * (fabsf(norm) - deadzone) / (1.0f - deadzone);
}

// Attitude axis (roll / pitch / yaw)
// Input raw in [-100, +100], output in [-STICK_MAX, +STICK_MAX] (deg)
float processAxis(float raw, float& lastValid) {
    if (isnan(raw) || isinf(raw) || fabsf(raw) > 1000.0f) {
        if (millis() - lastDataErrorTime > 2000) lastDataErrorTime = millis();
        return lastValid;
    }
    float norm = constrain(raw, -RAW_MAX, RAW_MAX) / RAW_MAX;
    norm = applyDeadzone(norm, stickDeadzone);
    lastValid = norm * STICK_MAX;
    return lastValid;
}

// Throttle axis
// Page: left stick Y, bottom=-100, top=+100
// Output: pct in [0, 100]
float processThrottle(float raw) {
    if (isnan(raw) || isinf(raw) || fabsf(raw) > 1000.0f) return lastValidThrottle;
    raw = constrain(raw, -RAW_MAX, RAW_MAX);
    // Linear map: -100 -> 0%, 0 -> 50%, +100 -> 100%
    float pct = (raw + RAW_MAX) / (2.0f * RAW_MAX) * THROTTLE_MAX;
    if (pct < throttleDeadzone * THROTTLE_MAX) pct = 0.0f;
    pct = constrain(pct * webRCThrottleScale, THROTTLE_MIN, THROTTLE_MAX);
    lastValidThrottle = pct;
    return pct;
}

// ==================== apply a packet ====================

void setWebRCInput(float roll, float pitch, float yaw, float throttle, uint16_t buttons) {
    float pThrottle = processThrottle(throttle);
    float pYaw      = processAxis(yaw,   lastValidYaw);
    float pPitch    = processAxis(pitch, lastValidPitch);
    float pRoll     = processAxis(roll,  lastValidRoll);

    // Keep the processed values for the status endpoint
    webRCThrottle = pThrottle;
    webRCYaw      = pYaw;
    webRCPitch    = pPitch;
    webRCRoll     = pRoll;
    webRCButtons  = buttons;
    webRCLastUpdate = millis();
    webRCUpdated  = true;

    // Write the shared control variables (same path as SBUS and MAVLink)
    controlRoll     = constrain(pRoll  * webRCStickScale / STICK_MAX, -1.0f, 1.0f);
    controlPitch    = constrain(pPitch * webRCStickScale / STICK_MAX, -1.0f, 1.0f);
    controlYaw      = constrain(pYaw   * webRCYawScale   / STICK_MAX, -1.0f, 1.0f);
    controlThrottle = pThrottle / THROTTLE_MAX;
    controlMode     = NAN;
    controlTime     = t;

    static float lastPrintedThrottle = -1.0f;
    if (fabsf(pThrottle - lastPrintedThrottle) > 5.0f) {
        print("WebRC T=%.0f%% R=%.1f P=%.1f Y=%.1f Btn=0x%04X\n",
              pThrottle, pRoll, pPitch, pYaw, buttons);
        lastPrintedThrottle = pThrottle;
    }
}

// ==================== JSON protocol ====================

const char* findJsonValue(const char* json, const char* key) {
    const char* p = strstr(json, key);
    if (!p) return nullptr;
    p = strchr(p, ':');
    if (!p) return nullptr;
    p++;
    while (*p == ' ' || *p == '"') p++;
    return p;
}

bool handleJSONProtocol(String& body) {
    if (body.length() == 0 || body.indexOf('{') == -1) return false;
    const char* json = body.c_str();
    const char* typePos = strstr(json, "\"t\":");
    if (!typePos) return false;
    int type = atoi(typePos + 4);
    const char* v;

    lastProcType        = type;
    lastProcButtonIdx   = -1;
    lastProcButtonState = -1;

    switch (type) {
        case 1: { // stick data
            float th = 0, r = 0, p = 0, y = 0;
            uint32_t ts = 0;
            if ((v = findJsonValue(json, "\"th\""))) th = atof(v);
            if ((v = findJsonValue(json, "\"r\"")))  r  = atof(v);
            if ((v = findJsonValue(json, "\"p\"")))  p  = atof(v);
            if ((v = findJsonValue(json, "\"y\"")))  y  = atof(v);
            if ((v = findJsonValue(json, "\"ts\""))) ts = atol(v);
            setWebRCInput(r, p, y, th, webRCButtons);
            break;
        }
        case 2: { // button event
            int idx = 0, state = 0;
            uint32_t ts = 0;
            if ((v = findJsonValue(json, "\"b\"")))  idx   = atoi(v);
            if ((v = findJsonValue(json, "\"s\"")))  state = atoi(v);
            if ((v = findJsonValue(json, "\"ts\""))) ts    = atol(v);
            if (idx >= 0 && idx < 16) {
                if (state) webRCButtons |=  (1 << idx);
                else       webRCButtons &= ~(1 << idx);
                webRCLastUpdate = millis();
                webRCUpdated    = true;
                lastProcButtonIdx   = idx;
                lastProcButtonState = state;
            }
            break;
        }
        case 4: // heartbeat
            webRCLastUpdate = millis();
            webRCUpdated    = true;
            break;
    }
    return true;
}

void setWebRCWarn(const char* msg) {
    strncpy(webRCWarnMsg, msg, sizeof(webRCWarnMsg) - 1);
    webRCWarnMsg[sizeof(webRCWarnMsg) - 1] = '\0';
}

// ==================== HTTP request ====================

void handleWebRCRequest() {
    if (!webRCServer.hasArg("plain")) {
        webRCServer.send(400, "application/json", "{\"e\":\"no data\"}");
        return;
    }
    String body = webRCServer.arg("plain");
    if (handleJSONProtocol(body)) {
        char resp[320];
        // warn is attached and then cleared on button (rt=2) and heartbeat (rt=4) replies:
        //   rt=2: a user action (arm rejected, and similar) needs an immediate reply
        //   rt=4: a system event (low-battery disarm, and similar). Heartbeat is about 2 s, which is fine
        // Stick packets (rt=1) do not consume warn, so they cannot clear it before the state=0 reply
        bool deliverWarn = (lastProcType == 2 || lastProcType == 4) && webRCWarnMsg[0];
        if (deliverWarn) {
            if (lastProcButtonIdx >= 0) {
                snprintf(resp, sizeof(resp),
                    "{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d,\"bi\":%d,\"bs\":%d,\"warn\":\"%s\"}",
                    mode, (int)armed, lastProcType, lastProcButtonIdx, lastProcButtonState, webRCWarnMsg);
            } else {
                snprintf(resp, sizeof(resp),
                    "{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d,\"warn\":\"%s\"}",
                    mode, (int)armed, lastProcType, webRCWarnMsg);
            }
            webRCWarnMsg[0] = '\0'; // clear as soon as it is sent
        } else {
            if (lastProcButtonIdx >= 0) {
                snprintf(resp, sizeof(resp),
                    "{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d,\"bi\":%d,\"bs\":%d}",
                    mode, (int)armed, lastProcType, lastProcButtonIdx, lastProcButtonState);
            } else {
                snprintf(resp, sizeof(resp),
                    "{\"s\":\"ok\",\"m\":%d,\"arm\":%d,\"rt\":%d}",
                    mode, (int)armed, lastProcType);
            }
        }
        webRCServer.send(200, "application/json", resp);
    } else {
        webRCServer.send(400, "application/json", "{\"e\":\"parse failed\"}");
    }
}

// ==================== link state ====================

bool isWebRCEnabled() {
    return webRCUpdated && (millis() - webRCLastUpdate < WEB_RC_TIMEOUT_MS);
}

bool isUsingWebRC() {
    return useWebRC && isWebRCEnabled();
}

// ------ Old PCBs printed :8080. Redirect 301 to port 80. Delete once those boards are gone. ------------
// Old remote port. Serve a 301 on port 8080 (raw WiFiServer TCP, HTTP parsed by hand).
// Compiled out on ESP32-C3. A 200 ms spin on the only core would starve the Wi-Fi task.
#ifndef CONFIG_IDF_TARGET_ESP32C3
static void handleRedirect8080() {
    if (!redirectServer8080) return;
    WiFiClient client = redirectServer8080->accept();
    if (!client) return;

    String path = "/";
    String host = "";
    unsigned long t0 = millis();
    while (!client.available() && millis() - t0 < 200) {}

    if (client.available()) {
        // Request line: GET /path HTTP/1.1
        String reqLine = client.readStringUntil('\n');
        int s1 = reqLine.indexOf(' ');
        int s2 = reqLine.indexOf(' ', s1 + 1);
        if (s1 >= 0 && s2 > s1) path = reqLine.substring(s1 + 1, s2);

        // Read headers and take Host, dropping the :8080 port
        while (client.available()) {
            String line = client.readStringUntil('\n');
            line.trim();
            if (line.length() == 0) break;
            if (line.startsWith("Host:") || line.startsWith("host:")) {
                host = line.substring(5);
                host.trim();
                int colon = host.indexOf(':');
                if (colon >= 0) host = host.substring(0, colon);
            }
        }
    }

    client.print("HTTP/1.1 301 Moved Permanently\r\nLocation: http://");
    client.print(host);
    client.print(path);
    client.print("\r\nConnection: close\r\nContent-Length: 0\r\n\r\n");
    client.stop();
}
#endif
// ------ Old PCBs printed :8080. Redirect 301 to port 80. Delete once those boards are gone. ------------

// ==================== setup ====================

void setupWebRC() {
    print("Setup WEB RC\n");
    lastValidThrottle = THROTTLE_MIN;
    lastValidRoll = lastValidPitch = lastValidYaw = 0.0f;

    webRCServer.on("/", HTTP_GET, []() {
        webRCServer.send(200, "text/html", webRCIndexHtml);
    });
    webRCServer.on("/web_rc",           HTTP_POST, handleWebRCRequest);
    webRCServer.on("/web_rc/heartbeat", HTTP_POST, handleWebRCRequest);

    webRCServer.on("/console", HTTP_GET, []() {
        int since = -1;
        int limit = 20;
        if (webRCServer.hasArg("since")) since = webRCServer.arg("since").toInt();
        if (webRCServer.hasArg("limit")) limit = webRCServer.arg("limit").toInt();
        if (limit <= 0) limit = 20;
        if (limit > CONSOLE_LINES) limit = CONSOLE_LINES;

        int availableFrom = max(0, consoleTotal - consoleFilled);
        int sendFrom = (since >= 0) ? since : availableFrom;
        if (sendFrom < availableFrom) sendFrom = availableFrom;
        if (sendFrom > consoleTotal) sendFrom = consoleTotal;
        int sendTo = min(consoleTotal, sendFrom + limit);

        String json = "{\"total\":";
        json += consoleTotal;
        json += ",\"next\":";
        json += sendTo;
        json += ",\"has_more\":";
        json += (sendTo < consoleTotal) ? "true" : "false";
        json += ",\"lines\":[";

        bool first = true;
        for (int i = sendFrom; i < sendTo; i++) {
            int idx = i % CONSOLE_LINES;
            if (!first) json += ",";
            first = false;
            json += "\"";
            String line = consoleBuf[idx];
            line.replace("\\", "\\\\");  // \ → \\
            line.replace("\"", "\\\"");  // " → \"
            line.replace("\n", "\\n");    // leftover newline -> \n
            line.replace("\r", "\\r");    // leftover CR -> \r
            json += line + "\"";
        }
        json += "]}";
        webRCServer.send(200, "application/json", json);
    });

    webRCServer.on("/console/cmd", HTTP_POST, []() {
        String cmd = webRCServer.arg("plain");
        cmd.trim();
        if (cmd.length() == 0) {
            webRCServer.send(400, "application/json", "{\"ok\":0,\"e\":\"empty command\"}");
            return;
        }

        char buf[CONSOLE_LINE_LEN];
        snprintf(buf, sizeof(buf), "> %s", cmd.c_str());
        webLog(buf);

        if (!enqueueConsoleCmd(cmd.c_str())) {
            webLog("! command queue is full");
            webRCServer.send(503, "application/json", "{\"ok\":0,\"e\":\"queue full\"}");
            return;
        }

        webRCServer.send(200, "application/json", "{\"ok\":1,\"queued\":1}");
    });

    webRCServer.on("/console/enable", HTTP_POST, []() {
        webConsoleEnabled = true;
        webRCServer.send(200, "application/json", "{\"ok\":1}");
    });

    webRCServer.on("/console/disable", HTTP_POST, []() {
        webConsoleEnabled = false;
        webRCServer.send(200, "application/json", "{\"ok\":1}");
    });

    webRCServer.on("/web_rc/status", HTTP_GET, []() {
        float vbat = readBatteryVoltage();
        if (isnan(vbat) || vbat < 0.0f) vbat = 0.0f;
        char json[384];
        snprintf(json, sizeof(json),
            "{\"enabled\":%s,\"active\":%s,"
            "\"voltage\":%.2f,"
            "\"throttle\":%.1f,\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f}",
            isWebRCEnabled() ? "true" : "false",
            isUsingWebRC()   ? "true" : "false",
            vbat,
            webRCThrottle, webRCRoll, webRCPitch, webRCYaw);
        webRCServer.send(200, "application/json", json);
    });

    webRCServer.begin();

    // ------ Old PCBs printed :8080. Redirect 301 to port 80. Delete once those boards are gone. ------------
#ifndef CONFIG_IDF_TARGET_ESP32C3
    redirectServer8080 = new WiFiServer(8080); // heap, so it does not sit in BSS
    redirectServer8080->begin(); // light redirect on port 8080 for old PCB silkscreen URLs
#endif
    // ------ Old PCBs printed :8080. Redirect 301 to port 80. Delete once those boards are gone. ------------

    print("Web RC started: http://192.168.4.1\n");
    print("  deadzone stick=%.0f%% throttle=%.0f%% | scale stick=%.2f yaw=%.2f\n",
          stickDeadzone * 100.0f, throttleDeadzone * 100.0f,
          webRCStickScale, webRCYawScale);
}

// ==================== loop ====================

void readWebRC() {
    webRCServer.handleClient();

    // ------ Old PCBs printed :8080. Redirect 301 to port 80. Delete once those boards are gone. ------------
#ifndef CONFIG_IDF_TARGET_ESP32C3
    handleRedirect8080(); // 8080 redirect for old PCB URLs
#endif
    // ------ Old PCBs printed :8080. Redirect 301 to port 80. Delete once those boards are gone. ------------
    
    if (isWebRCEnabled()) {
        webRCEnabled = useWebRC = true;
    } else {
        webRCEnabled = useWebRC = false;
    }
}

#else
void setupWebRC() { print("Web RC disabled\n"); }
void readWebRC()  {}
void processConsoleCommandQueue() {}
#endif
