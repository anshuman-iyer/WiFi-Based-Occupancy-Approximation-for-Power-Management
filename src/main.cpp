/*
 * ================================================================
 * ESP32 WIFI OCCUPANCY NODE + 8-PIXEL NEOPIXEL DISPLAY   (v8)
 * ================================================================
 * Flash this to the third board: it has an 8-pixel NeoPixel strip and
 * NO mic and NO buzzer. NODE-01 has the mic and NODE-02 has the buzzer; this board is
 * NODE-03 and is the plain display/sensor node.
 *
 * This board is a normal sniffer node AND renders whatever the
 * Python server tells it to via POST /api/actuator. It has no
 * opinion of its own about which of the 3 modes is active -- the
 * server is the single source of truth (see server.py's
 * current_display_mode). This board just renders the last payload
 * it received:
 *
 *   {"mode": "combined", "count": N}            -- bar-graph of the
 *                                                   FUSED room count,
 *                                                   identical on all
 *                                                   3 strips
 *   {"mode": "area", "count": N, "scale": 0..1}  -- bar-graph of
 *                                                   THIS node's own
 *                                                   local count;
 *                                                   overall brightness
 *                                                   set by `scale`
 *                                                   (1.0 = the
 *                                                   busiest node right
 *                                                   now, dimmer the
 *                                                   less busy this
 *                                                   zone is relative
 *                                                   to the others)
 *   {"mode": "standby"}                          -- steady warm light,
 *                                                   ignores occupancy
 *                                                   entirely
 *
 * If no update arrives for ACTUATOR_STALE_MS, falls back to a slow
 * breathing dim-amber "no fresh data" state rather than freezing on
 * a possibly-stale number.
 *
 * REQUIRED LIBRARY (add to platformio.ini):
 *     lib_deps = adafruit/Adafruit NeoPixel @ ^1.12.3
 * ================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include <math.h>
#include <Adafruit_NeoPixel.h>

// BOARD ROLE: DISPLAY/SNIFFER + 8-PIXEL NEOPIXEL
constexpr const char* DEFAULT_NODE_ID = "NODE-03";

// >>> CHANGE THIS PER BOARD <<<
constexpr uint8_t STATIC_IP_LAST_OCTET = 201;

// >>> SAME ON ALL 3 BOARDS <<<
constexpr const char* WIFI_SSID     = "shashank";
constexpr const char* WIFI_PASSWORD = "shekhar1";

static const IPAddress STATIC_GATEWAY(192, 168, 137, 1);
static const IPAddress STATIC_SUBNET(255, 255, 255, 0);
static const IPAddress STATIC_DNS(192, 168, 137, 1);

constexpr uint32_t HASH_SALT = 0xC001D00D;

constexpr uint32_t DEVICE_TIMEOUT_MS   = 30000;
constexpr uint32_t REPORT_INTERVAL_MS  = 5000;
constexpr int      MAX_DEVICES_TRACKED = 128;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
constexpr uint32_t WIFI_RETRY_INTERVAL_MS  = 10000;

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
constexpr int LED_PIN = LED_BUILTIN;
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;
constexpr uint32_t HEARTBEAT_ON_MS       = 50;
constexpr uint32_t ERROR_BLINK_MS        = 120;

constexpr int NEOPIXEL_PIN   = 4;
constexpr int NEOPIXEL_COUNT = 8;   // exactly 8 NeoPixels
constexpr int OCCUPANCY_CAP  = NEOPIXEL_COUNT;

constexpr uint32_t ACTUATOR_STALE_MS = 60000;

Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

static String   currentMode = "standby";
static int      currentCount = 0;
static float    currentScale = 1.0f;
static uint32_t lastActuatorUpdateMs = 0;
static bool     everReceivedActuatorUpdate = false;

WebServer server(80);
String nodeId;

struct SeenDevice {
    uint32_t id = 0;
    int8_t   lastRssi = 0;
    uint32_t firstSeenMs = 0;
    uint32_t lastSeenMs = 0;
    uint32_t probeCount = 0;
    bool     active = false;
};
static SeenDevice devices[MAX_DEVICES_TRACKED];

static uint32_t windowProbeCount = 0;
static uint32_t windowNewDevices = 0;
static uint32_t totalUniqueSinceBoot = 0;
static portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t lastReportTime = 0;
static uint32_t lastWifiRetry = 0;
static uint32_t lastBlinkToggle = 0;
static bool blinkOn = false;
static bool webServerStarted = false;

static uint32_t hashMac(const uint8_t* mac) {
    uint32_t hash = 2166136261u ^ HASH_SALT;
    for (int i = 0; i < 6; i++) { hash ^= mac[i]; hash *= 16777619u; }
    return hash;
}

static void recordDevice(uint32_t id, int8_t rssi, uint32_t now) {
    for (int i = 0; i < MAX_DEVICES_TRACKED; i++) {
        if (devices[i].active && devices[i].id == id) {
            devices[i].lastRssi = rssi;
            devices[i].lastSeenMs = now;
            devices[i].probeCount++;
            return;
        }
    }
    for (int i = 0; i < MAX_DEVICES_TRACKED; i++) {
        if (!devices[i].active) {
            devices[i].id = id;
            devices[i].lastRssi = rssi;
            devices[i].firstSeenMs = now;
            devices[i].lastSeenMs = now;
            devices[i].probeCount = 1;
            devices[i].active = true;
            windowNewDevices++;
            totalUniqueSinceBoot++;
            return;
        }
    }
}

static void pruneStaleDevices(uint32_t now) {
    for (int i = 0; i < MAX_DEVICES_TRACKED; i++)
        if (devices[i].active && (now - devices[i].lastSeenMs > DEVICE_TIMEOUT_MS))
            devices[i].active = false;
}

static void IRAM_ATTR wifiSnifferCallback(void* buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) return;
    auto* packet = reinterpret_cast<wifi_promiscuous_pkt_t*>(buf);
    const uint8_t* payload = packet->payload;
    uint8_t frameSubtype = payload[0] & 0xF0;
    if (frameSubtype != 0x40) return;

    const uint8_t* sourceMac = payload + 10;
    int8_t rssi = packet->rx_ctrl.rssi;
    uint32_t id = hashMac(sourceMac);
    uint32_t now = millis();

    portENTER_CRITICAL_ISR(&dataMux);
    recordDevice(id, rssi, now);
    windowProbeCount++;
    portEXIT_CRITICAL_ISR(&dataMux);
}

static String buildStatusJSON() {
    uint32_t now = millis();
    portENTER_CRITICAL(&dataMux);
    pruneStaleDevices(now);
    int active = 0;
    long rssiSum = 0;
    for (int i = 0; i < MAX_DEVICES_TRACKED; i++)
        if (devices[i].active) { active++; rssiSum += devices[i].lastRssi; }
    double avgRssi = active ? (double)rssiSum / active : 0.0;
    uint32_t probes = windowProbeCount;
    uint32_t newDev = windowNewDevices;
    windowProbeCount = 0;
    windowNewDevices = 0;
    portEXIT_CRITICAL(&dataMux);

    String json = "{";
    json += "\"node\":\"" + nodeId + "\",";
    json += "\"wifi_connected\":"; json += (WiFi.status() == WL_CONNECTED ? "true" : "false"); json += ",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    json += "\"devices\":" + String(active) + ",";
    json += "\"new_devices\":" + String(newDev) + ",";
    json += "\"total_unique_since_boot\":" + String(totalUniqueSinceBoot) + ",";
    json += "\"probes\":" + String(probes) + ",";
    json += "\"avg_rssi\":" + String(avgRssi, 2) + ",";
    json += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
    json += "\"display_mode\":\"" + currentMode + "\",";
    json += "\"uptime_ms\":" + String(now);
    json += "}";
    return json;
}

static String buildDevicesJSON() {
    uint32_t now = millis();
    String json = "{\"node\":\"" + nodeId + "\",\"now_ms\":" + String(now) + ",\"devices\":[";
    portENTER_CRITICAL(&dataMux);
    pruneStaleDevices(now);
    bool first = true;
    for (int i = 0; i < MAX_DEVICES_TRACKED; i++) {
        if (devices[i].active) {
            if (!first) json += ",";
            first = false;
            json += "{\"id\":" + String(devices[i].id) + ",";
            json += "\"rssi\":" + String(devices[i].lastRssi) + ",";
            json += "\"last_seen_ms\":" + String(devices[i].lastSeenMs) + ",";
            json += "\"probe_count\":" + String(devices[i].probeCount) + "}";
        }
    }
    portEXIT_CRITICAL(&dataMux);
    json += "]}";
    return json;
}

// ================================================================
// ACTUATOR ENDPOINT -- receives {"mode":..., "count":..., "scale":...}
// Minimal inline parser for this specific controlled payload shape.
// ================================================================

static bool extractString(const String& body, const String& key, String& out) {
    String pattern = "\"" + key + "\"";
    int idx = body.indexOf(pattern);
    if (idx < 0) return false;
    int colon = body.indexOf(':', idx);
    if (colon < 0) return false;
    int q1 = body.indexOf('"', colon + 1);
    if (q1 < 0) return false;
    int q2 = body.indexOf('"', q1 + 1);
    if (q2 < 0) return false;
    out = body.substring(q1 + 1, q2);
    return true;
}

static bool extractNumber(const String& body, const String& key, double& out) {
    String pattern = "\"" + key + "\"";
    int idx = body.indexOf(pattern);
    if (idx < 0) return false;
    int colon = body.indexOf(':', idx);
    if (colon < 0) return false;
    int i = colon + 1;
    while (i < (int)body.length() && body[i] == ' ') i++;
    int start = i;
    while (i < (int)body.length() && (isDigit(body[i]) || body[i] == '-' || body[i] == '.')) i++;
    if (i == start) return false;
    out = body.substring(start, i).toDouble();
    return true;
}

static void handleActuatorPost() {
    if (!server.hasArg("plain")) {
        server.send(400, "text/plain", "Missing JSON body");
        return;
    }
    String body = server.arg("plain");

    String mode;
    if (!extractString(body, "mode", mode)) {
        server.send(400, "text/plain", "Expected {\"mode\": \"...\"}");
        return;
    }

    double countVal = 0, scaleVal = 1.0;
    extractNumber(body, "count", countVal);   // optional depending on mode
    extractNumber(body, "scale", scaleVal);   // optional, only used in "area" mode

    currentMode = mode;
    currentCount = (int)countVal;
    currentScale = (float)scaleVal;
    lastActuatorUpdateMs = millis();
    everReceivedActuatorUpdate = true;

    server.send(200, "text/plain", "OK");
}

// ================================================================
// NEOPIXEL RENDERING
// ================================================================

static uint32_t colorForFraction(float fraction) {
    if (fraction <= 0.4f)       return strip.Color(40, 180, 40);   // green
    else if (fraction <= 0.75f) return strip.Color(200, 140, 0);   // amber
    else                        return strip.Color(200, 30, 30);   // red
}

static void renderStandby() {
    uint32_t warmWhite = strip.Color(120, 90, 40);
    strip.setBrightness(90);
    for (int i = 0; i < NEOPIXEL_COUNT; i++) strip.setPixelColor(i, warmWhite);
    strip.show();
}

static void renderBarGraph(int count, float overallBrightnessScale) {
    int lit = constrain(count, 0, OCCUPANCY_CAP);
    float fraction = (float)lit / NEOPIXEL_COUNT;
    uint32_t color = colorForFraction(fraction);

    // overallBrightnessScale: 1.0 = full brightness (combined mode always
    // uses 1.0; area mode uses the server-computed relative-busyness scale)
    uint8_t brightness = (uint8_t)constrain(40 + overallBrightnessScale * 215, 0, 255);
    strip.setBrightness(brightness);

    for (int i = 0; i < NEOPIXEL_COUNT; i++) {
        strip.setPixelColor(i, i < lit ? color : strip.Color(0, 0, 0));
    }
    strip.show();
}

static void renderStale() {
    float phase = (millis() % 3000) / 3000.0f;
    float breath = (sinf(phase * 2 * PI) + 1.0f) / 2.0f;
    strip.setBrightness(255);
    uint8_t r = (uint8_t)(120 * breath);
    uint8_t g = (uint8_t)(70 * breath);
    for (int i = 0; i < NEOPIXEL_COUNT; i++) strip.setPixelColor(i, strip.Color(r, g, 0));
    strip.show();
}

static void updateDisplay() {
    bool stale = everReceivedActuatorUpdate &&
                 (millis() - lastActuatorUpdateMs > ACTUATOR_STALE_MS);

    if (stale) {
        renderStale();
    } else if (currentMode == "standby") {
        renderStandby();
    } else if (currentMode == "area") {
        renderBarGraph(currentCount, currentScale);
    } else { // "combined" (and any unrecognized mode falls back here, full brightness)
        renderBarGraph(currentCount, 1.0f);
    }
}

// ================================================================
// WEB SERVER / WIFI (unchanged from plain v6)
// ================================================================

static void handleRoot() {
    String html = "<html><body style='font-family:monospace;background:#111;color:#0f0;padding:20px'>"
                  "<h2>" + nodeId + " -- occupancy node + display</h2>"
                  "<p>See <a style='color:#4fd8e8' href='/api/status'>/api/status</a>, "
                  "<a style='color:#4fd8e8' href='/api/devices'>/api/devices</a></p></body></html>";
    server.send(200, "text/html", html);
}

static void setupWebServer() {
    if (WiFi.status() != WL_CONNECTED) return;
    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/status", HTTP_GET, []() { server.send(200, "application/json", buildStatusJSON()); });
    server.on("/api/devices", HTTP_GET, []() { server.send(200, "application/json", buildDevicesJSON()); });
    server.on("/api/actuator", HTTP_POST, handleActuatorPost);
    server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
    server.begin();
    webServerStarted = true;
    Serial.print("Web server up: http://"); Serial.println(WiFi.localIP());
}

static void applyStaticIP() {
    IPAddress staticIp(192, 168, 137, STATIC_IP_LAST_OCTET);
    if (!WiFi.config(staticIp, STATIC_GATEWAY, STATIC_SUBNET, STATIC_DNS)) {
        Serial.println("WiFi.config() failed to set static IP -- falling back to DHCP.");
    }
}

static bool connectWiFi() {
    Serial.println("Connecting to WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    applyStaticIP();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() != WL_CONNECTED) { Serial.println("WiFi FAILED."); return false; }
    Serial.print("WiFi connected. IP: "); Serial.println(WiFi.localIP());
    Serial.print("AP channel: "); Serial.println(WiFi.channel());
    return true;
}

static void maintainWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;
    uint32_t now = millis();
    if (now - lastWifiRetry < WIFI_RETRY_INTERVAL_MS) return;
    lastWifiRetry = now;
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.disconnect();
    applyStaticIP();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS)
        delay(100);
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Reconnected. IP: "); Serial.println(WiFi.localIP());
        if (!webServerStarted) setupWebServer();
        esp_wifi_set_promiscuous(true);
    }
}

static void startSniffer() {
    esp_wifi_set_promiscuous_rx_cb(&wifiSnifferCallback);
    esp_wifi_set_promiscuous(true);
    Serial.println("Sniffer active.");
}

static void updateLed() {
    uint32_t now = millis();
    bool healthy = (WiFi.status() == WL_CONNECTED);
    uint32_t onDuration = healthy ? HEARTBEAT_ON_MS : ERROR_BLINK_MS;
    uint32_t period      = healthy ? HEARTBEAT_INTERVAL_MS : (ERROR_BLINK_MS * 2);
    if (blinkOn) {
        if (now - lastBlinkToggle >= onDuration) { digitalWrite(LED_PIN, LOW); blinkOn = false; lastBlinkToggle = now; }
    } else {
        if (now - lastBlinkToggle >= (period - onDuration)) { digitalWrite(LED_PIN, HIGH); blinkOn = true; lastBlinkToggle = now; }
    }
}

static void reportWindow() {
    uint32_t now = millis();
    portENTER_CRITICAL(&dataMux);
    pruneStaleDevices(now);
    int active = 0;
    for (int i = 0; i < MAX_DEVICES_TRACKED; i++) if (devices[i].active) active++;
    portEXIT_CRITICAL(&dataMux);
    Serial.printf("[%s] active devices: %d | total unique: %u | heap: %u | mode: %s\n",
                  nodeId.c_str(), active, totalUniqueSinceBoot, ESP.getFreeHeap(), currentMode.c_str());
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    nodeId = DEFAULT_NODE_ID;

    Serial.println();
    Serial.println("==========================================");
    Serial.print(" NODE-03 | DISPLAY + 8-PIXEL NEOPIXEL -- "); Serial.println(nodeId);
    Serial.print(" Static IP target: 192.168.137."); Serial.println(STATIC_IP_LAST_OCTET);
    Serial.println("==========================================");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    strip.begin();
    renderStandby();

    if (connectWiFi()) {
        setupWebServer();
        startSniffer();
    } else {
        lastWifiRetry = millis();
    }

    lastReportTime = millis();
    lastBlinkToggle = millis();
    Serial.println("Init complete.");
}

void loop() {
    if (WiFi.status() == WL_CONNECTED && webServerStarted) server.handleClient();
    maintainWiFi();

    uint32_t now = millis();
    if (now - lastReportTime >= REPORT_INTERVAL_MS) {
        reportWindow();
        lastReportTime = now;
    }

    updateLed();
    updateDisplay();
    yield();
}