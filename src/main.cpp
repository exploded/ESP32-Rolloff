// ─────────────────────────────────────────────────────────────────────────────
//  ESP32 Rolloff Roof Alpaca Driver  (Arduino framework)
// ─────────────────────────────────────────────────────────────────────────────
//
//  ┌─────────────────────────────────────────────────────────────────────────┐
//  │  PIN CONNECTIONS  (ESP32 DevKit-C 38-pin, USB-C end at top)             │
//  │                                                                         │
//  │  LEFT SIDE  (top → bottom from USB-C)                                   │
//  │    3V3  ──────────────────── OLED VCC                                   │
//  │    GND  ──────────────────── OLED GND                                   │
//  │    D15  (GPIO15)                                                        │
//  │    D2   (GPIO2)                                                         │
//  │    D4   (GPIO4)                                                         │
//  │    RX2  (GPIO16)                                                        │
//  │    TX2  (GPIO17)                                                        │
//  │    D5   (GPIO5)                                                         │
//  │    D18  (GPIO18) ──────────── RELAY signal                              │
//  │    D19  (GPIO19) ──────────── OLED SCK  ┐ adjacent pair                │
//  │    D21  (GPIO21) ──────────── OLED SDA  ┘                              │
//  │    RX0  (GPIO3)                                                         │
//  │    TX0  (GPIO1)                                                         │
//  │    D22  (GPIO22)                                                        │
//  │    D23  (GPIO23)                                                        │
//  │    GND                                                                  │
//  │                                                                         │
//  │  RIGHT SIDE (top → bottom from USB-C)                                   │
//  │    VIN                                                                  │
//  │    GND  ──────────────────── switch common (both switches)              │
//  │    D13  (GPIO13)                                                        │
//  │    D12  (GPIO12)                                                        │
//  │    D14  (GPIO14)                                                        │
//  │    D27  (GPIO27)                                                        │
//  │    D26  (GPIO26)                                                        │
//  │    D25  (GPIO25)                                                        │
//  │    D33  (GPIO33) ──────────── Limit switch OPEN  (other terminal→GND)  │
//  │    D32  (GPIO32) ──────────── Limit switch CLOSED (other terminal→GND) │
//  │    D35  (GPIO35)                                                        │
//  │    D34  (GPIO34)                                                        │
//  │    VN   (GPIO39)                                                        │
//  │    VP   (GPIO36)                                                        │
//  │    EN                                                                   │
//  └─────────────────────────────────────────────────────────────────────────┘
//
//  Limit switches: normally-open (NO) momentary switches.
//    One terminal → ESP32 GPIO (internal pull-up enabled).
//    Other terminal → GND.
//    Switch activates (GPIO reads LOW) when roof reaches that position.
//
//  Relay: pulse HIGH for 250 ms to toggle the motor controller.
//
//  OLED: I2C 128×32 SSD1306, address 0x3C.
//    SCK/SCL → GPIO19,  SDA → GPIO21  (adjacent pins on the board).
//
//  Web servers:
//    Port  80  — status/control page for browser
//    Port 11111 — ASCOM Alpaca API (dome) + management + setup
//    UDP  32227 — Alpaca discovery
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "wifi_credentials.h"

// ── Pin definitions ───────────────────────────────────────────────────────────
#define LIMIT_SWITCH_CLOSED  32   // INPUT_PULLUP, active LOW
#define LIMIT_SWITCH_OPEN    33   // INPUT_PULLUP, active LOW
#define RELAY_PIN            18   // OUTPUT, pulse HIGH for RELAY_PULSE_MS

#define OLED_SDA             21   // I2C data  (adjacent pair on left rail)
#define OLED_SCL             19   // I2C clock (adjacent pair on left rail)

// ── OLED ──────────────────────────────────────────────────────────────────────
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   32
#define OLED_RESET      -1
#define OLED_I2C_ADDR   0x3C

// ── Timing ────────────────────────────────────────────────────────────────────
#define RELAY_PULSE_MS       250
#define MOVEMENT_TIMEOUT_MS  (120UL * 1000UL)
#define OLED_REFRESH_MS      500

// ── OLED burn-in mitigation ───────────────────────────────────────────────────
// This display runs static 24/7, which is exactly how SSD1306 panels develop
// permanently dim patches.  Three defences:
//   1. Lower drive current — the Adafruit default (0xCF) is far brighter than
//      needed indoors and burns in proportionally faster.
//   2. Nudge the whole layout by a few pixels periodically, so no pixel is lit
//      continuously.  The layout leaves 1 px of vertical slack for this.
//   3. Blank the panel entirely once idle; any state change wakes it.
#define OLED_CONTRAST          0x30      // 0x00-0xFF, was 0xCF
#define OLED_SHIFT_INTERVAL_MS (60UL * 1000UL)
#define OLED_IDLE_BLANK_MS     (30UL * 60UL * 1000UL)   // 0 disables blanking

// ── Watchdogs ─────────────────────────────────────────────────────────────────
// 1. Hardware task watchdog — reboots if loop() stops running (hung handler,
//    deadlock, crashed network stack).  Fed once per loop() iteration.
// 2. WiFi supervisor — polls the link, re-associates, and finally reboots if
//    the association can't be recovered.  This is the case that left the OLED
//    showing "Connecting..." until it was power cycled.
#define WDT_TIMEOUT_S             30
#define WIFI_CHECK_INTERVAL_MS    5000UL   // how often to poll link state
#define WIFI_RECONNECT_WAIT_MS   15000UL   // grace period per reconnect attempt
#define WIFI_MAX_RECONNECT_TRIES  4        // ~60 s down, then reboot
#define BOOT_WIFI_REBEGIN_TRIES   30       // 30 × 500 ms = 15 s, then re-begin
#define BOOT_WIFI_MAX_TRIES       60       // 60 × 500 ms = 30 s, then reboot

// ── Ports ─────────────────────────────────────────────────────────────────────
#define INFO_PORT              80
#define ALPACA_HTTP_PORT    11111
#define ALPACA_DISCOVERY_PORT 32227

// ── Globals ───────────────────────────────────────────────────────────────────
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

WebServer infoServer(INFO_PORT);
WebServer alpacaServer(ALPACA_HTTP_PORT);
WiFiUDP   udpDiscovery;

bool s_oledOk = false;

// ── State ─────────────────────────────────────────────────────────────────────
enum ShutterStatus {
    SHUTTER_OPEN    = 0,
    SHUTTER_CLOSED  = 1,
    SHUTTER_OPENING = 2,
    SHUTTER_CLOSING = 3,
    SHUTTER_ERROR   = 4
};
enum MotionState { MOTION_NONE = 0, MOTION_OPENING, MOTION_CLOSING, MOTION_ERROR };
enum LastCommand  { LAST_CMD_NONE = 0, LAST_CMD_OPEN, LAST_CMD_CLOSE };

MotionState   s_motionState     = MOTION_NONE;
unsigned long s_motionStartMs   = 0;
LastCommand   s_lastCommand     = LAST_CMD_NONE;
bool          s_alpacaConnected = true;
uint32_t      s_serverTxId      = 1;

bool          s_relayActive   = false;
unsigned long s_relayStartMs  = 0;
unsigned long s_lastOledMs    = 0;

// WiFi supervisor state
unsigned long s_lastWifiCheckMs = 0;
unsigned long s_wifiDownSinceMs = 0;   // 0 = link is up
uint8_t       s_wifiRetries     = 0;
uint32_t      s_wifiDropCount   = 0;   // reported on the info page

// OLED burn-in state
unsigned long s_oledActivityMs = 0;    // last time something worth showing happened
unsigned long s_lastShiftMs    = 0;
uint8_t       s_oledShift      = 0;    // bit0 = y offset, bits1-2 = x offset
bool          s_oledBlanked    = false;

// Anything the operator would want to see on the panel calls this.
void oledWake()
{
    s_oledActivityMs = millis();
}

// ── Shutter logic ─────────────────────────────────────────────────────────────
ShutterStatus getShutterStatus()
{
    bool closedActive = (digitalRead(LIMIT_SWITCH_CLOSED) == LOW);
    bool openActive   = (digitalRead(LIMIT_SWITCH_OPEN)   == LOW);

    if (closedActive && openActive) {
        s_motionState = MOTION_ERROR;
        s_lastCommand = LAST_CMD_NONE;
        return SHUTTER_ERROR;
    }
    if (closedActive) {
        s_motionState = MOTION_NONE;
        if (s_lastCommand == LAST_CMD_CLOSE) s_lastCommand = LAST_CMD_NONE;
        return SHUTTER_CLOSED;
    }
    if (openActive) {
        if (s_motionState == MOTION_CLOSING) return SHUTTER_CLOSING;
        s_motionState = MOTION_NONE;
        if (s_lastCommand == LAST_CMD_OPEN) s_lastCommand = LAST_CMD_NONE;
        return SHUTTER_OPEN;
    }

    if (s_motionState == MOTION_OPENING) return SHUTTER_OPENING;
    if (s_motionState == MOTION_CLOSING) return SHUTTER_CLOSING;
    if (s_motionState == MOTION_ERROR)   return SHUTTER_ERROR;
    if (s_lastCommand == LAST_CMD_OPEN)  return SHUTTER_OPENING;
    if (s_lastCommand == LAST_CMD_CLOSE) return SHUTTER_CLOSING;
    return SHUTTER_ERROR;
}

bool isSlewing()
{
    ShutterStatus s = getShutterStatus();
    return s == SHUTTER_OPENING || s == SHUTTER_CLOSING;
}

void triggerRelayPulse()
{
    digitalWrite(RELAY_PIN, HIGH);
    s_relayActive  = true;
    s_relayStartMs = millis();
    oledWake();
}

const char* shutterLabel(ShutterStatus s)
{
    switch (s) {
        case SHUTTER_OPEN:    return "OPEN";
        case SHUTTER_CLOSED:  return "CLOSED";
        case SHUTTER_OPENING: return "OPENING";
        case SHUTTER_CLOSING: return "CLOSING";
        default:              return "ERROR";
    }
}

// ── OLED ──────────────────────────────────────────────────────────────────────
// Layout (128x32 pixels, font size 1 = 6x8 px per character), all offset by
// (dx, dy) so the image drifts slightly over time:
//   y= 0: "Rolloff Roof"
//   y=11: status word inside an outlined box
//   y=23: IP address (or link-down countdown)
// The status box is an outline rather than a filled bar: the old
// fillRect(0,11,128,9) lit 1152 pixels continuously, which is both the main
// burn-in source and a heavy enough load to sag the charge pump.
void updateOLED()
{
    if (!s_oledOk) return;

    // Blank the panel once nothing has happened for a while.
    if (OLED_IDLE_BLANK_MS && (millis() - s_oledActivityMs) > OLED_IDLE_BLANK_MS) {
        if (!s_oledBlanked) {
            display.clearDisplay();
            display.display();
            display.ssd1306_command(SSD1306_DISPLAYOFF);
            s_oledBlanked = true;
        }
        return;
    }
    if (s_oledBlanked) {
        display.ssd1306_command(SSD1306_DISPLAYON);
        s_oledBlanked = false;
    }

    const int16_t dx = (s_oledShift >> 1) & 0x03;   // 0-3 px
    const int16_t dy =  s_oledShift       & 0x01;   // 0-1 px

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    // Row 1 — title
    display.setCursor(dx, dy);
    display.print("Rolloff Roof");

    // Row 2 — status word in an outlined box
    ShutterStatus status = getShutterStatus();
    const char *label = shutterLabel(status);
    int16_t boxW = (int16_t)strlen(label) * 6 + 5;
    display.drawRect(dx, 11 + dy, boxW, 11, SSD1306_WHITE);
    display.setCursor(dx + 3, 13 + dy);
    display.print(label);

    // Row 3 — IP address, or how long the link has been down
    display.setCursor(dx, 23 + dy);
    if (WiFi.status() == WL_CONNECTED) {
        display.print(WiFi.localIP().toString());
    } else if (s_wifiDownSinceMs != 0) {
        display.printf("WiFi lost %lus", (millis() - s_wifiDownSinceMs) / 1000UL);
    } else {
        display.print("Connecting...");
    }

    display.display();
}

// ── Watchdog / recovery ───────────────────────────────────────────────────────
// Reboot, but never while the roof is physically moving — a reset floats the
// relay pin briefly, and we don't want to glitch the motor controller mid-travel.
// Returns having done nothing if motion is in progress; the caller retries.
void safeRestart(const char *reason)
{
    if (s_motionState == MOTION_OPENING || s_motionState == MOTION_CLOSING) {
        Serial.printf("Restart requested (%s) — deferred, roof is moving\n", reason);
        return;
    }

    Serial.printf("Restarting: %s\n", reason);
    if (s_oledOk) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.print("Restarting");
        display.setCursor(0, 12);
        display.print(reason);
        display.display();
    }
    Serial.flush();
    delay(1000);
    ESP.restart();
}

// Polled from loop().  Escalates: notice the drop → re-associate → reboot.
void maintainWifi()
{
    if (millis() - s_lastWifiCheckMs < WIFI_CHECK_INTERVAL_MS) return;
    s_lastWifiCheckMs = millis();

    if (WiFi.status() == WL_CONNECTED) {
        if (s_wifiDownSinceMs != 0) {
            Serial.print("WiFi recovered, IP: ");
            Serial.println(WiFi.localIP());
            oledWake();
        }
        s_wifiDownSinceMs = 0;
        s_wifiRetries     = 0;
        return;
    }

    // First time we've seen the link down — kick off a reconnect immediately.
    if (s_wifiDownSinceMs == 0) {
        s_wifiDownSinceMs = millis();
        s_wifiRetries     = 0;
        s_wifiDropCount++;
        Serial.println("WiFi link lost — reconnecting");
        oledWake();   // a drop is worth lighting the panel for
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        return;
    }

    // Give the current attempt its full grace period before doing anything else.
    if (millis() - s_wifiDownSinceMs < WIFI_RECONNECT_WAIT_MS) return;

    if (s_wifiRetries >= WIFI_MAX_RECONNECT_TRIES) {
        safeRestart("WiFi down");
        return;   // only returns here if the roof is moving — retry next poll
    }

    s_wifiRetries++;
    Serial.printf("WiFi reconnect attempt %u/%u\n", s_wifiRetries, WIFI_MAX_RECONNECT_TRIES);
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    s_wifiDownSinceMs = millis();
}

// ── Alpaca response helpers ───────────────────────────────────────────────────
void addCorsHeaders(WebServer &srv)
{
    srv.sendHeader("Access-Control-Allow-Origin",  "*");
    srv.sendHeader("Access-Control-Allow-Methods", "GET, PUT, OPTIONS");
    srv.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void sendAlpacaRaw(WebServer &srv, const String &valueJson,
                   uint32_t clientTx, uint32_t errorNum, const String &errorMsg)
{
    uint32_t serverTx = s_serverTxId++;
    String resp =
        "{\"ClientTransactionID\":" + String(clientTx) +
        ",\"ServerTransactionID\":"  + String(serverTx) +
        ",\"ErrorNumber\":"          + String(errorNum) +
        ",\"ErrorMessage\":\""       + errorMsg + "\"," + valueJson + "}";
    addCorsHeaders(srv);
    srv.send(200, "application/json", resp);
}

void sendAlpacaBool(WebServer &srv, bool v, uint32_t clientTx)
{
    sendAlpacaRaw(srv, String("\"Value\":") + (v ? "true" : "false"), clientTx, 0, "");
}

void sendAlpacaInt(WebServer &srv, int v, uint32_t clientTx)
{
    sendAlpacaRaw(srv, "\"Value\":" + String(v), clientTx, 0, "");
}

void sendAlpacaString(WebServer &srv, const String &v, uint32_t clientTx)
{
    sendAlpacaRaw(srv, "\"Value\":\"" + v + "\"", clientTx, 0, "");
}

void sendAlpacaArray(WebServer &srv, const String &json, uint32_t clientTx)
{
    sendAlpacaRaw(srv, "\"Value\":" + json, clientTx, 0, "");
}

void sendAlpacaError(WebServer &srv, uint32_t clientTx, uint32_t errNum, const String &msg)
{
    sendAlpacaRaw(srv, "\"Value\":null", clientTx, errNum, msg);
}

uint32_t alpacaClientTx()
{
    String v = alpacaServer.arg("ClientTransactionID");
    return v.length() ? (uint32_t)v.toInt() : 0;
}

// ── Dome GET ──────────────────────────────────────────────────────────────────
void handleDomeGet()
{
    uint32_t clientTx = alpacaClientTx();
    String ep = alpacaServer.uri().substring(String("/api/v1/dome/0/").length());

    if (ep == "connected")        return sendAlpacaBool  (alpacaServer, s_alpacaConnected, clientTx);
    if (ep == "description")      return sendAlpacaString(alpacaServer, "Rolloff roof Alpaca dome driver", clientTx);
    if (ep == "driverinfo")       return sendAlpacaString(alpacaServer, "ESP32 Alpaca Dome Driver", clientTx);
    if (ep == "driverversion")    return sendAlpacaString(alpacaServer, "1.0", clientTx);
    if (ep == "interfaceversion") return sendAlpacaInt   (alpacaServer, 3, clientTx);
    if (ep == "name")             return sendAlpacaString(alpacaServer, "Rolloff Roof", clientTx);
    if (ep == "supportedactions") return sendAlpacaArray (alpacaServer, "[]", clientTx);

    if (ep == "cansetshutter")    return sendAlpacaBool(alpacaServer, true,  clientTx);
    if (ep == "canfindhome")      return sendAlpacaBool(alpacaServer, false, clientTx);
    if (ep == "canpark")          return sendAlpacaBool(alpacaServer, false, clientTx);
    if (ep == "cansetaltitude")   return sendAlpacaBool(alpacaServer, false, clientTx);
    if (ep == "cansetazimuth")    return sendAlpacaBool(alpacaServer, false, clientTx);
    if (ep == "cansetpark")       return sendAlpacaBool(alpacaServer, false, clientTx);
    if (ep == "canslave")         return sendAlpacaBool(alpacaServer, false, clientTx);
    if (ep == "cansyncazimuth")   return sendAlpacaBool(alpacaServer, false, clientTx);

    if (ep == "shutterstatus")    return sendAlpacaInt (alpacaServer, (int)getShutterStatus(), clientTx);
    if (ep == "slewing")          return sendAlpacaBool(alpacaServer, isSlewing(), clientTx);
    if (ep == "slaved")           return sendAlpacaBool(alpacaServer, false, clientTx);

    if (ep == "athome" || ep == "atpark")
        return sendAlpacaError(alpacaServer, clientTx, 0x400, "PropertyNotImplementedException");
    if (ep == "azimuth" || ep == "altitude")
        return sendAlpacaError(alpacaServer, clientTx, 0x400, "PropertyNotImplementedException");

    sendAlpacaError(alpacaServer, clientTx, 0x401, "InvalidValueException");
}

// ── Dome PUT ──────────────────────────────────────────────────────────────────
void handleDomePut()
{
    uint32_t clientTx = alpacaClientTx();
    String ep = alpacaServer.uri().substring(String("/api/v1/dome/0/").length());

    if (ep == "connected") {
        String val = alpacaServer.arg("Connected");
        if (val == "true"  || val == "True"  || val == "1") s_alpacaConnected = true;
        if (val == "false" || val == "False" || val == "0") s_alpacaConnected = false;
        return sendAlpacaBool(alpacaServer, s_alpacaConnected, clientTx);
    }
    if (ep == "openshutter") {
        if (getShutterStatus() != SHUTTER_OPEN) {
            s_lastCommand   = LAST_CMD_OPEN;
            s_motionState   = MOTION_OPENING;
            s_motionStartMs = millis();
            triggerRelayPulse();
        }
        return sendAlpacaRaw(alpacaServer, "\"Value\":null", clientTx, 0, "");
    }
    if (ep == "closeshutter") {
        if (getShutterStatus() != SHUTTER_CLOSED) {
            s_lastCommand   = LAST_CMD_CLOSE;
            s_motionState   = MOTION_CLOSING;
            s_motionStartMs = millis();
            triggerRelayPulse();
        }
        return sendAlpacaRaw(alpacaServer, "\"Value\":null", clientTx, 0, "");
    }
    if (ep == "abortslew") {
        triggerRelayPulse();
        s_motionState = MOTION_ERROR;
        return sendAlpacaRaw(alpacaServer, "\"Value\":null", clientTx, 0, "");
    }

    if (ep == "action"        || ep == "commandblind" ||
        ep == "commandbool"   || ep == "commandstring")
        return sendAlpacaError(alpacaServer, clientTx, 0x400, "MethodNotImplementedException");

    if (ep == "findhome"       || ep == "park"             || ep == "setpark"  ||
        ep == "slewtoaltitude" || ep == "slewtoazimuth"    || ep == "synctoazimuth" ||
        ep == "slaved")
        return sendAlpacaError(alpacaServer, clientTx, 0x400, "MethodNotImplementedException");

    sendAlpacaError(alpacaServer, clientTx, 0x401, "InvalidValueException");
}

// ── Management ────────────────────────────────────────────────────────────────
void handleManagement()
{
    uint32_t clientTx = alpacaClientTx();
    String path = alpacaServer.uri();

    if (path == "/management/apiversions")
        return sendAlpacaArray(alpacaServer, "[1]", clientTx);

    if (path == "/management/v1/description") {
        String desc =
            "{\"ServerName\":\"ESP32 Alpaca\",\"Manufacturer\":\"Rolloff\","
            "\"ManufacturerVersion\":\"1.0\",\"Location\":\"Observatory\"}";
        return sendAlpacaRaw(alpacaServer, "\"Value\":" + desc, clientTx, 0, "");
    }
    if (path == "/management/v1/configureddevices") {
        String devices =
            "[{\"DeviceType\":\"Dome\",\"DeviceName\":\"Rolloff Roof\","
            "\"DeviceNumber\":0,\"UniqueID\":\"ESP32-ROLLOFF-0\"}]";
        return sendAlpacaArray(alpacaServer, devices, clientTx);
    }

    addCorsHeaders(alpacaServer);
    alpacaServer.send(404, "text/plain", "Not found");
}

// ── Alpaca dome route handler (GET + PUT + OPTIONS) ──────────────────────────
// Registered explicitly for every /api/v1/dome/0/* path so WebServer always
// has a non-null _currentHandler and the "request handler not found" log_e
// warning is never triggered.
void handleDomeAny()
{
    HTTPMethod m = alpacaServer.method();
    if (m == HTTP_OPTIONS) { addCorsHeaders(alpacaServer); alpacaServer.send(200); return; }
    if (m == HTTP_GET)     { handleDomeGet(); return; }
    if (m == HTTP_PUT)     { handleDomePut(); return; }
    alpacaServer.send(405, "text/plain", "Method Not Allowed");
}

// ── Info page (port 80) ───────────────────────────────────────────────────────
// The Open/Close buttons use window.location.hostname to build the URL so no
// IP is hardcoded.  The PUT goes cross-origin to port 11111; CORS is enabled
// on the Alpaca server so the browser preflight succeeds.
void handleInfoRoot()
{
    oledWake();   // someone is looking at the device

    bool closedActive = (digitalRead(LIMIT_SWITCH_CLOSED) == LOW);
    bool openActive   = (digitalRead(LIMIT_SWITCH_OPEN)   == LOW);
    ShutterStatus status = getShutterStatus();

    String closedColor = closedActive ? "#dc3545" : "#28a745";
    String openColor   = openActive   ? "#dc3545" : "#28a745";

    String html =
        "<html><head>"
        "<meta http-equiv='refresh' content='3'>"
        "<style>"
        "body{font-family:Arial,sans-serif;margin:40px;background:#f5f5f5}"
        "h1{color:#333}"
        ".card{padding:16px;margin:12px 0;border-radius:8px;background:#fff;"
              "box-shadow:0 2px 6px rgba(0,0,0,.1)}"
        ".btn{padding:10px 16px;margin-right:8px;border:none;border-radius:6px;"
             "cursor:pointer;font-size:1em}"
        ".open{background:#28a745;color:#fff}"
        ".close{background:#dc3545;color:#fff}"
        "</style>"
        "</head><body>"
        "<h1>Rolloff Roof Controller</h1>"
        "<div class='card'>"
        "<h2>Status</h2>"
        "<p><strong>Shutter:</strong> " + String(shutterLabel(status)) + "</p>"
        "<p><strong>Alpaca Connected:</strong> " + (s_alpacaConnected ? "Yes" : "No") + "</p>"
        "<p><strong>Closed switch (GPIO 32):</strong> "
            "<span style='color:" + closedColor + ";'>"
            + (closedActive ? "ACTIVE" : "inactive") + "</span></p>"
        "<p><strong>Open switch (GPIO 33):</strong> "
            "<span style='color:" + openColor + ";'>"
            + (openActive ? "ACTIVE" : "inactive") + "</span></p>"
        "</div>"
        "<div class='card'>"
        "<h2>Link</h2>"
        "<p><strong>Uptime:</strong> " + String(millis() / 60000UL) + " min</p>"
        "<p><strong>WiFi RSSI:</strong> " + String(WiFi.RSSI()) + " dBm</p>"
        "<p><strong>WiFi drops since boot:</strong> " + String(s_wifiDropCount) + "</p>"
        "<p><strong>Free heap:</strong> " + String(ESP.getFreeHeap()) + " bytes</p>"
        "</div>"
        "<div class='card'>"
        "<h2>Controls</h2>"
        "<button class='btn open'  onclick='sendCmd(\"openshutter\")'>Open Roof</button>"
        "<button class='btn close' onclick='sendCmd(\"closeshutter\")'>Close Roof</button>"
        "</div>"
        "<div class='card'>"
        "<h2>Alpaca</h2>"
        "<p>API base: <code>http://&lt;IP&gt;:" + String(ALPACA_HTTP_PORT) + "/api/v1/dome/0/</code></p>"
        "<p>Discovery UDP port: " + String(ALPACA_DISCOVERY_PORT) + "</p>"
        "</div>"
        "<script>"
        "function sendCmd(cmd){"
          "var url='http://'+window.location.hostname+':" + String(ALPACA_HTTP_PORT) + "/api/v1/dome/0/'+cmd;"
          "fetch(url,{method:'PUT',"
            "headers:{'Content-Type':'application/x-www-form-urlencoded'},"
            "body:'ClientID=1&ClientTransactionID='+Date.now()})"
          ".then(()=>location.reload())"
          ".catch(e=>alert('Error: '+e));"
        "}"
        "</script>"
        "</body></html>";

    infoServer.send(200, "text/html", html);
}

// ── Alpaca UDP discovery ──────────────────────────────────────────────────────
void handleDiscovery()
{
    int pktLen = udpDiscovery.parsePacket();
    if (pktLen <= 0) return;

    char buf[64];
    int n = udpDiscovery.read(buf, sizeof(buf) - 1);
    if (n <= 0) return;
    buf[n] = '\0';

    if (strncmp(buf, "alpacadiscovery1", 16) == 0) {
        char resp[64];
        snprintf(resp, sizeof(resp), "{\"AlpacaPort\":%d}", ALPACA_HTTP_PORT);
        udpDiscovery.beginPacket(udpDiscovery.remoteIP(), udpDiscovery.remotePort());
        udpDiscovery.print(resp);
        udpDiscovery.endPacket();
    }
}

// ── setup() ───────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    Serial.println("\n\nESP32 Rolloff Alpaca Driver starting...");

    // GPIO
    pinMode(LIMIT_SWITCH_CLOSED, INPUT_PULLUP);
    pinMode(LIMIT_SWITCH_OPEN,   INPUT_PULLUP);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);   // relay off at boot

    Serial.printf("Limit switches: GPIO%d (closed), GPIO%d (open)\n",
                  LIMIT_SWITCH_CLOSED, LIMIT_SWITCH_OPEN);
    Serial.printf("Relay: GPIO%d\n", RELAY_PIN);

    // OLED (I2C on GPIO19=SCL, GPIO21=SDA)
    Wire.begin(OLED_SDA, OLED_SCL);

    // Scan I2C bus to find the OLED address.
    // Note: Adafruit SSD1306 begin() returns true even when no device is present
    // (it only checks RAM allocation), so we use the scan as the real indicator.
    Serial.print("I2C scan: ");
    uint8_t oledAddr = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("0x%02X ", addr);
            if ((addr == 0x3C || addr == 0x3D) && oledAddr == 0) oledAddr = addr;
        }
    }
    Serial.println(oledAddr ? "" : "no devices found — check SDA→D21 SCL→D19 wiring");

    // Only initialise the display if a known SSD1306 address was actually found
    if (oledAddr) {
        s_oledOk = display.begin(SSD1306_SWITCHCAPVCC, oledAddr);
    }
    if (s_oledOk) {
        // Turn the drive current down before anything is shown — the library
        // default (0xCF) is needlessly bright and accelerates burn-in.
        display.ssd1306_command(SSD1306_SETCONTRAST);
        display.ssd1306_command(OLED_CONTRAST);

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.print("Rolloff Roof");
        display.setCursor(0, 12);
        display.print("Connecting...");
        display.display();
        Serial.println("OLED OK");
    } else {
        Serial.println("OLED init failed - continuing without display");
    }

    // WiFi — re-begin at 15 s, restart at 30 s
    WiFi.persistent(false);        // don't wear out flash rewriting the same creds
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);          // modem sleep makes the link flaky under polling
    WiFi.setAutoReconnect(true);   // first line of defence; maintainWifi() backs it up
    WiFi.setHostname("rolloff");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    {
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
            ++attempts;

            // Half way — tear the association down and start over.  Cheaper than
            // a reboot and clears a stuck WPA handshake.
            if (attempts == BOOT_WIFI_REBEGIN_TRIES) {
                Serial.print(" retrying");
                WiFi.disconnect(true);
                delay(200);
                WiFi.mode(WIFI_STA);
                WiFi.begin(WIFI_SSID, WIFI_PASS);
            }

            if (attempts >= BOOT_WIFI_MAX_TRIES) {
                Serial.println("\nWiFi failed — restarting in 3 s");
                if (s_oledOk) {
                    display.clearDisplay();
                    display.setTextSize(1);
                    display.setTextColor(SSD1306_WHITE);
                    display.setCursor(0, 0);
                    display.print("WiFi failed!");
                    display.setCursor(0, 12);
                    display.print(WIFI_SSID);
                    display.setCursor(0, 24);
                    display.print("Restarting...");
                    display.display();
                }
                delay(3000);
                ESP.restart();
            }
        }
    }
    Serial.println();
    Serial.print("Connected!  IP: ");
    Serial.println(WiFi.localIP());

    // Info server — port 80
    infoServer.on("/", HTTP_GET, handleInfoRoot);
    infoServer.on("/favicon.ico", HTTP_GET, []() { infoServer.send(204); });
    infoServer.onNotFound([]() { infoServer.send(404, "text/plain", "Not found"); });
    infoServer.begin();
    Serial.printf("Info server on port %d\n", INFO_PORT);

    // Alpaca server — port 11111
    // Register every known endpoint explicitly so WebServer's _currentHandler is
    // always non-null.  onNotFound is only reached for genuinely unknown paths.
    static const char* domeEps[] = {
        "/api/v1/dome/0/connected",        "/api/v1/dome/0/description",
        "/api/v1/dome/0/driverinfo",       "/api/v1/dome/0/driverversion",
        "/api/v1/dome/0/interfaceversion", "/api/v1/dome/0/name",
        "/api/v1/dome/0/supportedactions", "/api/v1/dome/0/cansetshutter",
        "/api/v1/dome/0/canfindhome",      "/api/v1/dome/0/canpark",
        "/api/v1/dome/0/cansetaltitude",   "/api/v1/dome/0/cansetazimuth",
        "/api/v1/dome/0/cansetpark",       "/api/v1/dome/0/canslave",
        "/api/v1/dome/0/cansyncazimuth",   "/api/v1/dome/0/shutterstatus",
        "/api/v1/dome/0/slewing",          "/api/v1/dome/0/slaved",
        "/api/v1/dome/0/athome",           "/api/v1/dome/0/atpark",
        "/api/v1/dome/0/azimuth",          "/api/v1/dome/0/altitude",
        "/api/v1/dome/0/openshutter",      "/api/v1/dome/0/closeshutter",
        "/api/v1/dome/0/abortslew",        "/api/v1/dome/0/action",
        "/api/v1/dome/0/commandblind",     "/api/v1/dome/0/commandbool",
        "/api/v1/dome/0/commandstring",    "/api/v1/dome/0/findhome",
        "/api/v1/dome/0/park",             "/api/v1/dome/0/setpark",
        "/api/v1/dome/0/slewtoaltitude",   "/api/v1/dome/0/slewtoazimuth",
        "/api/v1/dome/0/synctoazimuth",    nullptr
    };
    for (int i = 0; domeEps[i]; i++) {
        alpacaServer.on(domeEps[i], HTTP_ANY, handleDomeAny);
    }
    alpacaServer.on("/management/apiversions",          HTTP_ANY, handleManagement);
    alpacaServer.on("/management/v1/description",       HTTP_ANY, handleManagement);
    alpacaServer.on("/management/v1/configureddevices", HTTP_ANY, handleManagement);
    alpacaServer.on("/setup/v1/dome/0/setup", HTTP_ANY, []() {
        addCorsHeaders(alpacaServer);
        alpacaServer.send(200, "text/html",
            "<html><body><h1>Alpaca Setup</h1>"
            "<p>This device uses fixed configuration in firmware.</p>"
            "</body></html>");
    });
    alpacaServer.on("/favicon.ico", HTTP_ANY, []() { alpacaServer.send(204); });
    alpacaServer.onNotFound([]() {
        addCorsHeaders(alpacaServer);
        alpacaServer.send(404, "text/plain", "Not found");
    });
    alpacaServer.begin();
    Serial.printf("Alpaca server on port %d\n", ALPACA_HTTP_PORT);

    // Alpaca UDP discovery
    udpDiscovery.begin(ALPACA_DISCOVERY_PORT);
    Serial.printf("Alpaca discovery on UDP port %d\n", ALPACA_DISCOVERY_PORT);

    // Hardware task watchdog — armed last so a slow setup() can't trip it.
    // If loop() stops feeding it for WDT_TIMEOUT_S the chip resets.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t wdtCfg = {
        .timeout_ms     = WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true
    };
    // Core 3.x may already have initialised the TWDT — reconfigure in that case.
    if (esp_task_wdt_init(&wdtCfg) == ESP_ERR_INVALID_STATE) {
        esp_task_wdt_reconfigure(&wdtCfg);
    }
#else
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
    esp_task_wdt_add(NULL);   // watch the Arduino loop task
    Serial.printf("Task watchdog armed (%d s)\n", WDT_TIMEOUT_S);

    oledWake();
    updateOLED();
}

// ── loop() ────────────────────────────────────────────────────────────────────
void loop()
{
    esp_task_wdt_reset();   // feed the hardware watchdog

    // Non-blocking relay pulse — turn off after RELAY_PULSE_MS
    if (s_relayActive && (millis() - s_relayStartMs >= RELAY_PULSE_MS)) {
        digitalWrite(RELAY_PIN, LOW);
        s_relayActive = false;
    }

    // Movement timeout watchdog
    if ((s_motionState == MOTION_OPENING || s_motionState == MOTION_CLOSING) &&
        s_motionStartMs != 0 &&
        (millis() - s_motionStartMs) > MOVEMENT_TIMEOUT_MS) {
        s_motionState = MOTION_ERROR;
    }

    maintainWifi();

    infoServer.handleClient();
    alpacaServer.handleClient();
    handleDiscovery();

    // Wake the panel whenever the roof actually changes state
    {
        static ShutterStatus lastShutter = SHUTTER_ERROR;
        static bool          haveLast    = false;
        ShutterStatus now = getShutterStatus();
        if (!haveLast || now != lastShutter) {
            if (haveLast) oledWake();
            lastShutter = now;
            haveLast    = true;
        }
    }

    // Drift the layout a few pixels so nothing stays lit forever
    if (millis() - s_lastShiftMs >= OLED_SHIFT_INTERVAL_MS) {
        s_oledShift  = (s_oledShift + 1) & 0x07;
        s_lastShiftMs = millis();
    }

    // Refresh OLED every 500 ms
    if (millis() - s_lastOledMs >= OLED_REFRESH_MS) {
        updateOLED();
        s_lastOledMs = millis();
    }
}
