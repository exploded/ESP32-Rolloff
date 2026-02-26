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
// Layout (128x32 pixels, font size 1 = 6x8 px per character):
//   y= 0: "Rolloff Roof"
//   y=12: status word on inverted (white) background bar
//   y=24: IP address (or "Connecting...")
void updateOLED()
{
    if (!s_oledOk) return;

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    // Row 1 — title
    display.setCursor(0, 0);
    display.print("Rolloff Roof");

    // Row 2 — status on a highlighted bar
    ShutterStatus status = getShutterStatus();
    display.fillRect(0, 11, 128, 9, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(2, 12);
    display.print(shutterLabel(status));
    display.setTextColor(SSD1306_WHITE);

    // Row 3 — IP address
    display.setCursor(0, 24);
    if (WiFi.status() == WL_CONNECTED) {
        display.print(WiFi.localIP().toString());
    } else {
        display.print("Connecting...");
    }

    display.display();
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

    // WiFi — 30 s timeout, then restart and try again
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    {
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
            if (++attempts >= 60) {          // 60 × 500 ms = 30 s
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

    updateOLED();
}

// ── loop() ────────────────────────────────────────────────────────────────────
void loop()
{
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

    infoServer.handleClient();
    alpacaServer.handleClient();
    handleDiscovery();

    // Refresh OLED every 500 ms
    if (millis() - s_lastOledMs >= OLED_REFRESH_MS) {
        updateOLED();
        s_lastOledMs = millis();
    }
}
