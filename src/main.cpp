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
#include <esp_http_server.h>   // Alpaca port only — see startAlpacaServer()
#include <AsyncUDP.h>   // in the ESP32 core; unrelated to AsyncTCP, no lib_deps
#include <Wire.h>
#include <time.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
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

// ── HTTP instrumentation ──────────────────────────────────────────────────────
#define HEARTBEAT_MS       (30UL * 1000UL)        // serial status line
#define REQ_WINDOW_MS      (60UL * 1000UL)        // rolling request-rate window
#define NET_STALL_MS       (10UL * 60UL * 1000UL) // log-only: no requests at all

// ── Time (house convention: Melbourne, DST handled by configTzTime) ───────────
static const char *TZ_INFO      = "AEST-10AEDT,M10.1.0/2,M4.1.0/3";
static const char *NTP_SERVER_1 = "pool.ntp.org";
static const char *NTP_SERVER_2 = "time.nist.gov";
#define TIME_SYNCED_EPOCH  1704067200UL   // 2024-01-01; anything below = not synced

// ── Ports ─────────────────────────────────────────────────────────────────────
#define INFO_PORT              80
#define ALPACA_HTTP_PORT    11111
#define ALPACA_DISCOVERY_PORT 32227

// ── Globals ───────────────────────────────────────────────────────────────────
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// WiFiServer::begin() returns void and fails silently: on a failed socket/bind/
// listen it leaves _listening false and available() then returns nothing for
// ever, leaving the device pingable but permanently unable to serve.  WebServer
// keeps its WiFiServer protected, so expose its liveness rather than guess.
class GuardedWebServer : public WebServer {
public:
    using WebServer::WebServer;
    bool isListening() { return (bool)_server; }
};

GuardedWebServer infoServer(INFO_PORT);
AsyncUDP         udpDiscovery;

// The Alpaca API does not use WebServer — see startAlpacaServer() for why.
// Null means "not serving", which is the same liveness question isListening()
// answers for port 80.
httpd_handle_t   s_alpacaHttpd = nullptr;

void startDiscovery();      // defined below; both are re-armed by the WiFi
void startAlpacaServer();   // supervisor when the link comes back

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

// The Alpaca HTTP handlers run on esp_http_server's own FreeRTOS task, so these
// are read from a different task than the one that writes them.  Only loop()
// ever writes the motion state; the HTTP side reads it and queues commands (see
// s_pendingCmd).  Each is a single aligned word, so a reader sees either the old
// or the new value, never a torn one.  getShutterStatus() does read the motion
// state and the last command as a pair without locking, so it can briefly
// report a status derived from a half-updated pair — self-correcting on the
// next poll, and never used to make a movement decision.
volatile MotionState   s_motionState     = MOTION_NONE;
unsigned long          s_motionStartMs   = 0;   // loop() only
volatile LastCommand   s_lastCommand     = LAST_CMD_NONE;
volatile bool          s_alpacaConnected = true;
uint32_t               s_serverTxId      = 1;   // bumped atomically

// Roof commands arriving over Alpaca are queued here rather than executed on the
// HTTP task.  loop() stays the sole mutator of motion state, the relay and the
// diagnostics ring — the same discipline the WiFi event handler already follows.
// One slot, last-write-wins: two commands inside a single loop() iteration are
// under a millisecond apart, and coalescing them is both correct for a roof that
// takes two minutes to travel and safer than pulsing a toggle relay twice.
enum PendingCmd : uint8_t { PEND_NONE = 0, PEND_OPEN, PEND_CLOSE, PEND_ABORT };
volatile uint8_t s_pendingCmd = PEND_NONE;

bool          s_relayActive   = false;
unsigned long s_relayStartMs  = 0;
unsigned long s_lastOledMs    = 0;

// WiFi supervisor state
unsigned long s_lastWifiCheckMs = 0;
unsigned long s_wifiDownSinceMs = 0;   // 0 = link is up
uint8_t       s_wifiRetries     = 0;
uint32_t      s_wifiDropCount   = 0;   // reported on the info page

// Set from the WiFi event task, consumed in loop().  The handler must stay this
// trivial: WiFi.begin()/disconnect() must not be called from the event context,
// so all it does is raise a flag that forces the next poll to run immediately.
volatile bool    s_wifiEvtDisconnected = false;
volatile bool    s_wifiEvtGotIp        = false;
volatile uint8_t s_wifiDisconnectReason = 0;

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            s_wifiDisconnectReason = info.wifi_sta_disconnected.reason;
            s_wifiEvtDisconnected  = true;
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            s_wifiEvtGotIp = true;
            break;
        default:
            break;
    }
}

// ── Diagnostics ───────────────────────────────────────────────────────────────
// Lives in RTC memory, which survives a software reset and a watchdog reset but
// not a power cycle.  That's exactly what we want: if the device reboots itself
// overnight the evidence is still there in the morning.  A power cycle
// deliberately clears it, so the counters always mean "since mains was applied".
// Bump the magic whenever the layout below changes, otherwise a reflash can find
// stale RTC contents that still match the old magic and decode as garbage.
#define DIAG_MAGIC   0x524F4C47UL   // "ROLG" — rev 2, DiagEntry gained `detail`
#define DIAG_EVENTS  12

// Appended to only — existing values must keep their numbers so an RTC record
// written by an older build still decodes.  Adding a name does not change
// DiagEntry's layout, so DIAG_MAGIC does not need bumping for this.
enum DiagEvent : uint8_t {
    EV_NONE = 0, EV_BOOT, EV_WIFI_LOST, EV_WIFI_OK, EV_REBOOT, EV_OPEN, EV_CLOSE,
    EV_ABORT, EV_NET_STALL
};

struct DiagEntry {
    uint32_t epoch;      // 0 if the clock wasn't synced yet
    uint32_t uptimeSec;
    uint8_t  type;
    uint8_t  detail;     // EV_WIFI_LOST: the 802.11 disconnect reason code
};

struct DiagRecord {
    uint32_t  magic;
    uint32_t  bootCount;
    uint32_t  wifiDrops;   // cumulative across reboots, unlike s_wifiDropCount
    uint8_t   head;
    DiagEntry ev[DIAG_EVENTS];
};

RTC_NOINIT_ATTR DiagRecord s_diag;

const char *diagEventName(uint8_t t)
{
    switch (t) {
        case EV_BOOT:      return "boot";
        case EV_WIFI_LOST: return "WiFi lost";
        case EV_WIFI_OK:   return "WiFi recovered";
        case EV_REBOOT:    return "self-reboot";
        case EV_OPEN:      return "open cmd";
        case EV_CLOSE:     return "close cmd";
        case EV_ABORT:     return "abort cmd";
        case EV_NET_STALL: return "no requests";
        default:           return "";
    }
}

const char *resetReasonName()
{
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:  return "power-on";
        case ESP_RST_SW:       return "software restart";
        case ESP_RST_PANIC:    return "panic / exception";
        case ESP_RST_TASK_WDT: return "TASK WATCHDOG";
        case ESP_RST_INT_WDT:  return "interrupt watchdog";
        case ESP_RST_WDT:      return "other watchdog";
        case ESP_RST_BROWNOUT: return "BROWNOUT (power dip)";
        case ESP_RST_EXT:      return "external reset pin";
        case ESP_RST_DEEPSLEEP:return "deep sleep wake";
        default:               return "unknown";
    }
}

void diagLog(DiagEvent type, uint8_t detail = 0)
{
    DiagEntry &e = s_diag.ev[s_diag.head];
    time_t now   = time(nullptr);
    e.epoch      = (now > (time_t)TIME_SYNCED_EPOCH) ? (uint32_t)now : 0;
    e.uptimeSec  = millis() / 1000UL;
    e.type       = (uint8_t)type;
    e.detail     = detail;
    s_diag.head  = (s_diag.head + 1) % DIAG_EVENTS;
}

// The common 802.11 reason codes, so the log reads without a lookup table.
// 15 in particular means the AP's group-key rotation timed out — a classic
// cause of a single unexplained nightly drop.
const char *wifiReasonName(uint8_t r)
{
    switch (r) {
        case 1:   return "unspecified";
        case 2:   return "auth expired";
        case 4:   return "assoc expired (AP idle timeout)";
        case 8:   return "AP deauthenticated us";
        case 15:  return "4-way handshake timeout (group rekey)";
        case 200: return "beacon timeout (AP unreachable)";
        case 201: return "no AP found";
        case 202: return "auth failed";
        case 203: return "assoc failed";
        default:  return "";
    }
}

// Local wall-clock string, or an uptime fallback when NTP hadn't synced yet.
String diagWhen(const DiagEntry &e)
{
    if (e.epoch == 0) return "+" + String(e.uptimeSec) + "s (no clock)";
    time_t t = (time_t)e.epoch;
    struct tm tmv;
    localtime_r(&t, &tmv);
    char buf[24];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmv);
    return String(buf);
}

// Diagnostics card for the info page — newest event first.
String diagCardHtml()
{
    String h = "<div class='card'><h2>Diagnostics</h2>"
               "<p><strong>Last reset:</strong> " + String(resetReasonName()) + "</p>"
               "<p><strong>Boots since power-on:</strong> " + String(s_diag.bootCount) + "</p>"
               "<p><strong>WiFi drops since power-on:</strong> " + String(s_diag.wifiDrops) + "</p>";

    time_t now = time(nullptr);
    h += "<p><strong>Clock:</strong> ";
    if (now > (time_t)TIME_SYNCED_EPOCH) {
        struct tm tmv;
        localtime_r(&now, &tmv);
        char buf[24];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmv);
        h += buf;
    } else {
        h += "not synced";
    }
    h += "</p>";

    h += "<table><tr><th align='left'>When</th><th align='left'>Event</th></tr>";
    bool any = false;
    for (int i = 0; i < DIAG_EVENTS; i++) {
        // walk backwards from the most recently written slot
        int idx = (s_diag.head - 1 - i + 2 * DIAG_EVENTS) % DIAG_EVENTS;
        const DiagEntry &e = s_diag.ev[idx];
        if (e.type == EV_NONE) continue;
        any = true;
        String what = diagEventName(e.type);
        if (e.type == EV_WIFI_LOST) {
            what += " — reason " + String(e.detail);
            const char *rn = wifiReasonName(e.detail);
            if (rn[0]) what += " (" + String(rn) + ")";
        }
        h += "<tr><td><code>" + diagWhen(e) + "</code></td><td>" + what + "</td></tr>";
    }
    if (!any) h += "<tr><td colspan='2'>no events recorded</td></tr>";
    h += "</table></div>";
    return h;
}

// ── HTTP serving instrumentation ──────────────────────────────────────────────
// The failure being chased is TCP-level: requests never reach a handler at all.
// So the fingerprint is both servers' request counts going flat together while
// uptime and RSSI keep advancing.  Counters are RAM-only — the RTC log carries
// the events that need to survive a reboot.
uint32_t      s_infoRequests    = 0;
uint32_t      s_alpacaRequests  = 0;
unsigned long s_lastRequestMs   = 0;
uint32_t      s_recentRequests  = 0;   // completed 60 s window
uint32_t      s_windowCount     = 0;   // window in progress
unsigned long s_windowStartMs   = 0;
unsigned long s_lastHeartbeatMs = 0;
volatile bool s_stallLogged     = false;

// TCP connections accepted on the Alpaca port, counted from inside the server.
// This is the number that says whether the persistent-connection change is
// actually working: requests per connection = Δs_alpacaRequests / Δs_alpacaConns.
// One-per-request means something is still forcing a close; an observing session
// should show hundreds.
uint32_t      s_alpacaConns     = 0;

// noteRequest() is now called from two tasks (loop() for port 80, the httpd task
// for port 11111), so the increments have to be atomic or counts get lost.  The
// timestamp and the flag are single words and need no more than that.
static inline void bumpCounter(uint32_t &v) { __atomic_fetch_add(&v, 1, __ATOMIC_RELAXED); }

void noteRequest(bool alpaca)
{
    bumpCounter(alpaca ? s_alpacaRequests : s_infoRequests);
    s_lastRequestMs = millis();
    bumpCounter(s_windowCount);
    s_stallLogged   = false;   // re-arm the stall detector
}

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
// getShutterStatus() is a PURE read — it must never mutate motion state.  It is
// called from HTTP handlers, from updateOLED() and from loop(), and safeRestart()
// reads the same motion state to decide whether a reboot is safe.  When this
// function still mutated, a GET on /slewing could change whether the device was
// allowed to reboot.  All mutation now lives in updateMotionState(), called once
// per loop() so the state can't shift underneath a reader.
ShutterStatus getShutterStatus()
{
    bool closedActive = (digitalRead(LIMIT_SWITCH_CLOSED) == LOW);
    bool openActive   = (digitalRead(LIMIT_SWITCH_OPEN)   == LOW);

    if (closedActive && openActive) return SHUTTER_ERROR;
    if (closedActive)               return SHUTTER_CLOSED;
    if (openActive) {
        // Sitting on the open switch but commanded closed: still travelling.
        if (s_motionState == MOTION_CLOSING) return SHUTTER_CLOSING;
        return SHUTTER_OPEN;
    }

    if (s_motionState == MOTION_OPENING) return SHUTTER_OPENING;
    if (s_motionState == MOTION_CLOSING) return SHUTTER_CLOSING;
    if (s_motionState == MOTION_ERROR)   return SHUTTER_ERROR;
    if (s_lastCommand == LAST_CMD_OPEN)  return SHUTTER_OPENING;
    if (s_lastCommand == LAST_CMD_CLOSE) return SHUTTER_CLOSING;
    return SHUTTER_ERROR;
}

// The mutations that used to live inside getShutterStatus().  loop() only.
void updateMotionState()
{
    bool closedActive = (digitalRead(LIMIT_SWITCH_CLOSED) == LOW);
    bool openActive   = (digitalRead(LIMIT_SWITCH_OPEN)   == LOW);

    if (closedActive && openActive) {      // both switches — wiring or roof fault
        s_motionState = MOTION_ERROR;
        s_lastCommand = LAST_CMD_NONE;
        return;
    }
    if (closedActive) {
        s_motionState = MOTION_NONE;
        if (s_lastCommand == LAST_CMD_CLOSE) s_lastCommand = LAST_CMD_NONE;
        return;
    }
    if (openActive) {
        if (s_motionState == MOTION_CLOSING) return;   // leaving the open switch
        s_motionState = MOTION_NONE;
        if (s_lastCommand == LAST_CMD_OPEN) s_lastCommand = LAST_CMD_NONE;
    }
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

// ── Roof commands ─────────────────────────────────────────────────────────────
// The single implementation behind both the Alpaca PUTs and the same-origin
// /cmd endpoint, so the browser path and the NINA path can never diverge.
// Returns false when the roof is already where it was asked to go.
bool commandOpen()
{
    if (getShutterStatus() == SHUTTER_OPEN) return false;
    s_lastCommand   = LAST_CMD_OPEN;
    s_motionState   = MOTION_OPENING;
    s_motionStartMs = millis();
    diagLog(EV_OPEN);
    triggerRelayPulse();
    return true;
}

bool commandClose()
{
    if (getShutterStatus() == SHUTTER_CLOSED) return false;
    s_lastCommand   = LAST_CMD_CLOSE;
    s_motionState   = MOTION_CLOSING;
    s_motionStartMs = millis();
    diagLog(EV_CLOSE);
    triggerRelayPulse();
    return true;
}

// The relay is a toggle, so only pulse when the roof is actually travelling —
// otherwise "abort" would start it.  Motion state is left for loop() to resolve
// so safeRestart()'s motion guard stays armed.
bool commandAbort()
{
    if (s_motionState != MOTION_OPENING && s_motionState != MOTION_CLOSING) {
        s_lastCommand = LAST_CMD_NONE;
        return false;
    }
    diagLog(EV_ABORT);
    triggerRelayPulse();
    s_lastCommand = LAST_CMD_NONE;
    return true;
}

// Called from the HTTP task.  Does nothing but raise a flag — no relay, no
// motion state, no diagnostics ring, all of which belong to loop().
void queueCommand(PendingCmd c)
{
    s_pendingCmd = (uint8_t)c;
}

// Called from loop(), and only from loop().  ASCOM shutter commands are
// asynchronous by specification — the client polls shutterstatus afterwards —
// so deferring by one loop iteration is both spec-legal and far below what any
// client could observe.
void runPendingCommand()
{
    uint8_t c = s_pendingCmd;
    if (c == PEND_NONE) return;
    s_pendingCmd = PEND_NONE;

    switch (c) {
        case PEND_OPEN:  commandOpen();  break;
        case PEND_CLOSE: commandClose(); break;
        case PEND_ABORT: commandAbort(); break;
        default: break;
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
    diagLog(EV_REBOOT);
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
    // The driver knows about a drop long before a 5 s poll would notice.  Rather
    // than duplicate the recovery logic, just force the poll below to run now —
    // shrinking a ~10 s outage to ~2 s, which may be short enough that clients
    // like NINA never see a failed request.
    if (s_wifiEvtDisconnected || s_wifiEvtGotIp) {
        s_wifiEvtDisconnected = false;
        s_wifiEvtGotIp        = false;
        s_lastWifiCheckMs     = 0;   // millis() - 0 always exceeds the interval
    }

    if (millis() - s_lastWifiCheckMs < WIFI_CHECK_INTERVAL_MS) return;
    s_lastWifiCheckMs = millis();

    if (WiFi.status() == WL_CONNECTED) {
        if (s_wifiDownSinceMs != 0) {
            Serial.print("WiFi recovered, IP: ");
            Serial.println(WiFi.localIP());
            diagLog(EV_WIFI_OK);
            oledWake();
            startDiscovery();   // the UDP socket does not survive re-association

            // A listening TCP socket should survive re-association where the
            // UDP one does not, but "should" is not worth a night of downtime.
            // Restarting is free: the outage has already broken whatever
            // persistent connection a client was holding.
            startAlpacaServer();
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
        s_diag.wifiDrops++;
        uint8_t reason = s_wifiDisconnectReason;
        Serial.printf("WiFi link lost (reason %u %s) — reconnecting\n",
                      reason, wifiReasonName(reason));
        diagLog(EV_WIFI_LOST, reason);
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

// ─────────────────────────────────────────────────────────────────────────────
//  Alpaca API (port 11111) — ESP-IDF esp_http_server
// ─────────────────────────────────────────────────────────────────────────────
//  Arduino's WebServer sends "Connection: close" on every response, so each
//  request costs a fresh TCP connection.  lwIP is built here with 16 TCP PCBs
//  and a 60 s MSL, and WebServer always performs the active close, so every
//  served request holds a PCB for about 140 s (FIN_WAIT_2 + TIME_WAIT).  That
//  is a budget of roughly seven connections a minute.  NINA polls a dome nine
//  properties at a time, several times a second — 130 to 270 requests a minute,
//  each its own connection — so the pool is permanently under pressure and
//  tcp_alloc() spends its time reaping TIME_WAIT entries.  The symptom is not a
//  refusal but multi-second latency spikes, which is exactly what the field
//  fault looked like.
//
//  esp_http_server emits no Connection header at all, and HTTP/1.1 without one
//  is persistent by RFC 7230.  ASCOM's client holds a pooled HttpClient and
//  never asks to close, so an entire observing session now costs one connection
//  instead of tens of thousands.  Verify with s_alpacaConns: requests per
//  connection should be in the hundreds, not 1.
//
//  Port 80 deliberately stays on WebServer.  All the machine polling is here on
//  11111, and leaving the browser page alone keeps the change small.  Note the
//  PCB pool is still shared, so hammering port 80 can still slow this port —
//  keep the browser poll interval long.
// ─────────────────────────────────────────────────────────────────────────────

// esp_http_server does NOT copy header values, so every value passed here must
// outlive the response.  String literals only — never a stack buffer.
static void alpacaCors(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin",  "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, PUT, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

// Every response is built with snprintf into a stack buffer.  The old String
// version allocated five times per request; at NINA's polling rate that was the
// firmware's largest source of heap churn, for no benefit — the envelope is a
// fixed shape and always fits.
static esp_err_t alpacaSend(httpd_req_t *req, const char *valueJson,
                            uint32_t clientTx, uint32_t errorNum, const char *errorMsg)
{
    uint32_t serverTx = __atomic_fetch_add(&s_serverTxId, 1, __ATOMIC_RELAXED);
    char resp[320];
    snprintf(resp, sizeof(resp),
             "{\"ClientTransactionID\":%lu,\"ServerTransactionID\":%lu,"
             "\"ErrorNumber\":%lu,\"ErrorMessage\":\"%s\",%s}",
             (unsigned long)clientTx, (unsigned long)serverTx,
             (unsigned long)errorNum, errorMsg, valueJson);
    alpacaCors(req);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t alpacaBool(httpd_req_t *req, bool v, uint32_t clientTx)
{
    return alpacaSend(req, v ? "\"Value\":true" : "\"Value\":false", clientTx, 0, "");
}

static esp_err_t alpacaInt(httpd_req_t *req, int v, uint32_t clientTx)
{
    char b[32];
    snprintf(b, sizeof(b), "\"Value\":%d", v);
    return alpacaSend(req, b, clientTx, 0, "");
}

static esp_err_t alpacaStr(httpd_req_t *req, const char *v, uint32_t clientTx)
{
    char b[160];
    snprintf(b, sizeof(b), "\"Value\":\"%s\"", v);
    return alpacaSend(req, b, clientTx, 0, "");
}

// For values that are already JSON — arrays, objects, or null.
static esp_err_t alpacaRaw(httpd_req_t *req, const char *valueJson, uint32_t clientTx)
{
    return alpacaSend(req, valueJson, clientTx, 0, "");
}

static esp_err_t alpacaError(httpd_req_t *req, uint32_t clientTx,
                             uint32_t errNum, const char *msg)
{
    return alpacaSend(req, "\"Value\":null", clientTx, errNum, msg);
}

// ── Request parsing ───────────────────────────────────────────────────────────
// Case-insensitive lookup in an "a=1&b=2" string.  Alpaca parameter names are
// case-insensitive, and query strings and form-urlencoded bodies share this
// syntax, so one function serves both.  Values in this API are only booleans
// and integers, so no percent-decoding is needed.
static bool formValue(const char *src, const char *key, char *out, size_t outLen)
{
    if (!src || !*src) return false;
    size_t klen = strlen(key);

    for (const char *p = src; *p; ) {
        const char *amp = strchr(p, '&');
        const char *end = amp ? amp : p + strlen(p);
        const char *eq  = (const char *)memchr(p, '=', (size_t)(end - p));

        if (eq && (size_t)(eq - p) == klen && strncasecmp(p, key, klen) == 0) {
            size_t vlen = (size_t)(end - eq - 1);
            if (vlen >= outLen) vlen = outLen - 1;
            memcpy(out, eq + 1, vlen);
            out[vlen] = '\0';
            return true;
        }
        if (!amp) break;
        p = amp + 1;
    }
    return false;
}

// Gathers the request's parameters into one "a=1&b=2" buffer: query string
// first, then the form-urlencoded body.  Alpaca allows a parameter in either,
// and the body can only be read once, so a handler collects everything up front
// and looks it up from there.
static void alpacaParams(httpd_req_t *req, char *buf, size_t bufLen)
{
    size_t n = 0;
    if (httpd_req_get_url_query_str(req, buf, bufLen) == ESP_OK) n = strlen(buf);
    else buf[0] = '\0';

    if (req->content_len > 0 && n + 1 < bufLen) {
        if (n) buf[n++] = '&';
        size_t room = bufLen - n - 1;
        size_t want = req->content_len < room ? req->content_len : room;
        size_t got  = 0;
        while (got < want) {
            int r = httpd_req_recv(req, buf + n + got, want - got);
            if (r <= 0) break;          // timeout or peer closed — use what arrived
            got += (size_t)r;
        }
        buf[n + got] = '\0';
        // Any body beyond `want` is drained by httpd_req_delete() when the
        // handler returns, so truncating here cannot leave unread bytes to
        // desync the next request on a persistent connection.
    }
}

static uint32_t alpacaClientTx(const char *params)
{
    char v[16];
    if (!formValue(params, "ClientTransactionID", v, sizeof(v))) return 0;
    return (uint32_t)strtoul(v, nullptr, 10);
}

// The endpoint name, lowercased, so "ShutterStatus" and "shutterstatus" both
// match.  The path prefix ahead of it is still matched case-sensitively — httpd
// routes case-sensitively too, so a mixed-case prefix would never reach here
// anyway, and every ASCOM client sends it lowercase.
// req->uri may or may not carry the query string depending on the IDF version,
// so stop at '?' either way.
static void alpacaEndpoint(httpd_req_t *req, const char *prefix,
                           char *out, size_t outLen)
{
    const char *p    = req->uri;
    size_t      plen = strlen(prefix);
    if (strncmp(p, prefix, plen) == 0) p += plen;

    size_t i = 0;
    while (p[i] && p[i] != '?' && i < outLen - 1) {
        out[i] = (char)tolower((unsigned char)p[i]);
        i++;
    }
    out[i] = '\0';
}

#define DOME_PREFIX "/api/v1/dome/0/"

// ── Dome GET ──────────────────────────────────────────────────────────────────
// Runs on the httpd task.  Every branch here is a pure read: digitalRead(), the
// motion state, and constants.  Nothing in this function may mutate roof state.
static esp_err_t handleDomeGet(httpd_req_t *req)
{
    noteRequest(true);

    char ep[48];
    alpacaEndpoint(req, DOME_PREFIX, ep, sizeof(ep));

    char params[256];
    alpacaParams(req, params, sizeof(params));
    uint32_t tx = alpacaClientTx(params);

    if (!strcmp(ep, "connected"))     return alpacaBool(req, s_alpacaConnected, tx);
    if (!strcmp(ep, "description"))   return alpacaStr(req, "Rolloff roof Alpaca dome driver", tx);
    if (!strcmp(ep, "driverinfo"))    return alpacaStr(req, "ESP32 Alpaca Dome Driver", tx);
    if (!strcmp(ep, "driverversion")) return alpacaStr(req, "1.0", tx);
    if (!strcmp(ep, "name"))          return alpacaStr(req, "Rolloff Roof", tx);
    if (!strcmp(ep, "supportedactions")) return alpacaRaw(req, "\"Value\":[]", tx);

    // IDomeV2 (ASCOM Platform 6).  This was 3, which claims IDomeV3 and so
    // promises Platform 7's DeviceState, Connect, Disconnect and Connecting —
    // none of which this driver implements.  NINA never asked for them, so the
    // mismatch was latent, but any client that trusted the 3 would have got a
    // 404.  Declare what is actually here.
    if (!strcmp(ep, "interfaceversion")) return alpacaInt(req, 2, tx);

    if (!strcmp(ep, "cansetshutter"))  return alpacaBool(req, true,  tx);
    if (!strcmp(ep, "canfindhome"))    return alpacaBool(req, false, tx);
    if (!strcmp(ep, "canpark"))        return alpacaBool(req, false, tx);
    if (!strcmp(ep, "cansetaltitude")) return alpacaBool(req, false, tx);
    if (!strcmp(ep, "cansetazimuth"))  return alpacaBool(req, false, tx);
    if (!strcmp(ep, "cansetpark"))     return alpacaBool(req, false, tx);
    if (!strcmp(ep, "canslave"))       return alpacaBool(req, false, tx);
    if (!strcmp(ep, "cansyncazimuth")) return alpacaBool(req, false, tx);

    if (!strcmp(ep, "shutterstatus")) return alpacaInt (req, (int)getShutterStatus(), tx);
    if (!strcmp(ep, "slewing"))       return alpacaBool(req, isSlewing(), tx);
    if (!strcmp(ep, "slaved"))        return alpacaBool(req, false, tx);

    if (!strcmp(ep, "athome")  || !strcmp(ep, "atpark") ||
        !strcmp(ep, "azimuth") || !strcmp(ep, "altitude"))
        return alpacaError(req, tx, 0x400, "PropertyNotImplementedException");

    return alpacaError(req, tx, 0x401, "InvalidValueException");
}

// ── Dome PUT ──────────────────────────────────────────────────────────────────
// Also on the httpd task, so the three movement commands only queue work for
// loop() — see queueCommand().  The Alpaca response for a shutter command
// carries no value, so nothing is lost by answering before the relay fires.
static esp_err_t handleDomePut(httpd_req_t *req)
{
    noteRequest(true);

    char ep[48];
    alpacaEndpoint(req, DOME_PREFIX, ep, sizeof(ep));

    char params[256];
    alpacaParams(req, params, sizeof(params));
    uint32_t tx = alpacaClientTx(params);

    if (!strcmp(ep, "connected")) {
        char v[16];
        if (formValue(params, "Connected", v, sizeof(v))) {
            if (!strcasecmp(v, "true")  || !strcmp(v, "1")) s_alpacaConnected = true;
            if (!strcasecmp(v, "false") || !strcmp(v, "0")) s_alpacaConnected = false;
        }
        return alpacaBool(req, s_alpacaConnected, tx);
    }

    if (!strcmp(ep, "openshutter"))  { queueCommand(PEND_OPEN);  return alpacaRaw(req, "\"Value\":null", tx); }
    if (!strcmp(ep, "closeshutter")) { queueCommand(PEND_CLOSE); return alpacaRaw(req, "\"Value\":null", tx); }
    if (!strcmp(ep, "abortslew"))    { queueCommand(PEND_ABORT); return alpacaRaw(req, "\"Value\":null", tx); }

    if (!strcmp(ep, "action")      || !strcmp(ep, "commandblind") ||
        !strcmp(ep, "commandbool") || !strcmp(ep, "commandstring") ||
        !strcmp(ep, "findhome")    || !strcmp(ep, "park")          ||
        !strcmp(ep, "setpark")     || !strcmp(ep, "slewtoaltitude")||
        !strcmp(ep, "slewtoazimuth") || !strcmp(ep, "synctoazimuth") ||
        !strcmp(ep, "slaved"))
        return alpacaError(req, tx, 0x400, "MethodNotImplementedException");

    return alpacaError(req, tx, 0x401, "InvalidValueException");
}

// ── Management ────────────────────────────────────────────────────────────────
static esp_err_t handleManagement(httpd_req_t *req)
{
    noteRequest(true);

    char path[64];
    alpacaEndpoint(req, "", path, sizeof(path));   // whole path, lowercased

    char params[192];
    alpacaParams(req, params, sizeof(params));
    uint32_t tx = alpacaClientTx(params);

    if (!strcmp(path, "/management/apiversions"))
        return alpacaRaw(req, "\"Value\":[1]", tx);

    if (!strcmp(path, "/management/v1/description"))
        return alpacaRaw(req,
            "\"Value\":{\"ServerName\":\"ESP32 Alpaca\",\"Manufacturer\":\"Rolloff\","
            "\"ManufacturerVersion\":\"1.0\",\"Location\":\"Observatory\"}", tx);

    // UniqueID is the same on every board built from this firmware, so two of
    // them on one network are indistinguishable to a client.  Left alone here
    // deliberately: changing it makes NINA treat the device as a new one and
    // the saved profile has to be re-pointed by hand.
    if (!strcmp(path, "/management/v1/configureddevices"))
        return alpacaRaw(req,
            "\"Value\":[{\"DeviceType\":\"Dome\",\"DeviceName\":\"Rolloff Roof\","
            "\"DeviceNumber\":0,\"UniqueID\":\"ESP32-ROLLOFF-0\"}]", tx);

    alpacaCors(req);
    httpd_resp_set_status(req, "404 Not Found");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "Not found", HTTPD_RESP_USE_STRLEN);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
static esp_err_t handleAlpacaSetup(httpd_req_t *req)
{
    noteRequest(true);
    alpacaCors(req);
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req,
        "<html><body><h1>Alpaca Setup</h1>"
        "<p>This device uses fixed configuration in firmware.</p></body></html>",
        HTTPD_RESP_USE_STRLEN);
}

// ── CORS preflight ────────────────────────────────────────────────────────────
static esp_err_t handleAlpacaOptions(httpd_req_t *req)
{
    noteRequest(true);
    alpacaCors(req);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "", 0);
}

static esp_err_t handleAlpacaFavicon(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, "", 0);
}

// Returning ESP_OK keeps the connection open.  The stock 404 handler closes it,
// which would cost NINA its persistent connection over a single stray request.
static esp_err_t handleAlpacaNotFound(httpd_req_t *req, httpd_err_code_t err)
{
    noteRequest(true);
    alpacaCors(req);
    httpd_resp_set_status(req, "404 Not Found");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "Not found", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Counts accepted TCP connections from inside the server — the denominator of
// the requests-per-connection metric.  There is deliberately no close_fn: when
// one is configured the server hands it the socket instead of closing it, and
// getting the ownership of a file descriptor wrong on a roof controller is not
// worth a second counter.
static esp_err_t alpacaSessionOpen(httpd_handle_t hd, int sockfd)
{
    bumpCounter(s_alpacaConns);
    return ESP_OK;
}

// ── Server startup ────────────────────────────────────────────────────────────
static void registerAlpacaUri(const char *uri, httpd_method_t method,
                              esp_err_t (*handler)(httpd_req_t *))
{
    httpd_uri_t u = {};   // zero-init: the struct has extra websocket fields
    u.uri      = uri;
    u.method   = method;
    u.handler  = handler;
    u.user_ctx = nullptr;

    esp_err_t e = httpd_register_uri_handler(s_alpacaHttpd, &u);
    if (e != ESP_OK)
        Serial.printf("Alpaca httpd: register %s failed: %s\n", uri, esp_err_to_name(e));
}

void startAlpacaServer()
{
    if (s_alpacaHttpd) {
        httpd_stop(s_alpacaHttpd);
        s_alpacaHttpd = nullptr;
    }

    httpd_config_t cfg    = HTTPD_DEFAULT_CONFIG();
    cfg.server_port       = ALPACA_HTTP_PORT;
    cfg.task_priority     = tskIDLE_PRIORITY + 5;
    cfg.stack_size        = 6144;   // handlers keep ~600 B of buffers on the stack
    cfg.core_id           = 0;      // off the Arduino loop's core (1)
    cfg.max_open_sockets  = 4;      // NINA needs one; the rest is headroom
    cfg.max_uri_handlers  = 12;
    cfg.backlog_conn      = 5;
    cfg.lru_purge_enable  = true;   // a wedged client cannot lock the others out
    cfg.recv_wait_timeout = 5;      // a silent client is dropped, not held
    cfg.send_wait_timeout = 5;
    cfg.open_fn           = alpacaSessionOpen;
    cfg.uri_match_fn      = httpd_uri_match_wildcard;

    // Unlike WiFiServer::begin(), which returns void and fails silently, this
    // reports a bind failure — so the guard GuardedWebServer exists to provide
    // comes free here.
    esp_err_t e = httpd_start(&s_alpacaHttpd, &cfg);
    if (e != ESP_OK) {
        s_alpacaHttpd = nullptr;
        Serial.printf("Alpaca httpd start FAILED: %s\n", esp_err_to_name(e));
        return;
    }

    registerAlpacaUri(DOME_PREFIX "*", HTTP_GET,     handleDomeGet);
    registerAlpacaUri(DOME_PREFIX "*", HTTP_PUT,     handleDomePut);
    registerAlpacaUri(DOME_PREFIX "*", HTTP_OPTIONS, handleAlpacaOptions);
    registerAlpacaUri("/management/*", HTTP_GET,     handleManagement);
    registerAlpacaUri("/management/*", HTTP_OPTIONS, handleAlpacaOptions);
    registerAlpacaUri("/setup*",       HTTP_GET,     handleAlpacaSetup);
    registerAlpacaUri("/favicon.ico",  HTTP_GET,     handleAlpacaFavicon);
    httpd_register_err_handler(s_alpacaHttpd, HTTPD_404_NOT_FOUND, handleAlpacaNotFound);

    Serial.printf("Alpaca server on port %d (esp_http_server, persistent)\n",
                  ALPACA_HTTP_PORT);
}

// Live session count, for the status page.
static int alpacaSessionCount()
{
    if (!s_alpacaHttpd) return 0;
    int    fds[8];
    size_t n = sizeof(fds) / sizeof(fds[0]);
    if (httpd_get_client_list(s_alpacaHttpd, &n, fds) != ESP_OK) return -1;
    return (int)n;
}

// ── Info page (port 80) ───────────────────────────────────────────────────────
// Served straight from flash: no String building, no per-request heap churn.
// The page polls /status.json instead of reloading itself, and its buttons hit
// /cmd on this same origin — so the browser never opens a connection to the
// Alpaca port and never issues a CORS preflight.  The previous version reloaded
// a ~4 kB page every 3 s, which on its own exceeded what the device can sustain:
// lwIP has 16 TCP PCBs and each served request holds one for ~140 s
// (FIN_WAIT_2 + TIME_WAIT), giving a budget of roughly 7 connections a minute.
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rolloff Roof</title><style>
body{font-family:Arial,Helvetica,sans-serif;margin:24px;background:#f5f5f5;color:#222}
h1{color:#333;font-size:1.5em}h2{font-size:1.1em;margin:0 0 8px}
.card{padding:16px;margin:12px 0;border-radius:8px;background:#fff;box-shadow:0 2px 6px rgba(0,0,0,.1)}
.btn{padding:10px 16px;margin-right:8px;border:none;border-radius:6px;cursor:pointer;font-size:1em}
.open{background:#28a745;color:#fff}.close{background:#dc3545;color:#fff}
.on{color:#dc3545;font-weight:bold}.off{color:#28a745}
#msg{margin-left:10px}
body.stale{opacity:.45}
p{margin:6px 0}
</style></head><body>
<h1>Rolloff Roof Controller</h1>
<div class="card"><h2>Status</h2>
<p><b>Shutter:</b> <span id="sh">&hellip;</span></p>
<p><b>Alpaca connected:</b> <span id="ac">&hellip;</span></p>
<p><b>Closed switch (GPIO 32):</b> <span id="cs">&hellip;</span></p>
<p><b>Open switch (GPIO 33):</b> <span id="os">&hellip;</span></p>
</div>
<div class="card"><h2>Controls</h2>
<button class="btn open" onclick="cmd('open')">Open Roof</button>
<button class="btn close" onclick="cmd('close')">Close Roof</button>
<span id="msg"></span>
</div>
<div class="card"><h2>Link</h2>
<p><b>Uptime:</b> <span id="up">&hellip;</span> min</p>
<p><b>WiFi RSSI:</b> <span id="rs">&hellip;</span> dBm</p>
<p><b>WiFi drops since boot:</b> <span id="wd">&hellip;</span></p>
<p><b>Requests (info/alpaca):</b> <span id="rq">&hellip;</span></p>
<p><b>Request rate:</b> <span id="rt">&hellip;</span> /min</p>
<p><b>Alpaca connections:</b> <span id="cn">&hellip;</span> (<span id="sx">&hellip;</span> open)</p>
<p><b>Requests per connection:</b> <span id="pc">&hellip;</span></p>
<p><b>Free heap:</b> <span id="hp">&hellip;</span> bytes</p>
<p><b>Largest free block:</b> <span id="mb">&hellip;</span> bytes</p>
<p><b>Listeners up:</b> <span id="ls">&hellip;</span></p>
</div>
<p><a href="/diag">Diagnostics &amp; event log</a></p>
<script>
function T(i,v){document.getElementById(i).textContent=v;}
function sw(i,a){var e=document.getElementById(i);e.textContent=a?"ACTIVE":"inactive";e.className=a?"on":"off";}
function upd(){
 fetch("/status.json",{cache:"no-store"}).then(function(r){return r.json();}).then(function(d){
  document.body.classList.remove("stale");
  T("sh",d.shutter);T("ac",d.connected?"Yes":"No");
  sw("cs",d.closedSw);sw("os",d.openSw);
  T("up",d.uptimeMin);T("rs",d.rssi);T("wd",d.drops);
  T("rq",d.reqInfo+" / "+d.reqAlpaca);T("rt",d.rate);
  T("cn",d.conns);T("sx",d.sess);T("pc",d.reqPerConn);
  T("hp",d.heap);T("mb",d.maxBlock);T("ls",d.listen?"yes":"NO");
 }).catch(function(){document.body.classList.add("stale");});
}
function cmd(a){
 T("msg","sending…");
 fetch("/cmd?a="+a,{cache:"no-store"}).then(function(r){return r.text();})
  .then(function(t){T("msg",t);setTimeout(upd,400);})
  .catch(function(e){T("msg","failed: "+e);});
}
upd();setInterval(upd,10000);
</script></body></html>
)rawliteral";

void handleInfoRoot()
{
    noteRequest(false);
    oledWake();   // someone is looking at the device
    infoServer.send_P(200, "text/html", INDEX_HTML);
}

// Small fixed-size JSON built with snprintf into a stack buffer — deliberately
// no String, so polling this costs no heap and cannot fragment it.
void handleStatusJson()
{
    noteRequest(false);

    // reqPerConn is the number that says whether persistent connections are
    // actually happening: 1 means every request still costs a TCP connection.
    unsigned long conns = (unsigned long)s_alpacaConns;

    char buf[480];
    snprintf(buf, sizeof(buf),
        "{\"shutter\":\"%s\",\"connected\":%s,\"closedSw\":%s,\"openSw\":%s,"
        "\"uptimeMin\":%lu,\"rssi\":%d,\"drops\":%lu,"
        "\"reqInfo\":%lu,\"reqAlpaca\":%lu,\"rate\":%lu,"
        "\"conns\":%lu,\"reqPerConn\":%lu,\"sess\":%d,"
        "\"heap\":%lu,\"maxBlock\":%lu,\"listen\":%s}",
        shutterLabel(getShutterStatus()),
        s_alpacaConnected ? "true" : "false",
        (digitalRead(LIMIT_SWITCH_CLOSED) == LOW) ? "true" : "false",
        (digitalRead(LIMIT_SWITCH_OPEN)   == LOW) ? "true" : "false",
        millis() / 60000UL,
        (int)WiFi.RSSI(),
        (unsigned long)s_wifiDropCount,
        (unsigned long)s_infoRequests,
        (unsigned long)s_alpacaRequests,
        (unsigned long)s_recentRequests,
        conns,
        conns ? (unsigned long)s_alpacaRequests / conns : 0UL,
        alpacaSessionCount(),
        (unsigned long)ESP.getFreeHeap(),
        (unsigned long)ESP.getMaxAllocHeap(),
        (infoServer.isListening() && s_alpacaHttpd) ? "true" : "false");

    infoServer.send(200, "application/json", buf);
}

// Same-origin roof control.  Keeping this on port 80 means the browser makes no
// cross-origin request and so no OPTIONS preflight — halving what each click
// costs in connections.  CORS stays enabled on the Alpaca server for NINA and
// any other third-party client.
void handleCmd()
{
    noteRequest(false);
    oledWake();

    String a = infoServer.arg("a");
    const char *msg;

    if      (a == "open")  msg = commandOpen()  ? "opening"  : "already open";
    else if (a == "close") msg = commandClose() ? "closing"  : "already closed";
    else if (a == "abort") msg = commandAbort() ? "aborting" : "not moving";
    else {
        infoServer.send(400, "text/plain", "use ?a=open|close|abort");
        return;
    }
    infoServer.send(200, "text/plain", msg);
}

// Diagnostics live on their own page so the main page stays small and cheap.
// This one still builds a String, but it is only fetched when someone asks.
void handleDiagPage()
{
    noteRequest(false);

    String html =
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Rolloff Diagnostics</title><style>"
        "body{font-family:Arial,Helvetica,sans-serif;margin:24px;background:#f5f5f5;color:#222}"
        "h1{font-size:1.4em}h2{font-size:1.1em;margin:0 0 8px}"
        ".card{padding:16px;margin:12px 0;border-radius:8px;background:#fff;"
              "box-shadow:0 2px 6px rgba(0,0,0,.1)}"
        "table{border-collapse:collapse;width:100%}"
        "th,td{padding:4px 8px;border-bottom:1px solid #eee;font-size:.9em;text-align:left}"
        "p{margin:6px 0}"
        "</style></head><body><h1>Rolloff Roof Controller</h1>"
        + diagCardHtml() +
        "<div class='card'><h2>Alpaca</h2>"
        "<p>API base: <code>http://&lt;IP&gt;:" + String(ALPACA_HTTP_PORT) + "/api/v1/dome/0/</code></p>"
        "<p>Discovery UDP port: " + String(ALPACA_DISCOVERY_PORT) + "</p>"
        "<p>Lowest free heap since boot: " + String(ESP.getMinFreeHeap()) + " bytes</p>"
        "</div>"
        "<p><a href='/'>&larr; back</a></p></body></html>";

    infoServer.send(200, "text/html", html);
}

// ── Alpaca UDP discovery ──────────────────────────────────────────────────────
// Callback-driven rather than polled.  The old WiFiUDP version called
// parsePacket() on every loop() iteration, and that does a malloc(1460) + free()
// each time — by far the largest source of allocator churn in this firmware, and
// the most likely cause of any heap fragmentation.  The callback runs in the
// lwIP task but touches nothing loop() owns, so there is no race.
// The socket does not survive a re-association, so this is re-armed from
// maintainWifi() whenever the link comes back.
void startDiscovery()
{
    udpDiscovery.close();
    if (!udpDiscovery.listen(ALPACA_DISCOVERY_PORT)) {
        Serial.println("Alpaca discovery: listen FAILED");
        return;
    }
    udpDiscovery.onPacket([](AsyncUDPPacket packet) {
        if (packet.length() < 16) return;
        if (strncmp((const char *)packet.data(), "alpacadiscovery1", 16) != 0) return;
        // The reply must carry ONLY AlpacaPort.  The RG-11 safety monitor
        // records that adding a Devices array makes NINA silently drop the
        // device — do not "improve" this payload.
        char resp[40];
        snprintf(resp, sizeof(resp), "{\"AlpacaPort\":%d}", ALPACA_HTTP_PORT);
        packet.print(resp);
    });
}

// ── setup() ───────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    Serial.println("\n\nESP32 Rolloff Alpaca Driver starting...");

    // Diagnostics record — RTC memory is garbage after a power cycle, so gate on
    // both the magic and the reset reason before trusting it.
    esp_reset_reason_t rr = esp_reset_reason();
    if (s_diag.magic != DIAG_MAGIC || rr == ESP_RST_POWERON) {
        memset(&s_diag, 0, sizeof(s_diag));
        s_diag.magic = DIAG_MAGIC;
    }
    s_diag.bootCount++;
    Serial.printf("Reset reason: %s (boot #%lu)\n", resetReasonName(),
                  (unsigned long)s_diag.bootCount);

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
    // 400 kHz: pushing the 512-byte framebuffer at the 100 kHz default blocks
    // loop() for ~46 ms every 500 ms — ~9 % of the time accepting no
    // connections.  The SSD1306 is rated for fast-mode I2C.
    Wire.setClock(400000);

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
    WiFi.onEvent(onWiFiEvent);     // must be registered before begin()
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

    // NTP — non-blocking, syncs in the background.  Only needed so the event log
    // carries wall-clock times that line up with NINA's log.
    configTzTime(TZ_INFO, NTP_SERVER_1, NTP_SERVER_2);
    diagLog(EV_BOOT);

    // Info server — port 80
    infoServer.on("/",            HTTP_GET, handleInfoRoot);
    infoServer.on("/status.json", HTTP_GET, handleStatusJson);
    infoServer.on("/cmd",         HTTP_GET, handleCmd);
    infoServer.on("/diag",        HTTP_GET, handleDiagPage);
    infoServer.on("/favicon.ico", HTTP_GET, []() { infoServer.send(204); });
    infoServer.onNotFound([]() { infoServer.send(404, "text/plain", "Not found"); });
    infoServer.begin();
    if (!infoServer.isListening()) Serial.println("WARNING: info server failed to listen");
    Serial.printf("Info server on port %d\n", INFO_PORT);

    // Alpaca server — port 11111.  Routing is three wildcard handlers rather
    // than the old 35-entry endpoint table; the dispatch on the endpoint name
    // already lived inside the handler, so the table only existed to stop
    // WebServer logging "request handler not found".
    startAlpacaServer();

    // Alpaca UDP discovery
    startDiscovery();
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

    // Sole owner of motion state — everything else only reads it
    updateMotionState();

    // Alpaca commands arrive on the HTTP server's task and are executed here,
    // so the relay and the motion state have exactly one writer.
    runPendingCommand();

    // Movement timeout watchdog
    if ((s_motionState == MOTION_OPENING || s_motionState == MOTION_CLOSING) &&
        s_motionStartMs != 0 &&
        (millis() - s_motionStartMs) > MOVEMENT_TIMEOUT_MS) {
        s_motionState = MOTION_ERROR;
    }

    maintainWifi();

    infoServer.handleClient();
    // Alpaca runs on its own task and discovery is callback-driven — neither
    // needs polling here.  Port 80 is the only server left that does.

    // Roll the request-rate window
    if (millis() - s_windowStartMs >= REQ_WINDOW_MS) {
        s_recentRequests = s_windowCount;
        s_windowCount    = 0;
        s_windowStartMs  = millis();
    }

    // Log-only stall detector.  Deliberately takes no action: a quiet device is
    // indistinguishable from a wedged one without a client to compare against,
    // and a roof controller that reboots itself on a guess is worse than one
    // that is briefly unreachable.  Edge-triggered so a quiet day logs once.
    if (!s_stallLogged && s_lastRequestMs != 0 &&
        (millis() - s_lastRequestMs) > NET_STALL_MS) {
        s_stallLogged = true;
        diagLog(EV_NET_STALL, (uint8_t)((millis() - s_lastRequestMs) / 60000UL));
        Serial.printf("No HTTP requests for %lu min (listening: info=%d alpaca=%d)\n",
                      (millis() - s_lastRequestMs) / 60000UL,
                      infoServer.isListening(), s_alpacaHttpd != nullptr);
    }

    // Serial heartbeat — the fingerprint of the fault is both request counts
    // going flat together while uptime and RSSI keep advancing.
    if (millis() - s_lastHeartbeatMs >= HEARTBEAT_MS) {
        s_lastHeartbeatMs = millis();
        unsigned long conns = (unsigned long)s_alpacaConns;
        Serial.printf("up=%lumin rssi=%d req(info/alpaca)=%lu/%lu rate=%lu/min "
                      "conn=%lu req/conn=%lu sess=%d "
                      "listen=%d/%d heap=%u maxblk=%u minfree=%u\n",
                      millis() / 60000UL, WiFi.RSSI(),
                      (unsigned long)s_infoRequests, (unsigned long)s_alpacaRequests,
                      (unsigned long)s_recentRequests,
                      conns, conns ? (unsigned long)s_alpacaRequests / conns : 0UL,
                      alpacaSessionCount(),
                      infoServer.isListening(), s_alpacaHttpd != nullptr,
                      ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap());
    }

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
