/**
 * GootAC - Mitsubishi HomeKit Controller
 * 
 * LOGGING KEY:
 * [WIFI]       - Wireless Connection & Network Status
 * [BOOT]       - Startup Sequence & Initial Configuration
 * [MITSUBISHI] - Mitsubishi AC Physical Unit Control & Logic
 * [HOMEKIT]    - HomeKit Server, Pairings & Interactions
 * [SYS]        - System, OS, Watchdogs & Maintenance
 * [OTA]        - Over-the-Air Firmware Updates
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
extern "C" {
#include <user_interface.h>
}
#include "ac_controller.h"
#include "fs_logger.h"
#include "hk_log_bridge.h"
#include "homekit_ac.h"
#include <ArduinoLog.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <HeatPump.h>
#include <LittleFS.h>
#include <arduino_homekit_server.h>
#include <stdarg.h>

#include "config.h"

#ifdef USE_FAKE_HEATPUMP
#include "HeatPumpFake.h"
// Override identity so the fake unit can never be confused with a production
// device on the LAN. Also force HomeKit to start without waiting for a CN105
// handshake (the fake has no UART peer).
#undef DEVICE_NAME
#define DEVICE_NAME "FakeDiag"
#undef FW_VERSION
#define FW_VERSION "0.0.1-fake"
#undef FORCE_HK_START
#define FORCE_HK_START true
#endif

ESP8266WebServer server(80);

// Streams Print output to the HTTP client in ~256 B chunks so a full /metrics
// body never lives in RAM at once. Content-transparent (the metric writer
// supplies bare-LF lines).
class ChunkedPrint : public Print {
  ESP8266WebServer& _srv;
  uint8_t _buf[256];
  size_t _len = 0;
public:
  explicit ChunkedPrint(ESP8266WebServer& s) : _srv(s) {}
  size_t write(uint8_t c) override {
    _buf[_len++] = c;
    if (_len == sizeof(_buf)) flushChunk();
    return 1;
  }
  size_t write(const uint8_t* data, size_t n) override {
    for (size_t i = 0; i < n; i++) write(data[i]);
    return n;
  }
  void flushChunk() {
    if (_len) { _srv.sendContent((const char*)_buf, _len); _len = 0; yield(); }
  }
};

char hostName[32];
char accessoryName[32];
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;

unsigned long lastUptimeUpdate = 0;

HeatPump hp;
bool homekitStarted = false;

// Persistent reset info tracking (values captured in setup_wifi)
String previousResetInfo = "";
unsigned long resetReportStart = 0;
bool has_reset_info = false;

// Logging Prefix Helper for ArduinoLog
void printPrefix(Print *_logOutput, int level) {
  unsigned long ms = millis();
  time_t now = time(nullptr);
  struct tm *timeinfo = localtime(&now);

  static const char levels[] = "FEWITV";
  char levelChar = (level >= 1 && level <= 6) ? levels[level - 1] : '?';

  _logOutput->print("[");
  // Uptime in seconds
  _logOutput->print(ms / 1000);
  _logOutput->print(".");
  if (ms % 1000 < 100)
    _logOutput->print("0");
  if (ms % 1000 < 10)
    _logOutput->print("0");
  _logOutput->print(ms % 1000);
  _logOutput->print("s");

  // Real time if synced (after 2020)
  if (now > 1577836800) {
    char timeStr[16];
    strftime(timeStr, sizeof(timeStr), " %H:%M:%S", timeinfo);
    _logOutput->print(timeStr);
  }
  _logOutput->print("] ");
  _logOutput->print(levelChar);
  _logOutput->print(": ");
}

// Centralized Logger for external C libraries (like arduino-homekit-esp8266)
extern "C" void udp_log_printf(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf_P(buf, sizeof(buf), fmt, args);
  va_end(args);

  // Clean trailing whitespace/newlines from library logs
  size_t len = strlen(buf);
  while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r' ||
                     buf[len - 1] == ' ')) {
    buf[--len] = '\0';
  }

  if (len > 0) {
    // We already have the prefix in our log engine, so just write the message
    Log.traceln("%s", buf);
  }
}

// Helper to get formatted uptime string
String getUptimeString() {
  uint32_t s = millis() / 1000;
  char up[16];
  if (s < 3600)
    snprintf(up, sizeof(up), "%us", s);
  else if (s < 86400)
    snprintf(up, sizeof(up), "%uh%um", s / 3600, (s % 3600) / 60);
  else
    snprintf(up, sizeof(up), "%ud%uh", s / 86400, (s % 86400) / 3600);
  return String(up);
}

void setup_webserver() {
#ifdef GOOTAC_HK_LOG
  server.on("/hklog", HTTP_GET, []() {
    const char *a, *b;
    size_t alen, blen;
    hk_log_segments(&a, &alen, &b, &blen);
    server.setContentLength(alen + blen);
    server.send(200, "text/plain", "");
    if (alen) server.sendContent(a, alen);
    if (blen) server.sendContent(b, blen);
    if (server.hasArg("clear")) hk_log_clear();
  });
#endif

  server.on("/log", HTTP_GET, []() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/plain", ""); // Start chunked response

    char buf[512];
    if (LittleFS.exists("/system.log")) {
      File f = LittleFS.open("/system.log", "r");
      while (f && f.available()) {
        size_t n = f.read((uint8_t *)buf, sizeof(buf));
        server.sendContent(buf, n);
        yield();
      }
      if (f)
        f.close();
    }
    server.sendContent(""); // End response
  });

  server.on("/log.old", HTTP_GET, []() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/plain", ""); // Start chunked response

    char buf[512];
    if (LittleFS.exists("/system.log.old")) {
      File f = LittleFS.open("/system.log.old", "r");
      while (f && f.available()) {
        size_t n = f.read((uint8_t *)buf, sizeof(buf));
        server.sendContent(buf, n);
        yield();
      }
      if (f)
        f.close();
    }
    server.sendContent(""); // End response
  });

  server.on("/metrics", HTTP_GET, []() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/plain; version=0.0.4; charset=utf-8", "");
    ChunkedPrint out(server);
    ac_controller_write_metrics(out);
    out.flushChunk();
    server.sendContent(""); // terminate chunked response
  });

  // Recovery endpoint: wipe HomeKit pairings + reboot. Needed when stale
  // pairing data from a previous lib version leaves the device advertising
  // sf=0 (paired) while no usable pairings exist, causing iOS to refuse with
  // kCountErr -6764 because it tries pair-verify instead of pair-setup.
  // Requires ?confirm=yes-wipe-pairings to prevent accidental triggers.
  server.on("/hk_reset", HTTP_GET, []() {
    if (server.arg("confirm") != "yes-wipe-pairings") {
      server.send(400, "application/json",
                  "{\"error\":\"require ?confirm=yes-wipe-pairings\"}");
      return;
    }
    GLOG_WARN("HOMEKIT", "/hk_reset invoked: wiping pairings + rebooting");
    server.send(200, "application/json",
                "{\"ok\":true,\"action\":\"reset_and_reboot\"}");
    delay(200);
    homekit_server_reset();
    delay(200);
    ESP.restart();
  });

  // Plain reboot. Useful for recovering a unit stuck in a stale HK session
  // without losing pairings. Requires ?confirm=yes.
  server.on("/reboot", HTTP_GET, []() {
    if (server.arg("confirm") != "yes") {
      server.send(400, "application/json",
                  "{\"error\":\"require ?confirm=yes\"}");
      return;
    }
    GLOG_WARN("SYS", "/reboot invoked");
    server.send(200, "application/json", "{\"ok\":true,\"action\":\"reboot\"}");
    delay(200);
    ESP.restart();
  });

  // Out-of-band target-state control. Drives the exact same setter code paths
  // as HomeKit writes (currentState mirror, pending_update debounce, threshold
  // guards, persistence), so it exercises the controller without an iOS
  // device. Same LAN trust model as /reboot; requires ?confirm=yes.
  // Fields: dehumidifier|active|swing (0/1), mode (0 AUTO,1 HEAT,2 COOL),
  // heat|cool (16..31 C). Applied in fixed order dehumidifier, active, mode,
  // heat, cool, swing, so dehumidifier=0&active=1 cannot leave a transient
  // OFF target. All-or-nothing: any invalid value applies nothing.
  server.on("/control", HTTP_GET, []() {
    if (server.arg("confirm") != "yes") {
      server.send(400, "application/json",
                  "{\"error\":\"require ?confirm=yes\"}");
      return;
    }
    if (!homekitStarted) {
      server.send(503, "application/json",
                  "{\"error\":\"homekit not started\"}");
      return;
    }
    // Strict full-string numeric parse. String::toFloat() would turn garbage
    // into 0.0 — a VALID value for the 0/1 fields — so reject instead.
    auto parseNum = [](const String &s, float &v) -> bool {
      if (s.length() == 0) return false;
      char *end = nullptr;
      v = strtof(s.c_str(), &end);
      return end == s.c_str() + s.length() && !isnan(v) && !isinf(v);
    };
    struct CtlField { const char *name; float lo; float hi; bool integer; };
    static const CtlField FIELDS[] = {
        {"dehumidifier", 0, 1, true}, {"active", 0, 1, true},
        {"mode", 0, 2, true},         {"heat", 16, 31, false},
        {"cool", 16, 31, false},      {"swing", 0, 1, true},
    };
    float vals[6];
    bool has[6] = {false, false, false, false, false, false};
    int nset = 0;
    for (int i = 0; i < 6; i++) {
      if (!server.hasArg(FIELDS[i].name)) continue;
      float v;
      if (!parseNum(server.arg(FIELDS[i].name), v) || v < FIELDS[i].lo ||
          v > FIELDS[i].hi || (FIELDS[i].integer && v != (float)(int)v)) {
        server.send(400, "application/json",
                    String("{\"error\":\"bad value for ") + FIELDS[i].name +
                        "\"}");
        return;
      }
      vals[i] = v; has[i] = true; nset++;
    }
    if (nset == 0) {
      server.send(400, "application/json",
                  "{\"error\":\"no recognized field\"}");
      return;
    }
    GLOG_INFO("SYS", "/control applying %d field(s)", nset);
    for (int i = 0; i < 6; i++) {
      if (!has[i]) continue;
      homekit_value_t v;
      memset(&v, 0, sizeof(v));
      if (FIELDS[i].integer) {
        v.format = homekit_format_uint8;
        v.uint8_value = (uint8_t)vals[i];
      } else {
        v.format = homekit_format_float;
        v.float_value = vals[i];
      }
      switch (i) {
        case 0: set_dehumidifier_active(v); break;
        case 1: set_ac_active(v); break;
        case 2: set_ac_target_state(v); break;
        case 3: set_ac_heating_threshold(v); break;
        case 4: set_ac_cooling_threshold(v); break;
        case 5: set_ac_swing_mode(v); break;
      }
    }
    // Echo the resulting target mirrors so guard-dropped writes are visible.
    char hBuf[10], cBuf[10], body[160];
    dtostrf(cha_ac_heating_threshold.value.float_value, 1, 1, hBuf);
    dtostrf(cha_ac_cooling_threshold.value.float_value, 1, 1, cBuf);
    snprintf(body, sizeof(body),
             "{\"ok\":true,\"active\":%u,\"mode\":%u,\"heat\":%s,\"cool\":%s,"
             "\"swing\":%u,\"dehumidifier\":%u}",
             cha_ac_active.value.uint8_value,
             cha_ac_target_state.value.uint8_value, hBuf, cBuf,
             cha_ac_swing_mode.value.uint8_value,
             cha_dehumidifier_active.value.uint8_value);
    server.send(200, "application/json", body);
  });

#ifdef USE_FAKE_HEATPUMP
  // Diagnostic-only endpoints. Inject simulated IR-remote / sensor changes
  // so the controller's external-update path and HomeKit notifications can
  // be exercised without a real Mitsubishi AC.
  server.on("/fake/external", HTTP_GET, []() {
    String power = server.hasArg("power") ? server.arg("power") : String("");
    String mode  = server.hasArg("mode")  ? server.arg("mode")  : String("");
    float temp   = server.hasArg("temp")  ? server.arg("temp").toFloat() : 0.0f;
    gootac_fake_inject_external(
        power.length() ? power.c_str() : nullptr,
        mode.length() ? mode.c_str() : nullptr,
        temp);
    String body = String("{\"ok\":true,\"power\":\"") + power +
                  "\",\"mode\":\"" + mode + "\",\"temp\":" + String(temp, 1) +
                  "}";
    server.send(200, "application/json", body);
  });
  server.on("/fake/temp", HTTP_GET, []() {
    if (!server.hasArg("value")) {
      server.send(400, "application/json", "{\"error\":\"missing value\"}");
      return;
    }
    float v = server.arg("value").toFloat();
    gootac_fake_force_room_temperature(v);
    String body = String("{\"ok\":true,\"room_temp\":") + String(v, 2) + "}";
    server.send(200, "application/json", body);
  });
#endif

  server.begin();
  GLOG_INFO("SYS", "Web server started on port 80");
}

void setup_wifi() {
  WiFi.mode(WIFI_STA);
  // Capture reset info before WiFi clears anything
  String reason = ESP.getResetReason();
  if (reason.indexOf("Exception") != -1 || reason.indexOf("Watchdog") != -1) {
    previousResetInfo =
        "PREVIOUS CRASH: " + reason + " | " + ESP.getResetInfo();
    has_reset_info = true;
    resetReportStart = millis();
  }

  WiFi.hostname(hostName);
  WiFi.begin(ssid, password);
  GLOG_INFO("WIFI", "Connecting to %s...", ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    GLOG_TRACE("WIFI", "Still waiting for connection...");
    ESP.wdtFeed();
  }
  GLOG_INFO("WIFI", "Connected! IP: %s", WiFi.localIP().toString().c_str());
}

void setup_ota() {
  ArduinoOTA.setHostname(hostName);
  ArduinoOTA.onStart([]() { GLOG_INFO("OTA", "Update starting..."); });
  ArduinoOTA.onEnd([]() { GLOG_INFO("OTA", "Update complete! Rebooting..."); });
  ArduinoOTA.onError([](ota_error_t error) {
    GLOG_ERROR("OTA", "Update Error[%u]: %d", error, error);
  });
  ArduinoOTA.begin();

  // Add metadata to the _arduino._tcp service
  MDNS.addServiceTxt("arduino", "tcp", "project", "GootAC");
  MDNS.addServiceTxt("arduino", "tcp", "version", FW_VERSION);
  MDNS.addServiceTxt("arduino", "tcp", "uptime", "0s");

  // NOTE: mDNS TXT values MUST stay in scope or be copied by the library.
  // We use the 60s loop to update dynamic values safely.
  MDNS.addServiceTxt("arduino", "tcp", "rssi", "0");
  MDNS.addServiceTxt(
      "arduino", "tcp", "sdk",
      "v1.0"); // Static for now to avoid SDK string lifetime issues
}

void setup() {
  LittleFS.begin();
  fsLogger.rotate();
  // Initialize logging without the level prefix (the prefix callback will
  // handle it)
#ifndef APP_LOG_LEVEL
#define APP_LOG_LEVEL LOG_LEVEL_VERBOSE
#endif
  Log.begin(APP_LOG_LEVEL, &fsLogger, false);
  Log.setPrefix(printPrefix);

  // Set up SNTP for real-time logs
  configTime(1 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  GLOG_BOOT("--- GootAC Booting ---");

  // Use 160MHz for crypto performance (HomeKit requirement)
  system_update_cpu_freq(160);

  // Derive hostname and accessory name from DEVICE_NAME
  snprintf(hostName, sizeof(hostName), "GootAC-%s", DEVICE_NAME);
  snprintf(accessoryName, sizeof(accessoryName), "%s AC", DEVICE_NAME);

  // Sanitize hostname: replace spaces/invalid chars with dashes
  for (char *p = hostName; *p; p++) {
    if (*p == ' ' || !isalnum(*p)) *p = '-';
  }

  // Serial1 (UART1) is disabled. UART0 is strictly reserved for the AC CN105
  // protocol! No serial logging allowed to preserve AC communication
  // integerity.

  setup_wifi();
  MDNS.begin(hostName);
  setup_ota();
  setup_webserver();

  GLOG_BOOT("GootAC Booting...");
  GLOG_BOOT("Reset reason: %s", ESP.getResetReason().c_str());
  GLOG_BOOT("IP: %s", WiFi.localIP().toString().c_str());

  // Configuration Audit
  GLOG_BOOT("--- CONFIG ---");
  GLOG_BOOT("WIFI_SSID: %s", WIFI_SSID);
  GLOG_BOOT("DEVICE_NAME: %s", DEVICE_NAME);
  GLOG_BOOT("Hostname: %s", hostName);
  GLOG_BOOT("Accessory: %s", accessoryName);
  GLOG_BOOT("FORCE_HK_START: %s", FORCE_HK_START ? "YES" : "NO");
  GLOG_BOOT("--------------");

  GLOG_BOOT("Initial Heap: %d bytes", ESP.getFreeHeap());

  GLOG_INFO("BOOT", "Connecting to AC physical unit (CN105)...");
  hp.connect(&Serial);

  // status-change callback is registered later in ac_controller_init();
  // HeatPump uses a single-slot std::function, so the one set there wins.

  GLOG_BOOT("Setup complete! Entering loop...");
}

void loop() {
  ArduinoOTA.handle();
  MDNS.update();
  server.handleClient();
  yield();

  // Heartbeat & UART Processing (internal 500ms protocol sync)
  hp.loop();

  // Update mDNS diagnostic data every 60s
  if (millis() - lastUptimeUpdate > 60000) {
    lastUptimeUpdate = millis();
    MDNS.addServiceTxt("arduino", "tcp", "uptime", getUptimeString());
    MDNS.addServiceTxt("arduino", "tcp", "rssi", String(WiFi.RSSI()));
  }

  // --- Network Stability Watchdogs ---
  static unsigned long lastWiFiConnected = 0;
  if (WiFi.status() == WL_CONNECTED) {
    lastWiFiConnected = millis();
  } else if (millis() - lastWiFiConnected > 120000) {
    // Wi-Fi lost for > 2 minutes. Reboot to ensure clean state and prevent TCP
    // socket exhaustion.
    GLOG_ERROR("SYS", "CRITICAL: Wi-Fi connection lost for > 120s! Watchdog rebooting...");
    delay(100);
    ESP.restart();
  }


  if (!homekitStarted) {
    bool hasAcData =
        (hp.getRoomTemperature() > 0 && hp.getPowerSetting() != nullptr);
    if (FORCE_HK_START || hasAcData) {
      if (hasAcData) {
        GLOG_INFO("BOOT", "AC Handshake OK! Room=%sC Power=%s", String(hp.getRoomTemperature(), 1).c_str(),
                   hp.getPowerSetting() ? hp.getPowerSetting() : "N/A");
      } else {
        GLOG_WARN("BOOT", "AC DATA MISSING! Starting HomeKit anyway (FORCE_HK_START=YES)");
      }
      GLOG_TRACE("BOOT", "Entering HomeKit initialization...");

      ac_controller_init(&hp);

      GLOG_INFO("BOOT", "HomeKit Server Started!");
      homekitStarted = true;
    } else {
      // Periodic "Still waiting" log every 30s while not connected to AC
      static unsigned long lastWaitingLog = 0;
      if (millis() - lastWaitingLog > 30000) {
        lastWaitingLog = millis();
        GLOG_TRACE("BOOT", "Waiting for physical AC responses on UART...");
      }
    }
  } else {
    ac_controller_loop();

    // Every 60 seconds, report previous crash for 24h visibility
    static unsigned long lastResetReport = 0;
    if (has_reset_info && (millis() - resetReportStart < 86400000UL)) {
      if (millis() - lastResetReport > 60000) {
        GLOG_ERROR("SYS", "PERSISTENT CRASH INFO: %s", previousResetInfo.c_str());
        lastResetReport = millis();
      }
    }
  }
}
