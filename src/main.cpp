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
namespace base64 {
extern String encode(const uint8_t *data, size_t length,
                     bool usePadding = true);
extern String encode(const String &text, bool usePadding = true);
} // namespace base64
#include <ESP8266WebServer.h>
extern "C" {
#include <user_interface.h>
}
#include "ac_controller.h"
#include "fs_logger.h"
#include "homekit_ac.h"
#include <ArduinoLog.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <HeatPump.h>
#include <LittleFS.h>
#include <arduino_homekit_server.h>
#include <stdarg.h>

#include "config.h"

ESP8266WebServer server(80);
char hostName[32];
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;

unsigned long lastHeartbeatTime = 0;
unsigned long lastUptimeUpdate = 0;
unsigned long lastCapabilitiesTime = 0;

HeatPump hp;
bool homekitStarted = false;
bool serial_disabled = false;

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

void logMsg(const char *id, const char *msg) { GLOG_INFO(id, "%s", msg); }

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

  server.on("/status", HTTP_GET, []() {
    extern String ac_controller_get_json_status();
    server.send(200, "application/json", ac_controller_get_json_status());
  });

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
  MDNS.addServiceTxt("arduino", "tcp", "heap", "0");
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

  // Set unique hostname based on MAC if default is used
  if (strcmp(HOST_NAME, "GootAC") == 0) {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    snprintf(hostName, sizeof(hostName), "GootAC-%02X%02X%02X", mac[3], mac[4],
             mac[5]);
  } else {
    strncpy(hostName, HOST_NAME, sizeof(hostName));
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
  GLOG_BOOT("HOST_NAME: %s", HOST_NAME);
  GLOG_BOOT("FORCE_HK_START: %s", FORCE_HK_START ? "YES" : "NO");
  GLOG_BOOT("--------------");

  GLOG_BOOT("Initial Heap: %d bytes", ESP.getFreeHeap());

  GLOG_INFO("BOOT", "Connecting to AC physical unit (CN105)...");
  hp.connect(&Serial);

  // HomeKit Push Updates: Trigger sync immediately when AC data is received
  hp.setStatusChangedCallback([](heatpumpStatus status) {
    if (homekitStarted) {
      GLOG_TRACE("MITSUBISHI", "Status change callback triggered by physical unit");
      ac_controller_sync_from_ac();
    }
  });

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
    MDNS.addServiceTxt("arduino", "tcp", "heap", String(ESP.getFreeHeap()));
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

  // --- Periodic Preventative Maintenance ---
  // Reboot silently after 24 hours to cure memory fragmentation, HomeKit
  // session exhaustion, and mDNS sync issues. We only trigger this if the
  // physical AC isn't doing anything important (e.g., active = 0).
  if (millis() > 86400000UL) { // > 24 hours
    if (homekitStarted && cha_ac_active.value.uint8_value == 0) {
      GLOG_INFO("SYS", "Performing standard 24h maintenance reboot (AC is idle)...");
      delay(100);
      ESP.restart();
    } else if (millis() >
               172800000UL) { // > 48 hours (if the AC never stops running)
      GLOG_WARN("SYS", "Performing mandatory 48h maintenance reboot...");
      delay(100);
      ESP.restart();
    }
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
    homekit_server_t *hk = arduino_homekit_get_running_server();
    if (hk && hk->paired && serial_disabled) {
      GLOG_INFO("BOOT", "Device paired! Restoring Serial communication...");
      hp.connect(&Serial); // This will call Serial.begin() internally
      serial_disabled = false;
    }

    ac_controller_loop();

    // Every 60 seconds, report previous crash for 24h visibility
    static unsigned long lastResetReport = 0;
    if (has_reset_info && (millis() - resetReportStart < 86400000UL)) {
      if (millis() - lastResetReport > 60000) {
        GLOG_ERROR("SYS", "PERSISTENT CRASH INFO: %s", previousResetInfo.c_str());
        lastResetReport = millis();
      }
    }

    // Heartbeat & Status Report (every 10s)
    if (millis() - lastHeartbeatTime > 10000) {
      lastHeartbeatTime = millis();
      if (!serial_disabled) {
        ac_controller_report_status();
      }
    }

  }
}
