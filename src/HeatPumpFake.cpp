// Fake implementation of the HeatPump class for hardware-free HomeKit testing.
// Class declaration in HeatPump.h is unchanged; only this .cpp replaces
// HeatPump.cpp via the d1_mini_fake PlatformIO env's src_filter.
//
// Private-member access trick: ac_controller.cpp calls hp->wasExternalUpdate()
// which is inline in HeatPump.h and reads the private bool. To make the fake
// drive that flag without touching the header, the inject free functions set
// file-static "pending" flags that the class method bodies (which CAN read
// private members because they are member functions) act on the next loop()
// or update().

#include "HeatPump.h"
#include "HeatPumpFake.h"
#include <Arduino.h>
#include <string.h>
#include <math.h>

static HeatPump *g_instance = nullptr;

static const unsigned long ACK_LATENCY_MS = 500;

// Drift state
static unsigned long s_lastRoomTempDriftMs = 0;
static float s_lastNotifiedRoomTemp = NAN;

// update() ack state
static bool s_updatePending = false;
static unsigned long s_updateAckDeadlineMs = 0;

// Injection seams (set by gootac_fake_*; consumed by HeatPump method bodies)
static bool s_pendingExternalFlag = false;
static bool s_pendingRoomTemp = false;
static float s_pendingRoomTempValue = 0.0f;

bool HeatPump::isConnected() { return connected; }

HeatPump::HeatPump() {
  wantedSettings.power = "OFF";
  wantedSettings.mode = "AUTO";
  wantedSettings.temperature = 22.0f;
  wantedSettings.fan = "AUTO";
  wantedSettings.vane = "AUTO";
  wantedSettings.wideVane = "|";
  wantedSettings.iSee = false;

  currentSettings = wantedSettings;

  currentStatus.roomTemperature = 22.0f;
  currentStatus.outsideAirTemperature = 0.0f;
  currentStatus.operating = false;
  currentStatus.timers.mode = "NONE";
  currentStatus.timers.onMinutesSet = 0;
  currentStatus.timers.onMinutesRemaining = 0;
  currentStatus.timers.offMinutesSet = 0;
  currentStatus.timers.offMinutesRemaining = 0;
  currentStatus.statusByte3 = 0;
  currentStatus.statusFlags = 0;
  currentStatus.actualFanSpeed = 0;
  currentStatus.autoSubMode = 0;

  _HardSerial = nullptr;
  lastSend = 0;
  waitForRead = false;
  infoMode = 0;
  lastRecv = 0;
  connected = false;
  autoUpdate = false;
  firstRun = true;
  tempMode = false;
  externalUpdate = false;
  _externalUpdateOccurred = false;
  wideVaneAdj = false;

  onConnectCallback = nullptr;
  settingsChangedCallback = nullptr;
  statusChangedCallback = nullptr;
  packetCallback = nullptr;
  roomTempChangedCallback = nullptr;
}

bool HeatPump::connect(HardwareSerial *serial) {
  _HardSerial = serial;
  connected = true;
  g_instance = this;
  if (onConnectCallback) onConnectCallback();
  if (settingsChangedCallback) settingsChangedCallback();
  if (statusChangedCallback) statusChangedCallback(currentStatus);
  s_lastRoomTempDriftMs = millis();
  s_lastNotifiedRoomTemp = currentStatus.roomTemperature;
  return true;
}

bool HeatPump::connect(HardwareSerial *serial, bool /*retry*/) {
  return connect(serial);
}

bool HeatPump::update() {
  // If an inject helper queued an external-flag, set it now (we are inside a
  // member function, so private members are accessible).
  if (s_pendingExternalFlag) {
    _externalUpdateOccurred = true;
    s_pendingExternalFlag = false;
  }
  s_updatePending = true;
  s_updateAckDeadlineMs = millis() + ACK_LATENCY_MS;
  return true;
}

void HeatPump::sync(byte /*packetType*/) {
  if (settingsChangedCallback) settingsChangedCallback();
  if (statusChangedCallback) statusChangedCallback(currentStatus);
}

void HeatPump::enableExternalUpdate() { externalUpdate = true; }
void HeatPump::enableAutoUpdate() { autoUpdate = true; }
void HeatPump::disableAutoUpdate() { autoUpdate = false; }

heatpumpSettings HeatPump::getSettings() { return currentSettings; }
heatpumpSettings HeatPump::getWantedSettings() { return wantedSettings; }

void HeatPump::setSettings(heatpumpSettings settings) {
  wantedSettings = settings;
}

void HeatPump::setPowerSetting(bool setting) {
  wantedSettings.power = setting ? "ON" : "OFF";
}

void HeatPump::setPowerSetting(const char *setting) {
  if (setting) wantedSettings.power = setting;
}

bool HeatPump::getPowerSettingBool() {
  return currentSettings.power && strcmp(currentSettings.power, "ON") == 0;
}

const char *HeatPump::getPowerSetting() { return currentSettings.power; }
const char *HeatPump::getModeSetting() { return currentSettings.mode; }

void HeatPump::setModeSetting(const char *setting) {
  if (setting) wantedSettings.mode = setting;
}

void HeatPump::setModeIndex(uint8_t modeIndex) {
  if (modeIndex < sizeof(MODE_MAP) / sizeof(MODE_MAP[0])) {
    wantedSettings.mode = MODE_MAP[modeIndex];
  }
}

float HeatPump::getTemperature() { return currentSettings.temperature; }
void HeatPump::setTemperature(float setting) {
  wantedSettings.temperature = setting;
}

void HeatPump::setRemoteTemperature(float /*setting*/) { /* no-op */ }

const char *HeatPump::getFanSpeed() { return currentSettings.fan; }
void HeatPump::setFanSpeed(const char *setting) {
  if (setting) wantedSettings.fan = setting;
}
void HeatPump::setFanSpeedIndex(uint8_t fanIndex) {
  if (fanIndex < sizeof(FAN_MAP) / sizeof(FAN_MAP[0])) {
    wantedSettings.fan = FAN_MAP[fanIndex];
  }
}

const char *HeatPump::getVaneSetting() { return currentSettings.vane; }
void HeatPump::setVaneSetting(const char *setting) {
  if (setting) wantedSettings.vane = setting;
}
void HeatPump::setVaneIndex(uint8_t vaneIndex) {
  if (vaneIndex < sizeof(VANE_MAP) / sizeof(VANE_MAP[0])) {
    wantedSettings.vane = VANE_MAP[vaneIndex];
  }
}

const char *HeatPump::getWideVaneSetting() { return currentSettings.wideVane; }
void HeatPump::setWideVaneSetting(const char *setting) {
  if (setting) wantedSettings.wideVane = setting;
}
void HeatPump::setWideVaneIndex(uint8_t wideVaneIndex) {
  if (wideVaneIndex < sizeof(WIDEVANE_MAP) / sizeof(WIDEVANE_MAP[0])) {
    wantedSettings.wideVane = WIDEVANE_MAP[wideVaneIndex];
  }
}

bool HeatPump::getIseeBool() { return currentSettings.iSee; }

heatpumpStatus HeatPump::getStatus() { return currentStatus; }
float HeatPump::getRoomTemperature() { return currentStatus.roomTemperature; }
bool HeatPump::getOperating() { return currentStatus.operating; }

float HeatPump::FahrenheitToCelsius(int tempF) {
  return (tempF - 32.0f) * 5.0f / 9.0f;
}
int HeatPump::CelsiusToFahrenheit(float tempC) {
  return (int)(tempC * 9.0f / 5.0f + 32.0f);
}

void HeatPump::setOnConnectCallback(ON_CONNECT_CALLBACK_SIGNATURE) {
  this->onConnectCallback = onConnectCallback;
}
void HeatPump::setSettingsChangedCallback(SETTINGS_CHANGED_CALLBACK_SIGNATURE) {
  this->settingsChangedCallback = settingsChangedCallback;
}
void HeatPump::setStatusChangedCallback(STATUS_CHANGED_CALLBACK_SIGNATURE) {
  this->statusChangedCallback = statusChangedCallback;
}
void HeatPump::setPacketCallback(PACKET_CALLBACK_SIGNATURE) {
  this->packetCallback = packetCallback;
}
void HeatPump::setRoomTempChangedCallback(ROOM_TEMP_CHANGED_CALLBACK_SIGNATURE) {
  this->roomTempChangedCallback = roomTempChangedCallback;
}

void HeatPump::sendCustomPacket(byte * /*data*/, int /*len*/) { /* no-op */ }

void HeatPump::loop() {
  if (!connected) return;
  unsigned long now = millis();

  // Apply pending forced room temperature (from /fake/temp).
  if (s_pendingRoomTemp) {
    currentStatus.roomTemperature = s_pendingRoomTempValue;
    s_pendingRoomTemp = false;
    s_lastNotifiedRoomTemp = currentStatus.roomTemperature;
    if (statusChangedCallback) statusChangedCallback(currentStatus);
    if (roomTempChangedCallback)
      roomTempChangedCallback(currentStatus.roomTemperature);
  }

  // Ack pending update after ACK_LATENCY_MS.
  if (s_updatePending && (long)(now - s_updateAckDeadlineMs) >= 0) {
    s_updatePending = false;
    bool changed = (currentSettings.power != wantedSettings.power) ||
                   (currentSettings.mode != wantedSettings.mode) ||
                   (currentSettings.temperature != wantedSettings.temperature) ||
                   (currentSettings.fan != wantedSettings.fan) ||
                   (currentSettings.vane != wantedSettings.vane) ||
                   (currentSettings.wideVane != wantedSettings.wideVane);
    currentSettings = wantedSettings;
    currentStatus.operating =
        currentSettings.power && strcmp(currentSettings.power, "ON") == 0 &&
        currentSettings.mode && strcmp(currentSettings.mode, "FAN") != 0;
    if (changed && settingsChangedCallback) settingsChangedCallback();
    if (statusChangedCallback) statusChangedCallback(currentStatus);
  }

  // Drift roomTemperature based on current mode.
  unsigned long elapsedMs = now - s_lastRoomTempDriftMs;
  if (elapsedMs >= 1000) {
    float seconds = elapsedMs / 1000.0f;
    s_lastRoomTempDriftMs = now;
    float rate = 0.0f;
    float target = currentStatus.roomTemperature;
    bool poweredOn = currentSettings.power &&
                     strcmp(currentSettings.power, "ON") == 0;
    if (poweredOn && currentSettings.mode) {
      if (strcmp(currentSettings.mode, "HEAT") == 0 ||
          strcmp(currentSettings.mode, "COOL") == 0) {
        rate = 0.05f;
        target = currentSettings.temperature;
      }
    } else {
      rate = 0.02f;
      target = 22.0f;
    }
    if (rate > 0.0f && currentStatus.roomTemperature != target) {
      float delta = rate * seconds;
      if (currentStatus.roomTemperature < target) {
        currentStatus.roomTemperature += delta;
        if (currentStatus.roomTemperature > target)
          currentStatus.roomTemperature = target;
      } else {
        currentStatus.roomTemperature -= delta;
        if (currentStatus.roomTemperature < target)
          currentStatus.roomTemperature = target;
      }
    }
    if (isnan(s_lastNotifiedRoomTemp) ||
        fabsf(currentStatus.roomTemperature - s_lastNotifiedRoomTemp) >= 0.1f) {
      s_lastNotifiedRoomTemp = currentStatus.roomTemperature;
      if (statusChangedCallback) statusChangedCallback(currentStatus);
      if (roomTempChangedCallback)
        roomTempChangedCallback(currentStatus.roomTemperature);
    }
  }
}

// ---- HeatPump.h declares these free functions; provide stubs so the linker
// is satisfied even if the controller never calls them.
bool operator==(const heatpumpSettings &lhs, const heatpumpSettings &rhs) {
  return lhs.power == rhs.power && lhs.mode == rhs.mode &&
         lhs.temperature == rhs.temperature && lhs.fan == rhs.fan &&
         lhs.vane == rhs.vane && lhs.wideVane == rhs.wideVane &&
         lhs.iSee == rhs.iSee;
}
bool operator!=(const heatpumpSettings &lhs, const heatpumpSettings &rhs) {
  return !(lhs == rhs);
}
bool operator!(const heatpumpSettings &settings) {
  return settings.power == nullptr && settings.mode == nullptr;
}
bool operator==(const heatpumpTimers &lhs, const heatpumpTimers &rhs) {
  return lhs.mode == rhs.mode && lhs.onMinutesSet == rhs.onMinutesSet &&
         lhs.onMinutesRemaining == rhs.onMinutesRemaining &&
         lhs.offMinutesSet == rhs.offMinutesSet &&
         lhs.offMinutesRemaining == rhs.offMinutesRemaining;
}
bool operator!=(const heatpumpTimers &lhs, const heatpumpTimers &rhs) {
  return !(lhs == rhs);
}

// ---- Inject helpers driven by main.cpp /fake/* HTTP routes

void gootac_fake_inject_external(const char *power, const char *mode,
                                 float temperature) {
  if (!g_instance) return;
  heatpumpSettings s = g_instance->getSettings();
  if (power && *power) s.power = (strcmp(power, "ON") == 0) ? "ON" : "OFF";
  if (mode && *mode) {
    // Normalize to one of the canonical MODE_MAP strings.
    if (strcmp(mode, "HEAT") == 0) s.mode = "HEAT";
    else if (strcmp(mode, "COOL") == 0) s.mode = "COOL";
    else if (strcmp(mode, "AUTO") == 0) s.mode = "AUTO";
    else if (strcmp(mode, "DRY")  == 0) s.mode = "DRY";
    else if (strcmp(mode, "FAN")  == 0) s.mode = "FAN";
  }
  if (temperature > 0) s.temperature = temperature;
  g_instance->setSettings(s);
  // Mark the next update() as external-origin so wasExternalUpdate() trips.
  s_pendingExternalFlag = true;
  g_instance->update();
}

void gootac_fake_force_room_temperature(float t) {
  s_pendingRoomTempValue = t;
  s_pendingRoomTemp = true;
}
