#include "ac_controller.h"
#include "homekit_ac.h"
#include "config.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoLog.h>
#include <LittleFS.h>
#include <WiFiClient.h>
#include <arduino_homekit_server.h>

extern "C" {
#include <lwip/tcp.h>
#include <lwip/priv/tcp_priv.h>
}

// Plausibility bounds on the room-temperature sensor reading. The Mitsubishi
// CN105 protocol reports 10–41°C in 1°C steps via the standard packet plus
// half-degree extended packets, so anything outside [MIN, MAX] is a sensor
// glitch or parse error and must not be pushed to HomeKit. Defaults match
// Lisbon climate; override in config.h.
#ifndef MIN_VALID_ROOM_TEMP_C
#define MIN_VALID_ROOM_TEMP_C 10.0f
#endif
#ifndef MAX_VALID_ROOM_TEMP_C
#define MAX_VALID_ROOM_TEMP_C 45.0f
#endif

// Returns info about the current HomeKit client performing an action
static String get_client_info() {
  client_context_t *ctx = (client_context_t *)homekit_get_client_id();
  if (!ctx || !ctx->socket)
    return "System/Local";

  IPAddress ip = ctx->socket->remoteIP();
  int pairing_id = ctx->pairing_id;

  char buf[64];
  snprintf(buf, sizeof(buf), "%s (ID:%d)", ip.toString().c_str(), pairing_id);
  return String(buf);
}

// External specialized HK log implementation
void hk_log_info(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf_P(buf, sizeof(buf), fmt, args);
  va_end(args);

  Log.infoln(F("[HOMEKIT] %s (Client: %s)"), buf, get_client_info().c_str());
}

static HeatPump *hp;
static TargetState currentState;
static const char *STATE_FILE = "/target_state.bin";

// Helper: Calculate checksum for binary integrity
uint32_t calculate_checksum(TargetState *ts) {
  uint32_t sum = 0;
  uint8_t *bytes = (uint8_t *)ts;
  for (size_t i = 0; i < sizeof(TargetState) - 4; i++) {
    sum += bytes[i];
  }
  return sum;
}

// Save and Load binary state
void save_target_state() {
  File f = LittleFS.open(STATE_FILE, "w");
  if (f) {
    currentState.checksum = calculate_checksum(&currentState);
    f.write((uint8_t *)&currentState, sizeof(TargetState));
    f.close();
    char cBuf[10], hBuf[10];
    dtostrf(currentState.cooling_threshold, 1, 1, cBuf);
    dtostrf(currentState.heating_threshold, 1, 1, hBuf);
    GLOG_INFO("SYS", "Target State saved to LittleFS (Active: %d, Mode: %d, C: %s, H: %s)", 
              currentState.active, currentState.target_mode, cBuf, hBuf);
  } else {
    GLOG_ERROR("SYS", "Failed to open Target State for writing!");
  }
}

void load_target_state() {
  if (!LittleFS.exists(STATE_FILE)) {
    GLOG_INFO("SYS", "No state file found. Initializing fresh defaults.");
    currentState.active = 0;
    currentState.target_mode = 0; // AUTO
    currentState.cooling_threshold = 28.0;
    currentState.heating_threshold = 22.0;
    currentState.fan_mode = 1; // AUTO
    currentState.fan_speed = 0;
    currentState.swing_mode = 0;
    currentState.dehumidifier = 0;
    save_target_state();

    return;
  }

  File f = LittleFS.open(STATE_FILE, "r");
  if (f) {
    TargetState loaded;
    if (f.readBytes((char *)&loaded, sizeof(TargetState)) == sizeof(TargetState)) {
      uint32_t check = calculate_checksum(&loaded);
      if (check == loaded.checksum) {
        currentState = loaded;
        char cBuf[10], hBuf[10];
        dtostrf(currentState.cooling_threshold, 1, 1, cBuf);
        dtostrf(currentState.heating_threshold, 1, 1, hBuf);
        GLOG_INFO("SYS", "Target State loaded successfully (Active: %d, Mode: %d, C: %s, H: %s)", 
                  currentState.active, currentState.target_mode, cBuf, hBuf);
      } else {
        GLOG_WARN("SYS", "Target State Checksum Mismatch! Using default values.");
      }
    } else {
      GLOG_ERROR("SYS", "Persistence Error: Read truncated Target State! Using defaults.");
    }
    f.close();
  }
}

static bool pending_update = false;
static bool pending_sync_request = false;

// Timing and guards
static unsigned long lastSetTime = 0;
static unsigned long lastInteractionTime = 0;
static unsigned long lastHeatPushTime = 0;
static unsigned long lastCoolPushTime = 0;
static bool identify_active = false;
static unsigned long identify_start = 0;

// Helpers
void update_state(const char *reason) {
  GLOG_TRACE("MITSUBISHI", "State update requested. Reason: %s", reason);
  pending_update = true;
  lastInteractionTime = millis();
}

// ----------------------------------------------------
// HomeKit -> AC (Setters)
// ----------------------------------------------------

void set_ac_active(homekit_value_t value) {
  if (value.format != homekit_format_uint8) return;
  cha_ac_active.value = value;
  currentState.active = value.uint8_value;
  HKLOG_INFO("Characteristic Set Active -> %d", value.uint8_value);

  if (value.uint8_value == 1) {
    if (cha_dehumidifier_active.value.uint8_value == 1) {
      GLOG_TRACE("HOMEKIT", "Auto-disabling Dehumidifier to prevent mode conflict");
      cha_dehumidifier_active.value.uint8_value = 0;
      currentState.dehumidifier = 0;
      homekit_characteristic_notify(&cha_dehumidifier_active, cha_dehumidifier_active.value);
    }
  }
  update_state("HomeKit Active change");
}

void set_ac_target_state(homekit_value_t value) {
  if (value.format != homekit_format_uint8) return;
  cha_ac_target_state.value = value;
  currentState.target_mode = value.uint8_value;
  HKLOG_INFO("Characteristic Set Target State -> %u", value.uint8_value);
  update_state("HomeKit Target State change");
}

void set_ac_cooling_threshold(homekit_value_t value) {
  if (value.format != homekit_format_float) return;
  unsigned long now = millis();
  if (now - lastHeatPushTime < 1000) return;

  cha_ac_cooling_threshold.value = value;
  currentState.cooling_threshold = value.float_value;
  
  char tempBuf[10];
  dtostrf(value.float_value, 1, 1, tempBuf);
  HKLOG_INFO("Characteristic Set Cooling Threshold -> %s", tempBuf);
  homekit_characteristic_notify(&cha_ac_cooling_threshold, cha_ac_cooling_threshold.value);

  float min_gap = 1.0;
  if (value.float_value < cha_ac_heating_threshold.value.float_value + min_gap) {
    char tempBuf[10];
    dtostrf(cha_ac_heating_threshold.value.float_value, 1, 1, tempBuf);
    GLOG_TRACE("MITSUBISHI", "Threshold Gap Guard: Pushing Heating Threshold down to %s", tempBuf);
    
    currentState.heating_threshold = cha_ac_heating_threshold.value.float_value;
    lastCoolPushTime = now;
    homekit_characteristic_notify(&cha_ac_heating_threshold, cha_ac_heating_threshold.value);
  }
  update_state("HomeKit Cooling Threshold change");
}

void set_ac_heating_threshold(homekit_value_t value) {
  if (value.format != homekit_format_float) return;
  unsigned long now = millis();
  if (now - lastCoolPushTime < 1000) return;

  cha_ac_heating_threshold.value = value;
  currentState.heating_threshold = value.float_value;

  char tempBuf[10];
  dtostrf(value.float_value, 1, 1, tempBuf);
  HKLOG_INFO("Characteristic Set Heating Threshold -> %s", tempBuf);
  homekit_characteristic_notify(&cha_ac_heating_threshold, cha_ac_heating_threshold.value);

  float min_gap = 1.0;
  if (value.float_value > cha_ac_cooling_threshold.value.float_value - min_gap) {
    char tempBuf[10];
    dtostrf(cha_ac_cooling_threshold.value.float_value, 1, 1, tempBuf);
    GLOG_TRACE("MITSUBISHI", "Threshold Gap Guard: Pushing Cooling Threshold up to %s", tempBuf);

    currentState.cooling_threshold = cha_ac_cooling_threshold.value.float_value;
    lastHeatPushTime = now;
    homekit_characteristic_notify(&cha_ac_cooling_threshold, cha_ac_cooling_threshold.value);
  }
  update_state("HomeKit Heating Threshold change");
}

void set_ac_rotation_speed(homekit_value_t value) {
  if (value.format != homekit_format_float) return;
  cha_ac_rotation_speed.value = value;
  currentState.fan_speed = value.float_value;

  if (value.float_value == 0) {
    currentState.fan_mode = 1; // AUTO
    if (cha_ac_target_fan_state.value.uint8_value != 1) {
      cha_ac_target_fan_state.value.uint8_value = 1;
      homekit_characteristic_notify(&cha_ac_target_fan_state, cha_ac_target_fan_state.value);
    }
  } else {
    currentState.fan_mode = 0; // MANUAL
    if (cha_ac_target_fan_state.value.uint8_value != 0) {
      cha_ac_target_fan_state.value.uint8_value = 0;
      homekit_characteristic_notify(&cha_ac_target_fan_state, cha_ac_target_fan_state.value);
    }
  }
  char tempBuf[10];
  dtostrf(value.float_value, 1, 1, tempBuf);
  HKLOG_INFO("Characteristic Set Rotation Speed -> %s%%", tempBuf);
  update_state("HomeKit Fan Speed change");
}

void set_ac_target_fan_state(homekit_value_t value) {
  if (value.format != homekit_format_uint8) return;
  cha_ac_target_fan_state.value = value;
  currentState.fan_mode = (value.uint8_value == 1) ? 1 : 0;

  if (value.uint8_value == 1) { // Auto
    if (cha_ac_rotation_speed.value.float_value != 0) {
      cha_ac_rotation_speed.value.float_value = 0;
      currentState.fan_speed = 0;
      homekit_characteristic_notify(&cha_ac_rotation_speed, cha_ac_rotation_speed.value);
    }
  }
  HKLOG_INFO("Characteristic Set Target Fan State -> %u (%s)", value.uint8_value, (value.uint8_value == 1) ? "AUTO" : "MANUAL");
  update_state("HomeKit Fan Mode change");
}

void set_ac_swing_mode(homekit_value_t value) {
  if (value.format != homekit_format_uint8) return;
  cha_ac_swing_mode.value = value;
  currentState.swing_mode = value.uint8_value;
  HKLOG_INFO("Characteristic Set Swing Mode -> %u", value.uint8_value);
  update_state("HomeKit Swing change");
}

void set_dehumidifier_active(homekit_value_t value) {
  cha_dehumidifier_active.value = value;
  currentState.dehumidifier = value.uint8_value;
  HKLOG_INFO("Characteristic Set Dehumidifier Active -> %u", value.uint8_value);

  if (value.uint8_value == 1) {
    if (cha_ac_active.value.uint8_value == 1) {
      GLOG_TRACE("HOMEKIT", "Auto-disabling HeaterCooler to prevent mode conflict");
      cha_ac_active.value.uint8_value = 0;
      currentState.active = 0;
      homekit_characteristic_notify(&cha_ac_active, cha_ac_active.value);
    }
  }
  update_state("HomeKit Dehumidifier Active change");
}

// --- Deprecated Fan Mode removed ---

// ----------------------------------------------------
// Initialization & Loop
// ----------------------------------------------------

void ac_controller_init(HeatPump *heatPumpInstance) {
  hp = heatPumpInstance;

  // Set accessory display names from runtime-derived accessoryName
  extern char accessoryName[];
  cha_name.value.string_value = accessoryName;
  cha_conf_name.value.string_value = accessoryName;

  load_target_state();

  cha_ac_active.value.uint8_value = currentState.active;
  cha_ac_target_state.value.uint8_value = currentState.target_mode;
  cha_ac_cooling_threshold.value.float_value = currentState.cooling_threshold;
  cha_ac_heating_threshold.value.float_value = currentState.heating_threshold;
  cha_ac_target_fan_state.value.uint8_value = currentState.fan_mode;
  cha_ac_rotation_speed.value.float_value = currentState.fan_speed;
  cha_ac_swing_mode.value.uint8_value = currentState.swing_mode;
  cha_dehumidifier_active.value.uint8_value = currentState.dehumidifier;

  cha_ac_active.setter = set_ac_active;
  cha_ac_target_state.setter = set_ac_target_state;
  cha_ac_cooling_threshold.setter = set_ac_cooling_threshold;
  cha_ac_heating_threshold.setter = set_ac_heating_threshold;
  cha_ac_target_fan_state.setter = set_ac_target_fan_state;
  cha_ac_rotation_speed.setter = set_ac_rotation_speed;
  cha_ac_swing_mode.setter = set_ac_swing_mode;
  cha_dehumidifier_active.setter = set_dehumidifier_active;

  // Enable IR remote change detection in the library
  hp->enableExternalUpdate();

  // Set library callbacks for reactive updates
  hp->setSettingsChangedCallback([]() { 
    ac_controller_sync_from_ac(); 
  });
  
  hp->setStatusChangedCallback([](heatpumpStatus status) {
    ac_controller_sync_from_ac();
  });

  arduino_homekit_setup(&config);
}

void update_physical_ac() {
  if (!hp || !hp->isConnected()) return;

  bool target_active = (currentState.active == 1);
  uint8_t hk_target_mode = currentState.target_mode; // 0:Auto, 1:Heat, 2:Cool
  float heatThr = currentState.heating_threshold;
  float coolThr = currentState.cooling_threshold;
  float roomTemp = hp->getRoomTemperature();
  bool dehumidify = (currentState.dehumidifier == 1);

  bool hpPower = false;
  uint8_t hpMode = 0; // Library Index: 0:HEAT, 1:DRY, 2:COOL, 3:FAN, 4:AUTO
  float hpTemp = 21.0;

  // Decision Logic
  if (dehumidify) {
    hpPower = true; hpMode = 1; // DRY
  } else if (target_active) {
    if (hk_target_mode == 1) { // HEAT
      hpPower = true; hpMode = 0; hpTemp = heatThr;
    } else if (hk_target_mode == 2) { // COOL
      hpPower = true; hpMode = 2; hpTemp = coolThr;
    } else if (hk_target_mode == 0) { // Smart Auto
      if (roomTemp > 1.0) {
        if (roomTemp < heatThr) {
          hpPower = true; hpMode = 0; hpTemp = heatThr;
        } else if (roomTemp > coolThr) {
          hpPower = true; hpMode = 2; hpTemp = coolThr;
        } else {
          hpPower = true; hpMode = 3; // FAN: circulate air while in deadband
        }
      } else {
        hpPower = true; hpMode = 4; // Native AUTO if room temp unknown
      }
    }
  }

  // --- Decision Logging (Rationale) ---
  char rationale[128] = {0};
  char tempBuf[10];
  
  if (dehumidify) {
    strncpy_P(rationale, PSTR("Logic: Dehumidifier ACTIVE -> DRY"), sizeof(rationale)-1);
  } else if (!target_active) {
    strncpy_P(rationale, PSTR("Logic: Target OFF"), sizeof(rationale)-1);
  } else if (hk_target_mode == 1) { // HEAT
    dtostrf(hpTemp, 1, 1, tempBuf);
    snprintf_P(rationale, sizeof(rationale), PSTR("Logic: HEAT mode (Target %sC)"), tempBuf);
  } else if (hk_target_mode == 2) { // COOL
    dtostrf(hpTemp, 1, 1, tempBuf);
    snprintf_P(rationale, sizeof(rationale), PSTR("Logic: COOL mode (Target %sC)"), tempBuf);
  } else {
    dtostrf(roomTemp, 1, 1, tempBuf);
    snprintf_P(rationale, sizeof(rationale), PSTR("Logic: Smart Auto decision based on Room %sC"), tempBuf);
  }

  static char lastRationale[128] = {0};
  if (strcmp(lastRationale, rationale) != 0) {
    GLOG_TRACE("MITSUBISHI", "%s", rationale);
    strncpy(lastRationale, rationale, sizeof(lastRationale)-1);
  }

  // Identify Override
  if (identify_active) {
    hpPower = true; 
    hp->setModeIndex(3); // FAN
    hp->setVaneIndex(6); // SWING
    hp->setFanSpeedIndex(5); // 4 (Max)
  } else {
    hp->setVaneIndex(currentState.swing_mode == 1 ? 6 : 0); // 6:SWING, 0:AUTO
    if (currentState.fan_mode == 1) hp->setFanSpeedIndex(0); // AUTO
    else {
      int speed = (int)currentState.fan_speed;
      if (speed <= 20) hp->setFanSpeedIndex(1); // QUIET
      else if (speed <= 40) hp->setFanSpeedIndex(2); // 1
      else if (speed <= 60) hp->setFanSpeedIndex(3); // 2
      else if (speed <= 80) hp->setFanSpeedIndex(4); // 3
      else hp->setFanSpeedIndex(5); // 4
    }
  }

  unsigned long now = millis();
  bool changed = false;
  heatpumpSettings wanted = hp->getWantedSettings();

  // --- Physical Hardware Commands ---
  // Compare against library 'wanted' settings to avoid redundant packets while waiting for AC sync
  if (strcmp(wanted.power, hpPower ? "ON" : "OFF") != 0) {
    GLOG_INFO("MITSUBISHI", "COMMAND: Power -> %s (Currently: %s, Wanted: %s)", 
              hpPower ? "ON" : "OFF", hp->getPowerSettingBool() ? "ON" : "OFF", wanted.power);
    hp->setPowerSetting(hpPower);
    lastSetTime = now; changed = true;
  }
  
  if (hpPower) {
    // Mode comparison using library side-effects
    if (strcmp(wanted.mode, hp->MODE_MAP[hpMode]) != 0) {
       GLOG_INFO("MITSUBISHI", "COMMAND: Mode -> %s (Currently: %s, Wanted: %s)", 
                 hp->MODE_MAP[hpMode], hp->getModeSetting(), wanted.mode);
       hp->setModeIndex(hpMode);
       lastSetTime = now; changed = true;
    }
    
    // Temp comparison
    if (hpMode != 3 && abs(hpTemp - wanted.temperature) > 0.1) {
      char tempBuf[10];
      dtostrf(hpTemp, 1, 1, tempBuf);
      GLOG_INFO("MITSUBISHI", "COMMAND: Temp -> %sC", tempBuf);
      hp->setTemperature(hpTemp);
      lastSetTime = now; changed = true;
    }
  }

  if (changed || pending_update) {
    pending_update = false;
    GLOG_INFO("MITSUBISHI", "Updating physical AC unit...");
    hp->update();
    save_target_state();
    if (changed) pending_sync_request = true;
  }
}

void ac_controller_loop() {
  arduino_homekit_loop();
  yield();

  unsigned long now = millis();
  static unsigned long lastUpdate = 0;
  bool debounce_ready = (pending_update && (now - lastInteractionTime > 3000));
  bool periodic_ready = (!pending_update && (now - lastUpdate > 5000));

  if (debounce_ready || periodic_ready) {
    if (identify_active && (now - identify_start > 10000)) identify_active = false;
    update_physical_ac();
    lastUpdate = millis();

    if (pending_sync_request && hp && hp->isConnected()) {
      hp->sync(hp->RQST_PKT_SETTINGS);
      pending_sync_request = false;
    }
  }
}

void ac_controller_sync_from_ac() {
  if (!hp || !hp->isConnected())
    return;

  bool isExternal = hp->wasExternalUpdate();
  heatpumpSettings s = hp->getSettings();

  // 1. Update Current Temperature (Always)
  float roomTemp = hp->getRoomTemperature();
  if (cha_ac_current_temp.value.float_value != roomTemp &&
      roomTemp >= MIN_VALID_ROOM_TEMP_C &&
      roomTemp <= MAX_VALID_ROOM_TEMP_C) {
    cha_ac_current_temp.value.float_value = roomTemp;
    homekit_characteristic_notify(&cha_ac_current_temp,
                                  cha_ac_current_temp.value);
  } else if (roomTemp < MIN_VALID_ROOM_TEMP_C ||
             roomTemp > MAX_VALID_ROOM_TEMP_C) {
    char buf[12], minBuf[10], maxBuf[10];
    dtostrf(roomTemp, 1, 2, buf);
    dtostrf(MIN_VALID_ROOM_TEMP_C, 1, 1, minBuf);
    dtostrf(MAX_VALID_ROOM_TEMP_C, 1, 1, maxBuf);
    GLOG_TRACE("MITSUBISHI",
               "Ignoring out-of-range room temp %sC (valid %s..%s)", buf,
               minBuf, maxBuf);
  }

  // 2. Handle External Overrides (IR Remote / Physical Buttons)
  if (isExternal) {
    GLOG_INFO("MITSUBISHI", "External interaction detected! Syncing Intent...");

    // Sync Power -> Active
    uint8_t physActive = (strcmp(s.power, "ON") == 0) ? 1 : 0;
    if (currentState.active != physActive) {
      currentState.active = physActive;
      cha_ac_active.value.uint8_value = physActive;
      homekit_characteristic_notify(&cha_ac_active, cha_ac_active.value);
    }

    // Sync Mode -> Target State
    // Hardware: HEAT=0, DRY=1, COOL=2, FAN=3, AUTO=4
    // HomeKit: AUTO=0, HEAT=1, COOL=2
    uint8_t hkTargetMode = 0; // Default to AUTO
    if (strcmp(s.mode, "HEAT") == 0) hkTargetMode = 1;
    else if (strcmp(s.mode, "COOL") == 0) hkTargetMode = 2;
    else if (strcmp(s.mode, "DRY") == 0) {
        // Option: Sync DRY to COOL if preferred, or leave as AUTO
        hkTargetMode = 2; 
    }

    if (currentState.target_mode != hkTargetMode) {
      currentState.target_mode = hkTargetMode;
      cha_ac_target_state.value.uint8_value = hkTargetMode;
      homekit_characteristic_notify(&cha_ac_target_state,
                                    cha_ac_target_state.value);
    }

    save_target_state();
  }

  // 3. Update Current State (The "Status Light" logic)
  // 0: INACTIVE, 1: IDLE, 2: HEATING, 3: COOLING
  uint8_t current_state = 0;

  if (cha_ac_active.value.uint8_value == 0) {
    current_state = 0; // INACTIVE
  } else {
    // Unit is Active in HomeKit. Decide if it's Cooling, Heating, or Idle.
    bool physOn = (strcmp(s.power, "ON") == 0);
    
    if (!physOn) {
      current_state = 1; // IDLE (Deadband)
    } else {
      if (strcmp(s.mode, "HEAT") == 0) current_state = 2;
      else if (strcmp(s.mode, "COOL") == 0) current_state = 3;
      else current_state = 1; // FAN/DRY/AUTO(wait) -> IDLE
    }
  }

  if (cha_ac_current_state.value.uint8_value != current_state) {
    GLOG_INFO("MITSUBISHI", "Status Change: %d -> %d (%s)",
              cha_ac_current_state.value.uint8_value, current_state,
              current_state == 3 ? "Cooling" : current_state == 2 ? "Heating" : current_state == 1 ? "Idle" : "Off");
    cha_ac_current_state.value.uint8_value = current_state;
    homekit_characteristic_notify(&cha_ac_current_state,
                                  cha_ac_current_state.value);
  }

  // 4. Update Dehumidifier Current State (0:Inactive, 1:Idle, 3:Dehumidifying)
  //
  // "Dehumidifying" is reported when (a) the dehumidifier service is
  // active, (b) the AC is in DRY mode, and (c) the indoor coil is
  // running cold (causing condensation). The coil-temp threshold comes
  // from byte (compressorFrequency + 10) °C: <= 18°C means the coil is
  // pulling water out of the air. See docs/byte_resolution_2026-05-26.md
  // in the CN105Sniffer repo for the evidence chain.
  uint8_t dehum_state = 0; // 0 = Inactive (service off)
  if (cha_dehumidifier_active.value.uint8_value == 1) {
    bool physOn = (strcmp(s.power, "ON") == 0);
    bool inDry  = (strcmp(s.mode, "DRY") == 0);
    int coilByte = hp->getStatus().compressorFrequency;
    // Coil "actively cold" = byte 1..8 → ~11..18°C. Byte = 0 means the
    // probe is reporting the floor / not contributing, which we treat
    // as "not actively dehumidifying" — DRY just started, coil hasn't
    // condensed yet, or the unit is in HEAT/standby.
    bool coilCold = (coilByte >= 1) && (coilByte <= 8);
    if (physOn && inDry && coilCold) {
      dehum_state = 3; // Dehumidifying
    } else {
      dehum_state = 1; // Idle (service is on but not actively pulling moisture)
    }
  }
  if (cha_dehumidifier_current_state.value.uint8_value != dehum_state) {
    cha_dehumidifier_current_state.value.uint8_value = dehum_state;
    homekit_characteristic_notify(&cha_dehumidifier_current_state,
                                  cha_dehumidifier_current_state.value);
  }

  // 5. Update StatusFault on the HeaterCooler service. The AC raises
  // bit 3 of 0x09 status flags when another indoor unit on the same
  // outdoor compressor has requested a conflicting thermal direction
  // (e.g. this unit is set to HEAT while a sibling is in COOL). The
  // indoor unit blinks its red LED in this state. We surface it to
  // HomeKit as a fault so the user sees a warning badge on the tile
  // instead of silent non-response.
  uint8_t fault = (hp->getStatus().statusFlags & HP_STATUS_BLOCKED_BY_OTHER) ? 1 : 0;
  if (cha_ac_status_fault.value.uint8_value != fault) {
    cha_ac_status_fault.value.uint8_value = fault;
    homekit_characteristic_notify(&cha_ac_status_fault,
                                  cha_ac_status_fault.value);
    if (fault) {
      GLOG_INFO("MITSUBISHI",
                "StatusFault: blocked by other unit (multi-zone conflict)");
    } else {
      GLOG_INFO("MITSUBISHI", "StatusFault: cleared");
    }
  }
}

void ac_controller_identify() {
  GLOG_INFO("MITSUBISHI", "Identify started! Swinging vanes...");
  identify_active = true;
  identify_start = millis();
  pending_update = true;
}

// Walk lwIP's TCP PCB linked lists. Useful for spotting socket-pool
// pressure, TIME_WAIT pile-up, or listen-pool issues before they cause
// connection-refused failures from HomeKit controllers.
static int tcp_pcb_count(struct tcp_pcb *head) {
  int n = 0;
  for (struct tcp_pcb *p = head; p != nullptr; p = p->next) n++;
  return n;
}
static int tcp_listen_pcb_count() {
  int n = 0;
  for (struct tcp_pcb_listen *p = tcp_listen_pcbs.listen_pcbs; p != nullptr;
       p = p->next)
    n++;
  return n;
}

String ac_controller_get_json_status() {
  StaticJsonDocument<1024> doc;
  JsonObject internal = doc.createNestedObject("target_state_bus");
  internal["active"] = currentState.active;
  internal["mode"] = currentState.target_mode;
  internal["heat_thr"] = currentState.heating_threshold;
  internal["cool_thr"] = currentState.cooling_threshold;
  // Fan: mode is 0=MANUAL, 1=AUTO; speed is 0-100 (HK rotation_speed).
  // When mode==AUTO speed must be 0; when mode==MANUAL speed must be >0.
  // The two are kept consistent by set_ac_rotation_speed and
  // set_ac_target_fan_state.
  internal["fan_mode"] = currentState.fan_mode;
  internal["fan_speed"] = currentState.fan_speed;
  internal["swing_mode"] = currentState.swing_mode;
  internal["hk_target_fan_state"] = cha_ac_target_fan_state.value.uint8_value;
  internal["hk_rotation_speed"] = cha_ac_rotation_speed.value.float_value;

  if (hp && hp->isConnected()) {
    heatpumpSettings s = hp->getSettings();
    heatpumpStatus st = hp->getStatus();
    JsonObject hw = doc.createNestedObject("hardware_status");
    hw["power"] = s.power;
    hw["mode"] = s.mode;
    hw["temp"] = s.temperature;
    hw["room"] = hp->getRoomTemperature();
    hw["operating"] = st.operating;
    // 0x06 p[3] is the indoor coil thermistor (RT12), encoded as
    // `°C = byte + 10` — see docs/byte_resolution_2026-05-26.md in the
    // CN105Sniffer repo. Value 0 means probe inactive (coil floor or
    // HEAT mode where this probe isn't the controller); report only
    // when nonzero.
    if (st.compressorFrequency > 0) {
      hw["indoor_coil_c"] = st.compressorFrequency + 10;
    }
    // 0x09 status flags (bitmask). Surfaced individually for clarity.
    hw["filter_dirty"]      = (st.statusFlags & HP_STATUS_FILTER_DIRTY) != 0;
    hw["defrost_active"]    = (st.statusFlags & HP_STATUS_DEFROST) != 0;
    hw["preheat_active"]    = (st.statusFlags & HP_STATUS_PREHEAT) != 0;
    hw["blocked_by_other"]  = (st.statusFlags & HP_STATUS_BLOCKED_BY_OTHER) != 0;
    hw["actual_fan_speed"]  = st.actualFanSpeed;
    hw["auto_sub_mode"]     = st.autoSubMode & 0x0F;
    hw["auto_leader"]       = (st.autoSubMode & 0x40) != 0;
  }

  JsonObject diag = doc.createNestedObject("diag");
  diag["free_heap"] = ESP.getFreeHeap();
  diag["max_block"] = ESP.getMaxFreeBlockSize();
  diag["heap_frag_pct"] = ESP.getHeapFragmentation();
  diag["tcp_active"] = tcp_pcb_count(tcp_active_pcbs);
  diag["tcp_listen"] = tcp_listen_pcb_count();
  diag["tcp_tw"] = tcp_pcb_count(tcp_tw_pcbs);
  diag["uptime_s"] = millis() / 1000;
  homekit_server_t *hk = arduino_homekit_get_running_server();
  if (hk) {
    diag["hk_paired"] = hk->paired;
    diag["hk_clients"] = hk->nfds;
  }

  String output;
  serializeJson(doc, output);
  return output;
}
