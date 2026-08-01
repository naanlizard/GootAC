#include "ac_controller.h"
#include "ac_decision.h"
#include "homekit_ac.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>
#include <ArduinoLog.h>
#include <ESP8266WiFi.h>
#include <LittleFS.h>
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

// Decision core (mode/setpoint/fan-ramp logic + tuning constants) lives in
// ac_decision.h/.cpp so the host test harness can exercise it. These statics
// are the controller's single cross-tick DecisionState plus the last outputs,
// kept for /metrics.
static DecisionState  g_decision_state;
static DecisionOutput g_last_decision;

static uint8_t g_fan_target_idx = 0;   // last commanded fan index, for /metrics
static float   g_control_delta_c = 0.0f;

// The indoor fan is driven directly by ac_decide()'s multi-level ramp
// (fan_index_for_delta): QUIET at/below setpoint, ramping through "1".."4"
// (25/50/75/100%) up to 100% at FAN_RAMP_SPAN_C past setpoint. Ramp-up is
// immediate; a drop waits FAN_STEP_DOWN_MS of sustained lower demand. The
// smart-auto deadband runs QUIET; off/DRY/room-unknown delegate to the unit's
// native AUTO (index 0). See src/ac_decision.cpp.

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
  currentState.checksum = calculate_checksum(&currentState);
  // Skip byte-identical rewrites — most save calls repersist unchanged state.
  static TargetState lastSaved;
  static bool haveSaved = false;
  if (haveSaved && memcmp(&lastSaved, &currentState, sizeof(TargetState)) == 0)
    return;
  File f = LittleFS.open(STATE_FILE, "w");
  if (f) {
    f.write((uint8_t *)&currentState, sizeof(TargetState));
    f.close();
    lastSaved = currentState;
    haveSaved = true;
    char cBuf[10], hBuf[10];
    dtostrf(currentState.cooling_threshold, 1, 1, cBuf);
    dtostrf(currentState.heating_threshold, 1, 1, hBuf);
    GLOG_INFO("SYS", "Target State saved to LittleFS (Active: %d, Mode: %d, C: %s, H: %s)", 
              currentState.active, currentState.target_mode, cBuf, hBuf);
  } else {
    GLOG_ERROR("SYS", "Failed to open Target State for writing!");
  }
}

static void apply_default_target_state() {
  currentState.active = 0;
  currentState.target_mode = 0; // AUTO
  currentState.cooling_threshold = 28.0;
  currentState.heating_threshold = 22.0;
  currentState.swing_mode = 0;
  currentState.dehumidifier = 0;
}

// Restored and migrated state reaches ac_decide() without passing through any
// HomeKit setter, so the threshold guard never sees it. Firmware before 1.32
// enforced no gap at all (its guard assigned each threshold to itself), so a
// fielded unit can be holding an inverted or sub-2.0C pair right now, and an
// inverted pair alternates HEAT/COOL on every tick with the room stationary.
// Repair on the way in. Neither stored value is trusted, so an inverted pair is
// reordered rather than resolved in favour of one side.
static void sanitize_loaded_thresholds() {
  float heat = currentState.heating_threshold;
  float cool = currentState.cooling_threshold;
  if (!normalize_thresholds(heat, cool, *cha_ac_heating_threshold.min_value,
                            *cha_ac_cooling_threshold.max_value, THRESHOLD_UNTRUSTED))
    return;
  char cBuf[10], hBuf[10];
  dtostrf(cool, 1, 1, cBuf);
  dtostrf(heat, 1, 1, hBuf);
  GLOG_WARN("SYS", "Stored thresholds unusable; repaired to C: %s, H: %s", cBuf, hBuf);
  currentState.heating_threshold = heat;
  currentState.cooling_threshold = cool;
  save_target_state();
}

// Legacy on-disk layout from v1.11/v1.12 (before fan controls were removed
// from HK). Kept here so v1.13 can migrate existing user-set thresholds
// instead of silently reverting them to defaults on first boot.
struct LegacyTargetStateV12 {
  uint8_t active;
  uint8_t target_mode;
  float cooling_threshold;
  float heating_threshold;
  uint8_t fan_mode;
  float fan_speed;
  uint8_t swing_mode;
  uint8_t dehumidifier;
  uint32_t checksum;
};

static bool try_migrate_legacy_v12(File &f) {
  if (f.size() != sizeof(LegacyTargetStateV12)) return false;
  LegacyTargetStateV12 legacy;
  f.seek(0);
  if (f.readBytes((char *)&legacy, sizeof(legacy)) != sizeof(legacy)) return false;
  if (legacy.cooling_threshold < 10.0f || legacy.cooling_threshold > 40.0f) return false;
  if (legacy.heating_threshold < 10.0f || legacy.heating_threshold > 40.0f) return false;
  currentState.active = legacy.active;
  currentState.target_mode = legacy.target_mode;
  currentState.cooling_threshold = legacy.cooling_threshold;
  currentState.heating_threshold = legacy.heating_threshold;
  currentState.swing_mode = legacy.swing_mode;
  currentState.dehumidifier = legacy.dehumidifier;
  sanitize_loaded_thresholds();   // legacy admits 10-40C, unordered
  char cBuf[10], hBuf[10];
  dtostrf(currentState.cooling_threshold, 1, 1, cBuf);
  dtostrf(currentState.heating_threshold, 1, 1, hBuf);
  GLOG_INFO("SYS", "Migrated v1.12 -> v1.13 target state (Active: %d, Mode: %d, C: %s, H: %s)",
            currentState.active, currentState.target_mode, cBuf, hBuf);
  save_target_state();
  return true;
}

void load_target_state() {
  if (!LittleFS.exists(STATE_FILE)) {
    GLOG_INFO("SYS", "No state file found. Initializing fresh defaults.");
    apply_default_target_state();
    save_target_state();
    return;
  }

  File f = LittleFS.open(STATE_FILE, "r");
  if (!f) return;

  if (f.size() == sizeof(TargetState)) {
    TargetState loaded;
    size_t read = f.readBytes((char *)&loaded, sizeof(TargetState));
    f.close();
    if (read == sizeof(TargetState) && calculate_checksum(&loaded) == loaded.checksum) {
      currentState = loaded;
      char cBuf[10], hBuf[10];
      dtostrf(currentState.cooling_threshold, 1, 1, cBuf);
      dtostrf(currentState.heating_threshold, 1, 1, hBuf);
      GLOG_INFO("SYS", "Target State loaded successfully (Active: %d, Mode: %d, C: %s, H: %s)",
                currentState.active, currentState.target_mode, cBuf, hBuf);
      sanitize_loaded_thresholds();
      return;
    }
    GLOG_WARN("SYS", "Target State checksum mismatch; applying defaults");
  } else if (try_migrate_legacy_v12(f)) {
    f.close();
    return;
  } else {
    GLOG_WARN("SYS", "Target State size mismatch (got %u, expected %u); applying defaults",
              (unsigned)f.size(), (unsigned)sizeof(TargetState));
    f.close();
  }

  apply_default_target_state();
  save_target_state();
}

// Set when the guard has overridden a value a client wrote. The per-client event
// queue is LIFO and the flush loop coalesces by overwriting with each successive
// pop, so within one batch the OLDEST queued value is the one delivered — and
// the client's own value is already queued by the time the guard runs. Re-send
// from the loop instead, which lands in a later batch and therefore wins.
static bool pending_threshold_renotify = false;

// Hold the pair inside the declared characteristic range and at least
// THRESHOLD_MIN_GAP_C apart, moving the threshold the caller did NOT just set so
// their write is the one that survives. See normalize_thresholds().
static void enforce_threshold_gap(ThresholdAuthority who) {
  float heat = cha_ac_heating_threshold.value.float_value;
  float cool = cha_ac_cooling_threshold.value.float_value;
  if (!normalize_thresholds(heat, cool, *cha_ac_heating_threshold.min_value,
                            *cha_ac_cooling_threshold.max_value, who))
    return;

  char heatBuf[10], coolBuf[10];
  dtostrf(heat, 1, 1, heatBuf);
  dtostrf(cool, 1, 1, coolBuf);
  GLOG_TRACE("MITSUBISHI", "Threshold Gap Guard: %s / %s", heatBuf, coolBuf);

  cha_ac_heating_threshold.value.float_value = heat;
  currentState.heating_threshold = heat;
  cha_ac_cooling_threshold.value.float_value = cool;
  currentState.cooling_threshold = cool;
  pending_threshold_renotify = true;
}

static bool pending_update = false;
static bool pending_sync_request = false;

// Timing and guards
static unsigned long lastSetTime = 0;
static unsigned long lastInteractionTime = 0;
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

  cha_ac_cooling_threshold.value = value;
  currentState.cooling_threshold = value.float_value;

  char tempBuf[10];
  dtostrf(value.float_value, 1, 1, tempBuf);
  HKLOG_INFO("Characteristic Set Cooling Threshold -> %s", tempBuf);
  homekit_characteristic_notify(&cha_ac_cooling_threshold, cha_ac_cooling_threshold.value);

  enforce_threshold_gap(THRESHOLD_FROM_COOL_WRITE);
  update_state("HomeKit Cooling Threshold change");
}

void set_ac_heating_threshold(homekit_value_t value) {
  if (value.format != homekit_format_float) return;

  cha_ac_heating_threshold.value = value;
  currentState.heating_threshold = value.float_value;

  char tempBuf[10];
  dtostrf(value.float_value, 1, 1, tempBuf);
  HKLOG_INFO("Characteristic Set Heating Threshold -> %s", tempBuf);
  homekit_characteristic_notify(&cha_ac_heating_threshold, cha_ac_heating_threshold.value);

  enforce_threshold_gap(THRESHOLD_FROM_HEAT_WRITE);
  update_state("HomeKit Heating Threshold change");
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
  cha_ac_swing_mode.value.uint8_value = currentState.swing_mode;
  cha_dehumidifier_active.value.uint8_value = currentState.dehumidifier;

  cha_ac_active.setter = set_ac_active;
  cha_ac_target_state.setter = set_ac_target_state;
  cha_ac_cooling_threshold.setter = set_ac_cooling_threshold;
  cha_ac_heating_threshold.setter = set_ac_heating_threshold;
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

  // Gather inputs, decide (pure function), then apply below. The apply code
  // from the rationale block onward is unchanged from a805f40; these local
  // aliases keep it diff-identical.
  DecisionInput din;
  din.active = (currentState.active == 1);
  din.target_mode = currentState.target_mode; // 0:Auto, 1:Heat, 2:Cool
  din.heat_threshold = currentState.heating_threshold;
  din.cool_threshold = currentState.cooling_threshold;
  din.dehumidify = (currentState.dehumidifier == 1);
  din.room_temp = hp->getRoomTemperature();
  din.now_ms = millis();

  const DecisionOutput dout = ac_decide(din, g_decision_state);
  g_last_decision = dout;
  g_fan_target_idx = dout.fan_idx;
  g_control_delta_c = dout.control_delta_c;

  bool target_active = din.active;
  uint8_t hk_target_mode = din.target_mode;
  float roomTemp = din.room_temp;
  bool dehumidify = din.dehumidify;

  bool hpPower = dout.power;
  uint8_t hpMode = dout.mode; // Library Index: 0:HEAT, 1:DRY, 2:COOL, 3:FAN, 4:AUTO
  float hpTemp = dout.temp;
  // Fan target comes straight from ac_decide()'s multi-level ramp; 0 delegates
  // to the unit's native AUTO. g_fan_target_idx was already set from dout above.
  uint8_t fan_idx = dout.fan_idx;

  // --- Decision Logging (Rationale) ---
  char rationale[128] = {0};
  char tempBuf[10];
  
  if (!target_active) {
    strncpy_P(rationale, dehumidify ? PSTR("Logic: Target OFF, dehumidifying")
                                    : PSTR("Logic: Target OFF"), sizeof(rationale)-1);
  } else if (hk_target_mode == 1) { // HEAT
    dtostrf(hpTemp, 1, 1, tempBuf);
    snprintf_P(rationale, sizeof(rationale), PSTR("Logic: HEAT mode (Target %sC)"), tempBuf);
  } else if (hk_target_mode == 2) { // COOL
    dtostrf(hpTemp, 1, 1, tempBuf);
    snprintf_P(rationale, sizeof(rationale), PSTR("Logic: COOL mode (Target %sC)"), tempBuf);
  } else {
    dtostrf(roomTemp, 1, 1, tempBuf);
    snprintf_P(rationale, sizeof(rationale),
               hpMode == 1 ? PSTR("Logic: Smart Auto dehumidifying, Room %sC")
                           : PSTR("Logic: Smart Auto decision based on Room %sC"),
               tempBuf);
  }

  static char lastRationale[128] = {0};
  if (strcmp(lastRationale, rationale) != 0) {
    GLOG_TRACE("MITSUBISHI", "%s", rationale);
    strncpy(lastRationale, rationale, sizeof(lastRationale)-1);
  }

  // Fan target already decided by ac_decide() (fan_idx; 0 = delegate to AUTO).
  const char *prev_fan = hp->getWantedSettings().fan;

  // Identify Override
  if (identify_active) {
    hpPower = true; 
    hp->setModeIndex(3); // FAN
    hp->setVaneIndex(6); // SWING
    hp->setFanSpeedIndex(5); // 4 (Max)
  } else {
    hp->setVaneIndex(currentState.swing_mode == 1 ? 6 : 0); // 6:SWING, 0:AUTO
    // Fan driven by ac_decide()'s ramp; 0 = delegate to AUTO.
    hp->setFanSpeedIndex(fan_idx);
  }

  unsigned long now = millis();
  bool changed = false;
  bool pu = pending_update;
  bool fan_changed = (strcmp(prev_fan, hp->getWantedSettings().fan) != 0);
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
    
    // Temp: the library quantizes to the unit's grid; treat as changed only
    // if wantedSettings moved (raw-vs-quantized compares re-fire forever).
    // FAN and DRY carry no setpoint. Without the DRY case the untouched
    // DecisionOutput default of 21.0 is written on every entry into DRY.
    if (hpMode != 3 && hpMode != 1) {
      hp->setTemperature(hpTemp);
      float newTemp = hp->getWantedSettings().temperature;
      if (newTemp != wanted.temperature) {
        char tempBuf[10];
        dtostrf(newTemp, 1, 1, tempBuf);
        GLOG_INFO("MITSUBISHI", "COMMAND: Temp -> %sC", tempBuf);
        lastSetTime = now; changed = true;
      }
    }
  }

  if (changed || fan_changed || pu) {
    pending_update = false;
    GLOG_INFO("MITSUBISHI", "Updating physical AC unit...");
    hp->update();
    if (pu) save_target_state(); // persisted state only changes via HK setters
    if (changed || fan_changed) pending_sync_request = true;
  }
}

void ac_controller_loop() {
  arduino_homekit_loop();
  yield();

  // Deferred so it lands after the server's own post-setter notify, which
  // carries the value the client wrote rather than the corrected one.
  if (pending_threshold_renotify) {
    pending_threshold_renotify = false;
    homekit_characteristic_notify(&cha_ac_heating_threshold, cha_ac_heating_threshold.value);
    homekit_characteristic_notify(&cha_ac_cooling_threshold, cha_ac_cooling_threshold.value);
  }

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

    // In Smart-Auto (target_mode 0) the firmware itself drives the physical
    // HEAT/COOL/FAN mode, so the unit's reported mode is not user intent --
    // syncing it back would silently drop Smart-Auto to a fixed mode. Only
    // reflect an external mode change when the user picked a fixed mode.
    if (currentState.target_mode != 0 && currentState.target_mode != hkTargetMode) {
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

  // 4. Update Dehumidifier Current State (0:Inactive, 1:Idle, 3:Dehumidifying).
  // Report Dehumidifying whenever the service is active and the AC is
  // in DRY mode with power on. The earlier "coil cold = byte 1..8"
  // predicate was based on a wrong interpretation of statusByte3 as a
  // coil temperature, and has been removed.
  uint8_t dehum_state = 0; // 0 = Inactive (service off)
  if (cha_dehumidifier_active.value.uint8_value == 1) {
    bool physOn = (strcmp(s.power, "ON") == 0);
    bool inDry  = (strcmp(s.mode, "DRY") == 0);
    dehum_state = (physOn && inDry) ? 3 : 1;
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

// --- Prometheus metric emit helpers ---
// Line endings are bare LF; Prometheus's text parser rejects CR. Metric names
// and HELP text are flash strings (F()) so they never occupy RAM.
static void m_doc(Print& o, const __FlashStringHelper* n, const __FlashStringHelper* h) {
  o.print(F("# HELP ")); o.print(n); o.print(' '); o.print(h); o.print('\n');
  o.print(F("# TYPE ")); o.print(n); o.print(F(" gauge\n"));
}
static void m_u(Print& o, const __FlashStringHelper* n, const __FlashStringHelper* h, uint32_t v) {
  m_doc(o, n, h); o.print(n); o.print(' '); o.print(v); o.print('\n');
}
static void m_i(Print& o, const __FlashStringHelper* n, const __FlashStringHelper* h, int32_t v) {
  m_doc(o, n, h); o.print(n); o.print(' '); o.print(v); o.print('\n');
}
static void m_b(Print& o, const __FlashStringHelper* n, const __FlashStringHelper* h, bool v) {
  m_doc(o, n, h); o.print(n); o.print(' '); o.print(v ? '1' : '0'); o.print('\n');
}
static void m_f(Print& o, const __FlashStringHelper* n, const __FlashStringHelper* h, float v) {
  char b[16]; dtostrf(v, 1, 1, b); // 1 decimal — 0.1C room-sensor resolution
  m_doc(o, n, h); o.print(n); o.print(' '); o.print(b); o.print('\n');
}
// currentState.target_mode holds the HomeKit TargetHeaterCoolerState enum:
// 0=AUTO, 1=HEAT, 2=COOL (matches the setter and the decision logic).
static const __FlashStringHelper* target_mode_str(uint8_t m) {
  switch (m) {
    case 0:  return F("AUTO");
    case 1:  return F("HEAT");
    case 2:  return F("COOL");
    default: return F("UNKNOWN");
  }
}

void ac_controller_write_metrics(Print& out) {
  // --- Identity ---
  m_doc(out, F("gootac_info"), F("Firmware/build identity; value is always 1."));
  out.print(F("gootac_info{version=\"")); out.print(F(FW_VERSION));
  out.print(F("\",device=\""));           out.print(F(DEVICE_NAME));
  out.print(F("\"} 1\n"));

  // --- System / network (always emitted) ---
  m_u(out, F("gootac_uptime_seconds"),             F("Seconds since last boot; wraps at the ~49.7d millis() rollover."), millis() / 1000);
  m_u(out, F("gootac_free_heap_bytes"),            F("Free heap memory in bytes."),                   ESP.getFreeHeap());
  m_u(out, F("gootac_heap_max_free_block_bytes"),  F("Largest contiguous free heap block in bytes."), ESP.getMaxFreeBlockSize());
  m_u(out, F("gootac_heap_fragmentation_percent"), F("Heap fragmentation percentage (0-100)."),       ESP.getHeapFragmentation());
  m_i(out, F("gootac_wifi_rssi_dbm"),              F("WiFi signal strength in dBm."),                 WiFi.RSSI());

  m_doc(out, F("gootac_tcp_pcb"), F("Open lwIP TCP PCBs by state."));
  out.print(F("gootac_tcp_pcb{state=\"active\"} "));   out.print(tcp_pcb_count(tcp_active_pcbs)); out.print('\n');
  out.print(F("gootac_tcp_pcb{state=\"listen\"} "));   out.print(tcp_listen_pcb_count());         out.print('\n');
  out.print(F("gootac_tcp_pcb{state=\"timewait\"} ")); out.print(tcp_pcb_count(tcp_tw_pcbs));      out.print('\n');

  homekit_server_t* hk = arduino_homekit_get_running_server();
  m_b(out, F("gootac_homekit_paired"),  F("1 if a HomeKit controller is paired, else 0."), hk ? hk->paired : false);
  m_u(out, F("gootac_homekit_clients"), F("Active HomeKit client connections."),           hk ? (uint32_t)hk->nfds : 0);

  // --- AC / heatpump (guarded; gootac_ac_connected always emitted) ---
  bool connected = hp && hp->isConnected();
  m_b(out, F("gootac_ac_connected"), F("1 if the CN105 link to the AC is up, else 0."), connected);
  if (connected) {
    heatpumpSettings s  = hp->getSettings();
    heatpumpSettings w  = hp->getWantedSettings();
    heatpumpStatus   st = hp->getStatus();

    m_b(out, F("gootac_ac_power_on"),  F("1 if the AC is powered on, else 0."),               hp->getPowerSettingBool());
    m_b(out, F("gootac_ac_operating"), F("1 if the AC is actively heating/cooling, else 0."), st.operating);
    m_f(out, F("gootac_room_temperature_celsius"),   F("Room temperature measured by the AC, in Celsius."), hp->getRoomTemperature());
    m_f(out, F("gootac_target_temperature_celsius"), F("AC setpoint temperature in Celsius."),              s.temperature);
    m_f(out, F("gootac_heating_threshold_celsius"),  F("HomeKit auto-mode heating threshold in Celsius."),  currentState.heating_threshold);
    m_f(out, F("gootac_cooling_threshold_celsius"),  F("HomeKit auto-mode cooling threshold in Celsius."),  currentState.cooling_threshold);
    m_b(out, F("gootac_target_active"), F("1 if the HomeKit HeaterCooler service is active, else 0."), currentState.active != 0);
    m_b(out, F("gootac_swing_mode"),    F("1 if vane swing is enabled, else 0."),                      currentState.swing_mode != 0);
    m_u(out, F("gootac_actual_fan_speed"), F("Actual fan-speed index reported by the AC."), st.actualFanSpeed);
    m_i(out, F("gootac_fan_target_index"), F("GootAC-commanded fan index into FAN_MAP (0=AUTO, 1=QUIET, 2/3/4/5=25/50/75/100%); NOT the same scale as gootac_actual_fan_speed."), g_fan_target_idx);
    m_f(out, F("gootac_control_delta_celsius"), F("Room-vs-setpoint demand delta driving the fan ramp: >0 still needs conditioning, <0 past setpoint; 0 when idle."), g_control_delta_c);
    m_b(out, F("gootac_dehumidifier_active"), F("1 if the HomeKit dehumidifier service is targeted on, else 0."), currentState.dehumidifier != 0);
    m_b(out, F("gootac_decided_power"), F("Last decision: 1 if the unit should be powered on."), g_last_decision.power);
    m_i(out, F("gootac_decided_mode_index"), F("Last decision: MODE_MAP index (0 HEAT,1 DRY,2 COOL,3 FAN,4 AUTO); meaningful only when decided power=1."), g_last_decision.mode);
    m_f(out, F("gootac_decided_temp_celsius"), F("Last decision: setpoint in Celsius; not commanded when mode index=3."), g_last_decision.temp);
    m_i(out, F("gootac_sa_mode"), F("Smart-Auto direction memory: -1 uninit, 0 HEAT, 2 COOL, 3 FAN deadband."), g_decision_state.sa_mode);
    m_b(out, F("gootac_ac_filter_dirty"),     F("1 if the AC reports the filter needs cleaning, else 0."),          (st.statusFlags & HP_STATUS_FILTER_DIRTY) != 0);
    m_b(out, F("gootac_ac_defrost_active"),   F("1 if the AC is defrosting, else 0."),                              (st.statusFlags & HP_STATUS_DEFROST) != 0);
    m_b(out, F("gootac_ac_preheat_active"),   F("1 if the AC is preheating, else 0."),                              (st.statusFlags & HP_STATUS_PREHEAT) != 0);
    m_b(out, F("gootac_ac_blocked_by_other"), F("1 if blocked by another unit on the shared outdoor unit, else 0."), (st.statusFlags & HP_STATUS_BLOCKED_BY_OTHER) != 0);
    m_i(out, F("gootac_ac_status_byte3"), F("Raw byte 3 of the 0x06 status packet; unverified outdoor telemetry, NOT degrees C."), st.statusByte3);

    m_doc(out, F("gootac_ac_mode_info"), F("Hardware mode, fan, wanted fan, and HomeKit target mode as labels; value always 1."));
    out.print(F("gootac_ac_mode_info{hw_mode=\"")); out.print(s.mode);
    out.print(F("\",fan=\""));                      out.print(s.fan);
    out.print(F("\",wanted_fan=\""));               out.print(w.fan);
    out.print(F("\",target_mode=\""));              out.print(target_mode_str(currentState.target_mode));
    out.print(F("\"} 1\n"));
  }
}
