#include "ac_controller.h"
#include "ac_decision.h"
#include "homekit_ac.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>
#include <ESP8266WiFi.h>
#include <LittleFS.h>
#include <arduino_homekit_server.h>

extern "C" {
#include <lwip/tcp.h>
#include <lwip/priv/tcp_priv.h>
}

// Formats the current HomeKit client into out ("System/Local" for none).
static void get_client_info(char *out, size_t n) {
  client_context_t *ctx = (client_context_t *)homekit_get_client_id();
  if (!ctx || !ctx->socket) {
    strncpy(out, "System/Local", n - 1);
    out[n - 1] = '\0';
    return;
  }
  IPAddress ip = ctx->socket->remoteIP();
  snprintf(out, n, "%u.%u.%u.%u (ID:%d)", ip[0], ip[1], ip[2], ip[3],
           ctx->pairing_id);
}

// External specialized HK log implementation
void hk_log_info(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf_P(buf, sizeof(buf), fmt, args);
  va_end(args);

  char client[64];
  get_client_info(client, sizeof(client));
  glog_log(GLOG_L_INFO, "HOMEKIT", PSTR("%s (Client: %s)"), buf, client);
}

static HeatPump *hp;
static TargetState currentState;
static const char *STATE_FILE = "/target_state.bin";

// Decision core (mode/setpoint/fan logic + tuning constants) lives in
// ac_decision.h/.cpp so the host test harness can exercise it. These statics
// are the controller's single cross-tick DecisionState plus the last outputs,
// kept for /metrics.
static DecisionState  g_decision_state;
static DecisionOutput g_last_decision;

static CompGateState g_comp_gate;
static bool g_comp_deferred = false;   // last tick deferred a signature change; for /metrics

// MODE_MAP index for a reported mode string; -1 for anything unrecognized.
static int8_t mode_index_from_str(const char *mode) {
  for (int8_t i = 0; i < 5; i++)
    if (mode && strcmp(mode, hp->MODE_MAP[i]) == 0) return i;
  return -1;
}

// Signature of what the hardware reports right now. Unknown mode maps to the
// AUTO class so the gate stays conservative on anything unrecognized.
static uint8_t hw_comp_sig() {
  heatpumpSettings s = hp->getSettings();
  bool on = s.power && strcmp(s.power, "ON") == 0;
  int8_t idx = mode_index_from_str(s.mode);
  return comp_sig(on, idx < 0 ? MODE_IDX_AUTO : (uint8_t)idx);
}

// Sum of the struct bytes ahead of the trailing uint32 checksum field.
static uint32_t checksum_bytes(const void *p, size_t struct_size) {
  uint32_t sum = 0;
  const uint8_t *bytes = (const uint8_t *)p;
  for (size_t i = 0; i < struct_size - 4; i++) sum += bytes[i];
  return sum;
}

// Logs currentState's user-facing fields under a caller-supplied headline.
static void log_target_state(const char *what) {
  char cBuf[10], hBuf[10];
  dtostrf(currentState.cooling_threshold, 1, 1, cBuf);
  dtostrf(currentState.heating_threshold, 1, 1, hBuf);
  GLOG_INFO("SYS", "%s (Active: %d, Mode: %d, C: %s, H: %s)", what,
            currentState.active, currentState.target_mode, cBuf, hBuf);
}

// Save and Load binary state
void save_target_state() {
  currentState.checksum = checksum_bytes(&currentState, sizeof(TargetState));
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
    log_target_state("Target State saved to LittleFS");
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
  currentState.humidity_threshold = HUMIDITY_THRESHOLD_DEFAULT;
}

// Restored and migrated state reaches ac_decide() without passing through a
// HomeKit setter, so this is the only thing holding the gap on that path.
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
  log_target_state("Migrated v1.12 -> v1.13 target state");
  save_target_state();
  return true;
}

// On-disk layout v1.13-v1.40 (pre-humidity_threshold). Load dispatches on exact
// file size; without this migration those files fall through to defaults.
struct LegacyTargetStateV13 {
  uint8_t active;
  uint8_t target_mode;
  float cooling_threshold;
  float heating_threshold;
  uint8_t swing_mode;
  uint8_t dehumidifier;
  uint32_t checksum;
};

// Migration dispatch is by file size alone, so a collision would silently load
// one layout as another.
static_assert(sizeof(TargetState) != sizeof(LegacyTargetStateV13), "layout collision");
static_assert(sizeof(TargetState) != sizeof(LegacyTargetStateV12), "layout collision");
static_assert(sizeof(LegacyTargetStateV13) != sizeof(LegacyTargetStateV12), "layout collision");

static bool try_migrate_legacy_v13(File &f) {
  if (f.size() != sizeof(LegacyTargetStateV13)) return false;
  LegacyTargetStateV13 legacy;
  f.seek(0);
  if (f.readBytes((char *)&legacy, sizeof(legacy)) != sizeof(legacy)) return false;
  if (checksum_bytes(&legacy, sizeof(legacy)) != legacy.checksum) return false;
  currentState.active = legacy.active;
  currentState.target_mode = legacy.target_mode;
  currentState.cooling_threshold = legacy.cooling_threshold;
  currentState.heating_threshold = legacy.heating_threshold;
  currentState.swing_mode = legacy.swing_mode;
  currentState.dehumidifier = legacy.dehumidifier;
  currentState.humidity_threshold = HUMIDITY_THRESHOLD_DEFAULT;
  sanitize_loaded_thresholds();
  log_target_state("Migrated legacy target state");
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
    if (read == sizeof(TargetState) &&
        checksum_bytes(&loaded, sizeof(TargetState)) == loaded.checksum) {
      currentState = loaded;
      log_target_state("Target State loaded successfully");
      sanitize_loaded_thresholds();
      return;
    }
    GLOG_WARN("SYS", "Target State checksum mismatch; applying defaults");
  } else if (try_migrate_legacy_v13(f)) {
    f.close();
    return;
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

// The per-client event queue coalesces oldest-wins within a batch, and the
// client's own value is already queued here, so a correction must go out later.
static bool pending_threshold_renotify = false;

// Externally supplied indoor humidity. 0 = nothing has ever been reported;
// millis() 0 is otherwise indistinguishable from a stale entry, so the sentinel
// lives in the timestamp being zero AND g_humidity_seen being false.
static float         g_humidity_pct = 0.0f;
static unsigned long g_humidity_at = 0;
static bool          g_humidity_seen = false;
static constexpr unsigned long HUMIDITY_STALE_MS = 15UL * 60UL * 1000UL;

void ac_controller_report_humidity(float percent) {
  if (percent < 0.0f || percent > 100.0f) return;
  g_humidity_pct = percent;
  g_humidity_at = millis();
  g_humidity_seen = true;
  // TRACE, not INFO: a push every few minutes at INFO would rotate the log as
  // fast as the statusByte3 line it would sit next to. /metrics carries it.
  char buf[10];
  dtostrf(percent, 1, 1, buf);
  GLOG_TRACE("SYS", "Humidity reported: %s%%", buf);
  if (cha_dehumidifier_current_humidity.value.float_value != percent) {
    cha_dehumidifier_current_humidity.value.float_value = percent;
    homekit_characteristic_notify(&cha_dehumidifier_current_humidity,
                                  cha_dehumidifier_current_humidity.value);
  }
}

// False once the feed goes quiet, so a HomePod automation that breaks silently
// cannot pin a control decision to a reading from hours ago.
bool ac_controller_humidity_fresh(float& percent_out) {
  if (!g_humidity_seen) return false;
  if (millis() - g_humidity_at > HUMIDITY_STALE_MS) return false;
  percent_out = g_humidity_pct;
  return true;
}

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

void set_dehumidifier_threshold(homekit_value_t value) {
  if (value.format != homekit_format_float) return;
  float pct = value.float_value;
  if (isnan(pct) || pct < *cha_dehumidifier_threshold.min_value ||
      pct > *cha_dehumidifier_threshold.max_value)
    return;
  cha_dehumidifier_threshold.value.float_value = pct;
  currentState.humidity_threshold = pct;
  char buf[10];
  dtostrf(pct, 1, 0, buf);
  HKLOG_INFO("Characteristic Set Dehumidifier Threshold -> %s%%", buf);
  // Persist directly: no decision input changes, so update_state() would only
  // re-command the AC for a value nothing reads yet.
  save_target_state();
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
  cha_dehumidifier_threshold.value.float_value = currentState.humidity_threshold;

  cha_ac_active.setter = set_ac_active;
  cha_ac_target_state.setter = set_ac_target_state;
  cha_ac_cooling_threshold.setter = set_ac_cooling_threshold;
  cha_ac_heating_threshold.setter = set_ac_heating_threshold;
  cha_ac_swing_mode.setter = set_ac_swing_mode;
  cha_dehumidifier_active.setter = set_dehumidifier_active;
  cha_dehumidifier_threshold.setter = set_dehumidifier_threshold;

  // Enable IR remote change detection in the library
  hp->enableExternalUpdate();

  // Set library callbacks for reactive updates
  hp->setSettingsChangedCallback([]() { 
    ac_controller_sync_from_ac(); 
  });
  
  hp->setStatusChangedCallback([](heatpumpStatus status) {
    ac_controller_sync_from_ac();
  });

  homekit_ac_apply_humidity_gate();
  GLOG_INFO("SYS", "Target-humidity slider: %s",
            device_has_humidity_feed() ? "published" : "gated off (no HUMIDITY_UNITS match)");
  arduino_homekit_setup(&config);
}

void update_physical_ac() {
  if (!hp || !hp->isConnected()) return;

  // Gather inputs, decide (pure function), then apply below.
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

  // Compressor gate: seed once from the library's view (self-corrects via the
  // external-sync observe within one settings poll; boot counts as a transition
  // so a crash loop cannot bypass the dwell), then admit or defer. Deferral
  // sends nothing; the unit holds its last commanded state.
  if (g_comp_gate.sig == COMP_SIG_NONE) {
    g_comp_gate.sig = hw_comp_sig();
    g_comp_gate.since = din.now_ms;
  }
  {
    uint8_t sig = comp_sig(dout.power, dout.mode);
    if (!comp_gate_admit(g_comp_gate, sig, pending_update, din.now_ms)) {
      if (!g_comp_deferred)
        GLOG_INFO("MITSUBISHI", "Compressor gate: deferring sig %u -> %u (%us into dwell)",
                  g_comp_gate.sig, sig,
                  (unsigned)((din.now_ms - g_comp_gate.since) / 1000));
      g_comp_deferred = true;
      return;
    }
    if (g_comp_deferred)
      GLOG_INFO("MITSUBISHI", "Compressor gate: dwell over, applying sig %u", sig);
    g_comp_deferred = false;
  }

  bool hpPower = dout.power;
  uint8_t hpMode = dout.mode; // Library Index: 0:HEAT, 1:DRY, 2:COOL, 3:FAN, 4:AUTO
  float hpTemp = dout.temp;
  // From ac_decide()'s staircase; 0 delegates to the unit's native AUTO fan.
  uint8_t fan_idx = dout.fan_idx;

  if (glog_trace_on()) {
    // --- Decision Logging (Rationale) ---
    char rationale[128] = {0};
    char tempBuf[10];

    if (!din.active) {
      strncpy_P(rationale, din.dehumidify ? PSTR("Logic: Target OFF, dehumidifying")
                                          : PSTR("Logic: Target OFF"), sizeof(rationale)-1);
    } else if (din.target_mode == 1) { // HEAT
      dtostrf(hpTemp, 1, 1, tempBuf);
      snprintf_P(rationale, sizeof(rationale),
                 hpMode == 0 ? PSTR("Logic: HEAT call (Target %sC)")
                             : PSTR("Logic: HEAT idle (Target %sC)"), tempBuf);
    } else if (din.target_mode == 2) { // COOL
      dtostrf(hpTemp, 1, 1, tempBuf);
      snprintf_P(rationale, sizeof(rationale),
                 hpMode == 2 ? PSTR("Logic: COOL call (Target %sC)")
                             : PSTR("Logic: COOL idle (Target %sC)"), tempBuf);
    } else {
      dtostrf(din.room_temp, 1, 1, tempBuf);
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
  }

  const char *prev_fan = hp->getWantedSettings().fan;

  // Identify Override
  if (identify_active) {
    hpPower = true; 
    hp->setModeIndex(3); // FAN
    hp->setVaneIndex(6); // SWING
    hp->setFanSpeedIndex(5); // 4 (Max)
  } else {
    hp->setVaneIndex(currentState.swing_mode == 1 ? 6 : 0); // 6:SWING, 0:AUTO
    hp->setFanSpeedIndex(fan_idx);
  }

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
    changed = true;
  }
  
  if (hpPower) {
    // Mode comparison using library side-effects
    if (strcmp(wanted.mode, hp->MODE_MAP[hpMode]) != 0) {
       GLOG_INFO("MITSUBISHI", "COMMAND: Mode -> %s (Currently: %s, Wanted: %s)", 
                 hp->MODE_MAP[hpMode], hp->getModeSetting(), wanted.mode);
       hp->setModeIndex(hpMode);
       changed = true;
    }
    
    // Temp: the library quantizes to the unit's grid; treat as changed only
    // if wantedSettings moved (raw-vs-quantized compares re-fire forever).
    // FAN and DRY carry no setpoint. Without the DRY case the untouched
    // DecisionOutput default of 21.0 is written on every entry into DRY.
    if (hpMode != MODE_IDX_FAN && hpMode != MODE_IDX_DRY) {
      hp->setTemperature(hpTemp);
      float newTemp = hp->getWantedSettings().temperature;
      if (newTemp != wanted.temperature) {
        char tempBuf[10];
        dtostrf(newTemp, 1, 1, tempBuf);
        GLOG_INFO("MITSUBISHI", "COMMAND: Temp -> %sC", tempBuf);
        changed = true;
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

  // After the server's own post-setter notify, which carries the client's value.
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

    // Wait out an in-flight SET: an info request sent before it lands would
    // read back pre-SET state.
    if (pending_sync_request && hp && hp->isConnected() && !hp->updateInFlight()) {
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
  const bool physOn = s.power && strcmp(s.power, "ON") == 0;
  const int8_t mode_idx = mode_index_from_str(s.mode);

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
    comp_gate_observe_external(g_comp_gate, hw_comp_sig(), millis());

    // Sync Power -> Active
    uint8_t physActive = physOn ? 1 : 0;
    if (currentState.active != physActive) {
      currentState.active = physActive;
      cha_ac_active.value.uint8_value = physActive;
      homekit_characteristic_notify(&cha_ac_active, cha_ac_active.value);
    }

    // Sync Mode -> Target State
    // Hardware: HEAT=0, DRY=1, COOL=2, FAN=3, AUTO=4; HomeKit: AUTO=0, HEAT=1, COOL=2.
    // Only remote-chosen HEAT/COOL/AUTO is user intent: FAN/DRY match the
    // firmware's own idle band, and Smart Auto never syncs mode back.
    int8_t hkTargetMode = -1;
    if (mode_idx == MODE_IDX_HEAT) hkTargetMode = 1;
    else if (mode_idx == MODE_IDX_COOL) hkTargetMode = 2;
    else if (mode_idx == MODE_IDX_AUTO) hkTargetMode = 0;

    if (currentState.target_mode != 0 && hkTargetMode >= 0 &&
        currentState.target_mode != hkTargetMode) {
      currentState.target_mode = (uint8_t)hkTargetMode;
      cha_ac_target_state.value.uint8_value = (uint8_t)hkTargetMode;
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
  } else if (!physOn) {
    current_state = 1; // IDLE (idle band)
  } else if (mode_idx == MODE_IDX_HEAT) {
    current_state = 2;
  } else if (mode_idx == MODE_IDX_COOL) {
    current_state = 3;
  } else {
    current_state = 1; // FAN/DRY/AUTO(wait) -> IDLE
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
  uint8_t dehum_state = 0; // 0 = Inactive (service off)
  if (cha_dehumidifier_active.value.uint8_value == 1)
    dehum_state = (physOn && mode_idx == MODE_IDX_DRY) ? 3 : 1;
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
    m_i(out, F("gootac_fan_target_index"), F("GootAC-commanded fan index into FAN_MAP (0=AUTO, 1=QUIET, 2/3/4/5=25/50/75/100%); NOT the same scale as gootac_actual_fan_speed."), g_last_decision.fan_idx);
    m_f(out, F("gootac_control_delta_celsius"), F("Fan-driving delta: >0 short of the commanded target, <=0 depth past it (idle band: vs its anchor target); 0 when the fan is delegated."), g_last_decision.control_delta_c);
    m_b(out, F("gootac_dehumidifier_active"), F("1 if the HomeKit dehumidifier service is targeted on, else 0."), currentState.dehumidifier != 0);
    m_b(out, F("gootac_decided_power"), F("Last decision: 1 if the unit should be powered on."), g_last_decision.power);
    m_i(out, F("gootac_decided_mode_index"), F("Last decision: MODE_MAP index (0 HEAT,1 DRY,2 COOL,3 FAN,4 AUTO); meaningful only when decided power=1."), g_last_decision.mode);
    m_f(out, F("gootac_decided_temp_celsius"), F("Last decision: setpoint in Celsius; not commanded when mode index is 3 or 1."), g_last_decision.temp);
    m_i(out, F("gootac_call_state"), F("Call state, all modes: -1 uninit, 0 HEAT call, 2 COOL call, 3 idle band."), g_decision_state.call_state);
    m_i(out, F("gootac_sa_mode"), F("Deprecated alias of gootac_call_state; will be dropped once dashboards migrate."), g_decision_state.call_state);
    m_i(out, F("gootac_comp_applied_sig"), F("Compressor gate: last applied/observed signature (0 idle, 1 cool-dir, 2 heat, 3 auto, 255 unseeded)."), g_comp_gate.sig);
    m_i(out, F("gootac_comp_decided_sig"), F("Compressor gate: signature of the current decision."), comp_sig(g_last_decision.power, g_last_decision.mode));
    m_b(out, F("gootac_comp_deferred"), F("1 while the gate is holding a decided signature change back."), g_comp_deferred);
    m_u(out, F("gootac_comp_sig_age_seconds"), F("Seconds since the last applied/observed compressor signature change."), g_comp_gate.sig == COMP_SIG_NONE ? 0 : (uint32_t)((millis() - g_comp_gate.since) / 1000));
    m_b(out, F("gootac_ac_filter_dirty"),     F("1 if the AC reports the filter needs cleaning, else 0."),          (st.statusFlags & HP_STATUS_FILTER_DIRTY) != 0);
    m_b(out, F("gootac_ac_defrost_active"),   F("1 if the AC is defrosting, else 0."),                              (st.statusFlags & HP_STATUS_DEFROST) != 0);
    m_b(out, F("gootac_ac_preheat_active"),   F("1 if the AC is preheating, else 0."),                              (st.statusFlags & HP_STATUS_PREHEAT) != 0);
    m_b(out, F("gootac_ac_blocked_by_other"), F("1 if blocked by another unit on the shared outdoor unit, else 0."), (st.statusFlags & HP_STATUS_BLOCKED_BY_OTHER) != 0);
    m_i(out, F("gootac_ac_status_byte3"), F("Raw byte 3 of the 0x06 status packet; unverified outdoor telemetry, NOT degrees C."), st.statusByte3);
    {
      float h;
      bool fresh = ac_controller_humidity_fresh(h);
      m_b(out, F("gootac_humidity_fresh"), F("1 if an external humidity reading arrived within the staleness window."), fresh);
      m_f(out, F("gootac_humidity_percent"), F("Externally reported indoor relative humidity; meaningless unless gootac_humidity_fresh is 1."), g_humidity_seen ? g_humidity_pct : 0.0f);
      m_u(out, F("gootac_humidity_age_seconds"), F("Seconds since the last external humidity report; 0 if none has ever arrived."), g_humidity_seen ? (uint32_t)((millis() - g_humidity_at) / 1000UL) : 0UL);
      m_f(out, F("gootac_humidity_threshold_percent"), F("Target relative humidity set via HomeKit; nothing consumes it yet."), currentState.humidity_threshold);
      m_b(out, F("gootac_humidity_slider_exposed"), F("1 if this unit publishes the HomeKit target-humidity slider (DEVICE_NAME in HUMIDITY_UNITS)."), device_has_humidity_feed());
    }

    m_doc(out, F("gootac_ac_mode_info"), F("Hardware mode, fan, wanted fan, and HomeKit target mode as labels; value always 1."));
    out.print(F("gootac_ac_mode_info{hw_mode=\"")); out.print(s.mode);
    out.print(F("\",fan=\""));                      out.print(s.fan);
    out.print(F("\",wanted_fan=\""));               out.print(w.fan);
    out.print(F("\",target_mode=\""));              out.print(target_mode_str(currentState.target_mode));
    out.print(F("\"} 1\n"));
  }
}
