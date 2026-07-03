#pragma once
#include <stdint.h>

// Pure decision core: given HomeKit-targeted state, the room temperature, and
// cross-tick memory, compute what the physical AC should be told (power, mode,
// setpoint, fan index). No Arduino/HeatPump/HomeKit dependencies, so both
// PlatformIO envs and the host harness (host_tests/) compile it.
// Transplanted verbatim from update_physical_ac() at commit a805f40.
//
// The identify override, vane/swing handling, all change detection, and
// command dispatch stay in ac_controller.cpp — this function only decides.

// Fan/mode tuning. FAN_MAP indices: 0=AUTO, 1=QUIET, 2..5 = "1".."4"
// (25/50/75/100%).
constexpr uint8_t  FAN_IDX_MIN      = 2;      // 25% floor while actively conditioning
constexpr float    FAN_RAMP_SPAN_C  = 1.5f;   // room-vs-setpoint delta at which fan hits 100%
constexpr uint32_t FAN_STEP_DOWN_MS = 60000;  // sustained lower demand before easing fan down
constexpr float    MODE_HYST_C      = 0.5f;   // Smart-Auto COOL/HEAT release hysteresis

struct DecisionInput {
  bool     active;          // currentState.active == 1
  uint8_t  target_mode;     // HomeKit TargetHeaterCoolerState: 0 AUTO, 1 HEAT, 2 COOL
  float    heat_threshold;
  float    cool_threshold;
  bool     dehumidify;      // currentState.dehumidifier == 1
  float    room_temp;       // <= 1.0 means unknown
  uint32_t now_ms;          // caller passes millis(); uint32_t matches ESP8266 wrap
};

// Cross-tick memory (previously function-local statics). Read-modify-write by
// ac_decide(); reset only at boot. One instance lives in ac_controller.cpp;
// tests own their instances. Default init matches the old statics exactly.
struct DecisionState {
  int8_t   sa_mode      = -1; // -1 uninit; 0=HEAT, 2=COOL, 3=FAN(deadband)
  uint8_t  last_fan_idx = 0;  // FAN_MAP index of last ramp-commanded fan
  uint32_t lower_since  = 0;  // 0 = no step-down pending
};

// What the unit should be told, in library-index domain.
// When power == false, mode/temp keep their defaults (0 / 21.0) and are
// don't-cares: the caller's `if (hpPower)` gate is load-bearing — it is what
// prevents commanding mode/temp to an off unit, exactly as at a805f40.
struct DecisionOutput {
  bool    power;
  uint8_t mode;             // MODE_MAP index: 0 HEAT, 1 DRY, 2 COOL, 3 FAN, 4 AUTO
  float   temp;             // default 21.0; caller skips setTemperature when mode==3
  uint8_t fan_idx;          // FAN_MAP index; 0 = delegate to unit AUTO
  float   control_delta_c;  // raw demand delta — CAN be negative (preserved)
};

uint8_t fan_index_for_delta(float delta);

// Deterministic, zero side effects: same (input, pre-state) always yields the
// same (output, post-state).
DecisionOutput ac_decide(const DecisionInput& in, DecisionState& state);
