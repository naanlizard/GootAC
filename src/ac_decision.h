#pragma once
#include <stdint.h>

// Pure decision core: HomeKit-targeted state plus room temperature in, what to
// tell the AC out. No Arduino/HeatPump/HomeKit deps, so host_tests/ compiles it.

// FAN_MAP indices: 0=AUTO, 1=QUIET, 2..5 = "1".."4" (25/50/75/100%).
constexpr uint8_t  FAN_IDX_MIN      = 1;
constexpr uint8_t  FAN_IDX_MAX      = 5;
constexpr uint32_t FAN_STEP_DOWN_MS = 60000;  // sustained lower demand before easing down
constexpr float    SENSOR_STEP_C    = 0.5f;   // CN105 reports room temp in 0.5C steps

// How far past the calling threshold a call drives: static, one sensor step.
// Total hysteresis is 2 steps; the comp gate paces anything faster.
constexpr float    AUTO_PULL_C      = 0.5f;
// TEMP_MAP endpoints: targets outside this band cannot be commanded, and an
// uncommandable target is a release point the room never reaches.
constexpr float    TEMP_CMD_MIN_C   = 16.0f;
constexpr float    TEMP_CMD_MAX_C   = 31.0f;

// The target is both the commanded setpoint and the release point: the call ends
// exactly when the room reaches it, and the idle staircase picks up at the same
// depth, so the fan hands over continuously across the release.
float auto_pull_c(float heat_threshold, float cool_threshold);
float auto_cool_target(float heat_threshold, float cool_threshold);  // cool - pull
float auto_heat_target(float heat_threshold, float cool_threshold);  // heat + pull

// ac_decide has no defence of its own: an inverted pair trips both entry tests
// and flips HEAT/COOL every tick, so every caller must normalize first.
constexpr float THRESHOLD_MIN_GAP_C = 3.0f;

enum ThresholdAuthority : uint8_t {
  THRESHOLD_FROM_HEAT_WRITE = 0,
  THRESHOLD_FROM_COOL_WRITE = 1,
  THRESHOLD_UNTRUSTED       = 2,  // restored/migrated state; neither value trusted
};

// Assumes hi - lo >= THRESHOLD_MIN_GAP_C. Only UNTRUSTED reorders an inversion:
// on a write path a swap moves the client's value onto the other characteristic.
bool normalize_thresholds(float &heat, float &cool, float lo, float hi,
                          ThresholdAuthority who);

struct DecisionInput {
  bool     active;          // currentState.active == 1
  uint8_t  target_mode;     // HomeKit TargetHeaterCoolerState: 0 AUTO, 1 HEAT, 2 COOL
  float    heat_threshold;
  float    cool_threshold;
  bool     dehumidify;      // currentState.dehumidifier == 1
  float    room_temp;       // <= 1.0 means unknown
  uint32_t now_ms;          // caller passes millis(); uint32_t matches ESP8266 wrap
};

// Cross-tick memory. Reset only at boot.
struct DecisionState {
  int8_t   sa_mode      = -1; // call state, all modes: -1 uninit; 0=HEAT call, 2=COOL call, 3=idle band
  uint8_t  last_fan_idx = 0;  // FAN_MAP index of the last commanded fan level
  uint32_t lower_since  = 0;  // 0 = no step-down pending
};

// When power == false, mode/temp are don't-cares; the caller's `if (hpPower)`
// gate is what stops them being commanded to an off unit.
struct DecisionOutput {
  bool    power;
  uint8_t mode;             // MODE_MAP index: 0 HEAT, 1 DRY, 2 COOL, 3 FAN, 4 AUTO
  float   temp;             // caller skips setTemperature when mode is 3 or 1
  uint8_t fan_idx;          // FAN_MAP index; 0 = delegate to unit AUTO
  float   control_delta_c;  // demand delta; negative is preserved, not clamped
};

uint8_t fan_index_for_delta(float delta);

// Deterministic, zero side effects: same (input, pre-state) always yields the
// same (output, post-state).
DecisionOutput ac_decide(const DecisionInput& in, DecisionState& state);

// Compressor gate: classifies what a commanded (power, mode) pair asks of the
// compressor. DRY and COOL share a direction; AUTO's direction is the unit's
// own choice, so crossing its boundary is treated as a potential reversal.
enum : uint8_t {
  COMP_SIG_IDLE = 0,   // off, or FAN
  COMP_SIG_COOL = 1,   // COOL or DRY
  COMP_SIG_HEAT = 2,
  COMP_SIG_AUTO = 3,   // native AUTO (blind fallback, or observed from hardware)
  COMP_SIG_NONE = 0xFF,
};
// >= the typical 3-min compressor restart delay with margin; also >= the 5-min
// humidity push cadence so the dehumidify latch cannot flap faster than the gate.
constexpr uint32_t COMP_MIN_DWELL_MS = 300000;

uint8_t comp_sig(bool power, uint8_t mode_idx);

struct CompGateState {
  uint8_t  sig   = COMP_SIG_NONE;  // last applied/observed signature
  uint32_t since = 0;              // when it changed (millis domain)
};

// True = apply and record; false = defer (caller sends nothing this tick).
// `since` moves only on an applied change, so consecutive applied transitions
// are >= COMP_MIN_DWELL_MS apart and a flapping decision cannot postpone its
// own application.
bool comp_gate_admit(CompGateState& st, uint8_t sig, bool user_write, uint32_t now_ms);

// An IR/panel transition is a compressor transition: record it and restart the dwell.
void comp_gate_observe_external(CompGateState& st, uint8_t sig, uint32_t now_ms);
