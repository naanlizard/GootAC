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
constexpr uint8_t  FAN_IDX_MIN      = 1;      // QUIET floor at/below setpoint and in the deadband
constexpr float    FAN_RAMP_SPAN_C  = 1.5f;   // room-vs-setpoint delta at which fan hits 100%
constexpr uint32_t FAN_STEP_DOWN_MS = 60000;  // sustained lower demand before easing fan down
constexpr float    SENSOR_STEP_C    = 0.5f;   // CN105 reports room temp in 0.5C steps

// How far past the threshold that called it Smart Auto drives the room, as a
// fraction of the Auto range, clamped and snapped to the sensor grid. A fraction
// keeps a wide range from conditioning to its midpoint and a narrow one from
// barely moving; the cap stops a very wide range from running the compressor for
// hours to reach a point nobody asked for.
constexpr float    AUTO_PULL_FRAC   = 0.25f;
constexpr float    AUTO_PULL_MIN_C  = 1.0f;
constexpr float    AUTO_PULL_MAX_C  = 2.0f;

// Smart Auto's per-call target. This single point is BOTH the commanded setpoint
// and the release threshold: the mode hands back to the idle band exactly when
// the room reaches it. Splitting the two is what made the previous version
// misbehave — the release fired above the setpoint, so the commanded value never
// governed where the room landed and the fan ramp lost its bottom rungs.
float auto_pull_c(float heat_threshold, float cool_threshold);
float auto_cool_target(float heat_threshold, float cool_threshold);  // cool - pull
float auto_heat_target(float heat_threshold, float cool_threshold);  // heat + pull

// Smallest Auto range the rest of the firmware may present to ac_decide(). At
// 3.0C the pull reaches AUTO_PULL_MIN_C without also hitting the half-range cap,
// so the two targets stay 1.0C apart instead of collapsing onto the midpoint,
// which they do at exactly 2.0C. The decision core does NOT defend itself
// against narrower or inverted
// pairs: an inverted pair satisfies both entry tests at once and alternates
// HEAT/COOL on every tick with the room stationary, which is a mode-change
// packet to the AC every tick. Every path that can put a threshold pair in front
// of ac_decide() — HomeKit writes, restored flash state, legacy migration — must
// run it through normalize_thresholds() first.
constexpr float THRESHOLD_MIN_GAP_C = 3.0f;

// Which value, if either, the caller must not move.
enum ThresholdAuthority : uint8_t {
  THRESHOLD_FROM_HEAT_WRITE = 0,  // the heating threshold was just written
  THRESHOLD_FROM_COOL_WRITE = 1,  // the cooling threshold was just written
  THRESHOLD_UNTRUSTED       = 2,  // restored or migrated state; neither is trusted
};

// Force `heat`/`cool` into [lo, hi] and at least THRESHOLD_MIN_GAP_C apart, in
// place, and replace non-finite values. Returns true if either moved. Idempotent,
// so re-running it on its own output is a no-op. Assumes hi - lo >= the gap.
//
// Only THRESHOLD_UNTRUSTED reorders an inverted pair. On the two write paths a
// swap would move the value the client just set onto the OTHER characteristic
// — writing cool=17 over a 25/27 pair would silently become heat=17 — so there
// an inversion is treated as an extreme case of "too narrow" and resolved by
// moving the value the client did not touch.
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

// Cross-tick memory (previously function-local statics). Read-modify-write by
// ac_decide(); reset only at boot. One instance lives in ac_controller.cpp;
// tests own their instances. Default init matches the old statics exactly.
struct DecisionState {
  int8_t   sa_mode      = -1; // -1 uninit; 0=HEAT, 2=COOL, 3=FAN/DRY(idle band)
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
