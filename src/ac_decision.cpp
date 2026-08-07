#include "ac_decision.h"
#include <math.h>

float pull_c(float heat_threshold, float cool_threshold) {
  const float range = cool_threshold - heat_threshold;
  if (range <= 0.0f) return 0.0f;
  // Past half the range the targets cross, so releasing one call lands inside
  // the other and the unit reverses instead of idling. Floored onto the grid.
  const float half = SENSOR_STEP_C * floorf((range * 0.5f) / SENSOR_STEP_C);
  return CALL_PULL_C > half ? half : CALL_PULL_C;
}

bool normalize_thresholds(float &heat, float &cool, float lo, float hi,
                          ThresholdAuthority who) {
  const float in_h = heat, in_c = cool;

  // Independently: one corrupt value is no reason to discard the other.
  if (!isfinite(heat)) heat = lo;
  if (!isfinite(cool)) cool = hi;

  if (who == THRESHOLD_UNTRUSTED && heat > cool) {
    const float t = heat; heat = cool; cool = t;
  }
  if (heat < lo) heat = lo; else if (heat > hi) heat = hi;
  if (cool < lo) cool = lo; else if (cool > hi) cool = hi;

  if (cool - heat < THRESHOLD_MIN_GAP_C) {
    if (who == THRESHOLD_FROM_COOL_WRITE) {
      heat = cool - THRESHOLD_MIN_GAP_C;
      if (heat < lo) { heat = lo; cool = lo + THRESHOLD_MIN_GAP_C; }
    } else {
      cool = heat + THRESHOLD_MIN_GAP_C;
      if (cool > hi) { cool = hi; heat = hi - THRESHOLD_MIN_GAP_C; }
    }
  }
  return heat != in_h || cool != in_c;
}

static float clamp_cmd(float t) {
  if (t < TEMP_CMD_MIN_C) return TEMP_CMD_MIN_C;
  if (t > TEMP_CMD_MAX_C) return TEMP_CMD_MAX_C;
  return t;
}

float cool_call_target(float heat_threshold, float cool_threshold) {
  return clamp_cmd(cool_threshold - pull_c(heat_threshold, cool_threshold));
}

float heat_call_target(float heat_threshold, float cool_threshold) {
  return clamp_cmd(heat_threshold + pull_c(heat_threshold, cool_threshold));
}

// 100% until the room reaches the commanded target; past it, one rung down per
// SENSOR_STEP_C of depth, so circulation continues while the room coasts.
uint8_t fan_index_for_delta(float delta) {
  if (delta > 0.0f) return FAN_IDX_MAX;
  const int steps_past = (int)floorf(-delta / SENSOR_STEP_C);  // 0 at the target
  const int idx = (int)FAN_IDX_MAX - 1 - steps_past;
  return idx < (int)FAN_IDX_MIN ? FAN_IDX_MIN : (uint8_t)idx;
}

DecisionOutput ac_decide(const DecisionInput& in, DecisionState& state) {
  DecisionOutput out;
  out.power = false;
  out.mode = MODE_IDX_HEAT;
  out.temp = 21.0;
  out.fan_idx = 0;
  out.control_delta_c = 0.0f;

  // One call-cycle state machine for every target mode: Smart Auto runs both
  // sides, explicit HEAT/COOL run the same machine with the other side disabled.
  if (in.active) {
    if (in.target_mode <= HK_TARGET_COOL) {
      const bool heat_en = (in.target_mode != HK_TARGET_COOL);
      const bool cool_en = (in.target_mode != HK_TARGET_HEAT);
      if (in.room_temp > 1.0) {
        const float cool_target = cool_call_target(in.heat_threshold, in.cool_threshold);
        const float heat_target = heat_call_target(in.heat_threshold, in.cool_threshold);
        // Release is inclusive: target and sensor share the 0.5C grid, so the
        // room lands ON the target and a strict test would overshoot one step.
        if (state.call_state == CALL_COOL && cool_en) {
          if (heat_en && in.room_temp < in.heat_threshold) state.call_state = CALL_HEAT;
          else if (in.room_temp <= cool_target) state.call_state = CALL_IDLE;
        } else if (state.call_state == CALL_HEAT && heat_en) {
          if (cool_en && in.room_temp > in.cool_threshold) state.call_state = CALL_COOL;
          else if (in.room_temp >= heat_target) state.call_state = CALL_IDLE;
        } else {
          state.call_state = (heat_en && in.room_temp < in.heat_threshold) ? CALL_HEAT
                           : (cool_en && in.room_temp > in.cool_threshold) ? CALL_COOL
                           : CALL_IDLE;
        }
        // DRY covers the whole idle band with no floor, so it may cool the room
        // into a HEAT call; that hands back once satisfied and is intended.
        out.power = true;
        out.mode  = (state.call_state == CALL_HEAT) ? MODE_IDX_HEAT
                  : (state.call_state == CALL_COOL) ? MODE_IDX_COOL
                  : (in.dehumidify ? MODE_IDX_DRY : MODE_IDX_FAN);
        out.temp  = (state.call_state == CALL_HEAT || !cool_en)
                        ? heat_target
                        : cool_target; // ignored when mode is FAN or DRY
      } else if (!cool_en) {
        // Room unknown: no cycle to run, hand the unit its own thermostat.
        out.power = true; out.mode = MODE_IDX_HEAT; out.temp = in.heat_threshold;
      } else if (!heat_en) {
        out.power = true; out.mode = MODE_IDX_COOL; out.temp = in.cool_threshold;
      } else {
        out.power = true; out.mode = MODE_IDX_AUTO; // native AUTO
      }
    }
  } else if (in.dehumidify) {
    out.power = true; out.mode = MODE_IDX_DRY; // standalone DRY while the accessory is off
  }

  // --- Fan target: GootAC drives the fan directly. 0 = delegate to unit AUTO. ---
  {
    bool room_known = (in.room_temp > 1.0f);
    bool call = out.power && room_known &&
                (out.mode == MODE_IDX_COOL || out.mode == MODE_IDX_HEAT);
    bool idle_fan = out.power && room_known && out.mode == MODE_IDX_FAN;
    if (call || idle_fan) {
      float delta;
      if (call) {
        delta = (out.mode == MODE_IDX_COOL) ? (in.room_temp - out.temp)
                                            : (out.temp - in.room_temp);
      } else {
        // Idle band: depth past the target, so the wind-down continues across
        // a release. Auto anchors on the nearer side; explicit on its own.
        const float dc = in.room_temp - cool_call_target(in.heat_threshold, in.cool_threshold);
        const float dh = heat_call_target(in.heat_threshold, in.cool_threshold) - in.room_temp;
        delta = (in.target_mode == HK_TARGET_COOL) ? dc
              : (in.target_mode == HK_TARGET_HEAT) ? dh
              : (dc > dh ? dc : dh);
      }
      uint8_t desired = fan_index_for_delta(delta);
      // Up at once; down only after sustained lower demand, which rate-bounds
      // the packet chatter from a reading dithering across a level boundary.
      if (state.last_fan_idx < FAN_IDX_MIN || desired > state.last_fan_idx) {
        state.last_fan_idx = desired; state.lower_since = 0;
      } else if (desired < state.last_fan_idx) {
        if (state.lower_since == 0) state.lower_since = in.now_ms;
        if (in.now_ms - state.lower_since >= FAN_STEP_DOWN_MS) { state.last_fan_idx = desired; state.lower_since = 0; }
      } else {
        state.lower_since = 0;
      }
      out.fan_idx = state.last_fan_idx;
      out.control_delta_c = delta;
    } else {
      out.fan_idx = 0; state.last_fan_idx = 0; state.lower_since = 0; out.control_delta_c = 0.0f;
    }
  }

  return out;
}

uint8_t comp_sig(bool power, uint8_t mode_idx) {
  if (!power || mode_idx == MODE_IDX_FAN) return COMP_SIG_IDLE;
  if (mode_idx == MODE_IDX_HEAT) return COMP_SIG_HEAT;
  if (mode_idx == MODE_IDX_AUTO) return COMP_SIG_AUTO;
  return COMP_SIG_COOL;  // DRY, COOL
}

bool comp_gate_admit(CompGateState& st, uint8_t sig, bool user_write, uint32_t now_ms) {
  if (sig != st.sig && st.sig != COMP_SIG_NONE && !user_write &&
      now_ms - st.since < COMP_MIN_DWELL_MS)
    return false;
  if (sig != st.sig) { st.sig = sig; st.since = now_ms; }
  return true;
}

void comp_gate_observe_external(CompGateState& st, uint8_t sig, uint32_t now_ms) {
  if (sig != st.sig) { st.sig = sig; st.since = now_ms; }
}
