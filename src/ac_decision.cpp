#include "ac_decision.h"
#include <math.h>

float auto_pull_c(float heat_threshold, float cool_threshold) {
  const float range = cool_threshold - heat_threshold;
  if (range <= 0.0f) return 0.0f;
  float pull = range * AUTO_PULL_FRAC;
  if (pull < AUTO_PULL_MIN_C) pull = AUTO_PULL_MIN_C;
  if (pull > AUTO_PULL_MAX_C) pull = AUTO_PULL_MAX_C;
  pull = SENSOR_STEP_C * roundf(pull / SENSOR_STEP_C);
  // Past half the range the targets cross, so releasing one call lands inside
  // the other and the unit reverses instead of idling. Floored onto the grid.
  const float half = SENSOR_STEP_C * floorf((range * 0.5f) / SENSOR_STEP_C);
  if (pull > half) pull = half;
  return pull;
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

float auto_cool_target(float heat_threshold, float cool_threshold) {
  return cool_threshold - auto_pull_c(heat_threshold, cool_threshold);
}

float auto_heat_target(float heat_threshold, float cool_threshold) {
  return heat_threshold + auto_pull_c(heat_threshold, cool_threshold);
}

// Exponential ramp, QUIET at or below setpoint to 100% at FAN_RAMP_SPAN_C. This
// is the raw curve; the rung hysteresis in ac_decide gates which steps are taken.
uint8_t fan_index_for_delta(float delta) {
  if (delta <= 0.0f) return FAN_IDX_MIN;   // at/below setpoint -> QUIET
  float x = delta / FAN_RAMP_SPAN_C;
  if (x > 1.0f) x = 1.0f;
  int step = (int)lroundf(0.25f * powf(4.0f, x) * 4.0f); // 1..4
  if (step < 1) step = 1;
  if (step > 4) step = 4;
  return (uint8_t)(step + 1);                            // 2..5
}

DecisionOutput ac_decide(const DecisionInput& in, DecisionState& state) {
  DecisionOutput out;
  out.power = false;
  out.mode = 0; // Library Index: 0:HEAT, 1:DRY, 2:COOL, 3:FAN, 4:AUTO
  out.temp = 21.0;
  out.fan_idx = 0;
  out.control_delta_c = 0.0f;

  // Decision Logic
  if (in.active) {
    if (in.target_mode == 1) { // HEAT
      out.power = true; out.mode = 0; out.temp = in.heat_threshold;
    } else if (in.target_mode == 2) { // COOL
      out.power = true; out.mode = 2; out.temp = in.cool_threshold;
    } else if (in.target_mode == 0) { // Smart Auto
      if (in.room_temp > 1.0) {
        const float cool_target = auto_cool_target(in.heat_threshold, in.cool_threshold);
        const float heat_target = auto_heat_target(in.heat_threshold, in.cool_threshold);
        // Release is inclusive: target and sensor share the 0.5C grid, so the
        // room lands ON the target and a strict test would overshoot one step.
        if (state.sa_mode == 2) {
          if (in.room_temp < in.heat_threshold) state.sa_mode = 0;
          else if (in.room_temp <= cool_target) state.sa_mode = 3;
        } else if (state.sa_mode == 0) {
          if (in.room_temp > in.cool_threshold) state.sa_mode = 2;
          else if (in.room_temp >= heat_target) state.sa_mode = 3;
        } else {
          if (in.room_temp < in.heat_threshold) state.sa_mode = 0;
          else if (in.room_temp > in.cool_threshold) state.sa_mode = 2;
          else state.sa_mode = 3;
        }
        // DRY covers the whole idle band with no floor, so it may cool the room
        // into a HEAT call; that hands back once satisfied and is intended.
        out.power = true;
        out.mode  = (state.sa_mode == 0) ? 0
                  : (state.sa_mode == 2) ? 2
                  : (in.dehumidify ? 1 : 3);
        out.temp  = (state.sa_mode == 0) ? heat_target
                                         : cool_target; // ignored when mode==3 or 1
      } else {
        out.power = true; out.mode = 4; // Native AUTO if room temp unknown
      }
    }
  } else if (in.dehumidify) {
    out.power = true; out.mode = 1; // standalone DRY while the accessory is off
  }

  // --- Fan target: GootAC drives the fan directly. 0 = delegate to unit AUTO. ---
  {
    bool room_known = (in.room_temp > 1.0f);
    if (out.power && room_known && (out.mode == 2 || out.mode == 0)) {
      // Against the commanded setpoint, which in Smart Auto is also the release
      // point, so the ramp reaches its bottom rung exactly as the call ends.
      float delta = (out.mode == 2) ? (in.room_temp - out.temp)
                                    : (out.temp - in.room_temp);
      uint8_t desired = fan_index_for_delta(delta);
      // A step up must clear the rung by a whole sensor step. The room moves in
      // 0.5C jumps, so one parked on a boundary would raise the fan every tick
      // and cancel the pending step-down on the next, forever.
      if (desired > state.last_fan_idx &&
          fan_index_for_delta(delta - SENSOR_STEP_C) <= state.last_fan_idx)
        desired = state.last_fan_idx;
      // Ramp up at once; ease down only after sustained lower demand.
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
    } else if (out.power && room_known && out.mode == 3) {
      out.fan_idx = FAN_IDX_MIN; state.last_fan_idx = FAN_IDX_MIN; state.lower_since = 0; out.control_delta_c = 0.0f;
    } else {
      out.fan_idx = 0; state.last_fan_idx = 0; state.lower_since = 0; out.control_delta_c = 0.0f;
    }
  }

  return out;
}
