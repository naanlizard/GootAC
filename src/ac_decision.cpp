#include "ac_decision.h"
#include <math.h>

// Exponential airflow ramp: QUIET at or below setpoint, then 25% just past it up
// to 100% at FAN_RAMP_SPAN_C. Nearest-step quantization keeps every level
// reachable at the AC's 0.5C temp resolution.
uint8_t fan_index_for_delta(float delta) {
  if (delta <= 0.0f) return FAN_IDX_MIN;   // at/below setpoint -> QUIET
  float x = delta / FAN_RAMP_SPAN_C;
  if (x > 1.0f) x = 1.0f;
  int step = (int)lroundf(0.25f * powf(4.0f, x) * 4.0f); // 1..4
  if (step < 1) step = 1;
  if (step > 4) step = 4;
  return (uint8_t)(step + 1);                            // 2..5
}

// Transplanted verbatim from update_physical_ac() at a805f40 (decision core
// :375-410, fan block :437-464). Only mechanical renames: locals -> out.*,
// statics -> state.*, millis() -> in.now_ms. Comparison literals unchanged
// (`> 1.0` in the Smart-Auto branch vs `> 1.0f` in the fan block — as found).
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
        // Hold the current direction until the room crosses back past the
        // setpoint by MODE_HYST_C, so we don't chatter COOL<->FAN at the edge.
        if (state.sa_mode == 2) {
          if (in.room_temp < in.heat_threshold) state.sa_mode = 0;
          else if (in.room_temp < in.cool_threshold - MODE_HYST_C) state.sa_mode = 3;
        } else if (state.sa_mode == 0) {
          if (in.room_temp > in.cool_threshold) state.sa_mode = 2;
          else if (in.room_temp > in.heat_threshold + MODE_HYST_C) state.sa_mode = 3;
        } else {
          if (in.room_temp < in.heat_threshold) state.sa_mode = 0;
          else if (in.room_temp > in.cool_threshold) state.sa_mode = 2;
          else state.sa_mode = 3;
        }
        // Dehumidify replaces the deadband's FAN, but only above
        // heat_threshold + DRY_MARGIN_C: DRY cools, and the deadband-to-HEAT
        // edge above has no release margin, so running it lower would pull the
        // room into a HEAT call and cycle.
        const bool dry_ok = in.dehumidify &&
                            in.room_temp >= in.heat_threshold + DRY_MARGIN_C;
        out.power = true;
        out.mode  = (state.sa_mode == 0) ? 0
                  : (state.sa_mode == 2) ? 2
                  : (dry_ok ? 1 : 3);
        out.temp  = (state.sa_mode == 0) ? in.heat_threshold : in.cool_threshold; // ignored when mode==3 or 1
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
      float delta = (out.mode == 2) ? (in.room_temp - in.cool_threshold)
                                    : (in.heat_threshold - in.room_temp);
      uint8_t desired = fan_index_for_delta(delta);
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
