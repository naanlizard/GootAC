// Host tests for the pure decision core (src/ac_decision.*).
// Groups:
//   A - fan_index_for_delta quantization table
//   B - single-tick edge cases (current-state doc §9, decide()-domain only)
//   C - Smart-Auto hysteresis sequences
//   D - fan-ramp timing (step-down boundary, wrap, sentinel quirk)
//   E - oracle equivalence sweep vs the a805f40 reference (explicit modes only)
//   F - determinism
//   G - dehumidify as an idle-band toggle
//   H - Smart-Auto pull/target table across range widths
//   I - fan ramp stays multi-level inside Smart Auto
//   J - a call always releases into the idle band, never into the opposite call
//   K - normalize_thresholds: the only guard on restored/migrated flash state
#include "ac_decision.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int g_checks = 0, g_fails = 0;

#define CHECK(cond)                                                        \
  do {                                                                     \
    g_checks++;                                                            \
    if (!(cond)) {                                                         \
      g_fails++;                                                           \
      printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);               \
    }                                                                      \
  } while (0)

#define CHECK_EQF(got, want)                                               \
  do {                                                                     \
    g_checks++;                                                            \
    float _g = (got), _w = (want);                                         \
    if (_g != _w) {                                                        \
      g_fails++;                                                           \
      printf("FAIL %s:%d  %s == %.6f, want %.6f\n", __FILE__, __LINE__,    \
             #got, (double)_g, (double)_w);                                \
    }                                                                      \
  } while (0)

#define CHECK_EQI(got, want)                                               \
  do {                                                                     \
    g_checks++;                                                            \
    long _g = (long)(got), _w = (long)(want);                              \
    if (_g != _w) {                                                        \
      g_fails++;                                                           \
      printf("FAIL %s:%d  %s == %ld, want %ld\n", __FILE__, __LINE__,      \
             #got, _g, _w);                                                \
    }                                                                      \
  } while (0)

static DecisionInput mkin(bool active, uint8_t mode, float heat, float cool,
                          bool dehum, float room, uint32_t now) {
  DecisionInput in;
  in.active = active;
  in.target_mode = mode;
  in.heat_threshold = heat;
  in.cool_threshold = cool;
  in.dehumidify = dehum;
  in.room_temp = room;
  in.now_ms = now;
  return in;
}

// ---------------------------------------------------------------------------
// Reference implementation for the oracle sweep (Group E).
// Derived from `git show a805f40:src/ac_controller.cpp` lines 375-410 and
// 437-464; raw extraction stored in ref_extract_a805f40.txt and re-diffed
// against git by the Makefile's check-ref target. Mechanical substitutions
// only: the three function statics became explicit reference parameters,
// millis() became now_ms_in, and the g_* metric globals became outputs.
// The fan curve/floor are NOT frozen here: this reference calls the live
// fan_index_for_delta()/FAN_IDX_MIN, so the sweep verifies the thermal decision
// and ramp state machine, while independent fan-value checks live in group_a.
//
// Smart Auto's *decision* is deliberately absent: its targets and release points
// no longer resemble a805f40, and a reference that called the live
// auto_*_target() would be checking those functions against themselves. group_c,
// group_g and group_h pin the decision with literal expected values instead.
//
// The fan block below is still swept under Smart Auto, because it is the only
// frozen model of the step-down state machine. To avoid the circularity, the
// caller hands in the decision the live core produced rather than recomputing
// it: the sweep checks the ramp GIVEN a decision, never the decision. So for
// Smart Auto rows the power/mode/temp/sa_mode comparisons carry no information
// and are skipped; groups B/C/G/H/I/J are what pin those.
//
// One line of the fan block is NOT verbatim a805f40: the delta expression takes
// hpTemp rather than the raw thresholds, matching the change under review. That
// is a no-op for explicit modes, where hpTemp IS the threshold. ref_extract_
// a805f40.txt still holds the original, and check-ref re-extracts from git
// rather than diffing this function, so it does not flag the difference.
// ---------------------------------------------------------------------------
static void decide_reference(bool target_active, uint8_t hk_target_mode,
                             float heatThr, float coolThr, bool dehumidify,
                             float roomTemp, uint32_t now_ms_in,
                             int8_t &sa_mode, uint8_t &last_fan_idx,
                             uint32_t &lower_since, bool &out_power,
                             uint8_t &out_mode, float &out_temp,
                             uint8_t &out_fan, float &out_delta,
                             const DecisionOutput *live) {
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
    } else if (hk_target_mode == 0 && live) {
      // Smart Auto: adopt the live decision verbatim, then let the frozen fan
      // block below run on it. sa_mode is the live core's to own.
      hpPower = live->power; hpMode = live->mode; hpTemp = live->temp;
    }
  }
  (void)sa_mode;

  uint8_t fan_idx = 0;
  float g_control_delta_c = 0.0f;
  {
    bool room_known = (roomTemp > 1.0f);
    uint32_t now_ms = now_ms_in;
    if (hpPower && room_known && (hpMode == 2 || hpMode == 0)) {
      float delta = (hpMode == 2) ? (roomTemp - hpTemp) : (hpTemp - roomTemp);
      uint8_t desired = fan_index_for_delta(delta);
      if (last_fan_idx < FAN_IDX_MIN || desired > last_fan_idx) {
        last_fan_idx = desired; lower_since = 0;
      } else if (desired < last_fan_idx) {
        if (lower_since == 0) lower_since = now_ms;
        if (now_ms - lower_since >= FAN_STEP_DOWN_MS) { last_fan_idx = desired; lower_since = 0; }
      } else {
        lower_since = 0;
      }
      fan_idx = last_fan_idx;
      g_control_delta_c = delta;
    } else if (hpPower && room_known && hpMode == 3) {
      fan_idx = FAN_IDX_MIN; last_fan_idx = FAN_IDX_MIN; lower_since = 0; g_control_delta_c = 0.0f;
    } else {
      fan_idx = 0; last_fan_idx = 0; lower_since = 0; g_control_delta_c = 0.0f;
    }
  }

  out_power = hpPower;
  out_mode = hpMode;
  out_temp = hpTemp;
  out_fan = fan_idx;
  out_delta = g_control_delta_c;
}

// ---------------------------------------------------------------------------
static void group_a_fan_curve() {
  // At or below setpoint (delta <= 0) the floor is QUIET (FAN_IDX_MIN == 1).
  // Above setpoint: idx = clamp(lround(0.25*4^(clamp(delta,0,3.0)/3.0)*4),1,4)+1.
  // Rung boundaries sit where lround crosses .5, i.e. 4^(delta/3) == 1.5/2.5/3.5.
  CHECK_EQI(fan_index_for_delta(-1.0f), 1);   // past setpoint -> QUIET
  CHECK_EQI(fan_index_for_delta(0.0f), 1);    // exactly at setpoint -> QUIET
  CHECK_EQI(fan_index_for_delta(0.5f), 2);    // 25%, on the grid the sensor uses
  CHECK_EQI(fan_index_for_delta(0.85f), 2);   // idx-3 boundary is ~0.877
  CHECK_EQI(fan_index_for_delta(0.90f), 3);
  CHECK_EQI(fan_index_for_delta(1.0f), 3);
  CHECK_EQI(fan_index_for_delta(1.5f), 3);
  CHECK_EQI(fan_index_for_delta(1.95f), 3);   // idx-4 boundary is ~1.983
  CHECK_EQI(fan_index_for_delta(2.0f), 4);
  CHECK_EQI(fan_index_for_delta(2.5f), 4);
  CHECK_EQI(fan_index_for_delta(2.70f), 4);   // idx-5 boundary is ~2.711
  CHECK_EQI(fan_index_for_delta(2.75f), 5);
  CHECK_EQI(fan_index_for_delta(3.0f), 5);
  CHECK_EQI(fan_index_for_delta(99.0f), 5);

  // Every rung is reachable once room and setpoint share the 0.5C grid, which
  // is the point of the 3.0 span: at 1.5 the 25% rung spanned 0 to ~0.44 and
  // nothing landed in it.
  bool seen[6] = {false, false, false, false, false, false};
  for (float d = 0.0f; d <= 6.0f; d += 0.5f) seen[fan_index_for_delta(d)] = true;
  for (int i = 1; i <= 5; i++) CHECK(seen[i]);
}

static void group_b_edge_cases() {
  // §9.1 room unknown -> native AUTO, 21.0 setpoint carried, fan delegate.
  {
    DecisionState st;
    DecisionOutput o = ac_decide(mkin(true, 0, 18, 30, false, 0.0f, 1000), st);
    CHECK(o.power); CHECK_EQI(o.mode, 4); CHECK_EQF(o.temp, 21.0f);
    CHECK_EQI(o.fan_idx, 0); CHECK_EQF(o.control_delta_c, 0.0f);
    CHECK_EQI(st.sa_mode, -1); CHECK_EQI(st.last_fan_idx, 0); CHECK_EQI(st.lower_since, 0);
  }
  // Boundary: room exactly 1.0 still counts as unknown (`> 1.0`).
  {
    DecisionState st;
    DecisionOutput o = ac_decide(mkin(true, 0, 18, 30, false, 1.0f, 1000), st);
    CHECK(o.power); CHECK_EQI(o.mode, 4); CHECK_EQI(st.sa_mode, -1);
  }
  // Just above the boundary counts as known -> Smart-Auto runs (room << heat -> HEAT).
  {
    DecisionState st;
    DecisionOutput o = ac_decide(mkin(true, 0, 18, 30, false, 1.0625f, 1000), st);
    CHECK(o.power); CHECK_EQI(o.mode, 0);
    CHECK_EQF(o.temp, 20.0f);   // heat 18 + pull 2.0 (range 12 caps at AUTO_PULL_MAX_C)
    CHECK_EQI(st.sa_mode, 0); CHECK_EQI(o.fan_idx, 5); // delta 18.9 -> max
  }
  // §9.12-style: out-of-range-low sensor value still counts as "known".
  {
    DecisionState st;
    DecisionOutput o = ac_decide(mkin(true, 0, 18, 30, false, 5.0f, 1000), st);
    CHECK(o.power); CHECK_EQI(o.mode, 0); CHECK_EQI(o.fan_idx, 5);
  }
  // §9.2 DRY wins over active==0; setpoint default 21.0; fan delegates.
  {
    DecisionState st;
    DecisionOutput o = ac_decide(mkin(false, 0, 18, 30, true, 26.0f, 1000), st);
    CHECK(o.power); CHECK_EQI(o.mode, 1); CHECK_EQF(o.temp, 21.0f);
    CHECK_EQI(o.fan_idx, 0); CHECK_EQF(o.control_delta_c, 0.0f);
    CHECK_EQI(st.last_fan_idx, 0);
  }
  // §9.3 target off: power false, mode 0, temp default; fan memory resets
  // but Smart-Auto direction memory is retained.
  {
    DecisionState st; st.sa_mode = 2; st.last_fan_idx = 4; st.lower_since = 5000;
    DecisionOutput o = ac_decide(mkin(false, 0, 18, 30, false, 26.0f, 1000), st);
    CHECK(!o.power); CHECK_EQI(o.mode, 0); CHECK_EQF(o.temp, 21.0f);
    CHECK_EQI(o.fan_idx, 0);
    CHECK_EQI(st.sa_mode, 2); CHECK_EQI(st.last_fan_idx, 0); CHECK_EQI(st.lower_since, 0);
  }
  // Explicit COOL.
  {
    DecisionState st;
    DecisionOutput o = ac_decide(mkin(true, 2, 18, 25, false, 26.5f, 1000), st);
    CHECK(o.power); CHECK_EQI(o.mode, 2); CHECK_EQF(o.temp, 25.0f);
    CHECK_EQF(o.control_delta_c, 26.5f - 25.0f); CHECK_EQI(o.fan_idx, 3);
    CHECK_EQI(st.sa_mode, -1); // explicit modes never touch sa_mode
  }
  // Explicit HEAT.
  {
    DecisionState st;
    DecisionOutput o = ac_decide(mkin(true, 1, 27, 30, false, 26.5f, 1000), st);
    CHECK(o.power); CHECK_EQI(o.mode, 0); CHECK_EQF(o.temp, 27.0f);
    CHECK_EQF(o.control_delta_c, 27.0f - 26.5f); CHECK_EQI(o.fan_idx, 2);
  }
  // Negative delta preserved raw (explicit COOL with room below threshold).
  {
    DecisionState st;
    DecisionOutput o = ac_decide(mkin(true, 2, 18, 30, false, 26.5f, 1000), st);
    CHECK(o.power); CHECK_EQI(o.mode, 2);
    CHECK_EQF(o.control_delta_c, 26.5f - 30.0f); // == -3.5, NOT clamped
    CHECK_EQI(o.fan_idx, 1);                     // past setpoint -> QUIET floor
  }
  // Smart-Auto deadband (uninit): FAN mode, temp carries the cooling target
  // (ignored by the caller when mode==3, pinned here so nobody "cleans it up").
  {
    DecisionState st;
    DecisionOutput o = ac_decide(mkin(true, 0, 18, 30, false, 26.5f, 1000), st);
    CHECK(o.power); CHECK_EQI(o.mode, 3);
    CHECK_EQF(o.temp, 28.0f);   // cool 30 - pull 2.0
    CHECK_EQI(o.fan_idx, FAN_IDX_MIN); CHECK_EQF(o.control_delta_c, 0.0f);
    CHECK_EQI(st.sa_mode, 3); CHECK_EQI(st.last_fan_idx, FAN_IDX_MIN);
  }
  // Threshold-move-under-sticky-sa_mode (the attended HEAT-step shape):
  // deadband first, then heat raised above room -> HEAT direction.
  {
    DecisionState st;
    (void)ac_decide(mkin(true, 0, 18, 30, false, 26.0f, 1000), st);
    CHECK_EQI(st.sa_mode, 3);
    DecisionOutput o = ac_decide(mkin(true, 0, 27.5f, 30, false, 26.0f, 2000), st);
    CHECK_EQI(st.sa_mode, 0); CHECK(o.power); CHECK_EQI(o.mode, 0);
    CHECK_EQF(o.temp, 28.5f);   // heat 27.5 + pull 1.0 (range 2.5 floors at AUTO_PULL_MIN_C)
    // and back down releases to the idle band (26.0 >= heat target 20.0)
    DecisionOutput o2 = ac_decide(mkin(true, 0, 18, 30, false, 26.0f, 3000), st);
    CHECK_EQI(st.sa_mode, 3); CHECK_EQI(o2.mode, 3);
  }
}

static void group_c_hysteresis() {
  DecisionState st;
  const float H = 22.0f, C = 25.0f;
  uint32_t t = 0;
  auto tick = [&](float room) {
    return ac_decide(mkin(true, 0, H, C, false, room, t += 5000), st);
  };
  // Range 3.0 -> pull floors at AUTO_PULL_MIN_C: cool target 24.0, heat target 23.0.
  CHECK_EQF(auto_cool_target(H, C), 24.0f);
  CHECK_EQF(auto_heat_target(H, C), 23.0f);
  tick(26.0f); CHECK_EQI(st.sa_mode, 2);   // uninit, room > cool -> COOL
  tick(24.5f); CHECK_EQI(st.sa_mode, 2);   // holds: still above the cool target
  tick(24.0f); CHECK_EQI(st.sa_mode, 3);   // reached the target -> released
  tick(22.5f); CHECK_EQI(st.sa_mode, 3);   // idle band holds
  tick(21.9f); CHECK_EQI(st.sa_mode, 0);   // below heat -> HEAT
  tick(22.5f); CHECK_EQI(st.sa_mode, 0);   // holds: still below the heat target
  tick(23.0f); CHECK_EQI(st.sa_mode, 3);   // reached the target -> released
  tick(25.1f); CHECK_EQI(st.sa_mode, 2);   // above cool -> COOL

  // Retention across an explicit-mode excursion (AUTO -> COOL -> AUTO).
  (void)ac_decide(mkin(true, 2, H, C, false, 24.5f, t += 5000), st);
  CHECK_EQI(st.sa_mode, 2); // untouched by explicit mode
  DecisionOutput o = ac_decide(mkin(true, 0, H, C, false, 24.5f, t += 5000), st);
  CHECK_EQI(o.mode, 2);     // still COOLing at 24.5 thanks to retained direction

  // Fresh state mid-deadband derives FAN from thresholds alone.
  DecisionState fresh;
  DecisionOutput o2 = ac_decide(mkin(true, 0, H, C, false, 23.5f, 1000), fresh);
  CHECK_EQI(fresh.sa_mode, 3); CHECK_EQI(o2.mode, 3);
}

static void group_d_ramp_timing() {
  const float H = 16.0f;
  // Explicit COOL at cool=25; room chooses the desired index.
  {
    DecisionState st;
    // t0: high demand -> instant max.
    DecisionOutput o = ac_decide(mkin(true, 2, H, 25, false, 28.0f, 1000), st);
    CHECK_EQI(o.fan_idx, 5);
    // demand drops (desired 2): step-down arms, fan holds.
    o = ac_decide(mkin(true, 2, H, 25, false, 25.5f, 10000), st);
    CHECK_EQI(o.fan_idx, 5); CHECK_EQI(st.lower_since, 10000);
    // 59,999 ms elapsed: still holding.
    o = ac_decide(mkin(true, 2, H, 25, false, 25.5f, 69999), st);
    CHECK_EQI(o.fan_idx, 5);
    // 60,000 ms elapsed: drops to the CURRENT desired.
    o = ac_decide(mkin(true, 2, H, 25, false, 25.5f, 70000), st);
    CHECK_EQI(o.fan_idx, 2); CHECK_EQI(st.lower_since, 0);
    // any demand increase is instant.
    o = ac_decide(mkin(true, 2, H, 25, false, 28.0f, 71000), st);
    CHECK_EQI(o.fan_idx, 5);
  }
  // Step-down lands on the desired value AT EXPIRY, not the one that armed it.
  {
    DecisionState st;
    (void)ac_decide(mkin(true, 2, H, 25, false, 28.0f, 1000), st);   // fan 5
    (void)ac_decide(mkin(true, 2, H, 25, false, 25.5f, 2000), st);   // arm @2000 (desired 2)
    DecisionOutput o = ac_decide(mkin(true, 2, H, 25, false, 26.0f, 62000), st); // desired 3 now
    CHECK_EQI(o.fan_idx, 3);
  }
  // Deadband snaps to the floor and clears ramp memory.
  {
    DecisionState st; st.last_fan_idx = 5; st.lower_since = 12345;
    DecisionOutput o = ac_decide(mkin(true, 0, 18, 30, false, 26.0f, 1000), st);
    CHECK_EQI(o.fan_idx, FAN_IDX_MIN);
    CHECK_EQI(st.last_fan_idx, FAN_IDX_MIN); CHECK_EQI(st.lower_since, 0);
  }
  // millis() wrap across zero: armed near UINT32_MAX, expires after wrap.
  {
    DecisionState st;
    (void)ac_decide(mkin(true, 2, H, 25, false, 28.0f, 0xFFFFFE00u), st); // fan 5
    (void)ac_decide(mkin(true, 2, H, 25, false, 25.5f, 0xFFFFFF00u), st); // arm
    CHECK_EQI(st.lower_since, 0xFFFFFF00u);
    DecisionOutput o = ac_decide(mkin(true, 2, H, 25, false, 25.5f, 59744), st);
    // 59744 - 0xFFFFFF00 (mod 2^32) == 60000 -> steps down.
    CHECK_EQI(o.fan_idx, 2);
  }
  // Preserved quirk: arming exactly at now_ms==0 leaves the sentinel unset,
  // so the countdown silently restarts on the next tick. Same as a805f40.
  {
    DecisionState st;
    (void)ac_decide(mkin(true, 2, H, 25, false, 28.0f, 0xFFFF0000u), st); // fan 5
    (void)ac_decide(mkin(true, 2, H, 25, false, 25.5f, 0), st); // arm at 0 -> no-op
    CHECK_EQI(st.lower_since, 0);
    (void)ac_decide(mkin(true, 2, H, 25, false, 25.5f, 59999), st); // re-arms here
    CHECK_EQI(st.lower_since, 59999);
    DecisionOutput o = ac_decide(mkin(true, 2, H, 25, false, 25.5f, 119999), st);
    CHECK_EQI(o.fan_idx, 2); // drops only 60s after the RE-arm
  }
  // Easing all the way down to the QUIET floor: high demand, then hold the room
  // at/below setpoint (delta <= 0 -> desired QUIET(1)) past the step-down window.
  // Exercises the new floor through the real ramp path, not just the bare curve.
  {
    DecisionState st;
    DecisionOutput o = ac_decide(mkin(true, 2, H, 25, false, 28.0f, 1000), st); // fan 5
    CHECK_EQI(o.fan_idx, 5);
    o = ac_decide(mkin(true, 2, H, 25, false, 24.5f, 2000), st);   // delta -0.5, arm @2000
    CHECK_EQI(o.fan_idx, 5); CHECK_EQI(st.lower_since, 2000);
    o = ac_decide(mkin(true, 2, H, 25, false, 24.5f, 61999), st);  // still holding
    CHECK_EQI(o.fan_idx, 5);
    o = ac_decide(mkin(true, 2, H, 25, false, 24.5f, 62000), st);  // 60s up -> QUIET
    CHECK_EQI(o.fan_idx, 1); CHECK_EQI(st.last_fan_idx, 1); CHECK_EQI(st.lower_since, 0);
    o = ac_decide(mkin(true, 2, H, 25, false, 24.5f, 63000), st);  // holds at QUIET
    CHECK_EQI(o.fan_idx, 1);
  }
}

static void group_e_oracle_sweep() {
  const float rooms_fixed[] = {0.0f, 0.5f, 1.0f, 1.0625f, 5.0f};
  const float thr[][2] = {{18, 30}, {22, 28}, {20, 21}, {16, 31}, {25, 25}};
  const int8_t sas[] = {-1, 0, 2, 3};
  const uint8_t fans[] = {0, 2, 4, 5};
  const uint32_t times[][2] = {{0, 1000}, {5000, 64999}, {5000, 65000}, {5000, 70000}};
  long cases = 0, mismatches = 0;

  // deh stays 0: dehumidify now deliberately diverges from the a805f40 oracle
  // and is pinned by group_g instead. tm 0 IS swept, but only through the fan
  // block — see the note on decide_reference.
  for (int act = 0; act <= 1; act++)
   for (int deh = 0; deh <= 0; deh++)
    for (uint8_t tm = 0; tm <= 2; tm++)
     for (int ri = 0; ri < 5 + 81; ri++) {
       float room = ri < 5 ? rooms_fixed[ri] : 15.0f + 0.25f * (ri - 5);
       for (unsigned th = 0; th < 5; th++)
        for (unsigned si = 0; si < 4; si++)
         for (unsigned fi = 0; fi < 4; fi++)
          for (unsigned ti = 0; ti < 4; ti++) {
            DecisionInput in = mkin(act, tm, thr[th][0], thr[th][1], deh,
                                    room, times[ti][1]);
            DecisionState st_new;
            st_new.sa_mode = sas[si];
            st_new.last_fan_idx = fans[fi];
            st_new.lower_since = times[ti][0];

            DecisionOutput o = ac_decide(in, st_new);

            int8_t r_sa = sas[si];
            uint8_t r_fan = fans[fi];
            uint32_t r_low = times[ti][0];
            bool r_pow; uint8_t r_mode; float r_temp; uint8_t r_fidx; float r_delta;
            decide_reference(act, tm, thr[th][0], thr[th][1], deh, room,
                             times[ti][1], r_sa, r_fan, r_low, r_pow, r_mode,
                             r_temp, r_fidx, r_delta, &o);

            // Under Smart Auto the reference adopts the live power/mode/temp, so
            // those three and sa_mode carry no information; the ramp outputs do.
            const bool auto_case = (act && tm == 0);
            cases++;
            bool ok = o.fan_idx == r_fidx && o.control_delta_c == r_delta &&
                      st_new.last_fan_idx == r_fan && st_new.lower_since == r_low &&
                      (auto_case || (o.power == r_pow && o.mode == r_mode &&
                                     o.temp == r_temp && st_new.sa_mode == r_sa));
            if (!ok && mismatches++ < 5)
              printf("FAIL oracle: act=%d deh=%d tm=%u room=%.4f thr=(%.1f,%.1f) "
                     "sa=%d fan=%u low=%u now=%u\n",
                     act, deh, tm, (double)room, (double)thr[th][0],
                     (double)thr[th][1], sas[si], fans[fi], times[ti][0],
                     times[ti][1]);
          }
     }
  g_checks++;
  if (mismatches) { g_fails++; }
  printf("oracle sweep: %ld cases, %ld mismatches\n", cases, mismatches);
}

static void group_f_determinism() {
  DecisionState a; a.sa_mode = 2; a.last_fan_idx = 4; a.lower_since = 500;
  DecisionState b = a; // copy BEFORE the call — decide() mutates its state
  DecisionInput in = mkin(true, 0, 22, 25, false, 24.7f, 30000);
  DecisionOutput oa = ac_decide(in, a);
  DecisionOutput ob = ac_decide(in, b);
  CHECK(oa.power == ob.power && oa.mode == ob.mode && oa.temp == ob.temp &&
        oa.fan_idx == ob.fan_idx && oa.control_delta_c == ob.control_delta_c);
  CHECK(a.sa_mode == b.sa_mode && a.last_fan_idx == b.last_fan_idx &&
        a.lower_since == b.lower_since);
  // Smart Auto measures demand against its target, which is also where the call
  // releases, so demand stays positive for as long as the call runs.
  CHECK_EQI(oa.mode, 2);
  CHECK_EQF(oa.control_delta_c, 24.7f - 24.0f);

  // Negative delta is still preserved rather than clamped: fixed COOL commands
  // the setpoint the user chose, so the room can sit past it.
  DecisionState c; c.sa_mode = 2; c.last_fan_idx = 4; c.lower_since = 500;
  DecisionOutput oc = ac_decide(mkin(true, 2, 22, 25, false, 24.7f, 30000), c);
  CHECK_EQI(oc.mode, 2);
  CHECK_EQF(oc.temp, 25.0f);
  CHECK_EQF(oc.control_delta_c, 24.7f - 25.0f);
  // Demand has gone negative but the fan holds: only 29.5s of the 60s
  // step-down dwell has elapsed, so the ramp has not eased down yet.
  CHECK_EQI(oc.fan_idx, 4);
}

// Group G - dehumidify as an idle-mode toggle. Deliberately diverges from the
// a805f40 oracle, which let dehumidify pre-empt every other input. DRY now
// replaces the Smart-Auto idle band with no floor, so it covers the whole band
// and may cool the room into a HEAT call, and still runs standalone when off.
static DecisionOutput dec1(bool active, uint8_t mode, float heat, float cool,
                           bool dehum, float room, int8_t sa_in = 3) {
  DecisionState st;
  st.sa_mode = sa_in;
  DecisionInput in = mkin(active, mode, heat, cool, dehum, room, 10000);
  return ac_decide(in, st);
}

static void group_g_dehumidify_toggle() {
  // Off plus armed keeps standalone dehumidification, fan left to the unit.
  DecisionOutput o = dec1(false, 0, 18, 24, true, 22.0f);
  CHECK(o.power); CHECK_EQI(o.mode, 1); CHECK_EQI(o.fan_idx, 0);
  o = dec1(false, 0, 18, 24, false, 22.0f);
  CHECK(!o.power);

  // Smart-Auto deadband, cooling side: DRY replaces FAN.
  o = dec1(true, 0, 18, 24, true, 22.0f);
  CHECK(o.power); CHECK_EQI(o.mode, 1); CHECK_EQI(o.fan_idx, 0);
  o = dec1(true, 0, 18, 24, false, 22.0f);
  CHECK_EQI(o.mode, 3); CHECK_EQI(o.fan_idx, FAN_IDX_MIN);

  // DRY has no floor: it covers the idle band all the way to the heat end.
  for (float room = 18.0f; room <= 23.0f; room += 0.5f) {
    o = dec1(true, 0, 18, 24, true, room);
    CHECK_EQI(o.mode, 1);
    o = dec1(true, 0, 18, 24, false, room);
    CHECK_EQI(o.mode, 3);
  }

  // Active conditioning is never displaced by the toggle.
  o = dec1(true, 0, 18, 24, true, 25.0f, 2);
  CHECK_EQI(o.mode, 2);
  o = dec1(true, 0, 18, 24, true, 17.0f, 0);
  CHECK_EQI(o.mode, 0);
  o = dec1(true, 1, 18, 24, true, 22.0f);
  CHECK_EQI(o.mode, 0);
  o = dec1(true, 2, 18, 24, true, 22.0f);
  CHECK_EQI(o.mode, 2);

  // Room unknown has no deadband to gate on, so native AUTO stands.
  o = dec1(true, 0, 18, 24, true, 0.5f);
  CHECK_EQI(o.mode, 4);

  // Descending sweep, armed: COOL down to the release point, then DRY for the
  // whole idle band, then HEAT. DRY handing over to HEAT is intended, since
  // dehumidifying cools; the sweep pins the order, not the absence of a cycle.
  DecisionState st; st.sa_mode = 2;
  int seen[5] = {0, 0, 0, 0, 0};
  int last = -1, order[8], n = 0;
  for (float room = 25.0f; room >= 17.0f; room -= 0.5f) {
    DecisionInput in = mkin(true, 0, 18, 24, true, room, 10000);
    DecisionOutput s = ac_decide(in, st);
    seen[s.mode]++;
    if (s.mode != last && n < 8) { order[n++] = s.mode; last = s.mode; }
  }
  CHECK_EQI(n, 3);
  CHECK_EQI(order[0], 2);  // COOL
  CHECK_EQI(order[1], 1);  // DRY across the idle band
  CHECK_EQI(order[2], 0);  // HEAT
  CHECK_EQI(seen[3], 0);   // never FAN while armed
}

// Group H - where Smart Auto drives the room, as a function of range width.
// Every expected value here is a literal. Nothing in this group may call
// auto_pull_c()/auto_*_target() to compute what it then asserts, or the table
// stops being a specification and becomes a restatement of the code.
static void group_h_pull_table() {
  struct Row { float heat, cool, pull, heat_target, cool_target; };
  // range  pull                       reason
  //  2.0    1.0   AUTO_PULL_MIN_C floor, also capped at half the range
  //  3.0    1.0   AUTO_PULL_MIN_C floor
  //  4.0    1.0   fraction (1.00), on grid
  //  5.0    1.5   fraction (1.25) -> nearest 0.5
  //  6.0    1.5   fraction (1.50), on grid
  //  7.0    2.0   fraction (1.75) -> nearest 0.5
  //  8.0    2.0   fraction (2.00), on grid
  // 12.0+   2.0   AUTO_PULL_MAX_C cap
  static const Row rows[] = {
    {20.0f, 23.0f, 1.0f, 21.0f, 22.0f},
    {20.0f, 24.0f, 1.0f, 21.0f, 23.0f},
    {19.0f, 24.0f, 1.5f, 20.5f, 22.5f},
    {18.0f, 24.0f, 1.5f, 19.5f, 22.5f},
    {18.0f, 25.0f, 2.0f, 20.0f, 23.0f},
    {16.0f, 24.0f, 2.0f, 18.0f, 22.0f},
    {18.0f, 30.0f, 2.0f, 20.0f, 28.0f},
    {16.0f, 31.0f, 2.0f, 18.0f, 29.0f},
    // Odd-half widths, where the grid snap rounds rather than landing exactly.
    // Without these, roundf could be ceilf or floorf and nothing would notice.
    {19.5f, 24.0f, 1.0f, 20.5f, 23.0f},   // 4.5 -> 1.125 rounds down
    {18.5f, 24.0f, 1.5f, 20.0f, 22.5f},   // 5.5 -> 1.375 rounds up
    {17.5f, 24.0f, 1.5f, 19.0f, 22.5f},   // 6.5 -> 1.625 rounds down
    {16.5f, 24.0f, 2.0f, 18.5f, 22.0f},   // 7.5 -> 1.875 rounds up
    // Below the setters' minimum gap. Unreachable from HomeKit, kept because the
    // core must stay stable if a stored state or a future caller supplies one.
    // 2.0 is the width where both targets collapse onto the midpoint, which is
    // why THRESHOLD_MIN_GAP_C sits above it.
    {20.0f, 22.0f, 1.0f, 21.0f, 21.0f},
    {20.0f, 21.0f, 0.5f, 20.5f, 20.5f},
    {20.0f, 20.5f, 0.0f, 20.0f, 20.5f},
    {25.0f, 25.0f, 0.0f, 25.0f, 25.0f},
  };
  for (const Row &r : rows) {
    CHECK_EQF(auto_pull_c(r.heat, r.cool), r.pull);
    CHECK_EQF(auto_heat_target(r.heat, r.cool), r.heat_target);
    CHECK_EQF(auto_cool_target(r.heat, r.cool), r.cool_target);
  }

  // No range the setters permit parks on its own midpoint. Coincidence needs
  // pull == range/2, which cannot happen at or above THRESHOLD_MIN_GAP_C: the
  // pull floors at 1.0 while half the range is already 1.5. (At 2.0C, below the
  // minimum, both targets ARE the midpoint — see the degenerate rows above.)
  for (float heat = 5.0f; heat <= 40.0f - THRESHOLD_MIN_GAP_C; heat += 0.5f)
    for (float cool = heat + THRESHOLD_MIN_GAP_C; cool <= 40.0f; cool += 0.5f) {
      const float mid = 0.5f * (heat + cool);
      CHECK(auto_cool_target(heat, cool) > mid);
      CHECK(auto_heat_target(heat, cool) < mid);
    }

  // Structural invariants over every threshold pair the core can be handed.
  // The targets are only guaranteed commandable because they stay between the
  // thresholds, so the bound the AC cares about is enforced by whoever supplies
  // the thresholds — normalize_thresholds() below, not this function. Feeding it
  // an out-of-band pair really does produce an out-of-band setpoint.
  const float grid_lo = 5.0f, grid_hi = 40.0f;   // characteristic min/max
  CHECK_EQF(auto_cool_target(2.0f, 50.0f), 48.0f);
  CHECK_EQF(auto_heat_target(0.0f, 2.0f), 1.0f);
  for (float heat = grid_lo; heat <= grid_hi; heat += 0.5f)
    for (float cool = heat; cool <= grid_hi; cool += 0.5f) {
      const float ct = auto_cool_target(heat, cool);
      const float ht = auto_heat_target(heat, cool);
      CHECK(ht <= ct);                      // targets never cross
      CHECK(ct >= heat && ct <= cool);      // releasing COOL cannot trip HEAT
      CHECK(ht >= heat && ht <= cool);      // releasing HEAT cannot trip COOL
    }
}

// Group K - normalize_thresholds() is the only thing standing between a restored
// flash pair and a decision core that has no defences of its own. An inverted
// pair makes ac_decide alternate HEAT/COOL on every tick with the room
// stationary, so this is checked first as a live failure, then as a repair.
static void group_k_normalize() {
  const float LO = 5.0f, HI = 40.0f;   // characteristic min/max
  CHECK(HI - LO >= THRESHOLD_MIN_GAP_C);   // precondition the function assumes

  // The failure being defended against: inverted thresholds, room held still.
  {
    DecisionState st;
    int flips = 0, last = -1;
    for (int i = 0; i < 12; i++) {
      DecisionOutput o = ac_decide(mkin(true, 0, 25.0f, 20.0f, false, 22.0f, 10000), st);
      if (last >= 0 && o.mode != last) flips++;
      last = o.mode;
    }
    CHECK_EQI(flips, 11);   // every single tick is a mode change
  }

  struct Row { float in_h, in_c; ThresholdAuthority who; float out_h, out_c; bool moved; };
  static const Row rows[] = {
    // Already conforming: untouched under every authority.
    {18.0f, 24.0f, THRESHOLD_FROM_HEAT_WRITE, 18.0f, 24.0f, false},
    {18.0f, 24.0f, THRESHOLD_FROM_COOL_WRITE, 18.0f, 24.0f, false},
    {18.0f, 24.0f, THRESHOLD_UNTRUSTED,       18.0f, 24.0f, false},
    {5.0f,  8.0f,  THRESHOLD_FROM_HEAT_WRITE, 5.0f,  8.0f,  false},  // exactly the gap
    // Too narrow: the value the caller did NOT write is the one that moves.
    {20.0f, 21.0f, THRESHOLD_FROM_HEAT_WRITE, 20.0f, 23.0f, true},
    {20.0f, 21.0f, THRESHOLD_FROM_COOL_WRITE, 18.0f, 21.0f, true},
    {22.0f, 22.0f, THRESHOLD_FROM_HEAT_WRITE, 22.0f, 25.0f, true},
    // Too narrow AND against a bound: only then does the written value give way.
    {5.0f,  6.0f,  THRESHOLD_FROM_COOL_WRITE, 5.0f,  8.0f,  true},
    {39.0f, 40.0f, THRESHOLD_FROM_HEAT_WRITE, 37.0f, 40.0f, true},
    // Inverted by a write. The written value MUST survive: swapping here would
    // land it on the other characteristic, so writing cool=17 over 25/27 would
    // silently become heat=17. Resolve by moving the untouched value instead.
    {25.0f, 17.0f, THRESHOLD_FROM_COOL_WRITE, 14.0f, 17.0f, true},
    {25.0f, 24.0f, THRESHOLD_FROM_HEAT_WRITE, 25.0f, 28.0f, true},
    {39.0f, 24.0f, THRESHOLD_FROM_HEAT_WRITE, 37.0f, 40.0f, true},
    {22.0f, 19.0f, THRESHOLD_FROM_COOL_WRITE, 16.0f, 19.0f, true},
    // Inverted in restored state: neither value is trusted, so reorder.
    {25.0f, 20.0f, THRESHOLD_UNTRUSTED, 20.0f, 25.0f, true},
    {24.0f, 23.0f, THRESHOLD_UNTRUSTED, 23.0f, 26.0f, true},
    {28.0f, 24.0f, THRESHOLD_UNTRUSTED, 24.0f, 28.0f, true},
    // Reorder first even for a one-step inversion, so the repaired range sits
    // where the stored values were rather than a whole gap above them.
    {24.0f, 23.5f, THRESHOLD_UNTRUSTED, 23.5f, 26.5f, true},
    // Outside the characteristic band entirely (the legacy 10-40 migration
    // admits values this wide, and pre-1.33 flash holds pairs set under the
    // old 16-31 declaration).
    {0.0f,  50.0f, THRESHOLD_UNTRUSTED, 5.0f,  40.0f, true},
    {12.0f, 12.5f, THRESHOLD_UNTRUSTED, 12.0f, 15.0f, true},
    {2.0f,  3.0f,  THRESHOLD_UNTRUSTED, 5.0f,  8.0f,  true},
    {45.0f, 48.0f, THRESHOLD_UNTRUSTED, 37.0f, 40.0f, true},
  };
  for (const Row &r : rows) {
    float h = r.in_h, c = r.in_c;
    CHECK_EQI(normalize_thresholds(h, c, LO, HI, r.who), r.moved);
    CHECK_EQF(h, r.out_h);
    CHECK_EQF(c, r.out_c);
  }

  // The value a client wrote survives unless a bound forces otherwise. Sweep
  // every single-characteristic write over every conforming starting pair.
  for (float h0 = LO; h0 <= HI - THRESHOLD_MIN_GAP_C; h0 += 0.5f)
    for (float c0 = h0 + THRESHOLD_MIN_GAP_C; c0 <= HI; c0 += 0.5f)
      for (float v = LO; v <= HI; v += 0.5f) {
        float h = h0, c = v;   // client wrote the cooling threshold
        normalize_thresholds(h, c, LO, HI, THRESHOLD_FROM_COOL_WRITE);
        CHECK(c == v || v < LO + THRESHOLD_MIN_GAP_C);
        h = v; c = c0;         // client wrote the heating threshold
        normalize_thresholds(h, c, LO, HI, THRESHOLD_FROM_HEAT_WRITE);
        CHECK(h == v || v > HI - THRESHOLD_MIN_GAP_C);
      }

  // Non-finite input cannot reach ac_decide: NaN makes every comparison false,
  // which pins sa_mode and commands a NaN setpoint. Only the corrupt half is
  // replaced; the other may still be exactly what the user set.
  {
    float h = NAN, c = 24.0f;
    CHECK(normalize_thresholds(h, c, LO, HI, THRESHOLD_UNTRUSTED));
    CHECK_EQF(h, LO); CHECK_EQF(c, 24.0f);
    h = 18.0f; c = INFINITY;
    CHECK(normalize_thresholds(h, c, LO, HI, THRESHOLD_UNTRUSTED));
    CHECK_EQF(h, 18.0f); CHECK_EQF(c, HI);
  }

  // Exhaustive: every pair on the grid, ordered and inverted, in and out of
  // band, both authorities. Output must satisfy the invariant, be idempotent,
  // and never produce a pair ac_decide can flap on.
  static const ThresholdAuthority whos[] = {THRESHOLD_FROM_HEAT_WRITE,
                                            THRESHOLD_FROM_COOL_WRITE,
                                            THRESHOLD_UNTRUSTED};
  for (float h = 0.0f; h <= 50.0f; h += 0.5f)
    for (float c = 0.0f; c <= 50.0f; c += 0.5f)
      for (ThresholdAuthority w : whos) {
        float nh = h, nc = c;
        normalize_thresholds(nh, nc, LO, HI, w);
        CHECK(nh >= LO && nc <= HI);
        CHECK(nc - nh >= THRESHOLD_MIN_GAP_C);
        float ih = nh, ic = nc;
        CHECK(!normalize_thresholds(ih, ic, LO, HI, w));  // idempotent
        // And the repaired pair gives the core a usable idle band.
        CHECK(auto_pull_c(nh, nc) >= AUTO_PULL_MIN_C);
      }
}

// Group I - the fan ramp must stay multi-level inside Smart Auto. Measuring
// demand against a point the call never reaches truncates the ramp to its top
// rung; measuring it against the release target keeps the rungs reachable.
static void group_i_auto_fan_ramp() {
  // heat 18 / cool 24 -> cool target 22.5. Fresh state each tick so the 60s
  // step-down dwell does not mask which index the curve actually asks for.
  // Descending, cool target 22.5. Every rung appears, which is what the 3.0
  // span buys: delta 3.0/2.5/1.5/0.5 -> 100/75/50/25%, then the floor.
  struct { float room; uint8_t mode, fan; } expect[] = {
    {25.5f, 2, 5}, {25.0f, 2, 4}, {24.5f, 2, 4}, {24.0f, 2, 3},
    {23.5f, 2, 3}, {23.0f, 2, 2},
    {22.5f, 3, FAN_IDX_MIN},   // reached the target: released, fan to the floor
  };
  for (auto &e : expect) {
    DecisionState st; st.sa_mode = 2;
    DecisionOutput o = ac_decide(mkin(true, 0, 18, 24, false, e.room, 10000), st);
    CHECK_EQI(o.mode, e.mode);
    CHECK_EQI(o.fan_idx, e.fan);
  }

  // Same on the heating side: heat 18 / cool 24 -> heat target 19.5.
  struct { float room; uint8_t mode, fan; } heat_expect[] = {
    {16.5f, 0, 5}, {17.0f, 0, 4}, {17.5f, 0, 4}, {18.0f, 0, 3},
    {18.5f, 0, 3}, {19.0f, 0, 2},
    {19.5f, 3, FAN_IDX_MIN},
  };
  for (auto &e : heat_expect) {
    DecisionState st; st.sa_mode = 0;
    DecisionOutput o = ac_decide(mkin(true, 0, 18, 24, false, e.room, 10000), st);
    CHECK_EQI(o.mode, e.mode);
    CHECK_EQI(o.fan_idx, e.fan);
  }

  // Both legs together must cover all five rungs, not just three of them.
  bool seen[6] = {false, false, false, false, false, false};
  for (auto &e : expect)      seen[e.fan] = true;
  for (auto &e : heat_expect) seen[e.fan] = true;
  for (int i = 1; i <= 5; i++) CHECK(seen[i]);
}

// Group J - a call must always end in the idle band, never in the opposite
// call. This is the property the previous version broke on narrow ranges: COOL
// released below the heating threshold, HEAT fired, and the unit reversed
// forever. Walk a simulated room across every range the core can see.
static void group_j_no_reversal() {
  for (float heat = 5.0f; heat <= 39.0f; heat += 0.5f)
    for (float cool = heat; cool <= 40.0f; cool += 0.5f) {
      // Enter COOL from above, then run until the mode changes.
      DecisionState st; st.sa_mode = 2;
      float room = cool + 2.0f;
      int guard = 0;
      while (st.sa_mode == 2 && guard++ < 200) {
        (void)ac_decide(mkin(true, 0, heat, cool, false, room, 10000), st);
        if (st.sa_mode == 2) room -= SENSOR_STEP_C;
      }
      CHECK_EQI(st.sa_mode, 3);   // idle band, not a HEAT call
      // Holding the room there must keep it idle rather than re-arming.
      (void)ac_decide(mkin(true, 0, heat, cool, false, room, 20000), st);
      CHECK_EQI(st.sa_mode, 3);

      // Mirror: enter HEAT from below.
      DecisionState st2; st2.sa_mode = 0;
      room = heat - 2.0f;
      guard = 0;
      while (st2.sa_mode == 0 && guard++ < 200) {
        (void)ac_decide(mkin(true, 0, heat, cool, false, room, 10000), st2);
        if (st2.sa_mode == 0) room += SENSOR_STEP_C;
      }
      CHECK_EQI(st2.sa_mode, 3);
      (void)ac_decide(mkin(true, 0, heat, cool, false, room, 20000), st2);
      CHECK_EQI(st2.sa_mode, 3);
    }

  // A running call must also yield DIRECTLY to the opposite call when the user
  // drags a threshold past the room, without passing through the idle band. The
  // walk above can never reach this: it always meets its own target, which is
  // inside the range, before it can cross the far threshold.
  for (float heat = 5.0f; heat <= 40.0f - THRESHOLD_MIN_GAP_C; heat += 0.5f)
    for (float cool = heat + THRESHOLD_MIN_GAP_C; cool <= 40.0f; cool += 0.5f) {
      // Cooling, then the heating threshold is raised above the room.
      DecisionState st; st.sa_mode = 2;
      DecisionOutput o = ac_decide(mkin(true, 0, heat, cool, false, heat - 0.5f, 10000), st);
      CHECK_EQI(st.sa_mode, 0);
      CHECK_EQI(o.mode, 0);
      CHECK_EQF(o.temp, auto_heat_target(heat, cool));
      // Heating, then the cooling threshold is dropped below the room.
      DecisionState st2; st2.sa_mode = 0;
      DecisionOutput o2 = ac_decide(mkin(true, 0, heat, cool, false, cool + 0.5f, 10000), st2);
      CHECK_EQI(st2.sa_mode, 2);
      CHECK_EQI(o2.mode, 2);
      CHECK_EQF(o2.temp, auto_cool_target(heat, cool));
    }
}

int main() {
  group_a_fan_curve();
  group_b_edge_cases();
  group_c_hysteresis();
  group_d_ramp_timing();
  group_e_oracle_sweep();
  group_f_determinism();
  group_g_dehumidify_toggle();
  group_h_pull_table();
  group_i_auto_fan_ramp();
  group_j_no_reversal();
  group_k_normalize();
  printf("%d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
