// Host tests for the pure decision core (src/ac_decision.*).
// Groups:
//   A - fan_index_for_delta quantization table
//   B - single-tick edge cases (current-state doc §9, decide()-domain only)
//   C - Smart-Auto hysteresis sequences
//   D - fan-ramp timing (step-down boundary, wrap, sentinel quirk)
//   E - oracle equivalence sweep vs the a805f40 reference
//   F - determinism
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
// ---------------------------------------------------------------------------
static void decide_reference(bool target_active, uint8_t hk_target_mode,
                             float heatThr, float coolThr, bool dehumidify,
                             float roomTemp, uint32_t now_ms_in,
                             int8_t &sa_mode, uint8_t &last_fan_idx,
                             uint32_t &lower_since, bool &out_power,
                             uint8_t &out_mode, float &out_temp,
                             uint8_t &out_fan, float &out_delta) {
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
        if (sa_mode == 2) {
          if (roomTemp < heatThr) sa_mode = 0;
          else if (roomTemp < coolThr - MODE_HYST_C) sa_mode = 3;
        } else if (sa_mode == 0) {
          if (roomTemp > coolThr) sa_mode = 2;
          else if (roomTemp > heatThr + MODE_HYST_C) sa_mode = 3;
        } else {
          if (roomTemp < heatThr) sa_mode = 0;
          else if (roomTemp > coolThr) sa_mode = 2;
          else sa_mode = 3;
        }
        hpPower = true;
        hpMode  = (sa_mode == 0) ? 0 : (sa_mode == 2) ? 2 : 3;
        hpTemp  = (sa_mode == 0) ? heatThr : coolThr; // ignored when hpMode==3
      } else {
        hpPower = true; hpMode = 4; // Native AUTO if room temp unknown
      }
    }
  }

  uint8_t fan_idx = 0;
  float g_control_delta_c = 0.0f;
  {
    bool room_known = (roomTemp > 1.0f);
    uint32_t now_ms = now_ms_in;
    if (hpPower && room_known && (hpMode == 2 || hpMode == 0)) {
      float delta = (hpMode == 2) ? (roomTemp - coolThr) : (heatThr - roomTemp);
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
  // Above setpoint: idx = clamp(lround(0.25*4^(clamp(delta,0,1.5)/1.5)*4),1,4)+1.
  CHECK_EQI(fan_index_for_delta(-1.0f), 1);   // past setpoint -> QUIET
  CHECK_EQI(fan_index_for_delta(0.0f), 1);    // exactly at setpoint -> QUIET
  CHECK_EQI(fan_index_for_delta(0.2f), 2);    // just above -> "1" (25%)
  CHECK_EQI(fan_index_for_delta(0.5f), 3);
  CHECK_EQI(fan_index_for_delta(0.75f), 3);
  CHECK_EQI(fan_index_for_delta(1.0f), 4);
  CHECK_EQI(fan_index_for_delta(1.30f), 4); // idx-5 boundary is ~1.356
  CHECK_EQI(fan_index_for_delta(1.40f), 5);
  CHECK_EQI(fan_index_for_delta(1.5f), 5);
  CHECK_EQI(fan_index_for_delta(99.0f), 5);
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
    CHECK(o.power); CHECK_EQI(o.mode, 0); CHECK_EQF(o.temp, 18.0f);
    CHECK_EQI(st.sa_mode, 0); CHECK_EQI(o.fan_idx, 5); // delta 16.9 -> max
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
    CHECK_EQF(o.control_delta_c, 26.5f - 25.0f); CHECK_EQI(o.fan_idx, 5);
    CHECK_EQI(st.sa_mode, -1); // explicit modes never touch sa_mode
  }
  // Explicit HEAT.
  {
    DecisionState st;
    DecisionOutput o = ac_decide(mkin(true, 1, 27, 30, false, 26.5f, 1000), st);
    CHECK(o.power); CHECK_EQI(o.mode, 0); CHECK_EQF(o.temp, 27.0f);
    CHECK_EQF(o.control_delta_c, 27.0f - 26.5f); CHECK_EQI(o.fan_idx, 3);
  }
  // Negative delta preserved raw (explicit COOL with room below threshold).
  {
    DecisionState st;
    DecisionOutput o = ac_decide(mkin(true, 2, 18, 30, false, 26.5f, 1000), st);
    CHECK(o.power); CHECK_EQI(o.mode, 2);
    CHECK_EQF(o.control_delta_c, 26.5f - 30.0f); // == -3.5, NOT clamped
    CHECK_EQI(o.fan_idx, 1);                     // past setpoint -> QUIET floor
  }
  // Smart-Auto deadband (uninit): FAN mode, temp carries coolThr (ignored by
  // the caller when mode==3 — pinned here so nobody "cleans it up").
  {
    DecisionState st;
    DecisionOutput o = ac_decide(mkin(true, 0, 18, 30, false, 26.5f, 1000), st);
    CHECK(o.power); CHECK_EQI(o.mode, 3); CHECK_EQF(o.temp, 30.0f);
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
    CHECK_EQF(o.temp, 27.5f);
    // and back down releases to deadband (26.0 > 18 + 0.5)
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
  tick(26.0f); CHECK_EQI(st.sa_mode, 2);   // uninit, room > cool -> COOL
  tick(24.6f); CHECK_EQI(st.sa_mode, 2);   // holds: 24.6 >= 24.5 release point
  tick(24.4f); CHECK_EQI(st.sa_mode, 3);   // released below cool - 0.5
  tick(22.3f); CHECK_EQI(st.sa_mode, 3);   // deadband holds
  tick(21.9f); CHECK_EQI(st.sa_mode, 0);   // below heat -> HEAT
  tick(22.4f); CHECK_EQI(st.sa_mode, 0);   // holds: 22.4 <= 22.5 release point
  tick(22.6f); CHECK_EQI(st.sa_mode, 3);   // released above heat + 0.5
  tick(25.1f); CHECK_EQI(st.sa_mode, 2);   // above cool -> COOL

  // Retention across an explicit-mode excursion (AUTO -> COOL -> AUTO).
  (void)ac_decide(mkin(true, 2, H, C, false, 24.6f, t += 5000), st);
  CHECK_EQI(st.sa_mode, 2); // untouched by explicit mode
  DecisionOutput o = ac_decide(mkin(true, 0, H, C, false, 24.6f, t += 5000), st);
  CHECK_EQI(o.mode, 2);     // still COOLing at 24.6 thanks to retained direction

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
    DecisionOutput o = ac_decide(mkin(true, 2, H, 25, false, 26.5f, 1000), st);
    CHECK_EQI(o.fan_idx, 5);
    // demand drops (desired 3): step-down arms, fan holds.
    o = ac_decide(mkin(true, 2, H, 25, false, 25.5f, 10000), st);
    CHECK_EQI(o.fan_idx, 5); CHECK_EQI(st.lower_since, 10000);
    // 59,999 ms elapsed: still holding.
    o = ac_decide(mkin(true, 2, H, 25, false, 25.5f, 69999), st);
    CHECK_EQI(o.fan_idx, 5);
    // 60,000 ms elapsed: drops to the CURRENT desired.
    o = ac_decide(mkin(true, 2, H, 25, false, 25.5f, 70000), st);
    CHECK_EQI(o.fan_idx, 3); CHECK_EQI(st.lower_since, 0);
    // any demand increase is instant.
    o = ac_decide(mkin(true, 2, H, 25, false, 26.5f, 71000), st);
    CHECK_EQI(o.fan_idx, 5);
  }
  // Step-down lands on the desired value AT EXPIRY, not the one that armed it.
  {
    DecisionState st;
    (void)ac_decide(mkin(true, 2, H, 25, false, 26.5f, 1000), st);   // fan 5
    (void)ac_decide(mkin(true, 2, H, 25, false, 25.5f, 2000), st);   // arm @2000 (desired 3)
    DecisionOutput o = ac_decide(mkin(true, 2, H, 25, false, 26.0f, 62000), st); // desired 4 now
    CHECK_EQI(o.fan_idx, 4);
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
    (void)ac_decide(mkin(true, 2, H, 25, false, 26.5f, 0xFFFFFE00u), st); // fan 5
    (void)ac_decide(mkin(true, 2, H, 25, false, 25.5f, 0xFFFFFF00u), st); // arm
    CHECK_EQI(st.lower_since, 0xFFFFFF00u);
    DecisionOutput o = ac_decide(mkin(true, 2, H, 25, false, 25.5f, 59744), st);
    // 59744 - 0xFFFFFF00 (mod 2^32) == 60000 -> steps down.
    CHECK_EQI(o.fan_idx, 3);
  }
  // Preserved quirk: arming exactly at now_ms==0 leaves the sentinel unset,
  // so the countdown silently restarts on the next tick. Same as a805f40.
  {
    DecisionState st;
    (void)ac_decide(mkin(true, 2, H, 25, false, 26.5f, 0xFFFF0000u), st); // fan 5
    (void)ac_decide(mkin(true, 2, H, 25, false, 25.5f, 0), st); // arm at 0 -> no-op
    CHECK_EQI(st.lower_since, 0);
    (void)ac_decide(mkin(true, 2, H, 25, false, 25.5f, 59999), st); // re-arms here
    CHECK_EQI(st.lower_since, 59999);
    DecisionOutput o = ac_decide(mkin(true, 2, H, 25, false, 25.5f, 119999), st);
    CHECK_EQI(o.fan_idx, 3); // drops only 60s after the RE-arm
  }
  // Easing all the way down to the QUIET floor: high demand, then hold the room
  // at/below setpoint (delta <= 0 -> desired QUIET(1)) past the step-down window.
  // Exercises the new floor through the real ramp path, not just the bare curve.
  {
    DecisionState st;
    DecisionOutput o = ac_decide(mkin(true, 2, H, 25, false, 26.5f, 1000), st); // fan 5
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

  for (int act = 0; act <= 1; act++)
   for (int deh = 0; deh <= 1; deh++)
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

            int8_t r_sa = sas[si];
            uint8_t r_fan = fans[fi];
            uint32_t r_low = times[ti][0];
            bool r_pow; uint8_t r_mode; float r_temp; uint8_t r_fidx; float r_delta;
            decide_reference(act, tm, thr[th][0], thr[th][1], deh, room,
                             times[ti][1], r_sa, r_fan, r_low, r_pow, r_mode,
                             r_temp, r_fidx, r_delta);

            DecisionOutput o = ac_decide(in, st_new);
            cases++;
            bool ok = o.power == r_pow && o.mode == r_mode && o.temp == r_temp &&
                      o.fan_idx == r_fidx && o.control_delta_c == r_delta &&
                      st_new.sa_mode == r_sa && st_new.last_fan_idx == r_fan &&
                      st_new.lower_since == r_low;
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
  // Negative delta while Smart-Auto holds COOL inside hysteresis.
  CHECK_EQI(oa.mode, 2);
  CHECK_EQF(oa.control_delta_c, 24.7f - 25.0f);
}

int main() {
  group_a_fan_curve();
  group_b_edge_cases();
  group_c_hysteresis();
  group_d_ramp_timing();
  group_e_oracle_sweep();
  group_f_determinism();
  printf("%d checks, %d failures\n", g_checks, g_fails);
  return g_fails ? 1 : 0;
}
