#pragma once
#include <Arduino.h>
#include "config.h"

// Plausibility bounds on the room-temperature sensor reading: outside them a
// value is a glitch or parse error, rejected by the sync path and declared as
// the HomeKit characteristic min/max. Defaults suit Lisbon; override in config.h.
#ifndef MIN_VALID_ROOM_TEMP_C
#define MIN_VALID_ROOM_TEMP_C 10.0f
#endif
#ifndef MAX_VALID_ROOM_TEMP_C
#define MAX_VALID_ROOM_TEMP_C 45.0f
#endif

// Target RH seed shared by the characteristic, fresh defaults, and migrations.
#define HUMIDITY_THRESHOLD_DEFAULT 55.0f

struct TargetState {
    uint8_t active;              // 0: Off, 1: On
    uint8_t target_mode;         // HomeKit TargetHeaterCoolerState: 0: AUTO, 1: HEAT, 2: COOL
    float cooling_threshold;
    float heating_threshold;
    uint8_t swing_mode;          // 0: OFF, 1: SWING
    uint8_t dehumidifier;        // 0: OFF, 1: ON
    float humidity_threshold;    // Target RH %, HAP dehumidifier threshold
    uint32_t checksum;           // Integrity check
};

// --- Logging ---
// GLOG_* macros and the two-tier logger live in glog.h.
#include "glog.h"

// Specialized HomeKit logging with client context (implemented in ac_controller.cpp)
void hk_log_info(const char* fmt, ...);
#define HKLOG_INFO(fmt, ...) hk_log_info(fmt, ##__VA_ARGS__)

#ifdef __cplusplus
#include <HeatPump.h>
extern "C" {
#else
typedef struct HeatPump HeatPump;
#endif

void ac_controller_init(HeatPump* heatPumpInstance);
void ac_controller_loop();
void ac_controller_sync_from_ac();
void ac_controller_identify();

#ifdef __cplusplus
// Streams Prometheus text-exposition metrics to a Print sink (the web server's
// chunked-response adapter). Replaces the former JSON /status body.
void ac_controller_write_metrics(Print& out);

// HomeKit characteristic setters, shared with the /control HTTP endpoint in
// main.cpp so out-of-band writes take the exact same path as HomeKit writes
// (currentState mirror, pending_update debounce, guards, persistence).
#include <homekit/homekit.h>
void set_ac_active(homekit_value_t value);
void set_ac_target_state(homekit_value_t value);
void set_ac_cooling_threshold(homekit_value_t value);
void set_ac_heating_threshold(homekit_value_t value);
void set_ac_swing_mode(homekit_value_t value);
void set_dehumidifier_active(homekit_value_t value);
void set_dehumidifier_threshold(homekit_value_t value);

// Indoor humidity pushed in from outside, since the CN105 does not report it.
// The reading is timestamped: a HomePod automation feeding this can fail
// silently for weeks, so anything consuming it must check the age first.
void ac_controller_report_humidity(float percent);
bool ac_controller_humidity_fresh(float& percent_out);
#endif

#ifdef __cplusplus
}
#endif
