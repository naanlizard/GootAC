#pragma once
#include <Arduino.h>

struct TargetState {
    uint8_t active;              // 0: Off, 1: On
    uint8_t target_mode;         // HomeKit TargetHeaterCoolerState: 0: AUTO, 1: HEAT, 2: COOL
    float cooling_threshold;
    float heating_threshold;
    uint8_t swing_mode;          // 0: OFF, 1: SWING
    uint8_t dehumidifier;        // 0: OFF, 1: ON
    uint32_t checksum;           // Integrity check
};

// --- Logging Macros ---
#define GLOG_INFO(id, fmt, ...)  Log.infoln(F("[%s] " fmt), id, ##__VA_ARGS__)
#define GLOG_TRACE(id, fmt, ...) Log.traceln(F("[%s] " fmt " (Heap: %u)"), id, ##__VA_ARGS__, ESP.getFreeHeap())
#define GLOG_WARN(id, fmt, ...)  Log.warningln(F("[%s] " fmt), id, ##__VA_ARGS__)
#define GLOG_ERROR(id, fmt, ...) Log.errorln(F("[%s] " fmt " (Heap: %u)"), id, ##__VA_ARGS__, ESP.getFreeHeap())
#define GLOG_BOOT(fmt, ...)      Log.noticeln(F("[BOOT] " fmt), ##__VA_ARGS__)

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

// Indoor humidity pushed in from outside, since the CN105 does not report it.
// The reading is timestamped: a HomePod automation feeding this can fail
// silently for weeks, so anything consuming it must check the age first.
void ac_controller_report_humidity(float percent);
bool ac_controller_humidity_fresh(float& percent_out);
#endif

#ifdef __cplusplus
}
#endif
