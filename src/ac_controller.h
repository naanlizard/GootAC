#pragma once
#include <Arduino.h>

struct TargetState {
    uint8_t active;              // 0: Off, 1: On
    uint8_t target_mode;         // 0: COOL, 1: HEAT, 2: AUTO
    float cooling_threshold;
    float heating_threshold;
    uint8_t fan_mode;            // 0: MANUAL, 1: AUTO
    float fan_speed;             // 0-100%
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
void ac_controller_report_status();
void ac_controller_identify();

#ifdef __cplusplus
// Status for WebServer
String ac_controller_get_json_status();
#endif

#ifdef __cplusplus
}
#endif
