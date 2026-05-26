#pragma once

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#ifdef __cplusplus
extern "C" {
#endif

// Main configuration object used by the HomeKit daemon
extern homekit_server_config_t config;

// Accessory Name Characteristics (set at runtime from DEVICE_NAME)
extern homekit_characteristic_t cha_name;
extern homekit_characteristic_t cha_conf_name;

// HeaterCooler Characteristics (Main AC Tile)
extern homekit_characteristic_t cha_ac_active;
extern homekit_characteristic_t cha_ac_current_state;
extern homekit_characteristic_t cha_ac_target_state;
extern homekit_characteristic_t cha_ac_current_temp;
extern homekit_characteristic_t cha_ac_cooling_threshold;
extern homekit_characteristic_t cha_ac_heating_threshold;
extern homekit_characteristic_t cha_ac_target_fan_state;
extern homekit_characteristic_t cha_ac_rotation_speed;
extern homekit_characteristic_t cha_ac_swing_mode;
// Status Fault — set non-zero when the AC reports it can't comply (e.g.
// multi-zone direction conflict, bit 3 of 0x09 status flags).
extern homekit_characteristic_t cha_ac_status_fault;

// Dehumidifier
extern homekit_characteristic_t cha_dehumidifier_active;
extern homekit_characteristic_t cha_dehumidifier_current_state;
extern homekit_characteristic_t cha_dehumidifier_target_state;
// Current Relative Humidity — required by HAP for the
// HumidifierDehumidifier service. The CN105 protocol on this hardware
// does not expose a humidity reading, so this is a fixed 50% stub:
// the characteristic must exist for the service to be valid in iOS
// Home but the value is not meaningful.
extern homekit_characteristic_t cha_dehumidifier_current_humidity;

#ifdef __cplusplus
}
#endif
