#pragma once

#include <homekit/homekit.h>
#include <homekit/characteristics.h>

#ifdef __cplusplus
extern "C" {
#endif

// Main configuration object used by the HomeKit daemon
extern homekit_server_config_t config;

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

// Dehumidifier (Stubbed)
extern homekit_characteristic_t cha_dehumidifier_active;
extern homekit_characteristic_t cha_dehumidifier_current_state;
extern homekit_characteristic_t cha_dehumidifier_target_state;

#ifdef __cplusplus
}
#endif
