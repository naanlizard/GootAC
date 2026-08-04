#pragma once

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <stdbool.h>

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
extern homekit_characteristic_t cha_ac_swing_mode;
// Status Fault — set non-zero when the AC reports it can't comply (e.g.
// multi-zone direction conflict, bit 3 of 0x09 status flags).
extern homekit_characteristic_t cha_ac_status_fault;

// Dehumidifier
extern homekit_characteristic_t cha_dehumidifier_active;
extern homekit_characteristic_t cha_dehumidifier_current_state;
extern homekit_characteristic_t cha_dehumidifier_target_state;
// The CN105 reports no humidity, so both of these are fed from outside:
// current from /control?humidity=, target from HomeKit or /control.
extern homekit_characteristic_t cha_dehumidifier_current_humidity;
extern homekit_characteristic_t cha_dehumidifier_threshold;

// True when DEVICE_NAME is in HUMIDITY_UNITS. The gate must run before
// arduino_homekit_setup(); after that the server has read the database.
bool device_has_humidity_feed(void);
void homekit_ac_apply_humidity_gate(void);

#ifdef __cplusplus
}
#endif
