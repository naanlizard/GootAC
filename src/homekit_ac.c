#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include "homekit_ac.h"
#include "ac_controller.h"
#include "config.h"

#ifndef MIN_VALID_ROOM_TEMP_C
#define MIN_VALID_ROOM_TEMP_C 10.0f
#endif
#ifndef MAX_VALID_ROOM_TEMP_C
#define MAX_VALID_ROOM_TEMP_C 45.0f
#endif

extern char hostName[32];
extern char accessoryName[32];
 
// Identify callback (required by HAP spec)
void my_accessory_identify(homekit_value_t _value) {
    ac_controller_identify();
}

homekit_characteristic_t cha_name = HOMEKIT_CHARACTERISTIC_(NAME, "GootAC");
homekit_characteristic_t cha_conf_name = HOMEKIT_CHARACTERISTIC_(CONFIGURED_NAME, "GootAC");

// HeaterCooler Service
homekit_characteristic_t cha_ac_active = HOMEKIT_CHARACTERISTIC_(ACTIVE, 0);
homekit_characteristic_t cha_ac_current_state = HOMEKIT_CHARACTERISTIC_(CURRENT_HEATER_COOLER_STATE, 0);
homekit_characteristic_t cha_ac_target_state = HOMEKIT_CHARACTERISTIC_(TARGET_HEATER_COOLER_STATE, 0);
homekit_characteristic_t cha_ac_current_temp = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 22.0, .min_value = (float[]) {MIN_VALID_ROOM_TEMP_C}, .max_value = (float[]) {MAX_VALID_ROOM_TEMP_C});
// Wider than both the HAP defaults (cooling 10-35, heating 0-25) and the 16-31C
// the CN105 carries; the ends set an idle band, they are not commandable.
homekit_characteristic_t cha_ac_cooling_threshold = HOMEKIT_CHARACTERISTIC_(COOLING_THRESHOLD_TEMPERATURE, 24.0, .min_value = (float[]) {5}, .max_value = (float[]) {40});
homekit_characteristic_t cha_ac_heating_threshold = HOMEKIT_CHARACTERISTIC_(HEATING_THRESHOLD_TEMPERATURE, 18.0, .min_value = (float[]) {5}, .max_value = (float[]) {40});
homekit_characteristic_t cha_ac_swing_mode = HOMEKIT_CHARACTERISTIC_(SWING_MODE, 0);
homekit_characteristic_t cha_ac_temp_display_units = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS, 0);
// 0 = no fault, 1 = general fault. We set 1 when the AC reports it
// cannot comply with the requested mode (multi-zone direction conflict
// = bit 3 of 0x09 status flags). iOS Home shows a warning badge.
homekit_characteristic_t cha_ac_status_fault = HOMEKIT_CHARACTERISTIC_(STATUS_FAULT, 0);

// Dehumidifier Service
homekit_characteristic_t cha_dehumidifier_active = HOMEKIT_CHARACTERISTIC_(ACTIVE, 0);
homekit_characteristic_t cha_dehumidifier_current_state = HOMEKIT_CHARACTERISTIC_(CURRENT_HUMIDIFIER_DEHUMIDIFIER_STATE, 0);
homekit_characteristic_t cha_dehumidifier_target_state = HOMEKIT_CHARACTERISTIC_(TARGET_HUMIDIFIER_DEHUMIDIFIER_STATE, 2, .valid_values = { .count = 1, .values = (uint8_t[]) {2} });
// HAP-required characteristic for the HUMIDIFIER_DEHUMIDIFIER service.
// CN105 does not expose an indoor humidity reading on this hardware
// (confirmed by exhaustive sweep + Mitsubishi's licensed Modbus
// gateway publishing no humidity register). Fixed 50% stub satisfies
// the spec so iOS Home stops reporting "No Response" for the tile.
homekit_characteristic_t cha_dehumidifier_current_humidity = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 50.0);

// Build the Accessory Database
homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id = 1, .category = homekit_accessory_category_air_conditioner, .services = (homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics = (homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, hostName),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Mitsubishi"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, hostName),
            HOMEKIT_CHARACTERISTIC(MODEL, "ESP8266-GootAC"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, FW_VERSION),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, my_accessory_identify),
            NULL
        }),
        HOMEKIT_SERVICE(HEATER_COOLER, .primary = true, .characteristics = (homekit_characteristic_t*[]) {
            &cha_name,
            &cha_conf_name,
            &cha_ac_active,
            &cha_ac_current_state,
            &cha_ac_target_state,
            &cha_ac_current_temp,
            &cha_ac_cooling_threshold,
            &cha_ac_heating_threshold,
            &cha_ac_temp_display_units,
            &cha_ac_swing_mode,
            &cha_ac_status_fault,
            NULL
        }),
        HOMEKIT_SERVICE(HUMIDIFIER_DEHUMIDIFIER, .characteristics = (homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "AC Dehumidifier"),
            HOMEKIT_CHARACTERISTIC(CONFIGURED_NAME, "Dehumidifier"),
            &cha_dehumidifier_active,
            &cha_dehumidifier_current_state,
            &cha_dehumidifier_target_state,
            &cha_dehumidifier_current_humidity,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-22-333",
    // Bump every time the accessory database changes (services/characteristics
    // added or removed). iOS Home compares this number against what it has
    // cached for the paired accessory; if higher, it re-reads /accessories
    // and picks up the new schema. Pairings survive the bump.
    //
    // History:
    //   1 = up to v1.10 (HEATER_COOLER + HUMIDIFIER_DEHUMIDIFIER bare set)
    //   2 = v1.11 — added cha_ac_status_fault to HEATER_COOLER and
    //               cha_dehumidifier_current_humidity to HUMIDIFIER_DEHUMIDIFIER
    //   3 = v1.13 — removed cha_ac_rotation_speed + cha_ac_target_fan_state
    //               from HEATER_COOLER (fan permanently delegated to AC AUTO)
    //   4,5 = v1.18 canary-only fan-control experiment (rotation_speed, then
    //         + target_fan_state); flashed only to Simples, retired same night
    //         after iOS rendered the controls poorly
    //   6 = v1.18 — fan characteristics removed again; must exceed the 4/5
    //       schemas cached by the canary's paired controller
    //   7 = v1.33: threshold min/max widened from 16-31 to 5-40. Nothing added
    //       or removed, but iOS caches the declared range as part of the schema
    .config_number = 7
};
