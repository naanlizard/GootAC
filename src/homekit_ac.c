#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include "homekit_ac.h"
#include "ac_controller.h"
#include "config.h"

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
homekit_characteristic_t cha_ac_current_temp = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 22.0, .min_value = (float[]) {0});
homekit_characteristic_t cha_ac_cooling_threshold = HOMEKIT_CHARACTERISTIC_(COOLING_THRESHOLD_TEMPERATURE, 24.0, .min_value = (float[]) {16}, .max_value = (float[]) {31});
homekit_characteristic_t cha_ac_heating_threshold = HOMEKIT_CHARACTERISTIC_(HEATING_THRESHOLD_TEMPERATURE, 18.0, .min_value = (float[]) {16}, .max_value = (float[]) {31});
homekit_characteristic_t cha_ac_target_fan_state = HOMEKIT_CHARACTERISTIC_(TARGET_FAN_STATE, 0);
homekit_characteristic_t cha_ac_rotation_speed = HOMEKIT_CHARACTERISTIC_(ROTATION_SPEED, 0);
homekit_characteristic_t cha_ac_swing_mode = HOMEKIT_CHARACTERISTIC_(SWING_MODE, 0);
homekit_characteristic_t cha_ac_temp_display_units = HOMEKIT_CHARACTERISTIC_(TEMPERATURE_DISPLAY_UNITS, 0);

// Dehumidifier Service
homekit_characteristic_t cha_dehumidifier_active = HOMEKIT_CHARACTERISTIC_(ACTIVE, 0);
homekit_characteristic_t cha_dehumidifier_current_state = HOMEKIT_CHARACTERISTIC_(CURRENT_HUMIDIFIER_DEHUMIDIFIER_STATE, 0);
homekit_characteristic_t cha_dehumidifier_target_state = HOMEKIT_CHARACTERISTIC_(TARGET_HUMIDIFIER_DEHUMIDIFIER_STATE, 2, .valid_values = { .count = 1, .values = (uint8_t[]) {2} });

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
            &cha_ac_rotation_speed,
            &cha_ac_target_fan_state,
            &cha_ac_swing_mode,
            NULL
        }),
        HOMEKIT_SERVICE(HUMIDIFIER_DEHUMIDIFIER, .characteristics = (homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "AC Dehumidifier"),
            HOMEKIT_CHARACTERISTIC(CONFIGURED_NAME, "Dehumidifier"),
            &cha_dehumidifier_active,
            &cha_dehumidifier_current_state,
            &cha_dehumidifier_target_state,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-22-333"
};
