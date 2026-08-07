#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <string.h>
#include <stdbool.h>
#include "homekit_ac.h"
#include "ac_controller.h"
#include "config.h"

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
// Matches TEMP_MAP, the CN105 setpoint vocabulary. Smart Auto aims pull_c()
// inside the range, so a wider slider would target temperatures the unit cannot
// be told, and the call would never reach its release point.
homekit_characteristic_t cha_ac_cooling_threshold = HOMEKIT_CHARACTERISTIC_(COOLING_THRESHOLD_TEMPERATURE, 24.0, .min_value = (float[]) {16}, .max_value = (float[]) {31});
homekit_characteristic_t cha_ac_heating_threshold = HOMEKIT_CHARACTERISTIC_(HEATING_THRESHOLD_TEMPERATURE, 18.0, .min_value = (float[]) {16}, .max_value = (float[]) {31});
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
// CN105 reports no humidity; HAP requires this characteristic. A flat 50.0
// means "never reported" — /control?humidity= pushes overwrite the seed.
homekit_characteristic_t cha_dehumidifier_current_humidity = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 50.0);
// Target RH. Nothing consumes it yet: DRY still engages on the idle band
// regardless of this value or of any reported reading.
homekit_characteristic_t cha_dehumidifier_threshold = HOMEKIT_CHARACTERISTIC_(RELATIVE_HUMIDITY_DEHUMIDIFIER_THRESHOLD, HUMIDITY_THRESHOLD_DEFAULT);

static homekit_characteristic_t cha_dehumidifier_name = HOMEKIT_CHARACTERISTIC_(NAME, "AC Dehumidifier");
static homekit_characteristic_t cha_dehumidifier_conf_name = HOMEKIT_CHARACTERISTIC_(CONFIGURED_NAME, "Dehumidifier");

// Named rather than an inline literal so homekit_ac_apply_humidity_gate() can
// shorten it before the server reads the database.
static homekit_characteristic_t *dehumidifier_chars[] = {
    &cha_dehumidifier_name,
    &cha_dehumidifier_conf_name,
    &cha_dehumidifier_active,
    &cha_dehumidifier_current_state,
    &cha_dehumidifier_target_state,
    &cha_dehumidifier_current_humidity,
    &cha_dehumidifier_threshold,   // must stay last: the gate drops it by moving NULL up
    NULL
};

bool device_has_humidity_feed(void) {
    static const char *units[] = HUMIDITY_UNITS;
    for (size_t i = 0; i < sizeof(units) / sizeof(units[0]); i++)
        if (strcmp(DEVICE_NAME, units[i]) == 0) return true;
    return false;
}

void homekit_ac_apply_humidity_gate(void) {
    if (device_has_humidity_feed()) return;
    for (size_t i = 0; dehumidifier_chars[i]; i++) {
        if (dehumidifier_chars[i] == &cha_dehumidifier_threshold) {
            dehumidifier_chars[i] = NULL;
            return;
        }
    }
}

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
        HOMEKIT_SERVICE(HUMIDIFIER_DEHUMIDIFIER, .characteristics = dehumidifier_chars),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-22-333",
    // Bump on every accessory-database change, including declared min/max
    // ranges — iOS caches those as part of the schema and re-reads
    // /accessories only when the number rises. Pairings survive a bump.
    // Only ever increase it: paired controllers have cached every value
    // shipped OR canaried up to the current one.
    .config_number = 10
};
