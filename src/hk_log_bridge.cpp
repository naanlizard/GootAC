#include <Arduino.h>
#include <stdarg.h>
#include "hk_log_bridge.h"

#ifdef GOOTAC_HK_LOG

// Mirrors the enum the HomeKit library declares under HOMEKIT_USE_RATGDO_LOG.
// Guarded with the library's own macro so the two never collide if both land in
// one translation unit.
#ifndef ESP_LOG_LEVEL_T
#define ESP_LOG_LEVEL_T
typedef enum {
    ESP_LOG_NONE,
    ESP_LOG_ERROR,
    ESP_LOG_WARN,
    ESP_LOG_INFO,
    ESP_LOG_DEBUG,
    ESP_LOG_VERBOSE
} esp_log_level_t;
#endif

static const size_t HK_RING_SIZE = 4096;
static char   hk_ring[HK_RING_SIZE];
static size_t hk_head = 0;
static bool   hk_wrapped = false;

extern "C" {

// The library compares `level <= logLevel`, so DEBUG admits ERROR/WARN/INFO too.
esp_log_level_t logLevel = ESP_LOG_DEBUG;

void logToBuffer_P(const char *fmt, ...) {
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf_P(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if ((size_t) n > sizeof(buf) - 1) n = sizeof(buf) - 1;

    for (int i = 0; i < n; i++) {
        hk_ring[hk_head++] = buf[i];
        if (hk_head >= HK_RING_SIZE) { hk_head = 0; hk_wrapped = true; }
    }
}

} // extern "C"

void hk_log_segments(const char **a, size_t *a_len, const char **b, size_t *b_len) {
    if (hk_wrapped) {
        *a = hk_ring + hk_head; *a_len = HK_RING_SIZE - hk_head;
        *b = hk_ring;           *b_len = hk_head;
    } else {
        *a = hk_ring;           *a_len = hk_head;
        *b = hk_ring;           *b_len = 0;
    }
}

void hk_log_clear() {
    hk_head = 0;
    hk_wrapped = false;
}

#endif // GOOTAC_HK_LOG
