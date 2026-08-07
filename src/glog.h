#pragma once
#include <Arduino.h>

// Two-tier logger. Durable tier: INFO and up append to /log.0 on LittleFS,
// rotated by rename into /log.1..N (oldest dropped) — no data is ever copied.
// Verbose tier: every admitted line also enters a RAM ring served at /trace;
// TRACE lives only there, so full verbosity costs no flash wear. The ring sits
// in .noinit DRAM, which survives exception/wdt resets: glog_init() persists
// the pre-crash ring to /crash.log before reusing it.

// ArduinoLog-compatible numbering so APP_LOG_LEVEL keeps its meaning
// (2 error, 3 warning, 4 info, 5 trace).
#define GLOG_L_ERROR 2
#define GLOG_L_WARN  3
#define GLOG_L_INFO  4
#define GLOG_L_TRACE 5

#ifndef APP_LOG_LEVEL
#define APP_LOG_LEVEL 6 // verbose when the build defines nothing
#endif

#define GLOG_GEN_SIZE    32768
#define GLOG_GENERATIONS 16
#define GLOG_RING_SIZE   4096
#define GLOG_CRASH_PATH  "/crash.log"

// time() below this (2020-01-01) means SNTP has not synced yet.
#define TIME_SYNCED_EPOCH 1577836800

void glog_init(); // call once after LittleFS.begin(); runs the crash capture
void glog_set_level(uint8_t level);
uint8_t glog_level();
bool glog_trace_on();
bool glog_crash_captured(); // true if this boot wrote GLOG_CRASH_PATH

// fmt lives in PROGMEM (PSTR); string arguments are normal RAM pointers.
void glog_log(uint8_t level, const char *tag, PGM_P fmt, ...);

// The ring as two segments, oldest first; either may be empty.
void glog_ring_segments(const char **a, size_t *a_len,
                        const char **b, size_t *b_len);
void glog_ring_clear();

// Standard printf formatting (vsnprintf_P). ERROR and TRACE lines get a
// " (Heap: N)" suffix appended by glog_log.
#define GLOG_ERROR(id, fmt, ...) glog_log(GLOG_L_ERROR, id, PSTR(fmt), ##__VA_ARGS__)
#define GLOG_WARN(id, fmt, ...)  glog_log(GLOG_L_WARN, id, PSTR(fmt), ##__VA_ARGS__)
#define GLOG_INFO(id, fmt, ...)  glog_log(GLOG_L_INFO, id, PSTR(fmt), ##__VA_ARGS__)
#define GLOG_BOOT(fmt, ...)      glog_log(GLOG_L_INFO, "BOOT", PSTR(fmt), ##__VA_ARGS__)
// Checks the runtime level BEFORE evaluating arguments, so disabled TRACE
// costs one branch. Guard standalone prep blocks with glog_trace_on().
#define GLOG_TRACE(id, fmt, ...)                                          \
  do {                                                                    \
    if (glog_trace_on()) glog_log(GLOG_L_TRACE, id, PSTR(fmt), ##__VA_ARGS__); \
  } while (0)
