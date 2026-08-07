#include "glog.h"
#include <LittleFS.h>
#include <time.h>

extern "C" {
#include <user_interface.h>
}

static uint8_t g_level = APP_LOG_LEVEL;
static bool g_crash_captured = false;

// .noinit: survives exception/wdt resets, garbage after power-on. The magic
// plus the crash-only gate in glog_init() tells the two apart.
struct GlogRing {
  uint32_t magic;
  uint32_t head;
  uint32_t wrapped;
  char buf[GLOG_RING_SIZE];
};
static GlogRing g_ring __attribute__((section(".noinit")));
static const uint32_t GLOG_RING_MAGIC = 0x474C5231; // "GLR1"

void glog_set_level(uint8_t level) {
  if (level < GLOG_L_ERROR) level = GLOG_L_ERROR;
  if (level > 6) level = 6;
  g_level = level;
}
uint8_t glog_level() { return g_level; }
bool glog_trace_on() { return g_level >= GLOG_L_TRACE; }
bool glog_crash_captured() { return g_crash_captured; }

static void ring_put(const char *s, size_t n) {
  for (size_t i = 0; i < n; i++) {
    g_ring.buf[g_ring.head++] = s[i];
    if (g_ring.head >= GLOG_RING_SIZE) {
      g_ring.head = 0;
      g_ring.wrapped = 1;
    }
  }
}

void glog_ring_segments(const char **a, size_t *a_len,
                        const char **b, size_t *b_len) {
  if (g_ring.wrapped) {
    *a = g_ring.buf + g_ring.head; *a_len = GLOG_RING_SIZE - g_ring.head;
    *b = g_ring.buf;               *b_len = g_ring.head;
  } else {
    *a = g_ring.buf;               *a_len = g_ring.head;
    *b = g_ring.buf;               *b_len = 0;
  }
}

void glog_ring_clear() {
  g_ring.head = 0;
  g_ring.wrapped = 0;
}

static void gen_path(char *out, size_t n, int gen) {
  snprintf(out, n, "/log.%d", gen);
}

// One rename per generation, oldest deleted; no log data is ever copied.
static void rotate_generations() {
  char from[16], to[16];
  gen_path(from, sizeof(from), GLOG_GENERATIONS - 1);
  LittleFS.remove(from);
  for (int i = GLOG_GENERATIONS - 2; i >= 0; i--) {
    gen_path(from, sizeof(from), i);
    gen_path(to, sizeof(to), i + 1);
    LittleFS.rename(from, to); // absent generations fail silently; fine
  }
}

static void flash_append(const char *line, size_t n) {
  // A nested call (from inside LittleFS) keeps its line in the ring only.
  static bool in_write = false;
  if (in_write) return;
  in_write = true;
  File f = LittleFS.open("/log.0", "a");
  if (f) {
    if (f.size() > GLOG_GEN_SIZE) {
      f.close();
      rotate_generations();
      f = LittleFS.open("/log.0", "a");
    }
    if (f) {
      f.write((const uint8_t *)line, n);
      f.close();
    }
  }
  in_write = false;
}

void glog_log(uint8_t level, const char *tag, PGM_P fmt, ...) {
  if (level > g_level) return;

  char line[320];
  const size_t cap = sizeof(line) - 2; // room for '\n' + NUL
  unsigned long ms = millis();
  size_t pos = snprintf(line, cap, "[%lu.%03lus", ms / 1000, ms % 1000);
  time_t now = time(nullptr);
  if (now > TIME_SYNCED_EPOCH) {
    struct tm *ti = localtime(&now);
    pos += strftime(line + pos, cap - pos, " %H:%M:%S", ti);
  }
  char lc = (level == GLOG_L_ERROR) ? 'E'
          : (level == GLOG_L_WARN)  ? 'W'
          : (level == GLOG_L_INFO)  ? 'I'
          : 'T';
  pos += snprintf(line + pos, cap - pos, "] %c: [%s] ", lc, tag);
  if (pos > cap) pos = cap;

  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf_P(line + pos, cap - pos, fmt, ap);
  va_end(ap);
  if (n > 0) pos += (size_t)n;
  if (pos > cap) pos = cap;

  if (level == GLOG_L_ERROR || level == GLOG_L_TRACE) {
    n = snprintf(line + pos, cap - pos, " (Heap: %u)", ESP.getFreeHeap());
    if (n > 0) pos += (size_t)n;
    if (pos > cap) pos = cap;
  }
  line[pos++] = '\n';

  ring_put(line, pos);
  if (level <= GLOG_L_INFO) flash_append(line, pos);
}

void glog_init() {
  // Crash capture: on an exception/wdt reset the pre-crash ring is still in
  // DRAM. Persist it before the buffer is reused.
  const rst_info *ri = ESP.getResetInfoPtr();
  bool crash = ri && (ri->reason == REASON_EXCEPTION_RST ||
                      ri->reason == REASON_SOFT_WDT_RST ||
                      ri->reason == REASON_WDT_RST);
  if (crash && g_ring.magic == GLOG_RING_MAGIC && g_ring.head < GLOG_RING_SIZE) {
    File f = LittleFS.open(GLOG_CRASH_PATH, "w");
    if (f) {
      char hdr[96];
      int n = snprintf(hdr, sizeof(hdr), "--- ring at crash (%s) ---\n",
                       ESP.getResetReason().c_str());
      if (n > (int)sizeof(hdr) - 1) n = sizeof(hdr) - 1;
      if (n > 0) f.write((const uint8_t *)hdr, n);
      const char *a, *b;
      size_t al, bl;
      glog_ring_segments(&a, &al, &b, &bl);
      if (al) f.write((const uint8_t *)a, al);
      if (bl) f.write((const uint8_t *)b, bl);
      f.close();
      g_crash_captured = true;
    }
  }
  g_ring.magic = GLOG_RING_MAGIC;
  glog_ring_clear();

  // One-time cleanup of the pre-1.47 log files; they no longer rotate.
  LittleFS.remove("/system.log");
  LittleFS.remove("/system.log.old");

  flash_append("\n--- [Boot] ---\n", 16);
}
