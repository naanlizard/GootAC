#pragma once
#include <stddef.h>

// Off by default: costs HK_RING_SIZE of RAM and compiles the library's DEBUG
// strings into flash. Enable with -D GOOTAC_HK_LOG to serve /hklog.
//
// The HomeKit library writes its own logs to Serial, which is unreachable on an
// installed unit. GOOTAC_HK_LOG turns on the library's HOMEKIT_USE_RATGDO_LOG
// hook so it calls logToBuffer_P() instead and the last few KB stay in RAM.
// Deliberately not LittleFS: pair-verify is timing sensitive and a synchronous
// flash write in that path can change the behaviour being diagnosed.
#ifdef GOOTAC_HK_LOG
// The ring is returned as two segments (oldest first); either may be empty.
void hk_log_segments(const char **a, size_t *a_len, const char **b, size_t *b_len);
void hk_log_clear();
#endif
