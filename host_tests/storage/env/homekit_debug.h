#pragma once
#include <stdio.h>

extern int hk_log_enabled;

#define HK_LOG(pfx, fmt, ...) \
    do { if (hk_log_enabled) fprintf(stderr, pfx fmt "\n", ##__VA_ARGS__); } while (0)

#define ERROR(fmt, ...)   HK_LOG("[E] ", fmt, ##__VA_ARGS__)
#define INFO(fmt, ...)    HK_LOG("[I] ", fmt, ##__VA_ARGS__)
#define DEBUG(fmt, ...)   HK_LOG("[D] ", fmt, ##__VA_ARGS__)
#define DEBUG_HEAP()      do {} while (0)
