#pragma once

#ifdef USE_FAKE_HEATPUMP

// Inject helpers exposed for HTTP /fake/* endpoints in main.cpp.
// Implemented in HeatPumpFake.cpp. The class itself is the same HeatPump
// class declared in HeatPump.h — only its .cpp implementation is swapped.

void gootac_fake_inject_external(const char *power, const char *mode, float temperature);
void gootac_fake_force_room_temperature(float t);

#endif
