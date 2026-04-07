# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

GootAC is ESP8266 firmware (targeting Wemos D1 Mini) that provides native Apple HomeKit control for Mitsubishi air conditioners via the CN105 serial port. It exposes a HeaterCooler service (heat/cool/auto) and a Dehumidifier service to the Apple Home app.

## Build & Deploy

This is a PlatformIO project. All build/upload commands use `pio`:

```bash
# Build firmware
pio run

# Upload via USB serial (initial flash)
pio run -t upload

# Upload via OTA (over-the-air to a running device)
pio run -t upload --upload-port <DEVICE_IP>

# Serial monitor (115200 baud)
pio run -t monitor

# Erase flash (factory reset - wipes HomeKit pairings)
pio run -t erase
```

The management utility `manage.py` wraps these commands with mDNS device discovery:
```bash
pip install -r requirements.txt
python manage.py list          # discover devices on network + USB
python manage.py update        # OTA update with interactive device selection
python manage.py install       # factory flash a new device via USB
python manage.py monitor       # serial monitor
```

## Configuration

`src/config.h` is gitignored and must be created from `src/config.h.example`. It defines:
- `WIFI_SSID` / `WIFI_PASS` - network credentials
- `HOST_NAME` - mDNS hostname (e.g., `gootac-bedroom`)
- `ACCESSORY_NAME` - display name in Apple Home app
- `FW_VERSION` - semver string used for OTA version comparison
- `FORCE_HK_START` - if true, starts HomeKit server without waiting for AC handshake

## Architecture

### Data Flow: HomeKit <-> AC Unit

```
Apple Home App
    ↕ (HAP protocol)
homekit_ac.c          — HAP accessory database: defines services, characteristics, and pairing config
    ↕ (setter callbacks)
ac_controller.cpp     — Bridge logic: translates between HomeKit values and HeatPump library calls
    ↕ (HeatPump API)
HeatPump.cpp/.h       — Serial protocol driver for Mitsubishi CN105 (vendored from SwiCago/HeatPump)
    ↕ (UART0 @ 2400 baud)
Mitsubishi indoor unit
```

### Key Design Decisions

- **UART0 is reserved for AC communication.** Serial logging is not available; all logging goes through `LittleFSLogger` to flash storage, viewable via HTTP (`/log`, `/log.old`, `/status`).
- **Smart Auto mode** is implemented in firmware, not via the AC's native auto. When in Auto mode, the controller compares room temperature against heating/cooling thresholds and switches between HEAT and COOL modes (or stays OFF in the deadband).
- **HeaterCooler and Dehumidifier are mutually exclusive.** The Mitsubishi unit only operates in one mode at a time (HEAT, COOL, DRY, FAN, AUTO). HomeKit exposes HeaterCooler (heat/cool/auto) and Dehumidifier (dry) as separate services, but they map to the same physical mode register. When a user activates one service, `ac_controller.cpp` auto-deactivates the other and sends a HomeKit notification so the Home app UI updates immediately. Without this interlock, both services could try to set conflicting modes on the same `hp->setModeIndex()` call. The dehumidifier setter disables HeaterCooler active, and vice versa — see `set_ac_active()` and `set_dehumidifier_active()`.
- **Target state persistence** uses a binary file (`/target_state.bin`) on LittleFS with a simple checksum, so settings survive reboots.
- **External change detection** handles cases where the AC state changes outside of HomeKit — via IR remote, physical buttons, or timers on the unit itself. The vendored HeatPump library tracks a `wantedSettings` (what we last told the AC to do) vs `currentSettings` (what the AC reports back). When a settings packet arrives from the AC and `receivedSettings != wantedSettings`, the library flags `_externalUpdateOccurred = true`. The controller calls `hp->enableExternalUpdate()` at init to opt into this tracking, then checks `hp->wasExternalUpdate()` inside `ac_controller_sync_from_ac()`. When an external change is detected, the controller syncs power and mode back into `currentState` and pushes HomeKit characteristic notifications so the Home app reflects what the IR remote changed. Temperature thresholds are not synced back from external changes — only power and mode.

### Source Files

- `main.cpp` — Boot sequence (WiFi, mDNS, OTA, web server), main loop with watchdogs and maintenance reboots
- `ac_controller.cpp/.h` — All HomeKit↔AC translation logic, state persistence, logging macros (`GLOG_*`, `HKLOG_*`)
- `homekit_ac.c/.h` — HomeKit accessory definition (services, characteristics, pairing password)
- `fs_logger.h` — `LittleFSLogger`: Print-compatible logger that writes to flash with rotation (16KB active, 256KB archive, 7-day retention)
- `HeatPump.cpp/.h` — Vendored from [SwiCago/HeatPump](https://github.com/SwiCago/HeatPump) and modified locally. Key changes from upstream:
  - **Index-based setters** (`setModeIndex`, `setFanSpeedIndex`, `setVaneIndex`, `setWideVaneIndex`) added so `ac_controller.cpp` can set values by numeric index into the public `MODE_MAP`/`FAN_MAP`/`VANE_MAP` arrays, avoiding string comparisons and matching HomeKit's numeric characteristic model.
  - **`getWantedSettings()`** exposes the library's internal `wantedSettings` so the controller can compare desired vs actual state before sending redundant packets to the AC.
  - **`sync(byte packetType)`** added for on-demand info requests (e.g., force a settings re-read after an update) rather than waiting for the next polling cycle.
  - **`wasExternalUpdate()`** / `_externalUpdateOccurred` flag added to detect when a settings change came from the IR remote or unit itself rather than from our commands (see external change detection above).
  - **`GLOG_*` logging** integrated throughout — upstream uses `Serial.println` which conflicts with UART0 being reserved for the CN105 protocol. All debug output now routes through the flash-based `LittleFSLogger`.
  - **Public constant maps** (`MODE_MAP`, `FAN_MAP`, `VANE_MAP`, etc.) moved from private to public so `ac_controller.cpp` can reference them for command logging and comparisons.

### Build Flags (platformio.ini)

- `HOMEKIT_LOG_LEVEL=2` — Controls verbosity of the Arduino-HomeKit library
- `APP_LOG_LEVEL=4` — Controls app-level log verbosity (1=Fatal through 6=Verbose)
- `ARDUINO_HOMEKIT_LOWROM` / `ARDUINO_HOMEKIT_SKIP_ED25519_VERIFY` — Memory optimizations required for ESP8266
- CPU runs at 160MHz for HomeKit crypto performance

### Maintenance Behavior

The firmware auto-reboots after 24h (if AC idle) or 48h (unconditionally) to mitigate memory fragmentation and HomeKit session exhaustion. WiFi loss > 2 minutes also triggers a reboot.
