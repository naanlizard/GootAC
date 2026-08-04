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
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python3 manage.py list          # discover devices on network + USB
python3 manage.py update        # OTA update with interactive device selection
python3 manage.py install       # factory flash a new device via USB
python3 manage.py monitor       # serial monitor
```

## Configuration

`src/config.h` is gitignored and must be created from `src/config.h.example`. It defines:
- `WIFI_SSID` / `WIFI_PASS` - network credentials
- `DEVICE_NAME` - unit name; hostname and accessory name derive from it (`manage.py update` rewrites it per flash)
- `FW_VERSION` - semver string used for OTA version comparison
- `FORCE_HK_START` - if true, starts HomeKit server without waiting for AC handshake
- `HUMIDITY_UNITS` - device names that publish the HomeKit target-humidity slider
- `MIN/MAX_VALID_ROOM_TEMP_C` (optional) - sensor plausibility bounds

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

- **UART0 is reserved for AC communication.** No serial logging; everything goes through `LittleFSLogger` to flash, readable at `/log`, `/log.old` and `/metrics`.
- **Smart Auto is firmware, not the AC's native auto.** A call starts when the room leaves the heating/cooling range and ends when it reaches that call's target, `auto_pull_c()` inside the range. That target is both the commanded setpoint and the release point; separating them costs the fan ramp its low rungs. See `ac_decision.cpp` for the pull and ramp constants.
- **Dehumidify is a toggle over the idle band, not a mode.** Whenever Smart Auto is not running a HEAT or COOL call, the idle band is DRY when the toggle is armed and FAN when it is not; with the accessory off, an armed toggle still runs standalone DRY. The two HomeKit services have no interlock, because `ac_decide()` resolves the single physical mode register.
- **The idle band delegates the fan to the unit in DRY** (`fan_idx` 0) and commands QUIET in FAN. The `FAN_IDX_MIN` branch is gated on mode 3, so it does not apply to DRY.
- **The Auto range is held at least `THRESHOLD_MIN_GAP_C` (3.0 C) wide on every path into `ac_decide()`**, which has no defences of its own: an inverted pair trips both entry tests and flips HEAT/COOL every tick. `normalize_thresholds()` is the only guard, and runs from both threshold setters, from restored flash state, and from the legacy v1.12 migration. Below 3.0 the pull floor and the half-range cap coincide and both targets land on the range midpoint.
- **The threshold sliders span 16-31 C, matching `TEMP_MAP` (`HeatPump.h`), the CN105 setpoint vocabulary.** Smart Auto aims `auto_pull_c()` inside the range, so a wider slider produces targets the unit cannot be told: `setTemperature()` clamps, the room stops at the clamped value, and the call never reaches its release point. At 16-31 every target lands in 17.0-30.0. Changing the range needs a `config_number` bump in `homekit_ac.c`.
- **The fan ramp holds its rung unless demand clears the boundary by a whole sensor step.** The room moves in 0.5 C jumps, up-moves are immediate and down-moves are damped, so a room parked on a rung boundary would raise the fan every tick and cancel the pending step-down on the next, emitting a CN105 packet each time.
- **A guard correction is re-notified from `ac_controller_loop()`, not the setter.** The server re-notifies the client's own value after the setter returns, and the per-client queue coalesces oldest-wins, so an inline correction loses.
- **Target state persistence**: `/target_state.bin` on LittleFS with a checksum.
- **External change detection**: the library flags `_externalUpdateOccurred` when a received settings packet differs from `wantedSettings`, and the controller syncs power and mode, not thresholds, back into `currentState`. Fan is masked from that comparison, and so is temperature in FAN and DRY, since neither is commanded with a setpoint.

### Source Files

- `main.cpp` — Boot sequence (WiFi, mDNS, OTA, web server), main loop and watchdogs
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

### Logging Format Specifiers (GLOG_* macros)

The `GLOG_INFO/TRACE/WARN/ERROR` macros delegate to ArduinoLog (thijse/Arduino-Log), which has its **own non-printf format parser**. The single character after `%` is the entire specifier — width and length modifiers are **not** parsed and silently consume the wrong things:

- ✅ Works: `%s %S %d %i %u %l %F %D %x %X %p %b %B %c %C %t %T`
- ❌ Silently broken: `%02X` `%5d` `%lu` `%lus` — the parser sees `%` then the first char as the specifier, then the rest as literal text. The format arg is left unconsumed, so any subsequent format args shift down by one. This is especially bad for `GLOG_TRACE` and `GLOG_ERROR` which append `(Heap: %u)` — the heap counter reads the wrong arg.

When you need a hex byte or padded int, snprintf into a small char buffer first and pass it via `%s` — see the existing `subTypeHex` pattern in `HeatPump.cpp`.

### Maintenance Behavior

WiFi loss beyond 2 minutes triggers a reboot. There is no periodic maintenance reboot; units run for weeks.
