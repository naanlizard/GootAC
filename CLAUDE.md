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
- **Every target mode runs firmware call cycles; the unit's native auto is only the room-temp-unknown fallback.** A Smart Auto call starts when the room leaves the heating/cooling range; explicit HEAT/COOL run the same cycle one-sided off their own threshold. A call ends at its target, a static `AUTO_PULL_C` (0.5 C) past the threshold; that target is both the commanded setpoint and the release point, and the idle staircase picks up at the same depth on release.
- **Dehumidify is a toggle over the idle band, not a mode.** Whenever no HEAT or COOL call is running (any target mode), the idle band is DRY when the toggle is armed and FAN when it is not; with the accessory off, an armed toggle still runs standalone DRY. The two HomeKit services have no interlock, because `ac_decide()` resolves the single physical mode register.
- **The idle band delegates the fan to the unit in DRY** (`fan_idx` 0); in FAN it runs the same staircase on depth past its anchor target (the nearer one in Smart Auto, the mode's own side in explicit HEAT/COOL), so circulation winds down after a release and back up as the room re-approaches a trigger; a room between a target and its threshold idles at 100%.
- **The Auto range is held at least `THRESHOLD_MIN_GAP_C` (3.0 C) wide on every path into `ac_decide()`**, which has no defences of its own: an inverted pair trips both entry tests and flips HEAT/COOL every tick. `normalize_thresholds()` is the only guard, and runs from both threshold setters, from restored flash state, and from the legacy v1.12 migration. At width 1.0 the half-range cap collapses both targets onto the midpoint.
- **The threshold sliders span 16-31 C, matching `TEMP_MAP` (`HeatPump.h`), the CN105 setpoint vocabulary.** A target the unit cannot be told is a release point the room never reaches, so `auto_cool/heat_target()` clamp into 16-31 (`TEMP_CMD_MIN/MAX_C`). Changing the slider range needs a `config_number` bump in `homekit_ac.c`.
- **The fan is a staircase: 100% until the room reaches the commanded target, then one rung down per 0.5 C of depth past it (75/50/25/QUIET).** Up-moves are immediate; down-moves wait out 60 s of sustained lower demand. A reading dithering across a boundary faster than the dwell pins at the higher level; slower dither is rate-bounded to one change per dwell, not pinned.
- **A guard correction is re-notified from `ac_controller_loop()`, not the setter.** The server re-notifies the client's own value after the setter returns, and the per-client queue coalesces oldest-wins, so an inline correction loses.
- **Target state persistence**: `/target_state.bin` on LittleFS with a checksum.
- **External change detection**: the library flags `_externalUpdateOccurred` when a received settings packet differs from `wantedSettings`, and the controller syncs power always but mode only when the unit reports HEAT/COOL/AUTO in a fixed target mode: FAN and DRY are indistinguishable from the firmware's own idle band, and Smart Auto never syncs mode. Thresholds never sync. Fan is masked from that comparison, and so is temperature in FAN and DRY, since neither is commanded with a setpoint.

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
