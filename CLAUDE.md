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
- **Smart Auto mode** is implemented in firmware, not via the AC's native auto. A call starts when the room leaves the heating/cooling range and ends when it reaches that call's target, which sits `auto_pull_c()` inside the range: a quarter of the range width, clamped to 1.0-2.0 C, snapped to the sensor's 0.5 C grid, and never more than half the range so the two targets cannot cross. Over the 65 ranges the setters permit (3.0 to 35.0 C) that yields only three pulls: 1.0 for widths up to 4.5, 1.5 up to 6.5, and 2.0 for the remaining 57. The quarter-of-the-range formula governs a narrow band of widths and the 2.0 cap governs everything else. The target is both the commanded setpoint and the release point, so the fan ramp keeps stepping down until the call ends, reaching index 2 (25%) before handing back to the idle band, which forces `FAN_IDX_MIN`. `FAN_RAMP_SPAN_C` is 3.0 so that all five speeds land on the 0.5 C grid the room and the setpoint share: delta 0.5 is 25%, 1.0 and 1.5 are 50%, 2.0 and 2.5 are 75%, and 100% needs a 3.0 C excursion. At the previous 1.5 the 25% rung spanned delta 0 to ~0.44 and nothing ever landed in it, while any call starting at or beyond its own threshold was already saturated, since `AUTO_PULL_MAX_C` alone is 2.0. See `ac_decision.cpp`.
- **Dehumidify is a toggle over the idle band, not a separate mode.** The Mitsubishi unit only operates in one mode at a time (HEAT, COOL, DRY, FAN, AUTO), and HomeKit exposes HeaterCooler and Dehumidifier as separate services mapping to the same physical mode register. The two services coexist: whenever Smart Auto is not running a HEAT or COOL call, the idle band runs DRY if the toggle is armed and FAN if it is not. With the accessory off, an armed toggle still runs standalone DRY. There is no auto-deactivation interlock between the services; `ac_decide()` resolves the single physical mode instead.
- **The Auto range is held at least `THRESHOLD_MIN_GAP_C` (3.0 C) wide, on every path into `ac_decide()`.** The decision core has no defences of its own: an inverted pair satisfies both entry tests at once and alternates HEAT/COOL every tick with the room stationary, which is a CN105 mode-change packet every tick. 3.0 rather than 2.0 because at exactly 2.0 the pull floor and the half-range cap are both 1.0, putting both targets on the range midpoint. `normalize_thresholds()` (in `ac_decision.cpp`, so host tests cover it) is the single guard, and it runs from three places: both HomeKit threshold setters via `enforce_threshold_gap()`, restored flash state, and the legacy v1.12 migration, which admits 10-40 C unordered. Firmware before 1.32 enforced nothing (its guard assigned each threshold to itself), so a unit upgrading onto this build can be holding a pair that needs repair. It moves the threshold the caller did *not* set so their write survives, and is idempotent, so an echoed notify cannot start a volley.
- **The threshold sliders span 5-40 C, which is wider than the unit can be told.** `TEMP_MAP` in `HeatPump.cpp` is the CN105 vocabulary: 16-31 C in 1 C steps, or 10-31 in the extended 0.5 C `tempMode`. `HeatPump::setTemperature()` clamps to the nearer end, so a target outside that band becomes 16 or 31 rather than the value the slider showed. The wide ends are a way to say "do not condition on this side" and park the room in the idle band, not commandable setpoints. The declaration is also outside the HAP defaults for these characteristics (cooling 10-35, heating 0-25). Changing either range needs a `config_number` bump in `homekit_ac.c` so the Home app re-reads the schema.
- **A guard correction is re-notified from `ac_controller_loop()`, not from the setter.** The HomeKit server re-notifies the value the *client* wrote once the setter returns, and the event queue is last-wins, so a correction sent inline is overwritten for every other subscribed client.
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

### Logging Format Specifiers (GLOG_* macros)

The `GLOG_INFO/TRACE/WARN/ERROR` macros delegate to ArduinoLog (thijse/Arduino-Log), which has its **own non-printf format parser**. The single character after `%` is the entire specifier — width and length modifiers are **not** parsed and silently consume the wrong things:

- ✅ Works: `%s %S %d %i %u %l %F %D %x %X %p %b %B %c %C %t %T`
- ❌ Silently broken: `%02X` `%5d` `%lu` `%lus` — the parser sees `%` then the first char as the specifier, then the rest as literal text. The format arg is left unconsumed, so any subsequent format args shift down by one. This is especially bad for `GLOG_TRACE` and `GLOG_ERROR` which append `(Heap: %u)` — the heap counter reads the wrong arg.

When you need a hex byte or padded int, snprintf into a small char buffer first and pass it via `%s` — see the existing `subTypeHex` pattern in `HeatPump.cpp`.

### Maintenance Behavior

The firmware auto-reboots after 24h (if AC idle) or 48h (unconditionally) to mitigate memory fragmentation and HomeKit session exhaustion. WiFi loss > 2 minutes also triggers a reboot.
