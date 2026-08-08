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

# Host-side tests for the pure decision core (no hardware needed)
cd host_tests && make run
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
ac_controller.cpp     — Bridge logic: translates between HomeKit values and HeatPump library calls;
                        delegates the pure mode/setpoint/fan decision to ac_decision.cpp
    ↕ (HeatPump API)
HeatPump.cpp/.h       — Serial protocol driver for Mitsubishi CN105 (vendored from SwiCago/HeatPump)
    ↕ (UART0 @ 2400 baud)
Mitsubishi indoor unit
```

### Key Design Decisions

- **UART0 is reserved for AC communication.** No serial logging; everything goes through the two-tier `glog` logger: INFO+ to LittleFS generation files (`/log`), everything to a RAM ring (`/trace`, `.noinit` so the pre-crash tail survives to `/crash`). Verbosity is runtime-switchable at `/loglevel` (5 = trace); boot default is `APP_LOG_LEVEL`.
- **Every target mode runs firmware call cycles; the unit's native auto is only the room-temp-unknown fallback.** A Smart Auto call starts when the room leaves the heating/cooling range; explicit HEAT/COOL run the same cycle one-sided off their own threshold. A call ends at its target, a static `CALL_PULL_C` (0.5 C) past the threshold; that target is both the commanded setpoint and the release point, and the idle staircase picks up at the same depth on release.
- **Dehumidify is a gated toggle, not a mode.** An armed toggle only produces DRY while the humidity latch is set (reported humidity over the HomeKit threshold, released at threshold minus 5; unknown or >15-min-stale readings force it off) — units with no humidity feed never dry. In the active idle band DRY additionally waits for the fan staircase to reach QUIET, so it only ever replaces a quiet fan; with the accessory off, a latched toggle runs standalone DRY and otherwise powers off. The two HomeKit services have no interlock, because `ac_decide()` resolves the single physical mode register.
- **The idle band delegates the fan to the unit in DRY** (`fan_idx` 0) while the staircase keeps tracking depth underneath (its rung is the DRY exit condition); in FAN it runs the staircase on depth past its anchor target (the nearer one in Smart Auto, the mode's own side in explicit HEAT/COOL), so circulation winds down after a release and back up as the room re-approaches a trigger; a room between a target and its threshold idles at 100%.
- **The Auto range is held at least `THRESHOLD_MIN_GAP_C` (3.0 C) wide on every path into `ac_decide()`**, which has no defences of its own: an inverted pair trips both entry tests and flips HEAT/COOL every tick. `normalize_thresholds()` is the only guard, and runs from both threshold setters and from every flash load/migration path. At width 1.0 the half-range cap collapses both targets onto the midpoint.
- **The threshold sliders span 16-31 C, matching `TEMP_MAP` (`HeatPump.h`), the CN105 setpoint vocabulary.** A target the unit cannot be told is a release point the room never reaches, so `cool/heat_call_target()` clamp into 16-31 (`TEMP_CMD_MIN/MAX_C`). Changing the slider range needs a `config_number` bump in `homekit_ac.c`.
- **The fan is a staircase: 100% until the room reaches the commanded target, then one rung down per 0.5 C of depth past it (75/50/25/QUIET).** Up-moves are immediate; down-moves wait out 60 s of sustained lower demand. A reading dithering across a boundary faster than the dwell pins at the higher level; slower dither is rate-bounded to one change per dwell, not pinned.
- **A guard correction is re-notified from `ac_controller_loop()`, not the setter.** The server re-notifies the client's own value after the setter returns, and the per-client queue coalesces oldest-wins, so an inline correction loses.
- **Target state persistence**: `/target_state.bin` on LittleFS with a checksum.
- **External change detection**: the library flags `_externalUpdateOccurred` when a received settings packet differs from `wantedSettings`, and the controller syncs power always but mode only when the unit reports HEAT/COOL/AUTO in a fixed target mode: FAN and DRY are indistinguishable from the firmware's own idle band, and Smart Auto never syncs mode. Thresholds never sync. Fan is masked from that comparison, and so is temperature in FAN and DRY, since neither is commanded with a setpoint.

### Source Files

- `main.cpp` — Boot sequence (WiFi, mDNS, OTA, web server), main loop and watchdogs
- `ac_controller.cpp/.h` — HomeKit↔AC bridge: setters, persistence/migrations, external sync, `/metrics`, logging macros (`GLOG_*`, `HKLOG_*`)
- `ac_decision.cpp/.h` — Pure decision core (call cycles, fan staircase, `normalize_thresholds()`, compressor gate); no Arduino/HomeKit deps, exercised by `host_tests/test_decide.cpp`
- `homekit_ac.c/.h` — HomeKit accessory definition (services, characteristics, pairing password)
- `glog.h/.cpp` — Two-tier logger: `/log.0..15` generation files rotated by rename (32KB each, 512KB budget) for INFO+, plus a 4KB `.noinit` RAM ring for everything incl. TRACE; crash capture writes the ring to `/crash.log` on the first boot after an exception/wdt reset
- `hk_log_bridge.cpp/.h` — RAM ring for the HomeKit library's own logs; only in the `d1_mini_hklog` env, served at `/hklog`
- `HeatPumpFake.cpp/.h` — hardware-free `HeatPump` implementation for the `d1_mini_fake` diagnostic env (`/fake/*` endpoints)
- `HeatPump.cpp/.h` — Vendored from [SwiCago/HeatPump](https://github.com/SwiCago/HeatPump) and modified locally. Key changes from upstream:
  - **Index-based setters** (`setModeIndex`, `setFanSpeedIndex`, `setVaneIndex`, `setWideVaneIndex`) added so `ac_controller.cpp` can set values by numeric index into the public `MODE_MAP`/`FAN_MAP`/`VANE_MAP` arrays, avoiding string comparisons and matching HomeKit's numeric characteristic model.
  - **`getWantedSettings()`** exposes the library's internal `wantedSettings` so the controller can compare desired vs actual state before sending redundant packets to the AC.
  - **`sync(byte packetType)`** added for on-demand info requests (e.g., force a settings re-read after an update) rather than waiting for the next polling cycle.
  - **`wasExternalUpdate()`** / `_externalUpdateOccurred` flag added to detect when a settings change came from the IR remote or unit itself rather than from our commands (see external change detection above).
  - **`GLOG_*` logging** integrated throughout — upstream uses `Serial.println` which conflicts with UART0 being reserved for the CN105 protocol. All debug output now routes through `glog`.
  - **Public constant maps** (`MODE_MAP`, `FAN_MAP`, `VANE_MAP`, etc.) moved from private to public so `ac_controller.cpp` can reference them for command logging and comparisons.
  - **Non-blocking `connect()`/`update()`**: upstream busy-waits (2s settle + handshake, 1-3s per SET) replaced with state machines driven from `loop()`; `update()` queues, the ACK is consumed asynchronously with one resend, and reconnect backs off 10s between rounds. `updateInFlight()` lets the controller hold its post-update settings re-read until the SET lands.
  - **Protocol byte tables in PROGMEM** (file-scope in HeatPump.cpp) instead of per-instance RAM members; the free operators for the value structs live in `HeatPumpCommon.cpp`, shared with the fake driver.

### Build Flags (platformio.ini)

- `HOMEKIT_LOG_LEVEL=2` — Controls verbosity of the Arduino-HomeKit library
- `APP_LOG_LEVEL=4` — Controls app-level log verbosity (1=Fatal through 6=Verbose)
- `ARDUINO_HOMEKIT_LOWROM` / `ARDUINO_HOMEKIT_SKIP_ED25519_VERIFY` — Memory optimizations required for ESP8266
- CPU runs at 160MHz for HomeKit crypto performance

### Logging (GLOG_* macros)

`GLOG_INFO/TRACE/WARN/ERROR/BOOT` format with standard printf semantics (`vsnprintf_P`): width/length modifiers like `%02X` and `%lu` work. The format string is compiled into flash via `PSTR`; string arguments are normal RAM pointers. ERROR and TRACE lines get a `(Heap: N)` suffix appended automatically. `GLOG_TRACE` checks the runtime level before evaluating its arguments — guard any standalone prep block that only feeds a TRACE with `glog_trace_on()`.

### Maintenance Behavior

WiFi loss beyond 2 minutes triggers a reboot. There is no periodic maintenance reboot; units run for weeks. After an exception/wdt reset the pre-crash verbose ring is served at `/crash` and one ERROR line at boot points there.
