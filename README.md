# GootAC - Native Apple HomeKit Controller for Mitsubishi Air Conditioners

GootAC is a firmware for the ESP8266 (built for the Wemos D1 Mini v4 that I personally use for my mini splits) that provides native Apple HomeKit integration for Mitsubishi air conditioning units. It interfaces directly with the unit via the 5-pin CN105 serial port and presents a homekit native HeaterCooler and Dehumidifier service.

---

## Key Features

- **Native HomeKit Integration**: Pair directly with the Apple Home app.
- **Multifunction Interface**:
    - **HeaterCooler Service**: Heat, Cool, and Smart Auto modes with heating/cooling threshold sliders and Swing Mode; fan speed is driven by the firmware.
    - **Dehumidifier Service**: Toggle that arms drying; the unit runs Dry only while reported humidity exceeds the target slider.
- **Embedded Diagnostics**: Flash-persistent system logs stored on-device and served via HTTP.
- **OTA Updates**: Wireless firmware deployment via ArduinoOTA.

---

## Hardware Configuration

### Wiring and Interfacing

The ESP8266 connects directly to the 5-pin CN105 port on the Mitsubishi indoor unit's PCB. For the Wemos D1 Mini, use the hardware UART0 (RX/TX pins) to communicate with the unit.

![Common Wiring Diagram](wiring.webp)

> [!NOTE]
> Some sources (like the above image, from Swicago's HeatPump library) suggest using a level shifter, but it is unnecessary for the Wemos D1 mini at least, in my experience.

#### CN105 Pinout
The pins are numbered 1 to 5 from the 12V supply pin:
1.  **12V**: Not Connected (NC)
2.  **GND**: Ground (Connect to ESP8266 GND)
3.  **5V**: Power (Connect to ESP8266 5V/VCC)
4.  **TX**: Transmit (Connect to ESP8266 RX - Note: hardware UART0 mapping)
5.  **RX**: Receive (Connect to ESP8266 TX - Note: hardware UART0 mapping)

---

## Initial Setup

### 1. Configuration
Initialize your build environment by copying the template:

```bash
cp src/config.h.example src/config.h
```

Modify `src/config.h` with your WiFi credentials and accessory naming.

### 2. Manual Deployment
This project uses [PlatformIO Core](https://docs.platformio.org/).

```bash
# Upload via Serial (Initial Flash)
pio run -t upload

# Upload via Over-the-Air
pio run -t upload --upload-port <DEVICE_IP_ADDRESS>
```

### 3. HomeKit Pairing
1.  Open the Home App.
2.  Select **Add Accessory** > **More options...**
3.  Select **GootAC** from discovered devices.
4.  Enter the setup code: `111-22-333`

---

## Management Utility

The included `manage.py` utility handles routine administrative tasks, discovery, and automated updates.

### Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Commands

| Command | Description |
| :--- | :--- |
| `list` | Discover and display diagnostics for all GootAC units on the network and serial. |
| `update` | Automatically update firmware on specific or all outdated devices (OTA). |
| `install` | Perform a factory reset and initial serial installation for new devices. |
| `monitor` | Open the Serial Monitor for troubleshooting connected USB devices. |
| `debug-usb` | Display detailed hardware and port information for connected controllers. |

---

## Technical Specifications

Logs are stored on the internal LittleFS storage to preserve history across reboots and network instability. HTTP endpoints:
- `http://<DEVICE_IP>/log` - Full durable history (INFO and up), oldest first; 512KB of rename-rotated generation files.
- `http://<DEVICE_IP>/trace` - Verbose RAM ring (TRACE lives only here); `?clear` empties it.
- `http://<DEVICE_IP>/loglevel?set=5` - Runtime verbosity (2..6, 5 = trace); reboot returns to the build default.
- `http://<DEVICE_IP>/crash` - Pre-crash verbose ring, captured on the first boot after an exception/watchdog reset.
- `http://<DEVICE_IP>/metrics` - Prometheus text-format metrics (heap, WiFi, TCP, HomeKit, AC state).
- `http://<DEVICE_IP>/control?confirm=yes&...` - Out-of-band target-state writes and the external humidity feed (`humidity=`); drives the same code paths as HomeKit writes.
- `http://<DEVICE_IP>/reboot?confirm=yes` - Plain reboot, pairings kept.
- `http://<DEVICE_IP>/hk_reset?confirm=yes-wipe-pairings` - Wipe HomeKit pairings and reboot.

---

## Licensing
Licensed under the GNU Lesser General Public License (LGPL).
Incorporates logic and components from:
- [Arduino-HomeKit-ESP8266](https://github.com/Mixiaoxiao/Arduino-HomeKit-ESP8266)
- [SwiCago's HeatPump Library](https://github.com/SwiCago/HeatPump)
