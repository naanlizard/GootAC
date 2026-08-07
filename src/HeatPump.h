/*
  HeatPump.h - Mitsubishi Heat Pump control library for Arduino
  Copyright (c) 2017 Al Betschart.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef __HeatPump_H__
#define __HeatPump_H__
#include <HardwareSerial.h>
#include <math.h>
#include <stdint.h>
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/*
 * Callback function definitions. Code differs for the ESP8266 platform, which
 * requires the functional library. Based on callback implementation in the
 * Arduino Client for MQTT library (https://github.com/knolleary/pubsubclient)
 */
#ifdef ESP8266
#include <functional>
#define ON_CONNECT_CALLBACK_SIGNATURE std::function<void()> onConnectCallback
#define SETTINGS_CHANGED_CALLBACK_SIGNATURE                                    \
  std::function<void()> settingsChangedCallback
#define STATUS_CHANGED_CALLBACK_SIGNATURE                                      \
  std::function<void(heatpumpStatus newStatus)> statusChangedCallback
#define PACKET_CALLBACK_SIGNATURE                                              \
  std::function<void(byte * packet, unsigned int length,                       \
                     char *packetDirection)>                                   \
      packetCallback
#define ROOM_TEMP_CHANGED_CALLBACK_SIGNATURE                                   \
  std::function<void(float currentRoomTemperature)> roomTempChangedCallback
#else
#define ON_CONNECT_CALLBACK_SIGNATURE void (*onConnectCallback)()
#define SETTINGS_CHANGED_CALLBACK_SIGNATURE void (*settingsChangedCallback)()
#define STATUS_CHANGED_CALLBACK_SIGNATURE                                      \
  void (*statusChangedCallback)(heatpumpStatus newStatus)
#define PACKET_CALLBACK_SIGNATURE                                              \
  void (*packetCallback)(byte * packet, unsigned int length,                   \
                         char *packetDirection)
#define ROOM_TEMP_CHANGED_CALLBACK_SIGNATURE                                   \
  void (*roomTempChangedCallback)(float currentRoomTemperature)
#endif

typedef uint8_t byte;

struct heatpumpSettings {
  const char *power;
  const char *mode;
  float temperature;
  const char *fan;
  const char *vane;     // vertical vane, up/down
  const char *wideVane; // horizontal vane, left/right
  bool iSee; // iSee sensor, at the moment can only detect it, not set it
};

bool operator==(const heatpumpSettings &lhs, const heatpumpSettings &rhs);
bool operator!=(const heatpumpSettings &lhs, const heatpumpSettings &rhs);
bool operator!(const heatpumpSettings &settings);

struct heatpumpTimers {
  const char *mode;
  int onMinutesSet;
  int onMinutesRemaining;
  int offMinutesSet;
  int offMinutesRemaining;
};

bool operator==(const heatpumpTimers &lhs, const heatpumpTimers &rhs);
bool operator!=(const heatpumpTimers &lhs, const heatpumpTimers &rhs);

struct heatpumpStatus {
  float roomTemperature;
  float outsideAirTemperature;
  bool operating; // if true, the heatpump is operating to reach the desired
                  // temperature
  heatpumpTimers timers;
  // Raw byte 3 of the 0x06 status packet (SwiCago: "compressorFrequency").
  // Moves in lockstep across all indoor units on one outdoor unit, so it is
  // system-wide outdoor telemetry; semantics unverified, NOT degrees C.
  int statusByte3;
  // Status flags decoded from info-type 0x09 byte 3 (muart-group spec).
  // bit 0 filter, bit 1 defrost, bit 2 preheat, bit 3 standby/blocked.
  uint8_t statusFlags;
  // Actual fan speed reported by the AC (may differ from requested fan).
  // Kumo UI mapping: 0=Off, 1=VeryLow, 2=Quiet, 3=Low, 4=Powerful,
  // 5=SuperPowerful, 6=SuperQuiet.
  uint8_t actualFanSpeed;
  // Auto sub-mode (low nibble) + leader bit 0x40. Low nibble values:
  // 0=Direct, 1=AutoFan, 2=AutoHeat, 3=AutoCool.
  uint8_t autoSubMode;
};

// Bit positions in heatpumpStatus.statusFlags.
constexpr uint8_t HP_STATUS_FILTER_DIRTY    = 0x01;
constexpr uint8_t HP_STATUS_DEFROST         = 0x02;
constexpr uint8_t HP_STATUS_PREHEAT         = 0x04;
constexpr uint8_t HP_STATUS_BLOCKED_BY_OTHER = 0x08;

class HeatPump {
private:
  static const int PACKET_LEN = 22;
  static const int PACKET_SENT_INTERVAL_MS = 1000;
  static const int PACKET_INFO_INTERVAL_MS = 2000;
  static const int PACKET_TYPE_DEFAULT = 99;

  // Protocol byte tables (CONNECT, HEADER, POWER, MODE, ...) are file-scope
  // PROGMEM arrays in HeatPump.cpp; upstream keeps them as per-instance RAM
  // members (~450B on a 40KB-heap part).
  static const int CONNECT_LEN = 8;
  static const int HEADER_LEN = 8;
  static const int INFOHEADER_LEN = 5;
  static const int INFOMODE_LEN = 5;

  const int RCVD_PKT_FAIL = 0;
  const int RCVD_PKT_CONNECT_SUCCESS = 1;
  const int RCVD_PKT_SETTINGS = 2;
  const int RCVD_PKT_ROOM_TEMP = 3;
  const int RCVD_PKT_UPDATE_SUCCESS = 4;
  const int RCVD_PKT_STATUS = 5;
  const int RCVD_PKT_TIMER = 6;

  static const int TIMER_INCREMENT_MINUTES = 10;

  heatpumpSettings currentSettings;
  heatpumpSettings wantedSettings;
  heatpumpStatus currentStatus;

  HardwareSerial *_HardSerial;
  unsigned long lastSend;
  bool waitForRead;
  int infoMode;
  unsigned long lastRecv;
  bool connected = false;

  // Async SET and connect-handshake state, driven from loop(). Replaces the
  // upstream busy-waits that stalled the main loop 1-8s per command/reconnect.
  static const int UPDATE_ACK_TIMEOUT_MS = 2000;
  static const int CONNECT_SETTLE_MS = 2000;
  static const int CONNECT_ACK_TIMEOUT_MS = 2000;
  static const int CONNECT_BACKOFF_MS = 10000;
  enum ConnState : uint8_t { CONN_IDLE = 0, CONN_SETTLE, CONN_WAIT_ACK, CONN_BACKOFF };
  bool updatePending = false;
  bool awaitingAck = false;
  uint8_t updateRetries = 0;
  unsigned long ackDeadline = 0;
  ConnState connState = CONN_IDLE;
  unsigned long connSince = 0;
  bool connTriedBothRates = false;
  bool autoUpdate;
  bool firstRun;
  bool tempMode;
  bool externalUpdate;
  bool _externalUpdateOccurred;
  bool wideVaneAdj;
  int bitrate = 2400;

  const char *lookupByteMapValue(const char *valuesMap[], const byte byteMap[],
                                 int len, byte byteValue);
  int lookupByteMapValue(const int valuesMap[], const byte byteMap[], int len,
                         byte byteValue);
  int lookupByteMapIndex(const char *valuesMap[], int len,
                         const char *lookupValue);
  int lookupByteMapIndex(const int valuesMap[], int len, int lookupValue);

  bool canSend(bool isInfo);
  bool canRead();
  int pollRead();
  byte checkSum(byte bytes[], int len);
  void createPacket(byte *packet, heatpumpSettings settings);
  void createInfoPacket(byte *packet, byte packetType);
  int readPacket();
  void writePacket(byte *packet, int length);

  ON_CONNECT_CALLBACK_SIGNATURE;
  SETTINGS_CHANGED_CALLBACK_SIGNATURE;
  STATUS_CHANGED_CALLBACK_SIGNATURE;
  PACKET_CALLBACK_SIGNATURE;
  ROOM_TEMP_CHANGED_CALLBACK_SIGNATURE;

public:
  // Public maps for index-based control in application code
  const char *POWER_MAP[2] = {"OFF", "ON"};
  const char *MODE_MAP[5] = {"HEAT", "DRY", "COOL", "FAN", "AUTO"};
  const int TEMP_MAP[16] = {31, 30, 29, 28, 27, 26, 25, 24,
                            23, 22, 21, 20, 19, 18, 17, 16};
  const char *FAN_MAP[6] = {"AUTO", "QUIET", "1", "2", "3", "4"};
  const char *VANE_MAP[7] = {"AUTO", "1", "2", "3", "4", "5", "SWING"};
  const char *WIDEVANE_MAP[7] = {"<<", "<", "|", ">", ">>", "<>", "SWING"};
  const int ROOM_TEMP_MAP[32] = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                                 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
                                 32, 33, 34, 35, 36, 37, 38, 39, 40, 41};
  const char *TIMER_MODE_MAP[4] = {"NONE", "OFF", "ON", "BOTH"};

  const int RQST_PKT_SETTINGS = 0;
  const int RQST_PKT_ROOM_TEMP = 1;
  const int RQST_PKT_TIMERS = 3;
  const int RQST_PKT_STATUS = 4;

  HeatPump();
  bool connect(HardwareSerial *serial);
  bool connect(HardwareSerial *serial, bool retry);
  bool update();
  void sync(byte packetType);
  void loop();
  void enableExternalUpdate();
  void enableAutoUpdate();
  void disableAutoUpdate();

  heatpumpSettings getSettings();
  heatpumpSettings getWantedSettings();
  void setSettings(heatpumpSettings settings);
  void setPowerSetting(bool setting);
  bool getPowerSettingBool();
  const char *getPowerSetting();
  void setPowerSetting(const char *setting);
  const char *getModeSetting();
  void setModeSetting(const char *setting);
  void setModeIndex(uint8_t modeIndex);
  float getTemperature();
  void setTemperature(float setting);
  void setRemoteTemperature(float setting);
  const char *getFanSpeed();
  void setFanSpeed(const char *setting);
  void setFanSpeedIndex(uint8_t fanIndex);
  const char *getVaneSetting();
  void setVaneSetting(const char *setting);
  void setVaneIndex(uint8_t vaneIndex);
  const char *getWideVaneSetting();
  void setWideVaneSetting(const char *setting);
  void setWideVaneIndex(uint8_t wideVaneIndex);
  bool getIseeBool();

  heatpumpStatus getStatus();
  float getRoomTemperature();
  bool getOperating();
  bool isConnected();
  bool wasExternalUpdate() { return _externalUpdateOccurred; }
  // True while a queued SET has not yet been written and acknowledged; an
  // info request sent in that window would read back pre-SET state.
  bool updateInFlight();

  float FahrenheitToCelsius(int tempF);
  int CelsiusToFahrenheit(float tempC);

  void setOnConnectCallback(ON_CONNECT_CALLBACK_SIGNATURE);
  void setSettingsChangedCallback(SETTINGS_CHANGED_CALLBACK_SIGNATURE);
  void setStatusChangedCallback(STATUS_CHANGED_CALLBACK_SIGNATURE);
  void setPacketCallback(PACKET_CALLBACK_SIGNATURE);
  void setRoomTempChangedCallback(ROOM_TEMP_CHANGED_CALLBACK_SIGNATURE);

  void sendCustomPacket(byte data[], int len);
};
#endif
