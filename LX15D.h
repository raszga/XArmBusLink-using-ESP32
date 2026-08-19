#pragma once
// ============================================================================
//  LX15D.h  —  Hiwonder LX-15D bus servo library for ESP32 / Arduino
//  All 28 commands from the LX/Lobot protocol manual.
//
//  Half-duplex wiring (your transceiver board):
//    ESP32 TX  → GPIO17   (to 74LS126 buffer input)
//    ESP32 RX  → GPIO16   (from 74HC04 inverter output)
//    DIR pin   → GPIO4    (HIGH = transmit, LOW = receive)
//
//  Usage:
//    #include "LX15D.h"
//    LX15D bus(Serial2, 4);          // Serial2, DIR on GPIO4
//
//    void setup() {
//        bus.begin(115200, 16, 17);  // baud, RX pin, TX pin
//    }
//
//    void loop() {
//        bus.move(6, 500, 1000);     // servo 6 → centre in 1 s
//        delay(1200);
//    }
// ============================================================================

#include <Arduino.h>
#include <HardwareSerial.h>

// ── command codes ────────────────────────────────────────────────────────────
#define LX_MOVE_TIME_WRITE 0x01
#define LX_MOVE_TIME_READ 0x02
#define LX_MOVE_TIME_WAIT_WRITE 0x07
#define LX_MOVE_TIME_WAIT_READ 0x08
#define LX_MOVE_START 0x0B
#define LX_MOVE_STOP 0x0C
#define LX_ID_WRITE 0x0D
#define LX_ID_READ 0x0E
#define LX_ANGLE_OFFSET_ADJUST 0x11
#define LX_ANGLE_OFFSET_WRITE 0x12
#define LX_ANGLE_OFFSET_READ 0x13
#define LX_ANGLE_LIMIT_WRITE 0x14
#define LX_ANGLE_LIMIT_READ 0x15
#define LX_VIN_LIMIT_WRITE 0x16
#define LX_VIN_LIMIT_READ 0x17
#define LX_TEMP_MAX_LIMIT_WRITE 0x18
#define LX_TEMP_MAX_LIMIT_READ 0x19
#define LX_TEMP_READ 0x1A
#define LX_VIN_READ 0x1B
#define LX_POS_READ 0x1C
#define LX_MODE_WRITE 0x1D
#define LX_MODE_READ 0x1E
#define LX_LOAD_WRITE 0x1F
#define LX_LOAD_READ 0x20
#define LX_LED_CTRL_WRITE 0x21
#define LX_LED_CTRL_READ 0x22
#define LX_LED_ERROR_WRITE 0x23
#define LX_LED_ERROR_READ 0x24

// ── LED alarm bitmask constants ───────────────────────────────────────────────
#define LX_LED_ERR_OVER_TEMP 0x01
#define LX_LED_ERR_OVER_VOLT 0x02
#define LX_LED_ERR_STALL 0x04

// ── special IDs ──────────────────────────────────────────────────────────────
#define LX_BROADCAST 0xFE
#define LX_NO_DIR_PIN -1  // pass when no direction-control pin is used

// ── mode constants ────────────────────────────────────────────────────────────
#define LX_MODE_POSITION 0
#define LX_MODE_MOTOR 1

// ── read buffer size ──────────────────────────────────────────────────────────
#define LX_BUF_SIZE 16

// ── timeout for reading a reply (ms) ─────────────────────────────────────────
#define LX_READ_TIMEOUT_MS 10

// ── inter-packet gap (ms) ─────────────────────────────────────────────────────
#define LX_PACKET_GAP_MS 2


// ── data structures ───────────────────────────────────────────────────────────

/** Snapshot of all readable servo parameters. */
struct LX15DStatus {
  uint8_t id;
  uint8_t temperature;  // °C
  float voltage;        // V
  int16_t position;     // 0-1000
  uint8_t mode;         // LX_MODE_POSITION or LX_MODE_MOTOR
  int16_t motorSpeed;   // -1000 … +1000 (motor mode only)
  bool torqueEnabled;
  bool ledOn;
  uint8_t ledAlarmMask;  // bitmask: LX_LED_ERR_*
  int8_t angleOffset;    // -125 … +125
  uint16_t angleMin;     // 0-1000
  uint16_t angleMax;     // 0-1000
  uint16_t vinMinMv;     // mV
  uint16_t vinMaxMv;     // mV
  uint8_t tempLimit;     // °C
};

/** One entry in a synchronised multi-servo move. */
struct LX15DMove {
  uint8_t id;
  uint16_t position;  // 0-1000
};


// ── class ─────────────────────────────────────────────────────────────────────
class LX15D {
public:
  /**
     * @param serial   HardwareSerial instance to use (Serial1, Serial2, …)
     * @param dirPin   GPIO pin that controls TX/RX direction on your transceiver.
     *                 HIGH = transmit, LOW = receive.
     *                 Pass LX_NO_DIR_PIN (-1) if the transceiver handles direction
     *                 automatically (e.g. auto-direction chips).
     */
  LX15D(HardwareSerial &serial, int dirPin = LX_NO_DIR_PIN);

  /**
     * Initialise the serial port.
     * @param baud   Baud rate (default 115200).
     * @param rxPin  ESP32 RX GPIO (-1 = use hardware default).
     * @param txPin  ESP32 TX GPIO (-1 = use hardware default).
     */
  void begin(uint32_t baud = 115200, int rxPin = -1, int txPin = -1);

  // ── 0x01  move to position in time ──────────────────────────────────────
  void move(uint8_t id, uint16_t position, uint16_t durationMs = 1000);

  // ── 0x02  read last commanded target + duration ──────────────────────────
  bool readMoveTime(uint8_t id, uint16_t &position, uint16_t &durationMs);

  // ── 0x07  buffer a move (fires on moveStart) ─────────────────────────────
  void moveWait(uint8_t id, uint16_t position, uint16_t durationMs = 1000);

  // ── 0x08  read buffered target + duration ────────────────────────────────
  bool readMoveWait(uint8_t id, uint16_t &position, uint16_t &durationMs);

  // ── 0x0B  execute all buffered moves simultaneously ──────────────────────
  void moveStart(uint8_t id = LX_BROADCAST);

  // ── 0x0C  stop immediately ───────────────────────────────────────────────
  void moveStop(uint8_t id = LX_BROADCAST);

  // ── 0x0D  change servo ID (saved to flash, careful!) ─────────────────────
  void writeId(uint8_t id, uint8_t newId);

  // ── 0x0E  read servo ID ──────────────────────────────────────────────────
  bool readId(uint8_t id, uint8_t &result);

  // ── 0x11  set angle offset (RAM only, not saved) ─────────────────────────
  void setAngleOffset(uint8_t id, int8_t offset);

  // ── 0x12  save current angle offset to flash ─────────────────────────────
  void saveAngleOffset(uint8_t id);

  // ── 0x13  read stored angle offset ───────────────────────────────────────
  bool readAngleOffset(uint8_t id, int8_t &offset);

  // ── 0x14  set soft angle limits ──────────────────────────────────────────
  void setAngleLimits(uint8_t id, uint16_t minPos, uint16_t maxPos);

  // ── 0x15  read angle limits ───────────────────────────────────────────────
  bool readAngleLimits(uint8_t id, uint16_t &minPos, uint16_t &maxPos);

  // ── 0x16  set voltage protection limits (mV) ─────────────────────────────
  void setVinLimits(uint8_t id, uint16_t minMv, uint16_t maxMv);

  // ── 0x17  read voltage limits ─────────────────────────────────────────────
  bool readVinLimits(uint8_t id, uint16_t &minMv, uint16_t &maxMv);

  // ── 0x18  set over-temperature limit (°C) ────────────────────────────────
  void setTempLimit(uint8_t id, uint8_t maxTempC);

  // ── 0x19  read temperature limit ─────────────────────────────────────────
  bool readTempLimit(uint8_t id, uint8_t &maxTempC);

  // ── 0x1A  read current temperature (°C) ──────────────────────────────────
  bool readTemperature(uint8_t id, uint8_t &tempC);

  // ── 0x1B  read current voltage (mV) ──────────────────────────────────────
  bool readVoltage(uint8_t id, uint16_t &mv);

  // ── 0x1C  read current position (0-1000) ─────────────────────────────────
  bool readPosition(uint8_t id, int16_t &position);

  // ── 0x1D  set position mode ───────────────────────────────────────────────
  void setPositionMode(uint8_t id);

  // ── 0x1D  set motor (wheel) mode with speed ───────────────────────────────
  void setMotorMode(uint8_t id, int16_t speed);

  // ── 0x1D  update motor speed while already in motor mode ─────────────────
  void setMotorSpeed(uint8_t id, int16_t speed);  // alias for setMotorMode

  // ── 0x1E  read current mode and speed ────────────────────────────────────
  bool readMode(uint8_t id, uint8_t &mode, int16_t &speed);

  // ── 0x1F  enable / disable torque ────────────────────────────────────────
  void setTorque(uint8_t id, bool enable);

  // ── 0x20  read torque state ───────────────────────────────────────────────
  bool readTorque(uint8_t id, bool &enabled);

  // ── 0x21  LED on / off ────────────────────────────────────────────────────
  void setLed(uint8_t id, bool on);

  // ── 0x22  read LED state ──────────────────────────────────────────────────
  bool readLed(uint8_t id, bool &on);

  // ── 0x23  set LED alarm mask (LX_LED_ERR_* bitmask) ──────────────────────
  void setLedAlarm(uint8_t id, uint8_t mask);

  // ── 0x24  read LED alarm mask ─────────────────────────────────────────────
  bool readLedAlarm(uint8_t id, uint8_t &mask);

  // ── convenience: move multiple servos simultaneously ─────────────────────
  /**
     * Buffer a move on each servo, then fire a broadcast moveStart so all
     * servos depart at the same instant.
     *
     * @param moves      Array of LX15DMove structs {id, position}.
     * @param count      Number of entries in the array.
     * @param durationMs Travel time, same for all servos (0-30000 ms).
     *
     * Example:
     *   LX15DMove grp[] = {{1, 200}, {2, 800}, {6, 500}};
     *   bus.moveSync(grp, 3, 1500);
     */
  void moveSync(LX15DMove *moves, uint8_t count, uint16_t durationMs = 1000, bool wait = true);

  // ── convenience: read all parameters into LX15DStatus struct ─────────────
  bool getStatus(uint8_t id, LX15DStatus &status);

  // ── convenience: scan a range of IDs, returns count of found servos ───────
  /**
     * Tries each ID from startId to endId, fills foundIds[].
     * Returns number of servos found.
     *
     * Example:
     *   uint8_t found[16];
     *   uint8_t n = bus.scan(found, 16);
     */
  uint8_t scan(uint8_t *foundIds, uint8_t maxFound,
               uint8_t startId = 1, uint8_t endId = 253);

private:
  HardwareSerial &_serial;
  int _dirPin;
  uint8_t _rxBuf[LX_BUF_SIZE];

  static uint8_t _checksum(uint8_t id, uint8_t length,
                           uint8_t cmd, const uint8_t *params, uint8_t paramLen);

  void _txMode();
  void _rxMode();
  void _sendPacket(uint8_t id, uint8_t cmd,
                   const uint8_t *params = nullptr, uint8_t paramLen = 0);
  uint8_t _readResponse(uint8_t expectedCmd);  // returns param byte count, 0=fail
};
