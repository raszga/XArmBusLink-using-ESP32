// ============================================================================
//  LX15D.cpp  —  Hiwonder LX-15D bus servo library for ESP32 / Arduino
// ============================================================================

#include "LX15D.h"

// ── constructor / begin ───────────────────────────────────────────────────────

LX15D::LX15D(HardwareSerial &serial, int dirPin)
  : _serial(serial), _dirPin(dirPin) {}

void LX15D::begin(uint32_t baud, int rxPin, int txPin) {
  if (_dirPin != LX_NO_DIR_PIN) {
    pinMode(_dirPin, OUTPUT);
    _rxMode();  // start in receive mode
  }
  if (rxPin >= 0 && txPin >= 0) {
    _serial.begin(baud, SERIAL_8N1, rxPin, txPin);
  } else {
    _serial.begin(baud);
  }
  delay(10);
}


// ── private helpers ───────────────────────────────────────────────────────────

uint8_t LX15D::_checksum(uint8_t id, uint8_t length,
                         uint8_t cmd, const uint8_t *params, uint8_t paramLen) {
  uint16_t sum = id + length + cmd;
  for (uint8_t i = 0; i < paramLen; i++) sum += params[i];
  return (~sum) & 0xFF;
}

void LX15D::_txMode() {
  if (_dirPin != LX_NO_DIR_PIN) digitalWrite(_dirPin, HIGH);
}

void LX15D::_rxMode() {
  if (_dirPin != LX_NO_DIR_PIN) digitalWrite(_dirPin, LOW);
}

void LX15D::_sendPacket(uint8_t id, uint8_t cmd,
                        const uint8_t *params, uint8_t paramLen) {
  uint8_t length = 3 + paramLen;
  uint8_t cs = _checksum(id, length, cmd, params, paramLen);

  if (_dirPin != LX_NO_DIR_PIN) { _txMode(); }

  _serial.write(0x55);
  _serial.write(0x55);
  _serial.write(id);
  _serial.write(length);
  _serial.write(cmd);
  for (uint8_t i = 0; i < paramLen; i++) _serial.write(params[i]);
  _serial.write(cs);

  _serial.flush();  // wait until all bytes are shifted out

  // Switch back to RX as soon as the last stop bit leaves the UART.
  // flush() returns after the TX FIFO is empty, but the shift register
  // may still be sending the last byte.  One byte at 115200 baud ≈ 87 µs;
  // adding a small margin ensures the transceiver doesn't cut the tail.
  if (_dirPin != LX_NO_DIR_PIN) {
    delayMicroseconds(200);
    _rxMode();
  }
}

uint8_t LX15D::_readResponse(uint8_t expectedCmd) {
  // Wait for header 0x55 0x55
  uint32_t t0 = millis();
  uint8_t hdrCount = 0;
  while (millis() - t0 < LX_READ_TIMEOUT_MS) {
    if (!_serial.available()) continue;
    uint8_t b = _serial.read();
    if (b == 0x55) {
      hdrCount++;
      if (hdrCount == 2) break;
    } else {
      hdrCount = 0;
    }
  }
  if (hdrCount < 2) return 0;

  // Read ID, Length, Cmd
  t0 = millis();
  while (_serial.available() < 3) {
    if (millis() - t0 > LX_READ_TIMEOUT_MS) return 0;
  }
  _rxBuf[0] = _serial.read();  // ID
  _rxBuf[1] = _serial.read();  // Length
  _rxBuf[2] = _serial.read();  // Cmd

  uint8_t paramLen = _rxBuf[1] - 3;
  if (paramLen > LX_BUF_SIZE - 4) return 0;  // sanity check

  // Read params + checksum
  t0 = millis();
  while (_serial.available() < (int)(paramLen + 1)) {
    if (millis() - t0 > LX_READ_TIMEOUT_MS) return 0;
  }
  for (uint8_t i = 0; i < paramLen; i++) _rxBuf[3 + i] = _serial.read();
  _rxBuf[3 + paramLen] = _serial.read();  // checksum byte (not verified here)

  // Validate command matches what we expect
  if (_rxBuf[2] != expectedCmd) return 0;

  delay(LX_PACKET_GAP_MS);
  return paramLen;
}


// ── 0x01  MOVE_TIME_WRITE ─────────────────────────────────────────────────────

void LX15D::move(uint8_t id, uint16_t position, uint16_t durationMs) {
  position = constrain(position, 0, 1000);
  durationMs = constrain(durationMs, 0, 30000);
  uint8_t p[4] = {
    (uint8_t)(position & 0xFF), (uint8_t)(position >> 8),
    (uint8_t)(durationMs & 0xFF), (uint8_t)(durationMs >> 8)
  };
  _sendPacket(id, LX_MOVE_TIME_WRITE, p, 4);
}


// ── 0x02  MOVE_TIME_READ ─────────────────────────────────────────────────────

bool LX15D::readMoveTime(uint8_t id, uint16_t &position, uint16_t &durationMs) {
  _sendPacket(id, LX_MOVE_TIME_READ);
  uint8_t n = _readResponse(LX_MOVE_TIME_READ);
  if (n < 4) return false;
  position = _rxBuf[3] | ((uint16_t)_rxBuf[4] << 8);
  durationMs = _rxBuf[5] | ((uint16_t)_rxBuf[6] << 8);
  return true;
}


// ── 0x07  MOVE_TIME_WAIT_WRITE ───────────────────────────────────────────────

void LX15D::moveWait(uint8_t id, uint16_t position, uint16_t durationMs) {
  position = constrain(position, 0, 1000);
  durationMs = constrain(durationMs, 0, 30000);
  uint8_t p[4] = {
    (uint8_t)(position & 0xFF), (uint8_t)(position >> 8),
    (uint8_t)(durationMs & 0xFF), (uint8_t)(durationMs >> 8)
  };
  _sendPacket(id, LX_MOVE_TIME_WAIT_WRITE, p, 4);
}


// ── 0x08  MOVE_TIME_WAIT_READ ────────────────────────────────────────────────

bool LX15D::readMoveWait(uint8_t id, uint16_t &position, uint16_t &durationMs) {
  _sendPacket(id, LX_MOVE_TIME_WAIT_READ);
  uint8_t n = _readResponse(LX_MOVE_TIME_WAIT_READ);
  if (n < 4) return false;
  position = _rxBuf[3] | ((uint16_t)_rxBuf[4] << 8);
  durationMs = _rxBuf[5] | ((uint16_t)_rxBuf[6] << 8);
  return true;
}


// ── 0x0B  MOVE_START ─────────────────────────────────────────────────────────

void LX15D::moveStart(uint8_t id) {
  _sendPacket(id, LX_MOVE_START);
}


// ── 0x0C  MOVE_STOP ──────────────────────────────────────────────────────────

void LX15D::moveStop(uint8_t id) {
  _sendPacket(id, LX_MOVE_STOP);
}


// ── 0x0D  ID_WRITE ───────────────────────────────────────────────────────────

void LX15D::writeId(uint8_t id, uint8_t newId) {
  newId = constrain(newId, 1, 253);
  _sendPacket(id, LX_ID_WRITE, &newId, 1);
}


// ── 0x0E  ID_READ ────────────────────────────────────────────────────────────

bool LX15D::readId(uint8_t id, uint8_t &result) {
  _sendPacket(id, LX_ID_READ);
  uint8_t n = _readResponse(LX_ID_READ);
  if (n < 1) return false;
  result = _rxBuf[3];
  return true;
}


// ── 0x11  ANGLE_OFFSET_ADJUST ────────────────────────────────────────────────

void LX15D::setAngleOffset(uint8_t id, int8_t offset) {
  offset = (int8_t)constrain((int)offset, -125, 125);
  uint8_t b = (uint8_t)offset;  // two's complement
  _sendPacket(id, LX_ANGLE_OFFSET_ADJUST, &b, 1);
}


// ── 0x12  ANGLE_OFFSET_WRITE (save to flash) ─────────────────────────────────

void LX15D::saveAngleOffset(uint8_t id) {
  _sendPacket(id, LX_ANGLE_OFFSET_WRITE);
}


// ── 0x13  ANGLE_OFFSET_READ ──────────────────────────────────────────────────

bool LX15D::readAngleOffset(uint8_t id, int8_t &offset) {
  _sendPacket(id, LX_ANGLE_OFFSET_READ);
  uint8_t n = _readResponse(LX_ANGLE_OFFSET_READ);
  if (n < 1) return false;
  offset = (int8_t)_rxBuf[3];  // cast re-applies sign
  return true;
}


// ── 0x14  ANGLE_LIMIT_WRITE ──────────────────────────────────────────────────

void LX15D::setAngleLimits(uint8_t id, uint16_t minPos, uint16_t maxPos) {
  minPos = constrain(minPos, 0, 1000);
  maxPos = constrain(maxPos, 0, 1000);
  uint8_t p[4] = {
    (uint8_t)(minPos & 0xFF), (uint8_t)(minPos >> 8),
    (uint8_t)(maxPos & 0xFF), (uint8_t)(maxPos >> 8)
  };
  _sendPacket(id, LX_ANGLE_LIMIT_WRITE, p, 4);
}


// ── 0x15  ANGLE_LIMIT_READ ───────────────────────────────────────────────────

bool LX15D::readAngleLimits(uint8_t id, uint16_t &minPos, uint16_t &maxPos) {
  _sendPacket(id, LX_ANGLE_LIMIT_READ);
  uint8_t n = _readResponse(LX_ANGLE_LIMIT_READ);
  if (n < 4) return false;
  minPos = _rxBuf[3] | ((uint16_t)_rxBuf[4] << 8);
  maxPos = _rxBuf[5] | ((uint16_t)_rxBuf[6] << 8);
  return true;
}


// ── 0x16  VIN_LIMIT_WRITE ────────────────────────────────────────────────────

void LX15D::setVinLimits(uint8_t id, uint16_t minMv, uint16_t maxMv) {
  uint8_t p[4] = {
    (uint8_t)(minMv & 0xFF), (uint8_t)(minMv >> 8),
    (uint8_t)(maxMv & 0xFF), (uint8_t)(maxMv >> 8)
  };
  _sendPacket(id, LX_VIN_LIMIT_WRITE, p, 4);
}


// ── 0x17  VIN_LIMIT_READ ─────────────────────────────────────────────────────

bool LX15D::readVinLimits(uint8_t id, uint16_t &minMv, uint16_t &maxMv) {
  _sendPacket(id, LX_VIN_LIMIT_READ);
  uint8_t n = _readResponse(LX_VIN_LIMIT_READ);
  if (n < 4) return false;
  minMv = _rxBuf[3] | ((uint16_t)_rxBuf[4] << 8);
  maxMv = _rxBuf[5] | ((uint16_t)_rxBuf[6] << 8);
  return true;
}


// ── 0x18  TEMP_MAX_LIMIT_WRITE ───────────────────────────────────────────────

void LX15D::setTempLimit(uint8_t id, uint8_t maxTempC) {
  maxTempC = constrain(maxTempC, 50, 100);
  _sendPacket(id, LX_TEMP_MAX_LIMIT_WRITE, &maxTempC, 1);
}


// ── 0x19  TEMP_MAX_LIMIT_READ ────────────────────────────────────────────────

bool LX15D::readTempLimit(uint8_t id, uint8_t &maxTempC) {
  _sendPacket(id, LX_TEMP_MAX_LIMIT_READ);
  uint8_t n = _readResponse(LX_TEMP_MAX_LIMIT_READ);
  if (n < 1) return false;
  maxTempC = _rxBuf[3];
  return true;
}


// ── 0x1A  TEMP_READ ──────────────────────────────────────────────────────────

bool LX15D::readTemperature(uint8_t id, uint8_t &tempC) {
  _sendPacket(id, LX_TEMP_READ);
  uint8_t n = _readResponse(LX_TEMP_READ);
  if (n < 1) return false;
  tempC = _rxBuf[3];
  return true;
}


// ── 0x1B  VIN_READ ───────────────────────────────────────────────────────────

bool LX15D::readVoltage(uint8_t id, uint16_t &mv) {
  _sendPacket(id, LX_VIN_READ);
  uint8_t n = _readResponse(LX_VIN_READ);
  if (n < 2) return false;
  mv = _rxBuf[3] | ((uint16_t)_rxBuf[4] << 8);
  return true;
}


// ── 0x1C  POS_READ ───────────────────────────────────────────────────────────

bool LX15D::readPosition(uint8_t id, int16_t &position) {
  _sendPacket(id, LX_POS_READ);
  uint8_t n = _readResponse(LX_POS_READ);
  if (n < 2) return false;
  position = (int16_t)(_rxBuf[3] | ((uint16_t)_rxBuf[4] << 8));
  return true;
}


// ── 0x1D  MODE_WRITE (position) ──────────────────────────────────────────────

void LX15D::setPositionMode(uint8_t id) {
  uint8_t p[4] = { 0, 0, 0, 0 };
  _sendPacket(id, LX_MODE_WRITE, p, 4);
}


// ── 0x1D  MODE_WRITE (motor) ─────────────────────────────────────────────────

void LX15D::setMotorMode(uint8_t id, int16_t speed) {
  speed = (int16_t)constrain((int)speed, -1000, 1000);
  uint8_t p[4] = {
    1,                        // mode = motor
    0,                        // reserved
    (uint8_t)(speed & 0xFF),  // speed low byte (signed LE)
    (uint8_t)((speed >> 8) & 0xFF)
  };
  _sendPacket(id, LX_MODE_WRITE, p, 4);
}

void LX15D::setMotorSpeed(uint8_t id, int16_t speed) {
  setMotorMode(id, speed);  // alias
}


// ── 0x1E  MODE_READ ──────────────────────────────────────────────────────────

bool LX15D::readMode(uint8_t id, uint8_t &mode, int16_t &speed) {
  _sendPacket(id, LX_MODE_READ);
  uint8_t n = _readResponse(LX_MODE_READ);
  if (n < 4) return false;
  mode = _rxBuf[3];
  speed = (int16_t)(_rxBuf[5] | ((uint16_t)_rxBuf[6] << 8));
  return true;
}


// ── 0x1F  LOAD_WRITE ─────────────────────────────────────────────────────────

void LX15D::setTorque(uint8_t id, bool enable) {
  uint8_t b = enable ? 1 : 0;
  _sendPacket(id, LX_LOAD_WRITE, &b, 1);
}


// ── 0x20  LOAD_READ ──────────────────────────────────────────────────────────

bool LX15D::readTorque(uint8_t id, bool &enabled) {
  _sendPacket(id, LX_LOAD_READ);
  uint8_t n = _readResponse(LX_LOAD_READ);
  if (n < 1) return false;
  enabled = (_rxBuf[3] != 0);
  return true;
}


// ── 0x21  LED_CTRL_WRITE ─────────────────────────────────────────────────────

void LX15D::setLed(uint8_t id, bool on) {
  uint8_t b = on ? 0 : 1;  // protocol: 0=on, 1=off
  _sendPacket(id, LX_LED_CTRL_WRITE, &b, 1);
}


// ── 0x22  LED_CTRL_READ ──────────────────────────────────────────────────────

bool LX15D::readLed(uint8_t id, bool &on) {
  _sendPacket(id, LX_LED_CTRL_READ);
  uint8_t n = _readResponse(LX_LED_CTRL_READ);
  if (n < 1) return false;
  on = (_rxBuf[3] == 0);  // 0=on, 1=off
  return true;
}


// ── 0x23  LED_ERROR_WRITE ────────────────────────────────────────────────────

void LX15D::setLedAlarm(uint8_t id, uint8_t mask) {
  mask &= 0x07;  // only bits 0-2 are valid
  _sendPacket(id, LX_LED_ERROR_WRITE, &mask, 1);
}


// ── 0x24  LED_ERROR_READ ─────────────────────────────────────────────────────

bool LX15D::readLedAlarm(uint8_t id, uint8_t &mask) {
  _sendPacket(id, LX_LED_ERROR_READ);
  uint8_t n = _readResponse(LX_LED_ERROR_READ);
  if (n < 1) return false;
  mask = _rxBuf[3] & 0x07;
  return true;
}


// ── convenience: moveSync ─────────────────────────────────────────────────────

void LX15D::moveSync(LX15DMove *moves, uint8_t count, uint16_t durationMs, bool wait) {
  for (uint8_t i = 0; i < count; i++) {
    moveWait(moves[i].id, moves[i].position, durationMs);
  }
  moveStart(LX_BROADCAST);  // all servos depart simultaneously
  if (wait) {
    delay(durationMs);
  }
}


// ── convenience: getStatus ───────────────────────────────────────────────────

bool LX15D::getStatus(uint8_t id, LX15DStatus &s) {
  s.id = id;

  uint16_t mv = 0;
  s.voltage = readVoltage(id, mv) ? mv / 1000.0f : 0.0f;

  readTemperature(id, s.temperature);
  readTempLimit(id, s.tempLimit);

  int16_t pos = 0;
  readPosition(id, pos);
  s.position = pos;

  readMode(id, s.mode, s.motorSpeed);
  readTorque(id, s.torqueEnabled);
  readLed(id, s.ledOn);
  readLedAlarm(id, s.ledAlarmMask);

  int8_t off = 0;
  readAngleOffset(id, off);
  s.angleOffset = off;

  readAngleLimits(id, s.angleMin, s.angleMax);
  readVinLimits(id, s.vinMinMv, s.vinMaxMv);

  return true;
}


// ── convenience: scan ────────────────────────────────────────────────────────

uint8_t LX15D::scan(uint8_t *foundIds, uint8_t maxFound,
                    uint8_t startId, uint8_t endId) {
  uint8_t count = 0;
  for (uint8_t id = startId; id <= endId && count < maxFound; id++) {
    uint8_t result = 0;
    if (readId(id, result)) {
      foundIds[count++] = result;
    }
  }
  return count;
}
