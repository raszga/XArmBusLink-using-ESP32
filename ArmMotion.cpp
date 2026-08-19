#include "ArmMotion.h"
#include "TrajectoryStore.h"  // for the global `point` written by movXY()

// ── shared instances — single source of truth for these globals ───────────
LX15D bus(Serial1, -1);
XArmKinematics kin;
XArmIK cmd;
XArmIKStatus stk;
XArmFK resultK;
LX15DStatus st;

LX15DMove grp[6] = {
  { SERVO_1, 300 },
  { SERVO_2, 300 },
  { SERVO_3, 300 },
  { SERVO_4, 300 },
  { SERVO_5, 300 },
  { SERVO_6, 300 }
};

int safeReadPos(uint8_t servoId) {
  if (!bus.getStatus(servoId, st)) {
    Serial.printf("safeReadPos: comms fail servo %d — defaulting to 500\n", servoId);
    return 500;
  }
  int pos = constrain(st.position, 0, 1000);
  if (pos != st.position) {
    Serial.printf("safeReadPos: servo %d clamped %d -> %d\n", servoId, st.position, pos);
  }
  return pos;
}

bool positionSafe(int pos) {
  return (pos >= 0 && pos <= 1000);
}

void movSpeedSync(LX15DMove grp[], uint8_t count, float speed_dps) {
  int maxDeltaUnits = 0;

  for (uint8_t i = 0; i < count; i++) {
    int current = safeReadPos(grp[i].id);
    int delta = abs((int)grp[i].position - current);
    if (delta > maxDeltaUnits) maxDeltaUnits = delta;
  }

  uint16_t time_ms;
  if (speed_dps <= 0.0f || maxDeltaUnits == 0) {
    time_ms = MOVSPEEDSYNC_MIN_MS;
  } else {
    float maxDeltaDeg = (float)maxDeltaUnits * DEG_PER_UNIT;
    float time_s = maxDeltaDeg / speed_dps;
    uint32_t ms = (uint32_t)(time_s * 1000.0f + 0.5f);
    if (ms < MOVSPEEDSYNC_MIN_MS) ms = MOVSPEEDSYNC_MIN_MS;
    if (ms > 65535UL) ms = 65535UL;  // bus.moveSync's time_ms is uint16_t
    time_ms = (uint16_t)ms;
  }

  bus.moveSync(grp, count, time_ms);
}

void movXY(float X, float Y) {
  // X,Y here are the J3 pivot target (library's native convention) — NOT
  // the TCP/gripper tip. ikAutoTCP() assumed these should match calcXY()'s
  // TCP output, but that assumption didn't hold in practice — ikAuto() is
  // the correct call here. Side and elbow are both chosen automatically.
  Serial.print("CP->");
  Serial.print(X);
  Serial.print("   ");
  Serial.println(Y);
  stk = kin.ikAuto(X, Y, cmd);
  if (stk == IK_OK || stk == IK_OK_CLAMPED) {

    for (int i = 0; i < 6; i++) {
      grp[i] = (LX15DMove){ i + 1, safeReadPos(i + 1) };
    }
    grp[2] = (LX15DMove){ 3, cmd.u3 };
    grp[3] = (LX15DMove){ 4, cmd.u4 };
    grp[4] = (LX15DMove){ 5, cmd.u5 };
    //bus.moveSync(grp, 6, 1000);
    movSpeedSync(grp, 6, 25);
  }
  point.angle[2] = cmd.u3;
  point.angle[3] = cmd.u4;
  point.angle[4] = cmd.u5;
  // (int), not (uint16_t): X/Y can be negative, and angle[] is int — casting
  // through uint16_t first reinterpreted negative values as large positive
  // ones (e.g. -82 -> 65454) before the int assignment.
  point.angle[6] = (int)X;
  point.angle[7] = (int)Y;
  delay(1000);
}
