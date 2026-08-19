#include "TrajectoryStore.h"
#include "ArmMotion.h"       // for kin (FK/IK) and safeReadPos()
#include "XArmKinematics.h"  // XArmIK / XArmFK / XArmIKStatus / XArmElbow / XArmSide
#include "TrajectoryPoints.h"  // hand-edited TRJ_XY[] / NUM_TRJ_XY

const uint16_t maxPct = 100;
angles TRJ[maxPct];
angles point;
int pct = 0;
Preferences prefs;

void calcXY(uint16_t u3, uint16_t u4, uint16_t u5) {
  XArmFK out;  // local now — was a global read only by this function
  kin.fk(u3, u4, u5, out);
  point.angle[2] = u3;
  point.angle[3] = u4;
  point.angle[4] = u5;
  point.angle[6] = (int)out.tcp.x;
  point.angle[7] = (int)out.tcp.y;
}

int loadTrajectory(angles trjArray[], int maxCount) {
  Serial.println("Loading...");
  if (!prefs.begin("arm_trj", true)) return 0;

  int storedCount = prefs.getInt("count", 0);
  if (storedCount <= 0) {
    prefs.end();
    return 0;
  }
  if (storedCount > maxCount) storedCount = maxCount;

  prefs.getBytes("array", (uint8_t *)trjArray, storedCount * sizeof(angles));
  prefs.end();
  return storedCount;
}

bool saveTrajectory(angles trjArray[], int count) {
  Serial.println("Saving...");
  if (!prefs.begin("arm_trj", false)) return false;

  prefs.remove("count");
  prefs.remove("array");
  prefs.putInt("count", count);
  size_t written = prefs.putBytes("array", (const uint8_t *)trjArray,
                                  count * sizeof(angles));
  prefs.end();
  return (written == count * sizeof(angles));
}

// ============================================================================
//  loadTrajectoryFromXY()
//  See TrajectoryStore.h for contract. Source data: TrajectoryPoints.h.
// ============================================================================
int loadTrajectoryFromXY() {
  int loaded = 0;

  Serial.printf("[loadTrajectoryFromXY] solving %d point(s) from TrajectoryPoints.h ...\n",
                NUM_TRJ_XY);

  for (int i = 0; i < NUM_TRJ_XY && loaded < (int)maxPct; i++) {
    float x = TRJ_XY[i].x;
    float y = TRJ_XY[i].y;

    XArmIK cmd;
    XArmElbow elbowUsed;
    XArmSide sideUsed;
    XArmIKStatus st = kin.ikAuto(x, y, cmd, &elbowUsed, &sideUsed);

    if (st != IK_OK && st != IK_OK_CLAMPED) {
      Serial.printf("[loadTrajectoryFromXY] point %d (%.2f, %.2f) SKIPPED — IK status %d\n",
                    i, x, y, (int)st);
      continue;
    }

    TRJ[loaded].angle[0] = 500;     // Clamp  — held neutral
    TRJ[loaded].angle[1] = 500;     // Rotate — held neutral
    TRJ[loaded].angle[2] = cmd.u3;  // Stick
    TRJ[loaded].angle[3] = cmd.u4;  // Tilt
    TRJ[loaded].angle[4] = cmd.u5;  // Boom
    TRJ[loaded].angle[5] = 500;     // Swing — held neutral

    XArmFK out;
    kin.fk(cmd.u3, cmd.u4, cmd.u5, out);
    TRJ[loaded].angle[6] = (int)out.tcp.x;
    TRJ[loaded].angle[7] = (int)out.tcp.y;

    if (st == IK_OK_CLAMPED) {
      Serial.printf("[loadTrajectoryFromXY] point %d (%.2f, %.2f) OK — theta3 clamped\n", i, x, y);
    }

    loaded++;
  }

  pct = loaded;
  Serial.printf("[loadTrajectoryFromXY] loaded %d / %d point(s) into TRJ (array had %d)\n",
                loaded, (int)maxPct, NUM_TRJ_XY);
  return loaded;
}

void captureTrajectoryPoint() {
  for (int i = 0; i < 6; i++) {
    TRJ[pct].angle[i] = safeReadPos(i + 1);
  }
  calcXY(TRJ[pct].angle[2], TRJ[pct].angle[3], TRJ[pct].angle[4]);
  TRJ[pct].angle[6] = point.angle[6];
  TRJ[pct].angle[7] = point.angle[7];

  Serial.print(String(pct) + "->");
  for (int i = 0; i < 8; i++) {
    Serial.print(String(TRJ[pct].angle[i]) + ",");
  }
  pct++;
  Serial.println();
  delay(1000);
}

void CalculateTrajectoryPoint() {

  for (int i = 0; i < 6; i++) {
    TRJ[pct].angle[i] = safeReadPos(i + 1);
  }
  calcXY(TRJ[pct].angle[2], TRJ[pct].angle[3], TRJ[pct].angle[4]);
  TRJ[pct].angle[6] = point.angle[6];
  TRJ[pct].angle[7] = point.angle[7];
  Serial.print(" Calc>");
  for (int i = 0; i < 8; i++) {
    Serial.print(String(TRJ[pct].angle[i]) + ",");
  }
  
  Serial.println();
}
