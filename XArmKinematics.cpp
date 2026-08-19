// ============================================================================
//  XArmKinematics.cpp
//  See XArmKinematics.h for full description, coordinate system, and usage.
// ============================================================================

#include "XArmKinematics.h"

// Set to 1 to print each ik() solve to Serial. Off by default — a library
// function should not spam Serial on every call in normal operation.
#ifndef XARM_KIN_DEBUG
#define XARM_KIN_DEBUG 0
#endif

// ============================================================================
//  GEOMETRY PARAMETERS — adjust here when the physical build is measured.
//  All dimensions in millimetres, measured from the mechanical drawing
//  dated 2026-06-30 (XArmDims, sheet 1/1).
// ============================================================================
// NOTE: these are declared `extern` in XArmKinematics.h so the .ino (and any
// other translation unit) can read them directly. This .cpp remains the only
// place values are assigned — edit here, nowhere else.
const float L_J5_J4 = 101.00f;//96.4f;     // J5 pivot → J4 pivot
const float L_J4_J3 = 95.00f;//95.71f;    // J4 pivot → J3 pivot
const float L_J3_TCP = 172.7f;   // J3 pivot → TCP tip (427 - 254.73)
                                 // "one solid" segment — horizontal when unclamped
const float J5_HEIGHT = 65.00f;//62.61f;  // J5 pivot height above ground (Y=0)

// ── Servo scale ──────────────────────────────────────────────────────────────
const float DEG_PER_UNIT = 240.0f / 1000.0f;  // 0.24 deg per servo unit
const float UNIT_PER_DEG = 1.0f / DEG_PER_UNIT;
const uint16_t SERVO_CENTER = 500;

// ── Joint limits (servo units, inclusive) ────────────────────────────────────
// J3, J4 : full firmware/design range
const uint16_t U3_MIN = 50, U3_MAX = 950;
const uint16_t U4_MIN = 0, U4_MAX = 1000;
// J5     : widened to 50-950 (was 100-900) to enlarge the reachable envelope
const uint16_t U5_MIN = 50, U5_MAX = 950;

// ── Derived theta limits (deg) — computed from servo unit limits ──────────────
// J3: theta = +(u-500)*0.24  →  u=0 → theta=-120, u=1000 → theta=+120
const float THETA3_MIN = -120.0f;
const float THETA3_MAX = 120.0f;
// J4: theta = -(u-500)*0.24  →  u=0 → theta=+120, u=1000 → theta=-120
const float THETA4_MIN = -120.0f;
const float THETA4_MAX = 120.0f;
// J5: theta = +(u-500)*0.24
//   u=50  → theta = (50-500)*0.24  = -108
//   u=950 → theta = (950-500)*0.24 = +108
const float THETA5_MIN = -108.0f;
const float THETA5_MAX = 108.0f;

// ── Small geometry tolerance ─────────────────────────────────────────────────
// Guards floating-point rounding at the exact boundary of the reachable annulus
// so acos() never receives a value marginally outside [-1, 1].
static const float FP_GUARD = 1e-5f;

// ── Angle wrap helper ─────────────────────────────────────────────────────────
// Normalizes any radian angle into (-π, +π]. Used on every chain angle
// (phi5, phi4, theta3) before limit-checking or unit conversion, so a target
// in any of the 4 quadrants produces a consistent, repeatable result instead
// of an unwrapped angle occasionally landing outside the expected range.
static float wrapPi(float a) {
  while (a > (float)M_PI)  a -= 2.0f * (float)M_PI;
  while (a <= -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

// ============================================================================
//  Constructor
// ============================================================================
XArmKinematics::XArmKinematics() {}

// ============================================================================
//  Unit conversions
// ============================================================================

// J3 (S3): 0=left, 1000=right  →  phi increases (rightward) as u increases
//          theta3 = +(u3-500)*0.24
float XArmKinematics::u3ToTheta(uint16_t u) {
  return (float)(u - SERVO_CENTER) * DEG_PER_UNIT;
}

// J4 (S4): 0=right, 1000=left  →  phi decreases (leftward) as u increases
//          theta4 = -(u4-500)*0.24
float XArmKinematics::u4ToTheta(uint16_t u) {
  return -(float)(u - SERVO_CENTER) * DEG_PER_UNIT;
}

// J5 (S5): 0=left, 1000=right  →  phi increases (rightward) as u increases
//          theta5 = +(u5-500)*0.24
float XArmKinematics::u5ToTheta(uint16_t u) {
  return (float)(u - SERVO_CENTER) * DEG_PER_UNIT;
}

// Inverse conversions — signs exactly mirror the forward conversions above.
uint16_t XArmKinematics::thetaToU3(float deg) {
  float u = SERVO_CENTER + deg * UNIT_PER_DEG;
  return (uint16_t)constrain((int)roundf(u), 0, 1000);
}
uint16_t XArmKinematics::thetaToU4(float deg) {
  float u = SERVO_CENTER - deg * UNIT_PER_DEG;
  return (uint16_t)constrain((int)roundf(u), 0, 1000);
}
uint16_t XArmKinematics::thetaToU5(float deg) {
  float u = SERVO_CENTER + deg * UNIT_PER_DEG;
  return (uint16_t)constrain((int)roundf(u), 0, 1000);
}

// ============================================================================
//  Forward Kinematics
//
//  All three joint angles are RELATIVE (serial-chain convention):
//    phi5 = theta5                     (absolute link angle from vertical)
//    phi4 = phi5 + theta4
//    phi3 = phi4 + theta3
//
//  Positions in global frame (origin at ground/base centre):
//    J5  = (0, J5_HEIGHT)              — fixed pivot, always at this XY
//    J4  = J5 + L_J5_J4 * (sin phi5, cos phi5)
//    J3  = J4 + L_J4_J3 * (sin phi4, cos phi4)
//    TCP = J3 + L_J3_TCP * (sin phi3, cos phi3)
//
//  phi3 = +90° when "one solid" points right (+X),
//         -90° when it points left (−X).
//  At the neutral pose (all servos 500) phi5=phi4=phi3=0 → arm straight up.
// ============================================================================
void XArmKinematics::fk(uint16_t u3, uint16_t u4, uint16_t u5, XArmFK &out) const {

  float t3 = u3ToTheta(u3) * (float)(M_PI / 180.0);
  float t4 = u4ToTheta(u4) * (float)(M_PI / 180.0);
  float t5 = u5ToTheta(u5) * (float)(M_PI / 180.0);

  // Absolute link attitudes (from vertical, measured CCW positive)
  float phi5 = t5;
  float phi4 = wrapPi(phi5 + t4);
  float phi3 = wrapPi(phi4 + t3);

  out.j5.x = 0.0f;
  out.j5.y = J5_HEIGHT;

  out.j4.x = out.j5.x + L_J5_J4 * sinf(phi5);
  out.j4.y = out.j5.y + L_J5_J4 * cosf(phi5);

  out.j3.x = out.j4.x + L_J4_J3 * sinf(phi4);
  out.j3.y = out.j4.y + L_J4_J3 * cosf(phi4);

  out.tcp.x = out.j3.x ;//+ L_J3_TCP * sinf(phi3);
  out.tcp.y = out.j3.y ;//+ L_J3_TCP * cosf(phi3);

  out.phi3_deg = phi3 * (float)(180.0 / M_PI);
  
}

// ============================================================================
//  Inverse Kinematics — manual override (side/elbow forced by caller)
//
//  Target: J3 pivot position (x_mm, y_mm) in the global frame. Any quadrant
//  (X and Y each independently positive or negative) is a valid target as
//  long as it lies within the J5-J4-J3 reachable annulus.
//
//  Step 1 — 2-link sub-chain (J5→J4→J3):
//    d² = (x - 0)² + (y - J5_HEIGHT)²
//    cos θ4 = (d² - L_J5_J4² - L_J4_J3²) / (2·L_J5_J4·L_J4_J3)
//    θ4 = ±acos(cos θ4)       (+ = ELBOW_UP,  − = ELBOW_DOWN)
//    α  = atan2(x, y - J5_HEIGHT)           (angle from +Y to J3 from J5;
//                                             atan2's own range already
//                                             covers all 4 quadrants)
//    β  = atan2(L_J4_J3·sin θ4, L_J5_J4 + L_J4_J3·cos θ4)
//    φ5 = α − β
//    θ5 = φ5
//    φ4 = φ5 + θ4
//
//  Step 2 — orientation lock ("one solid" as horizontal as the servo allows):
//    φ3_target = +π/2  (SIDE_RIGHT)  or  −π/2  (SIDE_LEFT)
//    θ3 = φ3_target − φ4   (wrapped into (-π,+π])
//    If θ3 exceeds the J3 servo limit, CLAMP it to the limit (IK_OK_CLAMPED)
//    instead of rejecting the solution — the "one solid" ends up as close
//    to horizontal as physically possible rather than the target being
//    denied outright.
//
//  Step 3 — convert to servo units; theta4/theta5 remain HARD limits, since
//  violating them means the J3 target itself is not reachable by this chain.
// ============================================================================
XArmIKStatus XArmKinematics::ik(float x_mm, float y_mm,
                                XArmSide side, XArmElbow elbow,
                                XArmIK &cmd) const {

  float phi3_target = (side == SIDE_RIGHT) ? (float)(M_PI / 2.0)
                                           : -(float)(M_PI / 2.0);

  // ── Step 1: check geometric reachability ─────────────────────────────────
  float dx = x_mm;
  float dy = y_mm - J5_HEIGHT;
  float d2 = dx * dx + dy * dy;
  float d = sqrtf(d2);

  float reach_max = L_J5_J4 + L_J4_J3;
  float reach_min = fabsf(L_J5_J4 - L_J4_J3);

  if (d > reach_max || d < reach_min) {
    return IK_ERR_UNREACHABLE;
  }

  // ── Step 2: solve θ4 (two branches) ─────────────────────────────────────
  float cos_t4 = (d2 - L_J5_J4 * L_J5_J4 - L_J4_J3 * L_J4_J3)
                 / (2.0f * L_J5_J4 * L_J4_J3);

  // Clamp to [-1,1] to protect acos() from fp rounding at workspace boundary
  cos_t4 = constrain(cos_t4, -1.0f + FP_GUARD, 1.0f - FP_GUARD);

  float t4_mag = acosf(cos_t4);
  float t4 = (elbow == ELBOW_UP) ? t4_mag : -t4_mag;

  // ── Step 3: solve θ5 ─────────────────────────────────────────────────────
  float alpha = atan2f(dx, dy);
  float beta = atan2f(L_J4_J3 * sinf(t4), L_J5_J4 + L_J4_J3 * cosf(t4));
  float phi5 = wrapPi(alpha - beta);
  float t5 = phi5;

  // ── Step 4: solve θ3 from orientation lock, with soft clamp ──────────────
  float phi4 = wrapPi(phi5 + t4);
  float t3 = wrapPi(phi3_target - phi4);

  // ── Step 5: convert to degrees ────────────────────────────────────────────
  float t3_deg = t3 * (float)(180.0 / M_PI);
  float t4_deg = t4 * (float)(180.0 / M_PI);
  float t5_deg = t5 * (float)(180.0 / M_PI);

  bool t3_clamped = false;
  if (t3_deg < THETA3_MIN) { t3_deg = THETA3_MIN; t3_clamped = true; }
  else if (t3_deg > THETA3_MAX) { t3_deg = THETA3_MAX; t3_clamped = true; }

  // ── Step 6: hard limit checks — theta4/theta5 only ────────────────────────
  // (theta3 was already resolved by clamping above, so it never errors here.)
  if (t4_deg < THETA4_MIN || t4_deg > THETA4_MAX) return IK_ERR_J4_LIMIT;
  if (t5_deg < THETA5_MIN || t5_deg > THETA5_MAX) return IK_ERR_J5_LIMIT;

  // ── Step 7: convert to servo units ────────────────────────────────────────
  cmd.u3 = thetaToU3(t3_deg);
  cmd.u4 = thetaToU4(t4_deg);
  cmd.u5 = thetaToU5(t5_deg);

  // Second theta5 check: servo unit must also lie within the narrower J5
  // unit limits (thetaToU5() clamps to [0,1000], but doesn't know U5_MIN/MAX).
  if (cmd.u5 < U5_MIN || cmd.u5 > U5_MAX) return IK_ERR_J5_LIMIT;

#if XARM_KIN_DEBUG
  Serial.println("----------------");
  Serial.print("target x,y: "); Serial.print(x_mm); Serial.print(", "); Serial.println(y_mm);
  Serial.print("u3,u4,u5: "); Serial.print(cmd.u3); Serial.print(", ");
  Serial.print(cmd.u4); Serial.print(", "); Serial.println(cmd.u5);
  if (t3_clamped) Serial.println("theta3 CLAMPED");
#endif

  return t3_clamped ? IK_OK_CLAMPED : IK_OK;
}

// ============================================================================
//  solveBestElbow — shared by ikAuto() and ikAutoTCP()
//
//  Solves both elbow branches for a given (x_mm, y_mm, side) and picks the
//  one to use:
//    - If only one branch is commandable (OK or OK_CLAMPED), use it.
//    - If both are commandable, pick whichever gives the SMALLER |theta5|
//      (Boom stays closer to neutral) — the "natural reach" heuristic,
//      validated against manually-taught poses across the full envelope.
//    - If neither is commandable, report the ELBOW_UP branch's status
//      (UNREACHABLE is geometry-only and shared by both branches anyway).
// ============================================================================
XArmIKStatus XArmKinematics::solveBestElbow(float x_mm, float y_mm,
                                            XArmSide side,
                                            XArmIK &cmd,
                                            XArmElbow *elbowUsed) const {
  XArmIK cmdUp, cmdDown;
  XArmIKStatus stUp   = ik(x_mm, y_mm, side, ELBOW_UP,   cmdUp);
  XArmIKStatus stDown = ik(x_mm, y_mm, side, ELBOW_DOWN, cmdDown);

  bool okUp   = (stUp   == IK_OK || stUp   == IK_OK_CLAMPED);
  bool okDown = (stDown == IK_OK || stDown == IK_OK_CLAMPED);

  if (okUp && !okDown) {
    cmd = cmdUp;
    if (elbowUsed) *elbowUsed = ELBOW_UP;
    return stUp;
  }
  if (okDown && !okUp) {
    cmd = cmdDown;
    if (elbowUsed) *elbowUsed = ELBOW_DOWN;
    return stDown;
  }
  if (!okUp && !okDown) {
    if (elbowUsed) *elbowUsed = ELBOW_UP;
    return stUp;
  }

  // Both branches commandable — pick the smaller |theta5|.
  float t5Up   = fabsf(u5ToTheta(cmdUp.u5));
  float t5Down = fabsf(u5ToTheta(cmdDown.u5));
  if (t5Up <= t5Down) {
    cmd = cmdUp;
    if (elbowUsed) *elbowUsed = ELBOW_UP;
    return stUp;
  }
  cmd = cmdDown;
  if (elbowUsed) *elbowUsed = ELBOW_DOWN;
  return stDown;
}

// ============================================================================
//  ikAuto — target is the J3 pivot. Auto side (from target quadrant) + auto
//  elbow (min-|theta5| heuristic, see solveBestElbow).
// ============================================================================
XArmIKStatus XArmKinematics::ikAuto(float x_mm, float y_mm,
                                    XArmIK &cmd,
                                    XArmElbow *elbowUsed,
                                    XArmSide *sideUsed) const {

  // "One solid" points ahead of the target — away from the base centreline —
  // so it never has to fold back across the vertical axis to satisfy the
  // horizontal constraint. This is what makes all 4 quadrants reachable
  // without the caller having to guess a side.
  XArmSide side = (x_mm >= 0.0f) ? SIDE_RIGHT : SIDE_LEFT;
  if (sideUsed) *sideUsed = side;
  return solveBestElbow(x_mm, y_mm, side, cmd, elbowUsed);
}

// ============================================================================
//  ikAutoTCP — target is the TCP (gripper tip), NOT the J3 pivot.
//
//  Converts the TCP target into the equivalent J3 target and solves from
//  there. Valid only for the unclamped case, where phi3 = +-90 deg exactly
//  (i.e. "one solid" horizontal): TCP = J3 + (L_J3_TCP * sign, 0), so
//    J3.x = TCP.x - sign * L_J3_TCP
//    J3.y = TCP.y
//  Side is chosen from the TCP target's own sign (not the derived J3's),
//  since that's the point the caller actually asked for.
//
//  If theta3 ends up clamped (IK_OK_CLAMPED), the "one solid" is no longer
//  exactly horizontal, so the actual TCP will differ slightly from the
//  requested (x_tcp_mm, y_tcp_mm) — call fk() on the returned cmd if you
//  need the exact resulting TCP position.
// ============================================================================
XArmIKStatus XArmKinematics::ikAutoTCP(float x_tcp_mm, float y_tcp_mm,
                                       XArmIK &cmd,
                                       XArmElbow *elbowUsed,
                                       XArmSide *sideUsed) const {
  XArmSide side = (x_tcp_mm >= 0.0f) ? SIDE_RIGHT : SIDE_LEFT;
  if (sideUsed) *sideUsed = side;

  float sign = (side == SIDE_RIGHT) ? 1.0f : -1.0f;
  float x_j3 = x_tcp_mm - sign * L_J3_TCP;
  float y_j3 = y_tcp_mm;

  return solveBestElbow(x_j3, y_j3, side, cmd, elbowUsed);
}