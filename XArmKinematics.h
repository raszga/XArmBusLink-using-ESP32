// ============================================================================
//  XArmKinematics.h  —  Planar Forward / Inverse Kinematics for xArm
//
//  Covers joints J3 / J4 / J5 in the arm's vertical working plane.
//  J6 (swing about Y-axis) is handled separately — it maps the planar
//  solution into 3-D cylindrical coordinates; not implemented here yet.
//
//  COORDINATE SYSTEM (as marked on the mechanical drawing)
//  ─────────────────────────────────────────────────────────────────────────
//    Origin  :  ground level, arm base centre  (X=0, Y=0)
//    +X      :  to the right (as viewed facing the arm from the front)
//    +Y      :  upward  (Y<0 is a valid, physically-reachable "underground"
//               target — e.g. digging below the base plane)
//    All lengths in millimetres.
//
//    This single (X,Y) plane covers all 4 quadrants of a target J3 position:
//      Q1 : X>=0, Y>=0   Q2 : X<0, Y>=0
//      Q4 : X>=0, Y<0    Q3 : X<0, Y<0   (underground)
//
//  JOINT CONVENTION
//  ─────────────────────────────────────────────────────────────────────────
//    Servo units : 0 – 1000,  500 = neutral (straight-up pose)
//    Deg/unit    : 0.24  →  full travel = 0–240°, centre = 120° absolute
//    Joint angles θ are RELATIVE (standard serial-chain): θ4 rotates link
//    J4-J3 relative to link J5-J4's current orientation, not relative to
//    the global frame.
//
//    Sign conventions (phi increases = link bends toward +X):
//      J3 (S3, LX-35D ) : 0=left  1000=right  →  θ3 = +(u3-500)×0.24°
//      J4 (S4, LX-35D ) : 0=right 1000=left   →  θ4 = -(u4-500)×0.24°
//      J5 (S5, LX-225D) : 0=left  1000=right  →  θ5 = +(u5-500)×0.24°
//
//    All internal chain-angle math (phi5/phi4/phi3, theta3/4/5) is done in
//    radians and normalized into (-π, +π] before any limit check or unit
//    conversion, so results are consistent regardless of which quadrant the
//    target falls in — no ±180° wraparound discontinuities.
//
//  END-EFFECTOR DEFINITION
//  ─────────────────────────────────────────────────────────────────────────
//    The kinematic target point is the J3 pivot (not the gripper tip).
//    The "one solid" segment (J3→gripper, length L_J3_TCP) is kept as close
//    to HORIZONTAL as the J3 servo limit allows, pointing AHEAD of the
//    target (i.e. further away from the base centreline):
//      x_mm >= 0  →  solid points toward +X  (phi3_target = +90°)
//      x_mm <  0  →  solid points toward -X  (phi3_target = -90°)
//    This side selection is now AUTOMATIC in ikAuto() — derived from the
//    sign of the requested x_mm, not a caller-supplied guess. The manual
//    ik(..., side, elbow, ...) overload is kept for testing/debugging, but
//    ikAuto() is the recommended entry point for normal use.
//
//    θ3 is solved from this orientation constraint once θ4/θ5 have
//    positioned J3. If the required θ3 falls outside the J3 servo limit,
//    it is CLAMPED to the nearest limit (IK_OK_CLAMPED) rather than
//    rejecting the whole solution — the "one solid" ends up as close to
//    horizontal as the servo can physically get, still pointing ahead.
//
//  JOINT LIMITS
//  ─────────────────────────────────────────────────────────────────────────
//    J3, J4 : full servo range 0–1000 (±120° from neutral)
//    J5     : servo units 50–950 (±108° from neutral) — widened from the
//             original 100–900 (±96°) to enlarge the reachable envelope.
//
//  HOW TO USE
//  ─────────────────────────────────────────────────────────────────────────
//  1. Include this file and XArmKinematics.cpp in your Arduino sketch folder.
//  2. Instantiate: XArmKinematics kin;
//  3. Forward kinematics (servo positions → XY of every joint):
//       XArmFK result;
//       kin.fk(u3, u4, u5, result);
//       // result.j3, result.j4, result.j5, result.tcp : {x, y} in mm
//       // result.phi3_deg : absolute attitude of the "one solid"
//  4. Inverse kinematics — RECOMMENDED (auto side + auto elbow + soft clamp):
//       XArmIK cmd;
//       XArmSide sideUsed; XArmElbow elbowUsed;
//       XArmIKStatus st = kin.ikAuto(x_mm, y_mm, cmd, &elbowUsed, &sideUsed);
//       if (st == IK_OK || st == IK_OK_CLAMPED) {
//         bus.move(3, cmd.u3, 500);
//         bus.move(4, cmd.u4, 1000);
//         bus.move(5, cmd.u5, 2000);
//       }
//  5. Inverse kinematics — manual override (testing only):
//       XArmIKStatus st = kin.ik(x_mm, y_mm, SIDE_RIGHT, ELBOW_UP, cmd);
//  6. Adjust geometry by editing the PARAM block in XArmKinematics.cpp.
//     All lengths are named constants — change only there, not here.
// ============================================================================

#pragma once
#include <Arduino.h>
#include <math.h>

// ============================================================================
//  GEOMETRY / SERVO / LIMIT CONSTANTS — read-only from outside this library.
//
//  Defined (single source of truth) in XArmKinematics.cpp — edit values only
//  there. These extern declarations just expose them to the .ino and other
//  translation units that need them (e.g. speed-based sync-move timing).
// ============================================================================
extern const float L_J5_J4;     // J5 pivot -> J4 pivot (mm)
extern const float L_J4_J3;     // J4 pivot -> J3 pivot (mm)
extern const float L_J3_TCP;    // J3 pivot -> TCP tip (mm)
extern const float J5_HEIGHT;   // J5 pivot height above ground, Y=0 (mm)

extern const float DEG_PER_UNIT;      // 0.24 deg per servo unit
extern const float UNIT_PER_DEG;      // 1 / DEG_PER_UNIT
extern const uint16_t SERVO_CENTER;   // 500

extern const uint16_t U3_MIN, U3_MAX; //tilt
extern const uint16_t U4_MIN, U4_MAX; // stick
extern const uint16_t U5_MIN, U5_MAX; //hoist

extern const float THETA3_MIN, THETA3_MAX;
extern const float THETA4_MIN, THETA4_MAX;
extern const float THETA5_MIN, THETA5_MAX;

// ── Side: orientation of the "one solid" (J3-TCP) segment ────────────────────
enum XArmSide {
  SIDE_RIGHT = 0,   // solid points toward +X  (phi3_target = +90°)
  SIDE_LEFT  = 1    // solid points toward -X  (phi3_target = -90°)
};

// ── Elbow: which geometric solution branch to use for the J4/J5 sub-chain ────
enum XArmElbow {
  ELBOW_UP   = 0,   // J4 bends away from vertical (natural reach-out)
  ELBOW_DOWN = 1    // J4 bends toward vertical (folded configuration)
};

// ── Return status from ik() / ikAuto() ────────────────────────────────────────
enum XArmIKStatus {
  IK_OK              = 0,
  IK_ERR_UNREACHABLE = 1,  // target outside the J5-J4-J3 annular workspace
  IK_ERR_J3_LIMIT    = 2,  // (ik() manual overload only) theta3 violates J3 limit
  IK_ERR_J4_LIMIT    = 3,  // theta4 solution violates J4 servo limits
  IK_ERR_J5_LIMIT    = 4,  // theta5 solution violates J5 torque/balance limits
  IK_OK_CLAMPED      = 5   // solved OK, but theta3 was clamped to its limit —
                           // "one solid" is as horizontal as the servo allows
};

// ── XY position of one joint ──────────────────────────────────────────────────
struct XArmPoint {
  float x;  // mm, global frame
  float y;  // mm, global frame
};

// ── FK output: positions of all joints and the TCP ───────────────────────────
struct XArmFK {
  XArmPoint j5;       // J5 pivot  (fixed — depends only on geometry, not joint angles)
  XArmPoint j4;       // J4 pivot
  XArmPoint j3;       // J3 pivot  (kinematic target for IK)
  XArmPoint tcp;      // tip of the "one solid" (L_J3_TCP from J3)
  float phi3_deg;     // absolute attitude of J3-TCP segment from vertical (deg)
                      // ±90 when the horizontal constraint is fully satisfied;
                      // otherwise reflects wherever theta3 was clamped to.
};

// ── IK output: servo unit commands for J3/J4/J5 ──────────────────────────────
struct XArmIK {
  uint16_t u3;   // servo 3 (Stick)  command, 0-1000
  uint16_t u4;   // servo 4 (Tilt)   command, 0-1000
  uint16_t u5;   // servo 5 (Boom)   command, 0-1000
};


// ── Main kinematics class ─────────────────────────────────────────────────────
class XArmKinematics {
public:
  XArmKinematics();

  // ── Forward kinematics ──────────────────────────────────────────────────────
  // Input  : servo units u3/u4/u5 (0-1000)
  // Output : XArmFK struct populated with joint XY positions
  // Limits : NOT checked here — FK is a pure geometry function.
  //          Use to verify a pose after IK, not to validate a command.
  void fk(uint16_t u3, uint16_t u4, uint16_t u5, XArmFK &out) const;

  // ── Inverse kinematics — manual override (testing/debug) ─────────────────────
  // Input  : target J3 position (x_mm, y_mm) in global frame
  //          side   : SIDE_RIGHT or SIDE_LEFT (orientation of "one solid")
  //          elbow  : ELBOW_UP or ELBOW_DOWN (J4/J5 solution branch)
  // Output : XArmIK struct with u3/u4/u5 servo commands (0-1000)
  // Returns: IK_OK, IK_OK_CLAMPED (theta3 saturated to its limit — still a
  //          valid, commandable pose), or an error status.
  //          theta4/theta5 are hard limits — never clamped, since violating
  //          them means the J3 target itself is not physically reachable.
  //
  // NOTE: this overload does NOT auto-correct side/elbow for the target
  // quadrant — use ikAuto() unless you specifically need to force a branch.
  XArmIKStatus ik(float x_mm, float y_mm,
                  XArmSide side, XArmElbow elbow,
                  XArmIK &cmd) const;

  // ── Auto IK — RECOMMENDED entry point ─────────────────────────────────────────
  // Automatically selects:
  //   - side  : SIDE_RIGHT if x_mm >= 0, else SIDE_LEFT ("one solid" always
  //             points ahead of the target, away from the base centreline —
  //             this is what makes all 4 quadrants reachable without the
  //             caller having to guess).
  //   - elbow : min-|theta5| heuristic — if both elbow branches are
  //             commandable for the target, the one that keeps the Boom
  //             (theta5) closer to neutral is used; if only one branch is
  //             commandable, that one is used regardless of theta5.
  // theta3 soft-clamps per branch as in ik().
  // Returns IK_OK / IK_OK_CLAMPED if either branch produces a commandable
  // pose; otherwise returns the ELBOW_UP branch's error (unreachable or
  // hard-limit violation — both branches share the same reachability test).
  XArmIKStatus ikAuto(float x_mm, float y_mm,
                      XArmIK &cmd,
                      XArmElbow *elbowUsed = nullptr,
                      XArmSide *sideUsed = nullptr) const;

  // ── Auto IK — TCP (gripper tip) target ────────────────────────────────────────
  // Same auto side/elbow logic as ikAuto(), but the target (x_tcp_mm,
  // y_tcp_mm) is the TCP / gripper tip position, not the J3 pivot. Converts
  // internally to the equivalent J3 target assuming "one solid" ends up
  // horizontal; if theta3 is clamped, the actual TCP will be slightly off
  // from the requested point — check fk() on the result if exactness matters.
  XArmIKStatus ikAutoTCP(float x_tcp_mm, float y_tcp_mm,
                        XArmIK &cmd,
                        XArmElbow *elbowUsed = nullptr,
                        XArmSide *sideUsed = nullptr) const;

  // ── Unit conversions (public — useful for Serial debug prints) ──────────────
  static float u3ToTheta(uint16_t u);   // servo unit  → relative joint angle (deg)
  static float u4ToTheta(uint16_t u);
  static float u5ToTheta(uint16_t u);
  static uint16_t thetaToU3(float deg); // relative joint angle (deg) → servo unit
  static uint16_t thetaToU4(float deg);
  static uint16_t thetaToU5(float deg);

private:
  // Shared by ikAuto()/ikAutoTCP(): solves both elbow branches for a given
  // (x_mm, y_mm, side) and picks the best one (see .cpp for the rule).
  XArmIKStatus solveBestElbow(float x_mm, float y_mm, XArmSide side,
                              XArmIK &cmd, XArmElbow *elbowUsed) const;
};