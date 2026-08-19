// ============================================================================
//  ArmMotion.h  —  Servo bus helpers + kinematics-driven motion commands
//
//  Owns the LX15D bus instance and the XArmKinematics instance used to turn
//  (X,Y) targets into servo commands. Everything here talks to hardware
//  (via `bus`) or to the kinematics library (via `kin`) — no trajectory
//  storage concerns live in this module (see TrajectoryStore.h/.cpp).
// ============================================================================
#pragma once
#include <Arduino.h>
#include "LX15D.h"
#include "XArmKinematics.h"

// ── servo IDs (moved from XArmBusLink.ino — only used to build grp[]) ───────
#define SERVO_1 1
#define SERVO_2 2
#define SERVO_3 3
#define SERVO_4 4
#define SERVO_5 5
#define SERVO_6 6

#define MOVSPEEDSYNC_MIN_MS 500  // floor — avoids a 0 ms / near-instant snap

// ── shared bus / kinematics instances (defined in ArmMotion.cpp) ───────────
extern LX15D bus;
extern XArmKinematics kin;
extern XArmIK cmd;
extern XArmIKStatus stk;
extern XArmFK resultK;        // currently unused elsewhere — kept as global,
                               // moved verbatim to avoid any behavior change
extern LX15DStatus st;
extern LX15DMove grp[6];

// ============================================================================
//  safeReadPos()
//  Read servo position with fallback to neutral on comms failure.
//  Clamps result to 0-1000 — pot wiper can go outside valid range when
//  torque is off and a joint is backdriven past its physical travel limits.
// ============================================================================
int safeReadPos(uint8_t servoId);

// Returns true if position is within safe inner margin (avoids endstop grind
// under full PID torque when re-engaging servo mode after motor/relax mode).
bool positionSafe(int pos);

// ============================================================================
//  movSpeedSync()
//
//  Constant-speed synchronized move: every servo in grp[] arrives at its
//  target at the same time, timed so the servo with the LARGEST angular
//  travel moves at speed_dps (deg/sec); every other servo in the group
//  moves proportionally slower to arrive together.
//
//  grp[i].position must already hold the TARGET position for each servo —
//  same convention as a direct bus.moveSync(grp, count, time_ms) call.
//  Current position is read internally via safeReadPos(grp[i].id).
//
//  speed_dps <= 0 is treated as "as fast as possible" and falls back to
//  MOVSPEEDSYNC_MIN_MS, since a zero/negative speed has no physical meaning.
// ============================================================================
void movSpeedSync(LX15DMove grp[], uint8_t count, float speed_dps);

// ============================================================================
//  movXY()
//  Drives servos 3/4/5 to reach J3-pivot target (X, Y) via kin.ikAuto().
//  Side and elbow are both chosen automatically by the library.
//  Also stamps the resulting command + requested X/Y into the global
//  `point` (defined in TrajectoryStore.h/.cpp).
// ============================================================================
void movXY(float X, float Y);
