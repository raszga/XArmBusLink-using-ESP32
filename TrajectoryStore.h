// ============================================================================
//  TrajectoryStore.h  —  Trajectory point storage, FK bookkeeping, and
//  flash (Preferences) persistence.
//
//  `point` and `TRJ[]` are the shared trajectory buffers used across
//  XArmBusLink.ino (recTRJ, motorJogTRJ, loop) and ArmMotion.cpp (movXY
//  writes the last commanded pose into `point`).
// ============================================================================
#pragma once
#include <Arduino.h>
#include <Preferences.h>

struct angles {
  int angle[8] = { -1, -1, -1, -1, -1, -1, 0, 0 };
  /*
  Servo 1 to Servo 6, then X,Y calculated (TCP)
  */
};

extern const uint16_t maxPct;
extern angles TRJ[];     // sized [maxPct] — defined in TrajectoryStore.cpp
extern angles point;
extern int pct;
extern Preferences prefs;

// ============================================================================
//  calcXY()
//  Runs forward kinematics for (u3,u4,u5) and stamps servo units + resulting
//  TCP (x,y) into the global `point`. FK scratch result (`out`) is now local
//  to this function — nothing outside calcXY() ever read it as a global.
// ============================================================================
void calcXY(uint16_t u3, uint16_t u4, uint16_t u5);

// ============================================================================
//  loadTrajectory() / saveTrajectory()
// ============================================================================
int loadTrajectory(angles trjArray[], int maxCount);
bool saveTrajectory(angles trjArray[], int count);

// ============================================================================
//  loadTrajectoryFromXY()
//  Loads/overwrites TRJ[] and pct from the compile-time (x,y) J3-pivot
//  target list in TrajectoryPoints.h. Each point is solved via kin.ikAuto();
//  servos 1/2/6 (Clamp/Rotate/Swing) are held at neutral (500). Points IK
//  reports unreachable are SKIPPED (logged to Serial), not aborted. Capped
//  at maxPct (100) regardless of how many points are listed in the header.
//  Returns the number of points actually loaded.
// ============================================================================
int loadTrajectoryFromXY();

// ============================================================================
//  captureTrajectoryPoint()
//  Read all 6 servo positions into TRJ[pct] and advance pct.
//  Uses safeReadPos() — clamps out-of-range values from relaxed/backdriven joints.
// ============================================================================
void captureTrajectoryPoint();

// Same as captureTrajectoryPoint() but does NOT advance pct and has no delay —
// used for live display during manual-learn jogging. Kept as a separate
// function (not merged) per explicit instruction — no behavior change.
void CalculateTrajectoryPoint();
