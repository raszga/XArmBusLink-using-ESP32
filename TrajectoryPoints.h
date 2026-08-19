// ============================================================================
//  TrajectoryPoints.h  —  hand-edited (x,y) J3-pivot target list, mm.
//
//  This is the "intermediate solution" fork: no runtime file/Serial transfer.
//  Edit the TRJ_XY[] array below, then reflash the ESP32.
//
//  Each entry is a J3-pivot target (x_mm, y_mm) in the arm's global frame
//  (origin at ground/base centre, +X right, +Y up — see XArmKinematics.h).
//  It is consumed ONCE at boot by loadTrajectoryFromXY() (TrajectoryStore.cpp),
//  which:
//    - solves each point via kin.ikAuto() (auto side + auto elbow),
//    - holds servos 1 (Clamp), 2 (Rotate), 6 (Swing) at neutral (500),
//    - writes servos 3/4/5 (Stick/Tilt/Boom) from the IK result,
//    - SKIPS (does not abort on) any point IK reports as unreachable,
//    - overwrites TRJ[]/pct — capped at maxPct (100) regardless of how many
//      points are listed here.
//
//  After boot, use the existing menuState 0b101 (Save trajectory to flash)
//  if you want this list persisted — that step stays manual.
// ============================================================================
#pragma once

struct XYPoint {
  float x;
  float y;
};

// ── Edit this list — one (x_mm, y_mm) J3-pivot target per line ─────────────
const XYPoint TRJ_XY[] = {
  { 50.0f, 50.0f },
  { 50.0f, 100.0f },
  { -50.0f, 100.0f },
  { 0.0f, 250.0f },
};

const int NUM_TRJ_XY = sizeof(TRJ_XY) / sizeof(TRJ_XY[0]);
