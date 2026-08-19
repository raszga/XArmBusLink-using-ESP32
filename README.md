# XArmBusLink-using-ESP32
Repository for driving the LewanSoul Xarm robotic arm developped with Claude IA
# XArmBusLink

ESP32 firmware for controlling a LewanSoul/Hiwonder LX-15D bus-servo 6-DOF robotic arm over a BusLinker board.

## Overview

XArmBusLink drives a 6-servo arm using inverse kinematics for XY/TCP-frame targeting, synchronized speed-based moves, and flash-persisted trajectories.

## Hardware

- ESP32 (Serial1/Serial2 used for half-duplex bus servo comms)
- Hiwonder BusLinker V3.0 board
- LewanSoul/Hiwonder LX-15D bus servos (x6)
- Custom half-duplex transceiver (74HC04 + 74LS126) for direct servo bus access

## Firmware Structure

- `XArmKinematics` — forward/inverse kinematics (`ikAuto()`, `ikAutoTCP()`), geometry constants
- `ArmMotion` — synchronized servo motion (`movSpeedSync()`)
- `TrajectoryStore` — trajectory point storage/playback with flash persistence (Preferences library)
- `LX15D.h/.cpp` — Arduino C++ driver for LX-15D bus servos
- `lx15d_lib.py` — Python interactive library for the same servo protocol
- Slim `.ino` — top-level sketch tying the modules together

## Key Conventions

- Servo scale: 0.24°/unit, center = 500
- Joint sign conventions: J3 = +(u−500)×0.24°, J4 = −(u−500)×0.24°, J5 = +(u−500)×0.24°
- Core geometry: `L_J5_J4 = 101mm`, `L_J4_J3 = 95mm`, `J5_HEIGHT = 65mm`
- J5 servo range widened to 50–950 (±108°)
- IK sign conventions validated against 17 recorded physical poses
- Trajectory array (`TRJ[]`) capped at 100 points

## Status

Recent work computed the reachable J3-pivot workspace envelope and flagged a non-monotonic Xmin boundary that may affect path planning. Firmware testing in progress.

## License

See [LICENSE](LICENSE).

// ============================================================================
//  XArmBusLink_7_K_15.ino  —  xArm trajectory recorder / player on ESP32
//
//  Wiring (BusLinker V3.0, auto-direction — no external DIR pin needed):
//    ESP32 GPIO16  → RX
//    ESP32 GPIO17  → TX
//    Servo bus     → half-duplex single-wire S line
//    Servo power   → 6.0 – 7.4 V  (separate supply, NOT from USB!)
//
//  Button map:
//    Toggle (keep state) : GPIO 25 (b2), 13 (b1), 19 (b0)  → menuState bits
//    Simple pull-up      : GPIO 5  (pause/guard — hold LOW to run a menu action)
//                          GPIO 23 (enter manual-learn mode, pot mode only)
//                          GPIO 18 (exit manual-learn mode — pressed=LOW)
//                          GPIO 27 (capture TRJ point / exit pot-jog loop)
//                          GPIO 26 (neutral-menu modifier: relax vs erase-TRJ)
//
//  menuState encoding  (25<<2 | 13<<1 | 19), active only while GPIO5 is LOW:
//    0b000 = 0  → Fine-adjust Clamp/Rotate/Stick (servos 1-3) only — see
//                 adjustFine(). Servos 4-6 are never commanded, so they hold
//                 whatever position they're physically in. GPIO27 (or the
//                 toggle switches changing state) captures + appends a TRJ point.
//    0b001 = 1  → Manual XY jog over Serial ("X*Y\n" sets target; movXY() drives it)
//    0b010 = 2  → Load trajectory from compile-time XY array (TrajectoryPoints.h),
//                 convert via IK, and play once
//    0b011 = 3  → Record trajectory — recTRJ():
//                   GPIO23 LOW → manual-learn (torque off, move by hand,
//                     GPIO27 captures a point, GPIO18 exits)
//                   GPIO23 HIGH → pot-control jog (6 pots → 6 servos live),
//                     GPIO27 exits the jog loop and captures the point
//    0b101 = 5  → Save current TRJ buffer to flash (requires pct > 0)
//    0b110 = 6  → Load TRJ from flash and loop-play continuously
//                 (stays in this mode until the toggle bits change)
//    0b111 = 7  → Neutral menu:
//                   GPIO26 HIGH (not pressed) → move to neutral (500) + relax torque
//                   GPIO26 LOW  (pressed)     → erase TRJ: overwrite flash with an
//                     all-500 trajectory (joint 0 kept at 600)
//
//  Serial commands (readSerXY(), polled during manual XY jog):
//    "X*Y\n"  — set serX/serY target coordinates, e.g. "120.5*80.0"
//    "s" / "S" (no '*' in the line) — capture the current pose as a TRJ point
//
//  Module layout:
//    XArmKinematics.h/.cpp  — pure planar FK/IK math (no hardware deps)
//    ArmMotion.h/.cpp       — servo bus + kinematics-driven motion helpers
//    TrajectoryStore.h/.cpp — trajectory buffer, FK bookkeeping, flash I/O
//    TrajectoryPoints.h     — compile-time XY waypoint array for menuState 0b010
//    (this file)            — pin/button config, mode state machine only
// ============================================================================
