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
