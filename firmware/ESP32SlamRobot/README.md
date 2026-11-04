# ESP32SlamRobot firmware (approach B, MVP)

Draft, **never run on the robot**. Source: the user's `ESP32SlamRobot.ino` (SHA-256 3f675d9b…
in Downloads), built from the course sketch `ESP32RobotControl.ino`. Decision:
`docs/decisions/2026-11-04_mvp_http_firmware.md`.

## Changes made in this repository (S0, 2026-11-04)
1. A task watchdog of 1 s on the motor task and `loop()` (`TASK_WDT_MS`). A hung task reboots
   the ESP32, and `setup()` stops the motors first.
2. `enum MoveResult` moved above the first function: the Arduino builder's generated
   prototypes used it before its declaration, so the draft did not compile.

## Build (compile only)
Needs, outside this repository:
- `ld06.h`, `ld06.cpp`, `ld06crc.h` from `LD06.zip` (`LD06/src/`), copied into this folder.
  `.gitignore` keeps them out of Git (licence unconfirmed).
- Libraries: `MecanumCar_v2` (`MecanumCar_v2.zip`), `Async TCP` 3.5.0 and `ESP Async
  WebServer` 3.12.1 (ESP32Async), for ESP32 core 3.3.12, FQBN `esp32:esp32:esp32`.

Before flashing, change `ROBOT_SSID` and `ROBOT_PASSWORD`.

S0 result: compiles; 947,196 bytes of flash (72 %), 93,952 bytes of RAM (28 %); no sketch
warnings with `--warnings all`.

## S1 bench checklist (you flash; wheels off the ground; hand on the power switch)
Read `/status` throughout. Each step must stop the wheels:
1. Boot: the wheels stay still; `/status` shows `armed:0`.
2. `/arm`, then `/drive?vx=30&time=500&seq=1`: the wheels turn about 0.5 s, then stop alone.
3. A longer `/drive` (time=1000), then `/stop` mid-move: they stop at once; `armed:0`.
4. A `/drive` with the laptop's Wi-Fi turned off right after: they stop within 1 s.
5. Unplug the LiDAR (or stop it) during a move: they stop within 0.5 s; `/drive` answers `NO_LIDAR`.
6. **Press the ESP32 EN (reset) button during a move.** Do the wheels stop, and when? If they keep
   turning until the ESP32 has rebooted, note how long. If they keep turning after the reboot, or
   with the ESP32 held in reset, **stop: a hardware cut-off is needed before any floor test**.
7. Check each wheel's direction for `vx`, `vy` and `wz` against `mixWheels()`.
