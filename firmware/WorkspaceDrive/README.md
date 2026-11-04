# WorkspaceDrive firmware (I²C robot)

**Status: COMPILE-VERIFIED / HARDWARE-UNTESTED.** Spec: `docs/specs/workspace_drive.md`.

The professor's `ESP32RobotControl.ino` with a motor adapter (`motor_i2c.h`), a 500 ms longest
move, an optional `speed` and your own Wi-Fi name. Flash it only after Gate A of the Part 1
checklist has passed (power, cut-off, no spin at reset, 0x30 answers).

## Setup (once)
1. Arduino IDE 2, board package **esp32 3.3.12**, libraries **Async TCP 3.5.0** and
   **ESP Async WebServer 3.12.1** (already installed on this PC).
2. Copy into **this folder**, next to `WorkspaceDrive.ino`:
   - `MecanumCar_v2.cpp` and `MecanumCar_v2.h` from `Downloads\MecanumCar_v2.zip`;
   - `ld06.cpp`, `ld06.h` and `ld06crc.h` from `Downloads\WIFI ROBOT.zip` (`TP LIDAR\`).

   They stay out of Git (no licence information).
3. Copy `wifi_secrets.example.h` to `wifi_secrets.h` and set a unique `ROBOT_SSID` and a
   `ROBOT_PASSWORD` of 8 characters or more.

## Flash
Wheels in the air, hand on the battery switch. Board **ESP32 Dev Module**, port **COM7**,
Upload. The Serial Monitor (115200) prints `AP IP: 192.168.4.1`. The robot does not move at
boot.

## Routes (the same as the professor's sketch)
| Route | Effect |
|---|---|
| `/forward`, `/backward`, `/left`, `/right` `?time=ms&speed=0..255` | Move for `time` (max 500 ms), then stop |
| `/rotateleft`, `/rotateright` `?time=ms&speed=…` | Rotate for `time / 2` (as in the original) |
| `/stop` | Stop now |
| `/ping` | `OK` |
| `/scan`, `/pose` | Unchanged |

Defaults: `time` 300 ms, `speed` 80 (the library's value).

## Test after flashing (Gate B)
1. `http://192.168.4.1/ping` → `OK`.
2. `/forward?time=500`: the wheels turn about 0.5 s, then stop on their own.
3. Each other direction once.
4. `/forward?time=500` repeatedly while pressing EN: if any wheel keeps turning while EN is held
   or after the reboot, stop and use the battery switch.
