# ESP32SlamRobot: Arduino upload package

**Status: COMPILE-VERIFIED / HARDWARE-UNTESTED, DIAGNOSTIC BUILD (`MOTORS_LOCKED 1`).**
- It compiles for the ESP32 Dev Module with the exact dependencies below.
- The build uploaded at S1 (221c778) **reboot-looped** (`rst:0xc`). The likely cause, a task
  watchdog conflict with AsyncTCP, is fixed in this build but unconfirmed on hardware. See
  `docs/research/2026-11-04_s1_reboot_investigation.md`.
- This build **cannot arm**: `/arm` answers 423 `MOTORS_LOCKED`, so no route moves a wheel.
- It prints numbered `[BOOT]` checkpoints and the reset reason (section 4b).

Firmware for the MecanumCarV2 robot (ESP32-WROOM-32, LD06 LiDAR): a Wi-Fi access point with
HTTP routes. Python drives it with `autonomy/http_robot_link.py` and the controller
`robot_controller.py`. Decision: `docs/decisions/2026-11-04_mvp_http_firmware.md`.

## 1. What is in this folder
| File | In Git | Purpose |
|------|--------|---------|
| `ESP32SlamRobot.ino` | yes | **The firmware to upload** |
| `wifi_secrets.example.h` | yes | Template for your Wi-Fi name and password |
| `wifi_secrets.h` | **no** (`.gitignore`) | Your real Wi-Fi name and password; you create it |
| `ld06.h`, `ld06.cpp`, `ld06crc.h` | **no** (`.gitignore`) | LD06 library files; you copy them in (licence unconfirmed) |
| `DEPENDENCIES.md` | yes | Exact versions and SHA-256 of everything else |
| `compile_check.py` | yes | Reproducible compile check (never uploads) |
| `README.md` | yes | This file |

**Not included, and why:**
- **`ld06.*`:** the LD06 library (Robot-Maker) README says MIT, but it has no LICENSE file. Its
  status is unconfirmed (`docs/decisions/2026-11-04_ld06_library_reuse.md`).
- **MecanumCar_v2:** no licence at all.
- **Async TCP and ESP Async WebServer:** LGPL-3.0. Install them with the Library Manager, with
  the exact versions below.

## 2. One-time setup in Arduino IDE 2
1. **Board package:** Tools > Board > Boards Manager > "esp32" by Espressif Systems,
   version **3.3.12**.
2. **Libraries** (Sketch > Include Library > Manage Libraries), exact versions:
   - **Async TCP** by ESP32Async, **3.5.0**
   - **ESP Async WebServer** by ESP32Async, **3.12.1**

   Don't install the older "AsyncTCP" (dvarrel) or "ESPAsyncWebServer" (lacamera) forks; they
   target ESP32 core 2.x.
3. **MecanumCar_v2:** Sketch > Include Library > Add .ZIP Library… >
   `C:\Users\stuar\Downloads\MecanumCar_v2.zip` (SHA-256 in `DEPENDENCIES.md`).
4. **LD06 files:** open `C:\Users\stuar\Desktop\SLAM\LD06.zip` and copy `LD06\src\ld06.h`,
   `ld06.cpp` and `ld06crc.h` into **this folder**, next to the `.ino`.
5. **Wi-Fi secrets:** copy `wifi_secrets.example.h` to `wifi_secrets.h` in this folder and edit
   it:
   - `ROBOT_SSID`: a name unique to this robot, for example `RobotESP32-SLAM-01`;
   - `ROBOT_PASSWORD`: 8 to 63 characters, not the example one.

   The sketch **refuses to compile** without `wifi_secrets.h`, with the example password, or with
   a password that is too short. `wifi_secrets.h` is never committed.

## 3. Open and compile
1. File > Open… > `firmware\ESP32SlamRobot\ESP32SlamRobot.ino` in the worktree
   `C:\Users\stuar\Desktop\SLAM-autonomy` (branch `feature/http-robot-mvp`).
   **Not** `Downloads\ESP32SlamRobot.ino`: that is the original draft, which does not compile.
2. Tools > Board > esp32 > **ESP32 Dev Module**, with these settings (the core's defaults):

   | Setting | Value |
   |---|---|
   | Upload Speed | 921600 (use 115200 if uploads fail) |
   | CPU Frequency | 240MHz (WiFi/BT) |
   | Flash Frequency | 80MHz |
   | Flash Mode | QIO |
   | Flash Size | 4MB (32Mb) |
   | Partition Scheme | Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS) |
   | Core Debug Level | None |
   | PSRAM | Disabled |
   | Arduino Runs On / Events Run On | Core 1 / Core 1 |
   | Erase All Flash Before Sketch Upload | Disabled |

3. Sketch > Verify/Compile. Expected: about 947 kB of flash (72 %) and 94 kB of RAM (28 %).

**Command-line check** (no upload; does not read your `wifi_secrets.h`):
```
python firmware\ESP32SlamRobot\compile_check.py --ld06-zip C:\Users\stuar\Desktop\SLAM\LD06.zip --mecanum-zip C:\Users\stuar\Downloads\MecanumCar_v2.zip
```
It verifies every hash and version, then checks three cases: no secrets fails, the example
password fails, and a test password compiles.

## 4. Upload: only during S1, after the checklist in section 7
1. Robot on its stand, all four wheels in the air, motor power cut-off within reach (section 6).
2. Connect the ESP32 by USB. Tools > Port: the CH340 port (it was COM7 before).
3. Sketch > Upload. If it waits on "Connecting…", hold the BOOT button until writing starts.
4. Tools > Serial Monitor at 115200: it prints `AP IP: 192.168.4.1`.

## 4b. Expected serial output (115200 baud) and reset loops
A healthy boot prints, once:
```
=== ESP32SlamRobot: COMPILE-VERIFIED, HARDWARE-UNTESTED ===
[BOOT] reset reason: POWERON (power applied)
[BOOT ...ms] 1 motors zeroed: zero PWM written; the motor board ACKed
[BOOT ...ms] 2 serial up: Serial 115200 baud
[BOOT ...ms] 3 motor task started: 10 ms motor task on core 1; stall check 300 ms; task watchdog 5 s
[BOOT ...ms] 4 LiDAR UART started: Serial2 230400 baud, RX GPIO33, PWM GPIO32 HIGH; ...
[BOOT ...ms] 5 Wi-Fi AP started: SSID "<your SSID>", IP 192.168.4.1, channel 6
AP IP: 192.168.4.1
[BOOT ...ms] 6 HTTP server started: port 80 (starts the async_tcp task)
[BOOT ...ms] 7 setup complete: robot DISARMED; MOTORS_LOCKED build: /arm is refused
[BOOT ...ms] 8 running: first LiDAR scan: N points, D deg/s
```
- **A reset loop:** `rst:` lines keep repeating, and `boot N since power-on` climbs (the
  `RESET LOOP` warning appears from boot 3). Copy the lines **just before** each `rst:`, plus
  the next boot's `reset reason` and `previous boot reached checkpoint`. The investigation
  note's section 5 explains them: BROWNOUT means power, TASK_WDT names a task, PANIC means a
  crash with a backtrace.
- **`motor board did NOT ACK`:** stop and cut motor power. The zero writes may not reach the
  controller.

## 5. Wiring reference (from the course sketch; check it on your robot)
| Signal | ESP32 GPIO | Notes |
|---|---|---|
| Motor board I2C SDA | 19 | bit-banged I2C to the motor coprocessor at address 0x30 |
| Motor board I2C SCL | 21 | |
| LD06 TX → ESP32 RX (Serial2) | 33 | 230400 baud |
| LD06 PWM (speed control) | 32 | held HIGH (fixed speed) |
| LD06 5 V / GND | 5 V / GND | |
| USB serial (CH340) | — | 115200 baud, Serial Monitor and upload |

Motor PWM registers on the coprocessor: front-left 3/4, front-right 1/2, rear-left 7/8,
rear-right 5/6 (from `MecanumCar_v2.cpp`; which motor is physically "front" is unverified).

## 6. Safety precautions
- **Never upload or power the robot with its wheels on a surface** until S1 and S2 are passed.
- **Never flash `ESP32MecanumCar.ino`:** it drives the motors for 5 s after boot.
- The firmware boots **DISARMED**. `setup()` first writes zero PWM to all motors. Nothing moves
  before `/arm` and then a `/drive`.
- Every move is a **lease of at most 1 s** (the controller uses 300 ms). The robot stops on its
  own when the lease runs out, the LiDAR goes quiet for 0.5 s, `/stop` arrives, or the Wi-Fi
  client disconnects.
- A 1 s task watchdog reboots the ESP32 if the motor task or `loop()` hangs.
- **Unknown, check at S1:** the motor board may keep its last PWM while the ESP32 resets or
  hangs. Until that is known, keep the physical power cut-off in reach.
- Anyone on the robot's access point can drive it: use a unique password.
- A `/drive` delayed by Wi-Fi can still start late (at most one lease). The command sequence
  number is kept across `/arm`, so a delayed command from before a STOP is refused.

## 7. S1 bench checklist (you flash; wheels in the air; hand on the power cut-off)
**S1a, with this diagnostic build (`MOTORS_LOCKED 1`), motor power off if you can:**
1. A clean boot log (section 4b), with no reset loop over 2 minutes.
2. The phone or laptop sees the SSID and joins it. `http://192.168.4.1/status` answers, with
   `boot_count` 1, `motor_ack` 1 and `motors_locked` 1.
3. `/scan2` returns scans (`robot_controller.py` REAL ROBOT preview), and `/arm` answers
   `MOTORS_LOCKED`.
4. The wheel incident (separate gate): verify the physical cut-off stops a spinning wheel.
   Observe whether the controller holds PWM across an ESP32 reset. **Only then** rebuild with
   `MOTORS_LOCKED 0` for S1b.

**S1b, with `MOTORS_LOCKED 0`, only after S1a passes.** Read `/status` throughout, or use
`robot_controller.py` in REAL ROBOT mode. Each step must stop the wheels.
1. Boot: the wheels stay still; `/status` shows `armed:0`.
2. `/arm`, then `/drive?vx=30&time=500&seq=1`: the wheels turn about 0.5 s, then stop alone.
3. A longer `/drive` (time=1000), then `/stop` mid-move: they stop at once; `armed:0`.
4. A `/drive` with the laptop's Wi-Fi turned off right after: they stop within 1 s.
5. Unplug the LiDAR (or stop it) during a move: they stop within 0.5 s; `/drive` answers
   `NO_LIDAR`.
6. **Separate safety gate, only after 1-5 pass and the setup is independently checked:**
   reset the ESP32 during a move and watch whether, and when, the wheels stop. If they keep
   turning after the reboot, or while the ESP32 is held in reset, **stop: a hardware cut-off
   is needed before any floor test**.
7. Check each wheel's direction for `vx`, `vy` and `wz` against `mixWheels()`.

## 8. Changes from the original draft (`Downloads\ESP32SlamRobot.ino`, SHA-256 3f675d9b…)
1. A 1 s task watchdog on the motor task and `loop()` (`TASK_WDT_MS`).
2. `enum MoveResult` is declared before the first function. The Arduino builder's generated
   prototypes need it there, and the draft did not compile.
3. `/arm` no longer resets the command sequence number. A `/drive` delayed from before a STOP
   would otherwise be accepted after the next ARM.
4. The Wi-Fi name and password moved to `wifi_secrets.h`, with compile-time checks.
5. **After S1 (diagnostic build):**
   - the shared task watchdog is no longer shortened to 1 s (it conflicted with AsyncTCP);
   - a 300 ms motor-task stall check replaces it;
   - `[BOOT]` checkpoints, reset reasons and reset-loop detection;
   - the motor I²C ACK is checked at boot;
   - `/arm` is refused if the motor task is missing, or while `MOTORS_LOCKED` is 1.

   The S1 build is in Git: `git show 221c778:firmware/ESP32SlamRobot/ESP32SlamRobot.ino`.
