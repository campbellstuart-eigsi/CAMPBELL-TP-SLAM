# Original TP robot files: findings and effect on the Phase B firmware plan

Status: **approved by the user on 2026-11-04.** The decisions are in `docs/decisions/2026-11-04_*`.
Written 2026-11-04. The files were read in `C:\Users\stuar\Downloads`, read-only. Nothing was
copied, modified, extracted or flashed. Statements are labelled:
- **[code]**: read in the source;
- **[inferred]**: an engineering reading of the source;
- **[unknown]**: needs a measurement or a document we don't have.

## 1. Files inspected

| File | SHA-256 (first 16) | What it is |
|---|---|---|
| `ESP32RobotControl.ino` and `ESP32RobotControl (1).ino` | e45a9150b9fc2993 (both identical) | the robot firmware: SoftAP, HTTP control, LD06 scans |
| `RobotControl.html` | 5035b8b77ac7800e | the browser client (© J. Quilles 2026) |
| `ESP32MecanumCar.ino` | 4db15a2d1509fecd | a motor demo sketch |
| `MecanumCar_v2.zip` (`MecanumCar_v2.h/.cpp`) | 4b03d2cedb5a380c | the motor library (read via `unzip -p`, not extracted) |
| `LD06\LD06\src\ESP32_LD06_Test\` (`.ino`, `ld06.cpp/.h`, `ld06crc.h`) | ld06.cpp d7aa8f062e13875c | LD06 test sketch; the library is byte-identical to the one in `SLAM\LD06.zip` |
| `support lidar.stl` | 8260a0383c90224a | LiDAR support plate (CATIA ASCII STL) |

## 2. `ESP32RobotControl.ino` (read completely, 247 lines)
**Wi-Fi [code]**
- SoftAP (`WiFi.mode(WIFI_AP)`), SSID `RobotESP32`, an 8-digit numeric password hard-coded in
  the source, default IP 192.168.4.1.
- Power-save mode, channel and `max_connection` are not set, so they are at their defaults:
  modem sleep on, and Arduino's default of 4 clients (see `esp32_wifi_transport.md`).

**Protocol [code]**
- HTTP/1.1 on port 80, using `ESPAsyncWebServer` + `AsyncTCP` (third-party libraries, not in
  Downloads; their versions are unknown).
- Every route is a GET with `Access-Control-Allow-Origin: *`.

| Route | Effect |
|---|---|
| `/forward`, `/backward`, `/left`, `/right` (strafe), `/rotateleft`, `/rotateright` `?time=ms` | open-loop timed move; `time` clamped to 1-2000 ms (`MAX_MOVE_MS`); rotations use **time/2**; `time ≤ 0` stops |
| `/stop` | `mecanumCar.Stop()` |
| `/scan` | latest full scan, binary: `x_mm i16, y_mm i16, θ_cdeg u16` (the last `/pose` written), `n u16`, then n × `angle_cdeg u16, distance_mm u16, intensity u8`; `503 NO_VALID_SCAN` before the first scan |
| `/pose?x=&y=&z=` | stores a pose that is only echoed in `/scan`. The robot does not use it. |
| `/ping` | `OK` |

**Motors [code]**
- `mecanumCar(19, 21)`: a bit-banged I2C bus (section 4).
- Fixed speed: PWM 80/255 on every wheel.
- A move starts at once. A new command **replaces** the running one and restarts its timer.
- `loop()` stops the motors when `millis() - moveStarted ≥ moveDuration`.
- A FreeRTOS mutex serialises the HTTP-task and loop-task access to the I2C bus.

**LiDAR [code]**
- `Serial2` at 230400 baud: RX = GPIO33, TX unused, RX buffer 1024 bytes.
- LD06 PWM pin GPIO32 held HIGH (`lidar.init()`); no speed PWM.
- Robot-Maker LD06 library, full-scan mode. `publishScan()` copies each completed scan;
  **scans with more than 460 points are silently dropped**.
- **Angle convention [code, `ld06.cpp:137`]:** `angle = 360 − physAngle`. The library converts
  the LD06's clockwise angle to the mathematical **counter-clockwise** convention (unless
  `setUpsideDown(true)`).
  - `/scan` therefore sends **CCW** angles, and `RobotControl.html` draws them with `cos`/`sin`
    plus the heading.
  - The library spreads points as `start + (i + 0.5)·step`, not the manufacturer's
    `start + i·step` that Code 01 uses.

**Safety features present [code]:**
- The motors are stopped in `setup()`.
- Every move ends within 2 s.
- There's a `/stop` route, and the bus is mutex-protected.

**Safety gaps [code / inferred]:**

| Gap | Consequence |
|---|---|
| No arming; any client of the AP can drive the robot | **Any web page** open in a browser on a connected laptop could too (`CORS *` with plain GETs) |
| No heartbeat or watchdog | Motion is bounded only by the 2 s move time. A client that keeps sending moves keeps the robot moving, with no check that the link is healthy. |
| I2C ACKs are ignored (`IIC_RecvACK()` result unused) | A **failed STOP write is silent**. No driver fault is ever detected. |
| HTTP over TCP | A STOP can queue behind earlier requests (head-of-line blocking); the browser's request timeouts don't stop the robot |
| No telemetry, faults or communication health | The host cannot know the robot state |
| The motor coprocessor's behaviour on an ESP32 reset or hang is unknown | If it holds the last PWM values, a firmware hang leaves the wheels turning (**[unknown]**: the most important open hardware question) |

## 3. `ESP32MecanumCar.ino`: hazard
**[code]** For the first 5 s after boot, this sketch **drives the motors automatically** through
forward, back, right, left and both rotations, 0.5 s each. It must never be flashed while the
wheels can touch anything. It also never calls `Init()`.

## 4. `MecanumCar_v2` library (the actual motor interface)
**[code]**
- **Bus:** bit-banged I2C on **SDA = GPIO19, SCL = GPIO21** (constructor order), to 7-bit
  address **0x30**.
  - Each write is `START, 0x60, reg, value, STOP` with about 1 µs delays.
  - ACKs are read but ignored. `pinMode` is set in a global constructor, before `setup()`.
  - The members `SDA` and `SCL` shadow the ESP32 core's own `SDA`/`SCL` constants.
- **Register map:**

  | Register | Function |
  |---|---|
  | 1, 2 | `Motor_Upper_R` |
  | 3, 4 | `Motor_Upper_L` |
  | 5, 6 | `Motor_Lower_R` |
  | 7, 8 | `Motor_Lower_L` |
  | 0x09 | "P54" output (left LED) |
  | 0x0A | "P55" output (right LED) |

  - Registers 1-8 are PWM channels, 0-255.
  - Each motor uses two channels: direction 1 puts PWM on the even channel, direction 0 on the
    odd one, with the other channel at 0. This looks like an H-bridge controlled by a
    coprocessor **[inferred]**.
  - "Upper" and "Lower" are probably front and rear **[unknown]**.
- **API:** `Init`, `Stop`, `Advance`, `Back`, `Turn_Left`, `Turn_Right`, `L_Move`/`R_Move`
  (strafe), four diagonals, `drift_left`/`drift_right`, per-motor `Motor_*(dir, pwm)`, LEDs,
  `PWM_OUT(ch, pwm)`. Speed comes from the global `speed_*` bytes (80).
- **Origin:** the header comments say "raspberry pi pico", 2021/2022 (comments partly
  mis-encoded). There is no licence file **[unknown licence]**.
- **No feedback of any kind:** no encoders, no current sensing, no read-back.

## 5. `support lidar.stl` (geometry only; not proven to match the installed part)
**[code: mesh analysis]**
- A **flat plate**, 132 × 68 × 3 (assumed mm), 1428 triangles, closed (volume about
  20 700 mm³).
- Corners are rounded or chamfered: the outline passes about 10 mm inside the bounding-box
  corners, which suggests a radius of about 24 mm if they are arcs.
- **4 holes, Ø 4.2**, at (±61, ±19): a 122 × 38 pattern. It fits an M4 clearance hole
  **[inferred]**; the chassis mounting is presumed.
- **4 holes, Ø 2.5**, at (±24.5, ±29): a 49 × 58 pattern. It could be an M2.5 clearance or an M3
  tapping hole **[inferred]**.
- **2 slots, 8 × 40**, centred at (±26.5, 0), running along the short axis. Possibly cable
  passage or strap slots **[inferred]**.

It tells us nothing about the LiDAR's height, its offset from the robot's centre of rotation, or
which way its 0° faces. The LD06 datasheet PDF could not be rendered here (no poppler), so the
sensor's own hole pattern was **not compared**.

## 6. Original firmware vs the Phase B plan (protocol v2)

| Topic | Original TP firmware | Phase B plan (v2) | Gap / action |
|---|---|---|---|
| Transport | HTTP/TCP on SoftAP | UDP datagrams on SoftAP; USB serial fallback | **Same SoftAP idea (keep).** HTTP is replaced; `RobotControl.html` won't talk to the new firmware |
| Access | anyone on the AP, or any browser page | session binding; STOP/DISARM from anyone | v2 is stricter (intended) |
| Arming | none | ARM / DISARM, starts disarmed | new |
| Comm loss | none (2 s move limit only) | heartbeat with uptime echo, 300 ms watchdog | new |
| Command expiry | none | `valid_until_ms` + `timeout_ms` (≤ 3 s) | new; original ≤ 2 s → **adopt 2 s** as our limit for the first hardware tests |
| Motion command | 6 primitives × duration, fixed PWM 80 | MOVE: turn (cdeg) then forward (mm) at mm/s | needs a **PWM → speed calibration** (no encoders); strafing unused by v2 (optional later) |
| Rotation | `time/2` (unexplained) | turn rate in cdeg/s | calibrate; don't copy the /2 |
| Motor driver | MecanumCar_v2, ACKs ignored | `MotorDriver` interface, mock only | physical driver = I2C 0x30 register map; **must check ACKs → DRIVER_FAULT** |
| Scan format | 5-byte points (angle_cdeg, distance, intensity); ≤ 460 points; pose header | the same 5-byte points in 101-point chunks, any size, with CRC, resend | **compatible point layout**; drop the pose header |
| Scan angles | **CCW** (library `360 − phys`) | written as **raw clockwise** | **Incompatibility: decide one convention** (section 8) |
| LD06 acquisition | Robot-Maker library, Serial2 RX33, PWM32 HIGH | own parser planned (licence concern) | the library is proven on this robot and identical to `SLAM\LD06.zip` (README says MIT, no LICENSE file) |
| Concurrency | HTTP task + loop, mutex | single loop, polled UDP, no callbacks | keep v2 (simpler to reason about) |
| Telemetry | none | 5 Hz state, faults, link health | new |
| Credentials | hard-coded weak password | `wifi_secrets.h`, git-ignored | new |

## 7. Missing hardware information (needed before H2 motion; none can be inferred safely)
1. **Motor coprocessor at 0x30:**
   - What chip it is, its firmware, and whether it supports standard hardware I2C (`Wire`).
   - What it does when I2C stops or the ESP32 resets: does it hold the last PWM or time out?
     **This is the safety-critical one.**
   - Its power-up output state.
2. **Wheels:** which motor is which (Upper = front?), roller orientation (X or O layout), wheel
   diameter, gear ratio, and PWM versus speed under load. There are no encoders.
3. **Power:** battery type and voltage, whether the ESP32 and the LD06 share it with the motors,
   and current during Wi-Fi TX together with motor start (risk of brown-out resets).
4. **Hardware stop:** is there a physical power switch or emergency stop that cuts the motors?
5. **Geometry:** robot footprint and radius (the planner assumes 0.15 m), centre of rotation, LD06
   position and height on the plate and chassis, and which way the LD06's 0° faces relative to
   "forward". The library supports `setOffsetPosition`; we would apply the offset on the host.
6. **Free pins:** GPIO19/21 (motors) and 32/33 (LiDAR) are taken. Battery sensing must use ADC1
   (GPIO 34-39, input only) because of Wi-Fi.
7. The **ESP32 core and library versions** the original was built with (for comparison builds
   only).

## 8. Recommendation: yes, the firmware plan needs updating
I'm proposing these changes. None is made until you approve.

1. **Physical motor driver target (still mock until H2).**
   - Add `MecanumI2cDriver` behind the existing `MotorDriver` interface, using the register map
     from the library source (documented, not guessed). It stays compiled out behind
     `MOTOR_OUTPUTS_ENABLED 0` / `HARDWARE_VERIFIED`.
   - Unlike the original, it will:
     - **check every I2C ACK**; a NACK or bus error latches DRIVER_FAULT;
     - **re-write zero PWM** periodically while not moving, so a coprocessor that missed the
       STOP is corrected within one period;
     - read back nothing (none is possible) and say so in telemetry.
   - Hardware I2C (`Wire.begin(19, 21)`) instead of bit-banging, if the coprocessor accepts it
     (verify in H2). Otherwise, port the bit-bang with ACK checking.
2. **Body-velocity mapping:** forward/turn map to the four wheels, as `Advance`/`Turn_*` do.
   - Strafing (`L_Move`/`R_Move`) stays out of protocol v2 and could become an optional `vy` in
     v3 once the planner can use it.
   - `speed_mm_s` maps to PWM through a **calibration table**, measured in H2 with the wheels off
     the ground, then on the floor. Until then the table is empty and the physical driver
     refuses to start.
3. **LiDAR:** reuse the Robot-Maker LD06 library (proven on this robot, identical copy) behind
   our scan source, once you accept its licence status (README: MIT, no LICENSE file). Keep the
   same pins and settings (Serial2 RX33, PWM32 HIGH, 230400).
   - **Angle convention decision.** Option A (recommended): change protocol v2 SCAN_CHUNK to the
     library's **CCW** angles. This matches the robot's existing data and Code 01's CSV path, and
     the host converts with `cos`/`sin` directly.
   - Option B: keep raw clockwise angles and use our own parser.
   - Either way, the "more than 460 points dropped" limit disappears.
4. **Timing for the first hardware stages:** `MAX_TIMEOUT_MS` drops from 3000 to **2000**
   (matching the original's proven bound), and `MAX_FORWARD_MM` from 500 to **300**, until
   motion is calibrated.
5. **Keep the original firmware separate:**
   - The TP sketch and `RobotControl.html` stay as they are, as the "legacy TP" setup.
   - The new firmware has **no HTTP server**: no TCP stack load, and no unauthenticated
     browser control.
   - **Never flash `ESP32MecanumCar.ino`** (it moves on boot).
6. **A new hardware question for H2,** before any physical driver code runs: with the wheels off
   the ground and the original firmware, observe what the motors do when the ESP32 is reset or
   held in reset during a move. This needs your approval and your hands on the robot.
7. **H1 additions:**
   - Confirm that the LD06's 0° direction relative to the chassis "forward" uses the CCW
     convention: a scan facing a wall at a known bearing.
   - Measure the LiDAR offset from the robot's rotation centre.

**Reused as-is:**
- the SoftAP approach and the 192.168.4.1 address;
- the LD06 wiring and configuration;
- the LD06 library (subject to the licence decision);
- the 5-byte scan point layout.

**Reused as knowledge:**
- the MecanumCar_v2 register map and motor-to-channel mapping;
- the wheel patterns of each motion primitive.

**Not reused:**
- the HTTP control API;
- the ACK-ignoring bit-bang driver;
- the CORS-open endpoints;
- the hard-coded credentials;
- the rotation `time/2` factor.
