# Workspace drive: one ZQSD panel for I²C and PWM robots

Status: Approved
_Created: 2026-11-04 · Approved by the user 2026-11-04 · Branch `feature/workspace-drive`
(from 832e7ae)_

## Goal
Drive the real robot with Z/Q/S/D/A/E over Wi-Fi from a small panel in the existing four-view
Workspace. The classroom has two kinds of mecanum robot:
- **I²C:** an I²C motor controller at 0x30 on GPIO19 (SDA) / GPIO21 (SCL), driven by the
  professor's `MecanumCar_v2` library.
- **Direct PWM:** the ESP32 drives a motor-driver board directly. Pinout and driver model are
  unknown (section 5).

The same Python panel must work with both. **Only the I²C version is implemented and verified
now.**

## 1. The boundary: one HTTP contract
The Python panel only knows the routes of the professor's `ESP32RobotControl.ino`, which
`RobotControl.html` also uses:

| Route | Meaning |
|---|---|
| `/forward`, `/backward`, `/left`, `/right`, `/rotateleft`, `/rotateright` `?time=ms` | Move for `time` ms (capped at 2000), then stop on its own |
| `/stop` | Stop now |
| `/ping` | `OK` (connection check) |
| `/scan` | Latest LD06 scan, binary (unchanged) |

Any robot whose firmware answers these routes works with the panel. The motor type never
reaches Python.

Possible later additions (not needed for the first drive; each needs approval):
- an optional `speed=0..255` on the move routes, defaulting to the library's 80;
- a `/status` that **reports** the motor type. It is reported by the firmware, not detected.

## 2. Firmware: a small motor adapter
The professor's sketch already has the seam. Every move goes through `doMove()` with one
`mecanumCar` call (`Advance`, `Back`, `L_Move`, `R_Move`, `Turn_Left`, `Turn_Right`, `Stop`).
In Part 2 those calls move behind a tiny adapter with three functions:

```
void motorsBegin();                      // zero all motors at boot
void motorsStop();
void motorsMove(Move m, uint8_t speed);  // Move = FORWARD, BACKWARD, LEFT, RIGHT, ROT_LEFT, ROT_RIGHT
```

- **`motor_i2c.h`** (now): calls `MecanumCar_v2` exactly as the professor's sketch does.
  Speed sets the library's `speed_Upper_L` / `speed_Lower_L` / `speed_Upper_R` /
  `speed_Lower_R`.
- **`motor_pwm.h`** (later, not written): the same three functions for a PWM driver, once
  section 5 is known.
- The sketch includes one of them, chosen by a single `#include` line. There is no automatic
  detection and no second copy of the firmware.

Everything else stays shared: Wi-Fi, routes, the move timeout, `/stop` and the LD06 scan.

## 3. Python: one link, one panel
- **Link** (`robot_drive_link.py` or similar): `ping()`, `move(direction, time_ms)`, `stop()`
  over HTTP to `http://192.168.4.1`. It might reuse the request/timeout handling from
  `autonomy/http_robot_link.py` on `feature/http-robot-mvp` (read-only reference); the
  `/drive?vx=…` protocol is not reused.
- **Panel** in the Workspace controls column. It has:
  - Connect/Disconnect, a status line and a speed slider;
  - Z forward, S backward, Q strafe left, D strafe right, A rotate left, E rotate right,
    Space STOP.
- **Hold to move:** while a key is held, repeat the move with `time` ≈ 300 ms every ≈ 150 ms.
  Release, Space, focus loss, disconnect or a failed request sends `/stop`. If messages stop
  arriving, the firmware's own timeout still stops the wheels within `time`.
- The Workspace's four views and localisation are not changed.

## 4. Order of work
1. **Part 1 (user, hardware):** identify the board, check power, the physical stop and the
   wheel spin; flash `ESP32RobotControl.ino` unchanged and test its routes in a browser.
   Compile-verified unchanged on 2026-11-04 (944,180 B flash, 72 %).
2. **Part 2:** firmware = the professor's sketch, with the I²C adapter split out (behaviour
   unchanged, compile-checked).
3. **Part 3:** the Python link and the Workspace panel, tested against a local fake robot,
   then on the real robot by the user.
4. **Later:** `motor_pwm.h`, once section 5 is filled in.

## 5. Needed before a PWM version (unknown today, do not assume)
- Motor-driver board model and markings (for example L298N, TB6612FNG, DRV8833, or a custom
  shield), and a photo.
- For each wheel (front-left, front-right, rear-left, rear-right): the ESP32 GPIOs and their
  roles (IN1/IN2, or DIR + PWM, and any enable/STBY pin).
- Driver logic: which input levels mean forward, reverse, coast and brake.
- PWM frequency and resolution the driver accepts, and its logic voltage (3.3 V compatible?).
- Motor supply voltage, and whether the battery also powers the ESP32.
- Each wheel's forward direction (motors may be mounted mirrored).
- What the driver inputs do while the ESP32 resets or boots (pull-downs?). Floating inputs can
  spin a wheel; compare the unexplained front-right spin.
- Pin conflicts with the LD06 (GPIO32/33) and the ESP32 boot-strapping pins (0, 2, 5, 12, 15).
- Encoders, if any (not needed for manual driving).
- The professor's reference sketch for that robot type, if one exists.

## Open questions
- Which type is this robot? Part 1 step 1 answers that.
- The cause of the front-right wheel spin (hardware until shown otherwise).
- Speed control: whether the fixed 80/255 is enough, or the optional `speed` parameter is
  needed.
