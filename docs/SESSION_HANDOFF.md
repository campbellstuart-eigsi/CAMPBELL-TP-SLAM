# Session handoff

_Last updated: 2026-11-04 (resume Friday 2026-11-06)_ · _Session focus: restart robot driving
from the clean Workspace: a Drive (robot) panel and minimal I²C firmware on the professor's HTTP
routes_

## Project overview and current state
LiDAR SLAM Workbench (EIGSI university project): an LD06 LiDAR on an ESP32-WROOM-32 mecanum
robot. Python Tkinter/Matplotlib programs do the mapping (Codes 01-03) and localisation (Codes
04-06), in a seven-page desktop app (`00_main_app.py`) with a four-view Workspace (Scan Map,
Bitmap / Occupancy Grid, Vector Map, Current Scan / Live Robot).

**Now:**
- The Workspace has a **Drive (robot)** panel for manual ZQSD driving over Wi-Fi, and
  `firmware/WorkspaceDrive/` is a minimal I²C firmware based on the professor's sketch.
- Both are verified **on the PC only** (unit, GUI and smoke tests, Arduino compile).
- **Nothing has been flashed or tested on the robot.** The hardware blockers below are all
  open.

## Completed milestones
- Day 1 mapping (Codes 01-03) and Day 2 localisation (Codes 04-06), up to bd8b842 (`main`).
- M1 Stages 0-2 (e5f1a1d) and Stage 3, one-shot Workspace localisation (536dba2 + docs commit
  832e7ae, on `feature/ui-redesign`).
  - **Gate 3 has still not been formally accepted:** suite b never passed 3 times in a row,
    because the desktop minimised the test windows. Details:
    `results/m1_gate3_20261104_092409/REPORT.md`.
- Workspace Drive, software only: ca6826b (2026-11-04), pushed.

## Work performed this session
- **Restart decision:** the custom HTTP MVP (`feature/http-robot-mvp`, worktree
  `C:\Users\stuar\Desktop\SLAM-autonomy`) never moved the robot (its build was locked by
  design). The work restarted from 832e7ae on the new branch `feature/workspace-drive`.
  Record: `docs/decisions/2026-11-04_workspace_drive_restart.md`.
- **Professor's code reviewed** (in `C:\Users\stuar\Downloads`, not in Git):
  - `ESP32RobotControl.ino`: Wi-Fi access point `RobotESP32` / `12345678` at 192.168.4.1;
    timed moves of at most 2 s; `/stop`, `/scan`, `/pose`, `/ping`.
  - `RobotControl.html`: calls the same routes.
  - `ESP32MecanumCar.ino`: **moves for 5 s at boot**; use it only with the wheels in the air.
  - `MecanumCar_v2.zip`: software I²C on SDA GPIO19 / SCL GPIO21 to address 0x30, a fixed
    speed of 80/255, and **no ACK check**. Front-right = "Upper_R", PWM registers 1/2.
  - `WIFI ROBOT.zip`: holds the course `ld06.*` files.
- **Spec** `docs/specs/workspace_drive.md` (**Approved** by you):
  - one HTTP contract for I²C and future PWM robots;
  - a firmware motor adapter;
  - section 5 lists the hardware facts a PWM version needs.
- **Firmware** `firmware/WorkspaceDrive/`:
  - `WorkspaceDrive.ino`: the professor's sketch, with motor calls through `motor_i2c.h`
    (`motorsBegin`, `motorsStop`, `motorsMove(move, speed)`);
  - `MAX_MOVE_MS` 2000 → **500**;
  - optional `speed=0..255` (default 80);
  - SSID and password from a local `wifi_secrets.h`, which must be at least 8 characters
    (`static_assert`);
  - `wifi_secrets.example.h`, `.gitignore` and `README.md`.

  Rotations still last `time / 2`, as in the professor's sketch. Nothing moves at boot.
- **Python** `robot_drive.py`:
  - `DriveLink`: urllib, no proxy; timeouts 1.0 s ping, 0.3 s move, 0.5 s stop.
  - `DriveController`: one worker thread, `robot-drive`, does all the HTTP.
  - A held key renews a 300 ms move every 150 ms. Rotations send `time=600`, so they last
    300 ms.
  - It sends `/stop` (with one retry) on: release, Space or Esc, focus loss, no `keepalive()`
    for 0.5 s, disconnect, close, or a failed request (which also disconnects).
- **Workspace** `workspace_page.py` (+79 lines; the views and localisation are unchanged), a
  "Drive (robot)" group after Capture:
  - URL (default `http://192.168.4.1`), Connect/Disconnect, Speed 0-255 (default 80), a status
    line;
  - a **drive pad**: keys act only while it has focus. Z/S forward/back, Q/D strafe, A/E
    rotate, Space STOP.
  - `tick()` sends a keepalive only while the pad is focused and visible. `close()` stops and
    ends the thread.
  - Bug found by the smoke test and fixed: a closed page could still be ticked
    (`tick_drive` now checks `self.alive`).
- **Tests:**
  - `test_robot_drive.py` (11, headless, against a local fake robot);
  - `test_gui_workspace_drive.py` (5, real Tk, local fake robot; never 192.168.4.1).
- **Docs:** `AGENTS.md` test commands, `docs/STATUS.md`, this file, the decision record and the
  spec.

## Important files and locations
| Path | Purpose |
|------|---------|
| `00_main_app.py` | **Run this**: the seven-page app; the Drive panel is on the Workspace page |
| `robot_drive.py` | HTTP link and hold-to-drive controller (no Tk) |
| `workspace_page.py` | Workspace; the Drive code is `build_drive_controls` … `tick_drive` |
| `firmware/WorkspaceDrive/WorkspaceDrive.ino` | **Firmware to flash** (after Gate A) |
| `firmware/WorkspaceDrive/motor_i2c.h` | I²C motor adapter (MecanumCar_v2) |
| `firmware/WorkspaceDrive/README.md` | Library setup, flashing, routes |
| `docs/specs/workspace_drive.md` | Approved design; section 5 = PWM hardware needs |
| `docs/decisions/2026-11-04_workspace_drive_restart.md` | Why driving restarted this way |
| `test_robot_drive.py`, `test_gui_workspace_drive.py` | Drive tests |
| `C:\Users\stuar\Downloads\ESP32RobotControl.ino`, `MecanumCar_v2.zip`, `WIFI ROBOT.zip`, `RobotControl.html` | Professor's originals (not in Git) |
| `results/m1_gate3_20261104_092409/REPORT.md` | M1 Gate 3 evidence (untracked) |

## Tests and verification
| Date | Command or check | Result |
|------|------------------|--------|
| 2026-11-04 | `arduino-cli compile --fqbn esp32:esp32:esp32` on an unchanged copy of `ESP32RobotControl.ino` (the IDE's bundled arduino-cli, esp32 core 3.3.12) | OK: 944,180 B flash (72 %), 87,128 B RAM |
| 2026-11-04 | same, `firmware/WorkspaceDrive` scratch copy with test secrets, `--warnings all` | OK: 942,204 B (71 %), 87,128 B RAM; the only warning is in `ld06.cpp` (unused variable) |
| 2026-11-04 | same with a 5-character password | refused: `static assertion failed` (intended) |
| 2026-11-04 | `.\.venv\Scripts\python.exe -m unittest test_robot_drive -v` | 11 OK |
| 2026-11-04 | `.\.venv\Scripts\python.exe -m unittest test_gui_workspace_drive -v` | 5 OK, 2 runs, none skipped |
| 2026-11-04 | `.\.venv\Scripts\python.exe -m unittest test_workspace test_workspace_localisation test_robot_drive` | 33 OK (before the `tick_drive` fix) |
| 2026-11-04 | `.\.venv\Scripts\python.exe gui_smoke_test.py` | first run: TclError from `tick_drive` on a closed page (fixed); another run: `OSError: screen grab failed` (likely the window-minimising problem); final run: PASS, 0 tracebacks |
| 2026-11-04 | `.\.venv\Scripts\python.exe -m unittest test_robot_drive test_gui_workspace_drive test_workspace` | 19 OK (after the fix) |

Not verified this session:
- **Anything on the robot:** the board type, battery power, I²C, wheel directions, the 500 ms
  stop and real Wi-Fi timing. No COM port was opened and nothing was flashed.
- **Full suite a and suite b:** not re-run on this branch (focused tests only, as agreed).
- **The M1 Gate 3 items** (suite b three times in a row): still open.

## Known bugs, limitations and open questions
**Hardware blockers (all open, none solved):**
- The ESP32 stopped working when USB was unplugged, with the battery connected. It is probably
  powered by USB only (switch, charge or wiring). Unconfirmed.
- The front-right wheel spun unexpectedly, while the earlier firmware sent only zero speed.
  One hypothesis, unverified: a different, directly driven motor board, whose input is switched
  by the I²C signals on GPIO19/21.
- The motor-controller board type is not physically confirmed. The classroom has both I²C and
  direct-PWM robots.
- I²C to the motor board is not independently verified. `MecanumCar_v2` ignores ACKs, so a
  wrong board fails silently.
- The battery switch is not yet shown to remove motor power reliably.
- The new firmware has not been flashed or tested.

**Software limitations:**
- Rotations last `time / 2`; the panel compensates with `time=600`.
- `MecanumCar_v2` sends `Stop()` once, with no repeat.
- Linux key auto-repeat (press/release pairs) is not handled; Windows is the target.
- The professor's default SSID and password are shared by every classroom robot.
  `WorkspaceDrive` uses your own `wifi_secrets.h`.

**Documentation conflict:** `firmware/WorkspaceDrive/README.md` "Test after flashing" step 4
says to press EN during a move. Friday's plan says not to reset the ESP32 deliberately during
motion. **Skip that step.** An edit to the README is proposed and awaits your approval.

**Carried over from M1:**
- The desktop minimises test windows (cause unknown).
- The font zip licence is unresolved, and there is no root `.gitignore`.
- The green highlight blends with tab:green scans.
- Code 04 local search cannot be cancelled.

## Decisions and rationale
- **Restart from 832e7ae on the professor's routes; one HTTP contract for I²C and PWM; only
  I²C now:** reasons in `docs/decisions/2026-11-04_workspace_drive_restart.md`.
- **No ARM/DISARM and no motor lock (your request):** safety comes from the firmware's 500 ms
  timeout, the panel's stop rules and the battery switch.
- **Keys bound only to the drive pad, plus a keepalive from `tick()`:** typing in entry fields
  never drives, and a hidden page or a frozen GUI stops the robot.
- **urllib, not requests:** no new dependency.
- **Earlier M1 decisions:** see `results/m1_gate3_20261104_092409/REPORT.md`.

## Uncommitted changes and Git status
Checked on 2026-11-04 with `git status`, `git log` and `git rev-parse`:
- **Branch `feature/workspace-drive`:** HEAD **ca6826b** "Workspace Drive: ZQSD panel and
  minimal I2C firmware (hardware untested)", equal to `origin/feature/workspace-drive`.
- **Other branches, not modified:**
  - `feature/ui-redesign` = `origin/feature/ui-redesign` = 832e7ae;
  - `feature/http-robot-mvp` = origin = 74711dc (worktree `C:/Users/stuar/Desktop/SLAM-autonomy`);
  - `main` = bd8b842.
- **Uncommitted, from this handoff:** `docs/STATUS.md` and `docs/SESSION_HANDOFF.md`
  (modified), and `docs/decisions/2026-11-04_workspace_drive_restart.md` (new).
- **Untracked, leave out of commits:** `results/`, `.idea/`, `__pycache__/`, `hang.log`,
  `LD06.zip`, the three French TP `.md` files, `scans/classroom_extended.json`, `assets/fonts/`.
- **Never committed (ignored):** `firmware/WorkspaceDrive/wifi_secrets.h`, `ld06.*`,
  `MecanumCar_v2.*`. They don't exist in the repo yet; you add them on the flashing PC.

## Suggested spec status changes (awaiting approval)
- `docs/specs/workspace_drive.md`: keep it **Approved**. Change it to Implemented only after
  Gate B and real ZQSD driving pass on the robot.

## Next recommended milestone
Recommendation, not an agreed plan: **real ZQSD driving of the I²C robot from the Workspace.**

**Acceptance criteria:**
1. Gate A passes:
   - the board is confirmed I²C (0x30, GPIO19/21);
   - the ESP32 stays on with the battery alone;
   - the battery switch removes motor power every time;
   - no wheel moves at startup or shutdown.
2. `firmware/WorkspaceDrive` is flashed; the Serial Monitor shows `AP IP: 192.168.4.1`; `/ping`
   answers `OK` from the laptop.
3. In the Workspace, Connect shows "Connected to …". Each of Z, S, Q, D, A and E moves the
   intended wheels in the intended directions, with the wheels clear of the ground.
4. The wheels stop within about 0.5 s of: release, Space, clicking off the pad, Disconnect,
   and the laptop's Wi-Fi turned off.
5. Results recorded in a new `results/workspace_drive_<timestamp>/` and in `docs/STATUS.md`.

## How to resume
1. Read `AGENTS.md`, then this file and `docs/STATUS.md`. Run `git status` and
   `git log --oneline -3`. Expect `feature/workspace-drive` at ca6826b, or a later approved
   docs commit.
2. **Setup:**
   - Windows, `.\.venv\Scripts\python.exe` (Python 3.13);
   - Arduino IDE 2 with esp32 core 3.3.12, Async TCP 3.5.0 and ESP Async WebServer 3.12.1
     (installed on this PC);
   - USB is CH340 on **COM7**.
3. **Friday's first action is Gate A, at the robot, wheels off the ground, hand on the battery
   switch, no movement commands.** You do these; Claude never flashes, opens a COM port or
   drives motors.
   1. **Motor board:** photograph it and ask the professor whether it is the I²C (0x30)
      controller that `MecanumCar_v2` expects, or a direct-PWM driver. If it is PWM, stop: fill
      in spec section 5 first.
   2. **Power:** measure the battery voltage. Battery on, USB unplugged: the ESP32 LED stays on
      and its Wi-Fi network stays visible.
   3. **Cut-off:** check that the battery switch removes motor power every time (if a wheel
      spins, the switch stops it).
   4. **Startup and shutdown:** power up and power down normally a few times, and watch for any
      wheel movement. Do not reset the ESP32 deliberately during motion.
   5. **I²C, if appropriate:** with a non-moving diagnostic, confirm that 0x30 answers on
      GPIO19/21 only when motor power is on. Options:
      - the locked diagnostic build already on the robot, if it is still flashed (its `[I2C]
        … VERDICT` boot lines);
      - a 15-line address-only scanner sketch, which Claude can write on your approval.

   **If a wheel spins by itself, or motor power cannot be removed reliably, stop testing and
   take it to the professor.**
4. **Gate B, only after all of Gate A passes:**
   1. Open `firmware/WorkspaceDrive/WorkspaceDrive.ino` in Arduino IDE.
   2. Copy `MecanumCar_v2.cpp/.h` (from `Downloads\MecanumCar_v2.zip`) and `ld06.cpp`, `ld06.h`,
      `ld06crc.h` (from `Downloads\WIFI ROBOT.zip`, `TP LIDAR\`) next to it. Copy
      `wifi_secrets.example.h` to `wifi_secrets.h`, with a unique SSID and a password of at
      least 8 characters.
   3. Board ESP32 Dev Module, port COM7, Upload (hold BOOT if it stalls on "Connecting…").
   4. Serial Monitor at 115200: `AP IP: 192.168.4.1`, and no wheel movement.
   5. Join the robot's Wi-Fi from the laptop; open `http://192.168.4.1/ping`, which should
      answer `OK`.
   6. Run `.\.venv\Scripts\python.exe 00_main_app.py`, open the Workspace, find "Drive (robot)",
      and click Connect.
   7. Robot supported, wheels clear, cut-off verified: click the drive pad and make brief,
      supervised presses of Z, S, Q, D, A and E, then Space.
   8. Record which wheels move, in which direction, and whether they stop. Skip the README's
      "press EN during a move" step.
5. Bring the results back to Claude. Next come the acceptance criteria above, then (priority 4)
   live LiDAR localisation. Do not start autonomy, exploration, path planning, new mapping
   features or UI redesigns.

## Confirmed facts vs assumptions
- **Confirmed** (Git, files and commands run in this session):
  - the branch tips and the push above;
  - the files in ca6826b;
  - the compile and test results in the table;
  - the professor's files and their contents (pins 19/21, address 0x30, speed 80, routes, the
    2 s cap, the shared SSID and password);
  - Async TCP and ESP Async WebServer installed in `Documents\Arduino\libraries`;
  - esp32 core 3.3.12.
- **User-reported, not observed here:**
  - the ESP32 stops without USB;
  - the front-right wheel spin;
  - "no movement works";
  - some classroom robots use different motor hardware (from the professor);
  - LiDAR and Wi-Fi work on USB power.
- **Assumed:**
  - this robot is the I²C type;
  - the "Upper" motors are the front ones;
  - the earlier locked diagnostic build may still be on the robot (unknown);
  - the wheel-spin cause.
