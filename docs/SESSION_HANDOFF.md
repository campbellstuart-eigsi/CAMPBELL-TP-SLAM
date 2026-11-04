# Session handoff

_Last updated: 2026-11-04_ · _Session focus: robot controller simplified to "connect, drive,
stop"; firmware motor/I²C diagnostics; earlier this session: S1 reboot-loop investigation,
diagnostic firmware, headless live-localisation tracker_

## Project overview and current state
- **The project:** LiDAR SLAM Workbench (EIGSI TP). An LD06 LiDAR on an ESP32-WROOM-32 Mecanum
  robot whose wheels are driven by an **I²C motor controller** (confirmed by the user's
  professor). Python programs do the mapping and localisation (Codes 01-06), with a four-view
  Workspace.
- **This branch** (`feature/http-robot-mvp`, worktree `C:\Users\stuar\Desktop\SLAM-autonomy`)
  builds the real robot link:
  - an HTTP firmware on the ESP32 (`firmware/ESP32SlamRobot/`);
  - a keyboard controller on the laptop (`robot_controller.py`);
  - a headless live-localisation tracker.
- **Now:**
  - The robot boots and its Wi-Fi works with the diagnostic firmware (user-reported).
  - The controller has **not** moved the robot. The firmware is built with `MOTORS_LOCKED 1`
    on purpose, until an unexplained wheel spin and the motor cut-off are checked.
  - The latest work (controller simplification and motor diagnostics) is committed and pushed
    (23b0506).

## Completed milestones
- Phase B approach A, steps 1-2c: protocol v2, firmware model, `ProtocolRobotLink`, simulated
  Wi-Fi. Commit 364982a, branch `feature/autonomy-simulator`. Parked; see
  `docs/specs/autonomy_phase_b.md`.
- Approach B chosen: `docs/decisions/2026-11-04_mvp_http_firmware.md`.
- S0: firmware compiles; `HttpRobotLink`; fake robot (6cbefa9).
- Controller GUI and Arduino upload package (221c778).
- Diagnostic firmware after the S1 reboot loop (275782b):
  - the shared watchdog is no longer shortened (it conflicted with AsyncTCP); a 300 ms
    motor-task stall check replaces it;
  - `[BOOT]` checkpoints and the reset reason;
  - `MOTORS_LOCKED`.
- Headless live-localisation tracker (2cdd3e3):
  - 240/240 simulated scans `ok`;
  - position error max 5.8 cm (the fixture map's known cell bias; 4.9 mm on the control map);
  - simulated truth only.

All four approach-B commits are pushed to `origin/feature/http-robot-mvp`.

## Work performed this session
**Firmware** (`firmware/ESP32SlamRobot/ESP32SlamRobot.ino`, 23b0506, COMPILE-VERIFIED /
HARDWARE-UNTESTED):
- I²C ACK sampled with `INPUT_PULLUP` (`recvAckPulled`). The library's floating read could
  report a missing controller as an ACK.
- At boot, before the motor task and without writing any register:
  - SDA idle level;
  - address probe of 0x30, and of 0x31 as a control that nothing should answer;
  - an `[I2C] … VERDICT` line.
- `/status` adds `motors_locked`, `i2c_sda_idle_high`, `i2c_ack_0x30`, `i2c_ack_0x31`,
  `i2c_writes`, `pwm[4]`, `last_stop`, `drive_accepted`, `rejects`, `last_reject` and
  `last_reject_age_ms`.
- Serial: `[CMD] refused …`, `[CMD] drive accepted … PWM …`, `[MOTOR] move ended …` and
  `[I2C] … not ACKed`, at most once per 500 ms each.
- `MOTORS_LOCKED 1` (the user's working copy had been set to 0; set back to 1 at the user's
  request for these diagnostics).

**Controller** (23b0506). Decision `docs/decisions/2026-11-04_simplified_controller_auto_arm.md`:
- `autonomy/teleop.py`:
  - automatic `/arm` on the first new key press while stopped;
  - keys held through a STOP must be released before driving again;
  - a refused ARM is not retried by auto-repeat;
  - a STOP that races an ARM re-sends `/stop`;
  - the bench-safety acknowledgement was removed.
- `autonomy/controller_ui.py`:
  - no ARM/DISARM buttons, no safety checkboxes;
  - opens in REAL ROBOT mode;
  - new status rows: I²C, Motors (PWM), Last stop, Refused;
  - the Firmware row shows "MOTORS LOCKED build".
- `autonomy/http_robot_link.py`: refusals carry the firmware's text, e.g.
  `MOTORS_LOCKED (HTTP 423)`.
- `autonomy/fake_http_robot.py`: `locked=` option, and the same diagnostic `/status` fields.
- `robot_controller.py`: docstring describes the new use.

**Tests and docs:**
- `test_autonomy_teleop.py`, `test_gui_robot_controller.py`, `test_autonomy_http.py` and
  `test_firmware_static.py` updated and extended.
- The GUI tests switch to SIMULATOR (or a local fake) and never contact 192.168.4.1. Their key
  helper re-takes the window focus, or fails with an `ENVIRONMENT` message.
- Firmware README section 7 (S1a-motor procedure), `docs/STATUS.md` (rewritten), the new
  decision record, and this file.

## Important files and locations
| Path | Purpose |
|------|---------|
| `robot_controller.py` | **Run this** (PyCharm) to drive the robot or the simulator |
| `autonomy/teleop.py` | Driving core: worker threads, leases, auto-arm, STOP rules |
| `autonomy/controller_ui.py` | The controller window (`RobotControllerPanel`, embeddable later) |
| `autonomy/http_robot_link.py` | HTTP `RobotLink` for the firmware |
| `autonomy/fake_http_robot.py` | Fake robot on 127.0.0.1 (simulator and tests) |
| `autonomy/live_localisation.py`, `autonomy/localisation_sim.py` | Headless tracker (Code 04) and its simulated route |
| `firmware/ESP32SlamRobot/ESP32SlamRobot.ino` | **Firmware to upload** |
| `firmware/ESP32SlamRobot/README.md` | Build, upload, expected serial output, S1 procedures |
| `firmware/ESP32SlamRobot/DEPENDENCIES.md`, `compile_check.py` | Exact libraries and hashes; reproducible compile (no upload) |
| `docs/research/2026-11-04_s1_reboot_investigation.md` | Reboot loop, wheel incident, hardware facts vs assumptions |
| `docs/decisions/2026-11-04_*.md` | 7 decision records |
| `results/live_localisation_20261104_152215/` | Tracker evidence (untracked) |
| `C:\Users\stuar\Downloads\MecanumCar_v2.zip`, `C:\Users\stuar\Desktop\SLAM\LD06.zip` | Library sources (not in Git) |

## Tests and verification
All on 2026-11-04, in this worktree, with `C:\Users\stuar\Desktop\SLAM\.venv\Scripts\python.exe`.

| Date | Command or check | Result |
|------|------------------|--------|
| 2026-11-04 | `python firmware\ESP32SlamRobot\compile_check.py --ld06-zip C:\Users\stuar\Desktop\SLAM\LD06.zip --mecanum-zip C:\Users\stuar\Downloads\MecanumCar_v2.zip --libraries <scratchpad libraries>` | PASSED: 952,240 B flash (72 %), 94,056 B RAM (28 %); no-secrets → `#error`, example password → `static_assert` |
| 2026-11-04 | `arduino-cli compile --warnings all` of the same sketch | exit 0; the only warning is in `ld06.cpp` (unused variable) |
| 2026-11-04 | `-m unittest test_firmware_static test_autonomy_http test_autonomy_teleop` | 51 OK |
| 2026-11-04 | `-m unittest test_gui_robot_controller` | 14 OK, 3 runs in a row |
| 2026-11-04 | Full headless suite (command in "How to resume"), after the controller simplification | 600 OK (301 s) |
| 2026-11-04 | `python -m autonomy.localisation_sim` | 240/240 ok (2cdd3e3) |

Not verified this session:
- **Nothing on hardware since the diagnostic build:** I²C answers, PWM, wheel motion, the stall
  check. No COM port was opened; nothing was flashed.
- The M1 GUI suites (`test_gui_map_display`, `test_gui_workspace`) and `gui_smoke_test.py`: not
  re-run (no Workspace file changed on this branch).

## Known bugs, limitations and open questions
- **Safety (blocks all movement):**
  - an S1 wheel spun with no command from this firmware;
  - whether the I²C controller holds PWM through an ESP32 reset is unknown;
  - the physical motor cut-off is unverified.
- **Weak default speed:** the controller's 30 % = PWM 24 of 255 (cap 50 % = PWM 40), while the
  course library uses PWM 80. The wheels may not turn. Raise Drive % before changing firmware.
- **Reboot-loop cause unconfirmed** (watchdog conflict vs brownout); no serial log of a failing
  boot was captured.
- **Unchecked on the robot:** I²C address, pins and register map (library code only); which
  motor is "front".
- Late Wi-Fi commands can still start up to one 300 ms lease late. Legacy course routes
  (`/forward` …) bypass the seq check.
- Licences: LD06 library (README MIT, no LICENSE file); MecanumCar_v2 (none).
- From M1: GUI suite b instability (desktop takes window focus); font licence and root
  `.gitignore` decisions open.

## Decisions and rationale
- HTTP MVP instead of the custom protocol: faster to a TP demo; reuses the course firmware.
  (`docs/decisions/2026-11-04_mvp_http_firmware.md`)
- Connect, drive, stop, with automatic arming: the user wants the simplest workflow; the firmware
  already enforces arming. (`docs/decisions/2026-11-04_simplified_controller_auto_arm.md`)
- The shared task watchdog stays at 5 s, with a 300 ms motor-specific stall check: AsyncTCP needs
  more than 1.25 s. (`docs/research/2026-11-04_s1_reboot_investigation.md`)
- The live tracker calls Code 04 directly, not the Workspace adapter
  (`docs/specs/autonomy_phase_b.md`, section 2).
- Earlier records: SoftAP/UDP, CCW angles, Mecanum I²C driver, LD06 library reuse, interim motion
  limits (`docs/decisions/`).

## Git status
- This session's work is committed and pushed: **23b0506** "Simplified controller (connect,
  drive, stop) with automatic arming; firmware motor/I2C diagnostics (motors locked)" (14
  files), followed by a docs-only commit that brings this file and `docs/STATUS.md` up to date.
  HEAD = `origin/feature/http-robot-mvp`.
- No tracked changes remain.
- **Untracked, leave out of commits:** `results/`, `__pycache__/`, `autonomy/__pycache__/`,
  `firmware/autonomy_robot/` (parked approach-A C++).
- **Ignored local files:** `wifi_secrets.h`, `ld06.*`, `MecanumCar_v2.*`.
- The original checkout `C:\Users\stuar\Desktop\SLAM` (`feature/ui-redesign`, 832e7ae) was not
  modified.
- The previous version of this file (Phase B details) is in Git at 2cdd3e3.

## Suggested spec status changes (awaiting approval)
- `docs/specs/autonomy_phase_b.md`: Draft → Superseded (in part) by the HTTP MVP decision, or
  keep Draft for the parked approach A. Your choice; not changed.

## Next recommended milestone
Recommendation, not an agreed plan: **at the robot, still locked**, run the firmware README's
section 7 (S1a-motor):
1. boot log and `[I2C] VERDICT`;
2. `/status` with motor power on and off (0x30 ACK should follow motor power);
3. the controller shows `cannot drive: MOTORS_LOCKED`.

Then the physical gate (cut-off stops a wheel; reset with wheels in the air), and only then
`MOTORS_LOCKED 0` for wheels-up tests at 50 % Drive. In software meanwhile: the tracker on the
real classroom map.

## How to resume
1. Read `AGENTS.md`, then this file and `docs/STATUS.md`.
2. **Setup:**
   - Windows; Python `C:\Users\stuar\Desktop\SLAM\.venv\Scripts\python.exe` (the worktree has no
     venv).
   - Arduino IDE 2 / arduino-cli 1.5.1, esp32 core 3.3.12.
   - Libraries per `firmware/ESP32SlamRobot/DEPENDENCIES.md`. In this session they were only in
     a scratch folder; install them on the flashing PC.
3. **Drive:**
   1. Join the robot's Wi-Fi (SSID from your local `wifi_secrets.h`).
   2. Run `robot_controller.py`, click Connect (REAL ROBOT, `http://192.168.4.1`).
   3. Hold Z/S/Q/D/A/E to drive, release to stop; Space or Esc = STOP.

   With the locked build, Messages shows `cannot drive: MOTORS_LOCKED (HTTP 423)`; that is
   expected.
4. **Checks before continuing:**
   - `git status` matches the list above;
   - the full headless suite: `.\.venv\Scripts\python.exe -m unittest 01_lidar_visualizer
     02_grid_mapping 03_vector_mapping localisation_common 04_grid_localisation
     05_vector_localisation 06_scan_localisation test_application test_diagnose_lidar_serial
     test_workspace test_ui_theme test_workspace_localisation test_autonomy_protocol
     test_autonomy_link test_autonomy_wifi test_autonomy_http test_autonomy_teleop
     test_firmware_static test_autonomy_live_localisation`;
   - the compile check (command above).
5. **Never** flash `ESP32MecanumCar.ino`, and never unlock the motors before the physical gate.

## Confirmed facts vs assumptions
- **Confirmed** (files, Git or commands this session):
  - the Git state and commits above;
  - the compile results and test results listed;
  - AsyncTCP's 1250 ms watchdog sleep and the core's 5 s timeout (source and sdkconfig);
  - the library pins `mecanumCar(19, 21)` and registers 1-8 (code);
  - the course library's PWM 80 default (code);
  - the user's `wifi_secrets.h` unchanged by this session (hash only, never read).
- **User-reported, not observed here:**
  - the I²C controller type;
  - the S1 reboot loop and wheel spin;
  - Wi-Fi now working.
- **Assumed:**
  - the reboot loop was the watchdog conflict, not a brownout (no failing-boot serial log);
  - the I²C pins, address and registers on the physical robot match the library;
  - "upper" motors are the front ones.
