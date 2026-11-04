# Project status

_Last updated: 2026-11-04_ · _Branch `feature/http-robot-mvp`, worktree `C:\Users\stuar\Desktop\SLAM-autonomy`_

Current project state only. Keep under ~60 lines; replace outdated items rather than appending.
The fuller briefing is `docs/SESSION_HANDOFF.md`.

## Progress
- **Approach B (HTTP MVP):** ESP32 firmware `firmware/ESP32SlamRobot/` plus a Python controller
  (`robot_controller.py`). Decisions: `docs/decisions/2026-11-04_mvp_http_firmware.md`,
  `2026-11-04_simplified_controller_auto_arm.md`.
- **Pushed to `origin/feature/http-robot-mvp`:**
  - 6cbefa9: S0 firmware and `HttpRobotLink`;
  - 221c778: controller and upload package;
  - 275782b: diagnostic firmware after the S1 reboot loop;
  - 2cdd3e3: headless live-localisation tracker.
- **At the robot (user-reported):** S1 reboot loop, then (with the diagnostic build) Wi-Fi works.
  The controller has not moved the robot.
- **Uncommitted:**
  - Firmware motor diagnostics: I²C ACK read with a pull-up; write-free 0x30/0x31 probe;
    `/status` and serial report refusals, move ends, applied PWM and I²C counters.
    `MOTORS_LOCKED 1`.
  - Simplified controller: connect, drive, stop. No ARM/DISARM buttons; automatic arming on the
    first key press; REAL ROBOT by default.
  - The controller shows the firmware's exact refusal reasons.

## Next steps
1. Your review; commit and push only with your approval.
2. At the robot, still locked:
   - boot log and `[I2C] VERDICT`;
   - `/status` (`i2c_ack_0x30`, `i2c_nacks`, `scan_age_ms`);
   - the controller shows `cannot drive: MOTORS_LOCKED`.

   Firmware README section 7 (S1a-motor).
3. Physical gate, then `MOTORS_LOCKED 0` and S1b (wheels in the air).
4. Software: tracker on the real classroom map, then the Live Localisation workspace.

## Issues and safety blockers
- **Blocks all movement:**
  - an S1 wheel spun with no command from this firmware;
  - whether the I²C controller holds PWM across ESP32 resets is unknown;
  - the motor-power cut-off is unverified.

  Code cannot prove that motor power is off.
- **Speed:** the controller's 30 % is PWM 24 of 255 (cap 50 % = PWM 40); the course library
  drives at PWM 80. The wheels may not turn at 30 %.
- **I²C facts:** the controller type is confirmed (user, from the professor). 0x30, GPIO19/21 and
  the registers come from the library code only.
- The reboot-loop cause (watchdog vs brownout) was not confirmed from a serial log; the
  diagnostic build boots.
- Never flash `ESP32MecanumCar.ino` (it moves on boot).
- Local, never committed: `wifi_secrets.h`, `ld06.*` (licence open), `MecanumCar_v2.*` (no
  licence).
- From M1: GUI suite b instability; font licence and root `.gitignore` decisions open.

## Key decisions
- Codes 01-06, `localisation_common.py`, `scans/` and the original SLAM checkout are never
  modified.
- No flashing, COM port or motor activation by Claude.

## Verified tests (2026-11-04, this worktree, uncommitted tree)
| Command | Result |
|---------|--------|
| `python firmware\ESP32SlamRobot\compile_check.py --ld06-zip … --mecanum-zip … --libraries <scratch>` | PASSED: 952,240 B flash (72 %), 94,056 B RAM (28 %); no-secrets and example-password builds refused |
| `-m unittest test_firmware_static test_autonomy_http test_autonomy_teleop` | 51 OK |
| `-m unittest test_gui_robot_controller` | 14 OK, 3 runs in a row |
| Full headless suite (AGENTS.md list + autonomy + firmware static + live localisation), after the controller simplification | 600 OK (301 s) |

## Untested / unverified
- Everything on hardware: motor diagnostics, I²C answers, wheel motion, PWM dead-band, the
  watchdog and the stall check.
- M1 GUI suites and `gui_smoke_test.py` (no Workspace file changed on this branch).
