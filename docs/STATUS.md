# Project status

_Last updated: 2026-11-04_ · _Branch `feature/http-robot-mvp` (from b48037a), worktree `C:\Users\stuar\Desktop\SLAM-autonomy`_

Current project state only. Keep under ~60 lines; replace outdated items rather than appending.
The fuller briefing is in `docs/SESSION_HANDOFF.md` (written before the MVP change).

## Progress
- **Approach B (MVP) approved:** the robot runs the HTTP firmware `firmware/ESP32SlamRobot/`;
  Python drives it with `autonomy/http_robot_link.py`. Decision
  `docs/decisions/2026-11-04_mvp_http_firmware.md`. Gates: S0 desk → S1 bench → S2 manual
  driving + calibration → S3 drive-stop-scan-localise → S4 frontier exploration → S5 Workspace.
- **S0 (desk) committed in 6cbefa9:** sketch fixes, `HttpRobotLink`, fake robot, 17 tests.
- **Committed in 221c778:** robot controller GUI (`robot_controller.py`), teleop core, upload
  package, `/arm` keeps the command seq.
- **S1 (2026-11-04, user):** upload OK, then a reboot loop (`rst:0xc SW_CPU_RESET`), the AP
  never appeared, and one wheel spun before and after the upload. Controller not connected.
- **After S1 (uncommitted, awaiting review): diagnostic firmware** (COMPILE-VERIFIED /
  HARDWARE-UNTESTED). Investigation: `docs/research/2026-11-04_s1_reboot_investigation.md`.
  - Likely cause (code evidence, unconfirmed): the sketch shortened the shared task watchdog to
    1 s, but AsyncTCP's task sleeps up to 1.25 s between resets. Brownout is the alternative;
    the serial log decides.
  - Fix: the core's 5 s shared watchdog, plus a 300 ms motor-task stall check. `[BOOT]`
    checkpoints, reset reason, reset-loop count, motor I²C ACK at boot. `MOTORS_LOCKED 1`:
    `/arm` refused.
  - `test_firmware_static.py` (6 checks; fails on 221c778 as intended).
- **Hardware facts:** the I²C motor controller type is confirmed (user, from the professor).
  Address 0x30, GPIO19/21 and the register map come from the library code only, **not
  checked on the robot**.
- **Phase B approach A preserved:** steps 1-2c in 364982a on `feature/autonomy-simulator`
  (protocol v2, firmware model, `ProtocolRobotLink`, simulated Wi-Fi). Step 3 is parked:
  untracked, uncompiled `firmware/autonomy_robot/src/crc.*` and `protocol.*` (v2). The old v1
  draft is backed up in the session scratchpad only.
- **Base:** Codes 01-06, the desktop app and the Workspace are unchanged here. M1 Stage 3
  continues on `feature/ui-redesign` (`C:\Users\stuar\Desktop\SLAM`).

## Next steps
1. Your review of the diagnostic firmware; a commit only with your approval.
2. Next offline milestone (proposed): a headless live-localisation tracker (Code 04 on
   simulated scan sequences, measured against simulated truth), then the Live Localisation and
   Live Exploration workspaces.
3. At the robot: S1a with the diagnostic build (README section 7); S1b only after the cut-off
   and reset checks pass.

## Issues and safety blockers
- **Blocks all movement:**
  - a wheel spun at S1 without any command from this firmware (cause unknown);
  - whether the controller holds PWM across ESP32 resets is unknown;
  - a physical motor cut-off must be verified first.

  Code cannot prove that motor power is off.
- **No encoders:** `move()` refuses until a `Calibration` measured at S2 is given.
- **Late HTTP:** a `/drive` delayed in TCP can still start a lease late (300 ms from the
  controller, at most 1 s from the firmware); after STOP it is refused (DISARMED, or STALE after
  a re-ARM). The legacy course routes (`/forward` … , seq 0) bypass the seq check.
- **Open access point:** anyone joined can drive (CORS `*`); the sketch refuses the example password.
- **Never flash `ESP32MecanumCar.ino`:** it drives the motors for 5 s after boot.
- **LD06 library licence unconfirmed:** the `ld06.*` files stay out of Git (`.gitignore`).
- **Firmware libraries** (Async TCP 3.5.0, ESP Async WebServer 3.12.1, MecanumCar_v2) are only in
  the scratchpad here; the flashing PC needs them installed (`DEPENDENCIES.md`).
- **From M1:** GUI suite b is unstable; the font zip licence and the root `.gitignore` await a decision.

## Key decisions
- Phase B never modifies Codes 01-06, `localisation_common.py`, `scans/` or the Stage 3 files.
- `docs/decisions/2026-11-04_*`; the newest is the MVP decision (approach B).
- No flashing, COM port or robot operation by Claude; hardware stages only with your approval.

## Verified tests (2026-11-04, this worktree)
| Command | Result |
|---------|--------|
| AGENTS.md headless suite + `test_autonomy_protocol test_autonomy_link test_autonomy_wifi test_autonomy_http test_autonomy_teleop` | 578 OK (280 s) |
| `.\.venv\Scripts\python.exe -m unittest test_gui_robot_controller -v` | 12 OK (18 s) |
| `.\.venv\Scripts\python.exe -m unittest test_gui_map_display test_gui_workspace -v` | 12 OK (78 s), one run |
| `.\.venv\Scripts\python.exe gui_smoke_test.py` | PASS |
| `compile_check.py` on the 221c778 build | PASSED (947,180 B flash, 72 %) |
| `python firmware\ESP32SlamRobot\compile_check.py --ld06-zip … --mecanum-zip … --libraries <scratch>` on the diagnostic build (from the worktree, after restoring `wifi_secrets.example.h`) | PASSED: no secrets → `#error`; example password → `static_assert`; test password → 950,032 B flash (72 %), 93,992 B RAM (28 %). `--warnings all`: only the known `ld06.cpp` warning |
| `-m unittest test_firmware_static test_autonomy_http test_autonomy_teleop test_autonomy_protocol test_autonomy_link test_autonomy_wifi` (after the diagnostic changes) | 153 OK |

## Untested / unverified
- **Firmware: COMPILE-VERIFIED / HARDWARE-UNTESTED.** Never run on the robot: watchdog, leases,
  stops, wheel directions, Wi-Fi range and the motor board's reset behaviour are unverified.
- The controller in REAL ROBOT mode has only talked to the fake robot.
- Arduino IDE 2 itself was not used (arduino-cli, the IDE's own compiler, was).
- The parked v2 C++ (never compiled).
