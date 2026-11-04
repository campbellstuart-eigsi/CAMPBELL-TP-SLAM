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
- **After S0 (uncommitted, awaiting review):**
  - Robot controller GUI: `robot_controller.py` (run this), `autonomy/controller_ui.py` (an
    embeddable `ttk.Frame`), `autonomy/teleop.py` (no-Tk core, all networking on worker
    threads). SIMULATOR by default (fake robot in a simulated room); REAL ROBOT needs the bench
    checks ticked before ARM. AZERTY default (Z/S, Q/D, A/E, Space); layouts configurable;
    300 ms leases; key release, focus loss and link loss clear intent; no command queue.
  - Firmware safety fix: `/arm` keeps the command seq, so a `/drive` delayed from before a STOP
    is refused after the next ARM (link and fake updated to match).
  - Upload package `firmware/ESP32SlamRobot/`: `wifi_secrets.h` (untracked, compile-time
    checks), `README.md`, `DEPENDENCIES.md`, `compile_check.py`.
  - S1 bench test paused: no access to the robot yet.
- **Phase B approach A preserved:** steps 1-2c in 364982a on `feature/autonomy-simulator`
  (protocol v2, firmware model, `ProtocolRobotLink`, simulated Wi-Fi). Step 3 is parked:
  untracked, uncompiled `firmware/autonomy_robot/src/crc.*` and `protocol.*` (v2). The old v1
  draft is backed up in the session scratchpad only.
- **Base:** Codes 01-06, the desktop app and the Workspace are unchanged here. M1 Stage 3
  continues on `feature/ui-redesign` (`C:\Users\stuar\Desktop\SLAM`).

## Next steps
1. Your review of the controller and the upload package; a commit only with your approval.
2. S1 bench test when the robot is available (`firmware/ESP32SlamRobot/README.md`, section 7).
3. S2 calibration, then S3-S5 (not started).

## Issues and safety blockers
- **Unknown, blocks floor tests:** does the motor board hold its last PWM when the ESP32 resets
  or hangs? S1 step 6 observes it.
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
| `python firmware\ESP32SlamRobot\compile_check.py --ld06-zip … --mecanum-zip … --libraries <scratch>` | PASSED: no secrets → `#error`; example password → `static_assert`; test password → 947,180 B flash (72 %), 93,952 B RAM (28 %) |

## Untested / unverified
- **Firmware: COMPILE-VERIFIED / HARDWARE-UNTESTED.** Never run on the robot: watchdog, leases,
  stops, wheel directions, Wi-Fi range and the motor board's reset behaviour are unverified.
- The controller in REAL ROBOT mode has only talked to the fake robot.
- Arduino IDE 2 itself was not used (arduino-cli, the IDE's own compiler, was).
- The parked v2 C++ (never compiled).
