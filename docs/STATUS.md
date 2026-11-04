# Project status

_Last updated: 2026-11-04_ · _Branch `feature/http-robot-mvp` (from b48037a), worktree `C:\Users\stuar\Desktop\SLAM-autonomy`_

Current project state only. Keep under ~60 lines; replace outdated items rather than appending.
The fuller briefing is in `docs/SESSION_HANDOFF.md` (written before the MVP change).

## Progress
- **Approach B (MVP) approved:** the robot runs the HTTP firmware `firmware/ESP32SlamRobot/`;
  Python drives it with `autonomy/http_robot_link.py`. Decision
  `docs/decisions/2026-11-04_mvp_http_firmware.md`. Gates: S0 desk → S1 bench → S2 manual
  driving + calibration → S3 drive-stop-scan-localise → S4 frontier exploration → S5 Workspace.
- **S0 (desk) done, not committed:** sketch fixes (motor-task watchdog; `MoveResult` declared
  early so it compiles), ESP32 compile check, `HttpRobotLink`, `autonomy/fake_http_robot.py`,
  `test_autonomy_http.py` (17 tests).
- **Phase B approach A preserved:** steps 1-2c in 364982a on `feature/autonomy-simulator`
  (protocol v2, firmware model, `ProtocolRobotLink`, simulated Wi-Fi). Step 3 is parked:
  untracked, uncompiled `firmware/autonomy_robot/src/crc.*` and `protocol.*` (v2). The old v1
  draft is backed up in the session scratchpad only.
- **Base:** Codes 01-06, the desktop app and the Workspace are unchanged here. M1 Stage 3
  continues on `feature/ui-redesign` (`C:\Users\stuar\Desktop\SLAM`).

## Next steps
1. Your review of S0, and a commit only with your approval.
2. S1 bench test (you flash; wheels off the ground): `firmware/ESP32SlamRobot/README.md`.
3. S2-S5 (not started).

## Issues and safety blockers
- **Unknown, blocks floor tests:** does the motor board hold its last PWM when the ESP32 resets
  or hangs? S1 step 6 observes it.
- **No encoders:** `move()` refuses until a `Calibration` measured at S2 is given.
- **Late HTTP:** a `/drive` delayed in TCP can still start a lease (at most 1 s) late.
- **Open access point:** anyone joined can drive (CORS `*`); set a unique SSID and password.
- **Never flash `ESP32MecanumCar.ino`:** it drives the motors for 5 s after boot.
- **LD06 library licence unconfirmed:** the `ld06.*` files stay out of Git (`.gitignore`).
- **Firmware libraries** (Async TCP 3.5.0, ESP Async WebServer 3.12.1, MecanumCar_v2) are only in
  the scratchpad here; the flashing PC needs them installed.
- **From M1:** GUI suite b is unstable; the font zip licence and the root `.gitignore` await a decision.

## Key decisions
- Phase B never modifies Codes 01-06, `localisation_common.py`, `scans/` or the Stage 3 files.
- `docs/decisions/2026-11-04_*`; the newest is the MVP decision (approach B).
- No flashing, COM port or robot operation by Claude; hardware stages only with your approval.

## Verified tests (2026-11-04, this worktree)
| Command | Result |
|---------|--------|
| `.\.venv\Scripts\python.exe -m unittest test_autonomy_http -v` | 17 OK (17 s) |
| AGENTS.md headless suite + `test_autonomy_protocol test_autonomy_link test_autonomy_wifi test_autonomy_http` | 557 OK (251 s) |
| `arduino-cli compile -b esp32:esp32:esp32` of `ESP32SlamRobot` (scratch libraries) | exit 0; 72 % flash, 28 % RAM; with `--warnings all` the only warning is in `ld06.cpp` |
| Regenerated `protocol_vectors.h` vs the committed one | identical |

## Untested / unverified
- All hardware: the sketch has never run on the robot; the watchdog, leases and stops are
  untested on the ESP32.
- The parked v2 C++ (never compiled). GUI suites and `gui_smoke_test.py` (no GUI file changed).
