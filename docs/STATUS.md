# Project status

_Last updated: 2026-11-04_ · _Branch `feature/workspace-drive` (from 832e7ae)_

Current project state only. Keep under ~60 lines; replace outdated items rather than appending.
The fuller resume briefing is `docs/SESSION_HANDOFF.md` (it describes 832e7ae; not yet updated).

## Progress
- **Restart of robot driving from the clean Workspace (832e7ae).** The earlier HTTP MVP on
  `feature/http-robot-mvp` (worktree `SLAM-autonomy`) is left untouched and is not used.
- Spec **Approved**: `docs/specs/workspace_drive.md`. The panel speaks the professor's HTTP routes
  (`/forward` … `/rotateright` `?time=&speed=`, `/stop`, `/ping`). The motor type lives in a
  firmware adapter. Only I²C is implemented; PWM needs are listed in section 5.
- **Part 2 firmware** `firmware/WorkspaceDrive/`: the professor's `ESP32RobotControl.ino` +
  `motor_i2c.h`, 500 ms longest move, optional `speed`, own Wi-Fi name in `wifi_secrets.h`.
- **Part 3 Workspace Drive panel**: `robot_drive.py` (worker thread, hold-to-drive) and a
  "Drive (robot)" group in `workspace_page.py`. Keys act only on the drive pad. Release,
  Space, focus loss, disconnect, close or a failed request send `/stop`.
- Nothing on this branch is committed yet.

## Next steps (at the robot, wheels in the air; the user flashes)
1. Part 1 Gate A, no movement commands: board type (I²C 0x30?), the battery powers the ESP32
   with USB unplugged, the battery switch stops a wheel, no spin at EN/reset.
2. Gate B: flash `firmware/WorkspaceDrive` (README), browser `/ping` and short moves, the
   reset-during-move check.
3. Drive from the Workspace panel (`00_main_app.py`, Workspace, Drive (robot)).
4. Commit with your approval.

## Issues
- **Unexplained front-right wheel spin** (earlier firmware sent only zero speed). Possibly a
  different motor board; hardware until shown otherwise. Blocks movement tests.
- **Wi-Fi disappears without USB:** the ESP32 is probably on USB power only.
- The professor's sketch uses the same SSID/password on every robot; ours uses `wifi_secrets.h`.
- The library never checks I²C ACKs: a wrong board fails silently.
- Earlier M1 items still open: GUI suite b instability (desktop minimises test windows), font
  licence, root `.gitignore`.

## Key decisions
- Codes 01-06, `localisation_common.py` and `scans/` are never modified.
- One HTTP contract for I²C and PWM robots; no auto-detection; PWM only once its hardware is known.
- No ARM/DISARM or motor lock; safety = firmware 500 ms timeout + panel stop rules + battery switch.

## Verified tests (2026-11-04, this working tree)
| Command | Result |
|---------|--------|
| `arduino-cli compile --fqbn esp32:esp32:esp32` professor's `ESP32RobotControl.ino`, unchanged | OK, 944,180 B (72 %) |
| same, `firmware/WorkspaceDrive` (scratch copy, test secrets, `--warnings all`) | OK, 942,204 B (71 %); short password refused |
| `-m unittest test_robot_drive -v` | 11 OK |
| `-m unittest test_gui_workspace_drive -v` | 5 OK (2 runs) |
| `-m unittest test_workspace test_workspace_localisation test_robot_drive` | 33 OK (before the last `tick_drive` fix) |
| `-m unittest test_robot_drive test_gui_workspace_drive test_workspace` | 19 OK (after it) |
| `gui_smoke_test.py` | PASS after the fix (first run found the closed-page tick bug) |

## Untested / unverified
- Everything on hardware: the board type, I²C, power, wheel directions, the 500 ms stop and
  resets.
- Full suites a and b were not re-run.
