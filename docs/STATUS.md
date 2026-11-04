# Project status

_Last updated: 2026-11-04 (paused until Friday 2026-11-06)_ · _Branch `feature/workspace-drive`_

Current project state only. Keep under ~60 lines; replace outdated items rather than appending.
The fuller resume briefing is `docs/SESSION_HANDOFF.md`.

## Progress
- **Git:** `feature/workspace-drive` at **ca6826b**, pushed to `origin/feature/workspace-drive`
  on 2026-11-04 with your approval. It is based on 832e7ae (`feature/ui-redesign`, unchanged).
  `feature/http-robot-mvp` (74711dc, worktree `SLAM-autonomy`) is untouched and unused.
- **Software, verified on the PC:**
  - the seven-page app (`00_main_app.py`);
  - the four-view Workspace and Codes 01-06 mapping and localisation;
  - the new **Drive (robot)** panel: `robot_drive.py` plus the panel in `workspace_page.py`;
  - firmware `firmware/WorkspaceDrive/` (compile only).
- **Design:** `docs/specs/workspace_drive.md` (Approved); decision
  `docs/decisions/2026-11-04_workspace_drive_restart.md`.
- **Hardware:** nothing verified with the new firmware. It has not been flashed.
- M1 Gate 3 (Workspace localisation) is still not formally accepted (see `SESSION_HANDOFF.md`).

## Hardware blockers (none solved)
- The ESP32 stopped working when USB was unplugged, with the battery connected.
- The front-right wheel spun unexpectedly (earlier firmware sent only zero speed).
- The motor-controller board type is not physically confirmed (I²C 0x30 or direct PWM).
- I²C to the motor board is not independently verified; the library ignores ACKs.
- The battery switch is not yet shown to stop the motors reliably.

## Friday 2026-11-06: first action
Gate A at the robot (wheels off the ground, no movement commands): identify the motor board.
Full checklist: `docs/SESSION_HANDOFF.md`, "How to resume".

## Priorities
1. Reliable battery power and motor control.
2. ESP32 ↔ Python Wi-Fi link.
3. Real ZQSD driving in the Workspace.
4. Then live LiDAR localisation. Not now: autonomy, exploration, path planning, new mapping
   or UI redesign.

## Key decisions
- Codes 01-06, `localisation_common.py` and `scans/` are never modified.
- One HTTP contract (the professor's routes) for I²C and PWM robots; only I²C implemented.
- No ARM/DISARM or motor lock. Safety = the firmware's 500 ms timeout, the panel's stop rules
  and the battery switch.

## Verified tests (2026-11-04)
| Command | Result |
|---------|--------|
| `arduino-cli compile --fqbn esp32:esp32:esp32` professor's `ESP32RobotControl.ino`, unchanged | OK, 944,180 B (72 %) |
| same for `firmware/WorkspaceDrive` (scratch copy, test secrets, `--warnings all`) | OK, 942,204 B (71 %); a short password is refused |
| `.\.venv\Scripts\python.exe -m unittest test_robot_drive -v` | 11 OK |
| `.\.venv\Scripts\python.exe -m unittest test_gui_workspace_drive -v` | 5 OK (2 runs) |
| `... -m unittest test_workspace test_workspace_localisation test_robot_drive` | 33 OK (before the `tick_drive` fix) |
| `... -m unittest test_robot_drive test_gui_workspace_drive test_workspace` | 19 OK (after it) |
| `.\.venv\Scripts\python.exe gui_smoke_test.py` | PASS (after the fix) |

## Untested
- Everything on the robot: board, power, I²C, wheel directions, the 500 ms stop, real Wi-Fi.
- Full suites a and b, not re-run on this branch.
