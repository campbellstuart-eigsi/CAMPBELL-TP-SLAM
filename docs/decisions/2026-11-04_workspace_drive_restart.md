# Decision: restart robot driving from the clean Workspace, on the professor's HTTP routes

_Date: 2026-11-04 · Status: decided by the user · Spec: `docs/specs/workspace_drive.md` (Approved)_

## What
- Robot driving restarts on a new branch, `feature/workspace-drive`, created from 832e7ae (the
  four-view Workspace with one-shot localisation, no driving).
- The firmware is the professor's `ESP32RobotControl.ino`, with motor calls behind a small
  adapter (`motor_i2c.h`), a 500 ms longest move, an optional `speed` and a local Wi-Fi secret.
- Python drives the robot only through the professor's routes (`/forward`, `/backward`, `/left`,
  `/right`, `/rotateleft`, `/rotateright` with `?time=&speed=`, `/stop`, `/ping`), from a small
  "Drive (robot)" panel in the Workspace.
- One HTTP contract for both classroom robot types (I²C and direct PWM). Only I²C is
  implemented. There is no auto-detection and no second firmware copy.

## Why
- A full day on the custom HTTP MVP firmware (`feature/http-robot-mvp`, worktree
  `SLAM-autonomy`) did not move the robot. Its locked diagnostic build (`MOTORS_LOCKED 1`)
  refused every movement by design, and layers of arming and diagnostics made it slow to reach a
  first drive.
- The professor's sketch is the known-good reference other students use, and it already has
  timed moves, `/stop` and LD06 scans.
- The motor type stays in firmware, so the same panel works with either robot type.

## Alternatives considered
- **Keep the HTTP MVP and unlock it (`MOTORS_LOCKED 0`):** rejected by the user in favour of
  a clean start.
- **A separate controller window (`robot_controller.py`):** the user wants driving inside the
  existing Workspace.
- **ARM/DISARM and a compile-time motor lock:** not wanted in the normal interface. Safety comes
  from the firmware's 500 ms timeout, the panel's stop rules and the physical battery switch.

## Consequences
- `feature/http-robot-mvp` and the `SLAM-autonomy` worktree are kept, untouched, for
  read-only reference.
- A PWM robot needs a `motor_pwm.h`, once the hardware facts in the spec's section 5 are known.
