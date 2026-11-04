# Project status

_Last updated: 2026-11-04_ · _Branch `feature/autonomy-simulator`, worktree `C:\Users\stuar\Desktop\SLAM-autonomy`_

Current project state only. Keep under ~60 lines; replace outdated items rather than appending.
The fuller briefing is in `docs/SESSION_HANDOFF.md`.

## Progress
- **Base:** c65a78e (end of M1 Stage 2). Codes 01-06, the desktop app and the Workspace are
  unchanged here. M1 Stage 3 continues separately on `feature/ui-redesign`
  (`C:\Users\stuar\Desktop\SLAM`).
- **Phase B (simulated autonomous mapping), steps 1-2c done** (`docs/specs/autonomy_phase_b.md`):
  - protocol v2: `autonomy/protocol.py`;
  - firmware safety model: `autonomy/firmware_model.py`;
  - `ProtocolRobotLink` over a `Transport`: `autonomy/robot_link.py`;
  - simulated Wi-Fi: `autonomy/sim_network.py`.
- **Transport:** Wi-Fi SoftAP + UDP primary, USB serial fallback. The real links raise
  `NotImplementedError`.
- **Robot hardware:** motor I2C coprocessor at 0x30 (SDA GPIO19, SCL GPIO21); LD06 on Serial2
  RX33, PWM32; CCW scan angles. See `docs/research/original_robot_firmware_findings.md`.
- **Tools:** MSYS2 UCRT64 g++ 16.2.0 (`C:\msys64\ucrt64\bin`, not on PATH); Arduino CLI 1.5.1
  (bundled with the IDE); ESP32 core 3.3.12.

## Unfinished work
- **Step 3 (firmware C++ at v2): not started.** The untracked
  `firmware/autonomy_robot/src/protocol.h/.cpp` are an uncompiled v1 draft; rewrite them.
- **Steps 4-10:** world and robot simulation, mapping, localiser, frontiers and A*, explorer,
  dashboard.

## Next steps
1. Step 3, following `docs/SESSION_HANDOFF.md` "How to resume".
2. Steps 4-10 (spec section 6). Hardware stages H1-H3 only with explicit approval.

## Issues and safety blockers
- **Unknown:** what the motor coprocessor does on an ESP32 reset or I2C loss (it may hold the last
  PWM). Test in H2 before any motion.
- **Never flash `ESP32MecanumCar.ino`:** it drives the motors for 5 s after boot.
- **No encoders:** PWM → speed must be calibrated in H2.
- **LD06 library licence unconfirmed** (README MIT, no LICENSE file). Don't copy the library in.
- **Simulated Wi-Fi profiles are assumptions:** 4.8 % of MOVEs expired on "poor"; check the
  250 ms TTL in H1.
- **From M1:** GUI suite b is unstable; the font zip licence and the missing root `.gitignore`
  await a decision.

## Key decisions
- Phase B never modifies Codes 01-06, `localisation_common.py`, `scans/` or the Stage 3 files.
- `docs/decisions/2026-11-04_*`: SoftAP/UDP; CCW angles; Mecanum I2C driver (mock until H2);
  LD06 library reuse; 300 mm / 2000 ms per MOVE; no HTTP server.
- No hardware connection, flashing or real motor output without your approval.

## Verified tests
| Date | Command | Result |
|------|---------|--------|
| 2026-11-04 | `.\.venv\Scripts\python.exe -m unittest test_autonomy_protocol test_autonomy_link test_autonomy_wifi` | 33 + 60 + 16 = 109, OK |
| 2026-11-04 | AGENTS.md headless suite + the 3 autonomy modules, in this worktree | 540 tests, OK (431 + 109; 254 s) |

## Untested / unverified
- All C++ and any ESP32 build; real Wi-Fi; all hardware.
- GUI suites and `gui_smoke_test.py` (not run; no GUI file changed on this branch).
