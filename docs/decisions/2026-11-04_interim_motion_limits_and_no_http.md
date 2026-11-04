# Decision: interim motion limits (2 s, 300 mm); no HTTP server in the new firmware

Date: 2026-11-04. Status: Approved by the user (findings report, recommendations 4 and 5).

**Motion limits:**
- `MAX_TIMEOUT_MS` drops from 3000 to **2000**, matching the original firmware's proven
  `MAX_MOVE_MS`.
- `MAX_FORWARD_MM` drops from 500 to **300**.
- These apply now, in simulation and in protocol v2, until motion is calibrated on hardware.
  Raising them later needs a new decision.
- Turn limits are unchanged (90°, 90°/s).
- Planners must split longer motions into several MOVEs.

**Firmware separation:**
- The new autonomy firmware has **no HTTP or web server**: no TCP stack load, and no
  unauthenticated browser control (the original uses `CORS *` GET endpoints).
- The original TP firmware (`ESP32RobotControl.ino`) and `RobotControl.html` stay unchanged, as a
  separate "legacy TP" setup.
- **`ESP32MecanumCar.ino` must never be flashed:** it drives the motors for 5 s after boot.

**Alternatives:**
- Keep 3 s / 500 mm: unproven on this robot.
- A compatibility HTTP mode in the new firmware: it would bring back the unsafe access model.
