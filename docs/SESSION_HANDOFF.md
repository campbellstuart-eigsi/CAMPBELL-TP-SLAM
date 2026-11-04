# Session handoff

_Last updated: 2026-11-04_ · _Session focus: Phase B autonomy, steps 1-2c (protocol, firmware model, RobotLink, simulated Wi-Fi) and the original robot files_

## Project overview and current state
- **The project:** LiDAR SLAM Workbench (EIGSI TP), an LD06 LiDAR on an ESP32-WROOM-32 Mecanum
  robot.
- **Phase B goal:** put the robot in a room, press Start, and it explores and maps the room alone.
  For now everything is simulated.
- **This branch, `feature/autonomy-simulator`,** works in the separate worktree
  `C:\Users\stuar\Desktop\SLAM-autonomy`. It is based on c65a78e and is independent of M1
  Stage 3, which continues on `feature/ui-redesign`.
- **Done:** steps 1, 2, 2b and 2c of `docs/specs/autonomy_phase_b.md`.
- **Next:** step 3, the firmware C++. Not started.

## Completed milestones
- Step 1: framed binary protocol and golden vectors (2026-11-04).
- Step 2: Python firmware safety model and mock robot link (2026-11-04).
- Step 2b: protocol v2 for Wi-Fi, and `ProtocolRobotLink` over a `Transport` (2026-11-04).
- Step 2c: simulated Wi-Fi network and its tests, including a 30-minute soak (2026-11-04).
- Inspection of the original robot files (read-only, in `C:\Users\stuar\Downloads`) and the
  resulting approved decisions (2026-11-04).

## Work performed this session
**Architecture (Python, no GUI, no hardware):**
```
explorer (future) ──RobotLink──> ProtocolRobotLink ──Transport──> SimSerialTransport  ─> FirmwareModel ─> plant
                                 (all host rules)                 SimNetworkTransport ─> FirmwareModel ─> plant
                                                                  SerialTransport / UdpTransport: NotImplementedError
```
**Protocol v2** (`docs/design/robot_protocol_v2.md`):
- **Frame:** `A5 5A | ver=2 | type | seq u16 | len u16 | CRC-8 header | payload ≤ 512 | CRC-16`.
- **Wi-Fi additions:**
  - HELLO session binding;
  - HEARTBEAT echoes the robot's uptime, rejected if more than 800 ms old or older than the last
    accepted echo;
  - MOVE `valid_until_ms` on the robot's clock, so a late MOVE is refused (EXPIRED);
  - SCAN_RESEND; STOP and DISARM are obeyed from anyone.
- **Interim limits:** 300 mm, 2000 ms per MOVE (turn ≤ 90°, ≤ 90°/s).
- **Scan points:** CCW 0.01°, distance in mm, intensity; 101 points per chunk.

**Firmware rules (`autonomy/firmware_model.py`; the C++ must match):**
- It boots DISARMED with the driver off. ARM needs a live heartbeat.
- 300 ms without a valid heartbeat, a motion timeout, a driver fault or an RX overflow latches
  FAULT. Leaving FAULT needs CLEAR_FAULT, then ARM.
- STOP zeroes the outputs in the same pass. A replayed MOVE is STALE.
- A restart has a new boot_id and comes back DISARMED.

**Host rules (`autonomy/robot_link.py`):**
- Retries reuse the same seq and bytes. STALE_SEQ for the current MOVE means "accepted".
- STOP is re-sent every 50 ms, from any thread.
- On link loss, `move()` never returns while the robot may move: STOP, then it starves the
  watchdog if STOP isn't acknowledged.
- The outcome is inferred from TELEMETRY if MOVE_DONE is lost.
- The host refuses a robot reporting a PHYSICAL driver.

**Simulated Wi-Fi (`autonomy/sim_network.py`):**
- Seeded latency and jitter (which reorders), Bernoulli and Gilbert-Elliott loss, duplicates,
  corruption, bandwidth, hold windows, and full or one-way cuts.
- `PROFILES` "ideal", "typical" and "poor" are **assumptions**.

**Bugs found by tests and fixed:**
1. A corrupted length stalled the decoder. Fixed with the header CRC-8.
2. One-way loss let `move()` return while the robot moved.
3. A false "silent link" was reported when the first telemetry frame was lost.
4. **A duplicated MOVE made the host report "refused" while the robot was moving.**

**Original robot findings** (`docs/research/original_robot_firmware_findings.md`, approved):
- **Original firmware** `ESP32RobotControl.ino`: SoftAP `RobotESP32`, HTTP GET API, timed moves
  ≤ 2 s at a fixed PWM of 80. No arming, no watchdog, open CORS.
- **Motors:** library `MecanumCar_v2`, bit-banged I2C to a coprocessor at **0x30, SDA GPIO19,
  SCL GPIO21**.
  - PWM registers: 1/2 upper-right, 3/4 upper-left, 5/6 lower-right, 7/8 lower-left.
  - LED registers 0x09 and 0x0A.
  - ACKs are ignored, and there are no encoders.
- **LD06:** Serial2 RX GPIO33, PWM GPIO32 held HIGH, 230400 baud. Robot-Maker library (identical
  to `LD06.zip`). Angles are CCW (`360 − physical`), with the library's (i + 0.5) spacing.
- **`support lidar.stl`:** a flat 132 × 68 × 3 plate; Ø4.2 holes at (±61, ±19), Ø2.5 holes at
  (±24.5, ±29), 8 × 40 slots at x = ±26.5. Not proven to be the installed part.

**Earlier, still Draft and unimplemented:** the live four-view mapping dashboard design
(`docs/design/live_mapping_dashboard.md`) and its measured update rates.

**Tools installed:**
- MSYS2 (winget `MSYS2.MSYS2`, `C:\msys64`) with UCRT64 GCC 16.2.0 via pacman. **Not on the
  global PATH.**
- Already present: Arduino CLI 1.5.1, bundled at
  `C:\Users\stuar\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`,
  and the ESP32 core `esp32:esp32` 3.3.12 (FQBN `esp32:esp32:esp32`).

## Important files and locations
| Path | Purpose |
|------|---------|
| `docs/specs/autonomy_phase_b.md` | Phase B plan: file structure, steps 1-10, H1-H3, approved changes (section 5c) |
| `docs/design/robot_protocol_v2.md` | the protocol (v1 doc kept, superseded) |
| `docs/design/wifi_transport.md` | Wi-Fi architecture; simulation results (section 5b) |
| `docs/design/live_mapping_dashboard.md` | future four-view live dashboard and Workspace integration |
| `docs/research/esp32_wifi_transport.md` | SoftAP vs station, with Espressif sources |
| `docs/research/original_robot_firmware_findings.md` | original robot firmware, MecanumCar_v2, STL |
| `docs/decisions/2026-11-04_*.md` | 5 approved decisions |
| `autonomy/protocol.py` | framing, messages, `validate_move`, `points_to_sensor_xy`, golden vectors, C header writer |
| `autonomy/firmware_model.py` | reference firmware state machine (the C++ must match it) |
| `autonomy/robot_link.py` | `RobotLink`, `ProtocolRobotLink`, transports, `MockRobotLink` |
| `autonomy/sim_network.py` | simulated Wi-Fi, `SimNetworkTransport` |
| `test_autonomy_protocol.py`, `test_autonomy_link.py`, `test_autonomy_wifi.py` | 109 tests |
| `firmware/tests/protocol_vectors.h` | generated golden vectors (`python -m autonomy.protocol --write-c-header firmware/tests/protocol_vectors.h`) |
| `firmware/autonomy_robot/src/protocol.h/.cpp` | **untracked, uncompiled v1 draft: rewrite in step 3** |
| `C:\Users\stuar\Downloads\ESP32RobotControl.ino`, `MecanumCar_v2.zip`, `support lidar.stl` | original robot files: read-only, never modify or flash |

## Tests and verification
| Date | Command or check | Result |
|------|------------------|--------|
| 2026-11-04 | `.\.venv\Scripts\python.exe -m unittest test_autonomy_protocol` / `test_autonomy_link` / `test_autonomy_wifi` | 33 / 60 / 16 tests, OK |
| 2026-11-04 | full headless suite (the AGENTS.md list + the three autonomy modules), run in the worktree | 540 tests, OK (431 existing + 109 autonomy; 254 s) |
| 2026-11-04 | 30-minute soak, poor profile (in `test_autonomy_wifi`) | 938 moves completed, 87 faults, 48 link lost, 17 stopped (all safe); refused: 58 EXPIRED, 56 FAULT_ACTIVE, 3 other; 519 scans ok, 10 failed; **0 invariant violations, 0 returns from `move()` while the robot moved** |
| 2026-11-04 | `g++ -std=c++17 -Wall -Wextra -Werror` on a test file including `protocol_vectors.h` | compiled and ran (vectors from before the step 2b/approved changes; regenerated since) |
| 2026-11-04 | `arduino-cli board details -b esp32:esp32:esp32` | ESP32 Dev Module, core 3.3.12 |

Not verified this session:
- Any firmware C++ (none written or compiled at v2), and any ESP32 build.
- Real Wi-Fi and all hardware (not allowed without approval).
- GUI suites and `gui_smoke_test.py` (no GUI file changed on this branch).

## Known bugs, limitations and open questions
- **Safety:** what the motor coprocessor does on an ESP32 reset or I2C loss is unknown (it may
  hold the last PWM). It must be checked in H2 before any motion.
- **Safety:** never flash `ESP32MecanumCar.ino`; it moves on boot.
- **Open hardware facts** (findings section 7):
  - the coprocessor chip;
  - which motor is which, and the roller layout;
  - wheel diameter, gearing, PWM → speed;
  - battery and power;
  - any physical power cut-off or emergency stop;
  - robot footprint, LiDAR offset and the LD06's 0° direction.
- **Licence:** the LD06 library licence is unconfirmed (README MIT, no LICENSE file). The
  MecanumCar_v2 library has none.
- The Wi-Fi profiles are assumptions. 4.8 % EXPIRED on "poor" means the TTL needs checking in H1.
- Residual protocol risk: a false sync with a valid CRC-8 (1 in 256) can stall a serial frame.
  A 50 ms inter-byte timeout is proposed for the firmware.
- `firmware_model.py` does not model loop-time jitter or the cost of I2C writes.
- **Mergeability:** this branch replaces `docs/STATUS.md`. Merging with `feature/ui-redesign`
  will conflict in that file; resolve it by hand. No other shared file is changed.

## Decisions and rationale
- **SoftAP + UDP primary, serial fallback:** untethered; self-contained; no TCP head-of-line
  blocking. (`docs/decisions/2026-11-04_wifi_softap_udp_transport.md`)
- **CCW scan angles:** matches the LD06 library and the original robot.
  (`docs/decisions/2026-11-04_scan_angles_ccw.md`)
- **Mecanum I2C driver, ACK-checked, mock until H2.**
  (`docs/decisions/2026-11-04_motor_driver_mecanum_i2c.md`)
- **Reuse the LD06 library as an external dependency only.**
  (`docs/decisions/2026-11-04_ld06_library_reuse.md`)
- **Interim 300 mm / 2000 ms per MOVE; no HTTP server in the new firmware.**
  (`docs/decisions/2026-11-04_interim_motion_limits_and_no_http.md`)
- **Independent localiser planned:** call Code 04 directly; don't use Stage 3's adapter (spec
  section 2). No decision record yet.

## Uncommitted changes and Git status
After this session's commit, only `firmware/autonomy_robot/src/protocol.h` and `protocol.cpp`
remain untracked (the unverified v1 draft, deliberately not committed).
- There is no root `.gitignore` (an open M1 decision), so stage files explicitly by path.
- No `__pycache__` folders were present in this worktree when it was checked.
- The pushed branch head contains everything described above: steps 1-2c, the docs and the
  109 tests.

## Suggested spec status changes (awaiting approval)
- `docs/specs/autonomy_phase_b.md`: Draft → Approved. The plan and each later change were
  approved in this session ("Approve, start step 1", the dashboard design, the Wi-Fi design, the
  findings).
- `docs/design/robot_protocol_v2.md`, `wifi_transport.md`, `live_mapping_dashboard.md`: Draft →
  Approved, on the same basis.

## Next recommended milestone
**Recommendation:** step 3, the firmware C++ at protocol v2, host-tested. Its behaviour is fully
specified by `firmware_model.py`, and it is the prerequisite for H1.

## How to resume
1. **Open the right folder:** work in `C:\Users\stuar\Desktop\SLAM-autonomy` (branch
   `feature/autonomy-simulator`), **not** in `C:\Users\stuar\Desktop\SLAM` (Stage 3).
   - If the session's folder is the original, add the worktree folder (Read is blocked outside the
     session folders).
   - Read `AGENTS.md`, this file, `docs/STATUS.md`, then spec sections 5c and 6, and
     `docs/design/robot_protocol_v2.md`.
2. **Environment:**
   - Python is `C:\Users\stuar\Desktop\SLAM\.venv\Scripts\python.exe`; the worktree has no
     `.venv`.
   - For g++, prepend `C:\msys64\ucrt64\bin` to PATH **in the command only**.
   - Arduino CLI is at the path above, with FQBN `esp32:esp32:esp32`. **Compile only: never
     `upload`, never COM7.**
3. **Start step 3:**
   - Delete or rewrite the untracked v1 draft `firmware/autonomy_robot/src/protocol.h/.cpp` at
     v2.
   - Then write `safety.h/.cpp` mirroring `FirmwareModel` (including the v2 echo, expiry, binding
     and SCAN_RESEND rules, and per-datagram decoding), `motor_driver.h`,
     `mock_motor_driver.h`, `config.h` (`MOTOR_OUTPUTS_ENABLED 0`, with `#error` otherwise) and
     the thin `autonomy_robot.ino`.
   - Add a Python generator that records FirmwareModel traces (bytes in, time steps, expected
     bytes out, state, faults, motor outputs) into `firmware/tests/model_traces.h`, and a
     Python test that keeps the header current.
   - Add `firmware/tests/host_tests.cpp`, which replays `protocol_vectors.h` and the traces byte
     for byte, and a `run_host_tests.ps1`.
   - Use LD06 through the library behind a scan-source interface, with CCW angles, compiled
     only.
4. **Check before continuing:**
   - Regenerate and diff the vectors header (`test_autonomy_protocol` checks it).
   - Run the 109 autonomy tests.
   - Confirm `git status` shows only the untracked v1 draft.

## Confirmed facts vs assumptions
- **Confirmed** (files, Git or commands this session): every path above; the test results above;
  the tool versions; the original robot files' contents and SHA-256 hashes; the STL geometry; the
  LD06 library's CCW conversion (`ld06.cpp:137`).
- **Assumed:**
  - the STL units are mm;
  - "Upper" motors are the front ones;
  - the 0x30 device is a microcontroller that accepts hardware I2C;
  - the simulated Wi-Fi profiles;
  - the Arduino loop runs on core 1 and Wi-Fi on core 0 (to verify on the board).
