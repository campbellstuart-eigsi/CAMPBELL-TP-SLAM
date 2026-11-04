# Phase B: simulated autonomous room mapping, and ESP32 robot firmware

Status: Draft
Written: 2026-11-04. Branch `feature/autonomy-simulator`, worktree
`C:\Users\stuar\Desktop\SLAM-autonomy`, based on c65a78e. Nothing is implemented yet.

## 1. Goal and limits
**Requirement added 2026-11-04: the robot runs untethered.**
- **Wi-Fi** (ESP32 SoftAP, UDP) is the transport for control, telemetry and scans.
- **USB serial** is a development and debugging fallback, with identical semantics.
- Design: `docs/design/wifi_transport.md`. Research: `docs/research/esp32_wifi_transport.md`.

The end goal: put the robot in a room, press Start, and it explores and maps the room by itself.
This phase is **simulation only**. It covers:
- a Python stop-and-scan explorer, running against a simulated room and robot, through a
  `MockRobotLink`;
- ESP32-WROOM-32 firmware (Arduino framework) that shares one documented protocol with Python.
  Its motor outputs are mocked, and it is not flashed.

Out of scope until you approve a separate hardware-verification stage:
- a real serial link, COM7, flashing
- real motor pins and drivers
- ROS 2
- changes to Codes 01-06, `localisation_common.py`, the `scans/` files, the baseline results, or
  the Workspace and desktop files

**Gaps:**
- The "autonomous room-mapping research report" was not found on disk. This plan does not rely
  on it. Please give its path.
- The motor driver, wheel layout (differential or Mecanum?), wiring and encoders are unknown.

## 2. Findings from the code inspection
- **Code 02** already supports incremental mapping:
  - `create_log_odds_map(spec)` + `integrate_scan_log_odds(map, spec, scan, points_global)`
    add one scan at a time;
  - `scan = {"pose": (x_m, y_m, theta_deg), ...}`.
  A growing map uses one fixed, pre-allocated spec (the room bounds plus a margin).
  Unvisited cells simply stay UNKNOWN.
- **Code 04:**
  - `GridMap.from_part2(log_odds_map, spec)` accepts any log-odds map;
  - `localise_scan(grid, points, initial_pose, config)` is the local search (±1 m, ±30°, not
    cancellable);
  - `localise_global(grid, points, progress, should_stop)` is the whole-map search.
  Both return a `LocalisationResult`.
- **Simulation utilities:**
  - `part1.simulate_scan_of_walls(walls, pose_deg, angle_step_deg, noise_mm, max_range_m, seed)`
    ray-casts wall segments;
  - `localisation_common.simulate_scan` wraps it in radians and gives about 450 points per
    revolution;
  - `part1.polygon_walls` / `box_walls` / `CLASSROOM_WALLS` describe rooms.
  All are reused unchanged.
- **Can the Stage 2 adapter take a growing grid?** Technically yes:
  `workspace_localisation.make_job(Request("grid", "local", points, label, guess_day1), GridMap)`
  works with any `GridMap`. **It is not used**, for these reasons:
  1. It imports `app_session`, which pulls in `workspace_inputs` and Codes 05/06.
  2. It belongs to the Stage 2/3 code that the other session is changing.
  3. Its `Request` carries Workspace semantics (Day 1 units, session revision).
  4. `grid_initial_pose` clips the guess to the grid.

  **Proposal:** an independent `autonomy/localiser.py` calls Code 04's public functions
  directly, with the same input preparation (`common.filter_ranges`). A parity test reads the
  adapter (read-only) and checks that both give identical results on the same inputs.
  Nothing in Stage 3 changes.
- **Existing firmware:**
  - the TP sketch `ESP32_LD06_Test.ino` / the LD06 library example streams Teleplot text over
    USB at 115200;
  - the LD06 is on Serial2 at 230400 baud (RX = GPIO33, PWM = GPIO32), per
    `Acquisition des données issues du Lidar LD06.md`, line 206.
  The new firmware is a **separate sketch folder**. The existing sketches are never touched.
- **Toolchain:**
  - `arduino-cli` (inside the Arduino IDE) and the esp32 core 3.3.12 are installed;
  - **no host C++ compiler** was found (gcc, clang and cl are all absent);
  - the worktree has no `.venv`, so tests use `C:\Users\stuar\Desktop\SLAM\.venv\Scripts\python.exe`
    (read-only use).
- **Bandwidth:** one 456-point scan is about 1.9 kB binary. At 115200 baud (about 11.5 kB/s),
  continuous 10 Hz streaming does not fit. Stop-and-scan therefore sends **one scan on request**,
  in chunks.

## 3. File structure (all new)
```
autonomy/                      Python package; no tkinter, no serial import
  __init__.py
  world.py          SimWorld: wall segments, obstacles, bounds; ray cast (Code 01);
                    robot-disc vs segment collision; ground-truth raster for metrics
  robot_sim.py      SimRobot: ground-truth pose, MotionNoise (scale/drift/heading error,
                    seeded), executes bounded moves in small sub-steps, stops before contact
  protocol.py       Protocol v1: frame encode, streaming decoder (resync, CRC), messages
  firmware_model.py Python reference model of the firmware safety state machine and watchdog
  robot_link.py     RobotLink interface; MockRobotLink (frames -> firmware_model -> SimRobot);
                    SerialRobotLink is declared but raises NotImplementedError
  mapping.py        GrowingGrid: fixed spec, Code 02 log-odds integration, -> Code 04 GridMap
  localiser.py      Code 04 local search seeded by odometry; global-search recovery; policy
  frontiers.py      frontier cells, 8-connected clusters, selection (path cost vs size)
  planner.py        obstacle inflation, unknown-as-blocked, 8-connected A*, path ->
                    bounded (turn, forward) segments
  explorer.py       Explorer state machine (section 5), driven by step(); ground truth unseen
  scenarios.py      test rooms: empty box, L-room with obstacles, Code 01 classroom
  metrics.py        map accuracy and coverage against ground truth (tests and reports only)
  visualise.py      Matplotlib viewer: map, robot, frontiers, path, trajectories; buttons
                    Start/Pause/Resume/Stop; headless PNG/report mode
autonomy_sim.py                entry point: `python autonomy_sim.py [--scenario] [--headless]`
test_autonomy_protocol.py      framing, CRC, malformed/truncated/oversize, resync, golden vectors
test_autonomy_link.py          ARM/DISARM/STOP, watchdog expiry, comm loss, restart, seq replay
test_autonomy_world.py         ray cast, collision prevention, bounded moves, noise determinism
test_autonomy_mapping.py       incremental map = Code 02 batch map; GridMap conversion
test_autonomy_localiser.py     parity with the Stage 2 adapter; failure detection; no truth leak
test_autonomy_planning.py      frontiers, clustering, inflation, A* optimality, no path
test_autonomy_explorer.py      completion, map accuracy, zero collisions, loc failure -> safe stop,
                               Pause/Resume/Stop, STOP latency
firmware/
  README.md                    build-only instructions; never upload; safety summary
  autonomy_robot/              Arduino sketch (folder name = sketch name)
    autonomy_robot.ino         thin setup()/loop(): wires Serial, Serial2, MockMotorDriver
    config.h                   baud, timeouts, motion limits, LD06 pins (from the TP notes,
                               "confirm"); MOTOR_OUTPUTS_ENABLED 0, #error unless also
                               HARDWARE_VERIFIED: no physical driver exists yet
    src/crc.h/.cpp             CRC-16/CCITT-FALSE (frames), LD06 CRC-8
    src/protocol.h/.cpp        frame codec, byte-at-a-time non-blocking parser
    src/safety.h/.cpp          safety state machine + watchdog; time injected (testable)
    src/motor_driver.h         MotorDriver interface: enable(), disable(), setWheels(...)
    src/mock_motor_driver.h    records requested outputs; never touches a pin
    src/lidar_source.h/.cpp    LD06 via the Robot-Maker library (external dependency, not
                               copied in; licence open) -> scan buffer, CCW angles
    src/mecanum_i2c_driver.h/.cpp  physical driver for the I2C coprocessor at 0x30 (SDA 19,
                               SCL 21): ACK-checked, zero-PWM refresh, calibration table;
                               compiled out until H2
    src/telemetry.h/.cpp       state, faults, counters, uptime, boot id
    src/controller.h/.cpp      ties parser, safety, motion timer, scan requests together
  tests/
    host_tests.cpp             dependency-free asserts; same scenarios as the Python model
    protocol_vectors.h         GENERATED by test_autonomy_protocol.py (golden frames)
    run_host_tests.ps1         builds and runs the host tests (needs a host compiler)
docs/design/robot_protocol_v2.md   the protocol (single source of truth; v1 kept, superseded)
docs/decisions/                    independent localiser; own LD06 parser; STOP semantics
docs/AUTONOMY_HANDOFF.md           limitations, Workspace integration requirements, handoff
results/autonomy_<timestamp>/      demo PNGs, metrics JSON, test logs (new folders only)
```
Portable C++ in `src/` does not include `Arduino.h`. Only the `.ino` file and two thin wrappers
(`Serial` and `millis()`) touch the Arduino API, so the same code compiles on the host for tests.

## 4. Protocol v1 (summary; the full specification goes in docs/design)
**Frame layout:**
`0xA5 0x5A | version u8 (=1) | type u8 | seq u16 | length u16 | hcrc u8 | payload | crc u16`
- Little-endian.
- The CRC is CRC-16/CCITT-FALSE, computed over version..payload.
- The payload is at most 512 bytes.
- *Step 1 change:* `hcrc` (CRC-8 over the header) was added. Without it, a corrupted length
  stalled the link. The full details, and the final payloads (MOVE now also carries
  `turn_rate_cdeg_s`; scan points carry intensity), are in `docs/design/robot_protocol_v1.md`.

**Decoder behaviour:**
- A bad CRC, an unknown version or an oversize length is counted, and the decoder resyncs on the
  next sync pair.
- It never blocks and never allocates.

| Host → robot | Payload |
|---|---|
| HELLO | protocol version query |
| HEARTBEAT | — |
| ARM | — |
| DISARM | — |
| STOP | — |
| CLEAR_FAULT | — |
| MOVE | forward_mm i16 (abs ≤ 500), turn_cdeg i16 (abs ≤ 9000), speed_mm_s u16 (≤ 200), timeout_ms u16 (≤ 3000) |
| SCAN_REQUEST | — |

| Robot → host | Payload |
|---|---|
| ACK / NACK | ref_seq, reason code |
| TELEMETRY | at 5 Hz: state, fault bits, boot_id, uptime, rx_ok, crc_err, framing_err, last_seq, heartbeat age, motion progress, requested wheel outputs |
| MOVE_DONE | ref_seq, completed or aborted, reason |
| SCAN_CHUNK | scan_id, chunk i/n, points (angle 0.01° u16, distance mm u16) |

**Sequence numbers:**
- A MOVE with a stale or duplicate seq is NACKed and not re-executed.
- STOP is applied whatever its seq.

## 5. Safety and state machines
**Firmware:**

| Transition | Rule |
|---|---|
| BOOT → DISARMED | The motor driver is disabled before anything else. |
| DISARMED → ARMED | On ARM, only with a live heartbeat and no fault. |
| ARMED → MOVING | On MOVE, only if the move is within limits. |
| MOVING → ARMED | When the move is done, on STOP, or at its timeout. |
| any state → FAULT | Heartbeat older than 300 ms, a motion timeout or a driver fault. Outputs are zeroed and disabled immediately; FAULT then needs CLEAR_FAULT, then ARM. |

- STOP zeroes the outputs within one loop pass, in any state, and is idempotent.
- DISARM disables the driver.
- After a restart (new boot_id), the firmware is always DISARMED. The host must notice and
  re-ARM. The firmware never resumes a move.

**Explorer:**
- Main loop: IDLE → START → SCANNING → LOCALISING → MAPPING → SELECTING_FRONTIER → PLANNING →
  MOVING → SCANNING …
- Ends in COMPLETE when no reachable frontier remains, or in FAILED.
- PAUSE stops the robot and holds. RESUME re-scans and re-localises before moving. STOP is
  terminal and disarms.

**Localisation policy:**
- The odometry prior comes from the commanded motion, never from ground truth.
- `ok`: accept the pose, then integrate the scan.
- `weak` or a failure: no integration. Re-scan in place, up to 2 times; then try a global search
  that must agree with the prior within 1 m; otherwise FAILED (STOP + DISARM).
- Ground truth lives inside `SimRobot` behind the link. `Explorer` holds only a `RobotLink`.
  Only tests and metrics read the truth, through the simulator handle.

**Collision:**
- The planner inflates obstacles by the robot radius plus a margin (default radius 0.15 m,
  margin 0.10 m), and treats unknown space as blocked.
- `SimRobot` moves in 1 cm sub-steps and stops before contact. Any contact is recorded and fails
  the test.

## 5b. Live mapping dashboard (requirement added 2026-11-04)
The full design is in `docs/design/live_mapping_dashboard.md`. In summary:
- One `MappingSession` is the only source of all four views. Only scans localised `ok` are
  integrated, so failures never touch the maps.
- Views read immutable `MapSnapshot`s with per-layer revisions.
- `ScanSource`/`ScanFrame` make the simulated and ESP32 sources interchangeable.
- The engine and vector-merge threads never block the Tk thread.
- Redraw caps are based on measured costs.
- Workspace integration comes later, through Matplotlib layers that draw on `Viewport.axes`.

## 5c. Approved changes after inspecting the original robot files (2026-11-04)
Source: `docs/research/original_robot_firmware_findings.md`. Decisions: `docs/decisions/2026-11-04_*`.
- **Motors:** the robot drives through an I2C motor coprocessor (0x30, SDA = GPIO19, SCL = GPIO21,
  PWM registers 1-8).
  - The physical `MecanumI2cDriver` comes later: ACK-checked, zero-PWM refresh, needs a
    calibration table.
  - The mock driver stays the only one until H2.
- **LiDAR:** the Robot-Maker LD06 library is reused as an external dependency; our own parser is
  dropped. SCAN_CHUNK angles are **CCW**, and `protocol.points_to_sensor_xy` converts them.
- **Interim limits, now in protocol v2:** 300 mm and 2000 ms per MOVE. Planners split longer
  motions.
- **Firmware separation:** the new firmware has no HTTP server. The original TP firmware and
  `RobotControl.html` stay a separate legacy setup. Never flash `ESP32MecanumCar.ino` (it
  moves on boot).
- **H2 prerequisite:** observe the coprocessor when the ESP32 resets mid-move (wheels off the
  ground, with the user). If it holds the last PWM, a hardware stop path is required first.
- **H1 additions:** the LD06's 0° direction relative to the chassis, and the LiDAR offset from
  the rotation centre.
- **Still open:** the LD06 library licence; the hardware facts in findings section 7.

## 6. Implementation order (tests run after every step)
**Status and revised order (2026-11-04, after the Wi-Fi requirement):**
- **Done:** step 1 (protocol v1, 27 tests) and step 2 (firmware model and mock link, 45 tests).
- **Done:** step 2b (protocol v2, `ProtocolRobotLink` over `Transport`, firmware model v2 rules;
  90 protocol and link tests).
- **Done:** step 2c (`sim_network.py` and the Wi-Fi tests: 16 tests including a 30-minute
  soak). Results: `wifi_transport.md` section 5b.
- **Next:** step 3, the firmware C++ at v2 (with the approved changes of section 5c).
- **Step 3 is paused:** `protocol.h/.cpp` are written but not compiled or tested.

New steps, in this order:
- **2b. Protocol v2 and transports (Python):**
  - `ProtocolRobotLink` + `Transport`; MockRobotLink becomes a thin wrapper, and all 45 link
    tests must still pass;
  - v2: MOVE `valid_until_ms`, HEARTBEAT `echo_uptime_ms`, session binding, SCAN_RESEND,
    `EXPIRED` / `NOT_CONTROLLER`;
  - FirmwareModel v2 rules; regenerated golden vectors; the protocol document becomes v2.
- **2c. `autonomy/sim_network.py` + `test_autonomy_wifi.py`:** latency, jitter and reordering,
  Bernoulli and burst loss, duplication, corruption, bandwidth, complete and one-way cuts
  (`wifi_transport.md` section 5).
- **3. Firmware C++ at v2:**
  - safety controller, mock motor driver, `LinkTransport` with `SerialLink` and `UdpLink`
    (SoftAP; station as a build option);
  - host g++ tests replaying Python-generated traces byte for byte;
  - `arduino-cli compile` only.

Steps 4-10 follow as listed below. Wherever a step says "serial", it now means "either
`Transport`".

**Hardware stages** (each needs your explicit approval; none is started):
- **H1, Wi-Fi bring-up with the MOCK motor driver:** flash, then measure latency, loss, recovery
  and current (research note, section 4). Motors stay physically disconnected.
- **H2, motor driver verification:** wiring, pins, driver datasheet, and a hardware enable
  path that drops out on its own; then the physical driver behind the existing interface.
- **H3, supervised first motion:** wheels off the ground, then on the floor with a STOP within
  reach.

1. `protocol.py` + tests + golden vectors → `docs/design/robot_protocol_v1.md`.
2. `firmware_model.py` + `robot_link.py` (mock) + link tests: malformed frames, comm loss,
   watchdog, STOP, restart.
3. Firmware C++: crc, protocol, safety, mock motor and controller, plus host tests from the same
   vectors and scenarios. An `arduino-cli compile` check (compile only; nothing is uploaded).
4. `world.py`, `robot_sim.py` + tests (collision, bounded moves, noise).
5. `mapping.py`, `localiser.py` + tests (Code 02 parity, adapter parity, failure detection).
6. `frontiers.py`, `planner.py` + tests.
7. `explorer.py`, `metrics.py`, `scenarios.py` + end-to-end tests.
7b. `scan_source.py`, `mapping_state.py` + tests (the gate, layer consistency, no contamination).
8. Firmware `ld06_parser` (tested on Code 01's manufacturer example packet), SCAN_CHUNK and
   telemetry.
9. `engine.py`, `dashboard_layers.py`, `dashboard.py` (these replace `visualise.py`),
   `autonomy_sim.py` + tests (non-blocking, STOP latency, rendering), then a demo run with
   timing evidence in `results/autonomy_<ts>/`.
10. Docs: decisions, handoff, limitations, Workspace integration requirements; STATUS on this
    branch only.

**Acceptance targets (proposed):**
- 0 collisions;
- explored reachable free area ≥ 90 %;
- occupied cells within 1 cell of a true wall: precision ≥ 0.9, recall ≥ 0.8;
- final pose error ≤ 0.10 m / 5° with default noise;
- a forced localisation failure ends in FAILED with the robot stopped and disarmed;
- STOP is effective within one simulated step.

## 7. Open decisions (yours)
0. **Wi-Fi mode.** Recommended: SoftAP by default, station as a build option for WPA2-PSK
   networks, never eduroam. Also: approve the protocol v2 changes and the default timings in
   `wifi_transport.md` section 2.
1. **Host C++ compiler for the firmware tests.** None is installed. Options:
   - (a) you install MSYS2 g++;
   - (b) `pip install ziglang` in a separate venv (a new dependency);
   - (c) no host tests for now: only `arduino-cli compile` checks plus the Python reference model.
2. **Drive type.** Differential or Mecanum (the TP notes link a "MecanumCarV2" guide)? The
   protocol uses body-frame (forward, turn) moves, so both fit. The wheel-output count in
   `MotorDriver` waits for your answer.
3. **Approval of this plan**, the acceptance targets and the protocol limits (500 mm,
   90°, 200 mm/s, 300 ms heartbeat timeout).
