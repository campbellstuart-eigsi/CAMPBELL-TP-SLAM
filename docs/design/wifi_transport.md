# Wi-Fi transport and protocol v2

Status: Draft (part of `docs/specs/autonomy_phase_b.md`). Written 2026-11-04. Not implemented.
Background and sources: `docs/research/esp32_wifi_transport.md`.

**Requirement:** the robot runs **untethered**. Wi-Fi is the transport for control, telemetry
and LiDAR scans. USB serial stays as a development and debugging fallback, with identical
command and telemetry semantics.

## 1. One protocol, two transports
```
Explorer / engine
      │  RobotLink (connect, arm, disarm, stop, clear_fault, move, request_scan, pump, close)
ProtocolRobotLink      ← ALL host logic: seq, retries, STOP re-send, link-loss policy,
      │                  telemetry, boot_id, scan assembly (today's MockRobotLink logic)
      │  Transport (send_frame(bytes), poll() -> [received byte chunks], now_ms(), close())
 ┌────┴─────────────┬───────────────────┬──────────────────────┐
SerialTransport   UdpTransport        SimSerialTransport     SimNetworkTransport
(USB, fallback,   (Wi-Fi, primary,    (today's MockTransport) (datagram simulator:
 hardware stage)   hardware stage)                             loss, delay, reorder, cut)
```
- `MockRobotLink` becomes `ProtocolRobotLink(transport=SimSerial… / SimNetwork…, firmware
  model)`. `SerialRobotLink` and `WiFiRobotLink` are the same class over the real transports.
  Both real transports stay `NotImplementedError` until the hardware stage is approved.
- **Framing is unchanged.** The same 9-byte header, CRC-8 and CRC-16 are used on both transports:
  - UDP carries exactly **one frame per datagram**, fed into the same `FrameDecoder`. A truncated
    or corrupt datagram fails the CRC and is dropped.
  - Serial carries the same frames as a byte stream.
- **Firmware side:** a `LinkTransport` interface with `SerialLink` and `UdpLink`. The safety
  controller receives bytes and writes frames without knowing which transport carries them.
  Telemetry goes to the bound controller (section 3) only.

## 2. What Wi-Fi adds, and how v2 handles it
Wi-Fi brings delay, jitter, **reordering**, **duplication**, burst loss and outages lasting
seconds. TCP is rejected: one loss holds back every later frame, including STOP; see the research
note.

| Hazard | v1 behaviour | v2 change |
|---|---|---|
| Duplicate or reordered MOVE | `STALE_SEQ`: never executed twice, never an older one after a newer | unchanged |
| **Delayed MOVE** (arrives late, after the host gave up or planned something else) | executed late | **MOVE carries `valid_until_ms`** in the *robot's* uptime clock. The host sets it to the uptime echoed in the latest telemetry plus `COMMAND_TTL_MS` (default 250 ms). The robot NACKs `EXPIRED` if its uptime is past it. No clock synchronisation is needed. |
| **Buffered or delayed heartbeats** (a burst after an outage keeps the watchdog alive although the host had no control) | each heartbeat refreshes the watchdog | **HEARTBEAT carries `echo_uptime_ms`**, the robot uptime from the newest telemetry the host received. The robot accepts it as live only if `uptime - echo ≤ HEARTBEAT_ECHO_MAX_MS` (default 600 ms). This proves a recent **round trip**; old heartbeats are ignored. |
| **Another device** on the network sends commands | n/a (USB is point to point) | **Session binding.** HELLO carries a random 32-bit `session_id`. The robot binds to the first controller (session_id plus source address) and NACKs `NOT_CONTROLLER` for others, **except STOP, which is always obeyed**. The binding is released on DISARM, on FAULT plus 5 s of silence, or at reboot. |
| A lost scan chunk | the whole scan is requested again | **SCAN_RESEND(scan_id, chunk_index)**: the robot keeps the last scan and re-sends the missing chunk. If it is gone, `LIDAR_UNAVAILABLE`, and the host rescans. |
| An outage of seconds | watchdog fault after 300 ms; host link-loss policy | unchanged. The firmware stops by itself. After recovery the robot is in FAULT: the host must CLEAR_FAULT and ARM again, and the explorer **re-scans and re-localises before any move**. Never a blind resume. |
| Robot rebooted during the outage | new boot_id; host refuses moves until reconnect | unchanged |

**About the uptime fields:**
- The firmware uses 32-bit milliseconds, compared by unsigned difference, so the wrap after 49
  days is harmless.
- **Bounding a delayed command needs no clock synchronisation.** The host only echoes the
  robot's own uptime. A late command is therefore bounded by the robot's clock, whatever the
  network delay.
- v2 applies to **both** transports, so serial and Wi-Fi behave the same.
- The version byte becomes 2. A v1 peer is refused at HELLO, as already implemented.

**Default timings to keep, then verify on hardware:**

| Setting | Value |
|---|---|
| Heartbeat period | 100 ms |
| Watchdog | 300 ms |
| Heartbeat echo maximum | 600 ms |
| Command TTL | 250 ms |
| STOP re-send | 50 ms |
| Telemetry period | 200 ms |
| Host link-loss threshold | 1 s without telemetry |

If the measured Wi-Fi 99th-percentile round trip is close to these numbers, the motion limits
must shrink before the timeouts are allowed to grow.

## 3. Firmware independence from Wi-Fi
- **The watchdog is transport-agnostic:** no valid heartbeat for 300 ms latches HEARTBEAT_TIMEOUT
  and disables the driver.
  - A Wi-Fi disconnect, a host crash, a laptop out of range or an AP reboot all look the same to
    the firmware: heartbeats stop.
  - Wi-Fi events only start a reconnection; they are never needed to stop the motors.
- **Command expiry:** every MOVE also keeps its own `timeout_ms` (at most 2 s, interim), and in v2 it can
  also expire (`valid_until_ms`). A move can never outlive its own bounds, even if the network
  stays silent.
- **Cores** [to verify on the board]: the Arduino `loop()` runs on core 1 and the Wi-Fi stack on
  core 0. Socket reads are non-blocking (`WiFiUDP::parsePacket()`), and frames are sent from the
  loop with bounded work per pass. A Wi-Fi stall therefore cannot stall the watchdog check.
  - Measure the worst loop-pass time with Wi-Fi active.
  - Add the ESP32 task watchdog on the loop task (a reset leaves the robot DISARMED).
- **Power save off** (`WiFi.setSleep(false)`) and Bluetooth off. DTIM sleep could otherwise
  delay heartbeats by about 100 ms or more.
- **Physical stage (requirement noted now, not designed):** the motor enable should also have a
  hardware path that drops out without software refresh, such as an enable line driven by a
  periodic signal. A firmware hang then also stops the motors.

## 4. Access point vs station (choice: SoftAP by default)
- **SoftAP:**
  - SSID `slam-robot-XXXX`; WPA2-PSK with the password in a local, untracked
    `wifi_secrets.h`.
  - Fixed channel (configurable); `max_connection = 1`; robot at 192.168.4.1, UDP port 47800.
- **Station:**
  - A build-time option for a WPA2-PSK lab network.
  - Event-driven reconnect with exponential backoff (0.5 s up to 8 s); sockets re-created after
    the IP comes back.
  - The robot is found by its last address, or by a broadcast HELLO on the subnet.
  - Not for eduroam (see the research note).
- **Never both at once** (APSTA): the AP would follow the station's channel.
- **Credentials** live in `wifi_secrets.h`, git-ignored, with an example file committed. They
  never go in the repository.

## 5. Simulation first (`autonomy/sim_network.py`)
`SimNetwork` is a seeded, deterministic datagram channel in each direction:

| Setting | Behaviour |
|---|---|
| `latency_ms` | base delay, plus `jitter_ms`: each datagram gets its own delay, so **reordering** happens naturally |
| `loss` | Bernoulli loss, plus optional **burst loss** (Gilbert-Elliott: good/bad states with their own loss rates) |
| `duplicate` | probability of delivering a datagram twice |
| `corrupt` | a bit flip, rejected by the CRC |
| `bandwidth` | default about 2 Mbit/s effective, conservative against Espressif's 20-30 Mbit/s rating |
| `cut(start_ms, end_ms)` | complete disconnection, in one or both directions |
| `max_datagram` | 1472 bytes (Ethernet MTU minus IP/UDP headers); larger is refused |

**Planned tests** (`test_autonomy_wifi.py`):
1. The 45 link scenarios, re-run over SimNetwork with typical Wi-Fi settings (5-20 ms latency, 1 %
   loss): same outcomes as over serial.
2. **Reordering:** an older MOVE arriving after a newer one is never executed, and an old
   duplicate after completion is STALE.
3. **Delay:** a MOVE delayed past its TTL is `EXPIRED` and never moves the robot. A burst of
   delayed heartbeats after an outage does not revive the watchdog.
4. **Burst loss** (Gilbert-Elliott) and a **complete cut** in either or both directions, during a
   move, a scan and idle:
   - the motors stop within 300 ms plus one loop pass of the last valid round trip;
   - `move()` never returns while the robot may still be moving;
   - after reconnection: FAULT, CLEAR_FAULT + ARM needed, no automatic resume.
5. **Asymmetric cut** (commands get through, replies are lost, and the reverse): this repeats the
   one-way case found in step 2, now on Wi-Fi.
6. **Second controller:** MOVE from another session is refused; STOP from it is obeyed.
7. **Scan resend:** with chunk loss, the scan completes via SCAN_RESEND without rescanning.
   Throughput: 10 Hz scans sustained at the simulated bandwidth (prepares M4).
8. **Long run:** 30 minutes of simulated exploration at 2 % loss, with random cuts. Zero
   invariant violations, zero motion while not armed.

## 5b. Simulation results (step 2c, 2026-11-04)
Implemented: `autonomy/sim_network.py` and `test_autonomy_wifi.py` (16 tests).
- **Safety monitor:** every test runs with a plant that checks the firmware invariants on every
  10 ms tick. No violation occurred.
- **Typical profile** (5 ms latency, 15 ms jitter, 1 % loss), 10 seeds: at least 9 of 10 moves
  completed, and every scan completed.
- **Throughput:** Wi-Fi gave at least 10 scans/s, while the serial fallback stayed under
  5 scans/s.
- **Heavy jitter** (80 ms): reordering occurred, and no move was ever executed twice.
- **Every datagram duplicated:** this exposed a host bug, now fixed.
  - The duplicate's STALE_SEQ NACK could overwrite the ACK, so the host reported "refused" while
    the robot was executing the move.
  - Now an ACK is never overwritten, and STALE_SEQ for the current seq always means "accepted".
- **An older MOVE delayed 600 ms**, arriving after a newer one: STALE and never executed.
- **Round trip 320 ms against the 250 ms TTL:** MOVE refused EXPIRED, with no motion; heartbeats
  were still accepted.
- **Heartbeats buffered for 1.5 s mid-move:** FAULT, and the released burst was rejected (stale
  echoes) without reviving the watchdog.
- **Complete cut mid-move:** the motors were disabled at most 310 ms after the last accepted
  heartbeat. `move()` returned `link_lost` with the robot stopped. After the outage: FAULT,
  CLEAR_FAULT and ARM were needed, and nothing moved by itself.
- **One-way cuts:**
  - robot→host only: the host stopped the robot itself (STOP, then starving the watchdog);
  - host→robot only: the robot faulted and reported it.
- **Robot reboot during an outage:** `link_lost`; the reconnect saw the new boot_id; then normal
  operation.
- **Burst loss** (Gilbert-Elliott, 90 % loss in the bad state): always safe.
- **30-minute soak**, poor profile (20 ms latency, 60 ms jitter, 3 % loss plus bursts, 1 %
  duplicates, 0.5 % corruption, 60 random cuts):
  - moves: 938 completed; 87 faults, 48 `link_lost`, 17 stopped (all safe);
  - refused: 58 EXPIRED, 56 FAULT_ACTIVE (recovery), 3 other;
  - scans: 519 ok, 10 failed; 187 stale heartbeats rejected;
  - **0 invariant violations, and 0 returns from `move()` while the robot was moving.**
- **Tuning note for H1:** 4.8 % of moves EXPIRED on the poor profile, because a retried MOVE
  keeps its original deadline (by design). Measure the real round trip before changing
  `COMMAND_TTL_MS` (250 ms).

## 6. Effect on the existing work
- **Step 1 (protocol v1):** the framing stays. v2 changes only MOVE and HEARTBEAT payloads and
  adds the `EXPIRED` and `NOT_CONTROLLER` NACK reasons, SCAN_RESEND and session_id. The golden
  vectors are regenerated, and the v1 document becomes v2.
- **Step 2:** the `MockRobotLink` logic moves unchanged into `ProtocolRobotLink`; its 45 tests
  must still pass. The FirmwareModel gains the v2 rules.
- **Step 3 (C++, partly written):** `protocol.h/.cpp` exist and are untested. They will be
  updated to v2 before the safety controller is written, plus a `LinkTransport` interface. The
  firmware's Wi-Fi/UDP code is compile-checked only (`arduino-cli compile`).
- **Dashboard design:** `Esp32ScanSource` reads from either transport. Nothing else changes.
