# Robot link protocol v1

Status: Superseded by `robot_protocol_v2.md` (2026-11-04). Kept as the record of v1.
Originally: Draft (part of `docs/specs/autonomy_phase_b.md`). Written 2026-11-04.

This is the single specification of the link between the Python explorer and the ESP32 firmware.
- Reference implementation: `autonomy/protocol.py`.
- Conformance: `firmware/tests/protocol_vectors.h`, generated from the Python golden vectors.
  `test_autonomy_protocol` fails if the header is out of date.
- Transport: USB serial (CH340), 115200 baud, 8N1.
- Nothing in v1 is wireless. Nothing in this phase opens a port.

## 1. Frame
All integers are little-endian.

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 2 | sync | `0xA5 0x5A` |
| 2 | 1 | version | `1` |
| 3 | 1 | type | section 3 |
| 4 | 2 | seq | the sender's counter; wraps 65535 → 0 |
| 6 | 2 | length | payload bytes, 0-512 |
| 8 | 1 | hcrc | CRC-8, poly 0x07, init 0, over bytes 2-7 (check value `0xF4`) |
| 9 | length | payload | section 3 |
| 9+length | 2 | crc | CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), over bytes 2 … end of payload (check value `0x29B1`) |

- Frame size: 11 to 523 bytes.

**Why a header CRC?** An earlier draft had none. A test showed that one flipped bit in `length`
made the receiver wait for up to 512 bytes that never come. With heartbeats at 10 bytes every
100 ms, that hides valid frames for seconds and trips the watchdog. It fails safe, but it makes
the link unusable. With `hcrc`, a bad header is rejected as soon as its 9 bytes arrive.

**Residual risk:** random bytes that form a false sync pair and also pass the CRC-8 (1 in 256
false syncs) can still cause such a wait. **Proposed for the firmware:** a 50 ms inter-byte
timeout that abandons a partial frame. It is not in the Python decoder yet.

**Receiver rule** (`FrameDecoder` and the firmware parser behave identically):
1. Skip bytes until the sync pair appears.
2. Once 9 header bytes are present, check hcrc, then the version, then `length` ≤ 512. A failure
   is a **framing error**.
3. Wait for the payload and the CRC. A CRC mismatch is a **crc error**.
4. After any error, drop **one** byte and search again. A valid frame that starts inside a
   corrupt one is therefore still found.
5. Memory is bounded at 523 bytes. The receiver never blocks.

## 2. Sequence numbers
- Each side counts its own frames.
- `seq_newer(a, b)` is true when `a` is 1 to 32767 steps after `b` (modulo 2^16).
- **MOVE:** the robot executes a MOVE only if its seq is newer than the last MOVE accepted since
  boot. The first MOVE after boot is always newer. A replayed or duplicate MOVE is NACKed with
  `STALE_SEQ` and never re-executed.
- **STOP, DISARM, HEARTBEAT** are applied whatever their seq.
- **Replies** (ACK, NACK, MOVE_DONE) carry `ref_seq`, the seq of the command they answer.

## 3. Messages
Host → robot (types below 0x80). A robot-to-host type sent to the robot is NACKed with
`WRONG_DIRECTION`.

| Type | Name | Payload | Reply |
|---|---|---|---|
| 0x01 | HELLO | — | HELLO_REPLY |
| 0x02 | HEARTBEAT | — | none (refreshes the watchdog) |
| 0x03 | ARM | — | ACK / NACK (NO_HEARTBEAT, FAULT_ACTIVE) |
| 0x04 | DISARM | — | ACK |
| 0x05 | STOP | — | ACK |
| 0x06 | CLEAR_FAULT | — | ACK / NACK (NO_HEARTBEAT) |
| 0x10 | MOVE | `forward_mm i16, turn_cdeg i16, speed_mm_s u16, turn_rate_cdeg_s u16, timeout_ms u16` (10 B) | ACK then MOVE_DONE, or NACK |
| 0x11 | SCAN_REQUEST | — | ACK then SCAN_CHUNK × n, or NACK (BUSY, LIDAR_UNAVAILABLE) |

Robot → host:

| Type | Name | Payload |
|---|---|---|
| 0x80 | HELLO_REPLY | `protocol_version u8, firmware_version u16, boot_id u32, max_payload u16, motor_driver u8` (0 = MOCK, 1 = PHYSICAL) |
| 0x81 | ACK | `ref_seq u16, ref_type u8` |
| 0x82 | NACK | `ref_seq u16, ref_type u8, reason u8` |
| 0x83 | TELEMETRY | 31 B, every 200 ms: `state u8, faults u16, boot_id u32, uptime_ms u32, rx_frames u32, crc_errors u16, framing_errors u16, last_seq u16, heartbeat_age_ms u16, move_seq u16, move_progress_pct u8, cmd_forward_mm_s i16, cmd_turn_cdeg_s i16, outputs_enabled u8` |
| 0x84 | MOVE_DONE | `ref_seq u16, outcome u8` (0 completed, 1 stopped, 2 fault, 3 timeout)`, faults u16` |
| 0x85 | SCAN_CHUNK | `scan_id u16, chunk_index u8, chunk_count u8`, then 1-101 × `angle_cdeg u16, distance_mm u16, intensity u8` |

**Telemetry fields:**
- The 16-bit counters and `heartbeat_age_ms` saturate at 65535.
- `cmd_*` is the body velocity *requested* from the motor driver. It is not measured motion:
  there are no encoders in v1.

**Scan chunks:**
- Angles are the LD06's own: **clockwise**, in 0.01°, within [0, 36000), exactly as Code 01's
  `decode_ld06_packet`. The host converts them with Code 01's `polar_to_cartesian` (+X front,
  +Y left).
- `distance 0` means no echo. It is sent, and the host filters it out.
- A full 456-point scan is 5 chunks, about 2.3 kB, which takes about 0.2 s at 115200 baud.
- The host (`ScanAssembler`) drops a scan with a missing, out-of-order or foreign chunk. Two
  revolutions are never stitched together.

**NACK reasons:**

| Code | Name | Code | Name |
|---|---|---|---|
| 1 | BAD_LENGTH | 6 | STALE_SEQ |
| 2 | UNKNOWN_TYPE | 7 | BUSY |
| 3 | OUT_OF_BOUNDS | 8 | NO_HEARTBEAT |
| 4 | NOT_ARMED | 9 | LIDAR_UNAVAILABLE |
| 5 | FAULT_ACTIVE | 10 | WRONG_DIRECTION |

## 4. Bounded motion
A MOVE first turns in place by `turn_cdeg` (+ means counter-clockwise), then drives straight for
`forward_mm`. It is accepted only if all of these hold (`validate_move`, mirrored in the
firmware):

| Field | Allowed values |
|---|---|
| forward_mm | ≤ 500 in absolute value |
| turn_cdeg | ≤ 9000 in absolute value (90°) |
| speed_mm_s | 1-200 when forward_mm is not 0 |
| turn_rate_cdeg_s | 1-9000 when turn_cdeg is not 0 |
| timeout_ms | 1-3000 |

- At least one of the two stages must be non-zero.
- The nominal duration `|turn| / turn_rate + |forward| / speed` must fit inside `timeout_ms`.

**Firmware enforcement:** the firmware aborts any move still running at `timeout_ms`, with outcome
TIMEOUT and fault MOTION_TIMEOUT. A move can never run unbounded.

## 5. Safety states (reference model: `autonomy/firmware_model.py`, step 2)
States: `BOOT 0 → DISARMED 1 → ARMED 2 ⇄ MOVING 3`, and `FAULT 4`.

**Start-up:**
- Motors are disabled at BOOT, before the serial port opens.
- Each restart draws a new `boot_id`, and the robot is DISARMED.

**Arming and commands:**
- ARM needs a heartbeat received within 300 ms and no latching fault.
- STOP: outputs are set to zero within one loop pass. It works in every state, can be repeated
  safely, and leaves the robot ARMED (or DISARMED, or in FAULT, if it already was).
- DISARM: outputs are set to zero and the driver is disabled.

**Faults:**
- Heartbeat older than 300 ms while ARMED or MOVING: HEARTBEAT_TIMEOUT. Outputs are set to zero,
  the driver is disabled, and the robot enters FAULT.
- Leaving FAULT needs CLEAR_FAULT (with a live heartbeat), then ARM.
- Latching faults: HEARTBEAT_TIMEOUT, MOTION_TIMEOUT, DRIVER_FAULT, RX_OVERFLOW.
- LIDAR_NO_DATA is informational only.

**MOVE checks, in this order** (the first that fails gives the NACK):
1. STALE_SEQ: replay protection comes first, so a retransmitted MOVE is never executed twice.
2. FAULT_ACTIVE.
3. NOT_ARMED.
4. BUSY: no queuing; one move at a time.
5. NO_HEARTBEAT.
6. OUT_OF_BOUNDS.

**Other firmware behaviour:**
- The first motor output of an accepted move is set in the same pass that received it.
- TELEMETRY is sent at the first loop pass after boot, then every 200 ms.
- SCAN_REQUEST is allowed in every state except MOVING (stop-and-scan). Scanning moves nothing.
- More than 1024 bytes in one loop pass latches RX_OVERFLOW.

**Host behaviour** (`MockRobotLink`; a future serial link must do the same):
- Send HEARTBEAT every 100 ms while connected.
- `connect()` refuses a robot that reports another protocol version or a **PHYSICAL** motor
  driver. Physical motion needs the approved hardware stage.
- Commands wait 150 ms for ACK or NACK and are re-sent up to 3 times with the **same seq**. A
  STALE_SEQ reply to a re-sent MOVE means the first copy was accepted and only its ACK was lost.
- STOP is re-sent every 50 ms until acknowledged. It may be called from any thread.
- MOVE_DONE lost: the outcome is recovered from TELEMETRY (`move_seq`, `state`,
  `move_progress_pct`) and marked "inferred from telemetry".
- **Link lost** (no reply to MOVE, no MOVE_DONE, or no telemetry for 1 s): `move()` never
  returns while the robot might still move.
  - It sends STOP and waits up to 200 ms for the ACK.
  - If none arrives, it withholds heartbeats for 500 ms (longer than the 300 ms watchdog), so
    the firmware disables the driver by itself.
  - This covers a **one-way loss**: heartbeats still arrive (so the watchdog would never fire)
    but replies are lost. A test found this case.
- A change of `boot_id` means the robot restarted. Treat it as DISARMED, never assume an earlier
  MOVE is still running, and refuse moves until `connect()` and ARM.

## 6. Version history
- v1 (2026-11-04, draft): initial version, including the header CRC-8. Step 2 fixed the MOVE
  check order, the telemetry timing and the host retry and link-loss rules (section 5).
