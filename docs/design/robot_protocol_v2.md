# Robot link protocol v2

Status: Draft (part of `docs/specs/autonomy_phase_b.md`). Written 2026-11-04.
Supersedes `robot_protocol_v1.md`: the framing is unchanged, and the Wi-Fi additions are marked
**v2**. The reasons for each change are in `wifi_transport.md`.

**Where it lives:**
- Reference implementation: `autonomy/protocol.py`.
- Reference firmware behaviour: `autonomy/firmware_model.py`.
- Host behaviour: `autonomy/robot_link.py` (`ProtocolRobotLink`).
- Conformance: `firmware/tests/protocol_vectors.h` (17 vectors), generated from Python;
  `test_autonomy_protocol` fails if it is stale.

**Transports:**
- **Wi-Fi UDP is primary:** one frame per datagram, robot SoftAP at 192.168.4.1, port 47800
  (proposed).
- **USB serial is the fallback:** 115200 baud 8N1, frames as a byte stream.
- Both carry identical frames with identical semantics. Neither real transport is implemented
  yet; only the simulated ones are.

## 1. Frame (unchanged from v1)
All integers are little-endian.

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 2 | sync | `0xA5 0x5A` |
| 2 | 1 | version | **`2`**. A v1 peer is a framing error, and `connect()` refuses a version mismatch. |
| 3 | 1 | type | section 3 |
| 4 | 2 | seq | the sender's counter; wraps 65535 → 0 |
| 6 | 2 | length | payload bytes, 0-512 |
| 8 | 1 | hcrc | CRC-8, poly 0x07, init 0, over bytes 2-7 (check value `0xF4`) |
| 9 | length | payload | section 3 |
| 9+length | 2 | crc | CRC-16/CCITT-FALSE over bytes 2 … end of payload (check value `0x29B1`) |

**Receiver rule:** the same as v1 (skip to sync; on any error drop one byte and search again;
at most 523 bytes buffered). Over UDP each datagram is decoded on its own.

**Why the header CRC exists:** a corrupted length must never make the receiver wait for bytes
that will not come.

## 2. Two clocks, no synchronisation (v2)
- **seq (16-bit):** each side counts its own frames. `seq_newer` is modulo 2^16.
- **Robot uptime (32-bit ms, wrapping):** sent in TELEMETRY and HELLO_REPLY.
  - The host never sends its own time. It only **echoes robot uptime**:
    - `HEARTBEAT.echo_uptime_ms` = the newest uptime the host has received;
    - `MOVE.valid_until_ms` = that uptime + host time elapsed since it arrived +
      `COMMAND_TTL_MS` (250 ms).
  - The robot compares these against its own clock with wrapping arithmetic (`uptime_after`,
    `uptime_age`). The bounds therefore hold whatever the network delay, and no clock
    synchronisation is needed.
  - The host tracks the **newest** uptime only. Datagrams that arrive out of order never move it
    backwards.

## 3. Messages
Host → robot (types below 0x80). Unless marked "anyone", **only the bound controller** may send
it; others get NACK `NOT_CONTROLLER`.

| Type | Name | Payload | Reply |
|---|---|---|---|
| 0x01 | HELLO | **v2:** `session_id u32` | HELLO_REPLY, or NACK NOT_CONTROLLER (another session holds the robot) |
| 0x02 | HEARTBEAT | **v2:** `echo_uptime_ms u32` | none (a foreign heartbeat is silently ignored) |
| 0x03 | ARM | — | ACK / NACK (FAULT_ACTIVE, NO_HEARTBEAT) |
| 0x04 | DISARM | — (**anyone**) | ACK. The controller's own DISARM also releases the binding. |
| 0x05 | STOP | — (**anyone**) | ACK |
| 0x06 | CLEAR_FAULT | — | ACK / NACK (NO_HEARTBEAT) |
| 0x10 | MOVE | `forward_mm i16, turn_cdeg i16, speed_mm_s u16, turn_rate_cdeg_s u16, timeout_ms u16,` **v2:** `valid_until_ms u32` (14 B) | ACK then MOVE_DONE, or NACK |
| 0x11 | SCAN_REQUEST | — | ACK then SCAN_CHUNK × n, or NACK (BUSY, LIDAR_UNAVAILABLE) |
| 0x12 | **SCAN_RESEND** (v2) | `scan_id u16, chunk_index u8` | that SCAN_CHUNK, or NACK LIDAR_UNAVAILABLE |

Robot → host. ACK and NACK go to the sender; everything else goes to the bound controller. While
unbound, a serial line still delivers it and UDP drops it.

| Type | Name | Payload |
|---|---|---|
| 0x80 | HELLO_REPLY | `protocol_version u8, firmware_version u16, boot_id u32, max_payload u16, motor_driver u8,` **v2:** `uptime_ms u32` (14 B) |
| 0x81 | ACK | `ref_seq u16, ref_type u8` |
| 0x82 | NACK | `ref_seq u16, ref_type u8, reason u8` |
| 0x83 | TELEMETRY | 31 B, every 200 ms (first at the first loop pass): `state, faults, boot_id, uptime_ms, rx_frames, crc_errors, framing_errors, last_seq, heartbeat_age_ms, move_seq, move_progress_pct, cmd_forward_mm_s, cmd_turn_cdeg_s, outputs_enabled` (as in v1) |
| 0x84 | MOVE_DONE | `ref_seq u16, outcome u8, faults u16` |
| 0x85 | SCAN_CHUNK | `scan_id u16, chunk_index u8, chunk_count u8`, then 1-101 × `angle_cdeg u16` (**counter-clockwise** from the sensor's front, as the LD06 library gives it; decision `2026-11-04_scan_angles_ccw.md`), `distance_mm u16, intensity u8` |

**NACK reasons:**

| Code | Name | Code | Name |
|---|---|---|---|
| 1 | BAD_LENGTH | 7 | BUSY |
| 2 | UNKNOWN_TYPE | 8 | NO_HEARTBEAT |
| 3 | OUT_OF_BOUNDS | 9 | LIDAR_UNAVAILABLE |
| 4 | NOT_ARMED | 10 | WRONG_DIRECTION |
| 5 | FAULT_ACTIVE | 11 | **EXPIRED** (v2) |
| 6 | STALE_SEQ | 12 | **NOT_CONTROLLER** (v2) |

**Scans (v2):**
- Chunks may arrive in any order or twice.
- The host keeps one partial scan per `scan_id`. A chunk of another scan drops the partial, so
  two revolutions are never stitched together.
- After 150 ms without a new chunk, the host sends SCAN_RESEND for each missing index, up to 3
  rounds; then the scan is "incomplete" and the caller rescans.
- The robot keeps only its last scan for resending.

## 4. Bounded motion (unchanged from v1)
Turn in place by `turn_cdeg`, then drive `forward_mm`. The limits:

| Field | Allowed values |
|---|---|
| forward_mm | ≤ **300** in absolute value (interim) |
| turn_cdeg | ≤ 9000 in absolute value |
| speed_mm_s | 1-200 when forward_mm is not 0 |
| turn_rate_cdeg_s | 1-9000 when turn_cdeg is not 0 |
| timeout_ms | 1-**2000** (interim) |

- The nominal duration must fit inside `timeout_ms`.
- The firmware aborts any move still running at `timeout_ms` (MOTION_TIMEOUT).

## 5. Safety rules (firmware; transport-independent)
**States:** BOOT → DISARMED → ARMED ⇄ MOVING, and FAULT. The driver is disabled before anything
else at boot and at every restart (new `boot_id`).

**Session binding (v2):**
- The first HELLO binds `(session_id, sender)`.
- A HELLO with the **same** session_id from a new address rebinds, for example after a Wi-Fi
  reconnect gives a new IP address.
- A different session is refused while bound.
- The binding is released by the controller's DISARM, by a reboot, or after 5 s of controller
  silence while DISARMED or in FAULT. It is never released while ARMED: silence there trips the
  watchdog first.
- ARMED or MOVING without a bound controller is an invariant violation.

**Heartbeat (v2):** a HEARTBEAT from the controller counts only if both hold:
- `uptime - echo ≤ 800 ms` (`HEARTBEAT_ECHO_MAX_MS`); a "future" echo has a huge age and is
  rejected;
- `echo` is not older than the last accepted echo; reordered heartbeats are rejected.

Rejected heartbeats are counted. Delayed, buffered, reordered or foreign heartbeats therefore
**cannot keep the watchdog alive**.

**MOVE checks, in order** (the first that fails gives the NACK):
1. NOT_CONTROLLER.
2. STALE_SEQ.
3. **EXPIRED (v2):** robot uptime after `valid_until_ms`.
4. FAULT_ACTIVE.
5. NOT_ARMED.
6. BUSY.
7. NO_HEARTBEAT.
8. OUT_OF_BOUNDS.

The first motor output is set in the same loop pass that receives the MOVE.

**STOP and DISARM:**
- STOP (from anyone) sets the outputs to zero in the same loop pass, in every state. It can be
  repeated safely and leaves ARMED or FAULT as they were.
- DISARM (from anyone) disables the driver.

**Faults:**
- No live heartbeat for 300 ms while ARMED or MOVING, a motion timeout, a driver fault, or more
  than 1024 bytes in one pass latches FAULT. Outputs are set to zero and the driver disabled.
- Leaving FAULT needs CLEAR_FAULT (with a live heartbeat), then ARM.
- A **Wi-Fi outage of any length is just "no valid heartbeat"**. The firmware stops by itself and
  never needs a Wi-Fi event to do so.

**Other:** SCAN_REQUEST is allowed in every state except MOVING.

## 6. Host rules (`ProtocolRobotLink`; every transport behaves the same)
**Connecting:**
- HELLO with a random 32-bit session id per host run.
- `connect()` refuses: no reply, NOT_CONTROLLER, another protocol version, or a **PHYSICAL**
  motor driver.
- HEARTBEAT every 100 ms, echoing the newest robot uptime.

**Commands:**
- Commands wait 150 ms for a reply and are re-sent up to 3 times with the **same seq and the same
  bytes**, so a MOVE's deadline never moves.
- A STALE_SEQ reply to a re-sent MOVE means the first copy was accepted.

**STOP:** re-sent every 50 ms until acknowledged. It can be called from any thread.

**Lost replies:** a lost MOVE_DONE is recovered from TELEMETRY ("inferred from telemetry").

**Link lost** (no reply to MOVE, no MOVE_DONE, or no telemetry for 1 s):
- STOP, then 200 ms waiting for its ACK.
- If no ACK arrives, heartbeats are withheld for 500 ms, so the firmware watchdog disables the
  driver.
- `move()` never returns while the robot might still be moving. This covers one-way loss.

**Restart:**
- A new `boot_id` means DISARMED. Moves are refused until `connect()` and ARM.
- Late frames from an earlier boot are ignored.

## 7. Version history
- v1 (2026-11-04): framing, messages, safety core; header CRC-8 added after a test showed a
  corrupted length could stall the link.
- **v2 (2026-11-04, draft):** for Wi-Fi.
  - HELLO `session_id` and session binding; HEARTBEAT `echo_uptime_ms`.
  - MOVE `valid_until_ms`; HELLO_REPLY `uptime_ms`.
  - SCAN_RESEND, and the NACK reasons EXPIRED and NOT_CONTROLLER.
  - STOP and DISARM are accepted from anyone.
  - The host tracks the newest robot uptime and telemetry (reordering-safe).
  - Later on 2026-11-04 (approved after the original robot files were inspected):
    - SCAN_CHUNK angles are counter-clockwise (LD06 library convention);
    - interim limits: 300 mm and 2000 ms per MOVE (decision
      `2026-11-04_interim_motion_limits_and_no_http.md`).
