"""Python reference model of the ESP32 firmware's safety core (protocol v2).

This is the behaviour the C++ firmware (firmware/autonomy_robot/src/) must
reproduce, and what the simulated links talk to. Time is injected (integer
milliseconds), so every scenario is deterministic and testable:

    firmware.receive(data, now_ms, peer)   bytes from one sender (one loop pass)
    firmware.step(now_ms)                  watchdog, motion profile, telemetry
    firmware.read_frames()                 frames to send (any destination)
    firmware.read_addressed()              [(peer, frame)]: for datagram links

`peer` identifies the sender: SERIAL_PEER on USB, (address, port) on UDP.

Safety rules (docs/design/robot_protocol_v2.md, section 5):
  - the motor driver is disabled before anything else, at boot and restart;
  - the robot obeys one bound controller session (HELLO); STOP and DISARM
    are obeyed from ANY sender;
  - a heartbeat counts only if it echoes a robot uptime at most
    HEARTBEAT_ECHO_MAX_MS old and not older than the last accepted echo, so
    delayed or replayed heartbeats cannot keep the watchdog alive;
  - ARM needs a live heartbeat and no latching fault;
  - a MOVE is refused when replayed (seq) or EXPIRED (valid_until_ms);
  - STOP zeroes the outputs in the same pass that receives it, in any state;
  - no live heartbeat for HEARTBEAT_TIMEOUT_MS while ARMED or MOVING, a
    motion timeout, a driver fault or an RX overflow latch a FAULT: outputs
    zeroed, driver disabled, then CLEAR_FAULT and ARM are needed;
  - a move never outlives its timeout_ms and is never resumed after restart.

None of this depends on the transport: a Wi-Fi outage is simply "no valid
heartbeat". The motor driver here is a MOCK that energises nothing.
"""
from autonomy import protocol as p

FIRMWARE_VERSION = 2
RX_BUFFER_BYTES = 1024      # bytes one loop pass may receive (UART buffer)
SERIAL_PEER = "serial"


def saturate16(value):
    return max(0, min(int(value), 0xFFFF))


class MockMotorDriver:
    """Records the requested body velocity; touches no pin.

    history - (now_ms, enabled, forward_mm_s, turn_cdeg_s) on every change
    refused - set_body() calls with motion while disabled (must stay 0)
    """
    KIND = p.MotorDriverKind.MOCK

    def __init__(self):
        self.enabled = False
        self.forward_mm_s = 0
        self.turn_cdeg_s = 0
        self.faulted = False        # tests inject a driver fault with this
        self.refused = 0
        self.history = []

    def enable(self, now_ms):
        self.forward_mm_s = self.turn_cdeg_s = 0
        self.enabled = True
        self._record(now_ms)

    def disable(self, now_ms):
        self.forward_mm_s = self.turn_cdeg_s = 0
        self.enabled = False
        self._record(now_ms)

    def set_body(self, now_ms, forward_mm_s, turn_cdeg_s):
        if not self.enabled and (forward_mm_s or turn_cdeg_s):
            self.refused += 1
            return False
        if (forward_mm_s, turn_cdeg_s) != (self.forward_mm_s, self.turn_cdeg_s):
            self.forward_mm_s, self.turn_cdeg_s = int(forward_mm_s), int(turn_cdeg_s)
            self._record(now_ms)
        return True

    def moving(self):
        return bool(self.forward_mm_s or self.turn_cdeg_s)

    def _record(self, now_ms):
        entry = (now_ms, self.enabled, self.forward_mm_s, self.turn_cdeg_s)
        if not self.history or self.history[-1][1:] != entry[1:]:
            self.history.append(entry)


class FirmwareModel:
    """The firmware's state machine; see the module docstring."""

    def __init__(self, now_ms=0, boot_id=1, motor=None, scan_provider=None,
                 heartbeat_timeout_ms=p.HEARTBEAT_TIMEOUT_MS,
                 telemetry_period_ms=p.TELEMETRY_PERIOD_MS):
        self.motor = motor or MockMotorDriver()
        # scan_provider() -> [(angle_cdeg, distance_mm, intensity), ...] or
        # None (no LiDAR data); stands in for the LD06 acquisition.
        self.scan_provider = scan_provider
        self.heartbeat_timeout_ms = heartbeat_timeout_ms
        self.telemetry_period_ms = telemetry_period_ms
        self.restart(now_ms, boot_id)

    # -- lifecycle --------------------------------------------------------
    def restart(self, now_ms, boot_id):
        """Power-on / reset: everything volatile is lost, motors disabled."""
        self.motor.disable(now_ms)          # first action, as in setup()
        self.state = p.State.BOOT
        self.now = self.boot_ms = now_ms
        self.boot_id = boot_id
        self.decoder = p.FrameDecoder()
        self.tx = []                        # (peer or None, frame bytes)
        self.seq = p.SequenceCounter()
        self.faults = p.Fault.NONE
        self.controller = None              # (session_id, peer) once bound
        self.last_controller_ms = None
        self.last_heartbeat_ms = None
        self.last_echo = None
        self.rejected_heartbeats = 0
        self.last_rx_seq = 0
        self.last_move_seq = None           # no MOVE accepted since boot
        self.move = None
        self.scan_id = 0
        self.last_scan = None               # chunks of the last scan, for SCAN_RESEND
        self.next_telemetry_ms = now_ms
        self.state = p.State.DISARMED

    # -- inputs -----------------------------------------------------------
    def receive(self, data, now_ms, peer=SERIAL_PEER, datagram=False):
        """Bytes received from `peer` during one loop pass.

        datagram=True (UDP): `data` is one whole datagram; an incomplete
        frame left at its end is discarded, never joined to the next one.
        """
        self.now = now_ms
        if len(data) > RX_BUFFER_BYTES:
            data = data[:RX_BUFFER_BYTES]
            self._fault(p.Fault.RX_OVERFLOW)
        for frame in self.decoder.feed(data):
            self._handle(frame, peer)
        if datagram:
            self.decoder.discarded += len(self.decoder.buffer)
            self.decoder.buffer.clear()

    def step(self, now_ms):
        """End of a loop pass: faults, watchdog, motion, binding, telemetry."""
        self.now = now_ms
        if self.motor.faulted:
            self._fault(p.Fault.DRIVER_FAULT)
        if self.state in (p.State.ARMED, p.State.MOVING) and not self.heartbeat_live():
            self._fault(p.Fault.HEARTBEAT_TIMEOUT)
        if self.move is not None:
            self._run_motion()
        if (self.controller is not None and self.state in (p.State.DISARMED, p.State.FAULT)
                and now_ms - self.last_controller_ms > p.SESSION_RELEASE_MS):
            self.controller = None          # a silent controller loses the robot
        if now_ms >= self.next_telemetry_ms:
            self._send(self.telemetry())
            self.next_telemetry_ms = max(self.next_telemetry_ms + self.telemetry_period_ms,
                                         now_ms + 1)

    def read_frames(self):
        return [frame for _, frame in self.read_addressed()]

    def read_addressed(self):
        tx, self.tx = self.tx, []
        return tx

    # -- queries ----------------------------------------------------------
    def uptime(self):
        return (self.now - self.boot_ms) % p.UPTIME_MODULO

    def heartbeat_live(self):
        return (self.last_heartbeat_ms is not None
                and self.now - self.last_heartbeat_ms <= self.heartbeat_timeout_ms)

    def telemetry(self):
        move = self.move or {}
        age = (self.now - self.last_heartbeat_ms if self.last_heartbeat_ms is not None
               else 0xFFFF)
        return p.Telemetry(
            state=self.state, faults=int(self.faults), boot_id=self.boot_id,
            uptime_ms=self.uptime(),
            rx_frames=self.decoder.frames & 0xFFFFFFFF,
            crc_errors=saturate16(self.decoder.crc_errors),
            framing_errors=saturate16(self.decoder.framing_errors),
            last_seq=self.last_rx_seq, heartbeat_age_ms=saturate16(age),
            move_seq=self.last_move_seq or 0,
            move_progress_pct=self._progress(move) if move else
            (100 if self.last_move_seq is not None else 0),
            cmd_forward_mm_s=self.motor.forward_mm_s, cmd_turn_cdeg_s=self.motor.turn_cdeg_s,
            outputs_enabled=int(self.motor.enabled))

    def invariant_violations(self):
        """Safety invariants that must hold after every receive() / step()."""
        problems = []
        if self.motor.enabled and self.state not in (p.State.ARMED, p.State.MOVING):
            problems.append(f"driver enabled in {self.state.name}")
        if self.motor.moving() and self.state != p.State.MOVING:
            problems.append(f"outputs non-zero in {self.state.name}")
        if (self.move is not None) != (self.state == p.State.MOVING):
            problems.append("motion profile and MOVING state disagree")
        if self.faults & p.LATCHING_FAULTS and self.state != p.State.FAULT:
            problems.append("latching fault outside FAULT")
        if self.state in (p.State.ARMED, p.State.MOVING) and self.controller is None:
            problems.append("armed without a bound controller")
        if self.motor.refused:
            problems.append("motion requested from a disabled driver")
        return problems

    # -- command handling -------------------------------------------------
    def _handle(self, frame, peer):
        self.last_rx_seq = frame.seq
        if frame.type not in p.HOST_TO_ROBOT:
            reason = (p.NackReason.WRONG_DIRECTION if frame.type in p.MESSAGES
                      else p.NackReason.UNKNOWN_TYPE)
            return self._nack(frame, reason, peer)
        try:
            message = p.decode_message(frame)
        except p.ProtocolError as error:
            return self._nack(frame, error.reason, peer)
        from_controller = self.controller is not None and self.controller[1] == peer
        if from_controller:
            self.last_controller_ms = self.now
        if frame.type == p.MsgType.HELLO:
            return self._on_hello(frame, message, peer)
        if frame.type == p.MsgType.STOP:          # safe direction: anyone may stop
            return self._on_stop(frame, peer)
        if frame.type == p.MsgType.DISARM:
            return self._on_disarm(frame, peer, from_controller)
        if not from_controller:
            if frame.type != p.MsgType.HEARTBEAT:  # a foreign heartbeat is ignored
                self._nack(frame, p.NackReason.NOT_CONTROLLER, peer)
            return None
        handler = {
            p.MsgType.HEARTBEAT: self._on_heartbeat, p.MsgType.ARM: self._on_arm,
            p.MsgType.CLEAR_FAULT: self._on_clear_fault, p.MsgType.MOVE: self._on_move,
            p.MsgType.SCAN_REQUEST: self._on_scan_request,
            p.MsgType.SCAN_RESEND: self._on_scan_resend,
        }[frame.type]
        return handler(frame, message, peer)

    def _on_hello(self, frame, hello, peer):
        if self.controller is not None and self.controller[0] != hello.session_id:
            return self._nack(frame, p.NackReason.NOT_CONTROLLER, peer)
        # Same session from a new address (e.g. after a Wi-Fi reconnect) rebinds.
        self.controller = (hello.session_id, peer)
        self.last_controller_ms = self.now
        self._send(p.HelloReply(p.VERSION, FIRMWARE_VERSION, self.boot_id, p.MAX_PAYLOAD,
                                self.motor.KIND, self.uptime()), peer)

    def _on_heartbeat(self, frame, heartbeat, peer):
        echo = heartbeat.echo_uptime_ms
        fresh = p.uptime_age(self.uptime(), echo) <= p.HEARTBEAT_ECHO_MAX_MS
        ordered = self.last_echo is None or not p.uptime_after(self.last_echo, echo)
        if fresh and ordered:
            self.last_heartbeat_ms = self.now
            self.last_echo = echo
        else:
            self.rejected_heartbeats += 1

    def _on_arm(self, frame, _, peer):
        if self.state == p.State.FAULT:
            return self._nack(frame, p.NackReason.FAULT_ACTIVE, peer)
        if not self.heartbeat_live():
            return self._nack(frame, p.NackReason.NO_HEARTBEAT, peer)
        if self.state == p.State.DISARMED:
            self.motor.enable(self.now)
            self.state = p.State.ARMED
        self._ack(frame, peer)

    def _on_disarm(self, frame, peer, from_controller):
        if self.move is not None:
            self._end_move(p.MoveOutcome.STOPPED)
        self.motor.disable(self.now)
        if self.state != p.State.FAULT:
            self.state = p.State.DISARMED
        if from_controller:
            self.controller = None          # the controller hands the robot back
        self._ack(frame, peer)

    def _on_stop(self, frame, peer):
        if self.move is not None:
            self._end_move(p.MoveOutcome.STOPPED)
            self.state = p.State.ARMED
        self.motor.set_body(self.now, 0, 0)
        self._ack(frame, peer)

    def _on_clear_fault(self, frame, _, peer):
        if not self.heartbeat_live():
            return self._nack(frame, p.NackReason.NO_HEARTBEAT, peer)
        self.faults &= ~p.LATCHING_FAULTS
        if self.state == p.State.FAULT:
            self.state = p.State.DISARMED
        self._ack(frame, peer)

    def _on_move(self, frame, move, peer):
        # Replay protection first: a retransmitted MOVE whose ACK was lost
        # is answered STALE_SEQ and never executed twice. Then expiry: a
        # MOVE delayed in the network beyond its deadline never runs.
        if self.last_move_seq is not None and not p.seq_newer(frame.seq, self.last_move_seq):
            return self._nack(frame, p.NackReason.STALE_SEQ, peer)
        if p.uptime_after(self.uptime(), move.valid_until_ms):
            return self._nack(frame, p.NackReason.EXPIRED, peer)
        if self.state == p.State.FAULT:
            return self._nack(frame, p.NackReason.FAULT_ACTIVE, peer)
        if self.state == p.State.DISARMED:
            return self._nack(frame, p.NackReason.NOT_ARMED, peer)
        if self.state == p.State.MOVING:
            return self._nack(frame, p.NackReason.BUSY, peer)
        if not self.heartbeat_live():
            return self._nack(frame, p.NackReason.NO_HEARTBEAT, peer)
        reason = p.validate_move(move)
        if reason is not None:
            return self._nack(frame, reason, peer)
        self.last_move_seq = frame.seq
        self._ack(frame, peer)
        self._start_move(frame.seq, move)

    def _on_scan_request(self, frame, _, peer):
        if self.state == p.State.MOVING:
            return self._nack(frame, p.NackReason.BUSY, peer)   # stop-and-scan only
        points = self.scan_provider() if self.scan_provider else None
        if not points:
            return self._nack(frame, p.NackReason.LIDAR_UNAVAILABLE, peer)
        self._ack(frame, peer)
        self.scan_id = (self.scan_id + 1) % p.SEQ_MODULO
        self.last_scan = p.scan_to_chunks(self.scan_id, points)
        for chunk in self.last_scan:
            self._send(chunk)

    def _on_scan_resend(self, frame, resend, peer):
        chunks = self.last_scan
        if (not chunks or chunks[0].scan_id != resend.scan_id
                or resend.chunk_index >= len(chunks)):
            return self._nack(frame, p.NackReason.LIDAR_UNAVAILABLE, peer)
        self._send(chunks[resend.chunk_index])

    # -- motion -----------------------------------------------------------
    def _start_move(self, seq, move):
        turn_ms = (abs(move.turn_cdeg) * 1000.0 / move.turn_rate_cdeg_s
                   if move.turn_cdeg else 0.0)
        self.move = dict(seq=seq, command=move, start_ms=self.now, turn_ms=turn_ms,
                         total_ms=move.duration_ms(), timeout_ms=move.timeout_ms)
        self.state = p.State.MOVING
        self._run_motion()                  # first outputs in the same pass

    def _run_motion(self):
        move = self.move
        elapsed = self.now - move["start_ms"]
        command = move["command"]
        if elapsed >= move["total_ms"]:
            self._end_move(p.MoveOutcome.COMPLETED)
            self.state = p.State.ARMED
        elif elapsed > move["timeout_ms"]:
            self._fault(p.Fault.MOTION_TIMEOUT, p.MoveOutcome.TIMEOUT)
        elif elapsed < move["turn_ms"]:
            sign = 1 if command.turn_cdeg > 0 else -1
            self.motor.set_body(self.now, 0, sign * command.turn_rate_cdeg_s)
        else:
            sign = 1 if command.forward_mm > 0 else -1
            self.motor.set_body(self.now, sign * command.speed_mm_s, 0)

    def _end_move(self, outcome):
        """Zero the outputs and report; the caller sets the next state."""
        self.motor.set_body(self.now, 0, 0)
        self._send(p.MoveDone(self.move["seq"], outcome, int(self.faults)))
        self.move = None

    def _progress(self, move):
        if not move["total_ms"]:
            return 100
        return min(100, int((self.now - move["start_ms"]) * 100 // move["total_ms"]))

    def _fault(self, bits, outcome=p.MoveOutcome.FAULT):
        self.faults |= bits
        if not bits & p.LATCHING_FAULTS:
            return
        if self.move is not None:
            self._end_move(outcome)
        self.motor.disable(self.now)
        self.state = p.State.FAULT

    # -- replies ----------------------------------------------------------
    def _send(self, message, peer=None):
        """peer None = to the bound controller (telemetry, MOVE_DONE, scans);
        None also when unbound: a serial line delivers it, UDP drops it."""
        if peer is None and self.controller is not None:
            peer = self.controller[1]
        self.tx.append((peer, p.encode(message, self.seq.next())))

    def _ack(self, frame, peer):
        self._send(p.Ack(frame.seq, frame.type), peer)

    def _nack(self, frame, reason, peer):
        self._send(p.Nack(frame.seq, frame.type & 0xFF, reason), peer)
