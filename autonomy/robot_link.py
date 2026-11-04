"""Host side of the robot link: RobotLink, ProtocolRobotLink and transports.

The explorer only ever talks to a RobotLink. ProtocolRobotLink holds ALL the
host behaviour of protocol v2 (sequence numbers, retries, STOP re-send,
heartbeat echo, command expiry, link-loss policy, restart detection, scan
assembly and chunk resend) and runs over any Transport (its `wire`):

    SimSerialTransport    MockTransport byte pipe -> FirmwareModel (simulation)
    SimNetworkTransport   datagram simulator (autonomy/sim_network.py, step 2c)
    SerialTransport       USB serial fallback   } declared; refuse to start
    UdpTransport          Wi-Fi, primary        } until the hardware stage

so the serial and Wi-Fi links have identical command and telemetry
semantics (docs/design/wifi_transport.md). Nothing here opens a port.

Time: a simulated transport advances its clock (TICK_MS steps) only while a
call waits or pump() runs, so tests are deterministic. While connected,
every tick sends a HEARTBEAT when one is due.

Thread safety: one thread drives the link (the engine); stop() may be
called from any other thread (the GUI). It queues STOP at once, the driving
thread delivers it within one tick, and it is re-sent until acknowledged.
"""
import math
import random
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass

from autonomy import protocol as p
from autonomy.firmware_model import SERIAL_PEER, FirmwareModel

REPLY_TIMEOUT_MS = 150       # ACK / NACK expected within this
COMMAND_ATTEMPTS = 3         # same frame (same seq) re-sent on silence
STOP_RESEND_MS = 50
STOP_CONFIRM_MS = 200        # STOP unacknowledged for longer: starve the watchdog
TELEMETRY_SILENCE_MS = 1000  # no telemetry for longer: the link is lost
MOVE_DONE_MARGIN_MS = 500    # wait beyond the move's timeout for MOVE_DONE
SCAN_GAP_MS = 150            # no new chunk for this long: ask for the missing ones
SCAN_RESEND_ROUNDS = 3
DEFAULT_SPEED_M_S = 0.15
DEFAULT_TURN_RATE_RAD_S = math.radians(60.0)


class LinkError(RuntimeError):
    """The link cannot be used (wrong protocol, physical driver, other
    controller, not connected)."""


@dataclass(frozen=True)
class CommandResult:
    ok: bool
    reason: str = "ok"      # "ok", a NackReason name, "timeout", "sent"


@dataclass(frozen=True)
class MoveResult:
    outcome: str            # completed / stopped / fault / timeout / refused /
    #                         link_lost / restarted
    seq: int = None
    faults: int = 0
    detail: str = ""

    @property
    def completed(self):
        return self.outcome == "completed"


@dataclass(frozen=True)
class ScanResult:
    ok: bool
    points: tuple = ()      # raw LD06 ((angle_cdeg clockwise, distance_mm, intensity), ...)
    reason: str = "ok"
    timestamp_ms: int = None
    resent_chunks: int = 0


OUTCOME_NAMES = {p.MoveOutcome.COMPLETED: "completed", p.MoveOutcome.STOPPED: "stopped",
                 p.MoveOutcome.FAULT: "fault", p.MoveOutcome.TIMEOUT: "timeout"}


def make_move(forward_m, turn_rad, speed_m_s=DEFAULT_SPEED_M_S,
              turn_rate_rad_s=DEFAULT_TURN_RATE_RAD_S, valid_until_ms=0):
    """A protocol MOVE (turn first, then forward) with a timeout of 1.5 x
    the nominal duration + 200 ms, capped at MAX_TIMEOUT_MS."""
    move = p.Move(forward_mm=int(round(forward_m * 1000.0)),
                  turn_cdeg=int(round(math.degrees(turn_rad) * 100.0)),
                  speed_mm_s=int(round(abs(speed_m_s) * 1000.0)),
                  turn_rate_cdeg_s=int(round(math.degrees(abs(turn_rate_rad_s)) * 100.0)),
                  timeout_ms=1)
    nominal = 0.0
    if move.turn_cdeg and move.turn_rate_cdeg_s:
        nominal += abs(move.turn_cdeg) * 1000.0 / move.turn_rate_cdeg_s
    if move.forward_mm and move.speed_mm_s:
        nominal += abs(move.forward_mm) * 1000.0 / move.speed_mm_s
    timeout = min(p.MAX_TIMEOUT_MS, int(math.ceil(nominal * 1.5 + 200.0)))
    return p.Move(move.forward_mm, move.turn_cdeg, move.speed_mm_s, move.turn_rate_cdeg_s,
                  timeout, valid_until_ms % p.UPTIME_MODULO)


class RobotLink(ABC):
    """What the explorer may ask of a robot. Units: metres, radians."""

    @abstractmethod
    def connect(self):
        """HELLO handshake; returns the HelloReply. Raises LinkError."""

    @abstractmethod
    def arm(self) -> CommandResult: ...

    @abstractmethod
    def disarm(self) -> CommandResult: ...

    @abstractmethod
    def stop(self) -> CommandResult:
        """Safe from any thread; never waits behind another call."""

    @abstractmethod
    def clear_fault(self) -> CommandResult: ...

    @abstractmethod
    def move(self, forward_m, turn_rad, speed_m_s=DEFAULT_SPEED_M_S,
             turn_rate_rad_s=DEFAULT_TURN_RATE_RAD_S) -> MoveResult:
        """Turn in place, then drive straight; returns when the move ended."""

    @abstractmethod
    def request_scan(self, timeout_ms=2000) -> ScanResult: ...

    @abstractmethod
    def pump(self, duration_ms):
        """Keep the link alive (heartbeats) for duration_ms while idle."""

    @abstractmethod
    def close(self): ...


# ---------------------------------------------------------------------------
# Transports
# ---------------------------------------------------------------------------
class Transport(ABC):
    """Carries protocol frames between the host and one robot.

    datagrams - True if every received chunk is one whole datagram (UDP):
                each is decoded on its own, so a damaged datagram cannot
                disturb the next one.
    """
    datagrams = False

    @property
    @abstractmethod
    def now_ms(self): ...

    @abstractmethod
    def send(self, frame):
        """Queue one frame for the robot (never blocks)."""

    @abstractmethod
    def tick(self, dt_ms):
        """Let dt_ms pass; return the received chunks (bytes) in arrival order."""

    def close(self):
        pass


class SerialTransport(Transport):
    """USB serial (CH340, 115200 baud): development fallback. Not implemented."""

    def __new__(cls, *args, **kwargs):
        raise NotImplementedError(
            "SerialTransport needs the hardware-verification stage, which has not been "
            "approved. Use the simulated transports.")


class UdpTransport(Transport):
    """Wi-Fi UDP to the robot's SoftAP: the primary transport. Not implemented."""

    def __new__(cls, *args, **kwargs):
        raise NotImplementedError(
            "UdpTransport needs the hardware-verification stage (H1), which has not been "
            "approved. Use SimNetworkTransport.")


class MockTransport:
    """Two one-way byte pipes at a serial baud rate, with injectable faults.

    Writes are whole frames: drop_rate drops a frame, corrupt_rate flips one
    random bit in it (per direction). blackout drops everything, including
    bytes already in flight. Delivery is limited to baud / 10 bytes per
    second (8N1), so a 2.3 kB scan takes about 0.2 s at 115200 baud.
    """
    DIRECTIONS = ("to_robot", "to_host")

    def __init__(self, baud=115200, drop_rate=0.0, corrupt_rate=0.0, seed=0):
        self.bytes_per_ms = baud / 10000.0
        self.drop_rate = {d: drop_rate for d in self.DIRECTIONS}
        self.corrupt_rate = {d: corrupt_rate for d in self.DIRECTIONS}
        self.blackout = False
        self.rng = random.Random(seed)
        self.queues = {d: bytearray() for d in self.DIRECTIONS}
        self.budget = {d: 0.0 for d in self.DIRECTIONS}
        self.dropped = {d: 0 for d in self.DIRECTIONS}
        self.corrupted = {d: 0 for d in self.DIRECTIONS}

    def write(self, direction, frame):
        if self.blackout or self.rng.random() < self.drop_rate[direction]:
            self.dropped[direction] += 1
            return
        data = bytearray(frame)
        if self.rng.random() < self.corrupt_rate[direction]:
            data[self.rng.randrange(len(data))] ^= 1 << self.rng.randrange(8)
            self.corrupted[direction] += 1
        self.queues[direction] += data

    def read(self, direction, dt_ms):
        queue = self.queues[direction]
        if not queue:
            self.budget[direction] = 0.0
            return b""
        self.budget[direction] += dt_ms * self.bytes_per_ms
        count = min(len(queue), int(self.budget[direction]))
        self.budget[direction] -= count
        data = bytes(queue[:count])
        del queue[:count]
        return b"" if self.blackout else data


class SimSerialTransport(Transport):
    """A simulated USB serial line (MockTransport pipe) to a FirmwareModel.

    plant - optional object with advance(forward_mm_s, turn_cdeg_s, enabled,
            dt_ms): the simulated robot, fed the driver's requested velocity
            every tick (ground truth stays inside the plant).
    """

    def __init__(self, firmware, pipe=None, plant=None):
        self.firmware = firmware
        self.pipe = pipe or MockTransport()
        self.plant = plant
        self._now = firmware.now

    @property
    def now_ms(self):
        return self._now

    def send(self, frame):
        self.pipe.write("to_robot", frame)

    def tick(self, dt_ms):
        self._now += dt_ms
        self.firmware.receive(self.pipe.read("to_robot", dt_ms), self._now, SERIAL_PEER)
        self.firmware.step(self._now)
        for frame in self.firmware.read_frames():
            self.pipe.write("to_host", frame)
        received = self.pipe.read("to_host", dt_ms)
        if self.plant is not None:
            motor = self.firmware.motor
            self.plant.advance(motor.forward_mm_s, motor.turn_cdeg_s, motor.enabled, dt_ms)
        return [received] if received else []


# ---------------------------------------------------------------------------
# The protocol client
# ---------------------------------------------------------------------------
class ProtocolRobotLink(RobotLink):
    """RobotLink speaking protocol v2 over any Transport (module docstring)."""
    TICK_MS = 10

    def __init__(self, wire, session_id=None, realtime=0.0):
        self.wire = wire
        self.session_id = (random.SystemRandom().getrandbits(32) if session_id is None
                           else session_id % p.UPTIME_MODULO)
        self.realtime = realtime
        self.lock = threading.RLock()
        self.decoder = p.FrameDecoder()
        self.seq = p.SequenceCounter()
        self.replies = {}           # ref_seq -> Ack | Nack
        self.move_done = {}         # ref_seq -> MoveDone
        self.hello = None
        self.assembler = p.ScanAssembler()
        self.scans = []             # (timestamp_ms, points) completed scans
        self.last_chunk_ms = None
        self.telemetry = None
        self.telemetry_ms = None
        self.robot_clock = None     # (robot uptime, host now_ms when it arrived)
        self.boot_id = None
        self.old_boot_ids = set()
        self.restarted = False
        self.connected = False
        self.connected_ms = None
        self.heartbeat_enabled = True   # tests turn it off to simulate a host hang
        self.next_heartbeat_ms = 0
        self.stop_seq = None            # pending STOP, re-sent until acknowledged
        self.next_stop_ms = 0
        self.closed = False

    @property
    def now_ms(self):
        return self.wire.now_ms

    # -- the wire ---------------------------------------------------------
    def _next_seq(self):
        with self.lock:
            return self.seq.next()

    def _send(self, message, seq=None):
        with self.lock:
            seq = self.seq.next() if seq is None else seq
            self.wire.send(p.encode(message, seq))
        return seq

    def _tick(self):
        with self.lock:
            due = self.now_ms + self.TICK_MS
            if (self.connected and self.heartbeat_enabled and self.robot_clock is not None
                    and due >= self.next_heartbeat_ms):
                self._send(p.Heartbeat(self.robot_clock[0]))
                self.next_heartbeat_ms = due + p.HEARTBEAT_PERIOD_MS
            if self.stop_seq is not None and due >= self.next_stop_ms:
                self._send(p.Stop(), self.stop_seq)
                self.next_stop_ms = due + STOP_RESEND_MS
            for chunk in self.wire.tick(self.TICK_MS):
                decoder = p.FrameDecoder() if self.wire.datagrams else self.decoder
                for frame in decoder.feed(chunk):
                    self._on_frame(frame)
        if self.realtime:
            time.sleep(self.TICK_MS / 1000.0 * self.realtime)

    def _robot_time(self, uptime, boot_id):
        """Track the newest robot uptime (datagrams may arrive out of order)."""
        if boot_id != self.boot_id:
            return
        if self.robot_clock is None or p.uptime_after(uptime, self.robot_clock[0]):
            self.robot_clock = (uptime, self.now_ms)

    def _on_frame(self, frame):
        try:
            message = p.decode_message(frame)
        except p.ProtocolError:
            return
        if isinstance(message, (p.Ack, p.Nack)):
            # A duplicated command (UDP) gets an ACK and then a NACK for the
            # same seq: the ACK must never be overwritten by that NACK.
            if not isinstance(self.replies.get(message.ref_seq), p.Ack):
                self.replies[message.ref_seq] = message
            if isinstance(message, p.Ack) and message.ref_seq == self.stop_seq:
                self.stop_seq = None
        elif isinstance(message, p.MoveDone):
            self.move_done[message.ref_seq] = message
        elif isinstance(message, p.Telemetry):
            if message.boot_id in self.old_boot_ids:
                return                      # a late frame from before a restart
            if self.boot_id is not None and message.boot_id != self.boot_id:
                self.restarted = True
                return
            newer = (self.telemetry is None
                     or p.uptime_after(message.uptime_ms, self.telemetry.uptime_ms))
            if newer:
                self.telemetry, self.telemetry_ms = message, self.now_ms
            self._robot_time(message.uptime_ms, message.boot_id)
        elif isinstance(message, p.HelloReply):
            self.hello = message
        elif isinstance(message, p.ScanChunk):
            self.last_chunk_ms = self.now_ms
            points = self.assembler.add(message)
            if points is not None:
                self.scans.append((self.now_ms, points))

    def _wait(self, done, timeout_ms):
        """Tick until done() is true or timeout_ms passes; returns done()."""
        end = self.now_ms + timeout_ms
        while not done() and self.now_ms < end:
            self._tick()
        return done()

    def _link_silent(self):
        heard = self.telemetry_ms if self.telemetry_ms is not None else self.connected_ms
        return heard is None or self.now_ms - heard > TELEMETRY_SILENCE_MS

    def _valid_until(self):
        """Command deadline in ROBOT uptime: newest echo + host time since + TTL.

        The robot accepts the command only if the round trip (telemetry
        to us, command to it) took at most COMMAND_TTL_MS.
        """
        uptime, received = self.robot_clock
        return (uptime + (self.now_ms - received) + p.COMMAND_TTL_MS) % p.UPTIME_MODULO

    def _ensure_stopped(self):
        """After a lost link: never return while the robot may still move.

        STOP is re-sent until acknowledged; if no ACK comes within
        STOP_CONFIRM_MS, heartbeats are withheld for longer than the
        firmware watchdog, which then disables the driver by itself.
        """
        self.stop()
        if self._wait(lambda: self.stop_seq is None, STOP_CONFIRM_MS):
            return "stop acknowledged"
        self.heartbeat_enabled = False
        self._wait(lambda: False, p.HEARTBEAT_TIMEOUT_MS + 2 * p.HEARTBEAT_PERIOD_MS)
        self.heartbeat_enabled = True
        return "stop unconfirmed: watchdog starved"

    def _command(self, message):
        """Send with retries (same seq); returns (seq, Ack | Nack | None)."""
        self._require_connected()
        seq = self._next_seq()
        for _ in range(COMMAND_ATTEMPTS):
            self._send(message, seq)
            if self._wait(lambda: seq in self.replies, REPLY_TIMEOUT_MS):
                return seq, self.replies.pop(seq)
        return seq, None

    @staticmethod
    def _result(reply):
        if reply is None:
            return CommandResult(False, "timeout")
        if isinstance(reply, p.Ack):
            return CommandResult(True)
        return CommandResult(False, p.NackReason(reply.reason).name)

    def _require_connected(self):
        if self.closed:
            raise LinkError("the link is closed")
        if not self.connected:
            raise LinkError("not connected: call connect() first")

    # -- RobotLink --------------------------------------------------------
    def connect(self):
        if self.closed:
            raise LinkError("the link is closed")
        self.hello = None
        seq = self._next_seq()
        for _ in range(COMMAND_ATTEMPTS):
            self._send(p.Hello(self.session_id), seq)
            if self._wait(lambda: self.hello is not None or seq in self.replies,
                          REPLY_TIMEOUT_MS):
                break
        else:
            raise LinkError("no HELLO_REPLY: robot not responding")
        if self.hello is None:
            reason = p.NackReason(self.replies.pop(seq).reason).name
            raise LinkError(f"robot refused HELLO ({reason}): another session controls it")
        if self.hello.protocol_version != p.VERSION:
            raise LinkError(f"robot speaks protocol v{self.hello.protocol_version}, "
                            f"host speaks v{p.VERSION}")
        if self.hello.motor_driver != p.MotorDriverKind.MOCK:
            raise LinkError("robot reports a PHYSICAL motor driver: physical motion "
                            "needs the approved hardware-verification stage")
        if self.boot_id is not None and self.boot_id != self.hello.boot_id:
            self.old_boot_ids.add(self.boot_id)
        self.boot_id, self.restarted, self.connected = self.hello.boot_id, False, True
        self.connected_ms = self.now_ms
        self.telemetry = self.telemetry_ms = None
        self.robot_clock = None
        self._robot_time(self.hello.uptime_ms, self.hello.boot_id)
        self.next_heartbeat_ms = self.now_ms
        self._wait(lambda: False, p.HEARTBEAT_PERIOD_MS)   # first heartbeats + telemetry
        return self.hello

    def arm(self):
        return self._result(self._command(p.Arm())[1])

    def disarm(self):
        return self._result(self._command(p.Disarm())[1])

    def clear_fault(self):
        return self._result(self._command(p.ClearFault())[1])

    def stop(self):
        """Queue STOP now (any thread); re-sent every 50 ms until acknowledged."""
        with self.lock:
            if self.stop_seq is None:
                self.stop_seq = self.seq.next()
            self._send(p.Stop(), self.stop_seq)
            self.next_stop_ms = self.now_ms + STOP_RESEND_MS
        return CommandResult(True, "sent")

    def move(self, forward_m, turn_rad, speed_m_s=DEFAULT_SPEED_M_S,
             turn_rate_rad_s=DEFAULT_TURN_RATE_RAD_S):
        self._require_connected()
        command = make_move(forward_m, turn_rad, speed_m_s, turn_rate_rad_s)
        reason = p.validate_move(command)
        if reason is not None:
            return MoveResult("refused", detail=f"{reason.name}: {command}")
        if self.restarted:
            return MoveResult("restarted", detail="robot restarted: reconnect and re-arm")
        command = make_move(forward_m, turn_rad, speed_m_s, turn_rate_rad_s,
                            self._valid_until())
        seq = self._next_seq()
        accepted = False
        for attempt in range(COMMAND_ATTEMPTS):
            self._send(command, seq)     # the same frame: the deadline never moves
            if self._wait(lambda: seq in self.replies or seq in self.move_done,
                          REPLY_TIMEOUT_MS):
                reply = self.replies.pop(seq, None)
                if reply is None or isinstance(reply, p.Ack):
                    accepted = True
                elif reply.reason == p.NackReason.STALE_SEQ:
                    # Seqs are never reused, so STALE for THIS seq means a copy of
                    # this same MOVE (a retry or a network duplicate) was accepted.
                    accepted = True
                else:
                    return MoveResult("refused", seq, detail=p.NackReason(reply.reason).name)
                break
        if not accepted:
            return MoveResult("link_lost", seq, detail="no reply to MOVE; " + self._ensure_stopped())
        end = self.now_ms + command.timeout_ms + MOVE_DONE_MARGIN_MS
        while self.now_ms < end:
            if seq in self.move_done:
                done = self.move_done.pop(seq)
                return MoveResult(OUTCOME_NAMES[p.MoveOutcome(done.outcome)], seq, done.faults)
            if self.restarted:
                return MoveResult("restarted", seq, detail="robot restarted during the move")
            if self._link_silent():
                return MoveResult("link_lost", seq,
                                  detail="telemetry silent; " + self._ensure_stopped())
            self._tick()
        # MOVE_DONE lost: recover the outcome from telemetry if it reports this move.
        t = self.telemetry
        if t is not None and t.move_seq == seq and t.state != p.State.MOVING:
            outcome = "fault" if t.state == p.State.FAULT else (
                "completed" if t.move_progress_pct == 100 else "stopped")
            return MoveResult(outcome, seq, t.faults, detail="inferred from telemetry")
        return MoveResult("link_lost", seq, detail="no MOVE_DONE; " + self._ensure_stopped())

    def request_scan(self, timeout_ms=2000):
        """One stop-and-scan revolution; missing chunks are asked for again
        (SCAN_RESEND) before the scan is given up as incomplete."""
        start = len(self.scans)
        seq, reply = self._command(p.ScanRequest())
        if reply is None:
            return ScanResult(False, reason="timeout")
        if isinstance(reply, p.Nack):
            return ScanResult(False, reason=p.NackReason(reply.reason).name)
        self.last_chunk_ms = self.now_ms
        rounds = resent = 0
        end = self.now_ms + timeout_ms
        while self.now_ms < end:
            if len(self.scans) > start:
                timestamp, points = self.scans[-1]
                return ScanResult(True, points, timestamp_ms=timestamp, resent_chunks=resent)
            missing = self.assembler.missing()
            if (missing and rounds < SCAN_RESEND_ROUNDS
                    and self.now_ms - self.last_chunk_ms >= SCAN_GAP_MS):
                for index in missing:
                    self._send(p.ScanResend(self.assembler.scan_id, index))
                rounds, resent = rounds + 1, resent + len(missing)
                self.last_chunk_ms = self.now_ms
            self._tick()
        return ScanResult(False, reason="incomplete", resent_chunks=resent)

    def pump(self, duration_ms):
        self._wait(lambda: False, duration_ms)

    def close(self):
        """STOP and DISARM (best effort), then refuse further use."""
        if self.connected and not self.closed:
            self.stop()
            self._wait(lambda: self.stop_seq is None, 200)
            self.disarm()
        self.connected, self.closed = False, True
        self.wire.close()


class MockRobotLink(ProtocolRobotLink):
    """ProtocolRobotLink over a simulated USB serial line to a FirmwareModel.

    firmware  - the FirmwareModel (a default one if None)
    transport - the MockTransport byte pipe; tests inject faults through it
                (drop_rate, corrupt_rate, blackout, write)
    plant     - the simulated robot (see SimSerialTransport)
    """

    def __init__(self, firmware=None, transport=None, plant=None, realtime=0.0,
                 session_id=None):
        super().__init__(SimSerialTransport(firmware or FirmwareModel(), transport, plant),
                         session_id, realtime)
        self.transport = self.wire.pipe

    @property
    def firmware(self):
        return self.wire.firmware

    @property
    def plant(self):
        return self.wire.plant

    @plant.setter
    def plant(self, value):
        self.wire.plant = value


class SerialRobotLink(ProtocolRobotLink):
    """ProtocolRobotLink over USB serial (fallback). Not implemented yet."""

    def __new__(cls, *args, **kwargs):
        raise NotImplementedError(
            "SerialRobotLink needs the hardware-verification stage, which has not "
            "been approved. Use MockRobotLink.")


class WiFiRobotLink(ProtocolRobotLink):
    """ProtocolRobotLink over Wi-Fi UDP (primary). Not implemented yet."""

    def __new__(cls, *args, **kwargs):
        raise NotImplementedError(
            "WiFiRobotLink needs the hardware-verification stage (H1), which has not "
            "been approved. Use ProtocolRobotLink(SimNetworkTransport(...)).")
