"""Robot link protocol v2: framed binary messages between Python and the ESP32.

The specification is docs/design/robot_protocol_v2.md (v2 adds what Wi-Fi
needs: heartbeat echo, command expiry, session binding and chunk resend;
the framing is unchanged from v1). Transport: one frame per UDP datagram
(Wi-Fi, primary) or a byte stream (USB serial, fallback). This module and the
firmware (firmware/autonomy_robot/src/protocol.h) both implement it; the
golden frames of golden_vectors(), written to firmware/tests/protocol_vectors.h
by write_c_header(), keep the two byte-for-byte identical.

Frame (all integers little-endian):
    0xA5 0x5A | version u8 | type u8 | seq u16 | length u16 | hcrc u8 |
    payload | crc u16
    hcrc = CRC-8 (poly 0x07) over version .. length: a corrupted length is
    rejected at once instead of making the receiver wait for bytes that never
    come (which would starve the heartbeat watchdog).
    crc = CRC-16/CCITT-FALSE over version .. payload; length <= MAX_PAYLOAD.

Pure standard library. Nothing here opens a serial port or any device.

Write the firmware vectors (from the project folder):
    python -m autonomy.protocol --write-c-header firmware/tests/protocol_vectors.h
"""
import argparse
import math
import struct
from dataclasses import dataclass, fields
from enum import IntEnum, IntFlag
from pathlib import Path
from typing import ClassVar

# ---------------------------------------------------------------------------
# 1. Frame constants
# ---------------------------------------------------------------------------
SYNC = b"\xa5\x5a"
VERSION = 2
HEADER = struct.Struct("<2sBBHHB")  # sync, version, type, seq, length, hcrc
CRC = struct.Struct("<H")
HEADER_SIZE = HEADER.size           # 9
CRC_SIZE = CRC.size                 # 2
MAX_PAYLOAD = 512
MAX_FRAME = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE  # 523 bytes
SEQ_MODULO = 1 << 16

# Bounded motion (shared rule, see validate_move). A MOVE turns in place by
# turn_cdeg first, then drives straight forward_mm. Both stages must finish
# within timeout_ms at the requested speeds, so no move can outlive its timeout.
# Interim limits until motion is calibrated on the robot (decision
# docs/decisions/2026-11-04_interim_motion_limits_and_no_http.md; the original
# TP firmware's proven bound is 2 s per move).
MAX_FORWARD_MM = 300
MAX_TURN_CDEG = 9000                # 90 degrees
MAX_SPEED_MM_S = 200
MAX_TURN_RATE_CDEG_S = 9000         # 90 degrees per second
MAX_TIMEOUT_MS = 2000

# Watchdog and telemetry timing (the firmware's defaults; documented here so
# the host and the Python firmware model use the same numbers).
HEARTBEAT_TIMEOUT_MS = 300
HEARTBEAT_PERIOD_MS = 100
TELEMETRY_PERIOD_MS = 200
# v2 (docs/design/wifi_transport.md, section 2). Times are ROBOT uptime (ms,
# 32-bit, wrapping): the host only echoes values the robot sent, so no clock
# synchronisation is needed.
HEARTBEAT_ECHO_MAX_MS = 800        # a heartbeat counts only if its echo is this fresh
COMMAND_TTL_MS = 250               # MOVE valid_until = echo + host elapsed + TTL
SESSION_RELEASE_MS = 5000          # binding released after this much controller
#                                    silence, only while DISARMED or in FAULT
UPTIME_MODULO = 1 << 32


class MsgType(IntEnum):
    # host -> robot
    HELLO = 0x01
    HEARTBEAT = 0x02
    ARM = 0x03
    DISARM = 0x04
    STOP = 0x05
    CLEAR_FAULT = 0x06
    MOVE = 0x10
    SCAN_REQUEST = 0x11
    SCAN_RESEND = 0x12
    # robot -> host
    HELLO_REPLY = 0x80
    ACK = 0x81
    NACK = 0x82
    TELEMETRY = 0x83
    MOVE_DONE = 0x84
    SCAN_CHUNK = 0x85


class State(IntEnum):
    BOOT = 0
    DISARMED = 1
    ARMED = 2
    MOVING = 3
    FAULT = 4


class Fault(IntFlag):
    NONE = 0
    HEARTBEAT_TIMEOUT = 1 << 0      # no HEARTBEAT within HEARTBEAT_TIMEOUT_MS
    MOTION_TIMEOUT = 1 << 1         # a move outlived its timeout_ms
    DRIVER_FAULT = 1 << 2           # the motor driver reported a fault
    LIDAR_NO_DATA = 1 << 3          # informational: no LD06 packets lately
    RX_OVERFLOW = 1 << 4            # the receive buffer overflowed


# Faults that stop the motors and need CLEAR_FAULT; LIDAR_NO_DATA does not.
LATCHING_FAULTS = (Fault.HEARTBEAT_TIMEOUT | Fault.MOTION_TIMEOUT
                   | Fault.DRIVER_FAULT | Fault.RX_OVERFLOW)


class NackReason(IntEnum):
    BAD_LENGTH = 1                  # payload size wrong for the type
    UNKNOWN_TYPE = 2
    OUT_OF_BOUNDS = 3               # MOVE outside the limits above
    NOT_ARMED = 4
    FAULT_ACTIVE = 5
    STALE_SEQ = 6                   # MOVE seq not newer than the last MOVE
    BUSY = 7                        # e.g. SCAN_REQUEST while moving
    NO_HEARTBEAT = 8                # ARM without a live heartbeat
    LIDAR_UNAVAILABLE = 9
    WRONG_DIRECTION = 10            # a robot -> host type sent to the robot
    EXPIRED = 11                    # MOVE received after its valid_until_ms
    NOT_CONTROLLER = 12             # sender is not the bound session


class MoveOutcome(IntEnum):
    COMPLETED = 0
    STOPPED = 1                     # STOP or DISARM
    FAULT = 2                       # watchdog or driver fault
    TIMEOUT = 3


class MotorDriverKind(IntEnum):
    MOCK = 0                        # records outputs, energises nothing
    PHYSICAL = 1                    # not implemented: needs hardware approval


# ---------------------------------------------------------------------------
# 2. CRC-16/CCITT-FALSE and sequence numbers
# ---------------------------------------------------------------------------
# Polynomial 0x1021, initial value 0xFFFF, no reflection, no final XOR.
# Check value: crc16_ccitt(b"123456789") == 0x29B1.
CRC16_CHECK = 0x29B1


def _crc16_table():
    table = []
    for byte in range(256):
        crc = byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
        table.append(crc & 0xFFFF)
    return tuple(table)


CRC16_TABLE = _crc16_table()


def crc16_ccitt(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE of `data` (bytes-like), MSB first."""
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ CRC16_TABLE[(crc >> 8) ^ byte]
    return crc


CRC8_CHECK = 0xF4                   # crc8(b"123456789"), CRC-8/SMBUS


def crc8(data, crc=0):
    """CRC-8, polynomial 0x07, initial 0, MSB first (the header check)."""
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def seq_newer(seq, last):
    """True if `seq` comes after `last` in modulo-2^16 order.

    Serial-number arithmetic (RFC 1982 style): seq is newer when it is 1 to
    32767 steps ahead, so the counter can wrap from 65535 to 0.
    """
    return 0 < (seq - last) % SEQ_MODULO < SEQ_MODULO // 2


def uptime_after(a, b):
    """True if robot uptime `a` is later than `b` (32-bit wrapping)."""
    return 0 < (a - b) % UPTIME_MODULO < UPTIME_MODULO // 2


def uptime_age(now, then):
    """now - then in ms, 32-bit wrapping (a "future" `then` gives a huge age)."""
    return (now - then) % UPTIME_MODULO


class SequenceCounter:
    """The sender's seq: 1, 2, ..., 65535, 0, 1, ... (wraps)."""

    def __init__(self, start=1):
        self.value = start % SEQ_MODULO

    def next(self):
        seq = self.value
        self.value = (self.value + 1) % SEQ_MODULO
        return seq


# ---------------------------------------------------------------------------
# 3. Messages (payloads)
# ---------------------------------------------------------------------------
class ProtocolError(ValueError):
    """A frame decoded correctly but its payload is not a valid message."""

    def __init__(self, reason, message):
        super().__init__(message)
        self.reason = reason        # a NackReason the receiver can reply with


@dataclass(frozen=True)
class Message:
    """Fixed-layout message: the dataclass fields, packed with LAYOUT."""
    TYPE: ClassVar[MsgType]
    LAYOUT: ClassVar[struct.Struct] = struct.Struct("<")

    def payload(self):
        values = [getattr(self, item.name) for item in fields(self)]
        try:
            return self.LAYOUT.pack(*values)
        except struct.error as error:
            raise ValueError(f"{type(self).__name__}: {error}") from error

    @classmethod
    def from_payload(cls, payload):
        if len(payload) != cls.LAYOUT.size:
            raise ProtocolError(
                NackReason.BAD_LENGTH,
                f"{cls.__name__} payload is {len(payload)} bytes, "
                f"expected {cls.LAYOUT.size}")
        return cls(*cls.LAYOUT.unpack(payload))


@dataclass(frozen=True)
class Hello(Message):
    """Claims the robot for one session (random 32-bit id per host run)."""
    TYPE = MsgType.HELLO
    LAYOUT = struct.Struct("<I")
    session_id: int = 0


@dataclass(frozen=True)
class Heartbeat(Message):
    """echo_uptime_ms: robot uptime from the newest TELEMETRY / HELLO_REPLY
    the host received. Proves a recent round trip (HEARTBEAT_ECHO_MAX_MS)."""
    TYPE = MsgType.HEARTBEAT
    LAYOUT = struct.Struct("<I")
    echo_uptime_ms: int = 0


@dataclass(frozen=True)
class Arm(Message):
    TYPE = MsgType.ARM


@dataclass(frozen=True)
class Disarm(Message):
    TYPE = MsgType.DISARM


@dataclass(frozen=True)
class Stop(Message):
    TYPE = MsgType.STOP


@dataclass(frozen=True)
class ClearFault(Message):
    TYPE = MsgType.CLEAR_FAULT


@dataclass(frozen=True)
class ScanRequest(Message):
    TYPE = MsgType.SCAN_REQUEST


@dataclass(frozen=True)
class ScanResend(Message):
    """Re-send one chunk of the robot's last scan."""
    TYPE = MsgType.SCAN_RESEND
    LAYOUT = struct.Struct("<HB")
    scan_id: int
    chunk_index: int


@dataclass(frozen=True)
class Move(Message):
    """Turn in place by turn_cdeg (+ = counter-clockwise), then drive
    forward_mm (+ = forwards). Speeds are magnitudes; see validate_move.
    valid_until_ms: robot uptime after which the robot refuses it (EXPIRED).
    """
    TYPE = MsgType.MOVE
    LAYOUT = struct.Struct("<hhHHHI")
    forward_mm: int
    turn_cdeg: int
    speed_mm_s: int
    turn_rate_cdeg_s: int
    timeout_ms: int
    valid_until_ms: int = 0

    def duration_ms(self):
        """Nominal time of both stages at the requested speeds (ms)."""
        turn = (abs(self.turn_cdeg) * 1000.0 / self.turn_rate_cdeg_s
                if self.turn_cdeg else 0.0)
        drive = (abs(self.forward_mm) * 1000.0 / self.speed_mm_s
                 if self.forward_mm else 0.0)
        return turn + drive


@dataclass(frozen=True)
class HelloReply(Message):
    TYPE = MsgType.HELLO_REPLY
    LAYOUT = struct.Struct("<BHIHBI")
    protocol_version: int
    firmware_version: int
    boot_id: int                    # new random value at every restart
    max_payload: int
    motor_driver: int               # MotorDriverKind
    uptime_ms: int = 0              # first value the host can echo


@dataclass(frozen=True)
class Ack(Message):
    TYPE = MsgType.ACK
    LAYOUT = struct.Struct("<HB")
    ref_seq: int
    ref_type: int


@dataclass(frozen=True)
class Nack(Message):
    TYPE = MsgType.NACK
    LAYOUT = struct.Struct("<HBB")
    ref_seq: int
    ref_type: int
    reason: int                     # NackReason


@dataclass(frozen=True)
class Telemetry(Message):
    TYPE = MsgType.TELEMETRY
    LAYOUT = struct.Struct("<BHIIIHHHHHBhhB")
    state: int                      # State
    faults: int                     # Fault bits
    boot_id: int
    uptime_ms: int
    rx_frames: int                  # valid frames received
    crc_errors: int                 # saturating counters (65535 max)
    framing_errors: int
    last_seq: int                   # seq of the last valid frame received
    heartbeat_age_ms: int           # saturating at 65535
    move_seq: int                   # seq of the active / last MOVE
    move_progress_pct: int          # 0-100
    cmd_forward_mm_s: int           # body velocity requested from the driver
    cmd_turn_cdeg_s: int
    outputs_enabled: int            # 1 = the driver is enabled


@dataclass(frozen=True)
class MoveDone(Message):
    TYPE = MsgType.MOVE_DONE
    LAYOUT = struct.Struct("<HBH")
    ref_seq: int
    outcome: int                    # MoveOutcome
    faults: int


# Scan chunk: header, then (angle_cdeg u16, distance_mm u16, intensity u8)
# per point. Angles are COUNTER-CLOCKWISE from the sensor's +X (front), 0.01
# degree, [0, 36000): the Robot-Maker LD06 library's DataPoint.angle
# (360 - LD06 physical angle), as the original TP firmware sends them
# (decision docs/decisions/2026-11-04_scan_angles_ccw.md). The host converts
# with points_to_sensor_xy(). distance 0 = no echo (kept: the host filters).
SCAN_CHUNK_HEADER = struct.Struct("<HBB")    # scan_id, chunk_index, chunk_count
SCAN_POINT = struct.Struct("<HHB")
MAX_POINTS_PER_CHUNK = (MAX_PAYLOAD - SCAN_CHUNK_HEADER.size) // SCAN_POINT.size


@dataclass(frozen=True)
class ScanChunk:
    TYPE: ClassVar[MsgType] = MsgType.SCAN_CHUNK
    scan_id: int
    chunk_index: int
    chunk_count: int
    points: tuple                   # ((angle_cdeg, distance_mm, intensity), ...)

    def payload(self):
        if not 1 <= len(self.points) <= MAX_POINTS_PER_CHUNK:
            raise ValueError(f"a scan chunk holds 1 to {MAX_POINTS_PER_CHUNK} "
                             f"points, not {len(self.points)}")
        if not 0 <= self.chunk_index < self.chunk_count:
            raise ValueError("chunk_index must be below chunk_count")
        try:
            return SCAN_CHUNK_HEADER.pack(
                self.scan_id, self.chunk_index, self.chunk_count) + b"".join(
                SCAN_POINT.pack(*point) for point in self.points)
        except struct.error as error:
            raise ValueError(f"ScanChunk: {error}") from error

    @classmethod
    def from_payload(cls, payload):
        body = len(payload) - SCAN_CHUNK_HEADER.size
        if body < SCAN_POINT.size or body % SCAN_POINT.size:
            raise ProtocolError(NackReason.BAD_LENGTH,
                                f"ScanChunk payload of {len(payload)} bytes")
        scan_id, index, count = SCAN_CHUNK_HEADER.unpack_from(payload)
        if index >= count:
            raise ProtocolError(NackReason.BAD_LENGTH,
                                f"chunk {index} of {count}")
        points = tuple(SCAN_POINT.iter_unpack(payload[SCAN_CHUNK_HEADER.size:]))
        if any(angle >= 36000 for angle, _, _ in points):
            raise ProtocolError(NackReason.BAD_LENGTH, "angle >= 360 degrees")
        return cls(scan_id, index, count, points)


MESSAGES = {cls.TYPE: cls for cls in (
    Hello, Heartbeat, Arm, Disarm, Stop, ClearFault, ScanRequest, ScanResend, Move,
    HelloReply, Ack, Nack, Telemetry, MoveDone, ScanChunk)}
HOST_TO_ROBOT = frozenset(t for t in MsgType if t < 0x80)


def validate_move(move):
    """None if the MOVE is within bounds, else NackReason.OUT_OF_BOUNDS.

    The firmware applies the same rule before it moves anything:
      |forward_mm| <= 300, |turn_cdeg| <= 9000, timeout 1..2000 ms;
      a non-zero stage needs a speed of 1..MAX; at least one stage non-zero;
      the nominal duration must fit inside timeout_ms.
    """
    ok = (abs(move.forward_mm) <= MAX_FORWARD_MM
          and abs(move.turn_cdeg) <= MAX_TURN_CDEG
          and 0 < move.timeout_ms <= MAX_TIMEOUT_MS
          and move.speed_mm_s <= MAX_SPEED_MM_S
          and move.turn_rate_cdeg_s <= MAX_TURN_RATE_CDEG_S
          and (move.forward_mm or move.turn_cdeg)
          and (not move.forward_mm or move.speed_mm_s > 0)
          and (not move.turn_cdeg or move.turn_rate_cdeg_s > 0))
    if ok and move.duration_ms() <= move.timeout_ms:
        return None
    return NackReason.OUT_OF_BOUNDS


def points_to_sensor_xy(points, min_distance_mm=1):
    """Scan points -> (N, 2) list of (x_m, y_m), +X front, +Y left.

    points - ((angle_cdeg CCW, distance_mm, intensity), ...); distances below
    min_distance_mm (0 = no echo) are dropped. Pure Python (no NumPy) so the
    protocol module keeps no third-party dependency.
    """
    result = []
    for angle_cdeg, distance_mm, _ in points:
        if distance_mm < min_distance_mm:
            continue
        angle = math.radians(angle_cdeg / 100.0)
        result.append((distance_mm / 1000.0 * math.cos(angle),
                       distance_mm / 1000.0 * math.sin(angle)))
    return result


def scan_to_chunks(scan_id, points):
    """Split a scan [(angle_cdeg, distance_mm, intensity), ...] into chunks."""
    points = tuple(tuple(int(v) for v in point) for point in points)
    if not points:
        raise ValueError("an empty scan cannot be sent")
    pieces = [points[i:i + MAX_POINTS_PER_CHUNK]
              for i in range(0, len(points), MAX_POINTS_PER_CHUNK)]
    if len(pieces) > 255:
        raise ValueError("scan too large for 255 chunks")
    return [ScanChunk(scan_id % SEQ_MODULO, index, len(pieces), piece)
            for index, piece in enumerate(pieces)]


class ScanAssembler:
    """Reassemble SCAN_CHUNK messages into whole scans (v2: any order).

    add(chunk) returns the complete point tuple when the last missing chunk
    of a scan arrives, else None. Chunks may arrive out of order or twice
    (UDP). A chunk of another scan (different scan_id or chunk_count) drops
    the partial scan (counted in `dropped`): a scan is never stitched
    together from two revolutions. missing() lists the chunk indices still
    needed, for SCAN_RESEND. Late duplicates of the last completed scan are
    ignored.
    """

    def __init__(self):
        self.scan_id = None
        self.chunk_count = None
        self.parts = {}
        self.dropped = 0
        self.completed_id = None

    def add(self, chunk):
        if self.scan_id is None and chunk.scan_id == self.completed_id:
            return None
        if (chunk.scan_id, chunk.chunk_count) != (self.scan_id, self.chunk_count):
            if self.parts:
                self.dropped += 1
            self.scan_id, self.chunk_count, self.parts = chunk.scan_id, chunk.chunk_count, {}
        self.parts[chunk.chunk_index] = chunk
        if len(self.parts) < self.chunk_count:
            return None
        points = tuple(p for index in range(self.chunk_count)
                       for p in self.parts[index].points)
        self.completed_id = self.scan_id
        self.scan_id, self.chunk_count, self.parts = None, None, {}
        return points

    def missing(self):
        if self.scan_id is None:
            return []
        return [i for i in range(self.chunk_count) if i not in self.parts]


# ---------------------------------------------------------------------------
# 4. Frames: encoder and streaming decoder
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Frame:
    type: int
    seq: int
    payload: bytes
    version: int = VERSION


def encode_frame(msg_type, seq, payload=b"", version=VERSION):
    """Raw frame bytes. Any type/version is allowed (tests craft bad frames)."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload of {len(payload)} bytes > {MAX_PAYLOAD}")
    fields_ = struct.pack("<BBHH", version, int(msg_type), seq % SEQ_MODULO,
                          len(payload))
    head = SYNC + fields_ + bytes([crc8(fields_)])
    body = head[2:] + bytes(payload)
    return head[:2] + body + CRC.pack(crc16_ccitt(body))


def encode(message, seq):
    """Frame bytes of a message object."""
    return encode_frame(message.TYPE, seq, message.payload())


def decode_message(frame):
    """Message object of a frame; raises ProtocolError (with a NackReason)."""
    cls = MESSAGES.get(frame.type)
    if cls is None:
        raise ProtocolError(NackReason.UNKNOWN_TYPE,
                            f"unknown message type 0x{frame.type:02X}")
    return cls.from_payload(frame.payload)


class FrameDecoder:
    """Byte-stream -> frames. Never blocks; holds at most MAX_FRAME bytes
    between calls (plus what one feed() call adds).

    Rule (the firmware parser does exactly the same):
      - skip bytes until SYNC;
      - header CRC-8 mismatch, wrong version or length > MAX_PAYLOAD:
        framing error (checked in that order, as soon as the header is in);
      - CRC-16 mismatch: crc error;
      after an error, drop ONE byte (the first sync byte) and search again,
      so a valid frame that started inside a corrupted one is still found.
    Bytes skipped while searching are counted in `discarded`, not as errors.
    """

    def __init__(self):
        self.buffer = bytearray()
        self.frames = 0
        self.crc_errors = 0
        self.framing_errors = 0
        self.discarded = 0

    def feed(self, data):
        """Add received bytes; return the list of complete valid Frames."""
        self.buffer += data
        found = []
        while True:
            start = self.buffer.find(SYNC)
            if start < 0:
                # keep a trailing first sync byte: its partner may follow
                keep = 1 if self.buffer[-1:] == SYNC[:1] else 0
                self.discarded += len(self.buffer) - keep
                del self.buffer[:len(self.buffer) - keep]
                return found
            self.discarded += start
            del self.buffer[:start]
            if len(self.buffer) < HEADER_SIZE:
                return found
            _, version, msg_type, seq, length, hcrc = HEADER.unpack_from(
                self.buffer)
            if (hcrc != crc8(self.buffer[2:HEADER_SIZE - 1])
                    or version != VERSION or length > MAX_PAYLOAD):
                self.framing_errors += 1
                self._drop_one()
                continue
            total = HEADER_SIZE + length + CRC_SIZE
            if len(self.buffer) < total:
                return found
            body = bytes(self.buffer[2:HEADER_SIZE + length])
            (crc,) = CRC.unpack_from(self.buffer, HEADER_SIZE + length)
            if crc != crc16_ccitt(body):
                self.crc_errors += 1
                self._drop_one()
                continue
            found.append(Frame(msg_type, seq, body[HEADER_SIZE - 2:], version))
            self.frames += 1
            del self.buffer[:total]

    def _drop_one(self):
        del self.buffer[:1]
        self.discarded += 1


# ---------------------------------------------------------------------------
# 5. Golden vectors for the firmware tests
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Vector:
    """A byte stream and what a conforming decoder must report for it."""
    name: str
    data: bytes
    frames: tuple                   # expected Frames, in order
    crc_errors: int = 0
    framing_errors: int = 0


SAMPLE_TELEMETRY = Telemetry(
    state=State.ARMED, faults=Fault.LIDAR_NO_DATA, boot_id=0x12345678,
    uptime_ms=654321, rx_frames=1000, crc_errors=3, framing_errors=1,
    last_seq=41, heartbeat_age_ms=42, move_seq=40, move_progress_pct=100,
    cmd_forward_mm_s=-150, cmd_turn_cdeg_s=4500, outputs_enabled=0)
SAMPLE_MOVE = Move(forward_mm=200, turn_cdeg=-4500, speed_mm_s=150,
                   turn_rate_cdeg_s=9000, timeout_ms=2000, valid_until_ms=0x89ABCDEF)


def golden_vectors():
    """The shared conformance cases (also run by the C++ host tests)."""
    def frame(message, seq):
        return Frame(message.TYPE, seq, message.payload())

    def raw(f):
        return encode_frame(f.type, f.seq, f.payload)

    heartbeat = frame(Heartbeat(echo_uptime_ms=654000), 1)
    arm = frame(Arm(), 2)
    move = frame(SAMPLE_MOVE, 3)
    stop_wrap = frame(Stop(), 0xFFFF)
    telemetry = frame(SAMPLE_TELEMETRY, 7)
    # 0x5AA5 = 23205 cdeg: the sync pair appears inside the payload.
    chunk = frame(ScanChunk(9, 0, 1, ((0x5AA5, 0xA55A, 200), (100, 1500, 0),
                                      (35999, 0, 255))), 8)
    largest = frame(ScanChunk(10, 0, 1, tuple(
        (i * 100, 1000 + i, i % 256) for i in range(MAX_POINTS_PER_CHUNK))), 9)
    nack = frame(Nack(3, MsgType.MOVE, NackReason.OUT_OF_BOUNDS), 4)
    hello = frame(Hello(session_id=0xCAFEF00D), 5)
    resend = frame(ScanResend(9, 3), 6)

    corrupt = bytearray(raw(move))
    corrupt[HEADER_SIZE + 1] ^= 0x01            # one payload bit flipped
    # Valid checksums, so only the version / the length is wrong.
    bad_version = encode_frame(MsgType.HEARTBEAT, 1, bytes(4), version=1)  # a v1 peer
    oversize_fields = struct.pack("<BBHH", VERSION, MsgType.MOVE, 5,
                                  MAX_PAYLOAD + 1)
    oversize = SYNC + oversize_fields + bytes([crc8(oversize_fields)])
    bad_length = bytearray(raw(telemetry))
    bad_length[7] ^= 0x01                        # length 31 -> 287, hcrc stale
    truncated = raw(move)[:HEADER_SIZE + 5]      # cut inside the payload
    return (
        Vector("heartbeat", raw(heartbeat), (heartbeat,)),
        Vector("arm", raw(arm), (arm,)),
        Vector("move", raw(move), (move,)),
        Vector("stop_seq_wrap", raw(stop_wrap), (stop_wrap,)),
        Vector("telemetry", raw(telemetry), (telemetry,)),
        Vector("nack", raw(nack), (nack,)),
        Vector("hello", raw(hello), (hello,)),
        Vector("scan_resend", raw(resend), (resend,)),
        Vector("scan_chunk_sync_in_payload", raw(chunk), (chunk,)),
        Vector("max_payload", raw(largest), (largest,)),
        Vector("noise_then_two_frames",
               b"\x00\xff\xa5\x13\x5a" + raw(arm) + raw(heartbeat),
               (arm, heartbeat)),
        Vector("bad_crc_then_valid", bytes(corrupt) + raw(heartbeat),
               (heartbeat,), crc_errors=1),
        Vector("bad_version_then_valid", bytes(bad_version) + raw(arm),
               (arm,), framing_errors=1),
        Vector("oversize_length_then_valid", oversize + raw(heartbeat),
               (heartbeat,), framing_errors=1),
        Vector("corrupt_length_then_valid", bytes(bad_length) + raw(heartbeat),
               (heartbeat,), framing_errors=1),
        Vector("truncated_then_valid", truncated + raw(heartbeat) + raw(arm),
               (heartbeat, arm), crc_errors=1),
        Vector("trailing_sync_byte", raw(arm) + b"\xa5", (arm,)),
    )


def _c_bytes(data):
    rows = [", ".join(f"0x{b:02X}" for b in data[i:i + 16])
            for i in range(0, len(data), 16)]
    return "{\n    " + ",\n    ".join(rows) + "\n}"


def render_c_header(vectors=None):
    """C++ header with the protocol constants and golden vectors."""
    vectors = golden_vectors() if vectors is None else vectors
    lines = [
        "// GENERATED by `python -m autonomy.protocol --write-c-header` from",
        "// autonomy/protocol.py. Do not edit: test_autonomy_protocol checks it.",
        "#pragma once",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "",
        "namespace pv {",
        f"constexpr uint8_t kSync0 = 0x{SYNC[0]:02X};",
        f"constexpr uint8_t kSync1 = 0x{SYNC[1]:02X};",
        f"constexpr uint8_t kVersion = {VERSION};",
        f"constexpr uint16_t kMaxPayload = {MAX_PAYLOAD};",
        f"constexpr uint16_t kCrc16Check = 0x{CRC16_CHECK:04X};"
        "  // crc16(\"123456789\")",
        f"constexpr uint8_t kCrc8Check = 0x{CRC8_CHECK:02X};"
        "  // crc8(\"123456789\")",
        f"constexpr size_t kHeaderSize = {HEADER_SIZE};",
        f"constexpr int16_t kMaxForwardMm = {MAX_FORWARD_MM};",
        f"constexpr int16_t kMaxTurnCdeg = {MAX_TURN_CDEG};",
        f"constexpr uint16_t kMaxSpeedMmS = {MAX_SPEED_MM_S};",
        f"constexpr uint16_t kMaxTurnRateCdegS = {MAX_TURN_RATE_CDEG_S};",
        f"constexpr uint16_t kMaxTimeoutMs = {MAX_TIMEOUT_MS};",
        f"constexpr uint16_t kHeartbeatTimeoutMs = {HEARTBEAT_TIMEOUT_MS};",
        f"constexpr uint16_t kHeartbeatEchoMaxMs = {HEARTBEAT_ECHO_MAX_MS};",
        f"constexpr uint16_t kCommandTtlMs = {COMMAND_TTL_MS};",
        f"constexpr uint16_t kSessionReleaseMs = {SESSION_RELEASE_MS};",
        f"constexpr size_t kHelloSize = {Hello.LAYOUT.size};",
        f"constexpr size_t kHeartbeatSize = {Heartbeat.LAYOUT.size};",
        f"constexpr size_t kScanResendSize = {ScanResend.LAYOUT.size};",
        f"constexpr uint16_t kMaxPointsPerChunk = {MAX_POINTS_PER_CHUNK};",
        f"constexpr size_t kTelemetrySize = {Telemetry.LAYOUT.size};",
        f"constexpr size_t kHelloReplySize = {HelloReply.LAYOUT.size};",
        f"constexpr size_t kMoveSize = {Move.LAYOUT.size};",
    ]
    for enum in (MsgType, State, NackReason, MoveOutcome):
        for member in enum:
            lines.append(f"constexpr uint8_t k{enum.__name__}_{member.name} = "
                         f"{int(member)};")
    for member in Fault:
        if member.name != "NONE":
            lines.append(f"constexpr uint16_t kFault_{member.name} = "
                         f"0x{int(member):04X};")
    lines += ["",
              "struct ExpectedFrame { uint8_t type; uint16_t seq; "
              "uint16_t length; const uint8_t* payload; };",
              "struct Vector { const char* name; const uint8_t* data; "
              "size_t size; const ExpectedFrame* frames; size_t frame_count; "
              "uint32_t crc_errors; uint32_t framing_errors; };", ""]
    table = []
    for v_index, vector in enumerate(vectors):
        frame_rows = []
        for f_index, f in enumerate(vector.frames):
            name = "nullptr"
            if f.payload:
                name = f"v{v_index}_f{f_index}"
                lines.append(f"static const uint8_t {name}[] = "
                             f"{_c_bytes(f.payload)};")
            frame_rows.append(f"{{0x{f.type:02X}, {f.seq}, "
                              f"{len(f.payload)}, {name}}}")
        lines.append(f"static const uint8_t v{v_index}_data[] = "
                     f"{_c_bytes(vector.data)};")
        frames = "nullptr"
        if frame_rows:
            frames = f"v{v_index}_frames"
            lines.append(f"static const ExpectedFrame {frames}[] = {{"
                         + ", ".join(frame_rows) + "};")
        table.append(f'    {{"{vector.name}", v{v_index}_data, '
                     f"sizeof(v{v_index}_data), {frames}, {len(frame_rows)}, "
                     f"{vector.crc_errors}, {vector.framing_errors}}},")
    lines += ["", "static const Vector kVectors[] = {", *table, "};",
              f"constexpr size_t kVectorCount = {len(vectors)};",
              "}  // namespace pv", ""]
    return "\n".join(lines)


def write_c_header(path):
    Path(path).write_text(render_c_header(), encoding="ascii", newline="\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--write-c-header", metavar="PATH",
                        help="write the firmware golden-vector header")
    args = parser.parse_args(argv)
    if args.write_c_header:
        write_c_header(args.write_c_header)
        print(f"wrote {args.write_c_header}")
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
