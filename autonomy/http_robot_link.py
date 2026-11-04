"""RobotLink over the ESP32SlamRobot HTTP firmware (approach B, MVP).

The firmware is firmware/ESP32SlamRobot/ESP32SlamRobot.ino: an ESP32 SoftAP
(default http://192.168.4.1) with HTTP GET routes. The routes used here:

    /status   JSON: boot_id, armed, fault, moving, lease_ms, cmd_seq, scan_seq, ...
    /arm      arms (and clears a fault); the robot's command seq restarts at 0
    /stop     stops the motors AND disarms
    /drive    ?vx=&vy=&wz=&time=&seq=  percent of MAX_PWM; a lease of at most
              LEASE_MAX_MS: the robot stops by itself when it runs out
    /scan2    16-byte header (S2, version, still flag, scan seq, end ms) +
              the course /scan bytes (pose, count, points)

Safety comes from the firmware: every motion is a lease of at most 1 s, so a
dead laptop, a lost Wi-Fi link or a crash of this program stops the robot
within LEASE_MAX_MS. This client adds only:
  - Phase B's per-MOVE limits (300 mm, 90 degrees), refused before any HTTP;
  - stages longer than one lease are split into leases of at most 1 s;
  - move() never returns while the robot may still be moving (on link loss it
    waits until the lease has certainly expired);
  - stop() works from any thread and interrupts a waiting move();
  - a changed boot_id (the ESP32 restarted) ends the move as "restarted";
  - move() is refused until a MEASURED Calibration is given (S2): the robot
    has no encoders, so distance and angle come only from time x speed.

Scan angles are counter-clockwise, 0.01 degree (the LD06 library's
DataPoint.angle), as in protocol SCAN_CHUNK: autonomy.protocol.
points_to_sensor_xy converts them. Pure standard library; no proxy is used.
"""
import json
import math
import struct
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass

from autonomy import protocol as p
from autonomy.robot_link import (DEFAULT_SPEED_M_S, DEFAULT_TURN_RATE_RAD_S, CommandResult,
                                 LinkError, MoveResult, RobotLink, ScanResult)

DEFAULT_URL = "http://192.168.4.1"
LEASE_MAX_MS = 1000          # the firmware's MAX_MOVE_MS
HTTP_TIMEOUT_S = 0.5
STOP_TIMEOUT_S = 0.3
STOP_ATTEMPTS = 3
SETTLE_POLL_MS = 500         # after a lease: how long the robot may take to report "not moving"
LEASE_MARGIN_MS = 200        # on link loss, wait this beyond the lease before returning
POLL_MS = 50
LIDAR_STALE_MS = 500         # the firmware's LIDAR_STALE_MS: a move is cut short beyond it
SCAN2_HEADER = struct.Struct("<2sBBIIHH")    # 'S2', version, flags, seq, end_ms, dps, reserved
LEGACY_HEADER = struct.Struct("<hhHH")       # pose x, pose y, pose z, point count
SCAN_POINT = struct.Struct("<HHB")
STATUS_KEYS = ("boot_id", "armed", "fault", "moving", "cmd_seq", "scan_seq")


@dataclass(frozen=True)
class Calibration:
    """Open-loop motion model at one fixed PWM percentage per motion.

    forward_pct / turn_pct - /drive percent used for straight and turning stages
    forward_m_s / turn_rad_s - speeds MEASURED at those percentages (S2)
    measured - False for the placeholder: move() refuses to run with it
    """
    forward_pct: int = 50
    forward_m_s: float = 0.10
    turn_pct: int = 50
    turn_rad_s: float = math.radians(45.0)
    measured: bool = False


PLACEHOLDER_CALIBRATION = Calibration()


def parse_scan2(data):
    """/scan2 bytes -> (seq, end_ms, still, points). Raises ValueError."""
    if len(data) < SCAN2_HEADER.size + LEGACY_HEADER.size:
        raise ValueError(f"scan2 of {len(data)} bytes is too short")
    magic, version, flags, seq, end_ms, _, _ = SCAN2_HEADER.unpack_from(data)
    if magic != b"S2" or version != 1:
        raise ValueError(f"not a version 1 scan2 header: {magic!r} v{version}")
    _, _, _, count = LEGACY_HEADER.unpack_from(data, SCAN2_HEADER.size)
    start = SCAN2_HEADER.size + LEGACY_HEADER.size
    if len(data) != start + count * SCAN_POINT.size:
        raise ValueError(f"scan2 says {count} points but holds {len(data) - start} bytes")
    points = tuple(SCAN_POINT.iter_unpack(data[start:]))
    return seq, end_ms, bool(flags & 1), points


class HttpRobotLink(RobotLink):
    """RobotLink for the ESP32SlamRobot firmware; see the module docstring."""

    def __init__(self, base_url=DEFAULT_URL, calibration=PLACEHOLDER_CALIBRATION,
                 http_timeout_s=HTTP_TIMEOUT_S):
        self.base_url = base_url.rstrip("/")
        self.calibration = calibration
        self.http_timeout_s = http_timeout_s
        self.boot_id = None
        self.seq = 0
        self.min_scan_seq = 0        # a scan must be newer than the last motion
        self.stop_event = threading.Event()
        self.drive_log = []          # (vx, vy, wz, time_ms, seq) of every /drive sent
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    # -- HTTP -------------------------------------------------------------
    def _get(self, path, timeout_s=None):
        """(HTTP status, body bytes). Raises OSError on no answer."""
        url = self.base_url + path
        try:
            with self._opener.open(url, timeout=timeout_s or self.http_timeout_s) as reply:
                return reply.status, reply.read()
        except urllib.error.HTTPError as error:
            return error.code, error.read()

    def status(self):
        code, body = self._get("/status")
        if code != 200:
            raise LinkError(f"/status answered HTTP {code}")
        status = json.loads(body)
        missing = [key for key in STATUS_KEYS if key not in status]
        if missing:
            raise LinkError(f"/status lacks {missing}: not the ESP32SlamRobot firmware")
        return status

    # -- RobotLink ----------------------------------------------------------
    def connect(self):
        """Reads /status; returns it. Raises LinkError if the robot is unreachable."""
        try:
            status = self.status()
        except (OSError, ValueError) as error:
            raise LinkError(f"no robot at {self.base_url}: {error}") from error
        self.boot_id = status["boot_id"]
        self.min_scan_seq = status["scan_seq"]
        return status

    def arm(self):
        try:
            code, body = self._get("/arm")
        except OSError as error:
            return CommandResult(False, f"timeout: {error}")
        if code == 200 and body.strip() == b"ARMED":
            self.seq = 0             # the firmware restarts its command seq on /arm
            self.stop_event.clear()
            return CommandResult(True)
        return CommandResult(False, f"HTTP {code} {body[:40]!r}")

    def stop(self):
        """Stops and disarms. Safe from any thread; interrupts a waiting move()."""
        self.stop_event.set()
        error = None
        for _ in range(STOP_ATTEMPTS):
            try:
                code, body = self._get("/stop", timeout_s=STOP_TIMEOUT_S)
                if code == 200:
                    return CommandResult(True)
                error = f"HTTP {code}"
            except OSError as exc:
                error = str(exc)
        # Unanswered: the robot still stops when its lease (<= 1 s) runs out.
        return CommandResult(False, f"timeout: {error}")

    def disarm(self):
        return self.stop()           # the firmware's /stop also disarms

    def clear_fault(self):
        return CommandResult(False, "unsupported: /arm clears the fault")

    def drive(self, vx, vy, wz, time_ms):
        """One raw lease (manual driving, bench tests). Percent, signs as the
        firmware's mixWheels (+vx forward, +vy left, +wz counter-clockwise).
        Returns (HTTP status, reply text); time_ms is capped at LEASE_MAX_MS."""
        vx, vy, wz = (max(-100, min(100, int(v))) for v in (vx, vy, wz))
        time_ms = max(0, min(LEASE_MAX_MS, int(time_ms)))
        self.seq += 1
        self.drive_log.append((vx, vy, wz, time_ms, self.seq))
        code, body = self._get(f"/drive?vx={vx}&vy={vy}&wz={wz}&time={time_ms}&seq={self.seq}")
        return code, body.decode("ascii", "replace").strip()

    def move(self, forward_m, turn_rad, speed_m_s=DEFAULT_SPEED_M_S,
             turn_rate_rad_s=DEFAULT_TURN_RATE_RAD_S):
        """Turn in place, then drive straight. speed_m_s and turn_rate_rad_s
        are ignored: the speed is the calibration's (open loop, no encoders)."""
        cal = self.calibration
        if not cal.measured:
            return MoveResult("refused", detail="uncalibrated: measure a Calibration at S2")
        if self.boot_id is None:
            return MoveResult("refused", detail="not connected")
        if (abs(forward_m) * 1000.0 > p.MAX_FORWARD_MM
                or abs(math.degrees(turn_rad)) * 100.0 > p.MAX_TURN_CDEG):
            return MoveResult("refused", detail="OUT_OF_BOUNDS")
        if not forward_m and not turn_rad:
            return MoveResult("completed")
        stages = []
        if turn_rad:
            sign = 1 if turn_rad > 0 else -1
            stages.append(((0, 0, sign * cal.turn_pct), abs(turn_rad) / cal.turn_rad_s * 1000.0))
        if forward_m:
            sign = 1 if forward_m > 0 else -1
            stages.append(((sign * cal.forward_pct, 0, 0), abs(forward_m) / cal.forward_m_s * 1000.0))
        for command, total_ms in stages:
            remaining = int(round(total_ms))
            while remaining > 0:
                lease = min(LEASE_MAX_MS, remaining)
                result = self._run_lease(command, lease)
                if result is not None:
                    return result
                remaining -= lease
        return MoveResult("completed", seq=self.seq)

    def _run_lease(self, command, lease_ms):
        """Send one lease and wait until the robot is still. None if it ended
        normally, else the MoveResult that ends the move."""
        if self.stop_event.is_set():
            return MoveResult("stopped", seq=self.seq)
        sent_at = time.monotonic()
        try:
            trips_before = self.status().get("guard_trips", 0)
            code, text = self.drive(*command, lease_ms)
        except (ValueError, LinkError) as error:
            return MoveResult("refused", seq=self.seq, detail=f"/status: {error}")
        except OSError as error:
            return self._link_lost(sent_at, lease_ms, f"/drive: {error}")
        if code != 200:
            return MoveResult("refused", seq=self.seq, detail=text)
        self.stop_event.wait(lease_ms / 1000.0)
        deadline = sent_at + (lease_ms + SETTLE_POLL_MS) / 1000.0
        while True:
            try:
                status = self.status()
            except (OSError, ValueError, LinkError) as error:
                return self._link_lost(sent_at, lease_ms, f"/status: {error}")
            if status["boot_id"] != self.boot_id:
                self.boot_id = status["boot_id"]
                return MoveResult("restarted", seq=self.seq, detail="the ESP32 restarted")
            if not status["moving"]:
                self.min_scan_seq = status["scan_seq"]
                if self.stop_event.is_set() or not status["armed"]:
                    return MoveResult("stopped", seq=self.seq)
                if status["fault"]:
                    return MoveResult("fault", seq=self.seq)
                # The firmware also ends a move early on a stale LiDAR or a stop
                # zone; the distance travelled is then unknown.
                if status.get("scan_age_ms", 0) >= LIDAR_STALE_MS:
                    return MoveResult("fault", seq=self.seq,
                                      detail="LiDAR stale: the move may have been cut short")
                if status.get("guard_trips", 0) != trips_before:
                    return MoveResult("stopped", seq=self.seq, detail="stop zone tripped")
                return None
            if time.monotonic() > deadline:
                self.stop()
                return MoveResult("fault", seq=self.seq,
                                  detail="still moving after its lease: stopped")
            time.sleep(POLL_MS / 1000.0)

    def _link_lost(self, sent_at, lease_ms, detail):
        """Never return while the robot may move: STOP, then wait out the lease."""
        self.stop()
        end = sent_at + (lease_ms + LEASE_MARGIN_MS) / 1000.0
        while time.monotonic() < end:
            time.sleep(min(POLL_MS / 1000.0, max(0.0, end - time.monotonic())))
        return MoveResult("link_lost", seq=self.seq, detail=detail)

    def request_scan(self, timeout_ms=2000):
        """The first still scan newer than the last motion (CCW angles)."""
        deadline = time.monotonic() + timeout_ms / 1000.0
        reason = "timeout"
        while True:
            try:
                code, body = self._get("/scan2")
                if code == 200:
                    seq, end_ms, still, points = parse_scan2(body)
                    if seq > self.min_scan_seq and still:
                        return ScanResult(True, points, timestamp_ms=end_ms)
                    reason = "timeout: no still scan newer than the last motion"
                else:
                    reason = f"HTTP {code} {body[:40]!r}"
            except (OSError, ValueError) as error:
                reason = str(error)
            if time.monotonic() >= deadline:
                return ScanResult(False, reason=reason)
            time.sleep(POLL_MS / 1000.0)

    def pump(self, duration_ms):
        """Nothing to keep alive (no heartbeat in this firmware): wait."""
        self.stop_event.wait(duration_ms / 1000.0)

    def close(self):
        self.stop()
