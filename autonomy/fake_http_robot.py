"""A fake ESP32SlamRobot on localhost: the firmware's HTTP rules, no hardware.

It reproduces the rules of firmware/ESP32SlamRobot/ESP32SlamRobot.ino that
HttpRobotLink depends on, in real time:
  - boots DISARMED; /arm arms and clears the fault (the command seq is kept;
    it restarts only at boot);
  - /stop stops AND disarms;
  - /drive is refused (in this order) STALE_SEQ, DISARMED, NO_LIDAR; else it
    starts a lease of at most MAX_MOVE_MS that ends by itself;
  - a move also ends when no scan was published for LIDAR_STALE_MS;
  - scans are published every SCAN_PERIOD_MS; "still" means no move during the
    scan and none for SETTLE_MS before it.

Test hooks: restart() (new boot_id), lidar_ok, stuck (ignores lease ends and
/stop: a motor board that holds its PWM), silent (every request is dropped
without a reply), and `drives`, the /drive requests received.

    robot = FakeHttpRobot().start()   # robot.url is http://127.0.0.1:<port>
    ...
    robot.close()
"""
import json
import math
import random
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

MAX_MOVE_MS = 1000
SETTLE_MS = 300
LIDAR_STALE_MS = 500
SCAN_PERIOD_MS = 100
SCAN_POINTS = 450


def sample_points(count=SCAN_POINTS):
    """Deterministic CCW scan points (angle_cdeg, distance_mm, intensity)."""
    return tuple((i * 36000 // count, 1000 + i, 200) for i in range(count))


# Simulator room (metres): a 4 x 3 m box with a 0.5 m crate, robot at the origin.
ROOM_WALLS = (((-2.0, -1.5), (2.0, -1.5)), ((2.0, -1.5), (2.0, 1.5)), ((2.0, 1.5), (-2.0, 1.5)),
              ((-2.0, 1.5), (-2.0, -1.5)), ((0.8, 0.3), (1.3, 0.3)), ((1.3, 0.3), (1.3, 0.8)),
              ((1.3, 0.8), (0.8, 0.8)), ((0.8, 0.8), (0.8, 0.3)))
# Open-loop plant: speed at 100 % of MAX_PWM. Placeholders, not robot values.
SIM_FULL_SPEED_M_S = 0.30
SIM_FULL_TURN_RAD_S = math.radians(120.0)
SIM_MAX_RANGE_M = 12.0


MAX_PWM = 80  # the firmware's MAX_PWM (100 % in /drive)


def mix_wheels(vx, vy, wz):
    """Signed PWM per wheel (FL, FR, RL, RR), as the firmware's mixWheels()."""
    mix = (vx - vy - wz, vx + vy + wz, vx + vy - wz, vx - vy + wz)
    largest = max(100, *(abs(value) for value in mix))
    return [int(value * MAX_PWM / largest) for value in mix]


def _distance_to_segment(x, y, wall):
    (x1, y1), (x2, y2) = wall
    ex, ey = x2 - x1, y2 - y1
    t = max(0.0, min(1.0, ((x - x1) * ex + (y - y1) * ey) / (ex * ex + ey * ey)))
    return math.hypot(x - (x1 + t * ex), y - (y1 + t * ey))


def ray_scan(pose, walls=ROOM_WALLS, count=SCAN_POINTS):
    """CCW scan points of `walls` seen from pose (x, y, theta_rad)."""
    x, y, theta = pose
    points = []
    for i in range(count):
        angle_cdeg = i * 36000 // count
        a = theta + math.radians(angle_cdeg / 100.0)
        dx, dy = math.cos(a), math.sin(a)
        best = SIM_MAX_RANGE_M
        for (x1, y1), (x2, y2) in walls:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - x) * ey - (y1 - y) * ex) / den      # along the ray
            u = ((x1 - x) * dy - (y1 - y) * dx) / den      # along the wall
            if 0.0 < t < best and 0.0 <= u <= 1.0:
                best = t
        distance = 0 if best >= SIM_MAX_RANGE_M else int(round(best * 1000.0))
        points.append((angle_cdeg, distance, 200 if distance else 0))
    return tuple(points)


class FakeHttpRobot:
    """room=True: the robot moves in ROOM_WALLS and scans it (simulator mode);
    otherwise every scan is `points` (default sample_points(); tests)."""

    def __init__(self, points=None, room=False, locked=False):
        self.lock = threading.Lock()
        self.room = room
        self.locked = locked        # the diagnostic build: /arm answers 423 MOTORS_LOCKED
        self.last_reject = "none"   # as the firmware's /status diagnostics
        self.rejects = 0
        self.pose = (0.0, 0.0, 0.0)  # x m, y m, theta rad (room mode only)
        self.points = sample_points() if points is None else tuple(points)
        if room:
            self.points = ray_scan(self.pose)
        self.lidar_ok = True
        self.stuck = False
        self.silent = False
        self.drives = []            # (vx, vy, wz, time_ms, seq) of every /drive received
        self.boot_ms = self._now()
        self.restart()
        self._server = None
        self._threads = []
        self._closing = threading.Event()

    # -- lifecycle --------------------------------------------------------
    def start(self):
        robot = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                robot._handle(self)

            def log_message(self, *args):
                pass

        self._server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self._server.daemon_threads = True
        self.url = f"http://127.0.0.1:{self._server.server_address[1]}"
        self._threads = [threading.Thread(target=self._server.serve_forever, daemon=True),
                         threading.Thread(target=self._lidar_loop, daemon=True)]
        for thread in self._threads:
            thread.start()
        return self

    def close(self):
        self._closing.set()
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
        for thread in self._threads:
            thread.join(timeout=2)

    def restart(self):
        """Power cycle: everything volatile is lost; DISARMED."""
        with self.lock:
            self.boot_id = random.getrandbits(32)
            self.boot_ms = self._now()
            self.armed = False
            self.fault = False
            self.move_active = False
            self.wheel = (0, 0, 0)
            self.move_start = self.lease = 0
            self.last_motion_end = -10_000
            self.last_seq = 0
            self.scan_seq = 0
            self.scan_end = -10_000
            self.scan_still = False

    # -- model --------------------------------------------------------------
    @staticmethod
    def _now():
        return int(time.monotonic() * 1000)

    def _end_move(self, now):
        if self.move_active:
            self.last_motion_end = now
        self.move_active = False
        self.wheel = (0, 0, 0)

    def _motor_step(self, now):
        """The firmware's motorStep(): a lease or a stale LiDAR ends a move."""
        if self.stuck:
            return
        if self.move_active and (now - self.move_start >= self.lease
                                 or now - self.scan_end >= LIDAR_STALE_MS):
            self._end_move(now)
        if not self.armed or self.fault:
            self._end_move(now)

    def _integrate(self, dt_s):
        """Room mode: move the pose by the wheel command; never into a wall."""
        vx, vy, wz = (value / 100.0 for value in self.wheel)
        x, y, theta = self.pose
        forward, left = vx * SIM_FULL_SPEED_M_S * dt_s, vy * SIM_FULL_SPEED_M_S * dt_s
        nx = x + forward * math.cos(theta) - left * math.sin(theta)
        ny = y + forward * math.sin(theta) + left * math.cos(theta)
        theta = (theta + wz * SIM_FULL_TURN_RAD_S * dt_s + math.pi) % (2 * math.pi) - math.pi
        if min(_distance_to_segment(nx, ny, wall) for wall in ROOM_WALLS) < 0.15:
            nx, ny = x, y           # the simulated bumper: no translation into a wall
        self.pose = (nx, ny, theta)

    def _lidar_loop(self):
        while not self._closing.wait(SCAN_PERIOD_MS / 1000.0):
            with self.lock:
                now = self._now()
                if self.room:
                    if self.move_active:
                        self._integrate(SCAN_PERIOD_MS / 1000.0)
                    self.points = ray_scan(self.pose)
                if self.lidar_ok:
                    started = now - len(self.points) * 2 // 9
                    self.scan_seq += 1
                    self.scan_end = now
                    self.scan_still = (not self.move_active
                                       and started - self.last_motion_end >= SETTLE_MS)
                self._motor_step(now)

    def _status(self, now):
        remaining = max(0, self.lease - (now - self.move_start)) if self.move_active else 0
        return {"boot_id": self.boot_id, "uptime_ms": now - self.boot_ms, "reset_reason": 1,
                "armed": int(self.armed), "fault": int(self.fault),
                "moving": int(self.move_active), "lease_ms": remaining,
                "cmd_seq": self.last_seq, "scan_seq": self.scan_seq,
                "scan_age_ms": now - self.scan_end, "scan_points": len(self.points),
                "lidar_dps": 3600, "crc_fails": 0, "oversize_scans": 0, "i2c_nacks": 0,
                "guard_on": 0, "guard_trips": 0, "sector_mm": [65535] * 8, "clients": 1,
                "free_heap": 200000, "motors_locked": int(self.locked), "motor_ack": 1,
                "i2c_sda_idle_high": 1, "i2c_ack_0x30": 1, "i2c_ack_0x31": 0,
                "pwm": mix_wheels(*self.wheel), "last_stop": "none",
                "rejects": self.rejects, "last_reject": self.last_reject}

    def _reject(self, code, reason):
        self.rejects += 1
        self.last_reject = reason
        return code, reason

    def _drive(self, vx, vy, wz, time_ms, seq, now):
        if seq != 0 and seq <= self.last_seq:
            return self._reject(409, "STALE_SEQ")
        if not self.armed or self.fault:
            return self._reject(423, "DISARMED")
        if now - self.scan_end >= LIDAR_STALE_MS or self.scan_seq == 0:
            return self._reject(503, "NO_LIDAR")
        if seq != 0:
            self.last_seq = seq
        if time_ms <= 0:
            self._end_move(now)
        else:
            self.lease = max(1, min(time_ms, MAX_MOVE_MS))
            self.move_start = now
            self.wheel = (vx, vy, wz)
            self.move_active = True
        return 200, "OK"

    def _scan2(self):
        if self.scan_seq == 0:
            return None
        body = struct.pack("<hhHH", 0, 0, 0, len(self.points)) + b"".join(
            struct.pack("<HHB", *point) for point in self.points)
        header = struct.pack("<2sBBIIHH", b"S2", 1, int(self.scan_still), self.scan_seq,
                             self.scan_end - self.boot_ms, 3600, 0)
        return header + body

    # -- HTTP -----------------------------------------------------------------
    def _handle(self, request):
        if self.silent:
            request.close_connection = True
            return                  # no reply at all: the client sees a dropped link
        url = urlparse(request.path)
        args = {key: values[0] for key, values in parse_qs(url.query).items()}

        def arg(name, default=0):
            try:
                return int(args.get(name, default))
            except ValueError:
                return default

        with self.lock:
            now = self._now()
            self._motor_step(now)
            kind, code, body = "text/plain", 200, "OK"
            if url.path == "/status":
                kind, body = "application/json", json.dumps(self._status(now))
            elif url.path == "/arm" and self.locked:
                code, body = self._reject(423, "MOTORS_LOCKED")
            elif url.path == "/arm":
                self._end_move(now)
                self.fault = False
                self.armed = True       # last_seq is kept: it restarts only at boot
                body = "ARMED"
            elif url.path == "/stop":
                if not self.stuck:
                    self.armed = False
                    self._end_move(now)
                body = "STOPPED"
            elif url.path == "/drive":
                clamp = lambda v: max(-100, min(100, v))  # noqa: E731
                vx, vy, wz = clamp(arg("vx")), clamp(arg("vy")), clamp(arg("wz"))
                seq = max(arg("seq"), 0)
                self.drives.append((vx, vy, wz, arg("time"), seq))
                code, body = self._drive(vx, vy, wz, arg("time"), seq, now)
            elif url.path == "/scan2":
                data = self._scan2()
                if data is None:
                    code, body = 503, "NO_VALID_SCAN"
                else:
                    kind, body = "application/octet-stream", data
            elif url.path == "/ping":
                body = "OK"
            else:
                code, body = 404, "NOT_FOUND"
        payload = body if isinstance(body, bytes) else body.encode("ascii")
        request.send_response(code)
        request.send_header("Content-Type", kind)
        request.send_header("Content-Length", str(len(payload)))
        request.end_headers()
        request.wfile.write(payload)
