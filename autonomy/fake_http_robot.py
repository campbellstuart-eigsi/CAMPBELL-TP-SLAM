"""A fake ESP32SlamRobot on localhost: the firmware's HTTP rules, no hardware.

It reproduces the rules of firmware/ESP32SlamRobot/ESP32SlamRobot.ino that
HttpRobotLink depends on, in real time:
  - boots DISARMED; /arm arms, clears the fault and restarts the command seq;
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


class FakeHttpRobot:
    def __init__(self, points=None):
        self.lock = threading.Lock()
        self.points = sample_points() if points is None else tuple(points)
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

    def _lidar_loop(self):
        while not self._closing.wait(SCAN_PERIOD_MS / 1000.0):
            with self.lock:
                now = self._now()
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
                "free_heap": 200000}

    def _drive(self, vx, vy, wz, time_ms, seq, now):
        if seq != 0 and seq <= self.last_seq:
            return 409, "STALE_SEQ"
        if not self.armed or self.fault:
            return 423, "DISARMED"
        if now - self.scan_end >= LIDAR_STALE_MS or self.scan_seq == 0:
            return 503, "NO_LIDAR"
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
            elif url.path == "/arm":
                self._end_move(now)
                self.fault = False
                self.armed = True
                self.last_seq = 0
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
