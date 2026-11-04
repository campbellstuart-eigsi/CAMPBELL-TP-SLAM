"""Manual driving core of the robot controller: no Tk, all network off the GUI thread.

The GUI (autonomy/controller_ui.py) only calls the non-blocking methods below
and reads snapshot() from a Tk timer. Two worker threads own the network:

  control   sends /drive leases of LEASE_MS (300 ms), renewed every RENEW_MS
            while a movement key is held; runs connect/arm/disarm/disconnect
  telemetry polls /status and /scan2 every 250 ms

Safety rules (on top of the firmware's own lease expiry):
  - movement intent is the set of held keys, read at send time: there is no
    command queue, so an old movement can never be sent later;
  - releasing every key sends a zero /drive at once; release_all() (focus
    loss, disconnect, any failure) does the same;
  - STOP runs on its own thread immediately, disarms the robot and latches:
    nothing moves again until ARM succeeds;
  - a failed /drive, a robot that reports itself disarmed or restarted, or
    LINK_SILENCE_MS without /status latch the stop and clear the intent; the
    robot then stops on its own when its lease runs out;
  - manual commands are raw PWM percent, capped at MAX_MANUAL_PCT. They need
    no distance calibration; HttpRobotLink.move() (distance moves) still does;
  - in REAL ROBOT mode ARM is refused until the bench safety checks are
    confirmed for the current connection.
"""
import threading
import time
from dataclasses import dataclass, field

from autonomy.http_robot_link import HttpRobotLink, LinkError

LEASE_MS = 300
RENEW_MS = 100
TELEMETRY_PERIOD_MS = 250
LINK_SILENCE_MS = 1000
HTTP_TIMEOUT_S = 0.4
MAX_MANUAL_PCT = 50
DEFAULT_DRIVE_PCT = 30
DEFAULT_TURN_PCT = 30
MAX_ERRORS = 8

# action -> (vx, vy, wz) direction; signs as the firmware's mixWheels():
# +vx forward, +vy left, +wz counter-clockwise.
ACTIONS = {
    "forward": (1, 0, 0), "backward": (-1, 0, 0),
    "left": (0, 1, 0), "right": (0, -1, 0),
    "rotate_left": (0, 0, 1), "rotate_right": (0, 0, -1),
}
ACTION_LABELS = {
    "forward": "Forward", "backward": "Backward", "left": "Strafe left",
    "right": "Strafe right", "rotate_left": "Rotate left", "rotate_right": "Rotate right",
    "stop": "STOP",
}
# Tk keysyms (lower case). AZERTY is the default layout.
KEY_LAYOUTS = {
    "AZERTY": {"forward": "z", "backward": "s", "left": "q", "right": "d",
               "rotate_left": "a", "rotate_right": "e", "stop": "space"},
    "QWERTY": {"forward": "w", "backward": "s", "left": "a", "right": "d",
               "rotate_left": "q", "rotate_right": "e", "stop": "space"},
}
DEFAULT_LAYOUT = "AZERTY"


@dataclass
class Snapshot:
    """What the GUI shows; a copy, safe to read on the Tk thread."""
    link: str = "disconnected"       # disconnected / connecting / ok / lost
    url: str = ""
    real_robot: bool = False
    armed: bool = False              # ARM succeeded and nothing has latched a stop since
    stop_latched: bool = True
    safety_ack: bool = False
    busy: str = ""                   # the action in progress, if any
    status: dict = None              # the latest /status
    status_age_ms: int = None
    scan_seq: int = 0
    scan_points: tuple = ()
    command: tuple = (0, 0, 0)       # the movement intent now (vx, vy, wz percent)
    held: tuple = ()
    drive_pct: int = DEFAULT_DRIVE_PCT
    turn_pct: int = DEFAULT_TURN_PCT
    calibrated: bool = False
    drives_sent: int = 0
    last_reply: str = ""
    errors: list = field(default_factory=list)


class Teleop:
    def __init__(self, link_factory=HttpRobotLink, lease_ms=LEASE_MS, renew_ms=RENEW_MS,
                 http_timeout_s=HTTP_TIMEOUT_S):
        self.link_factory = link_factory
        self.lease_ms = lease_ms
        self.renew_ms = renew_ms
        self.http_timeout_s = http_timeout_s
        self.lock = threading.RLock()
        self.link = None
        self.url = ""
        self.link_state = "disconnected"
        self.real_robot = False
        self.safety_ack = False
        self.armed = False
        self.stop_latched = True
        self.held = set()
        self.drive_pct = DEFAULT_DRIVE_PCT
        self.turn_pct = DEFAULT_TURN_PCT
        self.status = None
        self.status_ms = None
        self.boot_id = None
        self.scan_seq = 0
        self.scan_points = ()
        self.drives_sent = 0
        self.last_reply = ""
        self.errors = []
        self.busy = ""
        self.frozen = False          # tests: the control thread stops renewing (a hung GUI)
        self._action = None
        self._sent_motion = False
        self._wake = threading.Event()
        self._closing = threading.Event()
        self._threads = [threading.Thread(target=self._control_loop, daemon=True),
                         threading.Thread(target=self._telemetry_loop, daemon=True)]
        for thread in self._threads:
            thread.start()

    # -- GUI side: never blocks ------------------------------------------------
    def press(self, action):
        if action not in ACTIONS:
            return
        with self.lock:
            if action in self.held:
                return               # key auto-repeat
            self.held.add(action)
        self._wake.set()

    def release(self, action):
        with self.lock:
            self.held.discard(action)
        self._wake.set()

    def release_all(self):
        """Focus loss, disconnect, failures: no movement intent at all."""
        with self.lock:
            self.held.clear()
        self._wake.set()

    def set_speeds(self, drive_pct, turn_pct):
        with self.lock:
            self.drive_pct = max(0, min(MAX_MANUAL_PCT, int(drive_pct)))
            self.turn_pct = max(0, min(MAX_MANUAL_PCT, int(turn_pct)))
        self._wake.set()

    def set_mode(self, real_robot):
        with self.lock:
            if self.link is not None or self.busy:
                return False         # the mode changes only while disconnected
            self.real_robot = bool(real_robot)
            self.safety_ack = False
            return True

    def set_safety_ack(self, value):
        with self.lock:
            self.safety_ack = bool(value)

    def stop(self):
        """STOP: latch, clear intent and send /stop now, on its own thread."""
        with self.lock:
            self._latch_locked(None)
            link = self.link
        self._wake.set()
        if link is not None:
            threading.Thread(target=self._send_stop, args=(link,), daemon=True).start()

    def request_connect(self, url):
        return self._request("connect", url)

    def request_disconnect(self):
        self.release_all()
        return self._request("disconnect")

    def request_arm(self):
        return self._request("arm")

    def request_disarm(self):
        self.release_all()
        return self._request("disarm")

    def command(self):
        with self.lock:
            return self._command_locked()

    def snapshot(self):
        with self.lock:
            age = None if self.status_ms is None else self._now() - self.status_ms
            calibrated = bool(self.link and self.link.calibration.measured)
            return Snapshot(
                link=self.link_state, url=self.url, real_robot=self.real_robot,
                armed=self.armed, stop_latched=self.stop_latched, safety_ack=self.safety_ack,
                busy=self.busy, status=dict(self.status) if self.status else None,
                status_age_ms=age, scan_seq=self.scan_seq, scan_points=self.scan_points,
                command=self._command_locked(), held=tuple(sorted(self.held)),
                drive_pct=self.drive_pct, turn_pct=self.turn_pct, calibrated=calibrated,
                drives_sent=self.drives_sent, last_reply=self.last_reply,
                errors=list(self.errors))

    def close(self):
        """Window closing: stop the robot, then end the worker threads."""
        with self.lock:
            self._latch_locked(None)
            link = self.link
        if link is not None:
            self._send_stop(link)
        self._closing.set()
        self._wake.set()
        for thread in self._threads:
            thread.join(timeout=2.0)

    # -- internals -----------------------------------------------------------
    @staticmethod
    def _now():
        return int(time.monotonic() * 1000)

    def _error(self, text):
        with self.lock:
            if not self.errors or self.errors[-1][1] != text:
                self.errors.append((time.strftime("%H:%M:%S"), text))
                del self.errors[:-MAX_ERRORS]

    def _latch_locked(self, reason):
        """Stop intent: clear the keys; no movement until ARM succeeds again."""
        self.held.clear()
        self.stop_latched = True
        self.armed = False
        self._sent_motion = False    # a /stop, not a zero /drive, ends the motion
        if reason:
            self._error(reason)

    def _command_locked(self):
        if self.stop_latched or not self.armed or self.link_state != "ok":
            return (0, 0, 0)
        vx = vy = wz = 0
        for action in self.held:
            dx, dy, dw = ACTIONS[action]
            vx, vy, wz = vx + dx, vy + dy, wz + dw
        return (vx * self.drive_pct, vy * self.drive_pct, wz * self.turn_pct)

    def _request(self, name, arg=None):
        with self.lock:
            if self._action is not None or self.busy:
                self._error(f"busy ({self.busy or self._action[0]}): {name} ignored")
                return False
            self._action = (name, arg)
            self.busy = name
        self._wake.set()
        return True

    def _send_stop(self, link):
        result = link.stop()
        with self.lock:
            self.last_reply = "STOPPED" if result.ok else f"STOP unanswered ({result.reason})"
            if not result.ok:
                self._error("STOP not acknowledged: the robot stops when its lease (300 ms) ends")

    def _control_loop(self):
        while not self._closing.is_set():
            self._wake.wait(self.renew_ms / 1000.0)
            self._wake.clear()
            if self._closing.is_set():
                return
            with self.lock:
                action, self._action = self._action, None
            if action is not None:
                try:
                    getattr(self, f"_do_{action[0]}")(*([action[1]] if action[1] else []))
                finally:
                    with self.lock:
                        self.busy = ""
            if not self.frozen:
                self._drive_once()

    def _drive_once(self):
        with self.lock:
            link = self.link
            command = self._command_locked()
            moving = command != (0, 0, 0)
            if link is None or (not moving and not self._sent_motion):
                return
        lease = self.lease_ms if moving else 0
        try:
            code, text = link.drive(*command, lease)
        except OSError as error:
            with self.lock:
                self._sent_motion = False
                self.link_state = "lost"
                self._latch_locked(f"link lost on /drive ({error}); the robot stops "
                                   f"when its lease ends")
            return
        with self.lock:
            self.drives_sent += 1
            self.last_reply = text
            # A STOP that latched while this request was in flight has its own /stop.
            self._sent_motion = moving and code == 200 and not self.stop_latched
            if code == 200:
                return
            if code == 423:
                self._latch_locked("the robot is DISARMED: press ARM")
            else:
                self.held.clear()
                self._error(f"/drive refused: HTTP {code} {text}")

    def _telemetry_loop(self):
        while not self._closing.wait(TELEMETRY_PERIOD_MS / 1000.0):
            with self.lock:
                link = self.link
                state = self.link_state
            if link is None or state not in ("ok", "lost"):
                continue
            try:
                status = link.status()
                scan = link.latest_scan()
            except (OSError, ValueError, LinkError) as error:
                with self.lock:
                    silent = self.status_ms is None or self._now() - self.status_ms > LINK_SILENCE_MS
                    if silent and self.link_state == "ok":
                        self.link_state = "lost"
                        self._latch_locked(f"no answer from the robot for {LINK_SILENCE_MS} ms "
                                           f"({error})")
                        threading.Thread(target=self._send_stop, args=(link,),
                                         daemon=True).start()
                        self._wake.set()
                continue
            with self.lock:
                self.status, self.status_ms = status, self._now()
                if self.link_state == "lost":
                    self.link_state = "ok"   # answers again; still latched until ARM
                    self._error("the robot answers again: press ARM to drive")
                if self.boot_id is not None and status["boot_id"] != self.boot_id:
                    self._latch_locked("the ESP32 restarted (new boot_id): it is DISARMED")
                    link.boot_id = status["boot_id"]
                self.boot_id = status["boot_id"]
                if self.armed and not status["armed"]:
                    self._latch_locked("the robot disarmed itself (STOP, Wi-Fi drop or fault)")
                if scan is not None and scan[0] != self.scan_seq:
                    self.scan_seq, self.scan_points = scan[0], scan[3]
            self._wake.set()

    # -- actions (control thread) ---------------------------------------------
    def _do_connect(self, url):
        with self.lock:
            if self.link is not None:
                self._error("already connected")
                return
            self.link_state, self.url = "connecting", url
        link = self.link_factory(url, http_timeout_s=self.http_timeout_s)
        try:
            status = link.connect()
        except LinkError as error:
            with self.lock:
                self.link_state = "disconnected"
                self._error(str(error))
            return
        with self.lock:
            self.link, self.link_state = link, "ok"
            self.status, self.status_ms = status, self._now()
            self.boot_id = status["boot_id"]
            self.safety_ack = False  # confirmed again for every connection
            self._latch_locked(None)
            self.last_reply = "connected"

    def _do_disconnect(self):
        with self.lock:
            link = self.link
            self._latch_locked(None)
        if link is not None:
            self._send_stop(link)   # disarm before letting go
        with self.lock:
            self.link, self.link_state = None, "disconnected"
            self.status = self.status_ms = self.boot_id = None
            self.scan_seq, self.scan_points = 0, ()
            self._sent_motion = False
            self.safety_ack = False

    def _do_arm(self):
        with self.lock:
            link = self.link
            if link is None or self.link_state != "ok":
                self._error("ARM refused: not connected")
                return
            if self.real_robot and not self.safety_ack:
                self._error("ARM refused: confirm the bench safety checks first")
                return
        result = link.arm()
        with self.lock:
            if result.ok:
                self.held.clear()    # keys pressed before ARM never start a move
                self.armed, self.stop_latched = True, False
                self.last_reply = "ARMED"
            else:
                self._error(f"ARM failed: {result.reason}")

    def _do_disarm(self):
        with self.lock:
            link = self.link
            self._latch_locked(None)
        if link is not None:
            self._send_stop(link)
