"""Manual ZQSD driving of the robot over Wi-Fi (docs/specs/workspace_drive.md).

Speaks only the professor's HTTP routes (/forward ... /rotateright ?time=&speed=,
/stop, /ping), so the same code drives an I2C or a PWM robot. No Tk here: the
Workspace Drive panel calls DriveController from the GUI thread, and all
networking runs on the controller's own worker thread.
"""
import threading
import time
import urllib.parse
import urllib.request

DEFAULT_URL = "http://192.168.4.1"
DEFAULT_SPEED = 80          # MecanumCar_v2 default PWM (0..255)
MOVE_TIME_MS = 300          # each command moves this long, then the firmware stops
RENEW_S = 0.15              # a held key renews its command this often
KEEPALIVE_S = 0.5           # no keepalive() for this long: the held move is dropped
PING_TIMEOUT_S = 1.0
MOVE_TIMEOUT_S = 0.3
STOP_TIMEOUT_S = 0.5

# AZERTY key -> (route, time factor). The firmware rotates for time / 2.
KEYS = {"z": ("forward", 1), "s": ("backward", 1), "q": ("left", 1), "d": ("right", 1),
        "a": ("rotateleft", 2), "e": ("rotateright", 2)}
ROUTE_NAMES = {"forward": "forward", "backward": "backward", "left": "strafe left",
               "right": "strafe right", "rotateleft": "rotate left",
               "rotateright": "rotate right"}


class DriveLink:
    """Blocking HTTP calls to the robot. Use only from a worker thread."""

    def __init__(self, base_url=DEFAULT_URL):
        self.base_url = base_url.rstrip("/")
        # No system proxy: 192.168.4.1 is the robot's own access point.
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def get(self, path, timeout_s, **params):
        url = self.base_url + path
        if params:
            url += "?" + urllib.parse.urlencode(params)
        with self.opener.open(url, timeout=timeout_s) as reply:
            return reply.read().decode("utf-8", "replace").strip()

    def ping(self):
        return self.get("/ping", PING_TIMEOUT_S)

    def move(self, route, time_ms, speed):
        return self.get("/" + route, MOVE_TIMEOUT_S, time=int(time_ms), speed=int(speed))

    def stop(self):
        return self.get("/stop", STOP_TIMEOUT_S)


class DriveController:
    """Hold-to-drive state shared by the GUI thread and one worker thread.

    The GUI sets what it wants (connect, hold, release, stop, speed) and calls
    keepalive() while the drive pad has focus; the worker turns that into HTTP
    requests. Any failure sends /stop and disconnects. The firmware also stops
    on its own MOVE_TIME_MS after the last command.
    """

    def __init__(self, link_factory=DriveLink):
        self.link_factory = link_factory
        self.lock = threading.Lock()
        self.wake = threading.Event()
        self.want_connected = False
        self.url = DEFAULT_URL
        self.speed = DEFAULT_SPEED
        self.held = None            # held key, e.g. "z"
        self.stop_pending = False
        self.last_keepalive = 0.0
        self.closed = False
        self.state = "disconnected"  # disconnected / connecting / connected / error
        self.text = "Not connected."
        self.requests = 0
        self.thread = None

    # -- GUI thread ---------------------------------------------------------------
    def connect(self, url):
        with self.lock:
            self.url, self.want_connected = url.strip() or DEFAULT_URL, True
            self.held, self.stop_pending = None, False
            self.state, self.text = "connecting", f"Connecting to {self.url} ..."
        if self.thread is None:
            self.thread = threading.Thread(target=self.run, name="robot-drive", daemon=True)
            self.thread.start()
        self.wake.set()

    def disconnect(self):
        with self.lock:
            self.want_connected, self.held = False, None
        self.wake.set()

    def hold(self, key):
        if key not in KEYS:
            return
        with self.lock:
            if not self.want_connected:
                return
            self.held, self.stop_pending = key, False
            self.last_keepalive = time.monotonic()
        self.wake.set()

    def release(self, key=None):
        with self.lock:
            if key is None or key == self.held:
                self.held = None
        self.wake.set()

    def stop(self):
        with self.lock:
            self.held, self.stop_pending = None, True
        self.wake.set()

    def keepalive(self):
        with self.lock:
            self.last_keepalive = time.monotonic()

    def set_speed(self, speed):
        with self.lock:
            self.speed = max(0, min(255, int(speed)))

    def status(self):
        with self.lock:
            return self.state, self.text

    def close(self, timeout_s=1.5):
        with self.lock:
            self.want_connected, self.held, self.closed = False, None, True
        self.wake.set()
        if self.thread is not None:
            self.thread.join(timeout_s)

    # -- worker thread ------------------------------------------------------------
    def set_status(self, state, text):
        with self.lock:
            self.state, self.text = state, text

    def send(self, call, *args):
        self.requests += 1
        return call(*args)

    def safe_stop(self, link):
        for _ in range(2):  # one retry
            try:
                self.send(link.stop)
                return True
            except Exception:
                pass
        return False

    def fail(self, link, action, error):
        stopped = self.safe_stop(link)
        with self.lock:
            self.want_connected, self.held = False, None
            self.state = "error"
            self.text = (f"Connection lost ({action}: {error}). "
                         + ("STOP sent." if stopped else "STOP not confirmed: use the power switch."))

    def run(self):
        link, sent, sent_at = None, None, 0.0
        while True:
            self.wake.wait(RENEW_S / 3)
            self.wake.clear()
            with self.lock:
                closed, want, url = self.closed, self.want_connected, self.url
                held, speed, stop_pending = self.held, self.speed, self.stop_pending
                self.stop_pending = False
                fresh = time.monotonic() - self.last_keepalive < KEEPALIVE_S
                if held is not None and not fresh:
                    self.held = held = None  # GUI lost the pad or froze: stop
            if link is not None and (not want or link.base_url != url.rstrip("/")):
                if sent is not None or not want:
                    self.safe_stop(link)
                link, sent = None, None
                if not want and self.status()[0] != "error":
                    self.set_status("disconnected", "Not connected.")
            if closed:
                return
            if want and link is None:
                candidate = self.link_factory(url)
                try:
                    reply = self.send(candidate.ping)
                except Exception as error:
                    with self.lock:
                        if self.url == url:
                            self.want_connected = False
                            self.state, self.text = "error", f"Cannot reach {url}: {error}"
                    continue
                link = candidate
                self.set_status("connected", f"Connected to {url} (ping: {reply}).")
                continue
            if link is None:
                continue
            if stop_pending or (held is None and sent is not None):
                if not self.safe_stop(link):
                    self.fail(link, "stop", "no answer")
                    link, sent = None, None
                    continue
                sent = None
                self.set_status("connected", "Stopped.")
            if held is not None:
                now = time.monotonic()
                if held != sent or now - sent_at >= RENEW_S:
                    route, factor = KEYS[held]
                    try:
                        self.send(link.move, route, MOVE_TIME_MS * factor, speed)
                    except Exception as error:
                        self.fail(link, route, error)
                        link, sent = None, None
                        continue
                    sent, sent_at = held, now
                    self.set_status("connected", f"Driving: {ROUTE_NAMES[route]} (speed {speed}).")
