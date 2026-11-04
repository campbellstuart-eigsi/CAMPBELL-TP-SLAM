"""robot_drive: hold-to-drive over the professor's HTTP routes, against a local fake robot."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
import unittest
from urllib.parse import parse_qs, urlparse

import robot_drive


class FakeRobot:
    """Records every request; answers like ESP32RobotControl.ino."""

    def __init__(self):
        self.requests = []
        self.lock = threading.Lock()
        robot = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                url = urlparse(self.path)
                params = {k: v[0] for k, v in parse_qs(url.query).items()}
                with robot.lock:
                    robot.requests.append((url.path, params))
                body = b"STOPPED" if url.path == "/stop" else b"OK"
                self.send_response(200)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.url = f"http://127.0.0.1:{self.server.server_address[1]}"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def paths(self):
        with self.lock:
            return [path for path, _ in self.requests]

    def last(self, path):
        with self.lock:
            return next(params for p, params in reversed(self.requests) if p == path)

    def close(self):
        self.server.shutdown()
        self.server.server_close()


class ThreadRecordingLink(robot_drive.DriveLink):
    threads = []

    def get(self, path, timeout_s, **params):
        self.threads.append(threading.current_thread().name)
        return super().get(path, timeout_s, **params)


class DriveControllerTests(unittest.TestCase):
    def setUp(self):
        self.robot = FakeRobot()
        ThreadRecordingLink.threads = []
        self.drive = robot_drive.DriveController(link_factory=ThreadRecordingLink)

    def tearDown(self):
        self.drive.close()
        self.robot.close()

    def wait_for(self, condition, timeout=3.0, keepalive=False):
        deadline = time.monotonic() + timeout
        while not condition():
            self.assertLess(time.monotonic(), deadline, f"timed out; status {self.drive.status()}")
            if keepalive:
                self.drive.keepalive()
            time.sleep(0.02)

    def connected(self):
        self.drive.connect(self.robot.url)
        self.wait_for(lambda: self.drive.status()[0] == "connected")
        self.assertEqual(self.robot.paths(), ["/ping"])

    def hold_for(self, key, seconds):
        self.drive.hold(key)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.drive.keepalive()  # what the Workspace tick does while the pad has focus
            time.sleep(0.05)

    def test_hold_renews_short_moves_and_release_stops(self):
        self.connected()
        self.hold_for("z", 0.7)
        self.drive.release("z")
        self.wait_for(lambda: self.robot.paths()[-1:] == ["/stop"])
        moves = [p for p in self.robot.paths() if p == "/forward"]
        self.assertGreaterEqual(len(moves), 3)  # renewed every RENEW_S
        self.assertEqual(self.robot.last("/forward"), {"time": "300", "speed": "80"})
        count = len(self.robot.paths())
        time.sleep(0.4)
        self.assertEqual(len(self.robot.paths()), count, "nothing is sent after the stop")

    def test_every_key_maps_to_its_route(self):
        self.connected()
        expected = {"z": ("/forward", "300"), "s": ("/backward", "300"), "q": ("/left", "300"),
                    "d": ("/right", "300"), "a": ("/rotateleft", "600"),
                    "e": ("/rotateright", "600")}  # the firmware rotates for time / 2
        for key, (path, time_ms) in expected.items():
            with self.subTest(key=key):
                self.hold_for(key, 0.2)
                self.drive.release(key)
                self.wait_for(lambda: self.robot.paths()[-1:] == ["/stop"])
                self.assertEqual(self.robot.last(path)["time"], time_ms)

    def test_releasing_another_key_keeps_the_held_move(self):
        self.connected()
        self.drive.hold("z")
        self.drive.release("s")
        self.hold_for("z", 0.3)
        self.assertNotIn("/stop", self.robot.paths())
        self.drive.release("z")
        self.wait_for(lambda: self.robot.paths()[-1:] == ["/stop"])

    def test_space_stop(self):
        self.connected()
        self.hold_for("d", 0.2)
        self.drive.stop()
        self.wait_for(lambda: self.robot.paths()[-1:] == ["/stop"])
        self.assertIsNone(self.drive.held)

    def test_missing_keepalive_stops(self):
        self.connected()
        self.drive.hold("z")  # no keepalive: the GUI lost the pad or froze
        self.wait_for(lambda: self.robot.paths()[-1:] == ["/stop"], timeout=2.0)
        self.assertIsNone(self.drive.held)

    def test_speed_is_sent_and_clamped(self):
        self.connected()
        self.drive.set_speed(400)
        self.hold_for("z", 0.2)
        self.assertEqual(self.robot.last("/forward")["speed"], "255")
        self.drive.set_speed(120.7)
        self.hold_for("z", 0.3)
        self.assertEqual(self.robot.last("/forward")["speed"], "120")

    def test_disconnect_sends_stop(self):
        self.connected()
        self.drive.disconnect()
        self.wait_for(lambda: self.drive.status()[0] == "disconnected")
        self.assertEqual(self.robot.paths()[-1], "/stop")
        self.drive.hold("z")  # ignored while disconnected
        time.sleep(0.3)
        self.assertNotIn("/forward", self.robot.paths())

    def test_close_sends_stop(self):
        self.connected()
        self.hold_for("z", 0.2)
        self.drive.close()
        self.assertFalse(self.drive.thread.is_alive())
        self.assertEqual(self.robot.paths()[-1], "/stop")

    def test_unreachable_robot_reports_an_error(self):
        url = self.robot.url
        self.robot.close()
        self.drive.connect(url)
        self.wait_for(lambda: self.drive.status()[0] == "error")
        self.assertIn("Cannot reach", self.drive.status()[1])
        self.drive.hold("z")
        self.assertIsNone(self.drive.held)

    def test_robot_lost_while_driving(self):
        self.connected()
        self.hold_for("z", 0.2)
        self.robot.close()
        self.wait_for(lambda: self.drive.status()[0] == "error", keepalive=True)
        self.assertIn("Connection lost", self.drive.status()[1])
        self.assertIsNone(self.drive.held)

    def test_all_requests_run_on_the_worker_thread(self):
        self.connected()
        self.hold_for("q", 0.3)
        self.drive.stop()
        self.wait_for(lambda: self.robot.paths()[-1:] == ["/stop"])
        self.assertTrue(ThreadRecordingLink.threads)
        self.assertEqual(set(ThreadRecordingLink.threads), {"robot-drive"})


if __name__ == "__main__":
    unittest.main()
