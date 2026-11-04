"""Tests of HttpRobotLink against the fake ESP32SlamRobot (approach B, gate S0).

Everything runs on 127.0.0.1 in real time; no robot, no Wi-Fi, no port.
Run: .\\.venv\\Scripts\\python.exe -m unittest test_autonomy_http -v
"""
import math
import struct
import threading
import time
import unittest

from autonomy import protocol as p
from autonomy.fake_http_robot import FakeHttpRobot, sample_points
from autonomy.http_robot_link import (LEASE_MAX_MS, Calibration, HttpRobotLink, LinkError,
                                      parse_scan2)

# Fast measured calibration for tests: 1 m/s and 360 deg/s at 40 %.
FAST = Calibration(forward_pct=40, forward_m_s=1.0, turn_pct=40,
                   turn_rad_s=math.radians(360.0), measured=True)


class HttpLinkTestCase(unittest.TestCase):
    calibration = FAST

    def setUp(self):
        self.robot = FakeHttpRobot().start()
        self.addCleanup(self.robot.close)
        self.link = HttpRobotLink(self.robot.url, calibration=self.calibration)
        time.sleep(0.15)            # the fake LiDAR publishes its first scan

    def connect_and_arm(self):
        self.link.connect()
        self.assertTrue(self.link.arm().ok)


class ConnectTests(HttpLinkTestCase):
    def test_connect_reads_status(self):
        status = self.link.connect()
        self.assertEqual(status["boot_id"], self.robot.boot_id)
        self.assertEqual(status["armed"], 0)

    def test_connect_fails_without_robot(self):
        link = HttpRobotLink("http://127.0.0.1:9", http_timeout_s=0.2)
        with self.assertRaises(LinkError):
            link.connect()

    def test_boots_disarmed_and_move_refused(self):
        self.link.connect()
        result = self.link.move(0.1, 0.0)
        self.assertEqual(result.outcome, "refused")
        self.assertEqual(result.detail, "DISARMED")
        self.assertFalse(self.robot.move_active)


class MoveTests(HttpLinkTestCase):
    def test_turn_then_forward_with_signs_and_times(self):
        self.connect_and_arm()
        result = self.link.move(0.2, math.radians(-45.0))
        self.assertTrue(result.completed, result)
        self.assertEqual(self.robot.drives, [(0, 0, -40, 125, 1), (40, 0, 0, 200, 2)])
        self.assertFalse(self.robot.move_active)

    def test_long_stage_split_into_leases(self):
        self.calibration = Calibration(40, 0.2, 40, math.radians(360.0), measured=True)
        self.link.calibration = self.calibration
        self.connect_and_arm()
        result = self.link.move(-0.3, 0.0)          # 1500 ms at 0.2 m/s
        self.assertTrue(result.completed, result)
        self.assertEqual(self.robot.drives, [(-40, 0, 0, LEASE_MAX_MS, 1), (-40, 0, 0, 500, 2)])
        for drive in self.robot.drives:
            self.assertLessEqual(drive[3], LEASE_MAX_MS)

    def test_limits_refused_before_any_request(self):
        self.connect_and_arm()
        for forward, turn in ((0.301, 0.0), (0.0, math.radians(90.5)), (-0.31, 0.0)):
            result = self.link.move(forward, turn)
            self.assertEqual((result.outcome, result.detail), ("refused", "OUT_OF_BOUNDS"))
        self.assertEqual(self.robot.drives, [])
        self.assertLessEqual(p.MAX_FORWARD_MM, 300)

    def test_uncalibrated_move_refused(self):
        link = HttpRobotLink(self.robot.url)        # placeholder calibration
        link.connect()
        link.arm()
        result = link.move(0.1, 0.0)
        self.assertEqual(result.outcome, "refused")
        self.assertIn("uncalibrated", result.detail)
        self.assertEqual(self.robot.drives, [])

    def test_no_lidar_refused(self):
        self.connect_and_arm()
        self.robot.lidar_ok = False
        time.sleep(0.6)                             # older than LIDAR_STALE_MS
        result = self.link.move(0.1, 0.0)
        self.assertEqual((result.outcome, result.detail), ("refused", "NO_LIDAR"))
        self.assertFalse(self.robot.move_active)

    def test_lidar_loss_mid_move_stops_robot(self):
        self.link.calibration = Calibration(40, 0.2, 40, math.radians(360.0), measured=True)
        self.connect_and_arm()
        threading.Timer(0.1, lambda: setattr(self.robot, "lidar_ok", False)).start()
        started = time.monotonic()
        result = self.link.move(0.18, 0.0)          # a 900 ms lease
        self.assertEqual(result.outcome, "fault")
        self.assertIn("LiDAR stale", result.detail)
        # the fake ended the move about LIDAR_STALE_MS after the last scan
        with self.robot.lock:
            stopped_after = self.robot.last_motion_end - self.robot.move_start
        self.assertLess(stopped_after, 800)
        self.assertLess(time.monotonic() - started, 2.0)

    def test_stop_from_another_thread_interrupts_move(self):
        self.link.calibration = Calibration(40, 0.2, 40, math.radians(360.0), measured=True)
        self.connect_and_arm()
        threading.Timer(0.2, self.link.stop).start()
        started = time.monotonic()
        result = self.link.move(0.3, 0.0)
        self.assertEqual(result.outcome, "stopped")
        self.assertLess(time.monotonic() - started, 0.9)
        self.assertFalse(self.robot.move_active)
        self.assertFalse(self.robot.armed)          # /stop disarms
        self.assertEqual(self.link.move(0.1, 0.0).outcome, "stopped")  # until arm()

    def test_restart_mid_move_reported(self):
        self.connect_and_arm()
        threading.Timer(0.05, self.robot.restart).start()
        result = self.link.move(0.25, 0.0)
        self.assertEqual(result.outcome, "restarted")
        self.assertFalse(self.robot.armed)
        self.assertEqual(self.link.move(0.1, 0.0).outcome, "refused")   # DISARMED

    def test_link_lost_waits_for_lease_end(self):
        self.connect_and_arm()
        threading.Timer(0.05, lambda: setattr(self.robot, "silent", True)).start()
        started = time.monotonic()
        result = self.link.move(0.25, 0.0)           # a 250 ms lease
        elapsed_ms = (time.monotonic() - started) * 1000.0
        self.assertEqual(result.outcome, "link_lost")
        self.assertGreaterEqual(elapsed_ms, 250 + 200)
        with self.robot.lock:
            self.robot._motor_step(self.robot._now())
            self.assertFalse(self.robot.move_active)  # the lease ended on the robot

    def test_robot_still_moving_after_lease_is_a_fault(self):
        self.connect_and_arm()
        self.robot.stuck = True                      # a board that holds its PWM
        result = self.link.move(0.1, 0.0)
        self.assertEqual(result.outcome, "fault")
        self.assertIn("still moving", result.detail)

    def test_seq_restarts_after_arm(self):
        self.connect_and_arm()
        self.assertTrue(self.link.move(0.05, 0.0).completed)
        self.assertTrue(self.link.arm().ok)
        self.assertTrue(self.link.move(0.05, 0.0).completed)
        self.assertEqual([d[4] for d in self.robot.drives], [1, 1])


class ScanTests(HttpLinkTestCase):
    def test_scan_after_move_is_still_and_newer(self):
        self.connect_and_arm()
        self.assertTrue(self.link.move(0.1, 0.0).completed)
        motion_end_seq = self.link.min_scan_seq
        result = self.link.request_scan(timeout_ms=2000)
        self.assertTrue(result.ok, result.reason)
        self.assertEqual(result.points, sample_points())
        self.assertGreater(self.robot.scan_seq, motion_end_seq)
        xy = p.points_to_sensor_xy(result.points[:1])
        self.assertAlmostEqual(xy[0][0], 1.0)        # angle 0 = straight ahead, CCW

    def test_scan_times_out_without_lidar(self):
        self.robot.lidar_ok = False
        self.link.connect()
        self.link.min_scan_seq = self.robot.scan_seq
        result = self.link.request_scan(timeout_ms=300)
        self.assertFalse(result.ok)
        self.assertIn("timeout", result.reason)

    def test_parse_scan2_rejects_malformed(self):
        good = struct.pack("<2sBBIIHH", b"S2", 1, 1, 5, 99, 3600, 0) + struct.pack(
            "<hhHH", 0, 0, 0, 2) + struct.pack("<HHB", 100, 500, 7) + struct.pack(
            "<HHB", 35999, 0, 0)
        self.assertEqual(parse_scan2(good), (5, 99, True, ((100, 500, 7), (35999, 0, 0))))
        for bad in (good[:10], b"X" + good[1:], good[:2] + b"\x02" + good[3:], good[:-1]):
            with self.assertRaises(ValueError):
                parse_scan2(bad)


if __name__ == "__main__":
    unittest.main()
