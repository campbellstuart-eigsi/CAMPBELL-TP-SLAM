"""Teleop (manual driving core) against the fake ESP32SlamRobot. No Tk, no hardware.

Run: .\\.venv\\Scripts\\python.exe -m unittest test_autonomy_teleop -v
"""
import time
import unittest

from autonomy.fake_http_robot import FakeHttpRobot
from autonomy.teleop import LEASE_MS, MAX_MANUAL_PCT, Teleop


def wait_until(condition, timeout_s=2.0, step_s=0.01):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(step_s)
    return condition()


class TeleopTestCase(unittest.TestCase):
    def setUp(self):
        self.robot = FakeHttpRobot().start()
        self.addCleanup(self.robot.close)
        self.teleop = Teleop()
        self.addCleanup(self.teleop.close)
        time.sleep(0.15)             # the fake LiDAR publishes its first scan

    def connect(self):
        self.assertTrue(self.teleop.request_connect(self.robot.url))
        self.assertTrue(wait_until(lambda: self.teleop.snapshot().link == "ok"))
        self.assertTrue(wait_until(lambda: not self.teleop.snapshot().busy))

    def arm(self):
        self.connect()
        self.teleop.request_arm()
        self.assertTrue(wait_until(lambda: self.teleop.snapshot().armed))

    def moving_drives(self):
        return [d for d in self.robot.drives if d[:3] != (0, 0, 0)]


class ConnectionTests(TeleopTestCase):
    def test_starts_disconnected_and_never_drives(self):
        self.teleop.press("forward")
        time.sleep(0.3)
        snap = self.teleop.snapshot()
        self.assertEqual((snap.link, snap.armed, snap.command), ("disconnected", False, (0, 0, 0)))
        self.assertEqual(self.robot.drives, [])

    def test_connect_alone_never_arms_or_drives(self):
        self.connect()
        time.sleep(0.3)
        self.assertEqual(self.robot.drives, [])
        self.assertTrue(self.teleop.snapshot().stop_latched)
        self.assertFalse(self.robot.armed)

    def test_first_key_press_arms_then_drives(self):
        self.connect()
        self.teleop.press("forward")
        self.assertTrue(wait_until(lambda: self.robot.move_active))
        self.assertTrue(self.robot.armed)
        self.assertEqual(self.moving_drives()[0][:4], (30, 0, 0, LEASE_MS))
        self.teleop.release("forward")
        self.assertTrue(wait_until(lambda: not self.robot.move_active, 0.3))

    def test_key_released_before_arm_completes_never_moves(self):
        self.connect()
        self.teleop.press("forward")
        self.teleop.release("forward")
        self.assertTrue(wait_until(lambda: self.teleop.snapshot().armed))
        time.sleep(0.3)
        self.assertEqual(self.moving_drives(), [])

    def test_disconnect_stops_and_disarms(self):
        self.arm()
        self.teleop.press("forward")
        self.assertTrue(wait_until(lambda: self.robot.move_active))
        self.teleop.request_disconnect()
        self.assertTrue(wait_until(lambda: self.teleop.snapshot().link == "disconnected"))
        self.assertFalse(self.robot.armed)
        self.assertFalse(self.robot.move_active)

    def test_mode_changes_only_while_disconnected(self):
        self.assertTrue(self.teleop.set_mode(True))
        self.connect()
        self.assertFalse(self.teleop.set_mode(False))
        self.assertTrue(self.teleop.snapshot().real_robot)


class DrivingTests(TeleopTestCase):
    def test_held_key_renews_short_leases(self):
        self.arm()
        self.teleop.press("forward")
        time.sleep(0.55)
        drives = self.moving_drives()
        self.assertGreaterEqual(len(drives), 4)
        for vx, vy, wz, lease, _ in drives:
            self.assertEqual((vx, vy, wz, lease), (30, 0, 0, LEASE_MS))
        self.assertTrue(self.robot.move_active)
        seqs = [d[4] for d in self.robot.drives]
        self.assertEqual(seqs, sorted(set(seqs)))       # strictly increasing

    def test_release_sends_zero_at_once(self):
        self.arm()
        self.teleop.press("rotate_left")
        self.assertTrue(wait_until(lambda: self.robot.move_active))
        released = time.monotonic()
        self.teleop.release("rotate_left")
        self.assertTrue(wait_until(lambda: not self.robot.move_active, 0.5))
        self.assertLess(time.monotonic() - released, 0.15)
        self.assertEqual(self.robot.drives[-1][:4], (0, 0, 0, 0))

    def test_release_all_on_focus_loss(self):
        self.arm()
        self.teleop.press("forward")
        self.teleop.press("left")
        self.assertTrue(wait_until(lambda: self.robot.move_active))
        self.teleop.release_all()
        self.assertTrue(wait_until(lambda: not self.robot.move_active, 0.15))
        count = len(self.moving_drives())
        time.sleep(0.3)
        self.assertEqual(len(self.moving_drives()), count)   # nothing queued

    def test_keys_combine_and_speeds_are_capped(self):
        self.arm()
        self.teleop.set_speeds(80, 45)
        self.assertEqual(self.teleop.snapshot().drive_pct, MAX_MANUAL_PCT)
        self.teleop.press("forward")
        self.teleop.press("rotate_left")
        self.assertEqual(self.teleop.command(), (MAX_MANUAL_PCT, 0, 45))
        self.teleop.press("backward")             # opposite keys cancel
        self.assertEqual(self.teleop.command(), (0, 0, 45))

    def test_firmware_lease_expires_if_controller_freezes(self):
        self.arm()
        self.teleop.press("forward")
        self.assertTrue(wait_until(lambda: self.robot.move_active))
        self.teleop.frozen = True                 # a hung controller: no renewals
        frozen_at = time.monotonic()
        self.assertTrue(wait_until(lambda: (self.robot._motor_step(self.robot._now()),
                                            not self.robot.move_active)[1], 1.0))
        self.assertLess(time.monotonic() - frozen_at, (LEASE_MS + 150) / 1000.0)

    def test_distance_moves_still_need_calibration(self):
        self.arm()
        self.assertFalse(self.teleop.link.calibration.measured)
        self.assertEqual(self.teleop.link.move(0.1, 0.0).outcome, "refused")


class StopAndFailureTests(TeleopTestCase):
    def test_stop_disarms_latches_and_ignores_keys(self):
        self.arm()
        self.teleop.press("forward")
        self.assertTrue(wait_until(lambda: self.robot.move_active))
        stopped = time.monotonic()
        self.teleop.stop()
        self.assertTrue(wait_until(lambda: not self.robot.armed, 0.3))
        self.assertLess(time.monotonic() - stopped, 0.2)
        self.assertFalse(self.robot.move_active)
        count = len(self.moving_drives())
        self.teleop.press("forward")
        time.sleep(0.3)
        self.assertEqual(len(self.moving_drives()), count)
        snap = self.teleop.snapshot()
        self.assertTrue(snap.stop_latched)
        self.assertFalse(snap.armed)

    def test_after_stop_a_held_key_must_be_released_to_drive_again(self):
        self.connect()
        self.teleop.press("forward")
        self.assertTrue(wait_until(lambda: self.robot.move_active))
        self.teleop.stop()
        self.assertTrue(wait_until(lambda: not self.robot.armed, 0.3))
        count = len(self.moving_drives())
        for _ in range(5):                         # keyboard auto-repeat of the held key
            self.teleop.press("forward")
        time.sleep(0.4)
        self.assertEqual(len(self.moving_drives()), count)
        self.assertFalse(self.robot.armed)
        self.teleop.release("forward")             # a deliberate new press arms and drives
        time.sleep(1.0)                            # past AUTO_ARM_RETRY_MS
        self.teleop.press("forward")
        self.assertTrue(wait_until(lambda: self.robot.move_active))
        self.teleop.release("forward")

    def test_stop_during_automatic_arm_leaves_the_robot_disarmed(self):
        self.connect()
        link = self.teleop.link
        original_arm = link.arm

        def arm_while_stop_is_pressed():
            self.teleop.stop()                     # STOP lands while /arm is in flight
            return original_arm()

        link.arm = arm_while_stop_is_pressed
        self.teleop.press("forward")
        self.assertTrue(wait_until(lambda: self.teleop.snapshot().busy == "", 2.0))
        time.sleep(0.3)
        self.assertFalse(self.robot.armed)         # the ARM after the STOP was undone
        self.assertFalse(self.teleop.snapshot().armed)
        self.assertEqual(self.moving_drives(), [])

    def test_delayed_drive_after_stop_and_rearm_is_refused(self):
        self.arm()
        self.teleop.press("forward")
        self.assertTrue(wait_until(lambda: len(self.moving_drives()) >= 2))
        old_seq = self.moving_drives()[0][4]
        self.teleop.stop()
        self.assertTrue(wait_until(lambda: not self.robot.armed))
        link = self.teleop.link
        unused = link.seq + 100                    # never sent: only DISARMED refuses it
        self.assertEqual(link._get(f"/drive?vx=30&time=300&seq={unused}")[0], 423)
        self.teleop.request_arm()
        self.assertTrue(wait_until(lambda: self.teleop.snapshot().armed))
        code, body = link._get(f"/drive?vx=30&time=300&seq={old_seq}")
        self.assertEqual((code, body), (409, b"STALE_SEQ"))
        self.assertFalse(self.robot.move_active)

    def test_link_loss_latches_and_never_resumes(self):
        self.arm()
        self.teleop.press("forward")
        self.assertTrue(wait_until(lambda: self.robot.move_active))
        self.robot.silent = True
        self.assertTrue(wait_until(lambda: self.teleop.snapshot().link == "lost", 2.0))
        snap = self.teleop.snapshot()
        self.assertTrue(snap.stop_latched)
        self.assertEqual(snap.held, ())
        self.robot.silent = False
        recovered = len(self.robot.drives)
        self.assertTrue(wait_until(lambda: self.teleop.snapshot().link == "ok", 2.0))
        time.sleep(0.4)
        self.assertEqual([d for d in self.robot.drives[recovered:] if d[:3] != (0, 0, 0)], [])
        self.assertTrue(self.teleop.snapshot().stop_latched)

    def test_restart_detected_and_latched(self):
        self.arm()
        self.robot.restart()
        self.assertTrue(wait_until(lambda: self.teleop.snapshot().stop_latched, 1.0))
        self.assertTrue(any("restarted" in text for _, text in self.teleop.snapshot().errors))

    def test_no_lidar_refusal_clears_intent(self):
        self.arm()
        self.robot.lidar_ok = False
        time.sleep(0.6)
        self.teleop.press("forward")
        self.assertTrue(wait_until(lambda: any("NO_LIDAR" in text for _, text in
                                               self.teleop.snapshot().errors)))
        self.assertEqual(self.teleop.snapshot().held, ())
        self.assertFalse(self.robot.move_active)

    def test_stop_returns_at_once_while_robot_is_silent(self):
        self.arm()
        self.robot.silent = True
        started = time.monotonic()
        self.teleop.stop()                         # must not wait for the network
        self.assertLess(time.monotonic() - started, 0.05)


class RejectionReasonTests(TeleopTestCase):
    def test_locked_build_shows_motors_locked(self):
        locked = FakeHttpRobot(locked=True).start()
        self.addCleanup(locked.close)
        self.assertTrue(self.teleop.request_connect(locked.url))
        self.assertTrue(wait_until(lambda: self.teleop.snapshot().link == "ok"))
        self.assertTrue(wait_until(lambda: not self.teleop.snapshot().busy))
        self.teleop.press("forward")               # the automatic ARM is refused
        self.assertTrue(wait_until(lambda: any("cannot drive: MOTORS_LOCKED (HTTP 423)" in text
                                               for _, text in self.teleop.snapshot().errors)))
        self.assertFalse(self.teleop.snapshot().armed)
        for _ in range(5):                         # auto-repeat does not retry the ARM
            self.teleop.press("forward")
        time.sleep(1.2)
        self.assertEqual(locked.rejects, 1)
        self.assertEqual(locked.drives, [])        # nothing is sent while not armed

    def test_drive_refusal_shows_the_firmware_reason(self):
        self.arm()
        self.robot.armed = False                   # e.g. the Wi-Fi-drop event disarmed it
        self.teleop.press("forward")
        self.assertTrue(wait_until(lambda: any("DISARMED (HTTP 423)" in text for _, text
                                               in self.teleop.snapshot().errors)))
        self.assertEqual(self.teleop.snapshot().held, ())


if __name__ == "__main__":
    unittest.main()
