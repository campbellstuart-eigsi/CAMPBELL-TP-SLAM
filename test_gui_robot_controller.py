"""Real-Tk tests of the robot controller window, in SIMULATOR mode (fake robot, no hardware).

Run: .\\.venv\\Scripts\\python.exe -m unittest test_gui_robot_controller -v
"""
import gc
import time
import tkinter as tk
import unittest

from autonomy.controller_ui import RobotControllerPanel
from ui_fonts import FontSet
from ui_theme import apply_styles


class ControllerGuiTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.geometry("1280x900")
        fonts = FontSet(self.root)
        apply_styles(self.root, fonts=fonts)
        self.panel = RobotControllerPanel(self.root, fonts=fonts)
        self.panel.pack(fill="both", expand=True)
        self.focus()

    def focus(self):
        """Keys only reach a focused window; without the OS focus every
        FocusOut correctly releases the keys."""
        self.root.deiconify()
        self.root.lift()
        self.root.focus_force()
        self.panel.focus_set()
        self.pump(0.2)

    def tearDown(self):
        self.panel.close()
        self.root.destroy()
        gc.collect()                 # free Tk Variables on the main thread (as in M1's GUI tests)

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.root.update()
            time.sleep(0.005)

    def pump_until(self, condition, timeout_s=3.0):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            self.root.update()
            if condition():
                return True
            time.sleep(0.005)
        return condition()

    def key(self, keysym, release=False, widget=None):
        (widget or self.root).event_generate("<KeyRelease>" if release else "<KeyPress>",
                                             keysym=keysym, when="tail")
        self.root.update()

    def connect_and_arm(self):
        self.panel.connect_button.invoke()
        self.assertTrue(self.pump_until(lambda: self.panel.teleop.snapshot().link == "ok"))
        self.assertTrue(self.pump_until(
            lambda: str(self.panel.arm_button["state"]) == "normal"))
        self.panel.arm_button.invoke()
        self.assertTrue(self.pump_until(lambda: self.panel.teleop.snapshot().armed))
        self.focus()

    def moving(self):
        return [d for d in self.panel.fake.drives if d[:3] != (0, 0, 0)]

    def test_launch_is_simulator_disconnected_and_disarmed(self):
        snap = self.panel.teleop.snapshot()
        self.assertFalse(snap.real_robot)
        self.assertEqual(snap.link, "disconnected")
        self.assertIsNone(self.panel.fake)           # nothing started or connected
        self.assertEqual(str(self.panel.arm_button["state"]), "disabled")
        self.assertIn("SIMULATOR", self.panel.banner["text"])

    def test_azerty_keys_drive_and_release_stops(self):
        self.connect_and_arm()
        self.assertIn("Z", self.panel.key_labels["forward"]["text"])
        self.assertIn("A", self.panel.key_labels["rotate_left"]["text"])
        self.key("z")
        self.assertTrue(self.pump_until(lambda: self.panel.fake.move_active))
        self.assertEqual(self.moving()[0][:4], (30, 0, 0, 300))
        self.assertTrue(self.pump_until(
            lambda: str(self.panel.key_labels["forward"]["style"]) == "KeyOn.TLabel"))
        self.key("z", release=True)
        self.assertTrue(self.pump_until(lambda: not self.panel.fake.move_active, 0.3))
        for keysym, expected in (("q", (0, 30, 0)), ("d", (0, -30, 0)), ("a", (0, 0, 30)),
                                 ("e", (0, 0, -30)), ("s", (-30, 0, 0))):
            count = len(self.moving())
            self.key(keysym)
            self.assertTrue(self.pump_until(lambda: len(self.moving()) > count))
            self.assertEqual(self.moving()[-1][:3], expected, keysym)
            self.key(keysym, release=True)
            self.assertTrue(self.pump_until(lambda: not self.panel.fake.move_active, 0.3))

    def test_layout_is_configurable(self):
        self.connect_and_arm()
        self.key("w")                                # not bound in AZERTY
        self.pump(0.3)
        self.assertEqual(self.moving(), [])
        self.panel.layout_name.set("QWERTY")
        self.panel._apply_layout()
        self.assertIn("W", self.panel.key_labels["forward"]["text"])
        self.key("w")
        self.assertTrue(self.pump_until(lambda: self.panel.fake.move_active))
        self.key("w", release=True)

    def test_space_is_stop(self):
        self.connect_and_arm()
        self.key("z")
        self.assertTrue(self.pump_until(lambda: self.panel.fake.move_active))
        self.key("space")
        self.assertTrue(self.pump_until(lambda: not self.panel.fake.armed, 0.3))
        self.assertFalse(self.panel.fake.move_active)
        self.assertTrue(self.pump_until(
            lambda: "STOP latched" in self.panel.status_values["Command"]["text"]))

    def test_stop_button(self):
        self.connect_and_arm()
        self.key("e")
        self.assertTrue(self.pump_until(lambda: self.panel.fake.move_active))
        self.panel.stop_button.invoke()
        self.assertTrue(self.pump_until(lambda: not self.panel.fake.armed, 0.3))

    def test_focus_loss_clears_intent(self):
        self.connect_and_arm()
        self.key("z")
        self.assertTrue(self.pump_until(lambda: self.panel.fake.move_active))
        self.panel.focus_get = lambda: None         # the OS gave the focus to another window
        self.root.event_generate("<FocusOut>", when="tail")
        self.assertTrue(self.pump_until(lambda: not self.panel.fake.move_active, 0.3))
        self.assertEqual(self.panel.teleop.snapshot().held, ())

    def test_typing_in_url_field_never_drives(self):
        self.panel._set_mode("real")
        self.pump(0.2)
        self.panel.url_entry.focus_force()
        self.pump(0.1)
        self.panel.focus_get = lambda: self.panel.url_entry
        self.key("z", widget=self.panel.url_entry)
        self.assertEqual(self.panel.teleop.snapshot().held, ())

    def test_real_mode_arm_needs_all_safety_checks(self):
        self.panel._set_mode("real")
        self.pump(0.2)
        self.assertIn("REAL ROBOT", self.panel.banner["text"])
        self.assertTrue(self.panel.safety_body.master.winfo_ismapped())
        self.assertFalse(self.panel.sim_body.master.winfo_ismapped())
        # Point the REAL ROBOT mode at a fake robot: no hardware is ever contacted.
        from autonomy.fake_http_robot import FakeHttpRobot
        fake = FakeHttpRobot().start()
        self.addCleanup(fake.close)
        self.panel.url.set(fake.url)
        self.panel.connect_button.invoke()
        self.assertTrue(self.pump_until(lambda: self.panel.teleop.snapshot().link == "ok"))
        self.pump(0.2)
        for index, var in enumerate(self.panel.checks):
            self.assertEqual(str(self.panel.arm_button["state"]), "disabled", index)
            var.set(True)
            self.panel._safety_changed()
            self.pump(0.15)
        self.assertEqual(str(self.panel.arm_button["state"]), "normal")
        self.panel.arm_button.invoke()
        self.assertTrue(self.pump_until(lambda: fake.armed))

    def test_preview_draws_scans(self):
        self.connect_and_arm()
        self.assertTrue(self.pump_until(lambda: (self.panel.drawn_scan or 0) > 0))
        self.assertGreater(len(self.panel.points_artist.get_offsets()), 300)
        first = self.panel.drawn_scan
        self.key("a")                                # rotate: the room scan changes
        self.pump(0.5)
        self.key("a", release=True)
        self.assertTrue(self.pump_until(lambda: self.panel.drawn_scan != first))

    def test_simulator_faults(self):
        self.connect_and_arm()
        self.panel.fault_vars["lidar_off"].set(True)
        self.panel._faults_changed()
        self.assertFalse(self.panel.fake.lidar_ok)
        self.assertTrue(self.pump_until(
            lambda: str(self.panel.status_values["LiDAR"]["style"]) == "Bad.TLabel"))
        self.panel.fault_vars["lidar_off"].set(False)
        self.panel._faults_changed()
        self.panel._sim_restart()
        self.assertTrue(self.pump_until(
            lambda: any("restarted" in t for _, t in self.panel.teleop.snapshot().errors)))

    def test_ui_stays_responsive_while_requests_time_out(self):
        self.connect_and_arm()
        self.key("z")
        self.panel.fault_vars["silent"].set(True)
        self.panel._faults_changed()
        gaps, last = [], [time.monotonic()]

        def tick():
            now = time.monotonic()
            gaps.append(now - last[0])
            last[0] = now
            self.root.after(20, tick)

        self.root.after(20, tick)
        self.pump(2.0)
        self.assertGreater(len(gaps), 30)
        self.assertLess(max(gaps), 0.15, f"Tk thread blocked for {max(gaps):.3f} s")
        self.assertEqual(self.panel.teleop.snapshot().link, "lost")
        self.assertEqual(str(self.panel.status_values["Link"]["style"]), "Bad.TLabel")

    def test_close_stops_and_disarms(self):
        self.connect_and_arm()
        fake = self.panel.fake
        self.key("z")
        self.assertTrue(self.pump_until(lambda: fake.move_active))
        fake_armed_before = fake.armed
        self.panel.teleop.close()
        self.assertTrue(fake_armed_before)
        self.assertFalse(fake.armed)
        self.assertFalse(fake.move_active)


if __name__ == "__main__":
    unittest.main()
