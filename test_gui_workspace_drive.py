"""Workspace Drive panel in a real Tk loop, against a local fake robot (never 192.168.4.1)."""
import time
import unittest

import test_gui_workspace as base
import workspace_page
from test_robot_drive import FakeRobot


class WorkspaceDriveTests(unittest.TestCase):
    open_workspace = base.WorkspaceCloseResizeTests.open_workspace
    pump = base.WorkspaceCloseResizeTests.pump
    close = base.WorkspaceCloseResizeTests.close

    def setUp(self):
        self.robot = FakeRobot()
        self.root, self.app, self.page, self.errors = self.open_workspace()

    def tearDown(self):
        if not self.app.closed:
            self.close(self.app)
        self.robot.close()

    def connect(self):
        page = self.page
        page.drive_url.delete(0, "end")
        page.drive_url.insert(0, self.robot.url)
        page.drive_connect()
        # the status line follows on the app's 80 ms tick
        self.pump(self.root, lambda: "Connected to" in page.drive_status.cget("text"), 10)

    def focus_pad(self):
        self.root.deiconify()
        self.root.lift()
        self.page.drive_pad.focus_force()
        deadline = time.monotonic() + 3
        while self.root.focus_get() is not self.page.drive_pad:
            if time.monotonic() > deadline:
                self.skipTest("ENVIRONMENT: the test window could not take keyboard focus")
            self.root.update()
            time.sleep(0.02)

    def test_panel_added_and_four_views_kept(self):
        self.assertEqual(set(self.page.views), set(workspace_page.VIEW_TITLES))
        self.assertEqual(self.page.drive.status()[0], "disconnected")
        self.assertEqual(self.page.drive_speed_label.cget("text"), "80")
        self.assertEqual(self.errors, [])

    def test_key_on_pad_drives_and_focus_loss_stops(self):
        self.connect()
        self.focus_pad()
        pad = self.page.drive_pad
        pad.event_generate("<KeyPress>", keysym="z")
        self.pump(self.root, lambda: "/forward" in self.robot.paths(), 5)
        self.page.drive_url.focus_force()  # leave the pad without releasing Z
        self.pump(self.root, lambda: self.robot.paths()[-1:] == ["/stop"], 5)
        self.assertIsNone(self.page.drive.held)
        self.assertEqual(self.errors, [])

    def test_release_and_space(self):
        self.connect()
        self.focus_pad()
        pad = self.page.drive_pad
        pad.event_generate("<KeyPress>", keysym="a")
        self.pump(self.root, lambda: "/rotateleft" in self.robot.paths(), 5)
        pad.event_generate("<KeyRelease>", keysym="a")
        self.pump(self.root, lambda: self.robot.paths()[-1:] == ["/stop"], 5)
        pad.event_generate("<KeyPress>", keysym="d")
        self.pump(self.root, lambda: "/right" in self.robot.paths(), 5)
        pad.event_generate("<KeyPress>", keysym="space")
        self.pump(self.root, lambda: self.robot.paths()[-1:] == ["/stop"], 5)
        self.assertEqual(self.errors, [])

    def test_typing_in_an_entry_does_not_drive(self):
        self.connect()
        entry = self.page.drive_url
        entry.focus_force()
        self.root.update()
        entry.event_generate("<KeyPress>", keysym="z")
        self.root.update()
        time.sleep(0.3)
        self.root.update()
        self.assertEqual(self.robot.paths(), ["/ping"])

    def test_closing_the_app_stops_the_robot(self):
        self.connect()
        self.focus_pad()
        self.page.drive_pad.event_generate("<KeyPress>", keysym="z")
        self.pump(self.root, lambda: "/forward" in self.robot.paths(), 5)
        drive = self.page.drive
        self.close(self.app)
        self.assertFalse(drive.thread.is_alive())
        self.assertEqual(self.robot.paths()[-1], "/stop")


if __name__ == "__main__":
    unittest.main()
