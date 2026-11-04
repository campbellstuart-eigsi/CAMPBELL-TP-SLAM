"""Headless integration regressions. Run: python -m unittest test_application -v"""
import hashlib
import importlib
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from app_session import (SharedSession, CaptureAdapter, build_grid, build_vectors,
                         part1, part2, part3, grid_localisation as loc)
import localisation_common as common

REFERENCE = common.SCANS_DIR / "classroom_final.json"
SEPARATE = common.SCANS_DIR / "classroom_session.json"


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.model = SharedSession(REFERENCE)
        self.before = REFERENCE.read_bytes()

    def tearDown(self):
        self.assertEqual(REFERENCE.read_bytes(), self.before)

    def test_adapter_preserves_grid_and_vector_fingerprints(self):
        grid, log = build_grid(self.model.scans)
        self.assertEqual(hashlib.sha256(log["log_odds"].tobytes()).hexdigest(), part2.VERIFIED_LOG_ODDS_SHA256)
        self.assertEqual(len(build_vectors(self.model.scans)[3]), 74)
        original = common.load_session_scans(REFERENCE)
        for actual, expected in zip(self.model.scans, original):
            np.testing.assert_array_equal(actual["points_local"], expected["points_local"])
            np.testing.assert_array_equal(actual["pose_rad"], expected["pose_rad"])

    def test_revision_cache_and_selection(self):
        self.model.cache["marker"] = object()
        self.model.session["selected_id"] = 2
        self.assertFalse(self.model.refresh())
        self.assertFalse(self.model.dirty)
        self.assertIn("marker", self.model.cache)
        points = self.model.scans[1]["points_local"].copy()
        part1.set_scan_pose(self.model.session, 2, (1, 2, 3))
        self.assertTrue(self.model.refresh())
        self.assertTrue(self.model.dirty)
        self.assertEqual(self.model.revision, 1)
        self.assertEqual(self.model.cache, {})
        np.testing.assert_array_equal(points, self.model.scans[1]["points_local"])

    def test_save_copy_protects_source(self):
        with self.assertRaises(ValueError):
            self.model.save_copy(REFERENCE)
        part1.set_scan_pose(self.model.session, 2, (1, 2, 3))
        self.model.refresh()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "edited.json"
            self.model.save_copy(path)
            self.assertEqual(self.model.path, path.resolve())
            with self.assertRaises(ValueError):
                self.model.save_copy(REFERENCE)
            restored = SharedSession(path)
            self.assertEqual(restored.scans[1]["pose"], (1, 2, 3))
            for a, b in zip(restored.scans, self.model.scans):
                np.testing.assert_array_equal(a["points_local"], b["points_local"])
        self.assertFalse(self.model.dirty)

    def test_late_binary_map_cannot_replace_an_edited_reference(self):
        from desktop_app import GridPage
        # A map worker started at revision 0; an edit arrives before completion.
        # Exercise its delivery callback without requiring a desktop.
        page = GridPage.__new__(GridPage)
        page.model, page.revision = self.model, self.model.revision
        page.set_info = mock.Mock()
        page.draw_map = mock.Mock()
        old_binary = np.zeros((3, 3), dtype=np.uint8)
        part1.set_scan_pose(self.model.session, 2, (1, 2, 3))
        self.model.refresh()
        page.binary_ready(old_binary)
        self.assertNotIn("binary", self.model.cache)
        page.draw_map.assert_not_called()
        self.assertIn("changed", page.set_info.call_args.args[0])

    def test_empty_session_and_hidden_scans(self):
        self.assertEqual(SharedSession().scans, [])
        with self.assertRaisesRegex(ValueError, "No visible scans"):
            build_grid([])
        part1.set_scan_visible(self.model.session, 2, False)
        self.model.refresh()
        self.assertNotIn(2, [s["id"] for s in self.model.scans])


class NewScanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = SharedSession(REFERENCE)
        cls.grid = build_grid(cls.model.scans)[0]

    def setUp(self):
        self.before = (self.grid.state.copy(), self.grid.probability.copy(), REFERENCE.read_bytes())
        self.viewer = loc.AutoAlignViewer(self.model.name, self.model.scans,
                                          mode="new", full_grid=self.grid)

    def tearDown(self):
        np.testing.assert_array_equal(self.grid.state, self.before[0])
        np.testing.assert_array_equal(self.grid.probability, self.before[1])
        self.assertEqual(REFERENCE.read_bytes(), self.before[2])
        plt.close("all")

    def test_separate_real_scan_ignores_source_pose_and_localises(self):
        target = loc.load_recorded_targets(SEPARATE)[0]
        self.assertNotIn("pose", target)
        points = target["points_local"].copy()
        self.viewer.set_initial_pose((0.1, 0.2, 0.03))
        self.viewer.set_new_scan(points, target["name"])
        self.assertEqual(self.viewer.initial_pose(), (0.1, 0.2, common.wrap_angle(0.03)))
        result = self.viewer.auto_align()
        self.assertIsNotNone(result.pose)
        np.testing.assert_array_equal(points, self.viewer.external_scan["points_local"])
        self.assertIs(self.grid, self.viewer.grid_map)
        text = self.viewer.info.get_text()
        for value in ("estimated", "score", "time", "physical accuracy"):
            self.assertIn(value, text)

    def test_duplicate_and_reordered_measurements_refused(self):
        for points in (self.model.scans[0]["points_local"], self.model.scans[0]["points_local"][::-1]):
            with self.assertRaisesRegex(ValueError, "Evaluate Recorded Scans"):
                self.viewer.set_new_scan(points, "different ID and name")
        self.assertIsNone(self.viewer.external_scan)

    def test_new_scan_stays_separate_and_evaluation_survives(self):
        points = loc.load_recorded_targets(SEPARATE)[1]["points_local"]
        self.viewer.set_new_scan(points)
        self.viewer.set_mode("evaluation")
        self.assertIn("excluded", self.viewer.map_description)
        self.assertEqual(len(self.viewer.scans), 16)
        self.viewer.set_mode("new")
        np.testing.assert_array_equal(self.viewer.external_scan["points_local"], points)
        self.assertIs(self.viewer.grid_map, self.grid)

    def test_poor_coverage_and_outside_map_not_successful(self):
        self.viewer.set_new_scan(np.column_stack((np.linspace(1, 7, 150), np.zeros(150))))
        self.assertEqual(self.viewer.auto_align().status, "insufficient_coverage")
        points = loc.load_recorded_targets(SEPARATE)[0]["points_local"] + (100, 100)
        self.viewer.set_new_scan(points)
        self.assertNotEqual(self.viewer.auto_align().status, "ok")

    def test_invalid_recordings_and_points(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "bad.json"
            path.write_text("not json")
            with self.assertRaisesRegex(part2.SessionError, "not valid JSON"):
                loc.load_recorded_targets(path)
        with self.assertRaises(ValueError):
            self.viewer.set_new_scan([[float("nan"), 1]])


class SeparateRecordingTests(unittest.TestCase):
    """Real LD06 recordings that are NOT in the reference map (stabilisation review)."""
    @classmethod
    def setUpClass(cls):
        cls.grid = build_grid(SharedSession(REFERENCE).scans)[0]
        cls.targets = loc.load_recorded_targets(SEPARATE)

    def test_open_view_scan_localises_from_a_hand_placed_guess(self):
        # Scan 3 (median range 6 m). Its pose was found by a whole-map
        # diagnostic search; the app's ±1 m / ±30° search must recover it.
        expected = common.pose_from_day1((-4.49, 3.54, -102.6))
        guess = common.pose_from_day1((-4.2, 3.2, -85.0))
        result = loc.localise_scan(self.grid, self.targets[2]["points_local"], guess)
        self.assertEqual(result.status, "ok", result.reason)
        distance, angle = common.pose_error(result.pose, expected)
        self.assertLess(distance, 0.03)
        self.assertLess(angle, 0.5)

    def test_wrong_heading_or_occluded_scan_is_not_reported_ok(self):
        scan3 = self.targets[2]["points_local"]
        wrong_heading = common.pose_from_day1((-4.49, 3.54, -162.6))  # 60° off
        self.assertNotEqual(loc.localise_scan(self.grid, scan3, wrong_heading).status, "ok")
        # Scan 1 sees mostly nearby obstacles (median range < 1 m).
        result = loc.localise_scan(self.grid, self.targets[0]["points_local"], (0.0, 0.0, 0.0))
        self.assertEqual(result.status, "low_score")


class NewDayRecordingTests(unittest.TestCase):
    """Real scans of the same classroom captured on another day."""
    PATH = common.SCANS_DIR / "Untitled session_edited.json"

    @classmethod
    def setUpClass(cls):
        if not cls.PATH.exists():
            raise unittest.SkipTest("new-day session absent")
        cls.before = cls.PATH.read_bytes()
        cls.session = part1.load_session(cls.PATH)[0]
        cls.scan6 = np.column_stack(part1.scan_to_local(part1.find_scan(cls.session, 6)))

    @classmethod
    def tearDownClass(cls):
        assert cls.PATH.read_bytes() == cls.before

    def test_code01_scan6_is_check_visually_not_failed(self):
        # Sensor above the desks: one-sided see-through conflicts only. The
        # pose was confirmed independently on the classroom_final grid.
        references = [(np.column_stack(part1.scan_to_local(s)), tuple(s["pose"]))
                      for s in part1.alignment_reference_scans(self.session, 6)]
        result = part1.align_scan(self.scan6, references)
        self.assertEqual(result["status"], "weak", result["reason"])
        self.assertIn("ONE-SIDED", result["reason"])
        x, y, theta = result["pose"]
        self.assertLess(np.hypot(x - 4.045, y - 3.766), 0.05)
        self.assertLess(part1.angle_difference_deg(theta, 93.5), 0.5)

    def test_code04_finds_scan6_globally_on_the_older_map(self):
        grid = build_grid(SharedSession(REFERENCE).scans)[0]
        result = loc.localise_global(grid, self.scan6)
        self.assertEqual(result.status, "ok", result.reason)
        expected = common.pose_from_day1((-4.48, 3.53, -173.0))
        distance, angle = common.pose_error(result.pose, expected)
        self.assertLess(distance, 0.05)
        self.assertLess(angle, 0.5)


class SharedBuildTests(unittest.TestCase):
    """Two pages needing the same map share ONE background computation."""
    def make_app(self):
        from desktop_app import DesktopApp
        app = DesktopApp.__new__(DesktopApp)
        app.model = SharedSession(REFERENCE)
        app.building, app.jobs = {}, {}
        app.submit = mock.Mock()
        return app

    def make_page(self):
        return mock.Mock(alive=True, busy=False)

    def test_concurrent_requests_share_one_build_and_release_it(self):
        app = self.make_app()
        pages = [self.make_page(), self.make_page()]
        callbacks = [mock.Mock(), mock.Mock()]
        for page, callback in zip(pages, callbacks):
            app.derived(page, "grid", build_grid, callback)
        app.submit.assert_called_once()
        job, label, work, ready = app.submit.call_args.args
        self.assertIn("grid", label)
        for page in pages:
            page.set_busy.assert_called_with(True, label)
        data = work()
        job.set_busy(False)            # what poll() does on completion
        ready(data)
        for callback in callbacks:
            callback.assert_called_once_with(data)
        self.assertEqual(app.building, {})
        self.assertIs(app.model.cache["grid"], data)
        later = mock.Mock()
        app.derived(self.make_page(), "grid", build_grid, later)  # cached: no new job
        later.assert_called_once_with(data)
        app.submit.assert_called_once()

    def test_failure_in_one_page_still_delivers_to_the_other(self):
        app = self.make_app()
        pages = [self.make_page(), self.make_page()]
        bad, good = mock.Mock(side_effect=RuntimeError("draw failed")), mock.Mock()
        app.derived(pages[0], "grid", build_grid, bad)
        app.derived(pages[1], "grid", build_grid, good)
        job, _, work, ready = app.submit.call_args.args
        job.set_busy(False)
        ready(work())
        good.assert_called_once()
        self.assertIn("draw failed", pages[0].set_info.call_args.args[0])

    def test_closed_owner_does_not_block_the_waiting_page(self):
        app = self.make_app()
        first, second = self.make_page(), self.make_page()
        callback = mock.Mock()
        app.derived(first, "vectors", build_vectors, mock.Mock())
        app.derived(second, "vectors", build_vectors, callback)
        first.alive = False            # e.g. the first page was closed
        job, _, work, ready = app.submit.call_args.args
        self.assertTrue(job.alive)
        job.set_busy(False)
        ready(work())
        self.assertEqual(len(callback.call_args.args[0][3]), 74)


class CaptureTests(unittest.TestCase):
    def test_reuses_code01_conversion_and_closes_port(self):
        points = part1.make_test_points(60)
        connection = part1.FakeSerial([part1.make_teleplot_line(points)])
        connection.close = mock.Mock()
        with mock.patch.object(part1, "open_serial_readonly", return_value=connection):
            actual = CaptureAdapter().capture("TEST", 115200)
        expected = np.column_stack(part1.teleplot_to_sensor_frame(
            np.array([p[0] for p in points]), np.array([p[1] for p in points])))
        np.testing.assert_array_equal(actual, expected)
        connection.close.assert_called_once()

    def test_csv_stream_gives_same_points_and_shows_format(self):
        points = part1.make_test_points(60)
        connection = part1.FakeSerial([b"\r\n" + part1.make_csv_scan(points)])
        connection.close = mock.Mock()
        adapter = CaptureAdapter()
        with mock.patch.object(part1, "open_serial_readonly", return_value=connection):
            actual = adapter.capture("TEST", 115200)
        expected = np.column_stack(part1.teleplot_to_sensor_frame(
            np.array([p[0] for p in points], dtype=float), np.array([p[1] for p in points], dtype=float)))
        np.testing.assert_array_equal(actual, expected)
        self.assertEqual(adapter.last_format, part1.FORMAT_CSV_XY)
        self.assertIn("6 columns", adapter.state)
        connection.close.assert_called_once()

    def test_missing_port_has_actionable_error(self):
        with mock.patch.object(part1, "open_serial_readonly", side_effect=part1.serial.SerialException("missing")):
            with self.assertRaisesRegex(RuntimeError, "Load Recorded Scan"):
                CaptureAdapter().capture("COM_MISSING", 115200)

    def test_timeout_and_cancellation_release_port(self):
        for cancel in (False, True):
            adapter = CaptureAdapter()
            connection = part1.FakeSerial([])
            connection.close = mock.Mock()
            with mock.patch.object(part1, "open_serial_readonly", return_value=connection):
                if cancel:
                    timer = threading.Timer(0.03, adapter.cancel)
                    timer.start()
                start = time.monotonic()
                with self.assertRaises(RuntimeError):
                    adapter.capture("TEST", 115200, timeout_s=0.08)
                self.assertLess(time.monotonic() - start, 0.5)
            connection.close.assert_called_once()
            self.assertFalse(adapter.active)


class PickerTests(unittest.TestCase):
    def test_default_cli_picker_cancel_and_explicit_paths(self):
        for module in (part2, part3):
            old = module.SESSION_FILE
            try:
                with mock.patch.object(module, "choose_scan_file", return_value=None) as picker, \
                        mock.patch.object(module, "main") as main:
                    module.cli([])
                    picker.assert_called_once()
                    main.assert_not_called()
                with mock.patch.object(module, "choose_scan_file") as picker, \
                        mock.patch.object(module, "main") as main:
                    module.cli(["--non-interactive", "--session", str(REFERENCE)])
                    picker.assert_not_called()
                    main.assert_called_once()
            finally:
                module.SESSION_FILE = old

    def test_import_entry_point_opens_no_window(self):
        with mock.patch("tkinter.Tk", side_effect=AssertionError("unexpected Tk root")):
            importlib.import_module("00_main_app")


if __name__ == "__main__":
    unittest.main()
