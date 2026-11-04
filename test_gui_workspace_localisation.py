"""M1 Stage 3: one-shot Workspace localisation of an independent scan, in a real Tk loop."""
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest import mock

import numpy as np
from matplotlib.colors import to_hex

import app_session
import desktop_app
import workspace_page
from app_session import part2
import localisation_common as common
import test_gui_map_display as gui
import test_gui_workspace as base


RECORDING = common.SCANS_DIR / "classroom_session.json"   # independent of classroom_final
GUESS = (-4.4, 3.5, -100.0)                                # near scan 3's pose, Day 1 units
PROJECT = Path(__file__).resolve().parent


class WorkspaceLocalisationTests(unittest.TestCase):
    open_workspace = base.WorkspaceCloseResizeTests.open_workspace
    pump = base.WorkspaceCloseResizeTests.pump
    close = base.WorkspaceCloseResizeTests.close

    def setUp(self):
        self.hashes = {p: part2.file_sha256(p) for p in common.SCANS_DIR.glob("*.json")}
        self.measured = set(PROJECT.rglob("*_measured_poses.json"))
        self.root, self.app, self.page, self.errors = self.open_workspace()
        self.digest = app_session.session_digest(self.app.model.session)
        self.revision = self.app.model.revision

    def tearDown(self):
        if not self.app.closed:
            self.pump(self.root, lambda: not any(p.busy for p in self.app.pages.values()), 120)
            self.close(self.app)
        for path, digest in self.hashes.items():
            self.assertEqual(part2.file_sha256(path), digest)
        self.assertEqual(set(PROJECT.rglob("*_measured_poses.json")), self.measured)

    # -- helpers ----------------------------------------------------------------
    def idle(self, page=None, timeout=120):
        page = page or self.page
        self.pump(self.root, lambda: not page.busy, timeout)

    def target_index(self, page, scan_id):
        return next(i for i, t in enumerate(page.targets) if t["id"] == scan_id)

    def set_controls(self, method, search="global", scan_reference="accumulated", guess=GUESS):
        page = self.page
        page.method_choice.current(tuple(workspace_page.wl.METHODS).index(method))
        page.search_choice.current(tuple(workspace_page.wl.SEARCHES).index(search))
        page.reference_mode.current(tuple(workspace_page.wl.SCAN_REFERENCES).index(scan_reference))
        page.inputs_changed()
        if search == "local":
            page.set_guess(guess)

    def load_scan(self, scan_id, path=RECORDING):
        page = self.page
        page.load_recording(path)  # False for a refused (self-matching) first scan
        self.assertEqual(page.target_source, Path(path).name)
        page.recorded_choice.current(self.target_index(page, scan_id))
        return page.choose_recorded_target()

    def workspace_localise(self, method, search="global", scan_id=3, guess=GUESS):
        self.app.show_page("Workspace")
        self.page = self.app.pages["Workspace"]
        self.load_scan(scan_id)
        self.set_controls(method, search, guess=guess)
        self.page.localise()
        self.idle()
        self.assertIsNotNone(self.page.outcome, self.page.info.cget("text"))
        return self.page.outcome

    def page_result(self, method, search="global", scan_id=3, guess=GUESS):
        """The same target on page 04, 05 or 06, driven through its own controls."""
        name = {"grid": "Grid Localisation", "vector": "Vector Localisation",
                "scans": "Scan Localisation"}[method]
        self.app.show_page(name)
        page = self.app.pages[name]
        ready = (lambda: page.viewer is not None) if method == "grid" else \
            (lambda: page.reference_ready)
        self.pump(self.root, lambda: ready() and not page.busy, 120)
        if method == "scans":
            page.reference_mode.current(1)  # accumulated
            page.change_reference()
        self.assertTrue(page.load_recorded(RECORDING))
        page.target_choice.current(self.target_index(page, scan_id))
        self.assertTrue(page.choose_target())
        page.search.current(0 if search == "global" else 1)
        page.change_search()
        if search == "local":
            page.set_pose(guess)
        (page.align if method == "grid" else page.localise)()
        self.idle(page)
        result = page.viewer.result if method == "grid" else page.result
        self.assertIsNotNone(result)
        return result

    def assert_session_untouched(self):
        self.assertEqual(app_session.session_digest(self.app.model.session), self.digest)
        self.assertEqual(self.app.model.revision, self.revision)

    def assert_no_pose_drawn(self):
        for points, marker, heading, *_ in self.page.estimate_artists.values():
            self.assertEqual(len(points.get_offsets()), 0)
            self.assertEqual(len(marker.get_xdata()), 0)
            self.assertEqual(len(heading.get_xdata()), 0)

    # -- tests --------------------------------------------------------------------
    def test_workspace_matches_pages_bit_for_bit(self):
        for method in ("grid", "vector", "scans"):
            for search in ("global", "local"):
                with self.subTest(method=method, search=search):
                    expected = self.page_result(method, search)
                    outcome = self.workspace_localise(method, search)
                    self.assertEqual(outcome.status, expected.status)
                    self.assertEqual(outcome.result.pose, expected.pose)
                    self.assertEqual(outcome.result.score, expected.score)
                    self.assertEqual(outcome.result.runner_up_score, expected.runner_up_score)
        self.assert_session_untouched()
        self.assertEqual(self.errors, [])

    def test_ok_weak_and_rejected_overlays(self):
        page = self.page
        # ok: Code 05 global on scan 3
        outcome = self.workspace_localise("vector")
        self.assertEqual(outcome.status, "ok")
        pose = outcome.pose
        aligned = common.transform_points(outcome.request.points_local, pose)
        for key, (points, marker, heading, guess, _) in page.estimate_artists.items():
            self.assertEqual((marker.get_xdata()[0], marker.get_ydata()[0]), (pose[0], pose[1]), key)
            np.testing.assert_array_equal(points.get_offsets(), aligned)
            self.assertNotEqual(marker.get_markerfacecolor(), "none")
            self.assertEqual(heading.get_linestyle(), "-")
            self.assertEqual(len(guess.get_xdata()), 0)  # global search: no guess marker
        np.testing.assert_array_equal(page.current_points.get_offsets(), outcome.request.points_local)
        self.assertEqual(page.source.get(), page.SOURCES[2])
        self.assertTrue(page.views["current"].caption.cget("text").startswith("Local · "))
        self.assertEqual(page.verdict_chip.cget("text"), "Localised · ok")
        estimate = self.app.model.workspace_inputs.latest_estimate
        self.assertEqual((estimate.x_m, estimate.y_m, estimate.heading_rad), tuple(pose))
        self.assertEqual(estimate.status, "ok")
        for style in workspace_page.PLOT_STYLES:
            page.plot_style.set(style)
            page.set_plot_style()
            colour = workspace_page.PLOT_STYLES[style]["estimate"]
            for points, marker, heading, *_ in page.estimate_artists.values():
                self.assertEqual(to_hex(marker.get_color()), to_hex(colour))
                self.assertEqual(to_hex(points.get_facecolor()[0]), to_hex(colour))
        # weak: Code 06 accumulated global on scan 3 (Gate 0 baseline)
        outcome = self.workspace_localise("scans")
        self.assertEqual(outcome.status, "weak")
        for points, marker, heading, *_ in page.estimate_artists.values():
            self.assertEqual(marker.get_markerfacecolor(), "none")
            self.assertEqual(heading.get_linestyle(), "--")
            self.assertEqual((marker.get_xdata()[0], marker.get_ydata()[0]),
                             (outcome.pose[0], outcome.pose[1]))
        self.assertEqual(page.verdict_chip.cget("text"), "Plausible, not confirmed · weak")
        # rejected: scan 1 on Code 05 global is low_score
        outcome = self.workspace_localise("vector", scan_id=1)
        self.assertNotIn(outcome.status, ("ok", "weak"))
        self.assert_no_pose_drawn()
        self.assertIsNone(self.app.model.workspace_inputs.latest_estimate)
        self.assertEqual(page.verdict_chip.cget("text"), f"Not localised · {outcome.status}")
        # local search: the guess marker is drawn at the guess on every global view
        self.set_controls("vector", "local")
        guess = common.pose_from_day1(GUESS)
        for *_, marker, heading in page.estimate_artists.values():
            self.assertAlmostEqual(marker.get_xdata()[0], guess[0])
            self.assertAlmostEqual(marker.get_ydata()[0], guess[1])
        page.clear_result()
        self.assert_no_pose_drawn()
        self.assertEqual(page.verdict_chip.cget("text"), "No result yet")
        self.assert_session_untouched()
        self.assertEqual(self.errors, [])

    def test_refusals_and_bad_files(self):
        page = self.page
        self.assertFalse(page.localise())
        self.assertIn("Capture a scan or load", page.info.cget("text"))
        self.assertFalse(page.load_recording(common.SCANS_DIR / "does_not_exist.json"))
        self.assertIn("Could not load", page.info.cget("text"))
        with tempfile.TemporaryDirectory() as folder:
            broken = Path(folder) / "broken.json"
            broken.write_text("{not json", encoding="utf-8")
            self.assertFalse(page.load_recording(broken))
            self.assertIn("Could not load", page.info.cget("text"))
        self.assertFalse(self.load_scan(1, gui.SESSION))  # its own scans: self-matching
        self.assertIn("self-matching", page.info.cget("text"))
        self.assertFalse(page.localise())
        self.assertIn("self-matching", page.info.cget("text"))
        self.assertFalse(page.busy)
        self.assertIsNone(page.outcome)
        self.assert_session_untouched()
        self.assertEqual(self.errors, [])

    def test_stale_results_are_discarded(self):
        page = self.page
        self.load_scan(3)
        for change in ("method", "guess", "target", "clear"):
            with self.subTest(change=change):
                self.set_controls("grid" if change == "guess" else "scans",
                                  "local" if change == "guess" else "global")
                self.assertTrue(page.localise())
                if change == "method":
                    page.method_choice.current(1)
                    page.inputs_changed()
                elif change == "guess":  # Code 04 local: not cancellable, only made stale
                    self.assertIn("disabled", page.cancel_button.state())
                    page.set_guess((-4.0, 3.0, -90.0))
                elif change == "target":
                    page.recorded_choice.current(self.target_index(page, 2))
                    page.choose_recorded_target()
                else:
                    page.clear_result()
                self.idle()
                self.assertIsNone(page.outcome)
                self.assert_no_pose_drawn()
                if change != "clear":
                    self.assertIn("changed during the search", page.info.cget("text"))
                self.load_scan(3)
        # a session change (new revision) during the search discards it too
        self.set_controls("vector")
        page.localise()
        self.idle()
        page.clear_result()
        self.assertTrue(page.localise())
        self.app.model.session["scans"][-1]["visible"] = False
        self.app.model.refresh()
        self.assertNotEqual(self.app.model.revision, self.revision)
        self.idle()
        self.assertIsNone(page.outcome)
        self.assertIn("changed during the search", page.info.cget("text"))
        self.assertEqual(self.errors, [])

    def test_busy_cancel_and_capture_interlock(self):
        page = self.page
        self.load_scan(3)
        self.set_controls("scans")
        self.assertNotIn("disabled", page.cancel_button.state())
        self.assertTrue(page.localise())
        self.assertFalse(page.localise())  # a second press while busy is ignored
        self.assertIn("already running", page.info.cget("text"))
        page.cancel_localisation()
        self.idle()
        self.assertIsNone(page.outcome)
        self.assertIn("cancelled", page.info.cget("text"))
        self.assert_no_pose_drawn()
        # Localise during a capture is refused; the capture is a fake, no port is opened.
        release = threading.Event()
        points = page.targets[self.target_index(page, 3)]["points_local"]

        def fake_capture(adapter, port, baud, *args, **kwargs):
            release.wait(30)
            return points.copy()
        with mock.patch.object(desktop_app.CaptureAdapter, "capture", fake_capture):
            page.start_capture()
            self.assertTrue(page.busy)
            self.assertFalse(page.localise())
            self.assertIn("capture is running", page.info.cget("text"))
            release.set()
            self.idle()
        # the fresh capture becomes the "Latest capture" target
        page.target_choice.current(0)
        page.target_changed()
        self.set_controls("vector")
        page.localise()  # the first 05 press prepares the shared VectorMap, then runs
        self.idle()
        self.assertEqual(page.outcome.status, "ok")
        self.assertEqual(page.outcome.request.target_label, "Fresh LD06 capture")
        self.assert_session_untouched()
        self.assertEqual(self.errors, [])

    def test_navigation_resize_popdown_and_poll_during_run(self):
        page = self.page
        self.load_scan(3)
        self.set_controls("scans")
        self.assertTrue(page.localise())
        self.app.show_page("Vector Localisation")
        self.root.update()
        self.app.show_page("Workspace")
        page.toggle_maximise("grid")
        self.root.geometry("1300x820")
        self.root.update()
        page.restore()
        # a failing tick must not stop result delivery
        with mock.patch.object(workspace_page.WorkspacePage, "tick_localisation",
                               side_effect=RuntimeError("injected tick failure")):
            self.idle()
        self.assertEqual(page.outcome.status, "weak")
        combo = page.method_choice
        combo.tk.call("ttk::combobox::Post", combo)
        deadline = time.monotonic() + 1.0  # about 12 polls with the popdown open
        self.pump(self.root, lambda: time.monotonic() > deadline)
        combo.tk.call("ttk::combobox::Unpost", combo)
        self.root.update()
        self.assertEqual(self.errors, [])
        # closing while a search runs raises nothing
        page.clear_result()
        self.assertTrue(page.localise())
        self.close(self.app)
        self.assertTrue(self.app.closed)
        self.assertTrue(page.stop.is_set())
        self.assertEqual(self.errors, [])


if __name__ == "__main__":
    unittest.main()
