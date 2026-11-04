"""Workspace localisation adapter: page parity on real classroom recordings.

Run: python -m unittest test_workspace_localisation -v   (headless, no Tk window)

Reference: scans/classroom_final.json (16 visible scans). Independent target:
scan 3 of scans/classroom_session.json (a separate recording, source pose
ignored). Each adapter result must be bit-identical to the result of the page's
own code path for the same target, search and guess (runtimes excluded).
"""
from dataclasses import asdict
import json
import math
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np
from matplotlib.figure import Figure

import desktop_app
import localisation_common as common
import workspace_localisation as wl
from app_session import (SharedSession, build_grid, build_vectors, part2,
                         grid_localisation as loc, vector_localisation as vloc)
from workspace_inputs import WorkspaceInputs

REFERENCE = common.SCANS_DIR / "classroom_final.json"
SEPARATE = common.SCANS_DIR / "classroom_session.json"
GUESS = (-4.2, 3.4, -95.0)  # within ±1 m / ±30° of separate scan 3's pose


def comparable(result):
    """A result as exact text, without wall-clock timings."""
    data = asdict(result)
    data.pop("runtime_ms")
    data["details"] = {k: v for k, v in data["details"].items()
                       if "time" not in k and not k.endswith("_ms")}
    return json.dumps(data, sort_keys=True,
                      default=lambda o: o.tolist() if hasattr(o, "tolist") else repr(o))


class AdapterFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hashes = {p: part2.file_sha256(p) for p in common.SCANS_DIR.glob("*.json")}
        cls.model = SharedSession(REFERENCE)
        cls.scans = cls.model.scans
        cls.grid = build_grid(cls.scans)[0]
        cls.vmap = vloc.VectorMap.from_merged(build_vectors(cls.scans)[3])
        cls.target = next(t for t in wl.load_targets(SEPARATE) if t["id"] == 3)
        cls.points = cls.target["points_local"]

    @classmethod
    def tearDownClass(cls):
        for path, digest in cls.hashes.items():
            assert part2.file_sha256(path) == digest, f"{path} changed"

    def request(self, method, search, **kwargs):
        return wl.Request(method, search, self.points, "classroom_session.json · Scan 3",
                          guess_day1=GUESS if search == "local" else None,
                          reference_revision=self.model.revision, **kwargs)

    def reference(self, method):
        return {"grid": self.grid, "vector": self.vmap, "scans": self.scans}[method]

    def run_adapter(self, request):
        return wl.make_job(request, self.reference(request.method))()


class PageParityTests(AdapterFixture):
    """Adapter == the page's own code path (desktop_app.py / Code 04 viewer)."""

    def grid_viewer(self, guess=GUESS):
        viewer = loc.AutoAlignViewer(self.model.name, self.scans, self.model.measured,
                                     figure=Figure(), mode="new", full_grid=self.grid)
        viewer.set_new_scan(self.points, "target")
        viewer.set_initial_pose(common.pose_from_day1(guess))  # LocalisationPage.set_pose
        return viewer

    def test_grid_inputs_match_the_code04_viewer(self):
        for guess in (GUESS, (40.0, -30.0, 179.9), (0.1, 0.2, -180.0)):  # incl. clipped guesses
            viewer = self.grid_viewer(guess)
            self.assertTrue(np.array_equal(viewer.points, common.filter_ranges(self.points)))
            self.assertEqual(wl.grid_initial_pose(guess, self.grid), viewer.initial_pose(), guess)
            self.assertIs(viewer.grid_map, self.grid)

    def test_grid_global_and_local(self):
        viewer = self.grid_viewer()
        page = {"global": lambda: loc.localise_global(viewer.grid_map, viewer.points.copy()),
                "local": lambda: loc.localise_scan(viewer.grid_map, viewer.points.copy(),
                                                   viewer.initial_pose(), viewer.config)}
        for search, run in page.items():
            with self.subTest(search=search):
                outcome = self.run_adapter(self.request("grid", search))
                self.assertEqual(comparable(outcome.result), comparable(run()))

    def test_grid_range_filter_matches_the_viewer(self):
        # Scan 3 has no point outside 0.10-12 m, so add one too near and one too
        # far: the prepared inputs then differ from the raw ones, and must
        # still equal the viewer's. (localise_scan also filters internally.)
        points = np.vstack([self.points, [[0.05, 0.0], [12.5, 0.0]]])
        viewer = loc.AutoAlignViewer(self.model.name, self.scans, self.model.measured,
                                     figure=Figure(), mode="new", full_grid=self.grid)
        viewer.set_new_scan(points, "target with out-of-range points")
        viewer.set_initial_pose(common.pose_from_day1(GUESS))
        prepared = wl.grid_points(points)
        self.assertEqual(len(prepared), len(points) - 2)
        self.assertTrue(np.array_equal(prepared, viewer.points))
        expected = loc.localise_scan(viewer.grid_map, viewer.points.copy(),
                                     viewer.initial_pose(), viewer.config)
        request = wl.Request("grid", "local", points, "t", guess_day1=GUESS)
        self.assertEqual(comparable(self.run_adapter(request).result), comparable(expected))

    def test_vector_global_and_local(self):
        page = SimpleNamespace(vmap=self.vmap)
        for search in ("global", "local"):
            with self.subTest(search=search):
                initial = vloc.pose_from_day1(GUESS) if search == "local" else None
                expected = desktop_app.VectorLocalisationPage.compute(
                    page, self.points, initial, None, None)()
                outcome = self.run_adapter(self.request("vector", search))
                self.assertEqual(comparable(outcome.result), comparable(expected))

    def scan_page(self, single, reference_id=None):
        index = next((i for i, s in enumerate(self.scans) if s["id"] == reference_id), 0)
        page = SimpleNamespace(model=SimpleNamespace(scans=self.scans), single=lambda: single,
                               reference_choice=SimpleNamespace(current=lambda: index))
        page.references = lambda: desktop_app.ScanLocalisationPage.references(page)
        return page

    def test_scans_accumulated_global_and_local(self):
        page = self.scan_page(single=False)
        for search in ("global", "local"):
            with self.subTest(search=search):
                initial = vloc.pose_from_day1(GUESS) if search == "local" else None
                expected = desktop_app.ScanLocalisationPage.compute(
                    page, self.points, initial, None, None)()
                outcome = self.run_adapter(self.request("scans", search))
                self.assertEqual(comparable(outcome.result), comparable(expected))

    def test_scans_single_reference_local(self):
        reference_id = self.scans[1]["id"]
        page = self.scan_page(single=True, reference_id=reference_id)
        expected = desktop_app.ScanLocalisationPage.compute(
            page, self.points, vloc.pose_from_day1(GUESS), None, None)()
        outcome = self.run_adapter(self.request("scans", "local", scan_reference="single",
                                                reference_id=reference_id))
        self.assertEqual(comparable(outcome.result), comparable(expected))
        self.assertEqual(outcome.result.details["mode"], "single")


class TargetTests(AdapterFixture):
    def test_every_reference_scan_is_refused_as_self_matching(self):
        for scan in self.scans:
            message = wl.check_target(scan["points_local"], self.scans)
            self.assertIsNotNone(message, scan["id"])
            self.assertIn(str(scan["id"]), message)
            self.assertIn("self-matching", message)

    def test_independent_recording_is_accepted(self):
        self.assertIsNone(wl.check_target(self.points, self.scans))

    def test_invalid_targets_are_refused(self):
        for points in (np.empty((0, 2)), np.array([[1.0, np.nan]]), np.ones((5, 3)), np.ones(4)):
            self.assertIn("finite local [x, y]", wl.check_target(points, self.scans))

    def test_missing_and_invalid_recordings(self):
        with self.assertRaises(ValueError):
            wl.load_targets(common.SCANS_DIR / "does_not_exist.json")
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as folder:
            bad = Path(folder) / "bad.json"
            bad.write_text("not json", encoding="utf-8")
            with self.assertRaises(ValueError):
                wl.load_targets(bad)
        targets = wl.load_targets(SEPARATE)
        self.assertTrue(all("pose" not in t for t in targets))  # source poses dropped

    def test_request_validation(self):
        with self.assertRaises(ValueError):
            wl.Request("grid", "local", self.points, "t")          # local needs a guess
        with self.assertRaises(ValueError):
            wl.Request("icp", "global", self.points, "t")
        with self.assertRaises(ValueError):
            wl.Request("scans", "global", self.points, "t", scan_reference="single")
        request = self.request("vector", "global")
        self.assertFalse(request.points_local.flags.writeable)


class StalenessAndCancelTests(AdapterFixture):
    def test_signature_changes_with_every_input(self):
        base = self.request("vector", "local")
        changed = [self.request("grid", "local"), self.request("vector", "global"),
                   wl.Request("vector", "local", self.points, "x", guess_day1=(0, 0, 0),
                              reference_revision=self.model.revision),
                   wl.Request("vector", "local", self.points[:-1], "x", guess_day1=GUESS,
                              reference_revision=self.model.revision),
                   wl.Request("vector", "local", self.points, "x", guess_day1=GUESS,
                              reference_revision=self.model.revision + 1)]
        for other in changed:
            self.assertNotEqual(base.signature, other.signature)
        same_label = wl.Request("vector", "local", self.points, "renamed", guess_day1=GUESS,
                                reference_revision=self.model.revision)
        self.assertEqual(base.signature, same_label.signature)  # the label is not an input

    def test_cancellable_searches_stop_and_report_cancelled(self):
        for method in ("grid", "vector", "scans"):
            with self.subTest(method=method):
                outcome = wl.make_job(self.request(method, "global"), self.reference(method),
                                      should_stop=lambda: True)()
                self.assertEqual(outcome.status, "cancelled")
                self.assertIsNone(outcome.pose)

    def test_grid_local_is_not_cancellable(self):
        self.assertFalse(wl.cancellable("grid", "local"))
        for method, search in (("grid", "global"), ("vector", "local"), ("scans", "local")):
            self.assertTrue(wl.cancellable(method, search))
        with mock.patch.object(loc, "localise_scan", return_value="sentinel") as run:
            outcome = wl.make_job(self.request("grid", "local"), self.grid,
                                  should_stop=lambda: True)()
        self.assertEqual(outcome.result, "sentinel")  # ran to completion; no stop hook
        self.assertNotIn("should_stop", run.call_args.kwargs)


class OutcomeTests(unittest.TestCase):
    def outcome(self, status, pose=(1.0, 2.0, 0.5), details=None):
        result = common.LocalisationResult("m", status, "r", None, pose=pose, score=0.5,
                                           details=details or {})
        request = wl.Request("vector", "global", np.ones((3, 2)), "t")
        return wl.Outcome(request, result)

    def test_verdict_levels_match_desktop(self):
        for status in ("ok", "weak", "low_score", "ambiguous", "out_of_map", "inconsistent",
                       "at_search_limit", "failed", "cancelled"):
            outcome = self.outcome(status)
            self.assertEqual(outcome.verdict_level, desktop_app.verdict(outcome.result)[1], status)

    def test_rejected_results_have_no_pose(self):
        self.assertEqual(self.outcome("ok").pose, (1.0, 2.0, 0.5))
        self.assertTrue(self.outcome("ok").accepted)
        self.assertEqual(self.outcome("weak").pose, (1.0, 2.0, 0.5))
        self.assertFalse(self.outcome("weak").accepted)
        for status in ("low_score", "ambiguous", "inconsistent", "failed"):
            outcome = self.outcome(status)
            self.assertIsNone(outcome.pose, status)
            self.assertEqual(outcome.candidate_pose, (1.0, 2.0, 0.5))
        self.assertAlmostEqual(self.outcome("ok").pose_day1[2], math.degrees(0.5))

    def test_runner_up_only_for_ambiguous(self):
        details = {"runner_up_pose": [3.0, 4.0, 90.0]}  # Day 1 format, as the pages read it
        self.assertEqual(self.outcome("ambiguous", details=details).runner_up_pose,
                         common.pose_from_day1((3.0, 4.0, 90.0)))
        self.assertIsNone(self.outcome("low_score", details=details).runner_up_pose)


class EstimatePublishingTests(unittest.TestCase):
    def test_publish_and_clear(self):
        inputs = WorkspaceInputs()
        estimate = inputs.publish_estimate((1.0, 2.0, 0.5), "05 Vector", "ok", "Scan 3")
        self.assertEqual((estimate.x_m, estimate.y_m, estimate.heading_rad), (1.0, 2.0, 0.5))
        self.assertEqual((estimate.method, estimate.status), ("05 Vector", "ok"))
        self.assertIsNone(inputs.latest_capture)  # separate from captures
        self.assertIsNone(inputs.publish_estimate(None, "05 Vector", "low_score", "Scan 3"))
        with self.assertRaises(ValueError):
            inputs.publish_estimate((1.0, 2.0, 0.5), "05 Vector", "low_score", "Scan 3")

    def test_session_is_not_touched(self):
        model = SharedSession(REFERENCE)
        digest, revision = model.digest, model.revision
        model.workspace_inputs.publish_estimate((1.0, 2.0, 0.5), "04 Grid", "weak", "t")
        self.assertEqual((model.digest, model.revision), (digest, revision))
        self.assertFalse(model.dirty)


if __name__ == "__main__":
    unittest.main()
