"""Headless live localisation tracker on simulated scan sequences (no GUI, no hardware).

Errors are measured against SIMULATED ground truth; they are not real-world accuracy.
Run: .\\.venv\\Scripts\\python.exe -m unittest test_autonomy_live_localisation -v
"""
import math
import unittest

import numpy as np

import localisation_common as common
from autonomy import protocol
from autonomy.fake_http_robot import ray_scan
from autonomy.live_localisation import LOST_AFTER, LiveTracker, evaluate, loc, pose_error
from autonomy.localisation_sim import WALLS, reference_map, route_poses, run, scans_along

# Code 04's own tolerance on its fixture map: the walls lie on 10 cm cell
# borders, so Part 2 draws them up to half a cell outwards (a known bias).
MAP_BIAS_TOLERANCE_M = loc.MAP_BIAS_TOLERANCE_M
REFERENCE = None


def reference():
    global REFERENCE
    if REFERENCE is None:
        REFERENCE = reference_map()
    return REFERENCE


class ConventionTests(unittest.TestCase):
    def test_robot_scan_angles_match_the_sensor_frame(self):
        """The robot's CCW scan (fake robot / firmware) -> points_to_sensor_xy
        gives the same points as Part 1's ray caster at the same pose."""
        pose = (2.3, 3.2, math.radians(30.0))
        robot = np.array(protocol.points_to_sensor_xy(ray_scan(pose, walls=WALLS)))
        part1 = common.simulate_scan(WALLS, pose, noise_mm=0.0)
        self.assertGreater(len(robot), 400)
        distances = np.min(np.linalg.norm(robot[:, None, :] - part1[None, :, :], axis=2), axis=1)
        self.assertLess(float(np.max(distances)), 0.002)


class TrackingTests(unittest.TestCase):
    def test_unknown_start_then_tracking_along_a_route(self):
        poses = route_poses()[:60]
        tracker = LiveTracker(reference())
        updates = run(tracker, scans_along(poses))
        summary = evaluate(updates, poses)
        self.assertEqual(updates[0].search, "global")
        self.assertTrue(all(u.search == "local" for u in updates[1:]))
        self.assertEqual(summary["accepted"], len(poses))
        self.assertLessEqual(summary["position_error_m"]["max"], MAP_BIAS_TOLERANCE_M)
        self.assertLessEqual(summary["heading_error_deg"]["max"], 1.0)
        self.assertEqual(len(tracker.trajectory()), len(poses))

    def test_control_map_isolates_the_tracking_error(self):
        """Code 04's CONTROL map (shifted 5 cm, walls at cell centres) removes the
        map's half-cell wall bias, so what remains is the tracking itself."""
        scans = loc.fixture_mapping_scans()
        spec = loc.session_spec(scans)
        control = loc.build_grid_map(scans, dict(spec, x_min=spec["x_min"] - 0.05,
                                                 y_min=spec["y_min"] - 0.05))[0]
        poses = route_poses()[:20]
        summary = evaluate(run(LiveTracker(control), scans_along(poses)), poses)
        self.assertEqual(summary["accepted"], len(poses))
        self.assertLessEqual(summary["position_error_m"]["max"], 0.015)   # Code 04's control bound
        self.assertLessEqual(summary["heading_error_deg"]["max"], 0.5)

    def test_known_start_skips_the_global_search(self):
        poses = route_poses()[:5]
        tracker = LiveTracker(reference())
        tracker.reset(initial_pose=poses[0])
        updates = run(tracker, scans_along(poses))
        self.assertTrue(all(u.search == "local" and u.accepted for u in updates))

    def test_reference_map_is_never_modified(self):
        grid = reference()
        before = (grid.state.copy(), grid.probability.copy())
        run(LiveTracker(grid), scans_along(route_poses()[:3]))
        np.testing.assert_array_equal(grid.state, before[0])
        np.testing.assert_array_equal(grid.probability, before[1])


class FailureTests(unittest.TestCase):
    def setUp(self):
        self.poses = route_poses()[:3]
        self.tracker = LiveTracker(reference())
        self.tracker.reset(initial_pose=self.poses[0])
        run(self.tracker, scans_along(self.poses))
        self.last_pose = self.tracker.pose

    def test_a_bad_scan_gives_no_pose_and_keeps_the_last_one(self):
        update = self.tracker.update(np.zeros((5, 2)), seq=10)
        self.assertFalse(update.accepted)
        self.assertIsNone(update.pose)
        self.assertEqual(update.status, "insufficient_points")
        self.assertEqual(self.tracker.pose, self.last_pose)
        self.assertEqual(self.tracker.state, "tracking")       # one failure is not "lost"

    def test_stale_scan_is_skipped(self):
        update = self.tracker.update(scans_along(self.poses[-1:])[0], seq=2)
        self.assertEqual((update.search, update.status, update.accepted), ("skipped", "stale", False))
        self.assertEqual(self.tracker.pose, self.last_pose)

    def test_kidnapped_robot_is_lost_then_recovered_by_global_search(self):
        kidnapped = (6.0, 4.3, math.radians(-120.0))   # far outside the local window
        scans = scans_along([kidnapped] * (LOST_AFTER + 1), seed=50)
        local = [self.tracker.update(points, seq=10 + i)
                 for i, points in enumerate(scans[:LOST_AFTER])]
        self.assertTrue(all(u.search == "local" for u in local))
        self.assertTrue(all(not u.accepted and u.pose is None for u in local),
                        [u.status for u in local])
        self.assertEqual(local[-1].state, "lost")
        self.assertEqual(self.tracker.pose, self.last_pose)    # unchanged while lost
        recovery = self.tracker.update(scans[LOST_AFTER], seq=10 + LOST_AFTER)
        self.assertEqual(recovery.search, "global")
        self.assertTrue(recovery.accepted, recovery.status)
        position, heading = pose_error(recovery.pose, kidnapped)
        self.assertLess(position, MAP_BIAS_TOLERANCE_M)
        self.assertLess(heading, 2.0)
        self.assertEqual(self.tracker.state, "tracking")

    def test_scan_from_outside_the_map_gives_no_pose(self):
        outside = scans_along([(1.0, 1.0, 0.0)], seed=70)[0] + np.array([25.0, 25.0])
        update = self.tracker.update(outside, seq=10)
        self.assertFalse(update.accepted)
        self.assertIsNone(update.pose)


if __name__ == "__main__":
    unittest.main()
