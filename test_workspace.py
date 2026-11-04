"""Data-boundary regressions; no Tk window or physical hardware."""
import unittest
from unittest import mock

import numpy as np

from app_session import SharedSession
from workspace_inputs import ScanFrame


class WorkspaceInputTests(unittest.TestCase):
    def test_capture_is_local_and_does_not_invalidate_maps_or_invent_pose(self):
        model = SharedSession()
        sentinel = model.cache["grid"] = object()
        points = np.array([[1., 2.], [3., 4.]])
        frame = model.workspace_inputs.publish_capture(points)
        points[:] = 99
        np.testing.assert_array_equal(frame.points_local, [[1., 2.], [3., 4.]])
        self.assertFalse(frame.points_local.flags.writeable)
        self.assertFalse(model.refresh())
        self.assertFalse(model.dirty)
        self.assertEqual(model.session["scans"], [])
        self.assertIs(model.cache["grid"], sentinel)
        self.assertIsNone(model.workspace_inputs.robot_pose_source)

    def test_scan_contract_rejects_invalid_coordinates(self):
        for points in ([], [[1, 2, 3]], [[np.nan, 0]], [[np.inf, 0]]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                ScanFrame(points, 1.)
        with self.assertRaises(ValueError):
            ScanFrame([[1, 2]], np.nan)

    def test_future_sources_are_independent_and_inactive(self):
        model = SharedSession()
        scan_source, pose_source = mock.Mock(), mock.Mock()
        model.workspace_inputs.live_scan_source = scan_source
        model.workspace_inputs.robot_pose_source = pose_source
        model.workspace_inputs.publish_capture([[1., 2.]])
        self.assertEqual(scan_source.mock_calls, [])
        self.assertEqual(pose_source.mock_calls, [])
        self.assertIsNone(SharedSession().workspace_inputs.latest_capture)


if __name__ == "__main__":
    unittest.main()
