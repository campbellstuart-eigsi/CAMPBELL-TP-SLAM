"""Separate contracts for sensor-local scans and future global pose estimates.

No provider is started or polled here. A scan never implies an estimated pose.
The desktop currently publishes only completed, bounded Code 01 captures.
"""
from dataclasses import dataclass
import time
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class ScanFrame:
    points_local: np.ndarray  # N x 2, metres, sensor X/Y; no global transform
    timestamp_s: float
    label: str = "Fresh LD06 capture"

    def __post_init__(self):
        points = np.array(self.points_local, dtype=float, copy=True)
        if points.ndim != 2 or points.shape[1] != 2 or not len(points):
            raise ValueError("A scan needs an N x 2 array of sensor-local points.")
        if not np.all(np.isfinite(points)) or not np.isfinite(self.timestamp_s):
            raise ValueError("Scan points and timestamp must be finite.")
        points.setflags(write=False)
        object.__setattr__(self, "points_local", points)


@dataclass(frozen=True)
class EstimatedRobotPose:
    x_m: float
    y_m: float
    heading_rad: float  # global frame, counterclockwise from +X
    timestamp_s: float


class LiveScanSource(Protocol):
    def latest_scan(self) -> ScanFrame | None: ...


class RobotPoseSource(Protocol):
    def latest_pose(self) -> EstimatedRobotPose | None: ...


class WorkspaceInputs:
    """Session-scoped inputs, independent of map cache invalidation.

    Optional providers document the future boundary; this version never calls
    them. Publishing a capture does not edit scans, build a map or localise.
    """
    def __init__(self):
        self.latest_capture: ScanFrame | None = None
        self.live_scan_source: LiveScanSource | None = None
        self.robot_pose_source: RobotPoseSource | None = None

    def publish_capture(self, points):
        self.latest_capture = ScanFrame(points, time.time())
        return self.latest_capture
