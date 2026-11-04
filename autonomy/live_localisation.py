"""Headless live localisation: track a robot on a FIXED map from a stream of LiDAR scans.

No GUI, no network, no threads: the Live Localisation workspace (later) feeds
each new scan to LiveTracker.update() from its worker thread. The localiser is
Code 04, unchanged and called directly (spec docs/specs/autonomy_phase_b.md,
section 2: not through the Workspace adapter):

  initialising  no pose yet: global search (localise_global) on each scan
  tracking      local search (localise_scan) seeded with the last ACCEPTED pose
  lost          after LOST_AFTER consecutive local failures: global search
                again on each scan until one is accepted

Rules (the project's): only "ok" and "weak" results give a pose; any other
status gives NO pose and the last accepted pose is never moved or guessed
forward. A scan whose seq is not newer than the last one processed is
skipped (a stale or repeated scan is never used twice). The reference map is
never modified. Ground truth is never an input: evaluate() compares results
with a truth pose only after the fact, for tests and reports.

Poses are Code 04's (x_m, y_m, theta_rad) in the map frame; scan points are
sensor-frame (N, 2) metres, +X front, +Y left (autonomy.protocol.
points_to_sensor_xy gives them from the robot's CCW scan angles).
"""
import importlib
import math
import time
from dataclasses import dataclass, field

import numpy as np

import localisation_common as common

loc = importlib.import_module("04_grid_localisation")

ACCEPTED = ("ok", "weak")
LOST_AFTER = 3


@dataclass(frozen=True)
class TrackUpdate:
    """The outcome of one scan. `pose` is set only when `accepted`."""
    seq: int
    timestamp_s: float
    search: str                  # "local", "global" or "skipped"
    status: str                  # Code 04's status, or "stale" when skipped
    accepted: bool
    pose: tuple = None           # (x, y, theta_rad) when accepted, else None
    state: str = ""              # tracker state AFTER this scan
    reason: str = ""
    score: float = float("nan")
    runtime_ms: float = 0.0


@dataclass
class LiveTracker:
    """Tracks one robot on one fixed GridMap; see the module docstring."""
    grid_map: object
    local_config: object = None
    global_config: object = None
    lost_after: int = LOST_AFTER
    state: str = "initialising"
    pose: tuple = None                         # last ACCEPTED pose
    failures: int = 0                          # consecutive local failures
    last_seq: int = None
    history: list = field(default_factory=list)  # every TrackUpdate, in order

    @classmethod
    def from_session(cls, path, **kwargs):
        """A tracker on the Code 04 grid map of a recorded session (read only)."""
        scans, _, _ = loc.load_session(path)
        grid_map, _ = loc.build_grid_map(scans)
        return cls(grid_map, **kwargs)

    def reset(self, initial_pose=None):
        """Forget the track. With initial_pose (a known start), track locally
        from it at once; without, the next scan runs a global search."""
        self.pose = None if initial_pose is None else tuple(float(v) for v in initial_pose)
        self.state = "tracking" if initial_pose is not None else "initialising"
        self.failures = 0
        self.last_seq = None
        self.history = []

    def trajectory(self):
        """(timestamp_s, pose, status) of every accepted scan."""
        return [(u.timestamp_s, u.pose, u.status) for u in self.history if u.accepted]

    def update(self, points_local, seq=None, timestamp_s=None):
        """Localise one scan; returns its TrackUpdate (also kept in history)."""
        timestamp_s = time.time() if timestamp_s is None else float(timestamp_s)
        if seq is not None and self.last_seq is not None and seq <= self.last_seq:
            update = TrackUpdate(seq, timestamp_s, "skipped", "stale", False, None,
                                 self.state, f"scan {seq} is not newer than {self.last_seq}")
            self.history.append(update)
            return update
        if seq is not None:
            self.last_seq = seq
        points = common.filter_ranges(points_local)
        if self.state == "tracking":
            search = "local"
            result = loc.localise_scan(self.grid_map, points, self._seed(), self.local_config)
        else:
            search = "global"
            result = loc.localise_global(self.grid_map, points, self.global_config)
        accepted = result.status in ACCEPTED and result.pose is not None
        if accepted:
            self.pose = tuple(float(v) for v in result.pose)
            self.state, self.failures = "tracking", 0
        elif search == "local":
            self.failures += 1
            if self.failures >= self.lost_after:
                self.state = "lost"
        update = TrackUpdate(seq if seq is not None else len(self.history), timestamp_s, search,
                             result.status, accepted, self.pose if accepted else None,
                             self.state, result.reason, float(result.score),
                             float(result.runtime_ms))
        self.history.append(update)
        return update

    def _seed(self):
        """The last accepted pose, clipped to the map as Code 04's viewer does."""
        left, right, bottom, top = self.grid_map.extent
        x, y, theta = self.pose
        return (float(np.clip(x, left, right)), float(np.clip(y, bottom, top)),
                common.wrap_angle(theta))


def pose_error(estimate, truth):
    """(position error m, heading error deg) of an estimate against a truth pose."""
    return (math.hypot(estimate[0] - truth[0], estimate[1] - truth[1]),
            common.angle_difference_deg(estimate[2], truth[2]))


def evaluate(updates, truths):
    """Summary of a run against simulated truth (tests and reports only).

    updates - TrackUpdates in order; truths - the true pose of each one's scan.
    Errors are measured only on accepted updates; a rejected scan is counted,
    never scored (it has no pose).
    """
    errors = [pose_error(u.pose, t) for u, t in zip(updates, truths) if u.accepted]
    position = np.array([e[0] for e in errors]) if errors else np.array([])
    heading = np.array([e[1] for e in errors]) if errors else np.array([])
    runtimes = np.array([u.runtime_ms for u in updates if u.search != "skipped"])
    return {
        "scans": len(updates),
        "accepted": sum(u.accepted for u in updates),
        "rejected": sum(not u.accepted and u.search != "skipped" for u in updates),
        "global_searches": sum(u.search == "global" for u in updates),
        "statuses": {s: sum(u.status == s for u in updates) for s in
                     sorted({u.status for u in updates})},
        "position_error_m": _stats(position),
        "heading_error_deg": _stats(heading),
        "runtime_ms": _stats(runtimes),
    }


def _stats(values):
    if not len(values):
        return {"median": None, "p95": None, "max": None}
    return {"median": float(np.median(values)), "p95": float(np.percentile(values, 95)),
            "max": float(np.max(values))}
