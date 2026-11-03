"""Day 2, Phase 1 - LiDAR localisation on the Part 2 occupancy grid.

Given the occupancy grid of a room (built by Part 2, unchanged), one LiDAR
scan in its own sensor frame, and an approximate pose, find the pose
(x, y, theta) at which the scan best agrees with the map.

Method: bounded, coarse-to-fine exhaustive search over (x, y, theta)
scored with a LIKELIHOOD FIELD computed from the grid (section 2). The
score distinguishes occupied, free and unknown cells.

Main call (no Matplotlib, no files, no GUI state; Day 3 can use it):
    result = localise_scan(grid_map, scan_points, initial_pose, config)

Data: REAL LD06 recordings only. The default session is
scans/classroom_final.json; running this file opens a file dialog to pick
another one (Cancel = the default). The selected scan is localised on the
Part 2 map of the OTHER scans (leave-one-out), so it is never matched
against its own measurements. Session files are only ever read.

Reference poses: the Day 1 poses were ESTIMATED by Part 1 (not
measured), so the deviation from them is reported as AGREEMENT with the
previous estimate, never as accuracy. Independently measured poses can be
added later in scans/<session>_measured_poses.json (format in
localisation_common.py); they are then evaluated as real errors.

Run this file in PyCharm (RUN_MODE below):
    "application" - desktop page: new scans, capture and recorded evaluation
    "viewer"     - original recorded-scan Matplotlib viewer
    "evaluation" - localise every scan from many initial guesses; results
                   saved in results/phase1_grid_localisation_<session>.*

Automated tests, from the project folder:
    python -m unittest 04_grid_localisation -v
(the search mathematics is tested on a simulated KNOWN-POSE fixture, the
only place where simulated scans are used.)
"""

import contextlib
import csv
import hashlib
import io
import json
import math
import sys
import tempfile
import time
import unittest
from dataclasses import asdict, dataclass, replace
from datetime import datetime
from pathlib import Path
from unittest import mock

import numpy as np

PROJECT_DIR = Path(__file__).resolve().parent
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))
import localisation_common as common  # noqa: E402  (needs the path above)
from localisation_common import (  # noqa: E402
    LocalisationResult, angular_coverage_deg, day1, filter_ranges,
    pose_from_day1, pose_to_day1, rotation, transform_points,
    voxel_downsample, wrap_angle)

part2 = day1("part2")  # the occupancy grid (imported, never modified)

# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------
RUN_MODE = "application"       # "application", legacy "viewer", or "evaluation"
# The recorded session: a bare name is looked for in scans/, an absolute
# path is used as it is. Only ever READ.
SESSION_FILE = "classroom_final.json"
ASK_FOR_SESSION_FILE = True    # run from PyCharm: file dialog first
# Independently measured poses: None = scans/<session>_measured_poses.json
# if that file exists (none exists yet), or a path to another file.
MEASURED_POSES_FILE = None
BENCHMARK_REPEATS = 3          # each localisation is timed this many times
RESULTS_FILE_STEM = "phase1_grid_localisation"


# ---------------------------------------------------------------------------
# 1. The grid map seen by the localisation
# ---------------------------------------------------------------------------
# Part 2 gives a log-odds map (belief per cell) and its spec (x_min, y_min,
# cell_size, width, height; cells stored grid[j, i], row j = Y). Each cell
# is given one of three STATES with Part 2's own thresholds and rule:
#     OCCUPIED - observed and p >= P_OCCUPIED_ABOVE (0.65)
#     FREE     - observed and p <= P_FREE_BELOW (0.35)
#     UNKNOWN  - never observed, OR observed with conflicting evidence
#                (0.35 < p < 0.65). Unknown is NOT free: a beam ending
#                there contradicts nothing we know.
FREE_STATE, UNKNOWN_STATE, OCCUPIED_STATE = -1, 0, 1


@dataclass
class GridMap:
    """An occupancy grid ready for localisation (one per map).

    state       - (height, width) int8: FREE_STATE / UNKNOWN_STATE /
                  OCCUPIED_STATE, indexed [j, i] like Part 2
    probability - (height, width) float: Part 2's p(occupied), for display
    x_min, y_min, cell_size - Part 2's spec: lower-left corner, metres
    """
    state: np.ndarray
    probability: np.ndarray
    x_min: float
    y_min: float
    cell_size: float

    @property
    def height(self):
        return self.state.shape[0]

    @property
    def width(self):
        return self.state.shape[1]

    @property
    def extent(self):
        """(left, right, bottom, top) in metres, as Part 2's grid_extent."""
        return (self.x_min, self.x_min + self.width * self.cell_size,
                self.y_min, self.y_min + self.height * self.cell_size)

    @classmethod
    def from_part2(cls, log_odds_map, spec):
        """Classify Part 2's log-odds map into the three cell states.

        Same thresholds and same "observed" rule as Part 2's
        classify_probabilities(), so the cell counts agree (tested).
        """
        probability = part2.log_odds_to_probability(log_odds_map["log_odds"])
        observed = (log_odds_map["hits"] + log_odds_map["misses"]) > 0
        state = np.full(probability.shape, UNKNOWN_STATE, dtype=np.int8)
        state[observed & (probability <= part2.P_FREE_BELOW)] = FREE_STATE
        state[observed & (probability >= part2.P_OCCUPIED_ABOVE)] = \
            OCCUPIED_STATE
        if min(state.shape) < 2:
            raise ValueError("the grid map needs at least 2 x 2 cells")
        return cls(state=state, probability=probability,
                   x_min=float(spec["x_min"]), y_min=float(spec["y_min"]),
                   cell_size=float(spec["cell_size"]))

    def spec(self):
        """The map as a Part 2 spec dictionary."""
        return {"x_min": self.x_min, "y_min": self.y_min,
                "cell_size": self.cell_size, "width": self.width,
                "height": self.height}

    def cell_coordinates(self, points):
        """Map points (..., 2) -> CONTINUOUS cell coordinates (u, v).

        u = (x - x_min) / cell_size - 0.5, so u = i exactly at the CENTRE of
        column i; floor(u + 0.5) is Part 2's world_to_grid() index i.
        """
        u = (points[..., 0] - self.x_min) / self.cell_size - 0.5
        v = (points[..., 1] - self.y_min) / self.cell_size - 0.5
        return u, v

    def distance_to_occupied(self, max_distance_m):
        """Distance (m) from every cell centre to the nearest occupied one.

        Exact Euclidean distance up to max_distance_m, +inf beyond (those
        cells score 0 anyway). Computed by shifting the occupied layer by
        every offset within the radius (SciPy is not available); a 6-cell
        radius is 113 shifts of a 25 000-cell map: a few milliseconds.
        """
        radius = int(math.ceil(max_distance_m / self.cell_size))
        occupied = self.state == OCCUPIED_STATE
        padded = np.pad(occupied, radius, constant_values=False)
        distance = np.full(occupied.shape, np.inf)
        for dj in range(-radius, radius + 1):
            for di in range(-radius, radius + 1):
                d = math.hypot(di, dj) * self.cell_size
                if d > max_distance_m:
                    continue
                shifted = padded[radius + dj:radius + dj + self.height,
                                 radius + di:radius + di + self.width]
                distance[shifted & (distance > d)] = d
        return distance

    def score_field(self, sigma_m, free_penalty):
        """Per-cell value of a beam ENDING in that cell (section 2)."""
        distance = self.distance_to_occupied(3.0 * sigma_m)
        near_wall = np.exp(-0.5 * (distance / sigma_m) ** 2)  # inf -> 0
        field = near_wall.copy()
        free = self.state == FREE_STATE
        field[free] -= free_penalty * (1.0 - near_wall[free])
        return field


# ---------------------------------------------------------------------------
# 2. The matching score (likelihood field with free-space penalty)
# ---------------------------------------------------------------------------
# For a candidate pose, every scan point p_k (sensor frame) is placed in the
# map, q_k = R(theta) p_k + (x, y), and looked up in a precomputed field:
#
#     g(c)  = exp(-d(c)^2 / (2 sigma^2))   d(c) = distance from cell c to
#                                          the nearest OCCUPIED cell
#     f(c)  = g(c)                         c occupied or unknown
#     f(c)  = g(c) - w (1 - g(c))          c confirmed FREE
#
#     score(pose) = (1 / N) sum_k  f(q_k)   (bilinear between cell centres)
#     points outside the grid count 0 and are reported as out of the map.
#
# Why this score:
#  - It is the "likelihood field" range-finder model (Thrun, Burgard & Fox,
#    Probabilistic Robotics, ch. 6.4): an endpoint is likely where a wall
#    is near. Unlike counting hits on occupied cells, it decreases SMOOTHLY
#    with distance, so a coarse search still sees a slope towards the right
#    pose and the fine search can place the scan to better than a cell.
#  - The free-space term uses NEGATIVE evidence: an endpoint in a cell the
#    map has seen empty contradicts the pose (score down to -w). Near a
#    wall g is close to 1 and the penalty fades, so the 10 cm cells do not
#    punish points that fall just in front of a wall.
#  - UNKNOWN cells only get g: never-observed space (the classroom's
#    missing walls, behind walls, doors) is neither rewarded nor punished.
#  - A wide sigma for the coarse levels and a narrow one for the fine
#    levels (coarse-to-fine) gives a wide basin first, precision last.
# Range: -w (all points in free space) to 1 (all points on occupied cells).
def sample_field(field, grid_map, points):
    """Bilinear field value under map points (..., 2); (values, inside).

    Points outside the grid rectangle get 0 and inside=False. Between the
    centres of the outermost cells and the grid border the edge value is
    used (the field is constant there).
    """
    u, v = grid_map.cell_coordinates(points)
    inside = ((u >= -0.5) & (u < grid_map.width - 0.5)
              & (v >= -0.5) & (v < grid_map.height - 0.5))
    u = np.clip(u, 0.0, grid_map.width - 1.0)
    v = np.clip(v, 0.0, grid_map.height - 1.0)
    i0 = np.minimum(np.floor(u).astype(np.int64), grid_map.width - 2)
    j0 = np.minimum(np.floor(v).astype(np.int64), grid_map.height - 2)
    fu, fv = u - i0, v - j0
    values = ((1 - fu) * (1 - fv) * field[j0, i0]
              + fu * (1 - fv) * field[j0, i0 + 1]
              + (1 - fu) * fv * field[j0 + 1, i0]
              + fu * fv * field[j0 + 1, i0 + 1])
    return np.where(inside, values, 0.0), inside


def score_poses(points, field, grid_map, positions, headings):
    """Score every (heading, position) pair; returns scores[h, p].

    points    - (N, 2) sensor-frame points
    positions - (P, 2) candidate (x, y)
    headings  - (H,) candidate theta in radians
    The scan is rotated once per heading; all positions are then placed at
    once by broadcasting (P, 1, 2) + (1, N, 2) -> (P, N, 2).
    """
    scores = np.empty((len(headings), len(positions)))
    for index, theta in enumerate(headings):
        rotated = points @ rotation(theta).T
        values, _ = sample_field(field, grid_map,
                                 positions[:, None, :] + rotated[None, :, :])
        scores[index] = values.mean(axis=1)
    return scores


def out_of_map_fraction(grid_map, points, pose):
    """Share of the scan's points that fall outside the grid at `pose`."""
    _, inside = sample_field(np.zeros(grid_map.state.shape), grid_map,
                             transform_points(points, pose))
    return float(1.0 - inside.mean())


# ---------------------------------------------------------------------------
# 3. Bounded coarse-to-fine exhaustive search
# ---------------------------------------------------------------------------
# Level 0 tries EVERY pose of a regular grid inside the search window
# around the initial pose (x0 +- xy_window, y0 +- xy_window,
# theta0 +- theta_window). The best `candidates` clearly different poses
# are kept (non-maximum suppression, as in Part 1), so a wrong local peak
# cannot hide the right one. Each later level searches +- one step of the
# previous level around each candidate with a finer step and a narrower
# field. The best final candidate is the answer.
@dataclass(frozen=True)
class SearchLevel:
    xy_step_m: float
    theta_step_deg: float
    sigma_m: float


@dataclass(frozen=True)
class GridSearchConfig:
    xy_window_m: float = 1.0       # search x0 +- this and y0 +- this
    theta_window_deg: float = 30.0  # search theta0 +- this
    levels: tuple = (SearchLevel(0.10, 2.0, 0.20),
                     SearchLevel(0.025, 0.5, 0.10),
                     SearchLevel(0.005, 0.1, 0.07))
    candidates: int = 5            # distinct level-0 poses refined further
    distinct_m: float = 0.30       # two poses differ if position OR
    distinct_deg: float = 10.0     # heading differs by more than this
    coarse_voxel_m: float = 0.10   # scan thinning for level 0
    voxel_m: float = 0.05          # scan thinning for the other levels
    min_range_m: float = common.MIN_RANGE_M
    max_range_m: float = common.MAX_RANGE_M
    free_penalty: float = 0.5      # w in the score formula
    min_points: int = 30           # after thinning
    min_coverage_deg: float = 90.0  # angle seen around the sensor
    min_score: float = 0.25        # below: "low_score"
    max_out_of_map: float = 0.5    # more points off the grid: "out_of_map"
    ambiguity_ratio: float = 0.95  # runner-up this close: "ambiguous"
    max_candidates: int = 3_000_000  # refuse searches that would freeze

    def to_dict(self):
        data = asdict(self)
        data["levels"] = [asdict(level) for level in self.levels]
        return data


def symmetric_offsets(half_width, step):
    """Offsets -half_width ... +half_width in steps of `step` (0 included)."""
    count = int(round(half_width / step)) if step > 0 else 0
    return np.arange(-count, count + 1) * step


def level_grid(centre, xy_half, theta_half_rad, level):
    """Candidate positions (P, 2) and headings (H,) around `centre`."""
    dx = symmetric_offsets(xy_half, level.xy_step_m)
    grid_x, grid_y = np.meshgrid(centre[0] + dx, centre[1] + dx)
    positions = np.column_stack([grid_x.ravel(), grid_y.ravel()])
    headings = centre[2] + symmetric_offsets(
        theta_half_rad, math.radians(level.theta_step_deg))
    return positions, headings


def planned_candidates(config):
    """Number of poses the search will score (checked before running)."""
    levels = config.levels
    first = levels[0]
    count = (len(symmetric_offsets(config.xy_window_m, first.xy_step_m)) ** 2
             * len(symmetric_offsets(config.theta_window_deg,
                                     first.theta_step_deg)))
    for previous, level in zip(levels, levels[1:]):
        count += config.candidates * (
            len(symmetric_offsets(previous.xy_step_m, level.xy_step_m)) ** 2
            * len(symmetric_offsets(previous.theta_step_deg,
                                    level.theta_step_deg)))
    return count


def search_reach(config):
    """Farthest (metres, degrees) the answer can be from the initial pose.

    The window, plus one step of every level but the last (each finer
    level searches +- one step of the previous level).
    """
    levels = config.levels[:-1]
    return (config.xy_window_m + sum(level.xy_step_m for level in levels),
            config.theta_window_deg
            + sum(level.theta_step_deg for level in levels))


def poses_distinct(a, b, config):
    return (math.hypot(a[0] - b[0], a[1] - b[1]) > config.distinct_m
            or common.angle_difference_deg(a[2], b[2]) > config.distinct_deg)


def best_distinct(scores, positions, headings, count, config):
    """Up to `count` best (pose, score), each clearly different."""
    chosen = []
    order = np.argsort(-scores, axis=None, kind="stable")
    for flat in order:
        h, p = np.unravel_index(flat, scores.shape)
        pose = (float(positions[p, 0]), float(positions[p, 1]),
                float(headings[h]))
        if all(poses_distinct(pose, other, config) for other, _ in chosen):
            chosen.append((pose, float(scores[h, p])))
            if len(chosen) == count:
                break
    return chosen


def prepare_points(scan_points, config):
    """Range filter, then thinning: (fine points, coarse points)."""
    points = filter_ranges(scan_points, config.min_range_m,
                           config.max_range_m)
    return (voxel_downsample(points, config.voxel_m),
            voxel_downsample(points, config.coarse_voxel_m))


def localise_scan(grid_map, scan_points, initial_pose, config=None,
                  true_pose=None):
    """Estimate the pose of one scan on an occupancy grid map.

    grid_map     - GridMap (from GridMap.from_part2)
    scan_points  - (N, 2) points in the SENSOR frame (metres), any order;
                   invalid or out-of-range points are dropped
    initial_pose - (x, y, theta_rad) approximate pose, centre of the search
    config       - GridSearchConfig (default settings if None)
    true_pose    - optional (x, y, theta_rad); only used AFTER the timed
                   search to fill the error fields of the result
    Returns a LocalisationResult (pose in radians; see localisation_common).
    """
    config = config or GridSearchConfig()
    if planned_candidates(config) > config.max_candidates:
        raise ValueError(
            f"search too large: {planned_candidates(config)} candidate "
            f"poses (limit {config.max_candidates}); reduce the windows or "
            f"use coarser steps")
    initial_pose = (float(initial_pose[0]), float(initial_pose[1]),
                    wrap_angle(float(initial_pose[2])))
    start = time.perf_counter()
    points, coarse_points = prepare_points(scan_points, config)

    def finish(status, reason, **values):
        result = LocalisationResult(
            method="grid_exhaustive", status=status, reason=reason,
            initial_pose=initial_pose, points_used=len(points),
            runtime_ms=(time.perf_counter() - start) * 1000.0, **values)
        return result.evaluate_against(true_pose) if true_pose else result

    if len(points) < config.min_points:
        return finish("insufficient_points",
                      f"{len(points)} usable points after filtering and "
                      f"thinning; at least {config.min_points} needed")
    coverage = angular_coverage_deg(points)
    if coverage < config.min_coverage_deg:
        return finish("insufficient_coverage",
                      f"the scan covers only {coverage:.0f} deg around the "
                      f"sensor; at least {config.min_coverage_deg:.0f} deg "
                      f"needed to fix a pose")

    levels = config.levels
    fields = [grid_map.score_field(level.sigma_m, config.free_penalty)
              for level in levels]
    evaluated = 0

    # Level 0: every pose in the window.
    positions, headings = level_grid(initial_pose, config.xy_window_m,
                                     math.radians(config.theta_window_deg),
                                     levels[0])
    scores = score_poses(coarse_points, fields[0], grid_map, positions,
                         headings)
    evaluated += scores.size
    candidates = best_distinct(scores, positions, headings,
                               config.candidates, config)

    # Finer levels around each candidate.
    refined = []
    for coarse_pose, _ in candidates:
        pose = coarse_pose
        for previous, level, level_field in zip(levels, levels[1:],
                                                fields[1:]):
            positions, headings = level_grid(
                pose, previous.xy_step_m,
                math.radians(previous.theta_step_deg), level)
            scores = score_poses(points, level_field, grid_map, positions,
                                 headings)
            evaluated += scores.size
            h, p = np.unravel_index(np.argmax(scores), scores.shape)
            pose = (float(positions[p, 0]), float(positions[p, 1]),
                    float(headings[h]))
        final_score = float(score_poses(points, fields[-1], grid_map,
                                        np.array([pose[:2]]),
                                        np.array([pose[2]]))[0, 0])
        refined.append((pose, final_score, coarse_pose))
    refined.sort(key=lambda item: -item[1])  # stable: ties keep coarse order
    pose, score, coarse_pose = refined[0]
    pose = (pose[0], pose[1], wrap_angle(pose[2]))
    runner_up = next((other_score for other, other_score, _ in refined[1:]
                      if poses_distinct(other, pose, config)), None)
    outside = out_of_map_fraction(grid_map, points, pose)

    # The method's own verdict (no ground truth used). "On the edge": the
    # level-0 pose that led to the answer is on the window's outer ring.
    edge_xy = config.xy_window_m - levels[0].xy_step_m / 2
    edge_theta = math.radians(config.theta_window_deg
                              - levels[0].theta_step_deg / 2)
    on_edge = (abs(coarse_pose[0] - initial_pose[0]) > edge_xy
               or abs(coarse_pose[1] - initial_pose[1]) > edge_xy
               or abs(wrap_angle(coarse_pose[2] - initial_pose[2]))
               > edge_theta)
    if outside > config.max_out_of_map:
        status, reason = "out_of_map", (
            f"{outside:.0%} of the points fall outside the grid at the best "
            f"pose (at most {config.max_out_of_map:.0%} allowed)")
    elif score < config.min_score:
        status, reason = "low_score", (
            f"best score {score:.3f} is below {config.min_score}: the scan "
            f"does not match the map anywhere in the search window")
    elif on_edge:
        status, reason = "at_search_limit", (
            "the best pose lies on the edge of the search window: the true "
            "pose may be outside it (enlarge the window or improve the "
            "initial pose)")
    elif runner_up is not None and \
            runner_up >= config.ambiguity_ratio * score:
        status, reason = "ambiguous", (
            f"a clearly different pose scores {runner_up:.3f}, almost as "
            f"well as the best {score:.3f}")
    else:
        status, reason = "ok", (
            f"score {score:.3f}"
            + (f", best different pose {runner_up:.3f}"
               if runner_up is not None else ""))
    return finish(status, reason, pose=pose, score=score,
                  candidates_evaluated=int(evaluated),
                  out_of_map_fraction=outside, runner_up_score=runner_up,
                  details={"coverage_deg": coverage,
                           "coarse_points": len(coarse_points),
                           "coarse_candidates": [
                               [round(v, 4) for v in pose_to_day1(c)] + [s]
                               for c, s in candidates]})


# ---------------------------------------------------------------------------
# 4. Reference maps of a recorded session, built by Part 2 (unchanged)
# ---------------------------------------------------------------------------
def build_grid_map(scans, spec=None):
    """Part 2's probabilistic map of `scans` -> (GridMap, log_odds_map).

    scans - Part 2-style dicts with "pose" (x_m, y_m, theta_DEG) and
            "points_local" (N, 2). spec - Part 2 grid spec; None = Part 2's
            automatic bounds (choose_grid_bounds) with its 10 cm cells.
    """
    global_points = part2.global_points_of_scans(scans)
    if spec is None:
        spec = session_spec(scans)
    log_odds_map, _ = part2.build_probabilistic_occupancy_map(
        spec, scans, global_points)
    return GridMap.from_part2(log_odds_map, spec), log_odds_map


def session_spec(scans):
    """Part 2's automatic grid spec for ALL the given scans (10 cm cells).

    Leave-one-out maps use the spec of the whole session, so every map of
    a session has the same cells.
    """
    global_points = part2.global_points_of_scans(scans)
    return part2.create_grid_spec(
        *part2.choose_grid_bounds(global_points,
                                  [scan["pose"] for scan in scans]),
        part2.CELL_SIZE_M)


def reference_map(scans, scan_id, spec, exclude=True):
    """The map a scan is localised on; returns (GridMap, description).

    exclude=True (default): built from every OTHER scan (leave-one-out),
    so the scan is never matched against its own measurements. With a
    single scan there is nothing else to build a map from: the map then
    contains the scan itself, and the description says so.
    """
    if all(scan["id"] != scan_id for scan in scans):
        raise KeyError(f"no scan with id {scan_id}")
    others = [scan for scan in scans if scan["id"] != scan_id]
    if exclude and others:
        return (build_grid_map(others, spec)[0],
                f"Part 2 map of the other {len(others)} scan(s) "
                f"(scan {scan_id} excluded)")
    note = ("only scan in the session: SELF-MATCHING" if exclude else
            "all scans INCLUDING this one: SELF-MATCHING, optimistic")
    return build_grid_map(scans, spec)[0], f"Part 2 map of {note}"


# ---------------------------------------------------------------------------
# 5. Choosing the recorded session (as in Parts 2 and 3) and its references
# ---------------------------------------------------------------------------
def session_path(file_name=None):
    """Bare names are looked for in scans/; absolute paths are kept."""
    return part2.resolve_session_path(file_name or SESSION_FILE,
                                      scans_dir=common.SCANS_DIR)


def choose_session_file(initial=SESSION_FILE):
    """File dialog to pick a session JSON; returns a Path or None.

    tkinter is imported here only: the localisation itself never needs it.
    """
    from tkinter import Tk, filedialog
    root = Tk()
    root.withdraw()
    try:
        file_name = filedialog.askopenfilename(
            title="Select a LiDAR session file - grid localisation",
            initialdir=str(common.SCANS_DIR), initialfile=initial,
            filetypes=[("JSON session files", "*.json"),
                       ("All files", "*.*")])
    finally:
        root.destroy()
    return Path(file_name) if file_name else None


def load_session(path, measured_file=None):
    """(scans, measured poses, measured-file path or None); read only.

    measured_file None: the default <session>_measured_poses.json next to
    the session is used if it exists; no file = no measured poses.
    """
    scans = common.load_session_scans(path)
    if not scans:
        raise part2.SessionError(f"{Path(path).name} has no visible scan")
    measured_path = Path(measured_file) if measured_file else \
        common.measured_poses_path(path)
    if measured_file is None and not measured_path.is_file():
        return scans, {}, None
    return scans, common.load_measured_poses(measured_path), measured_path


# ---------------------------------------------------------------------------
# 6. Experiments on the recorded scans (repeatable, saved)
# ---------------------------------------------------------------------------
# Every scan of the session is localised on its leave-one-out map, from
# its Day 1 pose and from guesses moved away from it by EXACT errors (the
# direction comes from a fixed seed). The same search configuration is
# used throughout; its window is +-1 m, +-30 deg, so the last condition is
# deliberately OUTSIDE it.
#
# What each number means with the data we have:
#   deviation from the Day 1 pose - AGREEMENT with the previous estimate
#       (Part 1's automatic alignment, or a hand placement for scans 22
#       and 25). It is NOT accuracy: both poses could be wrong together.
#   convergence spread - how far apart the final poses of one scan are
#       when starting from different guesses inside the window. It needs
#       no reference at all: a small spread means the result does not
#       depend on the initial guess.
#   error against a MEASURED pose - real accuracy; only computed for the
#       scans listed in a measured-poses file (none exist yet).
REAL_INIT_CONDITIONS = (("from Day 1 pose", 0.0, 0.0, 1),
                        ("0.10 m / 3 deg", 0.10, 3.0, 2),
                        ("0.30 m / 10 deg", 0.30, 10.0, 2),
                        ("0.50 m / 15 deg", 0.50, 15.0, 2),
                        ("0.75 m / 25 deg", 0.75, 25.0, 2),
                        ("1.00 m / 30 deg (window edge)", 1.00, 30.0, 2),
                        ("1.50 m / 45 deg (beyond window)", 1.50, 45.0, 2))
CONVERGED_M, CONVERGED_DEG = 0.02, 0.5  # "same answer" for the spread
CSV_COLUMNS = (
    "session", "scan", "reference_map", "condition", "trial",
    "day1_pose", "initial_pose", "estimated_pose",
    "initial_offset_m", "initial_offset_deg",
    "day1_deviation_m", "day1_deviation_deg", "agrees_with_day1",
    "measured_pose", "measured_sigma_xy_m", "measured_sigma_theta_deg",
    "measured_error_m", "measured_error_deg", "within_thresholds_measured",
    "status", "score", "runner_up_score", "candidates_evaluated",
    "points_used", "out_of_map_fraction", "runtime_ms_median", "repeats",
    "deterministic")


def timed_localisation(grid_map, points, initial_pose, config, repeats):
    """Median runtime over `repeats` identical runs; checks they agree."""
    results = []

    def run():
        result = localise_scan(grid_map, points, initial_pose, config)
        results.append(result)
        return result

    result, median_ms = common.median_runtime(run, repeats)
    deterministic = all(other.pose == result.pose
                        and other.score == result.score
                        for other in results)
    return replace(result, runtime_ms=median_ms), deterministic


def degrees_list(pose):
    return None if pose is None else \
        [round(value, 6) for value in pose_to_day1(pose)]


def make_row(session_name, scan, map_description, condition, trial,
             result, measured, repeats, deterministic):
    """One run: agreement with Day 1 and, if measured, the real error."""
    day1 = result.evaluate_against(scan["pose_rad"])
    row = {"session": session_name, "scan": scan["name"],
           "scan_id": scan["id"], "reference_map": map_description,
           "condition": condition, "trial": trial,
           "day1_pose": degrees_list(scan["pose_rad"]),
           "initial_pose": degrees_list(result.initial_pose),
           "estimated_pose": degrees_list(result.pose),
           "initial_offset_m": day1.initial_position_error_m,
           "initial_offset_deg": day1.initial_angle_error_deg,
           "day1_deviation_m": day1.position_error_m,
           "day1_deviation_deg": day1.angle_error_deg,
           "agrees_with_day1": day1.success,
           "measured_pose": None, "measured_sigma_xy_m": None,
           "measured_sigma_theta_deg": None, "measured_error_m": None,
           "measured_error_deg": None, "within_thresholds_measured": None,
           "status": result.status, "score": result.score,
           "runner_up_score": result.runner_up_score,
           "candidates_evaluated": result.candidates_evaluated,
           "points_used": result.points_used,
           "out_of_map_fraction": result.out_of_map_fraction,
           "runtime_ms_median": result.runtime_ms, "repeats": repeats,
           "deterministic": deterministic, "reason": result.reason}
    if measured is not None:
        truth = result.evaluate_against(measured["pose"])
        row.update({"measured_pose": degrees_list(measured["pose"]),
                    "measured_sigma_xy_m": measured["sigma_xy_m"],
                    "measured_sigma_theta_deg": measured["sigma_theta_deg"],
                    "measured_error_m": truth.position_error_m,
                    "measured_error_deg": truth.angle_error_deg,
                    "within_thresholds_measured": truth.success})
    return row


def session_rows(session_name, scans, measured, config, repeats,
                 conditions=REAL_INIT_CONDITIONS, scan_ids=None,
                 exclude=True):
    spec = session_spec(scans)
    rows = []
    for scan in scans:
        if scan_ids is not None and scan["id"] not in scan_ids:
            continue
        grid_map, description = reference_map(scans, scan["id"], spec,
                                              exclude)
        for level, (name, error_m, error_deg, trials) in \
                enumerate(conditions):
            for trial in range(trials):
                rng = np.random.default_rng(1000 * scan["id"] + 10 * level
                                            + trial)
                initial = common.perturb_pose(scan["pose_rad"], error_m,
                                              error_deg, rng)
                result, deterministic = timed_localisation(
                    grid_map, scan["points_local"], initial, config,
                    repeats)
                rows.append(make_row(session_name, scan, description, name,
                                     trial, result, measured.get(scan["id"]),
                                     repeats, deterministic))
    return rows


def median_or_none(values, function=np.median):
    values = [value for value in values if value is not None]
    return float(function(values)) if values else None


def summarise_conditions(rows):
    """Per initial condition: agreement rate, deviations, time, statuses."""
    groups = {}
    for row in rows:
        groups.setdefault(row["condition"], []).append(row)
    summary = []
    for condition, items in groups.items():
        statuses = {}
        for row in items:
            statuses[row["status"]] = statuses.get(row["status"], 0) + 1
        measured = [row for row in items
                    if row["within_thresholds_measured"] is not None]
        summary.append({
            "condition": condition, "runs": len(items),
            "agree_with_day1": sum(bool(r["agrees_with_day1"])
                                   for r in items),
            "median_day1_deviation_m": median_or_none(
                [r["day1_deviation_m"] for r in items]),
            "max_day1_deviation_m": median_or_none(
                [r["day1_deviation_m"] for r in items], np.max),
            "median_day1_deviation_deg": median_or_none(
                [r["day1_deviation_deg"] for r in items]),
            "max_day1_deviation_deg": median_or_none(
                [r["day1_deviation_deg"] for r in items], np.max),
            "measured_runs": len(measured),
            "within_thresholds_measured": sum(
                bool(r["within_thresholds_measured"]) for r in measured),
            "median_runtime_ms": median_or_none(
                [r["runtime_ms_median"] for r in items]),
            "max_runtime_ms": median_or_none(
                [r["runtime_ms_median"] for r in items], np.max),
            "candidates_evaluated": median_or_none(
                [r["candidates_evaluated"] for r in items]),
            "statuses": statuses,
            "all_deterministic": all(r["deterministic"] for r in items)})
    return summary


def summarise_scans(rows, config):
    """Per scan: Day 1 agreement and the convergence spread.

    spread - largest distance (m, deg) between the final pose from the
    Day 1 start and the final poses from the other starts INSIDE the
    search window: 0 means every start found the same pose.
    """
    reach_m, reach_deg = config.xy_window_m, config.theta_window_deg
    summary = []
    for scan_id in dict.fromkeys(row["scan_id"] for row in rows):
        items = [row for row in rows if row["scan_id"] == scan_id]
        anchor = next((row for row in items
                       if row["initial_offset_m"] == 0.0), items[0])
        inside = [row for row in items if row["estimated_pose"] is not None
                  and row["initial_offset_m"] <= reach_m + 1e-9
                  and row["initial_offset_deg"] <= reach_deg + 1e-9]
        spread_m = spread_deg = None
        converged = 0
        if anchor["estimated_pose"] is not None and inside:
            reference = pose_from_day1(anchor["estimated_pose"])
            errors = [common.pose_error(pose_from_day1(row["estimated_pose"]),
                                        reference) for row in inside]
            spread_m = max(error[0] for error in errors)
            spread_deg = max(error[1] for error in errors)
            converged = sum(1 for metres, degrees in errors
                            if metres <= CONVERGED_M
                            and degrees <= CONVERGED_DEG)
        summary.append({
            "scan": anchor["scan"], "scan_id": scan_id,
            "status_from_day1_pose": anchor["status"],
            "score_from_day1_pose": anchor["score"],
            "day1_deviation_m": anchor["day1_deviation_m"],
            "day1_deviation_deg": anchor["day1_deviation_deg"],
            "starts_inside_window": len(inside), "converged_starts": converged,
            "spread_m": spread_m, "spread_deg": spread_deg,
            "measured": any(row["measured_pose"] is not None
                            for row in items)})
    return summary


def print_summaries(condition_summary, scan_summary):
    def cm(value):
        return "     -" if value is None else f"{value * 100:6.1f}"

    def deg(value):
        return "    -" if value is None else f"{value:5.2f}"

    print("\nBy initial guess (deviation = AGREEMENT with the Day 1 "
          "estimate, not accuracy):")
    print(f"{'condition':32s} {'agree':>7s} {'med cm':>6s} {'max cm':>6s} "
          f"{'med deg':>7s} {'max deg':>7s} {'med ms':>6s}  statuses")
    for item in condition_summary:
        print(f"{item['condition']:32s} {item['agree_with_day1']:3d}/"
              f"{item['runs']:<3d} {cm(item['median_day1_deviation_m'])} "
              f"{cm(item['max_day1_deviation_m'])} "
              f"{deg(item['median_day1_deviation_deg']):>7s} "
              f"{deg(item['max_day1_deviation_deg']):>7s} "
              f"{item['median_runtime_ms']:6.0f}  {item['statuses']}")
    print("\nBy scan (start from the Day 1 pose; spread over the starts "
          "inside the window):")
    print(f"{'scan':8s} {'status':14s} {'score':>6s} {'dev cm':>6s} "
          f"{'dev deg':>7s} {'same pose':>9s} {'spread cm':>9s} "
          f"{'spread deg':>10s}")
    for item in scan_summary:
        print(f"{item['scan']:8s} {item['status_from_day1_pose']:14s} "
              f"{item['score_from_day1_pose']:6.3f} "
              f"{cm(item['day1_deviation_m'])} "
              f"{deg(item['day1_deviation_deg']):>7s} "
              f"{item['converged_starts']:4d}/{item['starts_inside_window']:<4d}"
              f" {cm(item['spread_m']):>9s} {deg(item['spread_deg']):>10s}")


GROUND_TRUTH_STATUS = (
    "No independently measured real pose exists yet. Measurable now on the "
    "real recordings: computation time, matching score and search status, "
    "agreement with the Day 1 estimate, determinism, and initial-pose "
    "sensitivity as convergence (do different starts reach the same "
    "pose?). Still requiring measured poses (scans/<session>_measured_"
    "poses.json, in the session's map frame, with uncertainty): position "
    "error (m), angular error (deg), success rate against the thresholds, "
    "and sensitivity of the ACCURACY to the initial pose. Until then, the "
    "Day 1 deviations must not be reported as accuracy.")


def save_results(rows, condition_summary, scan_summary, config, output_dir,
                 stem, session, measured_file, notes=()):
    """Write <stem>.json (everything) and <stem>.csv (one row per run)."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "phase": "Day 2 Phase 1 - grid map localisation, real LD06 data",
        "created": datetime.now().isoformat(timespec="seconds"),
        "session": str(session), "measured_poses_file":
            str(measured_file) if measured_file else None,
        "method": "bounded coarse-to-fine exhaustive search, likelihood "
                  "field with free-space penalty (04_grid_localisation.py)",
        "thresholds": {"position_m": common.SUCCESS_POSITION_M,
                       "angle_deg": common.SUCCESS_ANGLE_DEG,
                       "note": "provisional experimental thresholds, not "
                               "official course values"},
        "meaning": {
            "day1_deviation": "agreement with the Day 1 pose, which Part 1 "
                              "ESTIMATED; not ground-truth accuracy",
            "measured_error": "error against an independently measured "
                              "pose (only for scans in the measured file)"},
        "ground_truth_status": GROUND_TRUTH_STATUS,
        "timing": "median of the repeats; algorithm only (no file loading, "
                  "no map building, no drawing)",
        "search_config": config.to_dict(), "notes": list(notes),
        "summary_by_condition": condition_summary,
        "summary_by_scan": scan_summary, "runs": rows}
    json_path = output_dir / f"{stem}.json"
    json_path.write_text(json.dumps(payload, indent=1), encoding="utf-8")
    csv_path = output_dir / f"{stem}.csv"
    with open(csv_path, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=CSV_COLUMNS,
                                extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: (" ".join(f"{v:.4f}" for v in value)
                                   if isinstance(value, list) else value)
                             for key, value in row.items()})
    return json_path, csv_path


EVALUATION_NOTES = (
    "Leave-one-out: each scan is localised on the Part 2 map of the OTHER "
    "scans of the session (same grid cells as the full map), so it is never "
    "matched against its own measurements.",
    "The Day 1 poses of the other scans also come from Part 1's matcher (or "
    "hand placement), so the leave-one-out maps are not independent of "
    "Day 1; agreement with Day 1 is therefore partly self-consistency.",
    "Scans 10, 13, 16, 22 and 23 of classroom_final.json hold about two "
    "LiDAR revolutions. The 5 cm voxel thinning keeps at most one point per "
    "cell: near the sensor the revolutions merge; at long range the second "
    "one samples between the first one's points, so these scans keep about "
    "1.4-1.5x the points of one revolution (density capped, not doubled).",
    "Part 2's 10 cm cells limit precision: a wall is marked in the cell "
    "holding most of its hits, so map walls can sit up to half a cell "
    "(5 cm) from the real surface. The unit tests show this on a "
    "known-pose fixture (about 4-5 cm when walls lie on cell borders, "
    "about 0 when they lie at cell centres).")


def results_stem(path):
    return f"{RESULTS_FILE_STEM}_{Path(path).stem}"


def run_evaluation(session_file=None, measured_file=None,
                   output_dir=common.RESULTS_DIR, repeats=BENCHMARK_REPEATS,
                   conditions=REAL_INIT_CONDITIONS, scan_ids=None,
                   config=None):
    """Localise every recorded scan; print and save the results."""
    config = config or GridSearchConfig()
    path = session_path(session_file)
    print(f"=== Day 2 Phase 1: grid localisation on {path.name} "
          f"(real LD06 data) ===")
    fingerprint = part2.file_sha256(path)
    scans, measured, measured_path = load_session(path, measured_file)
    print(f"{len(scans)} scans; measured reference poses: "
          f"{len(measured) if measured else 'none'}"
          + (f" ({measured_path.name})" if measured_path else ""))
    rows = session_rows(path.name, scans, measured, config, repeats,
                        conditions, scan_ids)
    if part2.file_sha256(path) != fingerprint:
        raise RuntimeError(f"{path.name} changed during the evaluation")
    condition_summary = summarise_conditions(rows)
    scan_summary = summarise_scans(rows, config)
    print_summaries(condition_summary, scan_summary)
    print(f"\n{GROUND_TRUTH_STATUS}")
    json_path, csv_path = save_results(
        rows, condition_summary, scan_summary, config, output_dir,
        results_stem(path), path, measured_path, EVALUATION_NOTES)
    print(f"Saved {json_path} and {csv_path.name}")
    return rows, condition_summary, scan_summary


# ---------------------------------------------------------------------------
# 7. Interactive Auto Align viewer (Matplotlib, like Parts 1-3)
# ---------------------------------------------------------------------------
# The runtime shown is the one measured inside localise_scan(): drawing is
# never included.
ACTIVE_VIEWER = None  # the open viewer (keeps its widgets alive)
INITIAL_COLOR, ALIGNED_COLOR = "tab:orange", "tab:blue"
DAY1_COLOR, MEASURED_COLOR = "tab:green", "tab:purple"
VIEWER_GUESS_ERROR = (0.30, 10.0)  # "New guess": moderate error (m, deg)


def pose_marker(axes, color, linestyle="-"):
    """A dot + heading line for one pose; returns the two artists."""
    dot, = axes.plot([], [], "o", color=color, markeredgecolor="black",
                     markersize=7, zorder=5)
    arrow, = axes.plot([], [], linestyle, color=color, linewidth=2.2,
                       zorder=5)
    return dot, arrow


def set_pose_marker(marker, pose, length=0.5):
    dot, arrow = marker
    if pose is None:
        dot.set_data([], [])
        arrow.set_data([], [])
        return
    x, y, theta = pose
    dot.set_data([x], [y])
    arrow.set_data([x, x + length * math.cos(theta)],
                   [y, y + length * math.sin(theta)])


def pose_line(label, pose):
    x, y, theta = pose
    return f"{label:10s}{x:8.3f}{y:8.3f}{math.degrees(wrap_angle(theta)):10.2f}"


class AutoAlignViewer:
    """A recorded scan over the reference map; Auto Align snaps it on.

    Choose the scan with < > or by typing its number. Move the initial
    guess with the sliders, or on the map: LEFT click = put the guess
    there, RIGHT click = turn it towards the clicked point.
    """

    def __init__(self, session_name, scans, measured=None, config=None,
                 start_scan_id=None, figure=None, mode="evaluation",
                 full_grid=None):
        import matplotlib.pyplot as plt
        from matplotlib.lines import Line2D
        from matplotlib.widgets import Button, CheckButtons, Slider, TextBox

        self.session_name = session_name
        self.scans = scans
        self.measured = measured or {}
        self.config = config or GridSearchConfig()
        self.spec = session_spec(scans)
        self.maps = {}               # (scan id, exclude) -> (map, text)
        self.exclude = True
        ids = [scan["id"] for scan in scans]
        self.index = ids.index(start_scan_id) if start_scan_id in ids else 0
        self.result = None
        self.updating = False
        self.guess_seed = 0
        self.mode = mode
        self.external_scan = None
        self.full_grid = full_grid

        self.figure = figure if figure is not None else plt.figure(figsize=(14, 8.5))
        if self.figure.canvas.manager is not None:
            self.figure.canvas.manager.set_window_title(
                f"Day 2 Phase 1 - grid localisation - {session_name}")
        self.axes = self.figure.add_axes([0.05, 0.07, 0.60, 0.86])
        self.image = self.axes.imshow(np.zeros((2, 2)), cmap="gray_r",
                                      vmin=part2.FREE, vmax=part2.OCCUPIED,
                                      origin="lower", interpolation="nearest")
        self.initial_points = self.axes.scatter(
            np.empty(0), np.empty(0), s=5, color=INITIAL_COLOR, zorder=3)
        self.aligned_points = self.axes.scatter(
            np.empty(0), np.empty(0), s=5, color=ALIGNED_COLOR, zorder=4)
        self.initial_marker = pose_marker(self.axes, INITIAL_COLOR)
        self.aligned_marker = pose_marker(self.axes, ALIGNED_COLOR)
        self.day1_marker = pose_marker(self.axes, DAY1_COLOR, "--")
        self.measured_marker = pose_marker(self.axes, MEASURED_COLOR, ":")
        self.axes.set_xlabel("X (m)")
        self.axes.set_ylabel("Y (m)")
        self.axes.set_aspect("equal")
        legend_items = [
            Line2D([], [], marker="o", color=INITIAL_COLOR,
                   markeredgecolor="black", label="scan + pose: initial guess"),
            Line2D([], [], marker="o", color=ALIGNED_COLOR,
                   markeredgecolor="black",
                   label="scan + pose: Auto Align result"),
            Line2D([], [], marker="o", linestyle="--", color=DAY1_COLOR,
                   markeredgecolor="black",
                   label="Day 1 pose (estimate, not measured)"),
            Line2D([], [], marker="s", linestyle="", color="black",
                   label="occupied cell (p >= 0.65)"),
            Line2D([], [], marker="s", linestyle="", color="0.55",
                   label="unknown / uncertain cell"),
            Line2D([], [], marker="s", linestyle="", color="white",
                   markeredgecolor="0.6", label="free cell (p <= 0.35)")]
        if self.measured:
            legend_items.insert(3, Line2D(
                [], [], marker="o", linestyle=":", color=MEASURED_COLOR,
                markeredgecolor="black", label="measured pose"))
        self.legend_items = legend_items
        self.axes.legend(handles=legend_items, loc="upper right", fontsize=8,
                         framealpha=0.9)

        self.figure.text(0.70, 0.955, f"Session: {session_name}",
                         fontsize=10, weight="bold")
        self.previous_button = Button(self.figure.add_axes(
            [0.70, 0.89, 0.05, 0.04]), "<")
        self.next_button = Button(self.figure.add_axes(
            [0.76, 0.89, 0.05, 0.04]), ">")
        self.previous_button.on_clicked(lambda _: self.step_scan(-1))
        self.next_button.on_clicked(lambda _: self.step_scan(+1))
        self.scan_box = TextBox(self.figure.add_axes(
            [0.90, 0.89, 0.06, 0.04]), "scan number ")
        self.scan_box.on_submit(self.on_scan_number)
        self.exclude_box = CheckButtons(self.figure.add_axes(
            [0.70, 0.82, 0.26, 0.05]),
            ["exclude this scan from the map"], [True])
        self.exclude_box.on_clicked(self.on_exclude)
        self.sliders = {
            "x": Slider(self.figure.add_axes([0.73, 0.77, 0.22, 0.025]),
                        "x (m)", -1.0, 1.0, valinit=0.0),
            "y": Slider(self.figure.add_axes([0.73, 0.73, 0.22, 0.025]),
                        "y (m)", -1.0, 1.0, valinit=0.0),
            "theta": Slider(self.figure.add_axes([0.73, 0.69, 0.22, 0.025]),
                            "theta (deg)", -180.0, 180.0, valinit=0.0)}
        for slider in self.sliders.values():
            slider.on_changed(self.on_slider)
        self.buttons = {}
        for label, place, action in (
                ("Auto Align", [0.70, 0.61, 0.12, 0.05], self.auto_align),
                ("Day 1 pose", [0.83, 0.61, 0.12, 0.05], self.day1_guess),
                ("New guess", [0.70, 0.55, 0.12, 0.05], self.new_guess),
                ("Use result", [0.83, 0.55, 0.12, 0.05], self.use_result)):
            button = Button(self.figure.add_axes(place), label)
            button.on_clicked(lambda _, action=action: action())
            self.buttons[label] = button
        self.info_axes = self.figure.add_axes([0.70, 0.01, 0.29, 0.52])
        self.info_axes.axis("off")
        self.info = self.info_axes.text(0.0, 1.0, "", va="top",
                                        family="monospace", fontsize=8.2)
        self.figure.canvas.mpl_connect("button_press_event", self.on_click)
        self.load_scan()

    # -- state --------------------------------------------------------------
    @property
    def scan(self):
        if self.mode == "new":
            return self.external_scan or {"id": -1, "name": "No new scan loaded",
                                         "points_local": np.empty((0, 2))}
        return self.scans[self.index]

    def set_mode(self, mode):
        if mode not in ("new", "evaluation"):
            raise ValueError("Choose new or evaluation mode")
        self.mode = mode
        self.load_scan()

    def set_new_scan(self, points, name="Captured scan"):
        """Keep ORIGINAL local measurements; never use a source file's pose."""
        points = np.array(points, dtype=float, copy=True)
        if points.ndim != 2 or points.shape[1] != 2 or not len(points) \
                or not np.all(np.isfinite(points)):
            raise ValueError("The new scan must contain finite local [x, y] points in metres.")
        signature = measurement_signature(points)
        duplicates = [scan["id"] for scan in self.scans
                      if measurement_signature(scan["points_local"]) == signature]
        if duplicates:
            raise ValueError(f"These measurements already belong to reference scan(s) "
                             f"{duplicates}. Use Evaluate Recorded Scans with exclusion "
                             "enabled to avoid self-matching.")
        points.setflags(write=False)
        self.external_scan = {"id": -1, "name": str(name), "points_local": points}
        self.mode = "new"
        self.load_scan(keep_guess=True)

    def initial_pose(self):
        return (float(self.sliders["x"].val), float(self.sliders["y"].val),
                wrap_angle(math.radians(self.sliders["theta"].val)))

    def set_initial_pose(self, pose):
        self.updating = True  # one redraw at the end, not one per slider
        try:
            for key, value in (("x", pose[0]), ("y", pose[1])):
                slider = self.sliders[key]
                slider.set_val(float(np.clip(value, slider.valmin,
                                             slider.valmax)))
            self.sliders["theta"].set_val(math.degrees(wrap_angle(pose[2])))
        finally:
            self.updating = False
        self.redraw_initial()

    def current_map(self):
        if self.mode == "new":
            if self.full_grid is None:
                self.full_grid = build_grid_map(self.scans, self.spec)[0]
            return self.full_grid, f"fixed reference map ({len(self.scans)} scans)"
        key = (self.scan["id"], self.exclude)
        if key not in self.maps:  # built once (about 0.3 s)
            self.maps[key] = reference_map(self.scans, self.scan["id"],
                                           self.spec, self.exclude)
        return self.maps[key]

    def load_scan(self, keep_guess=False):
        legend_items = self.legend_items if self.mode == "evaluation" else [
            item for item in self.legend_items
            if "Day 1" not in item.get_label() and item.get_label() != "measured pose"]
        self.axes.legend(handles=legend_items, loc="upper right", fontsize=8,
                         framealpha=0.9)
        self.grid_map, self.map_description = self.current_map()
        self.points = filter_ranges(self.scan["points_local"])
        self.result = None
        self.aligned_points.set_offsets(np.empty((0, 2)))
        set_pose_marker(self.aligned_marker, None)
        left, right, bottom, top = self.grid_map.extent
        self.image.set_data(part2.probabilities_to_uint8(
            self.grid_map.probability))
        self.image.set_extent(self.grid_map.extent)
        self.axes.set_xlim(left, right)
        self.axes.set_ylim(bottom, top)
        for key, low, high in (("x", left, right), ("y", bottom, top)):
            slider = self.sliders[key]
            slider.valmin, slider.valmax = low, high
            slider.ax.set_xlim(low, high)
        self.axes.set_title(f"{self.scan['name']} "
                            f"({len(self.points)} points) on the "
                            f"{self.map_description}", fontsize=10)
        if self.mode == "evaluation":
            self.scan_box.set_val(str(self.scan["id"]))
        set_pose_marker(self.day1_marker, self.scan.get("pose_rad"))
        measured = self.measured.get(self.scan["id"]) if self.mode == "evaluation" else None
        set_pose_marker(self.measured_marker,
                        measured["pose"] if measured else None)
        if keep_guess:
            self.redraw_initial()
        else:
            self.guess_seed = 0
            self.new_guess()

    def new_guess(self):
        """A reproducible guess, a moderate error away from the Day 1 pose."""
        if self.mode == "new":
            self.set_initial_pose((0.0, 0.0, 0.0))
            return
        rng = np.random.default_rng(7919 * self.scan["id"] + self.guess_seed)
        self.guess_seed += 1
        self.set_initial_pose(common.perturb_pose(
            self.scan["pose_rad"], *VIEWER_GUESS_ERROR, rng))

    def day1_guess(self):
        if self.mode == "evaluation":
            self.set_initial_pose(self.scan["pose_rad"])

    def use_result(self):
        if self.result is not None and self.result.pose is not None:
            self.set_initial_pose(self.result.pose)

    # -- events --------------------------------------------------------------
    def step_scan(self, step):
        if self.mode != "evaluation":
            return
        self.index = (self.index + step) % len(self.scans)
        self.load_scan()

    def on_scan_number(self, text):
        if self.mode != "evaluation":
            return
        ids = [scan["id"] for scan in self.scans]
        try:
            index = ids.index(int(text.strip()))
        except ValueError:
            self.info.set_text(f"No scan number {text!r} in this session.\n"
                               f"Scans: {', '.join(map(str, ids))}")
            self.figure.canvas.draw_idle()
            return
        if index != self.index:
            self.index = index
            self.load_scan()

    def on_exclude(self, _label):
        self.exclude = self.exclude_box.get_status()[0]
        self.load_scan(keep_guess=True)

    def on_slider(self, _value):
        if not self.updating:
            self.redraw_initial()

    def on_click(self, event):
        toolbar = getattr(self.figure.canvas, "toolbar", None)
        if event.inaxes is not self.axes or event.xdata is None or \
                (toolbar is not None and getattr(toolbar, "mode", "")):
            return  # not on the map, or the zoom / pan tool is active
        x, y, theta = self.initial_pose()
        if event.button == 1:
            self.set_initial_pose((event.xdata, event.ydata, theta))
        elif event.button == 3:
            self.set_initial_pose((x, y, math.atan2(event.ydata - y,
                                                    event.xdata - x)))

    # -- localisation --------------------------------------------------------
    def auto_align(self):
        self.info.set_text("Aligning ...")
        self.figure.canvas.draw()
        result = localise_scan(self.grid_map, self.points,
                               self.initial_pose(), self.config)
        self.show_result(result)
        return result

    def show_result(self, result):
        """Present a result on the GUI thread (also used by the Tk host)."""
        self.result = result
        self.aligned_points.set_offsets(
            np.empty((0, 2)) if result.pose is None
            else transform_points(self.points, result.pose))
        set_pose_marker(self.aligned_marker, result.pose)
        self.update_info()
        self.figure.canvas.draw_idle()

    # -- drawing -------------------------------------------------------------
    def redraw_initial(self):
        initial = self.initial_pose()
        self.initial_points.set_offsets(transform_points(self.points,
                                                         initial))
        set_pose_marker(self.initial_marker, initial)
        if self.result is not None and self.result.initial_pose != initial:
            self.result = None  # the shown result was for another guess
            self.aligned_points.set_offsets(np.empty((0, 2)))
            set_pose_marker(self.aligned_marker, None)
        self.update_info()
        self.figure.canvas.draw_idle()

    def update_info(self):
        import textwrap
        if self.mode == "new":
            x, y, heading = pose_to_day1(self.initial_pose())
            lines = [f"Initial: X {x:.3f} m, Y {y:.3f} m",
                     f"Heading {heading:.2f} deg", ""]
            if self.result is None:
                lines += ["Load or capture a new scan." if self.external_scan is None
                          else "Press Auto Align.",
                          "Left click: move. Right click: heading."]
            else:
                result = self.result
                lines += [f"status: {result.status}"]
                if result.pose is not None:
                    x, y, heading = pose_to_day1(result.pose)
                    lines += [f"estimated: X {x:.3f} m, Y {y:.3f} m",
                              f"Heading {heading:.2f} deg"]
                lines += [f"score: {result.score:.3f}",
                          f"time: {result.runtime_ms:.0f} ms (algorithm only)",
                          f"points: {result.points_used} after thinning"]
                if result.runner_up_score is not None:
                    lines.append(f"runner-up: {result.runner_up_score:.3f}")
                lines += textwrap.wrap(result.reason, 44)
            lines += ["", "Reference map stays fixed.",
                      "No measured pose: matching status", "does not establish physical accuracy."]
            self.info.set_text("\n".join(lines))
            return
        day1 = self.scan["pose_rad"]
        measured = self.measured.get(self.scan["id"])
        initial = self.initial_pose()
        offset = common.pose_error(initial, day1)
        lines = [f"{'':10s}{'x (m)':>8s}{'y (m)':>8s}{'th (deg)':>10s}",
                 pose_line("guess", initial), pose_line("Day 1", day1)]
        if measured:
            lines.append(pose_line("measured", measured["pose"]))
        lines += [f"guess offset from Day 1: {offset[0] * 100:.1f} cm, "
                  f"{offset[1]:.1f} deg", ""]
        result = self.result
        if result is None:
            lines += ["Press Auto Align.",
                      "Left click on the map: move the guess.",
                      "Right click: turn the guess towards",
                      "the click."]
        else:
            lines += [f"status : {result.status}"]
            if result.pose is not None:
                agreement = common.pose_error(result.pose, day1)
                lines += [pose_line("aligned", result.pose),
                          "Agreement with Day 1 ESTIMATE",
                          "(not ground-truth accuracy):",
                          f"  {agreement[0] * 100:.1f} cm, "
                          f"{agreement[1]:.2f} deg -> "
                          + ("within" if common.is_success(*agreement)
                             else "OUTSIDE")
                          + f" {common.SUCCESS_POSITION_M * 100:.0f} cm/"
                            f"{common.SUCCESS_ANGLE_DEG:.0f} deg"]
                if measured:
                    error = common.pose_error(result.pose, measured["pose"])
                    lines += [f"Error vs MEASURED pose (+-"
                              f"{measured['sigma_xy_m'] * 100:.0f} cm, +-"
                              f"{measured['sigma_theta_deg']:.0f} deg):",
                              f"  {error[0] * 100:.1f} cm, {error[1]:.2f} deg"]
            lines += [f"score  : {result.score:.3f}"
                      + (f" (runner-up {result.runner_up_score:.3f})"
                         if result.runner_up_score is not None else ""),
                      f"time   : {result.runtime_ms:.0f} ms (algorithm only)",
                      f"poses  : {result.candidates_evaluated} scored",
                      f"points : {result.points_used} after thinning"]
            if result.status != "ok":
                lines += textwrap.wrap(result.reason, 46)
        if not self.measured:
            lines += ["", "No measured pose in this session: only",
                      "agreement with Day 1 can be shown."]
        self.info.set_text("\n".join(lines))


def measurement_signature(points):
    """Detect copied/reordered recordings independently of IDs, names and poses.

    Session files are written at micrometre precision by Code 01. This is an
    exact-recording safeguard, not a detector of overlapping physical views.
    """
    points = np.round(np.asarray(points, dtype=float), 6)
    points[points == 0] = 0  # canonicalise negative zero
    order = np.lexsort((points[:, 1], points[:, 0]))
    return hashlib.sha256(points[order].tobytes()).digest()


def load_recorded_targets(path):
    """Load local XY measurements from another session, deliberately omit poses."""
    scans = part2.load_session_scans(Path(path), visible_only=False)
    if not scans:
        raise ValueError("The recorded session contains no scans.")
    return [{"name": scan["name"], "id": scan["id"],
             "points_local": scan["points_local"].copy()} for scan in scans]


def run_viewer(path, measured_file=None):
    global ACTIVE_VIEWER
    import matplotlib.pyplot as plt
    print(f"=== Day 2 Phase 1: Auto Align viewer - {path.name} "
          f"(real LD06 data, read only) ===")
    scans, measured, measured_path = load_session(path, measured_file)
    print(f"{len(scans)} scans: "
          f"{', '.join(str(scan['id']) for scan in scans)}; measured poses: "
          + (f"{len(measured)} ({measured_path.name})" if measured_path
             else "none"))
    ACTIVE_VIEWER = AutoAlignViewer(path.name, scans, measured)
    plt.show()


VALID_RUN_MODES = ("application", "viewer", "evaluation")


def main(session_file=None):
    """Run RUN_MODE on session_file (default: SESSION_FILE)."""
    if RUN_MODE not in VALID_RUN_MODES:
        raise ValueError(f"RUN_MODE = {RUN_MODE!r} is not valid; choose one "
                         f"of {', '.join(map(repr, VALID_RUN_MODES))}")
    path = session_path(session_file)
    if not path.is_file():
        available = sorted(p.name for p in common.SCANS_DIR.glob("*.json"))
        print(f"Session file not found: {path}")
        print(f"  Files available in scans/: "
              f"{', '.join(available) if available else 'none'}")
        return
    try:
        if RUN_MODE == "application":
            from desktop_app import launch
            launch(path, initial_page="Grid Localisation")
        elif RUN_MODE == "evaluation":
            run_evaluation(path, MEASURED_POSES_FILE)
        else:
            run_viewer(path, MEASURED_POSES_FILE)
    except (part2.SessionError, common.MeasuredPoseError) as error:
        print(f"Cannot use {path.name}: {error}")


def start_from_pycharm():
    """Ask for the session file (as Parts 2 and 3 do), then run main().

    Cancelling the dialog uses the default SESSION_FILE.
    """
    chosen = choose_session_file() if ASK_FOR_SESSION_FILE else None
    if ASK_FOR_SESSION_FILE and chosen is None:
        print(f"No file selected: using the default {SESSION_FILE}.")
    main(chosen)


# ---------------------------------------------------------------------------
# Automated tests (no window): python -m unittest 04_grid_localisation -v
# ---------------------------------------------------------------------------
CLASSROOM_FINAL = common.SCANS_DIR / "classroom_final.json"


def tiny_map(state, cell_size=0.1, x_min=0.0, y_min=0.0):
    """A GridMap from a hand-written state array (tests only)."""
    state = np.asarray(state, dtype=np.int8)
    return GridMap(state=state, probability=np.full(state.shape, 0.5),
                   x_min=x_min, y_min=y_min, cell_size=cell_size)


# TEST FIXTURE ONLY - never used by the viewer or the results. Real scans
# have no measured pose yet, so the MATHEMATICS (known-pose recovery,
# wrap-around, bounds) is checked on scans simulated with Part 1's ray
# caster in its asymmetric L-shaped classroom, where the true pose is
# known exactly. The fixture map is built by Part 2 from 8 scans at
# mapping poses; the scans to localise come from OTHER poses and noise.
FIXTURE_MAPPING_POSES_DEG = ((1.0, 1.0, 0.0), (4.0, 0.8, 90.0),
                             (7.2, 2.5, 180.0), (6.5, 4.2, -45.0),
                             (3.2, 3.5, 30.0), (1.0, 4.0, -60.0),
                             (2.5, 6.2, 0.0), (4.4, 5.8, 120.0))
FIXTURE_TEST_POSES_DEG = ((2.3, 3.2, 30.0), (5.6, 2.2, -60.0),
                          (1.2, 6.0, 135.0), (7.2, 1.8, 100.0),
                          (3.4, 0.8, -170.0), (3.0, 4.8, 10.0))
TEST_POSE = pose_from_day1(FIXTURE_TEST_POSES_DEG[0])  # (2.3, 3.2, 30 deg)
# The fixture's walls lie on whole metres = ON the 10 cm cell borders, so
# Part 2's map places them up to half a cell outwards: known-pose recovery
# is checked to 6 cm on that map, and to 1.5 cm on a CONTROL map shifted
# by 5 cm (walls at cell centres), which isolates the search mathematics.
MAP_BIAS_TOLERANCE_M = 0.06
INIT_LEVELS = (("small", 0.10, 3.0), ("moderate", 0.30, 10.0),
               ("large", 0.75, 25.0), ("beyond_window", 1.50, 45.0))


def fixture_mapping_scans():
    walls = day1("part1").CLASSROOM_WALLS
    return [{"id": index + 1, "name": f"Map scan {index + 1}",
             "pose": pose_deg, "points_local": common.simulate_scan(
                 walls, pose_from_day1(pose_deg), noise_mm=10.0,
                 seed=100 + index)}
            for index, pose_deg in enumerate(FIXTURE_MAPPING_POSES_DEG)]


def fixture_scan(pose, noise_mm=10.0, seed=0, clutter=0.0):
    points = common.simulate_scan(day1("part1").CLASSROOM_WALLS, pose,
                                  noise_mm=noise_mm, seed=500 + seed)
    if clutter:
        points = common.add_clutter(points, clutter, seed=900 + seed)
    return points


def write_session_subset(folder, scan_ids, name="subset.json"):
    """A copy of some classroom_final.json scans as a new session file."""
    data = json.loads(CLASSROOM_FINAL.read_text(encoding="utf-8"))
    data["scans"] = [entry for entry in data["scans"]
                     if entry["id"] in scan_ids]
    path = Path(folder) / name
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def write_measured_poses(path, poses):
    """poses: {scan_id: (x, y, theta_deg)}; sigma 2 cm / 1 deg."""
    Path(path).write_text(json.dumps({
        "format": common.MEASURED_POSES_FORMAT, "version": 1,
        "method": "test file", "poses": [
            {"scan_id": scan_id, "x_m": x, "y_m": y, "theta_deg": theta,
             "sigma_xy_m": 0.02, "sigma_theta_deg": 1.0}
            for scan_id, (x, y, theta) in poses.items()]}),
        encoding="utf-8")


class FixtureTestCase(unittest.TestCase):
    """Builds the known-pose fixture maps once for all tests of a class."""

    @classmethod
    def setUpClass(cls):
        scans = fixture_mapping_scans()
        cls.grid_map, cls.log_odds_map = build_grid_map(scans)
        spec = cls.grid_map.spec()
        cls.centred_map = build_grid_map(
            scans, dict(spec, x_min=spec["x_min"] - 0.05,
                        y_min=spec["y_min"] - 0.05))[0]
        cls.scan = fixture_scan(TEST_POSE, seed=0)

    def assertRecovered(self, result, truth, position_m, angle_deg=1.0):
        self.assertIsNotNone(result.pose, result.reason)
        position, angle = common.pose_error(result.pose, truth)
        self.assertLessEqual(position, position_m, result)
        self.assertLessEqual(angle, angle_deg, result)


class GridMapTests(FixtureTestCase):

    def test_states_follow_part2_classification(self):
        counts = part2.classify_probabilities(self.log_odds_map)
        state = self.grid_map.state
        self.assertEqual(int(np.sum(state == FREE_STATE)), counts["free"])
        self.assertEqual(int(np.sum(state == OCCUPIED_STATE)),
                         counts["occupied"])
        self.assertEqual(int(np.sum(state == UNKNOWN_STATE)),
                         counts["uncertain"] + counts["unobserved"])

    def test_cell_coordinates_match_part2_world_to_grid(self):
        rng = np.random.default_rng(1)
        points = rng.uniform(-0.5, 8.5, (500, 2))
        u, v = self.grid_map.cell_coordinates(points)
        i, j = part2.world_to_grid(self.grid_map.spec(), points[:, 0],
                                   points[:, 1])
        np.testing.assert_array_equal(np.floor(u + 0.5).astype(int), i)
        np.testing.assert_array_equal(np.floor(v + 0.5).astype(int), j)
        centre = np.array(part2.grid_to_world(self.grid_map.spec(), 7, 3))
        np.testing.assert_allclose(self.grid_map.cell_coordinates(centre),
                                   (7.0, 3.0), atol=1e-9)

    def test_extent_matches_part2(self):
        self.assertEqual(self.grid_map.extent,
                         part2.grid_extent(self.grid_map.spec()))

    def test_distance_transform_matches_brute_force(self):
        rng = np.random.default_rng(2)
        state = np.where(rng.random((12, 15)) < 0.08, OCCUPIED_STATE,
                         FREE_STATE)
        grid_map = tiny_map(state)
        distance = grid_map.distance_to_occupied(0.45)
        occupied = np.argwhere(state == OCCUPIED_STATE)
        for j in range(12):
            for i in range(15):
                exact = np.min(np.hypot(occupied[:, 0] - j,
                                        occupied[:, 1] - i)) * 0.1
                if exact <= 0.45:
                    self.assertAlmostEqual(distance[j, i], exact)
                else:
                    self.assertEqual(distance[j, i], np.inf)

    def test_score_field_separates_occupied_free_unknown(self):
        state = np.full((5, 30), UNKNOWN_STATE)
        state[:, 0] = OCCUPIED_STATE
        state[:, 10:20] = FREE_STATE
        field = tiny_map(state).score_field(sigma_m=0.1, free_penalty=0.5)
        self.assertAlmostEqual(field[2, 0], 1.0)            # occupied
        self.assertAlmostEqual(field[2, 1], math.exp(-0.5))  # next to it
        self.assertAlmostEqual(field[2, 25], 0.0)           # far unknown
        self.assertAlmostEqual(field[2, 15], -0.5)          # far free
        self.assertLess(field[2, 15], field[2, 25])         # free < unknown

    def test_free_penalty_fades_near_walls(self):
        state = np.full((3, 10), FREE_STATE)
        state[:, 0] = OCCUPIED_STATE
        field = tiny_map(state).score_field(sigma_m=0.1, free_penalty=0.5)
        self.assertGreater(field[1, 1], 0.3)  # one cell from the wall
        self.assertAlmostEqual(field[1, 9], -0.5)

    def test_sample_field_bilinear_and_out_of_bounds(self):
        grid_map = tiny_map(np.zeros((3, 4)))
        field = np.arange(12, dtype=float).reshape(3, 4)
        centres = np.array([[0.05, 0.05], [0.15, 0.05], [0.05, 0.15]])
        values, inside = sample_field(field, grid_map, centres)
        np.testing.assert_allclose(values, [0.0, 1.0, 4.0])
        self.assertTrue(inside.all())
        values, _ = sample_field(field, grid_map, np.array([[0.10, 0.10]]))
        self.assertAlmostEqual(values[0], (0 + 1 + 4 + 5) / 4)
        values, inside = sample_field(field, grid_map, np.array(
            [[-0.01, 0.1], [0.41, 0.1], [0.1, 0.31], [5.0, 5.0]]))
        np.testing.assert_array_equal(values, 0.0)
        self.assertFalse(inside.any())

    def test_tiny_grid_refused(self):
        with self.assertRaises(ValueError):
            GridMap.from_part2(
                {"log_odds": np.zeros((1, 5)), "hits": np.zeros((1, 5)),
                 "misses": np.zeros((1, 5))},
                {"x_min": 0, "y_min": 0, "cell_size": 0.1})


class KnownPoseTests(FixtureTestCase):
    """Search mathematics on the simulated fixture (true pose known)."""

    def localise(self, initial, grid_map=None, scan=None, **options):
        config = GridSearchConfig(**options) if options else None
        return localise_scan(grid_map or self.grid_map,
                             self.scan if scan is None else scan, initial,
                             config, true_pose=TEST_POSE)

    def test_identity_pose_recovery(self):
        result = self.localise(TEST_POSE)
        self.assertEqual(result.status, "ok")
        self.assertRecovered(result, TEST_POSE, MAP_BIAS_TOLERANCE_M)

    def test_exact_recovery_on_centre_aligned_control_map(self):
        initial = (TEST_POSE[0] + 0.4, TEST_POSE[1] - 0.3,
                   TEST_POSE[2] + math.radians(12.0))
        result = self.localise(initial, grid_map=self.centred_map)
        self.assertRecovered(result, TEST_POSE, 0.015, 0.5)

    def test_translation_recovery(self):
        result = self.localise((TEST_POSE[0] + 0.6, TEST_POSE[1] - 0.45,
                                TEST_POSE[2]))
        self.assertRecovered(result, TEST_POSE, MAP_BIAS_TOLERANCE_M)
        self.assertTrue(result.success)

    def test_rotation_recovery(self):
        result = self.localise((TEST_POSE[0], TEST_POSE[1],
                                TEST_POSE[2] - math.radians(25.0)))
        self.assertRecovered(result, TEST_POSE, MAP_BIAS_TOLERANCE_M)

    def test_combined_translation_and_rotation(self):
        result = self.localise((TEST_POSE[0] - 0.5, TEST_POSE[1] + 0.5,
                                TEST_POSE[2] + math.radians(20.0)))
        self.assertRecovered(result, TEST_POSE, MAP_BIAS_TOLERANCE_M)
        self.assertGreater(result.initial_position_error_m, 0.7)

    def test_heading_across_plus_minus_180(self):
        truth = pose_from_day1((3.4, 0.8, 178.0))
        scan = fixture_scan(truth, seed=4)
        initial = (truth[0] + 0.2, truth[1], math.radians(-172.0))  # +10 deg
        result = localise_scan(self.grid_map, scan, initial,
                               true_pose=truth)
        self.assertRecovered(result, truth, MAP_BIAS_TOLERANCE_M)
        self.assertTrue(-math.pi <= result.pose[2] < math.pi)

    def test_output_fields(self):
        result = self.localise(TEST_POSE)
        self.assertEqual(len(result.pose), 3)
        self.assertEqual(result.initial_pose, TEST_POSE)
        self.assertEqual(result.candidates_evaluated,
                         planned_candidates(GridSearchConfig()))
        self.assertGreater(result.runtime_ms, 0.0)
        self.assertTrue(0.0 < result.score <= 1.0)
        self.assertEqual(result.out_of_map_fraction, 0.0)
        self.assertEqual(result.method, "grid_exhaustive")

    def test_deterministic(self):
        initial = (TEST_POSE[0] + 0.3, TEST_POSE[1], TEST_POSE[2])
        first, second = self.localise(initial), self.localise(initial)
        self.assertEqual(first.pose, second.pose)
        self.assertEqual(first.score, second.score)

    def test_true_pose_does_not_change_the_estimate(self):
        initial = (TEST_POSE[0] + 0.3, TEST_POSE[1], TEST_POSE[2])
        blind = localise_scan(self.grid_map, self.scan, initial)
        informed = localise_scan(self.grid_map, self.scan, initial,
                                 true_pose=(0.0, 0.0, 0.0))
        self.assertEqual(blind.pose, informed.pose)
        self.assertIsNone(blind.position_error_m)

    def test_inputs_not_modified(self):
        scan = self.scan.copy()
        state = self.grid_map.state.copy()
        self.localise(TEST_POSE)
        np.testing.assert_array_equal(scan, self.scan)
        np.testing.assert_array_equal(state, self.grid_map.state)

    def test_empty_and_invalid_scans(self):
        for scan in (np.empty((0, 2)), np.full((100, 2), np.nan),
                     np.full((100, 2), 0.02)):  # all inside the housing
            result = self.localise(TEST_POSE, scan=scan)
            self.assertEqual(result.status, "insufficient_points")
            self.assertIsNone(result.pose)
            self.assertFalse(result.success)

    def test_insufficient_angular_coverage(self):
        local = self.scan[np.abs(np.arctan2(self.scan[:, 1],
                                            self.scan[:, 0])) < 0.35]
        result = self.localise(TEST_POSE, scan=local)
        self.assertEqual(result.status, "insufficient_coverage")
        self.assertIsNone(result.pose)

    def test_unmatched_scan_is_reported(self):
        octagon = common.simulate_scan(day1("part1").OCTAGON_ROOM_WALLS,
                                       (0.0, 0.0, 0.0), seed=3)
        result = self.localise(TEST_POSE, scan=octagon)
        self.assertNotEqual(result.status, "ok", result.reason)

    def test_points_outside_the_map(self):
        result = localise_scan(self.grid_map, self.scan, (40.0, 40.0, 0.0))
        self.assertEqual(result.status, "out_of_map")
        self.assertEqual(result.out_of_map_fraction, 1.0)

    def test_search_stays_within_bounds(self):
        initial = (TEST_POSE[0] + 0.3, TEST_POSE[1] + 0.3, TEST_POSE[2])
        config = GridSearchConfig(xy_window_m=0.2, theta_window_deg=4.0)
        result = localise_scan(self.grid_map, self.scan, initial, config)
        reach_m, reach_deg = search_reach(config)
        self.assertAlmostEqual(reach_m, 0.2 + 0.10 + 0.025)
        self.assertLessEqual(abs(result.pose[0] - initial[0]), reach_m + 1e-9)
        self.assertLessEqual(abs(result.pose[1] - initial[1]), reach_m + 1e-9)
        self.assertLessEqual(
            common.angle_difference_deg(result.pose[2], initial[2]),
            reach_deg + 1e-9)
        # The true pose is 0.42 m away, outside this window.
        self.assertGreater(common.pose_error(TEST_POSE, initial)[0], reach_m)

    def test_true_pose_outside_window_is_not_a_success(self):
        initial = (TEST_POSE[0] + 1.5, TEST_POSE[1], TEST_POSE[2])
        result = localise_scan(self.grid_map, self.scan, initial,
                               GridSearchConfig(xy_window_m=0.5),
                               true_pose=TEST_POSE)
        self.assertFalse(result.success)
        self.assertNotEqual(result.status, "ok")

    def test_oversized_search_is_refused(self):
        config = GridSearchConfig(xy_window_m=10.0, theta_window_deg=180.0,
                                  levels=(SearchLevel(0.01, 0.1, 0.1),))
        with self.assertRaises(ValueError):
            localise_scan(self.grid_map, self.scan, TEST_POSE, config)

    def test_clutter_and_noise(self):
        scan = fixture_scan(TEST_POSE, noise_mm=25.0, seed=0, clutter=0.15)
        result = self.localise((TEST_POSE[0] + 0.25, TEST_POSE[1] - 0.15,
                                TEST_POSE[2] + math.radians(10)), scan=scan)
        self.assertRecovered(result, TEST_POSE, common.SUCCESS_POSITION_M,
                             2.0)

    def test_initialisation_sensitivity(self):
        # Inside the window every level succeeds; beyond it, none does.
        for case in (1, 3):
            truth = pose_from_day1(FIXTURE_TEST_POSES_DEG[case])
            scan = fixture_scan(truth, seed=case)
            for name, error_m, error_deg in INIT_LEVELS:
                initial = common.perturb_pose(truth, error_m, error_deg,
                                              np.random.default_rng(case))
                result = localise_scan(self.grid_map, scan, initial,
                                       true_pose=truth)
                with self.subTest(case=case, condition=name):
                    self.assertEqual(result.success,
                                     name != "beyond_window", result)


class RealDataTestCase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not CLASSROOM_FINAL.exists():
            raise unittest.SkipTest("classroom_final.json absent")
        cls.fingerprint = part2.file_sha256(CLASSROOM_FINAL)
        cls.scans, _, _ = load_session(CLASSROOM_FINAL)
        cls.spec = session_spec(cls.scans)

    @classmethod
    def tearDownClass(cls):
        # The original measurements are never modified.
        assert part2.file_sha256(CLASSROOM_FINAL) == cls.fingerprint

    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.folder.cleanup()


class RealSessionTests(RealDataTestCase):

    def test_default_session_is_classroom_final(self):
        self.assertEqual(SESSION_FILE, "classroom_final.json")
        self.assertEqual(session_path(), CLASSROOM_FINAL)
        self.assertEqual(len(self.scans), 16)

    def test_full_map_is_part2_verified_map(self):
        _, log_odds_map = build_grid_map(self.scans)
        self.assertEqual(
            hashlib.sha256(log_odds_map["log_odds"].tobytes()).hexdigest(),
            part2.VERIFIED_LOG_ODDS_SHA256)

    def test_real_map_states_follow_part2_classification(self):
        grid_map, log_odds_map = build_grid_map(self.scans)
        counts = part2.classify_probabilities(log_odds_map)
        self.assertEqual(int(np.sum(grid_map.state == OCCUPIED_STATE)),
                         counts["occupied"])
        self.assertEqual(int(np.sum(grid_map.state == FREE_STATE)),
                         counts["free"])

    def test_selected_scan_is_excluded_from_its_map(self):
        full = build_grid_map(self.scans, self.spec)[0]
        without, text = reference_map(self.scans, 5, self.spec)
        self.assertEqual(without.spec(), full.spec())
        self.assertFalse(np.array_equal(without.state, full.state))
        self.assertIn("excluded", text)
        included, text = reference_map(self.scans, 5, self.spec,
                                       exclude=False)
        np.testing.assert_array_equal(included.state, full.state)
        self.assertIn("SELF-MATCHING", text)
        with self.assertRaises(KeyError):
            reference_map(self.scans, 999, self.spec)

    def test_single_scan_session_says_self_matching(self):
        scans = self.scans[:1]
        _, text = reference_map(scans, scans[0]["id"], session_spec(scans))
        self.assertIn("SELF-MATCHING", text)

    def test_leave_one_out_agrees_with_day1_estimate(self):
        scan = next(s for s in self.scans if s["id"] == 4)
        grid_map, _ = reference_map(self.scans, 4, self.spec)
        initial = common.perturb_pose(scan["pose_rad"], 0.3, 10.0,
                                      np.random.default_rng(0))
        result = localise_scan(grid_map, scan["points_local"], initial)
        self.assertEqual(result.status, "ok", result.reason)
        # Agreement with the previous estimate, NOT accuracy.
        self.assertTrue(common.is_success(*common.pose_error(
            result.pose, scan["pose_rad"])), result)

    def test_doubled_revolution_scan_is_thinned(self):
        # At most one point per 5 cm cell: where the two revolutions
        # overlap they merge; at long range the second one samples BETWEEN
        # the first one's points (0.8 deg = 7 cm at 5 m) and is kept.
        scan = next(s for s in self.scans if s["id"] == 13)
        raw = filter_ranges(scan["points_local"])
        self.assertGreater(len(raw), 900)
        points, _ = prepare_points(raw, GridSearchConfig())
        cells = np.unique(np.floor(points / 0.05).astype(int), axis=0)
        self.assertEqual(len(cells), len(points))
        self.assertLess(len(points), 0.6 * len(raw))

    def test_another_session_file_can_be_used(self):
        path = write_session_subset(self.folder.name, {1, 2, 3, 4})
        scans, measured, measured_path = load_session(path)
        self.assertEqual([scan["id"] for scan in scans], [1, 2, 3, 4])
        self.assertEqual(measured, {})
        self.assertIsNone(measured_path)
        self.assertEqual(session_path(str(path)), path)

    def test_measured_poses_are_found_next_to_the_session(self):
        path = write_session_subset(self.folder.name, {1, 2})
        write_measured_poses(common.measured_poses_path(path),
                             {2: (0.10, 2.05, 1.5)})
        _, measured, measured_path = load_session(path)
        self.assertEqual(list(measured), [2])
        self.assertEqual(measured_path, common.measured_poses_path(path))

    def test_missing_explicit_measured_file_is_an_error(self):
        with self.assertRaises(common.MeasuredPoseError):
            load_session(CLASSROOM_FINAL, Path(self.folder.name) / "no.json")

    def test_session_without_visible_scans_is_refused(self):
        data = json.loads(CLASSROOM_FINAL.read_text(encoding="utf-8"))
        for entry in data["scans"]:
            entry["visible"] = False
        path = Path(self.folder.name) / "hidden.json"
        path.write_text(json.dumps(data), encoding="utf-8")
        with self.assertRaises(part2.SessionError):
            load_session(path)


class EvaluationTests(RealDataTestCase):

    def run_small(self, session, measured=None, scan_ids=frozenset({2, 4})):
        conditions = (REAL_INIT_CONDITIONS[0], REAL_INIT_CONDITIONS[2],
                      REAL_INIT_CONDITIONS[-1])
        with contextlib.redirect_stdout(io.StringIO()) as output:
            result = run_evaluation(session, measured, self.folder.name,
                                    repeats=2, conditions=conditions,
                                    scan_ids=scan_ids)
        return result, output.getvalue()

    def test_results_files_and_labels(self):
        (rows, by_condition, by_scan), text = self.run_small(CLASSROOM_FINAL)
        stem = results_stem(CLASSROOM_FINAL)
        data = json.loads((Path(self.folder.name) / f"{stem}.json")
                          .read_text(encoding="utf-8"))
        with open(Path(self.folder.name) / f"{stem}.csv",
                  encoding="utf-8") as file:
            table = list(csv.DictReader(file))
        self.assertEqual(len(table), len(rows))
        self.assertEqual(tuple(table[0]), CSV_COLUMNS)
        self.assertEqual(len(rows), 2 * (1 + 2 + 2))
        self.assertTrue(all(row["deterministic"] for row in rows))
        self.assertIn("not ground-truth accuracy",
                      data["meaning"]["day1_deviation"])
        self.assertIn("No independently measured real pose",
                      data["ground_truth_status"])
        self.assertIn("AGREEMENT", text)
        self.assertTrue(all(row["measured_error_m"] is None for row in rows))
        self.assertEqual(sum(item["runs"] for item in by_condition),
                         len(rows))
        self.assertEqual([item["scan_id"] for item in by_scan], [2, 4])
        self.assertTrue(all("excluded" in row["reference_map"]
                            for row in rows))

    def test_starts_inside_the_window_converge(self):
        (_, _, by_scan), _ = self.run_small(CLASSROOM_FINAL)
        for item in by_scan:
            self.assertEqual(item["converged_starts"],
                             item["starts_inside_window"], item)

    def test_measured_poses_are_evaluated_separately(self):
        path = write_session_subset(self.folder.name, set(range(1, 30)))
        scan = next(s for s in self.scans if s["id"] == 2)
        x, y, theta = pose_to_day1(scan["pose_rad"])
        measured_file = Path(self.folder.name) / "measured.json"
        write_measured_poses(measured_file, {2: (x + 0.03, y, theta)})
        (rows, by_condition, _), _ = self.run_small(path, measured_file)
        measured = [row for row in rows if row["scan_id"] == 2]
        others = [row for row in rows if row["scan_id"] != 2]
        self.assertTrue(all(row["measured_error_m"] is not None
                            for row in measured))
        self.assertTrue(all(row["measured_error_m"] is None
                            for row in others))
        self.assertEqual(measured[0]["measured_sigma_xy_m"], 0.02)
        self.assertEqual(sum(item["measured_runs"] for item in by_condition),
                         len(measured))


class ViewerTests(RealDataTestCase):

    def setUp(self):
        super().setUp()
        import matplotlib.pyplot as plt
        plt.switch_backend("Agg")
        self.viewer = AutoAlignViewer(CLASSROOM_FINAL.name, self.scans)

    def tearDown(self):
        import matplotlib.pyplot as plt
        plt.close("all")
        super().tearDown()

    def test_starts_on_the_first_scan_with_it_excluded(self):
        viewer = self.viewer
        self.assertEqual(viewer.scan["id"], 1)
        self.assertIn("excluded", viewer.map_description)
        offset = common.pose_error(viewer.initial_pose(),
                                   viewer.scan["pose_rad"])
        self.assertAlmostEqual(offset[0], VIEWER_GUESS_ERROR[0], places=6)

    def test_auto_align_and_report(self):
        viewer = self.viewer
        result = viewer.auto_align()
        self.assertIsNotNone(result.pose)
        np.testing.assert_allclose(
            viewer.aligned_points.get_offsets(),
            transform_points(viewer.points, result.pose))
        text = viewer.info.get_text()
        for words in ("status", "score", "time", "Agreement with Day 1",
                      "not ground-truth accuracy"):
            self.assertIn(words, text)

    def test_choose_scan_by_number_and_arrows(self):
        viewer = self.viewer
        viewer.on_scan_number("13")
        self.assertEqual(viewer.scan["id"], 13)
        viewer.step_scan(+1)
        self.assertEqual(viewer.scan["id"], 16)
        viewer.on_scan_number("99")
        self.assertEqual(viewer.scan["id"], 16)
        self.assertIn("No scan number", viewer.info.get_text())

    def test_exclude_toggle_keeps_the_guess(self):
        viewer = self.viewer
        guess = viewer.initial_pose()
        viewer.exclude_box.set_active(0)
        self.assertFalse(viewer.exclude)
        self.assertIn("SELF-MATCHING", viewer.map_description)
        self.assertEqual(viewer.initial_pose(), guess)

    def test_manual_guess_with_clicks_and_buttons(self):
        viewer = self.viewer
        viewer.on_click(mock.Mock(inaxes=viewer.axes, button=1, xdata=1.0,
                                  ydata=2.0))
        self.assertAlmostEqual(viewer.initial_pose()[0], 1.0)
        self.assertAlmostEqual(viewer.initial_pose()[1], 2.0)
        viewer.on_click(mock.Mock(inaxes=viewer.axes, button=3, xdata=1.0,
                                  ydata=5.0))
        self.assertAlmostEqual(viewer.initial_pose()[2], math.pi / 2)
        viewer.day1_guess()
        np.testing.assert_allclose(viewer.initial_pose(),
                                   viewer.scan["pose_rad"], atol=1e-9)

    def test_use_result_clears_the_old_result(self):
        viewer = self.viewer
        result = viewer.auto_align()
        viewer.use_result()
        np.testing.assert_allclose(viewer.initial_pose(), result.pose,
                                   atol=1e-9)
        self.assertIsNone(viewer.result)  # it was computed from the old guess
        self.assertEqual(len(viewer.aligned_points.get_offsets()), 0)

    def test_measured_pose_is_shown_when_available(self):
        x, y, theta = pose_to_day1(self.scans[0]["pose_rad"])
        measured = {1: {"pose": pose_from_day1((x + 0.02, y, theta)),
                        "sigma_xy_m": 0.02, "sigma_theta_deg": 1.0,
                        "method": "test"}}
        viewer = AutoAlignViewer(CLASSROOM_FINAL.name, self.scans, measured)
        viewer.auto_align()
        self.assertIn("Error vs MEASURED pose", viewer.info.get_text())
        self.assertEqual(len(viewer.measured_marker[0].get_xdata()), 1)


class MainTests(RealDataTestCase):

    def run_main(self, mode, session_file=None):
        import matplotlib.pyplot as plt
        plt.switch_backend("Agg")
        module = sys.modules[__name__]
        output = io.StringIO()
        with mock.patch.object(module, "RUN_MODE", mode), \
                mock.patch.object(plt, "show"), \
                contextlib.redirect_stdout(output):
            main(session_file)
        plt.close("all")
        return output.getvalue()

    def test_viewer_mode_uses_real_data(self):
        text = self.run_main("viewer")
        self.assertIn("classroom_final.json (real LD06 data", text)
        self.assertEqual(sys.modules[__name__].ACTIVE_VIEWER.session_name,
                         "classroom_final.json")

    def test_invalid_run_mode(self):
        with self.assertRaises(ValueError):
            self.run_main("simulation")

    def test_missing_session_file(self):
        self.assertIn("Session file not found",
                      self.run_main("viewer", "no_such_session.json"))

    def test_cancelled_dialog_uses_default_session(self):
        module = sys.modules[__name__]
        with mock.patch.object(module, "choose_session_file",
                               return_value=None), \
                mock.patch.object(module, "main") as fake_main, \
                contextlib.redirect_stdout(io.StringIO()) as output:
            start_from_pycharm()
        fake_main.assert_called_once_with(None)
        self.assertIn("using the default classroom_final.json",
                      output.getvalue())

    def test_chosen_file_is_used(self):
        module = sys.modules[__name__]
        chosen = write_session_subset(self.folder.name, {1, 2})
        with mock.patch.object(module, "choose_session_file",
                               return_value=chosen), \
                mock.patch.object(module, "main") as fake_main:
            start_from_pycharm()
        fake_main.assert_called_once_with(chosen)


if __name__ == "__main__":
    start_from_pycharm()
