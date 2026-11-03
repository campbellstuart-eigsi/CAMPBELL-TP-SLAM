"""Day 2 - shared tools for the three localisation methods.

Used by 04_grid_localisation.py now, and later by vector localisation,
scan-to-scan matching and the Day 3 SLAM. Contents:
    1. access to the Day 1 programs (imported on demand, never modified),
    2. pose mathematics in RADIANS, and conversion from/to Day 1 poses,
    3. scan preparation (range filter, thinning, angular coverage),
    4. the result record shared by every localisation method,
    5. reproducible data: perturbed initial poses, session loading, and
       simulated scans with KNOWN poses (used only by the unit tests),
    6. independently measured real poses (file format and loader).

Conventions (the same as Day 1, checked by the tests below):
    - sensor frame: +X = LiDAR front, +Y = LiDAR left, metres,
    - map frame: metres, +Y upwards, origin = Scan 1 of the session,
    - a pose is (x, y, theta): the LiDAR position in the map and its heading,
      counter-clockwise from the map's +X axis. Day 2 code uses RADIANS;
      Day 1 files and functions use degrees, so convert at the boundary
      with pose_from_day1() / pose_to_day1(),
    - local -> map:  x_m = x + cos(theta) x_l - sin(theta) y_l
                     y_m = y + sin(theta) x_l + cos(theta) y_l
    - the robot pose IS the LiDAR pose: no sensor offset is known.

This module needs only NumPy: no Matplotlib, no files, no GUI state, so a
live robot program can use it. Day 1 modules are imported only by the
functions that need their data (simulation, session loading).

Automated tests, from the project folder:
    python -m unittest localisation_common -v
"""

import importlib
import json
import math
import tempfile
import sys
import time
import unittest
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

import numpy as np

PROJECT_DIR = Path(__file__).resolve().parent
SCANS_DIR = PROJECT_DIR / "scans"
RESULTS_DIR = PROJECT_DIR / "results"

# Provisional success thresholds for the Day 2 experiments. They are our
# OWN experimental choice, not values given by the course: 10 cm is one
# grid cell, 5 deg moves a point 9 cm at 1 m range.
SUCCESS_POSITION_M = 0.10
SUCCESS_ANGLE_DEG = 5.0

# Measurements kept for matching (the same limits as Part 3).
MIN_RANGE_M = 0.10   # closer: inside the robot / LiDAR housing
MAX_RANGE_M = 12.0   # LD06 maximum range


# ---------------------------------------------------------------------------
# 1. Day 1 programs (names start with a digit: loaded with importlib)
# ---------------------------------------------------------------------------
DAY1_MODULES = {"part1": "01_lidar_visualizer", "part2": "02_grid_mapping",
                "part3": "03_vector_mapping"}


def day1(name):
    """Return a Day 1 module ("part1", "part2" or "part3"), imported once.

    Importing runs nothing: each Day 1 program only defines functions and
    settings until its main() is called.
    """
    if str(PROJECT_DIR) not in sys.path:
        sys.path.insert(0, str(PROJECT_DIR))
    return importlib.import_module(DAY1_MODULES[name])


# ---------------------------------------------------------------------------
# 2. Pose mathematics (radians)
# ---------------------------------------------------------------------------
def wrap_angle(angle_rad):
    """The same direction as an angle in [-pi, pi) (numbers or arrays)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def angle_difference_deg(a_rad, b_rad):
    """Smallest difference between two headings, in degrees (0 to 180)."""
    return abs(math.degrees(wrap_angle(a_rad - b_rad)))


def pose_from_day1(pose_deg):
    """Day 1 pose (x_m, y_m, theta_deg) -> Day 2 pose (x, y, theta_rad)."""
    x, y, theta_deg = pose_deg
    return (float(x), float(y), wrap_angle(math.radians(float(theta_deg))))


def pose_to_day1(pose_rad):
    """Day 2 pose (x, y, theta_rad) -> Day 1 pose (x_m, y_m, theta_deg)."""
    x, y, theta = pose_rad
    return (float(x), float(y), math.degrees(wrap_angle(float(theta))))


def rotation(theta_rad):
    """2x2 rotation matrix R(theta), counter-clockwise."""
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return np.array([[c, -s], [s, c]])


def transform_points(points, pose):
    """Sensor-frame (N, 2) points -> map frame: R(theta) p + (x, y)."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return points @ rotation(pose[2]).T + np.array(pose[:2], dtype=float)


def global_to_local(points, pose):
    """Inverse of transform_points(): R(-theta) (p - (x, y))."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return (points - np.array(pose[:2], dtype=float)) @ rotation(pose[2])


def compose_poses(a, b):
    """a (+) b: pose b, expressed in frame a, moved into a's parent frame.

    transform_points(p, compose_poses(a, b)) equals
    transform_points(transform_points(p, b), a).
    """
    x, y = transform_points([b[:2]], a)[0]
    return (float(x), float(y), wrap_angle(a[2] + b[2]))


def invert_pose(pose):
    """The pose that undoes `pose`: compose_poses(pose, invert) = identity."""
    x, y = global_to_local([[0.0, 0.0]], pose)[0]
    return (float(x), float(y), wrap_angle(-pose[2]))


def pose_error(estimated, true):
    """(position error in m, heading error in deg) between two poses.

    position error = sqrt((x_e - x_t)^2 + (y_e - y_t)^2)
    heading error  = |wrap(theta_e - theta_t)|, in degrees (0 to 180)
    """
    return (math.hypot(estimated[0] - true[0], estimated[1] - true[1]),
            angle_difference_deg(estimated[2], true[2]))


def is_success(position_error_m, angle_error_deg,
               max_position_m=SUCCESS_POSITION_M,
               max_angle_deg=SUCCESS_ANGLE_DEG):
    """True if both errors are within the (provisional) thresholds."""
    return position_error_m <= max_position_m and \
        angle_error_deg <= max_angle_deg


# ---------------------------------------------------------------------------
# 3. Scan preparation
# ---------------------------------------------------------------------------
def filter_ranges(points, min_range=MIN_RANGE_M, max_range=MAX_RANGE_M):
    """Keep finite points with min_range <= distance <= max_range."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    finite = np.all(np.isfinite(points), axis=1)
    ranges = np.full(len(points), np.nan)
    ranges[finite] = np.hypot(points[finite, 0], points[finite, 1])
    with np.errstate(invalid="ignore"):
        keep = finite & (ranges >= min_range) & (ranges <= max_range)
    return points[keep]


def voxel_downsample(points, cell_m):
    """Keep one point (the average) per square cell of size cell_m.

    The same method as Part 1's voxel_downsample() (checked by a test),
    rewritten here so this module does not import Part 1 (serial port,
    Matplotlib widgets). It evens out the point density: near walls give
    more points than far ones, and a scan that holds TWO revolutions of
    the LiDAR (scans 10, 13, 16, 22, 23 of the classroom) would otherwise
    count every wall twice.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) == 0:
        return points
    cells = np.floor(points / cell_m).astype(np.int64)
    _, cell_index, counts = np.unique(cells, axis=0, return_inverse=True,
                                      return_counts=True)
    cell_index = cell_index.ravel()
    return np.column_stack([
        np.bincount(cell_index, weights=points[:, 0]) / counts,
        np.bincount(cell_index, weights=points[:, 1]) / counts])


def angular_coverage_deg(points, bin_deg=5.0):
    """Total angle (deg) of the bins, seen from the sensor, holding a point.

    A full revolution gives about 360; a scan of a single wall far away
    gives a few tens of degrees, too little to fix a pose.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) == 0:
        return 0.0
    angles = np.degrees(np.arctan2(points[:, 1], points[:, 0])) + 180.0
    bins = np.unique(np.floor(angles / bin_deg).astype(int)
                     % int(round(360.0 / bin_deg)))
    return float(len(bins) * bin_deg)


# ---------------------------------------------------------------------------
# 4. Result of one localisation (shared by all Day 2 methods)
# ---------------------------------------------------------------------------
# status - the method's OWN verdict, computed without any ground truth:
#     "ok", "ambiguous", "at_search_limit", "low_score", "out_of_map",
#     "insufficient_points", "insufficient_coverage" (see each method)
# success - only set when a true or reference pose is supplied, from the
#     errors and the thresholds above. The two are kept apart on purpose:
#     a method can say "ok" and be wrong (symmetric room), or be right but
#     unsure.
@dataclass
class LocalisationResult:
    method: str
    status: str
    reason: str
    initial_pose: tuple
    pose: tuple = None                 # (x, y, theta_rad), None if failed
    score: float = float("nan")
    runtime_ms: float = float("nan")   # algorithm only: no loading, no GUI
    candidates_evaluated: int = 0
    points_used: int = 0
    out_of_map_fraction: float = float("nan")
    runner_up_score: float = None
    iterations: int = None             # for iterative methods (ICP)
    details: dict = field(default_factory=dict)
    # Filled by evaluate_against() when a true / reference pose is known.
    true_pose: tuple = None
    position_error_m: float = None
    angle_error_deg: float = None
    initial_position_error_m: float = None
    initial_angle_error_deg: float = None
    success: bool = None

    def evaluate_against(self, true_pose, max_position_m=SUCCESS_POSITION_M,
                         max_angle_deg=SUCCESS_ANGLE_DEG):
        """A copy with the errors and success measured against true_pose."""
        # A global search has no initial pose (None): no initial error.
        initial = (pose_error(self.initial_pose, true_pose)
                   if self.initial_pose is not None else (None, None))
        if self.pose is None:
            return replace(self, true_pose=tuple(true_pose),
                           initial_position_error_m=initial[0],
                           initial_angle_error_deg=initial[1], success=False)
        final = pose_error(self.pose, true_pose)
        return replace(self, true_pose=tuple(true_pose),
                       position_error_m=final[0], angle_error_deg=final[1],
                       initial_position_error_m=initial[0],
                       initial_angle_error_deg=initial[1],
                       success=is_success(*final, max_position_m,
                                          max_angle_deg))

    def to_dict(self):
        """Plain values for JSON/CSV; poses given in degrees for reading."""
        data = asdict(self)
        for key in ("initial_pose", "pose", "true_pose"):
            if data[key] is not None:
                data[key] = [round(value, 6) for value in
                             pose_to_day1(data[key])]
        return data


# ---------------------------------------------------------------------------
# 5. Reproducible test data
# ---------------------------------------------------------------------------
def simulate_scan(walls, pose, noise_mm=10.0, seed=0, angle_step_deg=0.8,
                  max_range_m=MAX_RANGE_M):
    """Simulated LiDAR revolution (Part 1's ray caster) at a Day 2 pose.

    Returns the (N, 2) sensor-frame points. 0.8 deg steps give about 450
    points per revolution, like the real LD06 scans (456 points).
    """
    x_local, y_local = day1("part1").simulate_scan_of_walls(
        walls, pose_to_day1(pose), angle_step_deg=angle_step_deg,
        noise_mm=noise_mm, max_range_m=max_range_m, seed=seed)
    return np.column_stack([x_local, y_local])


def add_clutter(points, fraction, seed=0, min_range=0.3, max_range=2.5):
    """Add `fraction` x N random points near the sensor (people, chairs).

    The same model as Part 1's make_alignment_case() outliers.
    """
    rng = np.random.default_rng(seed)
    count = int(fraction * len(points))
    radius = rng.uniform(min_range, max_range, count)
    angle = rng.uniform(-math.pi, math.pi, count)
    return np.vstack([points, np.column_stack([radius * np.cos(angle),
                                               radius * np.sin(angle)])])


def perturb_pose(pose, position_error_m, angle_error_deg, rng):
    """An initial guess EXACTLY position_error_m and angle_error_deg away.

    The direction of the position error and the sign of the heading error
    are drawn from `rng` (a seeded numpy Generator), so the experiment is
    repeatable and every guess has the stated initial error.
    """
    direction = rng.uniform(-math.pi, math.pi)
    sign = 1.0 if rng.random() < 0.5 else -1.0
    return (pose[0] + position_error_m * math.cos(direction),
            pose[1] + position_error_m * math.sin(direction),
            wrap_angle(pose[2] + sign * math.radians(angle_error_deg)))


def load_session_scans(path):
    """The visible scans of a Part 1 session, read with Part 2's loader.

    The file is only read. Each scan keeps Part 2's keys ("id", "name",
    "pose" in DEGREES, "points_local", ...) so it can be passed straight
    to Part 2's map builder; "pose_rad" is added for Day 2 code.
    """
    scans = day1("part2").load_session_scans(Path(path))
    for scan in scans:
        scan["pose_rad"] = pose_from_day1(scan["pose"])
    return scans


# ---------------------------------------------------------------------------
# 6. Independently MEASURED real poses (ground truth for real scans)
# ---------------------------------------------------------------------------
# No real scan has a measured pose yet: the Day 1 poses were ESTIMATED by
# Part 1. When the robot's positions are measured (tape, floor marks...),
# write them in a JSON file next to the session, e.g.
# scans/classroom_final_measured_poses.json:
#     {"format": "ld06_measured_poses", "version": 1,
#      "session": "classroom_final.json",
#      "method": "tape measure from floor marks; heading from wall edge",
#      "poses": [{"scan_id": 2, "x_m": 0.10, "y_m": 2.05, "theta_deg": 1.5,
#                 "sigma_xy_m": 0.02, "sigma_theta_deg": 1.0}]}
# The poses MUST be in the session's map frame (Scan 1 of the session at
# x = 0, y = 0, heading 0: measure each position relative to Scan 1's
# position and heading). sigma = the measurement's standard uncertainty;
# a localisation can only be judged to about that precision.
MEASURED_POSES_FORMAT = "ld06_measured_poses"


class MeasuredPoseError(ValueError):
    """The measured-poses file is missing pieces or has bad values."""


def load_measured_poses(path):
    """Read a measured-poses file; returns {scan_id: entry}, never writes.

    entry = {"pose": (x, y, theta_rad), "sigma_xy_m", "sigma_theta_deg",
             "method"}
    """
    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except OSError as error:
        raise MeasuredPoseError(f"cannot read {path}: {error}") from error
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        raise MeasuredPoseError(f"{path.name} is not valid JSON: {error}") \
            from error
    if not isinstance(data, dict) or \
            data.get("format") != MEASURED_POSES_FORMAT or \
            data.get("version") != 1 or \
            not isinstance(data.get("poses"), list):
        raise MeasuredPoseError(f"{path.name} is not a version 1 "
                                f"{MEASURED_POSES_FORMAT} file")
    method = str(data.get("method", "not stated"))
    poses = {}
    for index, entry in enumerate(data["poses"]):
        where = f"{path.name}, poses[{index}]"
        try:
            scan_id = entry["scan_id"]
            values = [float(entry[key]) for key in
                      ("x_m", "y_m", "theta_deg", "sigma_xy_m",
                       "sigma_theta_deg")]
        except (KeyError, TypeError, ValueError) as error:
            raise MeasuredPoseError(f"{where}: missing or invalid value "
                                    f"({error!r})") from error
        if isinstance(scan_id, bool) or not isinstance(scan_id, int):
            raise MeasuredPoseError(f"{where}: scan_id must be an integer")
        if not all(math.isfinite(value) for value in values) or \
                values[3] <= 0 or values[4] <= 0:
            raise MeasuredPoseError(f"{where}: values must be finite and "
                                    f"the uncertainties positive")
        if scan_id in poses:
            raise MeasuredPoseError(f"{where}: scan {scan_id} appears twice")
        poses[scan_id] = {"pose": pose_from_day1(values[:3]),
                          "sigma_xy_m": values[3],
                          "sigma_theta_deg": values[4], "method": method}
    return poses


def measured_poses_path(session_path):
    """Where the measured poses of a session are looked for by default."""
    session_path = Path(session_path)
    return session_path.with_name(f"{session_path.stem}_measured_poses.json")


def median_runtime(function, repeats):
    """Run function() `repeats` times; return (last result, median ms).

    Only the call is timed, so loading and drawing are never included.
    """
    times, result = [], None
    for _ in range(max(1, repeats)):
        start = time.perf_counter()
        result = function()
        times.append((time.perf_counter() - start) * 1000.0)
    return result, float(np.median(times))


# ---------------------------------------------------------------------------
# Automated tests: python -m unittest localisation_common -v
# ---------------------------------------------------------------------------
class PoseMathTests(unittest.TestCase):

    def test_degree_radian_conversion_round_trip(self):
        for pose in [(1.0, -2.0, 0.0), (0.5, 0.5, 90.0), (3.0, 1.0, -135.0),
                     (0.0, 0.0, 179.5)]:
            converted = pose_from_day1(pose)
            self.assertAlmostEqual(converted[2], math.radians(pose[2]))
            np.testing.assert_allclose(pose_to_day1(converted), pose,
                                       atol=1e-12)

    def test_conversion_wraps_heading(self):
        self.assertAlmostEqual(pose_from_day1((0, 0, 270.0))[2],
                               -math.pi / 2)
        self.assertAlmostEqual(pose_to_day1((0, 0, 3 * math.pi))[2], -180.0)

    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(math.pi + 0.1), -math.pi + 0.1)
        self.assertAlmostEqual(wrap_angle(-math.pi - 0.1), math.pi - 0.1)
        self.assertAlmostEqual(wrap_angle(4 * math.pi + 0.3), 0.3)
        np.testing.assert_allclose(
            wrap_angle(np.array([0.0, 2 * math.pi, -3 * math.pi])),
            [0.0, 0.0, -math.pi], atol=1e-12)

    def test_angle_difference_across_plus_minus_180(self):
        self.assertAlmostEqual(angle_difference_deg(math.radians(179.0),
                                                    math.radians(-179.0)),
                               2.0)
        self.assertAlmostEqual(angle_difference_deg(0.0, math.pi), 180.0)

    def test_transform_matches_part1_and_part2(self):
        rng = np.random.default_rng(3)
        points = rng.uniform(-5, 5, (50, 2))
        for pose_deg in [(0, 0, 0), (1.5, -2.0, 30.0), (-3.0, 4.0, -120.0),
                         (0.2, 0.1, 180.0)]:
            ours = transform_points(points, pose_from_day1(pose_deg))
            np.testing.assert_allclose(
                ours, day1("part2").local_to_global(points, pose_deg),
                atol=1e-12)
            np.testing.assert_allclose(
                ours, np.column_stack(day1("part1").local_to_global(
                    points[:, 0], points[:, 1], pose_deg)), atol=1e-12)

    def test_transform_rotation_direction(self):
        # A point 1 m in front of a sensor turned +90 deg (left) lies at +Y.
        np.testing.assert_allclose(
            transform_points([[1.0, 0.0]], (2.0, 3.0, math.pi / 2)),
            [[2.0, 4.0]], atol=1e-12)

    def test_global_to_local_inverts_and_matches_part1(self):
        rng = np.random.default_rng(4)
        points = rng.uniform(-5, 5, (40, 2))
        pose = (1.0, -0.5, math.radians(63.0))
        placed = transform_points(points, pose)
        np.testing.assert_allclose(global_to_local(placed, pose), points,
                                   atol=1e-12)
        np.testing.assert_allclose(
            global_to_local(placed, pose),
            day1("part1").global_to_local(placed, pose_to_day1(pose)),
            atol=1e-12)

    def test_compose_and_invert(self):
        rng = np.random.default_rng(5)
        points = rng.uniform(-2, 2, (10, 2))
        a = (1.0, 2.0, math.radians(40.0))
        b = (-0.5, 0.3, math.radians(-100.0))
        np.testing.assert_allclose(
            transform_points(points, compose_poses(a, b)),
            transform_points(transform_points(points, b), a), atol=1e-12)
        identity = compose_poses(a, invert_pose(a))
        np.testing.assert_allclose(identity, (0.0, 0.0, 0.0), atol=1e-12)

    def test_pose_error(self):
        position, angle = pose_error((1.0, 1.0, math.radians(178.0)),
                                     (1.3, 1.4, math.radians(-178.0)))
        self.assertAlmostEqual(position, 0.5)
        self.assertAlmostEqual(angle, 4.0)
        self.assertTrue(is_success(0.10, 5.0))
        self.assertFalse(is_success(0.11, 1.0))
        self.assertFalse(is_success(0.01, 5.1))


class ScanPreparationTests(unittest.TestCase):

    def test_voxel_downsample_matches_part1(self):
        rng = np.random.default_rng(6)
        points = rng.uniform(-4, 4, (500, 2))
        np.testing.assert_allclose(
            voxel_downsample(points, 0.1),
            day1("part1").voxel_downsample(points, 0.1), atol=1e-12)
        self.assertEqual(voxel_downsample(np.empty((0, 2)), 0.1).shape,
                         (0, 2))

    def test_doubled_revolution_is_not_counted_twice(self):
        ring = np.column_stack([np.cos(np.linspace(0, 6.2, 400)),
                                np.sin(np.linspace(0, 6.2, 400))]) * 3.0
        once = voxel_downsample(ring, 0.05)
        twice = voxel_downsample(np.vstack([ring, ring + 0.001]), 0.05)
        # The 1 mm shift moves a few points across cell borders, but the
        # second revolution must not double the number of kept points.
        self.assertLess(len(twice), 1.05 * len(once))

    def test_filter_ranges(self):
        points = np.array([[0.05, 0.0], [1.0, 0.0], [20.0, 0.0],
                           [np.nan, 1.0], [0.0, -3.0]])
        np.testing.assert_array_equal(filter_ranges(points),
                                      [[1.0, 0.0], [0.0, -3.0]])
        self.assertEqual(filter_ranges([]).shape, (0, 2))

    def test_angular_coverage(self):
        angles = np.radians(np.arange(0, 360, 1.0))
        full = np.column_stack([np.cos(angles), np.sin(angles)])
        self.assertAlmostEqual(angular_coverage_deg(full), 360.0)
        self.assertLessEqual(angular_coverage_deg(full[:30]), 35.0)
        self.assertEqual(angular_coverage_deg(np.empty((0, 2))), 0.0)


class TestDataTests(unittest.TestCase):

    def test_perturb_pose_has_exact_error_and_is_repeatable(self):
        pose = (2.0, 3.0, math.radians(170.0))
        first = perturb_pose(pose, 0.3, 10.0, np.random.default_rng(1))
        again = perturb_pose(pose, 0.3, 10.0, np.random.default_rng(1))
        self.assertEqual(first, again)
        position, angle = pose_error(first, pose)
        self.assertAlmostEqual(position, 0.3)
        self.assertAlmostEqual(angle, 10.0)

    def test_simulated_scan_uses_day2_pose(self):
        walls = day1("part1").CLASSROOM_WALLS
        pose = (2.3, 3.2, math.radians(30.0))
        ours = simulate_scan(walls, pose, noise_mm=0.0)
        x_l, y_l = day1("part1").simulate_scan_of_walls(
            walls, (2.3, 3.2, 30.0), angle_step_deg=0.8, noise_mm=0.0)
        np.testing.assert_allclose(ours, np.column_stack([x_l, y_l]))
        # Every simulated point lies on a wall once placed at the pose.
        self.assertGreater(len(ours), 300)

    def test_result_evaluation_and_dict(self):
        result = LocalisationResult(
            method="test", status="ok", reason="", pose=(1.0, 0.0, 0.0),
            initial_pose=(1.5, 0.0, math.radians(10.0)))
        evaluated = result.evaluate_against((1.05, 0.0, math.radians(2.0)))
        self.assertAlmostEqual(evaluated.position_error_m, 0.05)
        self.assertAlmostEqual(evaluated.angle_error_deg, 2.0)
        self.assertAlmostEqual(evaluated.initial_position_error_m, 0.45)
        self.assertTrue(evaluated.success)
        self.assertIsNone(result.success)  # the original is not changed
        self.assertAlmostEqual(evaluated.to_dict()["true_pose"][2], 2.0)
        failed = replace(result, pose=None).evaluate_against((0, 0, 0))
        self.assertFalse(failed.success)
        self.assertIsNone(failed.position_error_m)


class MeasuredPoseTests(unittest.TestCase):

    def write(self, folder, data, name="s_measured_poses.json"):
        path = Path(folder) / name
        path.write_text(json.dumps(data), encoding="utf-8")
        return path

    def valid(self, **changes):
        entry = {"scan_id": 2, "x_m": 1.0, "y_m": 2.0, "theta_deg": 90.0,
                 "sigma_xy_m": 0.02, "sigma_theta_deg": 1.0}
        entry.update(changes)
        return {"format": MEASURED_POSES_FORMAT, "version": 1,
                "method": "tape", "poses": [entry]}

    def test_valid_file(self):
        with tempfile.TemporaryDirectory() as folder:
            poses = load_measured_poses(self.write(folder, self.valid()))
        self.assertEqual(list(poses), [2])
        np.testing.assert_allclose(poses[2]["pose"],
                                   (1.0, 2.0, math.pi / 2))
        self.assertEqual(poses[2]["sigma_xy_m"], 0.02)
        self.assertEqual(poses[2]["method"], "tape")

    def test_invalid_files_are_refused(self):
        bad = [self.valid(x_m="far"), self.valid(sigma_xy_m=0.0),
               self.valid(theta_deg=float("nan")), self.valid(scan_id="2"),
               dict(self.valid(), format="other"),
               dict(self.valid(), poses=[self.valid()["poses"][0]] * 2)]
        with tempfile.TemporaryDirectory() as folder:
            for data in bad:
                with self.subTest(data=data), \
                        self.assertRaises(MeasuredPoseError):
                    load_measured_poses(self.write(folder, data))
            with self.assertRaises(MeasuredPoseError):
                load_measured_poses(Path(folder) / "missing.json")
            (Path(folder) / "broken.json").write_text("{", encoding="utf-8")
            with self.assertRaises(MeasuredPoseError):
                load_measured_poses(Path(folder) / "broken.json")

    def test_default_location(self):
        self.assertEqual(measured_poses_path(Path("scans") / "room.json"),
                         Path("scans") / "room_measured_poses.json")


if __name__ == "__main__":
    unittest.main()
