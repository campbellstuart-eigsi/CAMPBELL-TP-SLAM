"""Part 3 - 2D vector map from the aligned classroom LiDAR scans.

Follows the course document "Construction d'une carte vectorielle 2D a
partir d'un scan Lidar".

Milestone 1 - section 2 of the document:
    - load a Part 1 session (version 2 JSON) READ-ONLY, any number of scans,
    - transform every measurement into the global classroom frame with its
      scan's SAVED pose (poses are never changed or realigned here),
    - display the global point cloud and the LiDAR positions.
Milestone 2 - sections 2 (pre-processing) and 3.1 (segmentation):
    - pre-processing: reject invalid coordinates and measurements outside
      the LiDAR's range (and poor quality, if the data had quality values),
      keeping every measurement's original position in its scan,
    - distance-based segmentation of EACH scan, in measurement order, into
      clusters; small clusters are kept aside for the statistics,
    - two views: the point cloud, or the clusters in colour.
Milestone 3 - section 3.2 (geometric recognition, "regression lineaire"):
    - orthogonal least-squares (PCA) line fitted to every accepted cluster,
      giving segments (x1, y1, x2, y2) in global metres,
    - segments that are too short or not straight enough are rejected
      (corners and curves are not forced into one line),
    - a third view: the fitted segments.
Milestone 4 - section 3.2 ("simplification polygonale", Douglas-Peucker):
    - clusters rejected as NOT STRAIGHT (they bend around a corner) are cut
      at their corners; each piece is fitted with the same PCA and checked
      with the same thresholds; accepted segments are never split,
    - the segments view shows original and recovered segments.
Milestone 5 - section 4 ("fusion" of similar features):
    - segments of all scans that lie on the same surface (nearly parallel,
      close, overlapping or almost touching) are merged into one segment,
      checked against ALL their LiDAR points; their scans are recorded,
    - a fourth view: the merged vector map (colour = supporting scans).
Milestone 6 - inspection:
    - an interactive viewer (VIEW_MODE = "layers", the default): tick boxes
      show / hide the measured points, clusters, PCA lines, Douglas-Peucker
      lines, merged lines, LiDAR positions and endpoints over one map, to
      check the evidence behind every line. Nothing is recomputed on a
      click, and no geometry is added: every line comes from measurements.
Not yet: the vector map file (export).

The local -> global transformation and the session path rule are REUSED
from Part 2 (02_grid_mapping.py, unchanged).

Automated tests, from the project folder:
    python -m unittest 03_vector_mapping -v
"""
import argparse
import contextlib
import hashlib
import importlib
import io
import json
import math
import os
import random
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import matplotlib.colors
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.widgets import Button, CheckButtons
import matplotlib.pyplot as plt
import numpy as np


# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------
# The Part 1 session to use. A bare file name is looked for in the "scans"
# folder NEXT TO THIS PROGRAM (whatever folder PyCharm runs it from), e.g.
#     SESSION_FILE = "classroom_final.json"
#     SESSION_FILE = "classroom_session.json"
# An absolute path loads a file from anywhere else, e.g.
#     SESSION_FILE = r"C:\Users\me\Documents\other_room.json"
# The file is only ever READ.
SESSION_FILE = "classroom_final.json"
PROJECT_DIR = Path(__file__).resolve().parent
SCANS_DIR = PROJECT_DIR / "scans"

def choose_scan_file():
    from tkinter import Tk, filedialog
    root = Tk()
    root.withdraw()

    try:
        filename = filedialog.askopenfilename(
            title="Select LiDAR Scan File — Vector Mapping",
            initialdir=str(SCANS_DIR),
            filetypes=[("JSON Files", "*.json")]
        )
    finally:
        root.destroy()

    return Path(filename) if filename else None
# What to display:
#   "points"   - milestone 1: the global point cloud (colours: see below)
#   "clusters" - milestone 2: each accepted cluster in its own colour
#   "segments" - milestones 3-4: the fitted straight line segments
#   "merged"   - milestone 5: segments of several scans merged into walls
#   "layers"   - INTERACTIVE viewer: tick boxes show / hide each layer
#                (points, clusters, PCA lines, recovered lines, merged
#                lines, LiDAR positions, endpoints) over one map
VIEW_MODE = "layers"
VALID_VIEW_MODES = ("points", "clusters", "segments", "merged", "layers")
# "layers" viewer: which layers are ticked when the window opens.
LAYERS_VISIBLE_AT_START = ("points", "merged", "lidar")
INSPECTION_STYLE = True  # "layers" viewer: measured points as small light-
#                          grey dots so the lines stand out (False: points
#                          in their scan's colour)
SHOW_REJECTED_POINTS = True  # "clusters" view: draw in light grey what the
#                              filters removed (x) and small clusters (dots);
#                              "segments" view: draw faintly in red the
#                              clusters whose line fit was rejected
SHOW_POINTS_UNDER_SEGMENTS = True  # "segments" view: faint grey LiDAR points
#                                    of the fitted clusters under the lines
SHOW_UNMERGED_UNDER_MERGED = True  # "merged" view: the unmerged segments
#                                    drawn faintly underneath
MERGED_COLOR_BY_SUPPORT = True  # "merged" view: colour = number of scans
#                                 that saw the segment (False: one colour)
DISTINGUISH_RECOVERED_SEGMENTS = True  # "segments" view: Douglas-Peucker
#                                    segments drawn black and dashed (False:
#                                    in their scan's colour like the others)
COLOR_BY_SCAN = True        # "points" view: True = each scan in its Part 1
#                             colour, False = everything in SINGLE_COLOR
SINGLE_COLOR = "tab:blue"
SHOW_LIDAR_POSITIONS = True  # dot + heading arrow + number at each pose
POINT_SIZE = 2

# Pre-processing (course section 2: remove aberrant points).
MIN_RANGE_M = 0.10   # closer: inside the robot / LiDAR housing, not a wall
MAX_RANGE_M = 12.0   # LD06 maximum range (same limit as Part 2)
MIN_QUALITY = None   # quality filter threshold; None = off. The version 2
#                      files of Part 1 contain NO per-point quality (the
#                      Teleplot output has only x and y), so nothing is
#                      filtered on quality; the program says so.

# Segmentation (course section 3.1). Consecutive measurements of one scan
# farther apart than SEGMENT_DISTANCE_M start a new cluster. The LD06 gives
# one point every ~0.73 deg, i.e. points r * 0.0127 m apart on a wall at
# range r (6 cm at 5 m, 13 cm at 10 m), doubled when an echo is missing:
# 0.15 m keeps walls up to ~6 m whole even with one missing echo, while
# chair legs ~40 cm apart stay separate.
SEGMENT_DISTANCE_M = 0.15
MIN_CLUSTER_POINTS = 5   # fewer points cannot support a line fit later
# The first and last clusters of a scan are joined across the 0/360 deg
# boundary only for a genuine single revolution whose last and first
# measurements are at most this angle apart (one missing echo allowed).
MAX_WRAP_GAP_DEG = 2.5

# Straight-line fitting (course section 3.2). Chosen from the real classroom
# data: half the clusters fit a line within 1.1 cm RMS (the LD06's noise on
# a flat surface); curved chair legs, corners and mixed clusters give 2-25 cm.
LINE_MIN_POINTS = 5        # at least this many points per segment
LINE_MIN_LENGTH_M = 0.20   # shorter: chair-leg sections, desk feet, not walls
LINE_MAX_RMS_M = 0.02      # average distance of the points to the line
#                            (root mean square, ~2 x the LiDAR noise)
LINE_MAX_ERROR_M = 0.05    # worst single point: farther = a bend (corner)

# Douglas-Peucker corner splitting (course section 3.2: "simplification
# polygonale"). Only clusters whose whole-cluster line is NOT STRAIGHT
# ENOUGH (RMS or max error) are split; each piece is then fitted and checked
# with the SAME line thresholds as above. These settings are independent of
# the pre-processing and segmentation ones.
DP_ENABLED = True          # False = milestone 3 behaviour (no splitting)
DP_TOLERANCE_M = 0.05      # split where a point is farther than this from
#                            the chord joining a piece's two ends. Above the
#                            LiDAR noise of a straight wall (max error of
#                            straight classroom walls <= 5 cm), well below a
#                            corner's deviation (20-100 cm in the classroom).
DP_MAX_DEPTH = 6           # at most 6 nested splits (<= 64 pieces): a safety
#                            limit, far above the corners of one cluster
DP_MIN_POINTS = 8          # a RECOVERED segment needs at least this many
#                            points (stricter than LINE_MIN_POINTS = 5), so
#                            noisy furniture does not become many tiny lines

# Cross-scan segment merging (course section 4: "fusion" of similar
# features). Measured on the classroom: overlapping duplicates of one wall
# lie within 6 cm of each other (most) to ~15 cm (alignment error), while
# genuinely different parallel surfaces are >= 30 cm apart; duplicates
# differ by 2 deg (median), 6 deg for 95 %.
MERGE_MAX_ANGLE_DEG = 7.0  # lines at most this far from parallel
#                            (theta and theta + 180 deg are the same line)
MERGE_MAX_OFFSET_M = 0.10  # every endpoint within one 10 cm grid cell of
#                            the other line (and of the merged line)
MERGE_MAX_GAP_M = 0.15     # collinear pieces closer than this are joined
#                            (= SEGMENT_DISTANCE_M; a doorway is >= 0.7 m)
MERGE_MAX_RMS_M = 0.04     # the merged line must fit ALL contributing LiDAR
#                            points within this RMS (2 x LINE_MAX_RMS_M,
#                            for the scans' alignment error)

# Reasons why a fitted line is rejected ("" = accepted).
FEW_POINTS = "fewer than LINE_MIN_POINTS points"
DEGENERATE = "degenerate (all points at one place)"
TOO_SHORT = "shorter than LINE_MIN_LENGTH_M"
RMS_TOO_LARGE = "RMS error above LINE_MAX_RMS_M (not straight)"
ERROR_TOO_LARGE = "a point farther than LINE_MAX_ERROR_M (bend/corner)"
LINE_REJECTION_REASONS = (FEW_POINTS, DEGENERATE, TOO_SHORT, RMS_TOO_LARGE,
                          ERROR_TOO_LARGE)

# Reasons why pre-processing rejects a measurement ("" = kept).
KEPT = ""
INVALID = "invalid coordinates"
TOO_CLOSE = "closer than MIN_RANGE_M"
TOO_FAR = "farther than MAX_RANGE_M"
LOW_QUALITY = "quality below MIN_QUALITY"
REJECTION_REASONS = (INVALID, TOO_CLOSE, TOO_FAR, LOW_QUALITY)


# ---------------------------------------------------------------------------
# 1. Reused from Part 2 (02_grid_mapping.py is imported, never modified)
# ---------------------------------------------------------------------------
# A module whose name starts with a digit cannot be written in an "import"
# line, but importlib can load it. Importing it runs nothing: Part 2 only
# defines functions and settings until its main() is called. The project
# folder is put on Python's search path so this works from any folder.
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))
part2 = importlib.import_module("02_grid_mapping")

SessionError = part2.SessionError          # bad / missing session file
local_to_global = part2.local_to_global    # P_global = R(theta) P_local + T
file_sha256 = part2.file_sha256


def resolve_session_path(file_name):
    """SESSION_FILE setting -> full path: bare names inside scans/.

    Same rule as Part 2 (absolute paths unchanged, ".json" added if no
    extension), but relative to THIS program's scans/ folder.
    """
    return part2.resolve_session_path(file_name, scans_dir=SCANS_DIR)


# ---------------------------------------------------------------------------
# 2. Loading (tolerant to bad measurements) and the global point cloud
# ---------------------------------------------------------------------------
# Part 2's loader refuses a whole file if one point is NaN. Here a bad
# MEASUREMENT must only remove that measurement, so this loader keeps every
# measurement at its original position and turns unreadable coordinates
# into NaN; pre-processing then rejects them with a reason. A bad file
# structure or a bad POSE (needed to place the whole scan) is still refused.
def parse_coordinate(value):
    """A JSON number -> float; anything else (text, null, bool) -> NaN."""
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    return math.nan


def load_session_scans(path, visible_only=True):
    """Read a Part 1 version 2 session; never writes. Returns a list of:

        "id", "name", "pose" (x_m, y_m, theta_deg), "aligned", "color"
        "points_local" - (N, 2) float array in MEASUREMENT ORDER; a point
                         that is not two numbers becomes (NaN, NaN)
        "quality"      - None: Part 1 stores no per-point quality
    """
    path = Path(path)
    try:
        with open(path, "r", encoding="utf-8") as file:  # "r": read only
            data = json.load(file)
    except OSError as error:
        raise SessionError(f"cannot read {path}: {error.strerror}") from error
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        raise SessionError(f"{path.name} is not valid JSON: {error}") from error
    if not isinstance(data, dict) or data.get("format") != \
            part2.SESSION_FORMAT_NAME:
        raise SessionError(f"{path.name} is not a Part 1 session file")
    if data.get("version") != part2.SESSION_FORMAT_VERSION:
        raise SessionError(f"{path.name} has version {data.get('version')!r}; "
                           f"open it in Part 1 and save it again "
                           f"(version {part2.SESSION_FORMAT_VERSION})")
    if not isinstance(data.get("scans"), list):
        raise SessionError(f"{path.name}: 'scans' must be a list")

    scans = []
    for index, entry in enumerate(data["scans"]):
        where = f"{path.name}, scans[{index}]"
        try:
            pose = tuple(float(entry["pose"][key])
                         for key in ("x_m", "y_m", "theta_deg"))
            raw_points = entry["points_local_m"]
        except (KeyError, TypeError, ValueError) as error:
            raise SessionError(f"{where}: missing or invalid pose/points "
                               f"({error!r})") from error
        if not all(math.isfinite(value) for value in pose):
            raise SessionError(f"{where}: the pose is not finite")
        if not isinstance(raw_points, list) or not raw_points:
            raise SessionError(f"{where}: points_local_m must be a non-empty "
                               f"list")
        points = np.array([
            [parse_coordinate(point[0]), parse_coordinate(point[1])]
            if isinstance(point, list) and len(point) == 2
            else [math.nan, math.nan] for point in raw_points])
        if visible_only and not entry.get("visible", True):
            continue
        scans.append({"id": entry.get("id", index + 1),
                      "name": entry.get("name", f"Scan {index + 1}"),
                      "pose": pose, "points_local": points,
                      "aligned": bool(entry.get("aligned", False)),
                      "color": entry.get("color"), "quality": None})
    return scans


def load_point_cloud(path):
    """Load the visible scans and place their points in the classroom.

    Returns (scans, hidden_count). Each scan also gets
        "points_global" - (N, 2) array, same order as "points_local"
    The ORDER of the points is the LiDAR's measurement order (by angle);
    it is kept, because the segmentation splits a scan where two SUCCESSIVE
    measurements are far apart. The saved poses are used as they are.
    """
    all_scans = load_session_scans(path, visible_only=False)
    scans = load_session_scans(path, visible_only=True)
    for scan in scans:
        # Invalid (NaN) measurements stay NaN at their place, untransformed.
        local = scan["points_local"]
        finite = np.all(np.isfinite(local), axis=1)
        scan["points_global"] = np.full(local.shape, np.nan)
        scan["points_global"][finite] = local_to_global(local[finite],
                                                        scan["pose"])
    return scans, len(all_scans) - len(scans)


def stack_point_cloud(scans):
    """All global points in one (N, 2) array, and the scan index of each."""
    if not scans:
        return np.empty((0, 2)), np.empty(0, dtype=int)
    points = np.vstack([scan["points_global"] for scan in scans])
    owner = np.concatenate([np.full(len(scan["points_global"]), index)
                            for index, scan in enumerate(scans)])
    return points, owner


# ---------------------------------------------------------------------------
# 3. Pre-processing (course section 2)
# ---------------------------------------------------------------------------
def preprocess_measurements(points_local, min_range=MIN_RANGE_M,
                            max_range=MAX_RANGE_M, quality=None,
                            min_quality=MIN_QUALITY):
    """Return one reason per measurement: KEPT ("") or why it is rejected.

    Nothing is removed or reordered: measurement i keeps index i, so the
    segmentation can see WHERE filtered measurements were. Checks, in order:
      INVALID     - x or y is NaN or infinite (cannot be placed anywhere)
      TOO_CLOSE   - range sqrt(x^2 + y^2) < min_range (robot's own body)
      TOO_FAR     - range > max_range (beyond the LiDAR's specification)
      LOW_QUALITY - quality < min_quality, only if quality values exist
    The range is measured in the LiDAR's own frame, from the sensor.
    """
    points_local = np.asarray(points_local, dtype=float)
    reasons = np.full(len(points_local), KEPT, dtype=object)
    finite = np.all(np.isfinite(points_local), axis=1)
    reasons[~finite] = INVALID
    ranges = np.full(len(points_local), np.nan)
    ranges[finite] = np.hypot(points_local[finite, 0],
                              points_local[finite, 1])
    reasons[finite & (ranges < min_range)] = TOO_CLOSE
    reasons[finite & (ranges > max_range)] = TOO_FAR
    if quality is not None and min_quality is not None:
        low = (reasons == KEPT) & (np.asarray(quality) < min_quality)
        reasons[low] = LOW_QUALITY
    return reasons


def preprocess_scans(scans, min_range=MIN_RANGE_M, max_range=MAX_RANGE_M,
                     min_quality=MIN_QUALITY):
    """Add "reject_reason" and "valid" to every scan; return reason counts.

    The scan's points, pose and the JSON file are not changed.
    """
    counts = {reason: 0 for reason in REJECTION_REASONS}
    for scan in scans:
        reasons = preprocess_measurements(scan["points_local"], min_range,
                                          max_range, scan.get("quality"),
                                          min_quality)
        scan["reject_reason"] = reasons
        scan["valid"] = reasons == KEPT
        for reason in REJECTION_REASONS:
            counts[reason] += int(np.sum(reasons == reason))
    return counts


# ---------------------------------------------------------------------------
# 4. Distance-based segmentation (course section 3.1)
# ---------------------------------------------------------------------------
# Each scan is cut separately, walking through its measurements in the order
# the LiDAR took them. Between two consecutive KEPT measurements p1, p2:
#     d = sqrt((x2 - x1)^2 + (y2 - y1)^2)
# A new cluster starts when d > SEGMENT_DISTANCE_M (a jump in distance =
# another object), or when a FILTERED measurement lies between them (we do
# not know what it saw, so we do not bridge it). d is computed on the LOCAL
# points: a rigid transformation keeps distances, so the result does not
# depend on the scan's pose. Nothing is sorted by X or Y.
#
# The 0/360 deg boundary: the LiDAR turns in a circle, so the last
# measurement of a revolution is next to the first one, and an object
# straddling the start direction is cut in two. The last and first clusters
# are joined only if ALL of these hold:
#   - the scan is a single revolution: the measurement angles always turn
#     the same way and in total less than 360 deg (some Part 1 scans hold
#     2 revolutions or 1.5: their first and last points are not neighbours
#     in the measurement sequence),
#   - the first and the last measurements were both kept (not filtered),
#   - they are at most MAX_WRAP_GAP_DEG apart in angle,
#   - they satisfy the distance rule.
def measurement_angles_deg(points_local):
    """Direction of each measurement seen from the LiDAR, in degrees."""
    return np.degrees(np.arctan2(points_local[:, 1], points_local[:, 0]))


def revolution_check(points_local):
    """Return (is_single_revolution, wrap_gap_deg) for kept measurements.

    The angle step between consecutive measurements is wrapped into
    [-180, 180). The LiDAR's turning direction is the sign of the median
    step. A single revolution turns that way at every step and less than
    360 deg in total; wrap_gap_deg is then the angle left between the last
    and the first measurement (None otherwise).
    """
    if len(points_local) < 3:
        return False, None
    steps = (np.diff(measurement_angles_deg(points_local)) + 180.0) \
        % 360.0 - 180.0
    direction = np.sign(np.median(steps))
    forward = steps * direction
    total = float(np.sum(forward))
    if direction == 0 or np.any(forward <= 0) or total >= 360.0:
        return False, None
    return True, 360.0 - total


def segment_scan(points_local, valid, distance_threshold=SEGMENT_DISTANCE_M,
                 min_points=MIN_CLUSTER_POINTS,
                 max_wrap_gap_deg=MAX_WRAP_GAP_DEG):
    """Split ONE scan into clusters; return a list of dictionaries:

        "indices"  - original measurement indices, in measurement order
                     (for a cluster joined across 0/360: the end of the
                     scan, then its start)
        "accepted" - True if it has at least min_points measurements
        "wraps"    - True if it was joined across the 0/360 boundary
    Every kept measurement belongs to exactly one cluster.
    """
    kept_indices = np.flatnonzero(valid)
    if len(kept_indices) == 0:
        return []
    kept_points = points_local[kept_indices]
    distances = np.hypot(*np.diff(kept_points, axis=0).T)
    filtered_between = np.diff(kept_indices) > 1
    breaks = np.flatnonzero((distances > distance_threshold)
                            | filtered_between) + 1
    runs = np.split(kept_indices, breaks)
    wraps = [False] * len(runs)

    if len(runs) > 1 and valid[0] and valid[-1]:
        single_revolution, wrap_gap = revolution_check(kept_points)
        boundary_distance = float(np.hypot(*(points_local[-1]
                                             - points_local[0])))
        if (single_revolution and wrap_gap <= max_wrap_gap_deg
                and boundary_distance <= distance_threshold):
            runs[0] = np.concatenate([runs[-1], runs[0]])
            wraps[0] = True
            runs, wraps = runs[:-1], wraps[:-1]

    return [{"indices": run, "accepted": len(run) >= min_points,
             "wraps": wrap} for run, wrap in zip(runs, wraps)]


def segment_scans(scans, distance_threshold=SEGMENT_DISTANCE_M,
                  min_points=MIN_CLUSTER_POINTS,
                  max_wrap_gap_deg=MAX_WRAP_GAP_DEG):
    """Segment every scan independently; return all clusters, in order.

    Each cluster also gets "scan_index" (position in `scans`), "scan_id"
    and "points_global" (its points in the classroom, same order). Points
    of different scans are never in the same cluster.
    Needs preprocess_scans() first.
    """
    clusters = []
    for scan_index, scan in enumerate(scans):
        for cluster in segment_scan(scan["points_local"], scan["valid"],
                                    distance_threshold, min_points,
                                    max_wrap_gap_deg):
            cluster["scan_index"] = scan_index
            cluster["scan_id"] = scan["id"]
            cluster["points_global"] = scan["points_global"][
                cluster["indices"]]
            clusters.append(cluster)
    return clusters


def segmentation_statistics(scans, clusters, rejection_counts):
    """Numbers for the summary (all derived, nothing changed)."""
    accepted = [cluster for cluster in clusters if cluster["accepted"]]
    small = [cluster for cluster in clusters if not cluster["accepted"]]
    return {
        "scans": len(scans),
        "points": sum(len(scan["points_local"]) for scan in scans),
        "rejected_by_reason": dict(rejection_counts),
        "rejected_total": sum(rejection_counts.values()),
        "kept_after_filters": sum(int(np.sum(scan["valid"]))
                                  for scan in scans),
        "clusters": len(clusters),
        "accepted_clusters": len(accepted),
        "small_clusters": len(small),
        "points_in_accepted": sum(len(c["indices"]) for c in accepted),
        "points_in_small": sum(len(c["indices"]) for c in small),
        "wrapped_clusters": sum(cluster["wraps"] for cluster in clusters),
        "single_revolution_scans": sum(
            revolution_check(scan["points_local"][scan["valid"]])[0]
            for scan in scans),
    }


def print_statistics(stats, quality_available):
    """Print the pre-processing and segmentation summary."""
    print(f"Input: {stats['scans']} scans, {stats['points']} measurements")
    print(f"Pre-processing (range {MIN_RANGE_M}-{MAX_RANGE_M} m): kept "
          f"{stats['kept_after_filters']}, rejected "
          f"{stats['rejected_total']}:")
    for reason in REJECTION_REASONS:
        if reason == LOW_QUALITY and not quality_available:
            print(f"  {reason:28s}     - (no quality values in this "
                  f"session: filter not applied)")
        else:
            print(f"  {reason:28s} {stats['rejected_by_reason'][reason]:5d}")
    print(f"Segmentation (new cluster when consecutive measurements are > "
          f"{SEGMENT_DISTANCE_M} m apart or a filtered one lies between; "
          f"accepted if >= {MIN_CLUSTER_POINTS} points):")
    print(f"  clusters detected      {stats['clusters']:5d} "
          f"({stats['wrapped_clusters']} joined across 0/360 deg; "
          f"{stats['single_revolution_scans']} of {stats['scans']} scans "
          f"are single revolutions)")
    print(f"  accepted clusters      {stats['accepted_clusters']:5d} "
          f"({stats['points_in_accepted']} points)")
    print(f"  rejected small         {stats['small_clusters']:5d} "
          f"({stats['points_in_small']} points, kept for diagnostics)")
    print(f"  points retained for line fitting: "
          f"{stats['points_in_accepted']} of {stats['points']} "
          f"({stats['points_in_accepted'] / max(stats['points'], 1):.1%})")


# ---------------------------------------------------------------------------
# 5. Straight-line fitting (course section 3.2: "regression lineaire")
# ---------------------------------------------------------------------------
# Ordinary regression y = m x + b measures the errors VERTICALLY (along y)
# and needs a finite slope m: for a vertical wall (x = constant) m is
# infinite and the fit fails or becomes unstable, and the result changes if
# the same scan is turned. Walls of a room point in every direction, so we
# minimise the PERPENDICULAR distances of the points to the line instead:
# "total least squares", solved exactly by PCA (principal component
# analysis), the same way for any direction:
#   1. centroid      c = (1/n) sum p_i                  (the line passes here)
#   2. scatter       S = (1/n) sum (p_i - c)(p_i - c)^T  (2 x 2 matrix)
#   3. its eigenvectors: d (largest eigenvalue) = direction in which the
#      points spread the most = the line's direction; n (smallest) = the
#      normal. The smallest eigenvalue is the mean squared perpendicular
#      distance, so the line through c along d is the best possible one.
#   4. along_i  = (p_i - c) . d  position of each point along the line,
#      across_i = (p_i - c) . n  its signed perpendicular error.
#   5. endpoints = c + min(along) d  and  c + max(along) d: the extreme
#      points PROJECTED onto the line, so the segment lies exactly on it.
# The result does not depend on the axes, the robot's pose or rotation.
def fit_line(points):
    """Fit a straight line to (N, 2) points by orthogonal least squares.

    Returns a dictionary (always, even when it cannot be used):
        "points"    - N
        "centroid"  - c;  "direction" - d (unit);  "normal" - n (unit)
        "start", "end" - segment endpoints (x, y), in the order of the
                         points (the LiDAR's measurement order)
        "length"    - distance between the endpoints (m)
        "rms_error", "max_error" - perpendicular errors (m)
        "degenerate" - True if fewer than 2 points or all at one place
    """
    points = np.asarray(points, dtype=float)
    result = {"points": len(points), "degenerate": True, "length": 0.0,
              "rms_error": 0.0, "max_error": 0.0}
    if len(points) < 2:
        return result
    centroid = points.mean(axis=0)
    centred = points - centroid
    scatter = centred.T @ centred / len(points)
    eigenvalues, eigenvectors = np.linalg.eigh(scatter)  # ascending order
    if eigenvalues[1] <= 1e-12:  # no spread at all: no direction
        return result
    direction = eigenvectors[:, 1]
    normal = eigenvectors[:, 0]
    along = centred @ direction
    across = centred @ normal
    if along[-1] < along[0]:  # orient the segment like the measurements
        direction, along = -direction, -along
    result.update({
        "degenerate": False, "centroid": centroid, "direction": direction,
        "normal": normal,
        "start": centroid + along.min() * direction,
        "end": centroid + along.max() * direction,
        "length": float(along.max() - along.min()),
        "rms_error": float(np.sqrt(np.mean(across ** 2))),
        "max_error": float(np.max(np.abs(across)))})
    return result


def validate_line(fit, min_points=LINE_MIN_POINTS,
                  min_length=LINE_MIN_LENGTH_M, max_rms=LINE_MAX_RMS_M,
                  max_error=LINE_MAX_ERROR_M):
    """Return "" if the fitted line is a good segment, else the reason.

    The checks: enough points, a real direction, long enough, straight on
    average (RMS), and no point too far (a cluster that bends - a corner,
    a curved chair leg - is NOT forced into one line; it is rejected).
    """
    if fit["points"] < min_points:
        return FEW_POINTS
    if fit["degenerate"]:
        return DEGENERATE
    if fit["length"] < min_length:
        return TOO_SHORT
    if fit["rms_error"] > max_rms:
        return RMS_TOO_LARGE
    if fit["max_error"] > max_error:
        return ERROR_TOO_LARGE
    return ""


# ---------------------------------------------------------------------------
# 5b. Douglas-Peucker corner splitting (course section 3.2)
# ---------------------------------------------------------------------------
# A cluster that bends around a corner is not ONE straight line, but its
# parts are. Douglas-Peucker (1973) finds where to cut, on the points IN
# MEASUREMENT ORDER:
#   1. draw the chord from the piece's first point A to its last point B;
#   2. compute every point's perpendicular distance to that line;
#   3. take the point P with the LARGEST distance;
#   4. if it is farther than DP_TOLERANCE_M, cut the piece at P into
#      A..P and P..B (P belongs to both: it is the corner shared by the two
#      walls) and repeat on each part; otherwise the piece is straight.
# Why the farthest point is the corner: along an L-shaped wall the chord
# A-B cuts across the inside of the L; points on the walls get farther from
# it as they approach the corner, and the corner itself is the farthest.
# Perpendicular distance of P to the line through A and B (2D cross
# product divided by the chord length):
#     d = |(Bx - Ax)(Py - Ay) - (By - Ay)(Px - Ax)| / |AB|
# If A and B are (almost) the same point - a closed loop, repeated points -
# the line has no direction, so the distance to A itself, |AP|, is used.
# Safety: pieces of fewer than 3 points cannot be cut, and the splits stop
# after DP_MAX_DEPTH levels. Pieces never leave their cluster, so nothing
# is joined across clusters or scans.
def perpendicular_distances(points, start, end):
    """Distance (m) of each point to the line through start and end."""
    points = np.asarray(points, dtype=float)
    chord = np.asarray(end, dtype=float) - np.asarray(start, dtype=float)
    offsets = points - np.asarray(start, dtype=float)
    chord_length = float(np.hypot(chord[0], chord[1]))
    if chord_length < 1e-9:  # zero-length baseline
        return np.hypot(offsets[:, 0], offsets[:, 1])
    return np.abs(chord[0] * offsets[:, 1]
                  - chord[1] * offsets[:, 0]) / chord_length


def douglas_peucker_pieces(points, tolerance=DP_TOLERANCE_M,
                           max_depth=DP_MAX_DEPTH):
    """Cut ordered points into straight pieces; return [(first, last), ...].

    first / last are POSITIONS in `points` (inclusive), in measurement
    order; consecutive pieces share their cutting point. A straight input
    gives a single piece [(0, n - 1)]. An explicit list of pending pieces
    is used instead of recursion, so no Python recursion limit can be hit.
    """
    points = np.asarray(points, dtype=float)
    if len(points) == 0:
        return []
    finished = []
    pending = [(0, len(points) - 1, 0)]  # (first, last, depth)
    while pending:
        first, last, depth = pending.pop()
        if last - first < 2 or depth >= max_depth:  # no inner point / limit
            finished.append((first, last))
            continue
        distances = perpendicular_distances(points[first + 1:last],
                                            points[first], points[last])
        farthest = first + 1 + int(np.argmax(distances))
        if distances.max() <= tolerance:
            finished.append((first, last))  # straight enough
        else:
            pending.append((first, farthest, depth + 1))
            pending.append((farthest, last, depth + 1))
    return sorted(finished)


def make_record(cluster, thresholds, origin):
    """Fit and check one cluster (or piece); return its segment record."""
    fit = fit_line(cluster["points_global"])
    reason = validate_line(fit, **thresholds)
    return {"cluster": cluster, "fit": fit, "reason": reason,
            "accepted": reason == "", "origin": origin, "pieces": [],
            "endpoints": (tuple(float(value) for value in
                                (*fit["start"], *fit["end"]))
                          if reason == "" else None)}


def split_cluster(cluster, tolerance=DP_TOLERANCE_M, max_depth=DP_MAX_DEPTH):
    """Douglas-Peucker pieces of a cluster, as smaller clusters.

    Each piece keeps the cluster's scan and its ORIGINAL measurement
    indices (a contiguous part of the cluster's "indices", same order).
    """
    pieces = []
    for first, last in douglas_peucker_pieces(cluster["points_global"],
                                              tolerance, max_depth):
        piece = dict(cluster)
        piece["indices"] = cluster["indices"][first:last + 1]
        piece["points_global"] = cluster["points_global"][first:last + 1]
        piece["accepted"] = True
        piece["wraps"] = False
        pieces.append(piece)
    return pieces


def fit_segments(clusters, split=DP_ENABLED, dp_tolerance=DP_TOLERANCE_M,
                 dp_max_depth=DP_MAX_DEPTH, dp_min_points=DP_MIN_POINTS,
                 **thresholds):
    """Fit a line to every ACCEPTED cluster; return one record per cluster:

        "cluster"   - the cluster (scan_index, scan_id, indices, points)
        "fit"       - fit_line()'s result, in GLOBAL metres
        "reason"    - "" if accepted, else why not (validate_line)
        "accepted"  - True / False
        "endpoints" - (x1, y1, x2, y2) in global metres (accepted only,
                      else None): the course's segment model
        "origin"    - "original" (whole cluster)
        "pieces"    - Douglas-Peucker piece records ("origin" "recovered"),
                      only for a cluster rejected as NOT STRAIGHT
    A cluster whose whole line is accepted is never split (its record is
    exactly milestone 3's). Pieces must pass the same thresholds, with at
    least max(LINE_MIN_POINTS, dp_min_points) points.
    """
    piece_thresholds = dict(thresholds)
    piece_thresholds["min_points"] = max(thresholds.get("min_points",
                                                        LINE_MIN_POINTS),
                                         dp_min_points)
    records = []
    for cluster in clusters:
        if not cluster["accepted"]:
            continue
        record = make_record(cluster, thresholds, "original")
        if split and record["reason"] in (RMS_TOO_LARGE, ERROR_TOO_LARGE):
            pieces = split_cluster(cluster, dp_tolerance, dp_max_depth)
            if len(pieces) > 1:
                record["pieces"] = [make_record(piece, piece_thresholds,
                                                "recovered")
                                    for piece in pieces]
        records.append(record)
    return records


def accepted_segments(records):
    """All accepted segments, original then recovered, cluster by cluster."""
    segments = []
    for record in records:
        if record["accepted"]:
            segments.append(record)
        segments.extend(piece for piece in record["pieces"]
                        if piece["accepted"])
    return segments


def line_fitting_statistics(records):
    """Counts and lengths of accepted / rejected segments.

    The first keys describe the WHOLE-cluster fits (as in milestone 3);
    the "dp_" keys describe the Douglas-Peucker recovery.
    """
    accepted = [record for record in records if record["accepted"]]
    examined = [record for record in records
                if record["reason"] in (RMS_TOO_LARGE, ERROR_TOO_LARGE)]
    split = [record for record in examined if record["pieces"]]
    pieces = [piece for record in split for piece in record["pieces"]]
    recovered = [piece for piece in pieces if piece["accepted"]]
    rescued = [record for record in split
               if any(piece["accepted"] for piece in record["pieces"])]
    rescued_ids = {id(record) for record in rescued}  # records hold arrays:
    still_rejected = [record for record in records    # compare identities
                      if not record["accepted"]
                      and id(record) not in rescued_ids]
    return {
        "fitted_clusters": len(records),
        "accepted_segments": len(accepted),
        "rejected_segments": len(records) - len(accepted),
        "rejected_by_reason": {reason: sum(record["reason"] == reason
                                           for record in records)
                               for reason in LINE_REJECTION_REASONS},
        "total_length_m": sum(record["fit"]["length"] for record in accepted),
        "longest_m": max((record["fit"]["length"] for record in accepted),
                         default=0.0),
        "median_rms_m": (float(np.median([record["fit"]["rms_error"]
                                          for record in accepted]))
                         if accepted else 0.0),
        "dp_examined": len(examined),
        "dp_split": len(split),
        "dp_pieces": len(pieces),
        "dp_recovered_segments": len(recovered),
        "dp_recovered_length_m": sum(piece["fit"]["length"]
                                     for piece in recovered),
        "dp_rescued_clusters": len(rescued),
        "dp_pieces_rejected_by_reason": {
            reason: sum(piece["reason"] == reason for piece in pieces)
            for reason in LINE_REJECTION_REASONS},
        "still_rejected_by_reason": {
            reason: sum(record["reason"] == reason
                        for record in still_rejected)
            for reason in LINE_REJECTION_REASONS},
        "segments_after_splitting": len(accepted) + len(recovered),
        "length_after_splitting_m": (
            sum(record["fit"]["length"] for record in accepted)
            + sum(piece["fit"]["length"] for piece in recovered)),
    }


def print_line_statistics(stats):
    """Print the line-fitting summary."""
    print(f"Line fitting (orthogonal least squares / PCA) on "
          f"{stats['fitted_clusters']} accepted clusters; a segment needs "
          f">= {LINE_MIN_POINTS} points, length >= {LINE_MIN_LENGTH_M} m, "
          f"RMS <= {LINE_MAX_RMS_M} m, every point within "
          f"{LINE_MAX_ERROR_M} m:")
    print(f"  accepted segments      {stats['accepted_segments']:5d} "
          f"(total {stats['total_length_m']:.1f} m, longest "
          f"{stats['longest_m']:.2f} m, median RMS "
          f"{stats['median_rms_m'] * 100:.1f} cm)")
    print(f"  rejected clusters      {stats['rejected_segments']:5d}:")
    for reason, count in stats["rejected_by_reason"].items():
        if count:
            print(f"    {reason:52s} {count:5d}")
    if not DP_ENABLED:
        print("Douglas-Peucker splitting: off (DP_ENABLED = False)")
        return
    print(f"Douglas-Peucker splitting (tolerance {DP_TOLERANCE_M} m, depth "
          f"<= {DP_MAX_DEPTH}, recovered segments need >= "
          f"{max(LINE_MIN_POINTS, DP_MIN_POINTS)} points):")
    print(f"  clusters examined (not straight)  {stats['dp_examined']:4d}")
    print(f"  clusters actually split           {stats['dp_split']:4d} "
          f"(into {stats['dp_pieces']} pieces)")
    print(f"  recovered segments                "
          f"{stats['dp_recovered_segments']:4d} "
          f"(total {stats['dp_recovered_length_m']:.1f} m, from "
          f"{stats['dp_rescued_clusters']} clusters)")
    rejected_pieces = {reason: count for reason, count
                       in stats["dp_pieces_rejected_by_reason"].items()
                       if count}
    if rejected_pieces:
        print("  pieces still rejected:")
        for reason, count in rejected_pieces.items():
            print(f"    {reason:52s} {count:5d}")
    print(f"Segments after splitting: {stats['segments_after_splitting']} "
          f"(total {stats['length_after_splitting_m']:.1f} m)")
    print("Clusters still without any segment:")
    for reason, count in stats["still_rejected_by_reason"].items():
        if count:
            print(f"    {reason:52s} {count:5d}")


# ---------------------------------------------------------------------------
# 5c. Cross-scan segment merging (course section 4: "fusion")
# ---------------------------------------------------------------------------
# Several scans see the same wall, so it appears as several overlapping
# segments. Two segments are COMPATIBLE (good evidence of one surface) if:
#   - angle: their lines are nearly parallel. A line has no direction, so
#     theta and theta + 180 deg are the same: angle = acos(|d1 . d2|);
#   - offset: every endpoint of each is close to the OTHER's line
#     (perpendicular distance |(p - c) x d|), so they are not two nearby
#     parallel surfaces;
#   - gap: projected on one line, their intervals overlap or leave at most a
#     small gap (they are not two sides of a doorway).
# Compatible pairs are tried from the best (smallest offset) to the worst.
# A pair is merged only if the WHOLE resulting group still forms one
# surface: a PCA line fitted to ALL the group's LiDAR points (each
# observation counts) must fit them within MERGE_MAX_RMS_M, every member
# must still pass the angle and offset tests against that line, and the
# members must cover it without a gap. This stops "chains" A~B, B~C
# from joining two different walls A and C. The merged segment's ends are
# the extreme member endpoints projected onto the merged line.
# Segments get an ID in a CANONICAL order (scan, then endpoints) and ties
# are broken by IDs, so the result does not depend on the input order.
def line_angle_deg(direction_a, direction_b):
    """Angle between two LINES (0-90 deg): theta and theta+180 are equal."""
    cosine = abs(float(np.dot(direction_a, direction_b)))
    return math.degrees(math.acos(min(1.0, cosine)))


def distance_to_line(points, origin, direction):
    """Perpendicular distance of points to the line origin + t*direction."""
    offsets = np.atleast_2d(points) - origin
    return np.abs(offsets[:, 0] * direction[1] - offsets[:, 1] * direction[0])


def projected_interval(start, end, origin, direction):
    """(t_min, t_max) of a segment projected on the line origin + t*d."""
    values = sorted([float((start - origin) @ direction),
                     float((end - origin) @ direction)])
    return values[0], values[1]


def describe_segments(segments):
    """Give each segment a canonical ID; return items sorted by that ID.

    An item: "id" (S001...), "segment" (the record), "scan_id", "origin",
    "start", "end", "direction" (unit), "length", "points" (its LiDAR
    points, global). The order depends only on the geometry, never on the
    input order.
    """
    items = []
    for segment in segments:
        start = np.asarray(segment["fit"]["start"], dtype=float)
        end = np.asarray(segment["fit"]["end"], dtype=float)
        ends = sorted([tuple(np.round(start, 9)), tuple(np.round(end, 9))])
        items.append({"key": (segment["cluster"]["scan_id"], *ends),
                      "segment": segment,
                      "scan_id": segment["cluster"]["scan_id"],
                      "origin": segment.get("origin", "original"),
                      "start": start, "end": end,
                      "direction": (end - start) / np.linalg.norm(end - start),
                      "length": float(np.linalg.norm(end - start)),
                      "points": segment["cluster"]["points_global"]})
    items.sort(key=lambda item: item["key"])
    for number, item in enumerate(items, start=1):
        item["id"] = f"S{number:03d}"
    return items


def pair_compatibility(a, b, max_angle_deg=MERGE_MAX_ANGLE_DEG,
                       max_offset=MERGE_MAX_OFFSET_M, max_gap=MERGE_MAX_GAP_M):
    """Return the worst endpoint offset (m) if a and b are compatible, else
    None. The gap is measured along the longer segment's line."""
    if line_angle_deg(a["direction"], b["direction"]) > max_angle_deg:
        return None
    offset = max(distance_to_line(np.array([a["start"], a["end"]]),
                                  b["start"], b["direction"]).max(),
                 distance_to_line(np.array([b["start"], b["end"]]),
                                  a["start"], a["direction"]).max())
    if offset > max_offset:
        return None
    base = a if a["length"] >= b["length"] else b
    first = projected_interval(a["start"], a["end"], base["start"],
                               base["direction"])
    second = projected_interval(b["start"], b["end"], base["start"],
                                base["direction"])
    if max(second[0] - first[1], first[0] - second[1]) > max_gap:
        return None
    return float(offset)


def merge_group(items, max_angle_deg=MERGE_MAX_ANGLE_DEG,
                max_offset=MERGE_MAX_OFFSET_M, max_gap=MERGE_MAX_GAP_M,
                max_rms=MERGE_MAX_RMS_M):
    """Merge segments into one if they form ONE surface; else return None.

    The merged line is the PCA line of all the members' LiDAR points (the
    same fit_line() as for single segments), so every observation counts.
    """
    items = sorted(items, key=lambda item: item["id"])
    fit = fit_line(np.vstack([item["points"] for item in items]))
    if fit["degenerate"] or fit["rms_error"] > max_rms:
        return None
    centroid, direction = fit["centroid"], fit["direction"]
    intervals = []
    for item in items:
        if line_angle_deg(item["direction"], direction) > max_angle_deg:
            return None
        if distance_to_line(np.array([item["start"], item["end"]]),
                            centroid, direction).max() > max_offset:
            return None
        intervals.append(projected_interval(item["start"], item["end"],
                                            centroid, direction))
    intervals.sort()
    reach = intervals[0][1]
    for low, high in intervals[1:]:  # the members must cover the line
        if low - reach > max_gap:
            return None
        reach = max(reach, high)
    start = centroid + intervals[0][0] * direction
    end = centroid + reach * direction
    if (end[0], end[1]) < (start[0], start[1]):  # canonical orientation
        start, end = end, start
    scan_ids = sorted({item["scan_id"] for item in items})
    return {"start": start, "end": end,
            "length": float(np.linalg.norm(end - start)),
            "direction": (end - start) / np.linalg.norm(end - start),
            "member_ids": [item["id"] for item in items],
            "scan_ids": scan_ids, "support": len(scan_ids),
            "observations": fit["points"], "rms_error": fit["rms_error"],
            "recovered_members": sum(item["origin"] == "recovered"
                                     for item in items)}


def single_segment(item):
    """A segment that merged with nothing, as a merged segment (same ends)."""
    start, end = item["start"], item["end"]
    if (end[0], end[1]) < (start[0], start[1]):
        start, end = end, start
    return {"start": start, "end": end, "length": item["length"],
            "direction": (end - start) / item["length"],
            "member_ids": [item["id"]], "scan_ids": [item["scan_id"]],
            "support": 1, "observations": len(item["points"]),
            "rms_error": item["segment"]["fit"]["rms_error"],
            "recovered_members": int(item["origin"] == "recovered")}


def merge_segments(segments, max_angle_deg=MERGE_MAX_ANGLE_DEG,
                   max_offset=MERGE_MAX_OFFSET_M, max_gap=MERGE_MAX_GAP_M,
                   max_rms=MERGE_MAX_RMS_M):
    """Merge the accepted segments of all scans; return (merged, items).

    segments - accepted_segments() (original + recovered); never modified
    merged   - list of merged segments ("id" M001..., "start", "end",
               "length", "member_ids", "scan_ids", "support", ...)
    items    - the input segments with their canonical IDs (S001...)
    """
    items = describe_segments(segments)
    options = {"max_angle_deg": max_angle_deg, "max_offset": max_offset,
               "max_gap": max_gap}
    pairs = []
    for i, a in enumerate(items):
        for b in items[i + 1:]:
            offset = pair_compatibility(a, b, **options)
            if offset is not None:
                pairs.append((offset, a["id"], b["id"]))
    pairs.sort()  # best evidence first; ties broken by the canonical IDs

    by_id = {item["id"]: item for item in items}
    group_of = {item["id"]: item["id"] for item in items}   # id -> group
    members = {item["id"]: [item["id"]] for item in items}  # group -> ids
    merged_of = {}                                          # group -> merge
    for _, id_a, id_b in pairs:
        group_a, group_b = group_of[id_a], group_of[id_b]
        if group_a == group_b:
            continue
        candidate = members[group_a] + members[group_b]
        result = merge_group([by_id[member] for member in candidate],
                             max_rms=max_rms, **options)
        if result is None:
            continue  # would join different surfaces: keep apart
        keep, gone = sorted([group_a, group_b])
        members[keep] = candidate
        for member in members.pop(gone):
            group_of[member] = keep
        merged_of.pop(gone, None)
        merged_of[keep] = result

    merged = [merged_of.get(group) or single_segment(by_id[group])
              for group in sorted(members)]
    for number, segment in enumerate(merged, start=1):
        segment["id"] = f"M{number:03d}"
    return merged, items


def merge_statistics(merged, items):
    """Counts, lengths and the distribution of supporting scans."""
    support = {}
    for segment in merged:
        support[segment["support"]] = support.get(segment["support"], 0) + 1
    multi = [segment for segment in merged if len(segment["member_ids"]) > 1]
    return {"original_segments": len(items),
            "merged_segments": len(merged),
            "consolidated": len(items) - len(merged),
            "groups_with_several_members": len(multi),
            "original_length_m": sum(item["length"] for item in items),
            "merged_length_m": sum(segment["length"] for segment in merged),
            "support_distribution": dict(sorted(support.items()))}


def print_merge_statistics(stats, merged):
    """Print the merging summary and the longest merged walls."""
    print(f"Merging (lines within {MERGE_MAX_ANGLE_DEG} deg, endpoints "
          f"within {MERGE_MAX_OFFSET_M} m, gaps <= {MERGE_MAX_GAP_M} m, "
          f"merged RMS <= {MERGE_MAX_RMS_M} m):")
    print(f"  {stats['original_segments']} segments "
          f"({stats['original_length_m']:.1f} m of observations) -> "
          f"{stats['merged_segments']} merged segments "
          f"({stats['merged_length_m']:.1f} m); "
          f"{stats['consolidated']} duplicate observations consolidated "
          f"into {stats['groups_with_several_members']} multi-segment "
          f"walls")
    print("  scans supporting each merged segment: " + ", ".join(
        f"{count} scan(s): {number}"
        for count, number in stats["support_distribution"].items()))
    print("  longest merged segments:")
    for segment in sorted(merged, key=lambda s: -s["length"])[:5]:
        (x1, y1), (x2, y2) = segment["start"], segment["end"]
        print(f"    {segment['id']}: ({x1:5.2f}, {y1:5.2f}) -> ({x2:5.2f}, "
              f"{y2:5.2f}), {segment['length']:.2f} m, "
              f"{len(segment['member_ids'])} segments from scans "
              f"{segment['scan_ids']}")


# ---------------------------------------------------------------------------
# 6. Display
# ---------------------------------------------------------------------------
def scan_colors(scans):
    """Each scan's Part 1 colour (a fallback palette if none was saved)."""
    palette = plt.get_cmap("tab10")
    return [scan.get("color") or palette(index % 10)
            for index, scan in enumerate(scans)]


def draw_lidar_positions(axes, scans, colors):
    """Dot (in `colors`), heading arrow and scan number at each pose.

    Returns the created artists (the layer viewer shows / hides them).
    """
    artists = []
    for scan, color in zip(scans, colors):
        x_r, y_r, theta_deg = scan["pose"]
        artists += axes.plot(x_r, y_r, "o", color=color,
                             markeredgecolor="black", markersize=7, zorder=5)
        theta_rad = np.radians(theta_deg)
        artists.append(axes.annotate(
            "", xy=(x_r + 0.5 * np.cos(theta_rad),
                    y_r + 0.5 * np.sin(theta_rad)),
            xytext=(x_r, y_r), zorder=5,
            arrowprops={"arrowstyle": "->", "color": "black"}))
        artists.append(axes.annotate(
            str(scan["id"]), (x_r, y_r), xytext=(4, 4),
            textcoords="offset points", fontsize=8, fontweight="bold",
            zorder=6))
    return artists


def finish_axes(figure, axes, title, legend_title=None):
    """Equal scale, metres, grid, title and legend outside on the right."""
    axes.set_aspect("equal")  # 1 m on X has the same length as 1 m on Y
    axes.set_xlabel("X (m)")
    axes.set_ylabel("Y (m)")
    axes.grid(True, alpha=0.4)
    axes.set_title(title)
    if axes.get_legend_handles_labels()[0]:
        axes.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0),
                    fontsize="small", markerscale=4, title=legend_title,
                    title_fontsize="small")


def plot_point_cloud(scans, color_by_scan=COLOR_BY_SCAN,
                     show_lidar_positions=SHOW_LIDAR_POSITIONS, title=None):
    """Milestone 1 view: the global point cloud; return the figure.

    color_by_scan=True draws each scan in its Part 1 colour (to check how
    well the scans overlap); False draws everything in SINGLE_COLOR.
    """
    figure, axes = plt.subplots(figsize=(12, 8), layout="constrained")
    colors = scan_colors(scans)
    points, _ = stack_point_cloud(scans)
    if color_by_scan:
        for scan, color in zip(scans, colors):
            axes.scatter(scan["points_global"][:, 0],
                         scan["points_global"][:, 1], s=POINT_SIZE,
                         color=color, label=f"{scan['name']} "
                                            f"({len(scan['points_global'])})")
    else:
        axes.scatter(points[:, 0], points[:, 1], s=POINT_SIZE,
                     color=SINGLE_COLOR,
                     label=f"all measurements ({len(points)})")
    if show_lidar_positions:
        draw_lidar_positions(axes, scans, colors if color_by_scan
                             else ["white"] * len(scans))
    finish_axes(figure, axes,
                title or (f"Part 3 - Vector map, step 1: global LiDAR point "
                          f"cloud ({len(scans)} scans, {len(points)} points)"),
                "dots = LiDAR positions" if show_lidar_positions and scans
                else None)
    return figure


def plot_clusters(scans, clusters, show_rejected=SHOW_REJECTED_POINTS,
                  show_lidar_positions=SHOW_LIDAR_POSITIONS, title=None):
    """Milestone 2 view: each accepted cluster in its own colour.

    Colours come from a 20-colour palette in cluster order, so neighbouring
    clusters of a scan always differ (the palette repeats for far-apart
    clusters). With show_rejected, light grey shows what was removed:
    crosses = rejected by pre-processing, dots = clusters too small.
    """
    figure, axes = plt.subplots(figsize=(12, 8), layout="constrained")
    palette = plt.get_cmap("tab20")
    accepted = [cluster for cluster in clusters if cluster["accepted"]]
    for number, cluster in enumerate(accepted):
        axes.scatter(cluster["points_global"][:, 0],
                     cluster["points_global"][:, 1], s=POINT_SIZE + 1,
                     color=palette(number % 20),
                     label=(f"accepted clusters ({len(accepted)}, colours "
                            f"repeat)" if number == 0 else None))
    if show_rejected:
        small = [cluster["points_global"] for cluster in clusters
                 if not cluster["accepted"]]
        if small:
            small_points = np.vstack(small)
            axes.scatter(small_points[:, 0], small_points[:, 1],
                         s=POINT_SIZE, color="0.75", zorder=0,
                         label=f"small clusters (< {MIN_CLUSTER_POINTS} "
                               f"points): {len(small_points)} points")
        filtered = [scan["points_global"][~scan["valid"]] for scan in scans]
        filtered = np.vstack(filtered) if filtered else np.empty((0, 2))
        filtered = filtered[np.all(np.isfinite(filtered), axis=1)]
        if len(filtered):
            axes.scatter(filtered[:, 0], filtered[:, 1], s=12, marker="x",
                         color="0.6", zorder=0,
                         label=f"filtered (range): {len(filtered)} points")
    if show_lidar_positions:
        draw_lidar_positions(axes, scans, scan_colors(scans))
    finish_axes(figure, axes,
                title or (f"Part 3 - Vector map, step 2: {len(accepted)} "
                          f"clusters of >= {MIN_CLUSTER_POINTS} points "
                          f"(threshold {SEGMENT_DISTANCE_M} m, "
                          f"{len(scans)} scans)"),
                "dots = LiDAR positions" if show_lidar_positions and scans
                else None)
    return figure


def unused_points(record):
    """Points of a rejected cluster that no recovered segment uses."""
    used = [piece["cluster"]["indices"] for piece in record["pieces"]
            if piece["accepted"]]
    keep = ~np.isin(record["cluster"]["indices"],
                    np.concatenate(used) if used else [])
    return record["cluster"]["points_global"][keep]


def plot_segments(scans, records, show_points=SHOW_POINTS_UNDER_SEGMENTS,
                  show_rejected=SHOW_REJECTED_POINTS,
                  show_lidar_positions=SHOW_LIDAR_POSITIONS,
                  distinguish_recovered=DISTINGUISH_RECOVERED_SEGMENTS,
                  title=None):
    """Milestones 3-4 view: the fitted line segments; return the figure.

    records - fit_segments()'s result (one record per cluster).
    Original segments (whole clusters) are thick lines in their scan's
    colour; Douglas-Peucker recovered segments are black and dashed when
    distinguish_recovered is True. Several scans see the same wall, so
    walls appear several times until segments are merged. Black dots mark
    the endpoints. Faint grey: points used by segments; faint red
    (show_rejected): points no segment could use.
    """
    figure, axes = plt.subplots(figsize=(12, 8), layout="constrained")
    colors = scan_colors(scans)
    originals = [record for record in records if record["accepted"]]
    recovered = [piece for record in records for piece in record["pieces"]
                 if piece["accepted"]]
    used = originals + recovered
    without_segment = [record for record in records
                       if not record["accepted"]
                       and not any(piece["accepted"]
                                   for piece in record["pieces"])]

    if show_points and used:
        under = np.vstack([segment["cluster"]["points_global"]
                           for segment in used])
        axes.scatter(under[:, 0], under[:, 1], s=POINT_SIZE, color="0.7",
                     zorder=1, label=f"points used by segments "
                                     f"({len(under)})")
    refused = [unused_points(record) for record in records
               if not record["accepted"]]
    refused = np.vstack(refused) if refused else np.empty((0, 2))
    if show_rejected and len(refused):
        axes.scatter(refused[:, 0], refused[:, 1], s=POINT_SIZE,
                     color="salmon", alpha=0.6, zorder=1,
                     label=f"points not used by any segment "
                           f"({len(refused)})")
    for group, label, style in (
            (originals, f"original segments ({len(originals)}, colour = "
                        f"scan)", "solid"),
            (recovered, f"recovered by Douglas-Peucker ({len(recovered)})",
             (0, (3, 1.5)) if distinguish_recovered else "solid")):
        if not group:
            continue
        lines = [[segment["fit"]["start"], segment["fit"]["end"]]
                 for segment in group]
        line_colors = (["black"] * len(group)
                       if group is recovered and distinguish_recovered
                       else [colors[segment["cluster"]["scan_index"]]
                             for segment in group])
        axes.add_collection(LineCollection(
            lines, linewidths=2.5, zorder=2, colors=line_colors,
            linestyles=style, label=label))
    if used:
        ends = np.array([point for segment in used
                         for point in (segment["fit"]["start"],
                                       segment["fit"]["end"])])
        axes.scatter(ends[:, 0], ends[:, 1], s=9, color="black", zorder=3,
                     label="segment endpoints")
        axes.autoscale_view()
    if show_lidar_positions:
        draw_lidar_positions(axes, scans, colors)
    finish_axes(figure, axes,
                title or (f"Part 3 - Vector map, step 4: {len(used)} "
                          f"segments ({len(originals)} original + "
                          f"{len(recovered)} recovered) from {len(records)} "
                          f"clusters, {len(without_segment)} without a "
                          f"segment, {len(scans)} scans"),
                "dots = LiDAR positions" if show_lidar_positions and scans
                else None)
    return figure


def plot_merged(scans, merged, items, show_unmerged=SHOW_UNMERGED_UNDER_MERGED,
                color_by_support=MERGED_COLOR_BY_SUPPORT,
                show_lidar_positions=SHOW_LIDAR_POSITIONS, title=None):
    """Milestone 5 view: the merged vector map; return the figure.

    Merged segments are thick lines; with color_by_support their colour is
    the number of scans that saw them (a colour bar explains it): a wall
    seen by many scans is more trustworthy than one seen once. The
    unmerged segments are drawn thin and faint underneath (show_unmerged).
    """
    figure, axes = plt.subplots(figsize=(12, 8), layout="constrained")
    if show_unmerged and items:
        axes.add_collection(LineCollection(
            [[item["start"], item["end"]] for item in items], linewidths=1,
            colors="0.65", zorder=1,
            label=f"unmerged segments ({len(items)})"))
    if merged:
        lines = [[segment["start"], segment["end"]] for segment in merged]
        support = np.array([segment["support"] for segment in merged])
        if color_by_support:
            norm = matplotlib.colors.Normalize(1, max(2, support.max()))
            collection = LineCollection(lines, linewidths=3, zorder=2,
                                        cmap="viridis_r", norm=norm,
                                        label=f"merged segments "
                                              f"({len(merged)})")
            collection.set_array(support)
            axes.add_collection(collection)
            colorbar = figure.colorbar(collection, ax=axes, shrink=0.6,
                                       pad=0.01)
            colorbar.set_label("scans supporting the segment")
        else:
            axes.add_collection(LineCollection(
                lines, linewidths=3, colors="navy", zorder=2,
                label=f"merged segments ({len(merged)})"))
        ends = np.array([point for line in lines for point in line])
        axes.scatter(ends[:, 0], ends[:, 1], s=10, color="black", zorder=3,
                     label="endpoints")
        axes.autoscale_view()
    if show_lidar_positions:
        draw_lidar_positions(axes, scans, scan_colors(scans))
    finish_axes(figure, axes,
                title or (f"Part 3 - Vector map, step 5: {len(items)} "
                          f"segments merged into {len(merged)} "
                          f"({sum(s['length'] for s in merged):.1f} m), "
                          f"{len(scans)} scans"),
                "dots = LiDAR positions" if show_lidar_positions and scans
                else None)
    return figure


# ---------------------------------------------------------------------------
# 6b. Interactive layer viewer (VIEW_MODE = "layers")
# ---------------------------------------------------------------------------
# Every layer is drawn ONCE, when the window opens, from the results that
# main() has already computed. A tick box only switches its artists'
# visibility (artist.set_visible) and asks Matplotlib to redraw: nothing is
# recomputed or recreated, and the axis limits are not touched, so a zoom
# or a pan made with the toolbar is kept.
ACTIVE_VIEWER = None  # the open "layers" viewer (keeps its widgets alive)
LAYER_KEYS = ("points", "clusters", "pca", "recovered", "merged", "lidar",
              "endpoints")
ORIGINAL_STYLE = {"colors": "tab:blue", "linewidths": 1.6,
                  "linestyles": "solid"}
RECOVERED_STYLE = {"colors": "tab:orange", "linewidths": 1.6,
                   "linestyles": (0, (3, 1.5))}
MERGED_STYLE = {"colors": "black", "linewidths": 3.2, "linestyles": "solid"}


def layer_home_limits(scans, margin_fraction=0.04):
    """Axis limits showing every measured point and LiDAR position."""
    points, _ = stack_point_cloud(scans)
    points = points[np.all(np.isfinite(points), axis=1)]
    poses = np.array([scan["pose"][:2] for scan in scans])
    everything = np.vstack([points, poses]) if len(points) else poses
    low, high = everything.min(axis=0), everything.max(axis=0)
    margin = margin_fraction * max(high - low) + 0.2
    return ((low[0] - margin, high[0] + margin),
            (low[1] - margin, high[1] + margin))


def build_layer_viewer(scans, clusters, records, merged,
                       visible_at_start=LAYERS_VISIBLE_AT_START,
                       inspection_style=INSPECTION_STYLE, figure=None):
    """Open the map with one tick box per layer; return the viewer.

    The returned dictionary keeps the widgets alive (Matplotlib widgets stop
    working if Python deletes them) and gives access to everything:
        "figure", "axes", "layers" {key: [artists]}, "labels" {key: text},
        "checkboxes", "buttons", "home_limits",
        "set_layer(key, visible)", "show_all()", "hide_all()",
        "reset_view()", "visible_layers()"
    """
    figure = figure if figure is not None else plt.figure(figsize=(13, 8))
    axes = figure.add_axes([0.05, 0.06, 0.66, 0.88])
    palette = plt.get_cmap("tab20")
    layers = {key: [] for key in LAYER_KEYS}
    originals = [record for record in records if record["accepted"]]
    recovered = [piece for record in records for piece in record["pieces"]
                 if piece["accepted"]]
    accepted_clusters = [cluster for cluster in clusters
                         if cluster["accepted"]]

    # 1. All measured points: ONE scatter (colours per point if needed).
    points, owner = stack_point_cloud(scans)
    finite = np.all(np.isfinite(points), axis=1)
    if inspection_style:
        point_colors = "0.72"
    else:
        scan_rgba = [matplotlib.colors.to_rgba(color)
                     for color in scan_colors(scans)]
        point_colors = [scan_rgba[index] for index in owner[finite]]
    layers["points"].append(axes.scatter(
        points[finite, 0], points[finite, 1], s=1.5 if inspection_style
        else 2, c=point_colors, zorder=1, linewidths=0))
    # 2. Accepted clusters: ONE scatter, a palette colour per cluster.
    if accepted_clusters:
        cluster_points = np.vstack([cluster["points_global"]
                                    for cluster in accepted_clusters])
        cluster_colors = np.vstack([np.tile(palette(number % 20),
                                            (len(cluster["indices"]), 1))
                                    for number, cluster
                                    in enumerate(accepted_clusters)])
        layers["clusters"].append(axes.scatter(
            cluster_points[:, 0], cluster_points[:, 1], s=4,
            c=cluster_colors, zorder=2, linewidths=0))
    # 3-5. Line layers: ONE LineCollection each, one style each.
    for key, segments, style, zorder in (
            ("pca", [[r["fit"]["start"], r["fit"]["end"]] for r in originals],
             ORIGINAL_STYLE, 3),
            ("recovered", [[p["fit"]["start"], p["fit"]["end"]]
                           for p in recovered], RECOVERED_STYLE, 3),
            ("merged", [[s["start"], s["end"]] for s in merged],
             MERGED_STYLE, 4)):
        if segments:
            layers[key].append(axes.add_collection(
                LineCollection(segments, zorder=zorder, **style)))
    # 6. LiDAR positions, heading arrows and scan numbers.
    layers["lidar"] = draw_lidar_positions(axes, scans, scan_colors(scans))
    # 7. Endpoints of the merged segments.
    if merged:
        ends = np.array([point for segment in merged
                         for point in (segment["start"], segment["end"])])
        layers["endpoints"].append(axes.scatter(
            ends[:, 0], ends[:, 1], s=14, color="crimson", zorder=5,
            linewidths=0))

    home = layer_home_limits(scans)
    axes.set_xlim(*home[0])
    axes.set_ylim(*home[1])
    axes.set_aspect("equal")  # 1 m on X has the same length as 1 m on Y
    axes.set_xlabel("X (m)")
    axes.set_ylabel("Y (m)")
    axes.grid(True, alpha=0.35)
    axes.set_title(f"Part 3 - Vector map layers: {len(scans)} scans, "
                   f"{int(finite.sum())} points, {len(originals)} PCA + "
                   f"{len(recovered)} recovered segments -> {len(merged)} "
                   f"merged")

    labels = {
        "points": f"Original LiDAR points ({int(finite.sum())})",
        "clusters": f"Segmented clusters ({len(accepted_clusters)})",
        "pca": f"Original PCA lines ({len(originals)})",
        "recovered": f"Douglas-Peucker lines ({len(recovered)})",
        "merged": f"Merged vector lines ({len(merged)})",
        "lidar": f"LiDAR positions ({len(scans)})",
        "endpoints": f"Merged endpoints ({2 * len(merged)})"}

    # Side panel: tick boxes, buttons, then a small legend of the styles.
    figure.text(0.735, 0.955, "Layers", fontweight="bold")
    checkbox_axes = figure.add_axes([0.73, 0.60, 0.26, 0.34])
    checkboxes = CheckButtons(
        checkbox_axes, [labels[key] for key in LAYER_KEYS],
        actives=[key in visible_at_start for key in LAYER_KEYS])
    for text in checkboxes.labels:
        text.set_fontsize(9)
    show_button = Button(figure.add_axes([0.73, 0.53, 0.125, 0.05]),
                         "Show all")
    hide_button = Button(figure.add_axes([0.865, 0.53, 0.125, 0.05]),
                         "Hide all")
    reset_button = Button(figure.add_axes([0.73, 0.465, 0.26, 0.05]),
                          "Reset view")
    legend_axes = figure.add_axes([0.73, 0.06, 0.26, 0.37])
    legend_axes.axis("off")
    legend_axes.legend(handles=[
        Line2D([], [], marker="o", linestyle="none", color="0.72"
               if inspection_style else "tab:gray", markersize=4,
               label="measured points" + ("" if inspection_style
                                          else " (scan colours)")),
        Line2D([], [], marker="o", linestyle="none", color=palette(0),
               markersize=4, label="clusters (arbitrary colours)"),
        Line2D([], [], color=ORIGINAL_STYLE["colors"],
               linewidth=ORIGINAL_STYLE["linewidths"],
               label="PCA segment (one scan)"),
        Line2D([], [], color=RECOVERED_STYLE["colors"],
               linewidth=RECOVERED_STYLE["linewidths"], linestyle="--",
               label="Douglas-Peucker segment"),
        Line2D([], [], color=MERGED_STYLE["colors"],
               linewidth=MERGED_STYLE["linewidths"],
               label="merged vector segment"),
        Line2D([], [], marker="o", linestyle="none", color="crimson",
               markersize=4, label="merged endpoint"),
        Line2D([], [], marker="o", linestyle="none", color="white",
               markeredgecolor="black", markersize=7,
               label="LiDAR position (+ heading, scan no.)")],
        loc="upper left", fontsize=8, frameon=True, title="Legend",
        title_fontsize=9)

    def apply(key, visible):
        for artist in layers[key]:
            artist.set_visible(visible)

    def visible_layers():
        return [key for key, active in zip(LAYER_KEYS,
                                           checkboxes.get_status()) if active]

    def on_checkbox(label):
        """A tick box changed: show / hide that layer only, then redraw."""
        key = LAYER_KEYS[[labels[k] for k in LAYER_KEYS].index(label)]
        apply(key, key in visible_layers())
        figure.canvas.draw_idle()

    def set_layer(key, visible):
        """Tick or untick a layer (through its box, so both stay in sync)."""
        if (key in visible_layers()) != bool(visible):
            checkboxes.set_active(LAYER_KEYS.index(key))  # calls on_checkbox

    def show_all():
        for key in LAYER_KEYS:
            set_layer(key, True)

    def hide_all():
        for key in LAYER_KEYS:
            set_layer(key, False)

    def reset_view():
        """Back to the whole map (the layers' visibility is unchanged)."""
        axes.set_xlim(*home[0])
        axes.set_ylim(*home[1])
        figure.canvas.draw_idle()

    checkboxes.on_clicked(on_checkbox)
    show_button.on_clicked(lambda event: show_all())
    hide_button.on_clicked(lambda event: hide_all())
    reset_button.on_clicked(lambda event: reset_view())
    for key in LAYER_KEYS:
        apply(key, key in visible_at_start)

    return {"figure": figure, "axes": axes, "layers": layers,
            "labels": labels, "checkboxes": checkboxes,
            "buttons": (show_button, hide_button, reset_button),
            "home_limits": home, "set_layer": set_layer,
            "show_all": show_all, "hide_all": hide_all,
            "reset_view": reset_view, "visible_layers": visible_layers}


def check_view_mode(value):
    """Refuse an unknown VIEW_MODE instead of silently showing another view."""
    if value not in VALID_VIEW_MODES:
        raise ValueError(f"VIEW_MODE = {value!r} is not valid; choose one of "
                         f"{', '.join(map(repr, VALID_VIEW_MODES))}")


# ---------------------------------------------------------------------------
# 7. Program
# ---------------------------------------------------------------------------
def main():
    check_view_mode(VIEW_MODE)  # a clear error for a wrong value
    print(f"=== Part 3: vector map - view '{VIEW_MODE}' ===")
    session_path = resolve_session_path(SESSION_FILE)
    if not session_path.is_file():
        available = sorted(path.name for path in SCANS_DIR.glob("*.json"))
        print(f"Session file not found: {session_path}")
        print(f"  (SESSION_FILE = {SESSION_FILE!r}; a bare name is looked "
              f"for in {SCANS_DIR})")
        print(f"  Files available in scans/: "
              f"{', '.join(available) if available else 'none'}")
        return
    print(f"Session: {session_path} (read only)")
    fingerprint_before = file_sha256(session_path)
    try:
        scans, hidden_count = load_point_cloud(session_path)
    except SessionError as error:
        print(f"Cannot load the session: {error}")
        return
    if not scans:
        print(f"The session has no visible scan ({hidden_count} hidden): "
              f"nothing to show.")
        return

    points, _ = stack_point_cloud(scans)
    hidden_note = (f" ({hidden_count} hidden scan(s) skipped)"
                   if hidden_count else "")
    print(f"{len(scans)} scans, {len(points)} points{hidden_note}:")
    rejection_counts = preprocess_scans(scans)
    clusters = segment_scans(scans)
    for scan_index, scan in enumerate(scans):
        x_r, y_r, theta_deg = scan["pose"]
        own = [c for c in clusters if c["scan_index"] == scan_index]
        print(f"  {scan['name']:8s} pose ({x_r:6.2f} m, {y_r:6.2f} m, "
              f"{theta_deg:6.1f} deg)  {len(scan['points_global']):5d} "
              f"points, {int(np.sum(~scan['valid'])):3d} filtered, "
              f"{len(own):4d} clusters "
              f"({sum(c['accepted'] for c in own):3d} accepted)")
    finite = points[np.all(np.isfinite(points), axis=1)]
    low, high = finite.min(axis=0), finite.max(axis=0)
    print(f"Point cloud: x {low[0]:.2f} to {high[0]:.2f} m, y {low[1]:.2f} "
          f"to {high[1]:.2f} m ({high[0] - low[0]:.1f} m x "
          f"{high[1] - low[1]:.1f} m)")
    print_statistics(segmentation_statistics(scans, clusters,
                                             rejection_counts),
                     any(scan.get("quality") is not None for scan in scans))
    segments = fit_segments(clusters, split=DP_ENABLED,
                            dp_tolerance=DP_TOLERANCE_M,
                            dp_max_depth=DP_MAX_DEPTH,
                            dp_min_points=DP_MIN_POINTS)
    print_line_statistics(line_fitting_statistics(segments))
    merged, merge_items = merge_segments(
        accepted_segments(segments), max_angle_deg=MERGE_MAX_ANGLE_DEG,
        max_offset=MERGE_MAX_OFFSET_M, max_gap=MERGE_MAX_GAP_M,
        max_rms=MERGE_MAX_RMS_M)
    print_merge_statistics(merge_statistics(merged, merge_items), merged)
    print(f"Session file unchanged: "
          f"{file_sha256(session_path) == fingerprint_before}")

    if VIEW_MODE == "points":
        plot_point_cloud(scans, color_by_scan=COLOR_BY_SCAN,
                         show_lidar_positions=SHOW_LIDAR_POSITIONS)
    elif VIEW_MODE == "layers":
        # Kept in a module variable: the tick boxes stop working if Python
        # deletes the widgets while the window is open.
        global ACTIVE_VIEWER
        ACTIVE_VIEWER = build_layer_viewer(
            scans, clusters, segments, merged,
            visible_at_start=LAYERS_VISIBLE_AT_START,
            inspection_style=INSPECTION_STYLE)
    elif VIEW_MODE == "merged":
        plot_merged(scans, merged, merge_items,
                    show_unmerged=SHOW_UNMERGED_UNDER_MERGED,
                    color_by_support=MERGED_COLOR_BY_SUPPORT,
                    show_lidar_positions=SHOW_LIDAR_POSITIONS)
    elif VIEW_MODE == "segments":
        plot_segments(scans, segments,
                      show_points=SHOW_POINTS_UNDER_SEGMENTS,
                      show_rejected=SHOW_REJECTED_POINTS,
                      show_lidar_positions=SHOW_LIDAR_POSITIONS,
                      distinguish_recovered=DISTINGUISH_RECOVERED_SEGMENTS)
    else:
        plot_clusters(scans, clusters, show_rejected=SHOW_REJECTED_POINTS,
                      show_lidar_positions=SHOW_LIDAR_POSITIONS)
    plt.show()


# ---------------------------------------------------------------------------
# Automated tests (no window): python -m unittest 03_vector_mapping -v
# ---------------------------------------------------------------------------
CLASSROOM_FINAL = SCANS_DIR / "classroom_final.json"


def make_scan_json(scan_id, pose, points, visible=True, color=None):
    """One scan in Part 1's version 2 format."""
    entry = {"id": scan_id, "name": f"Scan {scan_id}", "visible": visible,
             "aligned": scan_id != 1,
             "pose": {"x_m": pose[0], "y_m": pose[1], "theta_deg": pose[2]},
             "points_local_m": points}
    if color:
        entry["color"] = color
    return entry


def write_session(folder, scans_json, name="session.json", **changes):
    """Write a version 2 session file; return its path."""
    data = {"format": "ld06_slam_session", "version": 2, "scans": scans_json}
    data.update(changes)
    path = Path(folder) / name
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def make_scans_json(count, seed=0):
    """`count` valid scans with random poses and 20 points each."""
    rng = np.random.default_rng(seed)
    return [make_scan_json(scan_id, (float(rng.uniform(-3, 3)),
                                     float(rng.uniform(-3, 3)),
                                     float(rng.uniform(-180, 180))),
                           rng.uniform(-4, 4, (20, 2)).round(4).tolist())
            for scan_id in range(1, count + 1)]


class Part3TestCase(unittest.TestCase):
    """Temporary folder + off-screen Matplotlib for every test."""

    def setUp(self):
        plt.switch_backend("Agg")
        self.folder = tempfile.TemporaryDirectory()

    def tearDown(self):
        plt.close("all")
        self.folder.cleanup()

    def run_main(self, session_file, scans_dir=None, view_mode="points"):
        """Run main() off-screen; return (console text, figure or None).

        The milestone 1 tests use the "points" view; the segmentation
        tests ask for "clusters".
        """
        module = sys.modules[__name__]
        output = io.StringIO()
        with mock.patch.object(module, "SESSION_FILE", session_file), \
                mock.patch.object(module, "SCANS_DIR",
                                  Path(scans_dir) if scans_dir else SCANS_DIR), \
                mock.patch.object(module, "VIEW_MODE", view_mode), \
                mock.patch.object(plt, "show"), \
                contextlib.redirect_stdout(output):
            main()
        figure = plt.gcf() if plt.get_fignums() else None
        return output.getvalue(), figure


class LoadingTests(Part3TestCase):
    def test_loads_poses_points_and_keeps_point_order(self):
        points = [[3.0, 0.0], [0.0, 2.0], [-1.0, 0.0], [0.5, -0.5]]
        path = write_session(self.folder.name, [
            make_scan_json(1, (0.0, 0.0, 0.0), [[1.0, 0.0]]),
            make_scan_json(5, (1.5, -2.0, 90.0), points)])
        scans, hidden = load_point_cloud(path)
        self.assertEqual(([scan["id"] for scan in scans], hidden), ([1, 5], 0))
        self.assertEqual(scans[1]["pose"], (1.5, -2.0, 90.0))  # as saved
        np.testing.assert_array_equal(scans[1]["points_local"], points)
        # Same order as in the file (needed by the segmentation later).
        np.testing.assert_allclose(scans[1]["points_global"],
                                   [[1.5, 1.0], [-0.5, -2.0], [1.5, -3.0],
                                    [2.0, -1.5]], atol=1e-12)

    def test_invalid_sessions_are_refused(self):
        good = make_scan_json(1, (0.0, 0.0, 0.0), [[1.0, 0.0]])
        # (A NaN MEASUREMENT is no longer a reason to refuse the file:
        # see test_bad_measurements_are_kept_in_place.)
        cases = {"wrong format": ([good], {"format": "other"}),
                 "old version": ([good], {"version": 1}),
                 "no points": ([dict(good, points_local_m=[])], {}),
                 "pose NaN": ([dict(good, pose={"x_m": float("nan"),
                                                "y_m": 0, "theta_deg": 0})],
                              {}),
                 "pose missing": ([{k: v for k, v in good.items()
                                    if k != "pose"}], {})}
        for name, (scans_json, changes) in cases.items():
            with self.subTest(name):
                path = write_session(self.folder.name, scans_json, **changes)
                with self.assertRaises(SessionError):
                    load_point_cloud(path)
        broken = Path(self.folder.name) / "broken.json"
        broken.write_text("{oops", encoding="utf-8")
        with self.assertRaises(SessionError):
            load_point_cloud(broken)

    def test_bad_measurements_are_kept_in_place(self):
        raw = [[1.0, 0.0], [float("nan"), 0.0], [1.0, float("inf")],
               ["text", 1.0], [None, 2.0], [1.0], [True, 0.0], [0.0, 1.0]]
        path = write_session(self.folder.name, [
            make_scan_json(1, (0.0, 0.0, 0.0), raw)])
        scans, _ = load_point_cloud(path)
        points = scans[0]["points_local"]
        self.assertEqual(len(points), 8)  # nothing dropped or reordered
        np.testing.assert_array_equal(points[0], [1.0, 0.0])
        np.testing.assert_array_equal(points[7], [0.0, 1.0])
        reasons = preprocess_measurements(points)
        self.assertEqual(list(reasons),
                         [KEPT] + [INVALID] * 6 + [KEPT])

    def test_hidden_scans_are_skipped_and_counted(self):
        scans_json = make_scans_json(4)
        scans_json[1]["visible"] = False
        scans_json[3]["visible"] = False
        path = write_session(self.folder.name, scans_json)
        scans, hidden = load_point_cloud(path)
        self.assertEqual([scan["id"] for scan in scans], [1, 3])
        self.assertEqual(hidden, 2)
        points, owner = stack_point_cloud(scans)
        self.assertEqual(len(points), 40)
        np.testing.assert_array_equal(np.bincount(owner), [20, 20])
        output, figure = self.run_main(str(path))
        self.assertIn("2 hidden scan(s) skipped", output)
        self.assertIn("2 scans", figure.axes[0].get_title())

    def test_only_hidden_scans(self):
        scans_json = [dict(scan, visible=False) for scan in make_scans_json(2)]
        output, figure = self.run_main(str(write_session(self.folder.name,
                                                         scans_json)))
        self.assertIn("no visible scan (2 hidden)", output)
        self.assertIsNone(figure)


class TransformTests(Part3TestCase):
    def test_global_points_use_each_scans_saved_pose(self):
        local = [[1.0, 0.0], [0.0, 1.0]]
        cases = {(0.0, 0.0, 0.0): [[1, 0], [0, 1]],
                 (2.0, -1.0, 0.0): [[3, -1], [2, 0]],
                 (0.0, 0.0, 90.0): [[0, 1], [-1, 0]],
                 (1.0, 1.0, 180.0): [[0, 1], [1, 0]],
                 (0.0, 0.0, -90.0): [[0, -1], [1, 0]]}
        path = write_session(self.folder.name, [
            make_scan_json(index + 1, pose, local)
            for index, pose in enumerate(cases)])
        scans, _ = load_point_cloud(path)
        for scan, expected in zip(scans, cases.values()):
            with self.subTest(pose=scan["pose"]):
                np.testing.assert_allclose(scan["points_global"], expected,
                                           atol=1e-12)

    def test_formula_matches_rotation_then_translation(self):
        rng = np.random.default_rng(3)
        local = rng.uniform(-5, 5, (50, 2))
        x_r, y_r, theta = 1.86, 9.0, -0.3
        rad = np.radians(theta)
        expected = np.column_stack([
            x_r + local[:, 0] * np.cos(rad) - local[:, 1] * np.sin(rad),
            y_r + local[:, 0] * np.sin(rad) + local[:, 1] * np.cos(rad)])
        np.testing.assert_allclose(local_to_global(local, (x_r, y_r, theta)),
                                   expected, atol=1e-12)

    def test_same_global_points_as_part_2(self):
        if not CLASSROOM_FINAL.exists():
            self.skipTest("classroom_final.json absent")
        scans, _ = load_point_cloud(CLASSROOM_FINAL)
        part2_points = part2.global_points_of_scans(
            part2.load_session_scans(CLASSROOM_FINAL))
        for scan, expected in zip(scans, part2_points):
            np.testing.assert_array_equal(scan["points_global"], expected)


class SessionFileTests(Part3TestCase):
    def test_names_resolve_next_to_the_program(self):
        self.assertEqual(resolve_session_path("classroom_final.json"),
                         PROJECT_DIR / "scans" / "classroom_final.json")
        self.assertEqual(resolve_session_path("new_room"),
                         SCANS_DIR / "new_room.json")
        original = Path.cwd()
        try:
            os.chdir(self.folder.name)  # started from another folder
            self.assertEqual(resolve_session_path("a.json"),
                             SCANS_DIR / "a.json")
        finally:
            os.chdir(original)

    def test_absolute_path_and_other_names(self):
        path = write_session(self.folder.name, make_scans_json(3),
                             name="other_room.json")
        self.assertEqual(resolve_session_path(str(path)), path)
        output, figure = self.run_main(str(path))
        self.assertIn("3 scans", output)
        output, figure = self.run_main("other_room.json",
                                       scans_dir=self.folder.name)
        self.assertIn("3 scans, 60 points", output)

    def test_missing_file_gives_a_clear_message(self):
        output, figure = self.run_main("no_such_room.json")
        self.assertIn("Session file not found", output)
        self.assertIn(str(SCANS_DIR / "no_such_room.json"), output)
        self.assertIsNone(figure)

    def test_any_number_of_scans_and_both_colour_modes(self):
        module = sys.modules[__name__]
        for count in (1, 2, 9, 40):
            path = write_session(self.folder.name, make_scans_json(count))
            for color_by_scan, collections in ((True, count), (False, 1)):
                with self.subTest(scans=count, color_by_scan=color_by_scan), \
                        mock.patch.object(module, "COLOR_BY_SCAN",
                                          color_by_scan):
                    plt.close("all")
                    output, figure = self.run_main(str(path))
                    self.assertIn(f"{count} scans, {20 * count} points",
                                  output)
                    axes = figure.axes[0]
                    self.assertEqual(len(axes.collections), collections)
                    self.assertEqual(len(axes.lines), count)  # LiDAR dots
                    self.assertIn("Part 3", axes.get_title())

    def test_part_1_colours_are_used(self):
        scans_json = make_scans_json(2)
        scans_json[0]["color"] = "tab:red"
        scans, _ = load_point_cloud(write_session(self.folder.name,
                                                  scans_json))
        figure = plot_point_cloud(scans, color_by_scan=True)
        first = figure.axes[0].collections[0].get_facecolor()[0]
        np.testing.assert_allclose(first[:3], matplotlib.colors.to_rgb(
            "tab:red"))


@unittest.skipUnless(CLASSROOM_FINAL.exists(), "classroom_final.json absent")
class ClassroomTests(Part3TestCase):
    def test_default_setting_loads_the_real_classroom_read_only(self):
        self.assertEqual(resolve_session_path(SESSION_FILE), CLASSROOM_FINAL)
        before = file_sha256(CLASSROOM_FINAL)
        output, figure = self.run_main("classroom_final.json")
        self.assertIn("16 scans, 9713 points", output)
        self.assertIn("Session file unchanged: True", output)
        axes = figure.axes[0]
        self.assertEqual(axes.get_aspect(), 1.0)  # equal X/Y scale
        self.assertEqual((axes.get_xlabel(), axes.get_ylabel()),
                         ("X (m)", "Y (m)"))
        self.assertEqual(file_sha256(CLASSROOM_FINAL), before)

    def test_saved_poses_are_used_unchanged(self):
        data = json.loads(CLASSROOM_FINAL.read_text(encoding="utf-8"))
        saved = {entry["id"]: (entry["pose"]["x_m"], entry["pose"]["y_m"],
                               entry["pose"]["theta_deg"])
                 for entry in data["scans"] if entry.get("visible", True)}
        scans, _ = load_point_cloud(CLASSROOM_FINAL)
        self.assertEqual({scan["id"]: scan["pose"] for scan in scans}, saved)

    def test_program_runs_as_a_script_from_another_folder(self):
        # What PyCharm's Run does, here from an unrelated working folder
        # with an off-screen Matplotlib backend (no window opens).
        before = file_sha256(CLASSROOM_FINAL)
        result = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), "--non-interactive"],
            cwd=self.folder.name, capture_output=True, text=True,
            env=dict(os.environ, MPLBACKEND="Agg"), timeout=120)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("16 scans, 9713 points", result.stdout)
        self.assertIn("accepted clusters", result.stdout)  # default view
        self.assertEqual(file_sha256(CLASSROOM_FINAL), before)


# ---------------------------------------------------------------------------
# Milestone 2 tests: pre-processing and segmentation
# ---------------------------------------------------------------------------
def line_points(start, end, count):
    """`count` evenly spaced points from start to end (inclusive)."""
    return np.linspace(start, end, count)


def ring_scan(radius_m=2.0, step_deg=0.8, start_deg=0.0, far_arc=None,
              far_radius_m=5.0):
    """One simulated revolution: points on a circle around the LiDAR,
    clockwise like the LD06. Angles inside far_arc (deg_from, deg_to) are
    at far_radius_m instead (e.g. an open door)."""
    angles = (start_deg - np.arange(0.0, 360.0 - step_deg / 2, step_deg)) \
        % 360.0
    radii = np.full(len(angles), radius_m)
    if far_arc is not None:
        radii[(angles >= far_arc[0]) & (angles <= far_arc[1])] = far_radius_m
    radians = np.radians(angles)
    return np.column_stack([radii * np.cos(radians), radii * np.sin(radians)])


def cluster_sets(clusters, points):
    """Clusters as a set of frozensets of rounded point coordinates."""
    return {frozenset(tuple(np.round(points[index], 9))
                      for index in cluster["indices"])
            for cluster in clusters}


def segment(points, **options):
    """Pre-process and segment one scan's local points."""
    points = np.asarray(points, dtype=float)
    valid = preprocess_measurements(points) == KEPT
    return segment_scan(points, valid, **options)


class PreprocessingTests(Part3TestCase):
    def test_range_limits_and_reasons(self):
        points = np.array([[0.05, 0.0], [0.0, 0.10], [3.0, 4.0],
                           [12.0, 0.0], [12.0, 0.1], [np.nan, 1.0]])
        reasons = preprocess_measurements(points, min_range=0.10,
                                          max_range=12.0)
        self.assertEqual(list(reasons), [TOO_CLOSE, KEPT, KEPT, KEPT,
                                         TOO_FAR, INVALID])

    def test_quality_filter_only_with_real_quality_values(self):
        points = np.array([[1.0, 0.0], [2.0, 0.0], [3.0, 0.0]])
        self.assertEqual(list(preprocess_measurements(points,
                                                      min_quality=100)),
                         [KEPT] * 3)  # no quality data: nothing invented
        reasons = preprocess_measurements(points, quality=[200, 50, 150],
                                          min_quality=100)
        self.assertEqual(list(reasons), [KEPT, LOW_QUALITY, KEPT])

    def test_counts_and_nothing_changed(self):
        scans_json = [make_scan_json(1, (1.0, 2.0, 30.0),
                                     [[0.01, 0.0], [1.0, 0.0], [20.0, 0.0],
                                      ["bad", 0.0]])]
        scans, _ = load_point_cloud(write_session(self.folder.name,
                                                  scans_json))
        local_before = scans[0]["points_local"].copy()
        counts = preprocess_scans(scans)
        self.assertEqual(counts, {INVALID: 1, TOO_CLOSE: 1, TOO_FAR: 1,
                                  LOW_QUALITY: 0})
        np.testing.assert_array_equal(scans[0]["valid"],
                                      [False, True, False, False])
        np.testing.assert_array_equal(scans[0]["points_local"],
                                      local_before)
        self.assertEqual(scans[0]["pose"], (1.0, 2.0, 30.0))


class SegmentationTests(Part3TestCase):
    def test_continuous_points_form_one_cluster(self):
        clusters = segment(line_points([1.0, -1.0], [1.0, 1.0], 41))  # 5 cm
        self.assertEqual(len(clusters), 1)
        np.testing.assert_array_equal(clusters[0]["indices"], np.arange(41))
        self.assertTrue(clusters[0]["accepted"])

    def test_distance_jump_separates_groups(self):
        points = np.vstack([line_points([1, 0], [1, 0.5], 11),
                            line_points([3, 0.5], [3, 1.0], 11)])
        clusters = segment(points)
        self.assertEqual([list(c["indices"]) for c in clusters],
                         [list(range(11)), list(range(11, 22))])

    def test_threshold_is_respected_exactly(self):
        points = np.array([[1.0, 0.0], [1.0, 0.15], [1.0, 0.30],
                           [1.0, 0.4501]])
        clusters = segment(points, distance_threshold=0.15, min_points=1)
        self.assertEqual([len(c["indices"]) for c in clusters], [3, 1])

    def test_single_point_cluster_is_kept_for_diagnostics(self):
        points = np.vstack([line_points([1, 0], [1, 0.4], 9),
                            [[2.5, 2.5]],                       # isolated
                            line_points([1, 1.0], [1, 1.4], 9)])
        clusters = segment(points)
        self.assertEqual([len(c["indices"]) for c in clusters], [9, 1, 9])
        self.assertEqual([c["accepted"] for c in clusters],
                         [True, False, True])

    def test_minimum_cluster_size(self):
        points = line_points([1, 0], [1, 0.15], 4)  # 4 points, 5 cm apart
        self.assertFalse(segment(points, min_points=5)[0]["accepted"])
        self.assertTrue(segment(points, min_points=4)[0]["accepted"])

    def test_filtered_measurement_breaks_continuity(self):
        points = line_points([1, 0], [1, 1.0], 21)  # 5 cm apart
        for bad in ([np.nan, np.nan], [0.01, 0.0], [30.0, 0.0]):
            with self.subTest(replacement=bad):
                damaged = points.copy()
                damaged[10] = bad  # invalid / too close / too far
                clusters = segment(damaged)
                # Points 9 and 11 are only 10 cm apart, but are NOT joined.
                self.assertEqual([list(c["indices"]) for c in clusters],
                                 [list(range(10)), list(range(11, 21))])

    def test_circular_boundary_joins_a_wall_cut_by_the_start(self):
        # A ring 2 m away seen through a full revolution, except an open
        # door at 170-200 deg. The measurements start at 0 deg, in the
        # middle of the ring: its two pieces must be joined.
        points = ring_scan(far_arc=(170.0, 200.0))
        clusters = segment(points)
        self.assertEqual(len(clusters), 2)
        joined = [c for c in clusters if c["wraps"]]
        self.assertEqual(len(joined), 1)
        # Measurement order kept: the end of the scan, then its start.
        indices = list(joined[0]["indices"])
        boundary = indices.index(0)
        self.assertGreater(boundary, 0)
        self.assertEqual(indices[boundary - 1], len(points) - 1)
        self.assertEqual(indices[:boundary],
                         list(range(indices[0], len(points))))
        self.assertEqual(indices[boundary:], list(range(0, indices[-1] + 1)))
        # Same clusters whatever direction the revolution starts at.
        for start in (0, 37, 200, 449):
            with self.subTest(start=start):
                rolled = np.roll(points, -start, axis=0)
                self.assertEqual(cluster_sets(segment(rolled), rolled),
                                 cluster_sets(clusters, points))

    def test_circular_boundary_not_joined_when_not_genuinely_adjacent(self):
        ring = ring_scan(far_arc=(170.0, 200.0))
        first_filtered = ring.copy()
        first_filtered[0] = [np.nan, np.nan]
        partial = ring[:400]                      # only ~320 deg: gap at end
        two_revolutions = np.vstack([ring, ring])  # last is not next to first
        moved_end = ring.copy()
        moved_end[-1] *= 1.2                      # 40 cm from the first one
        for name, points in (("first measurement filtered", first_filtered),
                             ("partial revolution", partial),
                             ("two revolutions", two_revolutions),
                             ("distance too large", moved_end)):
            with self.subTest(name):
                clusters = segment(points)
                self.assertFalse(any(c["wraps"] for c in clusters))
                self.assertGreaterEqual(len(clusters), 3)

    def test_rotated_scans_give_the_same_clusters(self):
        points = ring_scan(far_arc=(60.0, 100.0))
        reference = [list(c["indices"]) for c in segment(points)]
        for angle in (35.0, 90.0, 180.0, -123.0):
            with self.subTest(angle=angle):
                turned = local_to_global(points, (0.0, 0.0, angle))
                self.assertEqual([list(c["indices"])
                                  for c in segment(turned)], reference)
        # A different saved POSE never changes the (local) segmentation.
        path = write_session(self.folder.name, [
            make_scan_json(1, (0.0, 0.0, 0.0), points.tolist()),
            make_scan_json(2, (3.0, -2.0, 140.0), points.tolist())])
        scans, _ = load_point_cloud(path)
        preprocess_scans(scans)
        clusters = segment_scans(scans)
        by_scan = [[list(c["indices"]) for c in clusters
                    if c["scan_id"] == scan_id] for scan_id in (1, 2)]
        self.assertEqual(by_scan[0], by_scan[1])

    def test_scans_are_never_joined_together(self):
        # Scan 1 ends exactly where scan 2 starts (same classroom points).
        wall = line_points([1.0, 0.0], [1.0, 1.0], 21)
        path = write_session(self.folder.name, [
            make_scan_json(1, (0.0, 0.0, 0.0), wall[:11].tolist()),
            make_scan_json(2, (0.0, 0.0, 0.0), wall[10:].tolist())])
        scans, _ = load_point_cloud(path)
        preprocess_scans(scans)
        clusters = segment_scans(scans)
        self.assertEqual([(c["scan_id"], len(c["indices"]))
                          for c in clusters], [(1, 11), (2, 11)])
        for cluster in clusters:
            scan = scans[cluster["scan_index"]]
            np.testing.assert_array_equal(
                cluster["points_global"],
                scan["points_global"][cluster["indices"]])

    def test_every_kept_point_is_in_exactly_one_cluster(self):
        rng = np.random.default_rng(5)
        points = rng.uniform(-6, 6, (300, 2))
        points[rng.choice(300, 20, replace=False)] = np.nan
        valid = preprocess_measurements(points) == KEPT
        clusters = segment_scan(points, valid, min_points=1)
        members = np.concatenate([c["indices"] for c in clusters])
        np.testing.assert_array_equal(np.sort(members),
                                      np.flatnonzero(valid))

    def test_statistics_add_up(self):
        points = np.vstack([line_points([1, 0], [1, 0.5], 11),
                            [[0.01, 0.0]], [[4.0, 4.0]],
                            line_points([1, 1.0], [1, 1.2], 3)])
        path = write_session(self.folder.name, [
            make_scan_json(1, (0.0, 0.0, 0.0), points.tolist())])
        scans, _ = load_point_cloud(path)
        counts = preprocess_scans(scans)
        stats = segmentation_statistics(scans, segment_scans(scans), counts)
        self.assertEqual((stats["clusters"], stats["accepted_clusters"],
                          stats["small_clusters"]), (3, 1, 2))
        self.assertEqual(stats["points_in_accepted"]
                         + stats["points_in_small"]
                         + stats["rejected_total"], stats["points"])


class ViewModeTests(Part3TestCase):
    def test_invalid_view_mode_is_a_clear_error(self):
        for value in ("cluster", "lines", "", None):
            with self.subTest(value=value):
                with self.assertRaises(ValueError) as context:
                    check_view_mode(value)
                self.assertIn("clusters", str(context.exception))
        for value in VALID_VIEW_MODES:
            check_view_mode(value)

    def test_clusters_view_and_rejected_points(self):
        points = np.vstack([line_points([1, 0], [1, 0.5], 11),
                            [[0.01, 0.0]], [[4.0, 4.0]],
                            line_points([1, 1.0], [1, 1.5], 11)])
        path = write_session(self.folder.name, [
            make_scan_json(1, (0.0, 0.0, 0.0), points.tolist())])
        module = sys.modules[__name__]
        for show_rejected, expected_collections in ((True, 4), (False, 2)):
            with self.subTest(show_rejected=show_rejected), \
                    mock.patch.object(module, "SHOW_REJECTED_POINTS",
                                      show_rejected):
                plt.close("all")
                output, figure = self.run_main(str(path),
                                               view_mode="clusters")
                axes = figure.axes[0]
                # 2 accepted clusters (+ small clusters + filtered points)
                self.assertEqual(len(axes.collections), expected_collections)
                self.assertEqual(len(axes.lines), 1)  # the LiDAR position
                self.assertIn("step 2: 2 clusters", axes.get_title())
                self.assertEqual(axes.get_aspect(), 1.0)
                self.assertIn("accepted clusters          2", output)

    def test_any_number_of_scans_in_clusters_view(self):
        for count in (1, 2, 16, 25):
            path = write_session(self.folder.name, [
                make_scan_json(scan_id, (scan_id * 0.5, 0.0, scan_id * 20.0),
                               ring_scan(far_arc=(60.0, 100.0)).tolist())
                for scan_id in range(1, count + 1)])
            with self.subTest(scans=count):
                plt.close("all")
                output, figure = self.run_main(str(path),
                                               view_mode="clusters")
                self.assertIn(f"{count} scans", output)
                # Each scan: the ring (joined across 0/360) + the far arc.
                self.assertIn(f"clusters detected       {2 * count:4d}",
                              output)
                self.assertEqual(len(figure.axes[0].lines), count)


@unittest.skipUnless(CLASSROOM_FINAL.exists(), "classroom_final.json absent")
class ClassroomSegmentationTests(Part3TestCase):
    def test_real_classroom_segmentation(self):
        before = file_sha256(CLASSROOM_FINAL)
        scans, _ = load_point_cloud(CLASSROOM_FINAL)
        counts = preprocess_scans(scans)
        clusters = segment_scans(scans)
        stats = segmentation_statistics(scans, clusters, counts)
        self.assertEqual(stats["points"], 9713)
        self.assertEqual(stats["points_in_accepted"]
                         + stats["points_in_small"]
                         + stats["rejected_total"], stats["points"])
        self.assertTrue(all(len(c["indices"]) >= MIN_CLUSTER_POINTS
                            for c in clusters if c["accepted"]))
        self.assertEqual(stats["single_revolution_scans"], 11)
        for cluster in clusters:  # one scan per cluster, original order
            scan = scans[cluster["scan_index"]]
            self.assertEqual(scan["id"], cluster["scan_id"])
            if not cluster["wraps"]:
                self.assertTrue(np.all(np.diff(cluster["indices"]) == 1))
        self.assertEqual(file_sha256(CLASSROOM_FINAL), before)

    def test_points_view_is_unchanged_on_the_classroom(self):
        output, figure = self.run_main("classroom_final.json",
                                       view_mode="points")
        axes = figure.axes[0]
        self.assertIn("step 1: global LiDAR point cloud (16 scans, 9713 "
                      "points)", axes.get_title())
        self.assertEqual(len(axes.collections), 16)  # one per scan
        self.assertEqual(len(axes.lines), 16)        # LiDAR positions


# ---------------------------------------------------------------------------
# Milestone 3 tests: straight-line fitting
# ---------------------------------------------------------------------------
def endpoints_match(test, fit, expected_start, expected_end, places=6):
    """The segment's endpoints, in either order, equal the expected ones."""
    got = [np.asarray(fit["start"]), np.asarray(fit["end"])]
    expected = [np.asarray(expected_start, float),
                np.asarray(expected_end, float)]
    if np.linalg.norm(got[0] - expected[0]) > np.linalg.norm(got[0]
                                                             - expected[1]):
        got.reverse()
    for point, target in zip(got, expected):
        np.testing.assert_allclose(point, target, atol=10 ** -places)


def corner_points(arm_m=0.6, count=13):
    """An L-shaped cluster: two perpendicular walls meeting at (2, 0)."""
    first = line_points([2.0 - arm_m, 0.0], [2.0, 0.0], count)
    second = line_points([2.0, 0.0], [2.0, arm_m], count)[1:]
    return np.vstack([first, second])


def arc_points(radius=0.5, degrees=120.0, count=25):
    """A curved cluster (e.g. a round pillar) of the given angular extent."""
    angles = np.radians(np.linspace(0.0, degrees, count))
    return np.column_stack([3 + radius * np.cos(angles),
                            1 + radius * np.sin(angles)])


class LineFittingTests(Part3TestCase):
    def test_horizontal_vertical_and_diagonal_lines(self):
        for start, end in (([0, 1], [2, 1]),        # horizontal
                           ([1.5, -1], [1.5, 2]),   # vertical: y = mx fails
                           ([0, 0], [2, 2]),        # diagonal
                           ([3, -1], [-1, 2])):     # other diagonal
            with self.subTest(start=start, end=end):
                fit = fit_line(line_points(start, end, 21))
                endpoints_match(self, fit, start, end)
                self.assertAlmostEqual(fit["length"],
                                       math.dist(start, end))
                self.assertLess(fit["rms_error"], 1e-9)
                self.assertEqual(validate_line(fit), "")

    def test_vertical_wall_direction_and_normal(self):
        fit = fit_line(line_points([1.5, -1.0], [1.5, 2.0], 31))
        self.assertAlmostEqual(abs(fit["direction"][1]), 1.0)  # along y
        self.assertAlmostEqual(abs(fit["normal"][0]), 1.0)     # along x
        np.testing.assert_allclose(fit["centroid"], [1.5, 0.5])

    def test_noisy_line(self):
        rng = np.random.default_rng(4)
        clean = line_points([0.0, 0.0], [3.0, 1.0], 120)
        normal = np.array([-1.0, 3.0]) / math.sqrt(10.0)
        noise = rng.normal(0.0, 0.01, 120)               # 1 cm, like LD06
        fit = fit_line(clean + noise[:, None] * normal)
        self.assertAlmostEqual(fit["rms_error"], float(np.std(noise)),
                               delta=0.002)
        self.assertLess(abs(abs(fit["direction"] @ np.array([3.0, 1.0])
                                / math.sqrt(10.0)) - 1.0), 1e-4)
        endpoints_match(self, fit, [0, 0], [3, 1], places=1)
        self.assertEqual(validate_line(fit), "")

    def test_endpoints_are_projections_onto_the_line(self):
        # Extreme points off the line: endpoints are their projections.
        points = np.array([[0.0, 0.05], [0.5, -0.05], [1.0, 0.05],
                           [1.5, -0.05], [2.0, 0.05]])
        fit = fit_line(points)
        endpoints_match(self, fit, [0.0, 0.01], [2.0, 0.01])
        for end in (fit["start"], fit["end"]):
            self.assertAlmostEqual((end - fit["centroid"]) @ fit["normal"],
                                   0.0)
        # The line is y = 0.01 (mean of the y values); the worst point,
        # y = -0.05, is 6 cm from it.
        self.assertAlmostEqual(fit["max_error"], 0.06)

    def test_endpoints_follow_the_measurement_order(self):
        points = line_points([2.0, 0.0], [0.0, 0.0], 11)  # measured right->left
        fit = fit_line(points)
        np.testing.assert_allclose(fit["start"], [2.0, 0.0], atol=1e-9)
        np.testing.assert_allclose(fit["end"], [0.0, 0.0], atol=1e-9)

    def test_degenerate_clusters(self):
        for points in ([[1.0, 1.0]], [[1.0, 1.0]] * 6, np.empty((0, 2))):
            with self.subTest(points=len(points)):
                fit = fit_line(points)
                self.assertTrue(fit["degenerate"])
                self.assertIn(validate_line(fit, min_points=1),
                              (DEGENERATE, FEW_POINTS))
        self.assertEqual(validate_line(fit_line([[1.0, 1.0]] * 6)),
                         DEGENERATE)
        self.assertEqual(validate_line(fit_line(line_points([0, 0], [1, 0],
                                                            4))),
                         FEW_POINTS)

    def test_short_segments_are_rejected(self):
        fit = fit_line(line_points([1.0, 0.0], [1.0, 0.15], 8))  # 15 cm
        self.assertEqual(validate_line(fit), TOO_SHORT)
        self.assertEqual(validate_line(fit, min_length=0.10), "")

    def test_corners_and_curves_are_not_forced_into_a_line(self):
        corner = fit_line(corner_points())
        self.assertIn(validate_line(corner), (RMS_TOO_LARGE, ERROR_TOO_LARGE))
        arc = fit_line(arc_points())
        self.assertIn(validate_line(arc), (RMS_TOO_LARGE, ERROR_TOO_LARGE))
        # A long straight wall with a short bend at its end: the average
        # error is small, but one point is far: caught by the max error.
        wall = line_points([0.0, 0.0], [3.0, 0.0], 151)  # 2 cm spacing
        bent = np.vstack([wall, [[3.05, 0.08], [3.08, 0.16]]])
        fit = fit_line(bent)
        self.assertLessEqual(fit["rms_error"], LINE_MAX_RMS_M)
        self.assertEqual(validate_line(fit), ERROR_TOO_LARGE)

    def test_same_segment_whatever_the_robot_pose(self):
        local = line_points([1.0, -0.5], [1.0, 0.7], 25)  # wall ahead
        for pose in ((0, 0, 0), (2.5, -1.0, 90.0), (-3.0, 4.0, 180.0),
                     (1.2, 1.2, -137.0), (0.0, 0.0, 33.3)):
            with self.subTest(pose=pose):
                fit = fit_line(local_to_global(local, pose))
                endpoints_match(self, fit,
                                local_to_global(np.array([[1.0, -0.5]]),
                                                pose)[0],
                                local_to_global(np.array([[1.0, 0.7]]),
                                                pose)[0])
                self.assertAlmostEqual(fit["length"], 1.2)
                self.assertEqual(validate_line(fit), "")

    def test_ordinary_regression_would_fail_on_a_vertical_wall(self):
        # Why orthogonal fitting: y = m x + b cannot describe x = 1.5.
        points = line_points([1.5, -1.0], [1.5, 2.0], 31)
        rng = np.random.default_rng(1)
        points[:, 0] += rng.normal(0, 0.01, 31)  # 1 cm noise across
        slope, _ = np.polyfit(points[:, 0], points[:, 1], 1)
        self.assertLess(abs(slope), 20)  # y = mx fit: far from vertical
        fit = fit_line(points)
        self.assertGreater(abs(fit["direction"][1]), 0.999)  # PCA: vertical

    def test_fit_segments_records_and_multiple_scans(self):
        wall = line_points([1.0, -0.5], [1.0, 0.7], 25).tolist()
        corner = (corner_points() - [1.0, 0.0]).tolist()
        path = write_session(self.folder.name, [
            make_scan_json(1, (0.0, 0.0, 0.0), wall),
            make_scan_json(2, (2.0, 1.0, 90.0), wall),
            make_scan_json(3, (0.0, 0.0, 0.0), corner)])
        scans, _ = load_point_cloud(path)
        preprocess_scans(scans)
        records = fit_segments(segment_scans(scans))
        self.assertEqual([(r["cluster"]["scan_id"], r["accepted"])
                          for r in records],
                         [(1, True), (2, True), (3, False)])
        x1, y1, x2, y2 = records[1]["endpoints"]
        np.testing.assert_allclose([x1, y1, x2, y2],
                                   [2.5, 2.0, 1.3, 2.0], atol=1e-9)
        self.assertIsNone(records[2]["endpoints"])
        stats = line_fitting_statistics(records)
        self.assertEqual((stats["accepted_segments"],
                          stats["rejected_segments"]), (2, 1))
        self.assertAlmostEqual(stats["total_length_m"], 2.4)

    def test_only_accepted_clusters_are_fitted(self):
        small = {"accepted": False, "points_global": np.zeros((3, 2))}
        self.assertEqual(fit_segments([small]), [])


class SegmentsViewTests(Part3TestCase):
    def test_segments_view_draws_lines_endpoints_and_poses(self):
        wall = line_points([1.0, -0.5], [1.0, 0.7], 25).tolist()
        path = write_session(self.folder.name, [
            make_scan_json(scan_id, (scan_id * 0.4, 0.0, scan_id * 30.0),
                           wall) for scan_id in range(1, 4)])
        output, figure = self.run_main(str(path), view_mode="segments")
        axes = figure.axes[0]
        lines = [c for c in axes.collections
                 if isinstance(c, LineCollection)]
        self.assertEqual(len(lines), 1)
        self.assertEqual(len(lines[0].get_segments()), 3)
        self.assertEqual(len(axes.lines), 3)  # LiDAR positions
        self.assertIn("step 4: 3 segments (3 original + 0 recovered) from 3 "
                      "clusters, 0 without a segment", axes.get_title())
        self.assertEqual(axes.get_aspect(), 1.0)
        self.assertEqual(axes.get_xlabel(), "X (m)")
        self.assertIn("accepted segments          3", output)

    def test_points_and_rejected_layers_are_optional(self):
        wall = line_points([1.0, -0.5], [1.0, 0.7], 25)
        points = np.vstack([wall, [[4.0, 4.0]], corner_points() + [3, 3]])
        path = write_session(self.folder.name, [
            make_scan_json(1, (0.0, 0.0, 0.0), points.tolist())])
        scans, _ = load_point_cloud(path)
        preprocess_scans(scans)
        # Without splitting, the corner stays unused (milestone 3 layout).
        records = fit_segments(segment_scans(scans), split=False)
        for show_points, show_rejected, scatters in (
                (True, True, 3), (False, True, 2), (True, False, 2),
                (False, False, 1)):
            with self.subTest(show_points=show_points,
                              show_rejected=show_rejected):
                figure = plot_segments(scans, records, show_points,
                                       show_rejected)
                axes = figure.axes[0]
                kinds = [type(c).__name__ for c in axes.collections]
                self.assertEqual(kinds.count("PathCollection"), scatters)
                self.assertEqual(kinds.count("LineCollection"), 1)
                plt.close(figure)

    def test_all_three_views_still_work(self):
        path = write_session(self.folder.name, [
            make_scan_json(1, (0.0, 0.0, 0.0),
                           ring_scan(far_arc=(60.0, 100.0)).tolist())])
        for mode, title in (("points", "step 1"), ("clusters", "step 2"),
                            ("segments", "step 4")):
            with self.subTest(mode=mode):
                plt.close("all")
                output, figure = self.run_main(str(path), view_mode=mode)
                self.assertIn(title, figure.axes[0].get_title())


@unittest.skipUnless(CLASSROOM_FINAL.exists(), "classroom_final.json absent")
class ClassroomLineTests(Part3TestCase):
    def test_real_classroom_segments(self):
        before = file_sha256(CLASSROOM_FINAL)
        scans, _ = load_point_cloud(CLASSROOM_FINAL)
        preprocess_scans(scans)
        clusters = segment_scans(scans)
        records = fit_segments(clusters)
        self.assertEqual(len(records), sum(c["accepted"] for c in clusters))
        stats = line_fitting_statistics(records)
        self.assertEqual(stats["accepted_segments"]
                         + stats["rejected_segments"], 457)
        self.assertGreater(stats["accepted_segments"], 100)
        self.assertEqual(sum(stats["rejected_by_reason"].values()),
                         stats["rejected_segments"])
        for record in records:
            fit = record["fit"]
            if record["accepted"]:
                self.assertGreaterEqual(fit["length"], LINE_MIN_LENGTH_M)
                self.assertLessEqual(fit["rms_error"], LINE_MAX_RMS_M)
                self.assertLessEqual(fit["max_error"], LINE_MAX_ERROR_M)
                self.assertEqual(len(record["endpoints"]), 4)
                self.assertTrue(np.all(np.isfinite(record["endpoints"])))
        self.assertEqual(file_sha256(CLASSROOM_FINAL), before)

    def test_script_prints_the_segment_summary(self):
        result = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), "--non-interactive"],
            cwd=self.folder.name, capture_output=True, text=True,
            env=dict(os.environ, MPLBACKEND="Agg"), timeout=120)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("view 'layers'", result.stdout)  # default view
        self.assertIn("accepted segments", result.stdout)
        self.assertIn("recovered segments", result.stdout)


# ---------------------------------------------------------------------------
# Milestone 4 tests: Douglas-Peucker corner splitting
# ---------------------------------------------------------------------------
# The 176 whole-cluster segments of the classroom before splitting existed
# (SHA-256 of their endpoints array): splitting must never change them.
MILESTONE_3_SEGMENTS_SHA256 = (
    "9db6cef1ff06fedc37b8c5d1f8850c82871f748dfc649b9f41bc4066ddda217c")


def polyline_points(corners, spacing=0.05):
    """Points every `spacing` m along a polyline through `corners`."""
    pieces = []
    for start, end in zip(corners[:-1], corners[1:]):
        count = max(2, int(round(math.dist(start, end) / spacing)) + 1)
        pieces.append(line_points(start, end, count)[:-1])
    return np.vstack(pieces + [np.array([corners[-1]], dtype=float)])


def make_cluster(points, scan_index=0, scan_id=1, first_index=0):
    """A cluster dictionary like segment_scans() makes."""
    points = np.asarray(points, dtype=float)
    return {"indices": np.arange(first_index, first_index + len(points)),
            "accepted": True, "wraps": False, "scan_index": scan_index,
            "scan_id": scan_id, "points_global": points}


def recovered_pieces(record):
    return [piece for piece in record["pieces"] if piece["accepted"]]


class DouglasPeuckerTests(Part3TestCase):
    def test_perpendicular_distance_formula(self):
        distances = perpendicular_distances(
            [[1.0, 1.0], [2.0, -3.0], [5.0, 0.0], [-1.0, 2.0]],
            [0.0, 0.0], [4.0, 0.0])  # the x axis
        np.testing.assert_allclose(distances, [1.0, 3.0, 0.0, 2.0])
        diagonal = perpendicular_distances([[0.0, 2.0]], [0, 0], [1, 1])
        self.assertAlmostEqual(float(diagonal[0]), math.sqrt(2.0))
        # Zero-length baseline: distance to the point itself.
        same = perpendicular_distances([[3.0, 4.0]], [0, 0], [0, 0])
        self.assertAlmostEqual(float(same[0]), 5.0)

    def test_straight_wall_is_one_unchanged_segment(self):
        points = line_points([0.0, 0.0], [3.0, 0.0], 61)
        self.assertEqual(douglas_peucker_pieces(points), [(0, 60)])
        with_split = fit_segments([make_cluster(points)])[0]
        without = fit_segments([make_cluster(points)], split=False)[0]
        self.assertTrue(with_split["accepted"])
        self.assertEqual(with_split["pieces"], [])  # never examined
        self.assertEqual(with_split["endpoints"], without["endpoints"])

    def test_right_angle_corner_gives_two_segments(self):
        points = polyline_points([[0.0, 0.0], [2.0, 0.0], [2.0, 1.5]])
        corner = int(np.argmin(np.hypot(points[:, 0] - 2.0, points[:, 1])))
        self.assertEqual(douglas_peucker_pieces(points),
                         [(0, corner), (corner, len(points) - 1)])
        record = fit_segments([make_cluster(points)])[0]
        self.assertFalse(record["accepted"])  # not one straight line ...
        pieces = recovered_pieces(record)    # ... but two straight walls
        self.assertEqual(len(pieces), 2)
        endpoints_match(self, pieces[0]["fit"], [0, 0], [2, 0])
        endpoints_match(self, pieces[1]["fit"], [2, 0], [2, 1.5])
        for piece in pieces:
            self.assertLess(piece["fit"]["rms_error"], 1e-9)

    def test_several_corners(self):
        u_shape = polyline_points([[0, 1.2], [0, 0], [2, 0], [2, 1.2]])
        zigzag = polyline_points([[0, 0], [1, 0.8], [2, 0], [3, 0.8],
                                  [4, 0]])
        for points, expected in ((u_shape, 3), (zigzag, 4)):
            with self.subTest(segments=expected):
                record = fit_segments([make_cluster(points)])[0]
                self.assertEqual(len(recovered_pieces(record)), expected)

    def test_noisy_straight_wall_is_not_split(self):
        rng = np.random.default_rng(11)
        points = line_points([0.0, 0.0], [4.0, 0.0], 160)
        points[:, 1] += rng.normal(0.0, 0.01, 160)  # 1 cm, like the LD06
        self.assertEqual(len(douglas_peucker_pieces(points)), 1)
        record = fit_segments([make_cluster(points)])[0]
        self.assertTrue(record["accepted"])
        self.assertEqual(record["pieces"], [])

    def test_degenerate_and_repeated_points(self):
        self.assertEqual(douglas_peucker_pieces(np.empty((0, 2))), [])
        self.assertEqual(douglas_peucker_pieces([[1.0, 1.0]]), [(0, 0)])
        self.assertEqual(douglas_peucker_pieces([[0, 0], [1, 0]]), [(0, 1)])
        self.assertEqual(douglas_peucker_pieces([[2.0, 2.0]] * 10), [(0, 9)])
        # A closed loop: first point == last point (zero-length chord).
        square = polyline_points([[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]])
        pieces = douglas_peucker_pieces(square)
        self.assertEqual(len(pieces), 4)
        record = fit_segments([make_cluster(square)])[0]
        self.assertEqual(len(recovered_pieces(record)), 4)
        # Repeated measurements of the same point inside a wall.
        doubled = np.repeat(polyline_points([[0, 0], [2, 0], [2, 1.5]]), 2,
                            axis=0)
        self.assertEqual(len(recovered_pieces(
            fit_segments([make_cluster(doubled)])[0])), 2)

    def test_tiny_branches_are_rejected(self):
        wall = line_points([0.0, 0.0], [3.0, 0.0], 61)
        hook = np.array([[3.03, 0.06], [3.06, 0.12], [3.09, 0.18]])
        record = fit_segments([make_cluster(np.vstack([wall, hook]))])[0]
        self.assertIn(record["reason"], (RMS_TOO_LARGE, ERROR_TOO_LARGE))
        pieces = record["pieces"]
        self.assertEqual([piece["accepted"] for piece in pieces],
                         [True, False])
        self.assertEqual(pieces[1]["reason"], FEW_POINTS)  # 4 < 8 points
        endpoints_match(self, pieces[0]["fit"], [0, 0], [3, 0])

    def test_short_furniture_pieces_are_not_kept(self):
        # A small zigzag (chair base): every piece is too short or too small.
        points = polyline_points([[0, 0], [0.12, 0.1], [0.24, 0],
                                  [0.36, 0.1]], spacing=0.03)
        record = fit_segments([make_cluster(points)])[0]
        self.assertEqual(recovered_pieces(record), [])

    def test_indices_are_preserved_in_order(self):
        points = polyline_points([[0, 0], [2, 0], [2, 1.5]])
        cluster = make_cluster(points, first_index=100)
        cluster["indices"] = np.concatenate([np.arange(400, 420),
                                             np.arange(0, len(points) - 20)])
        pieces = split_cluster(cluster)
        joined = np.concatenate([piece["indices"] for piece in pieces])
        shared = pieces[0]["indices"][-1]
        self.assertEqual(shared, pieces[1]["indices"][0])  # the corner
        np.testing.assert_array_equal(
            np.concatenate([pieces[0]["indices"], pieces[1]["indices"][1:]]),
            cluster["indices"])
        for piece in pieces:
            position = int(np.flatnonzero(cluster["indices"]
                                          == piece["indices"][0])[0])
            np.testing.assert_array_equal(
                piece["points_global"],
                cluster["points_global"][position:position
                                         + len(piece["indices"])])
        self.assertEqual(len(joined), len(points) + 1)

    def test_no_splitting_across_clusters_or_scans(self):
        # Two straight halves of an L in DIFFERENT scans stay separate
        # original segments; nothing is joined or split across them.
        points = polyline_points([[0, 0], [2, 0], [2, 1.5]])
        corner = int(np.argmin(np.hypot(points[:, 0] - 2.0, points[:, 1])))
        clusters = [make_cluster(points[:corner], 0, 1),
                    make_cluster(points[corner:], 1, 2)]
        records = fit_segments(clusters)
        self.assertEqual([record["accepted"] for record in records],
                         [True, True])
        self.assertEqual([record["pieces"] for record in records], [[], []])
        bent = fit_segments([make_cluster(points, 3, 7)])[0]
        for piece in bent["pieces"]:
            self.assertEqual((piece["cluster"]["scan_index"],
                              piece["cluster"]["scan_id"]), (3, 7))

    def test_depth_limit_and_long_inputs_are_safe(self):
        zigzag = polyline_points([[x, 0.5 * (x % 2)] for x in range(200)])
        pieces = douglas_peucker_pieces(zigzag, max_depth=2)
        self.assertLessEqual(len(pieces), 4)
        pieces = douglas_peucker_pieces(zigzag)  # default depth: no crash
        self.assertLessEqual(len(pieces), 2 ** DP_MAX_DEPTH)
        self.assertEqual(pieces[0][0], 0)
        self.assertEqual(pieces[-1][1], len(zigzag) - 1)
        for (_, last), (first, _) in zip(pieces[:-1], pieces[1:]):
            self.assertEqual(last, first)  # contiguous, nothing lost

    def test_disabled_splitting_is_milestone_3(self):
        points = polyline_points([[0, 0], [2, 0], [2, 1.5]])
        record = fit_segments([make_cluster(points)], split=False)[0]
        self.assertFalse(record["accepted"])
        self.assertEqual(record["pieces"], [])

    def test_recovered_segments_are_drawn_distinctly(self):
        wall = line_points([1.0, -0.5], [1.0, 0.7], 25)
        bent = polyline_points([[3, 0], [5, 0], [5, 1.5]])
        points = np.vstack([wall, bent])
        path = write_session(self.folder.name, [
            make_scan_json(1, (0.0, 0.0, 0.0), points.tolist(),
                           color="tab:red")])
        scans, _ = load_point_cloud(path)
        preprocess_scans(scans)
        records = fit_segments(segment_scans(scans))
        for distinguish, expected in ((True, (0, 0, 0)),
                                      (False, matplotlib.colors.to_rgb(
                                          "tab:red"))):
            with self.subTest(distinguish=distinguish):
                figure = plot_segments(scans, records,
                                       distinguish_recovered=distinguish)
                lines = [c for c in figure.axes[0].collections
                         if isinstance(c, LineCollection)]
                self.assertEqual([len(c.get_segments()) for c in lines],
                                 [1, 2])  # 1 original, 2 recovered
                np.testing.assert_allclose(lines[1].get_colors()[0][:3],
                                           expected)
                self.assertIn("1 original + 2 recovered",
                              figure.axes[0].get_title())
                plt.close(figure)


@unittest.skipUnless(CLASSROOM_FINAL.exists(), "classroom_final.json absent")
class ClassroomSplittingTests(Part3TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fingerprint = file_sha256(CLASSROOM_FINAL)
        cls.scans, _ = load_point_cloud(CLASSROOM_FINAL)
        preprocess_scans(cls.scans)
        cls.clusters = segment_scans(cls.scans)
        cls.records = fit_segments(cls.clusters)

    def test_original_176_segments_are_identical(self):
        originals = np.array([record["endpoints"] for record in self.records
                              if record["accepted"]])
        self.assertEqual(len(originals), 176)
        self.assertEqual(hashlib.sha256(originals.tobytes()).hexdigest(),
                         MILESTONE_3_SEGMENTS_SHA256)
        without = fit_segments(self.clusters, split=False)
        self.assertEqual([r["endpoints"] for r in without if r["accepted"]],
                         [r["endpoints"] for r in self.records
                          if r["accepted"]])

    def test_long_corner_clusters_are_recovered(self):
        long_bent = [record for record in self.records
                     if record["reason"] in (RMS_TOO_LARGE, ERROR_TOO_LARGE)
                     and record["fit"]["length"] >= 1.0]
        self.assertEqual(len(long_bent), 19)
        rescued = [record for record in long_bent
                   if recovered_pieces(record)]
        self.assertEqual(len(rescued), 19)
        recovered_length = sum(piece["fit"]["length"]
                               for record in long_bent
                               for piece in recovered_pieces(record))
        self.assertGreater(recovered_length, 30.0)

    def test_every_recovered_segment_meets_the_thresholds(self):
        stats = line_fitting_statistics(self.records)
        self.assertEqual(stats["dp_examined"], 119)
        self.assertGreater(stats["dp_recovered_segments"], 0)
        for record in self.records:
            for piece in recovered_pieces(record):
                fit = piece["fit"]
                self.assertGreaterEqual(fit["points"],
                                        max(LINE_MIN_POINTS, DP_MIN_POINTS))
                self.assertGreaterEqual(fit["length"], LINE_MIN_LENGTH_M)
                self.assertLessEqual(fit["rms_error"], LINE_MAX_RMS_M)
                self.assertLessEqual(fit["max_error"], LINE_MAX_ERROR_M)
                self.assertEqual(piece["cluster"]["scan_id"],
                                 record["cluster"]["scan_id"])
                self.assertTrue(np.isin(piece["cluster"]["indices"],
                                        record["cluster"]["indices"]).all())
        self.assertEqual(stats["segments_after_splitting"],
                         176 + stats["dp_recovered_segments"])

    def test_source_json_unchanged(self):
        self.assertEqual(file_sha256(CLASSROOM_FINAL), self.fingerprint)


# ---------------------------------------------------------------------------
# Milestone 5 tests: cross-scan segment merging
# ---------------------------------------------------------------------------
# The 258 unmerged segments (originals + recovered) before merging existed.
MILESTONE_4_SEGMENTS_SHA256 = (
    "a2898a7c88cb30ae6a4fb81d307f94d8b4ab52c4b56780601ca83b8af51eb135")


def global_to_local(points, pose):
    """Inverse of local_to_global (to build test scans): R(-theta)(p - T)."""
    x_r, y_r, theta_deg = pose
    rotation = np.array([[math.cos(math.radians(theta_deg)),
                          -math.sin(math.radians(theta_deg))],
                         [math.sin(math.radians(theta_deg)),
                          math.cos(math.radians(theta_deg))]])
    return (np.asarray(points, dtype=float) - [x_r, y_r]) @ rotation


def wall_segment(start, end, scan_id=1, spacing=0.03, noise=0.0, seed=0,
                 reverse=False):
    """An accepted segment record of a straight wall seen by one scan."""
    points = polyline_points([start, end], spacing)
    if noise:
        points = points + np.random.default_rng(seed).normal(0, noise,
                                                             points.shape)
    if reverse:
        points = points[::-1]
    record = make_record(make_cluster(points, scan_index=scan_id - 1,
                                      scan_id=scan_id), {}, "original")
    assert record["accepted"], record["reason"]
    return record


def merged_summary(merged):
    """(rounded start, rounded end, scan ids) of each merged segment."""
    return sorted((tuple(np.round(segment["start"], 2)),
                   tuple(np.round(segment["end"], 2)),
                   tuple(segment["scan_ids"])) for segment in merged)


class MergingTests(Part3TestCase):
    def test_identical_overlapping_segments_merge(self):
        merged, items = merge_segments([wall_segment([0, 0], [2, 0], 1),
                                        wall_segment([0, 0], [2, 0], 2)])
        self.assertEqual(len(merged), 1)
        self.assertEqual(merged[0]["scan_ids"], [1, 2])
        self.assertEqual(merged[0]["member_ids"], ["S001", "S002"])
        np.testing.assert_allclose(merged[0]["start"], [0, 0], atol=1e-9)
        np.testing.assert_allclose(merged[0]["end"], [2, 0], atol=1e-9)

    def test_partial_overlap_gives_the_combined_extent(self):
        merged, _ = merge_segments([wall_segment([0, 1], [2, 1], 1),
                                    wall_segment([1.2, 1], [3.5, 1], 2)])
        self.assertEqual(merged_summary(merged),
                         [((0.0, 1.0), (3.5, 1.0), (1, 2))])

    def test_opposite_directions_are_the_same_line(self):
        self.assertAlmostEqual(line_angle_deg(np.array([1.0, 0.0]),
                                              np.array([-1.0, 0.0])), 0.0)
        merged, _ = merge_segments([
            wall_segment([0, 0], [0, 2], 1),
            wall_segment([0, 0.5], [0, 2.5], 2, reverse=True)])
        self.assertEqual(merged_summary(merged),
                         [((0.0, 0.0), (0.0, 2.5), (1, 2))])

    def test_many_slightly_misaligned_scans_merge(self):
        rng = np.random.default_rng(2)
        segments = []
        for scan_id in range(1, 7):  # +-3 cm, +-1 deg, noise 1 cm
            shift = rng.uniform(-0.03, 0.03)
            tilt = math.radians(rng.uniform(-1, 1))
            start = [0.2 * scan_id, 3 + shift]
            end = [3 + 0.2 * scan_id, 3 + shift + 3 * math.tan(tilt)]
            segments.append(wall_segment(start, end, scan_id, noise=0.01,
                                         seed=scan_id))
        merged, _ = merge_segments(segments)
        self.assertEqual(len(merged), 1)
        self.assertEqual(merged[0]["support"], 6)
        self.assertLessEqual(merged[0]["rms_error"], MERGE_MAX_RMS_M)

    def test_small_gaps_join_large_gaps_do_not(self):
        for gap, expected in ((0.0, 1), (0.10, 1), (0.14, 1), (0.25, 2),
                              (0.8, 2)):
            with self.subTest(gap=gap):
                merged, _ = merge_segments([
                    wall_segment([0, 0], [2, 0], 1),
                    wall_segment([2 + gap, 0], [4, 0], 2)])
                self.assertEqual(len(merged), expected)

    def test_parallel_walls_stay_separate(self):
        for distance in (0.12, 0.3, 0.6):  # e.g. a wall and a bench
            with self.subTest(distance=distance):
                merged, _ = merge_segments([
                    wall_segment([0, 0], [3, 0], 1),
                    wall_segment([0, distance], [3, distance], 2)])
                self.assertEqual(len(merged), 2)

    def test_perpendicular_and_angled_walls_stay_separate(self):
        for angle in (90.0, 45.0, 20.0, 10.0):
            with self.subTest(angle=angle):
                rad = math.radians(angle)
                merged, _ = merge_segments([
                    wall_segment([0, 0], [2, 0], 1),
                    wall_segment([2, 0], [2 + 1.5 * math.cos(rad),
                                          1.5 * math.sin(rad)], 2)])
                self.assertEqual(len(merged), 2)

    def test_doorway_edges_are_not_joined(self):
        # A wall with a 0.9 m door, both sides seen by two scans.
        segments = [wall_segment([0, 5], [2, 5], 1),
                    wall_segment([0.1, 5], [1.95, 5], 2),
                    wall_segment([2.9, 5], [5, 5], 1),
                    wall_segment([2.95, 5], [4.8, 5], 2)]
        merged, _ = merge_segments(segments)
        self.assertEqual(merged_summary(merged),
                         [((0.0, 5.0), (2.0, 5.0), (1, 2)),
                          ((2.9, 5.0), (5.0, 5.0), (1, 2))])

    def test_unrelated_furniture_is_not_merged(self):
        segments = [wall_segment([0, 0], [4, 0], 1),
                    wall_segment([0.5, 0.4], [1.3, 0.4], 2),   # desk edge
                    wall_segment([2.0, 0.6], [2.0, 1.2], 2),   # desk side
                    wall_segment([3.0, 0.42], [3.6, 0.42], 3)]  # other desk
        merged, _ = merge_segments(segments)
        self.assertEqual(len(merged), 4)

    def test_chains_cannot_join_different_surfaces(self):
        # Each neighbour pair is compatible (8 cm apart), but the three
        # together are not one surface: at most two of them merge.
        segments = [wall_segment([0, 0.00], [3, 0.00], 1),
                    wall_segment([0, 0.08], [3, 0.08], 2),
                    wall_segment([0, 0.16], [3, 0.16], 3)]
        self.assertIsNotNone(pair_compatibility(
            *describe_segments(segments[:2])))
        merged, _ = merge_segments(segments)
        self.assertEqual(len(merged), 2)
        self.assertTrue(all(segment["rms_error"] <= MERGE_MAX_RMS_M
                            for segment in merged))

    def test_cross_scan_merging_through_the_whole_pipeline(self):
        # One wall x = 3 seen by two LiDARs at different poses.
        wall = polyline_points([[3, -1], [3, 2]], 0.03)
        poses = [(0.0, 0.0, 0.0), (1.0, 1.5, 90.0)]
        scans_json = [make_scan_json(index + 1, pose, global_to_local(
            wall, pose).round(6).tolist()) for index, pose in enumerate(poses)]
        scans, _ = load_point_cloud(write_session(self.folder.name,
                                                  scans_json))
        preprocess_scans(scans)
        segments = accepted_segments(fit_segments(segment_scans(scans)))
        self.assertEqual(len(segments), 2)
        merged, _ = merge_segments(segments)
        self.assertEqual(merged_summary(merged),
                         [((3.0, -1.0), (3.0, 2.0), (1, 2))])

    def test_input_order_does_not_matter(self):
        rng = np.random.default_rng(9)
        segments = []
        for scan_id in range(1, 5):
            for start, end in (([0, 0], [3, 0]), ([3, 0], [3, 2]),
                               ([0, 2], [1.4, 2]), ([1.6, 2], [3, 2])):
                offset = rng.uniform(-0.02, 0.02, 2)
                segments.append(wall_segment(np.add(start, offset),
                                             np.add(end, offset), scan_id))
        reference, _ = merge_segments(segments)
        for seed in range(5):
            shuffled = list(segments)
            random.Random(seed).shuffle(shuffled)
            merged, _ = merge_segments(shuffled)
            self.assertEqual([s["member_ids"] for s in merged],
                             [s["member_ids"] for s in reference])
            for a, b in zip(merged, reference):
                np.testing.assert_array_equal(a["start"], b["start"])
                np.testing.assert_array_equal(a["end"], b["end"])

    def test_every_segment_is_kept_once_with_its_scan(self):
        segments = [wall_segment([0, 0], [2, 0], 1),
                    wall_segment([0.5, 0], [2.5, 0], 3),
                    wall_segment([0, 1], [0, 3], 2)]
        merged, items = merge_segments(segments)
        member_ids = sorted(member for segment in merged
                            for member in segment["member_ids"])
        self.assertEqual(member_ids, sorted(item["id"] for item in items))
        scan_of = {item["id"]: item["scan_id"] for item in items}
        for segment in merged:
            self.assertEqual(segment["scan_ids"], sorted(
                {scan_of[member] for member in segment["member_ids"]}))
            self.assertEqual(segment["support"], len(segment["scan_ids"]))

    def test_input_segments_are_not_modified(self):
        segments = [wall_segment([0, 0], [2, 0], 1),
                    wall_segment([1, 0.02], [3, 0.02], 2)]
        before = [(segment["endpoints"], segment["fit"]["start"].copy(),
                   segment["cluster"]["points_global"].copy())
                  for segment in segments]
        merge_segments(segments)
        for segment, (endpoints, start, points) in zip(segments, before):
            self.assertEqual(segment["endpoints"], endpoints)
            np.testing.assert_array_equal(segment["fit"]["start"], start)
            np.testing.assert_array_equal(
                segment["cluster"]["points_global"], points)

    def test_merged_view_and_all_four_views(self):
        wall = polyline_points([[3, -1], [3, 2]], 0.03)
        path = write_session(self.folder.name, [
            make_scan_json(scan_id, pose, global_to_local(wall, pose)
                           .round(6).tolist())
            for scan_id, pose in ((1, (0, 0, 0)), (2, (1, 1, 90)))])
        for mode, title in (("points", "step 1"), ("clusters", "step 2"),
                            ("segments", "step 4"), ("merged", "step 5")):
            with self.subTest(mode=mode):
                plt.close("all")
                output, figure = self.run_main(str(path), view_mode=mode)
                self.assertIn(title, figure.axes[0].get_title())
        axes = figure.axes[0]  # the merged view
        self.assertIn("2 segments merged into 1", axes.get_title())
        lines = [c for c in axes.collections if isinstance(c, LineCollection)]
        self.assertEqual([len(c.get_segments()) for c in lines], [2, 1])
        self.assertEqual(len(axes.lines), 2)  # LiDAR positions
        self.assertEqual(axes.get_aspect(), 1.0)
        self.assertEqual(len(figure.axes), 2)  # map + support colour bar
        scans, _ = load_point_cloud(path)
        preprocess_scans(scans)
        merged, items = merge_segments(accepted_segments(fit_segments(
            segment_scans(scans))))
        plain = plot_merged(scans, merged, items, show_unmerged=False,
                            color_by_support=False)
        self.assertEqual(len([c for c in plain.axes[0].collections
                              if isinstance(c, LineCollection)]), 1)
        self.assertEqual(len(plain.axes), 1)  # no colour bar


@unittest.skipUnless(CLASSROOM_FINAL.exists(), "classroom_final.json absent")
class ClassroomMergingTests(Part3TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fingerprint = file_sha256(CLASSROOM_FINAL)
        scans, _ = load_point_cloud(CLASSROOM_FINAL)
        preprocess_scans(scans)
        cls.segments = accepted_segments(fit_segments(segment_scans(scans)))
        cls.merged, cls.items = merge_segments(cls.segments)

    def test_unmerged_segments_are_unchanged(self):
        ends = np.array([segment["endpoints"] for segment in self.segments])
        self.assertEqual(len(ends), 258)
        self.assertEqual(hashlib.sha256(ends.tobytes()).hexdigest(),
                         MILESTONE_4_SEGMENTS_SHA256)

    def test_classroom_merge(self):
        stats = merge_statistics(self.merged, self.items)
        self.assertEqual(stats["original_segments"], 258)
        self.assertLess(stats["merged_segments"], 258)
        self.assertEqual(stats["consolidated"],
                         258 - stats["merged_segments"])
        self.assertEqual(sum(stats["support_distribution"].values()),
                         stats["merged_segments"])
        members = sorted(member for segment in self.merged
                         for member in segment["member_ids"])
        self.assertEqual(members, sorted(item["id"] for item in self.items))
        by_id = {item["id"]: item for item in self.items}
        for segment in self.merged:  # every merge is one surface
            if len(segment["member_ids"]) == 1:
                continue
            self.assertLessEqual(segment["rms_error"], MERGE_MAX_RMS_M)
            for member in segment["member_ids"]:
                item = by_id[member]
                self.assertLessEqual(line_angle_deg(item["direction"],
                                                    segment["direction"]),
                                     MERGE_MAX_ANGLE_DEG)

    def test_classroom_merge_is_order_independent(self):
        shuffled = list(self.segments)
        random.Random(1).shuffle(shuffled)
        merged, _ = merge_segments(shuffled)
        self.assertEqual([s["member_ids"] for s in merged],
                         [s["member_ids"] for s in self.merged])

    def test_source_json_unchanged(self):
        self.assertEqual(file_sha256(CLASSROOM_FINAL), self.fingerprint)


# ---------------------------------------------------------------------------
# Milestone 6 tests: interactive layer viewer, traceability, generality
# ---------------------------------------------------------------------------
# The 74 merged classroom segments before the viewer existed.
MILESTONE_5_MERGED_ENDPOINTS_SHA256 = (
    "cd3aa6934cff4b1937cf71ada749f534247c8750f0b61d1dac91ba92b921bb3d")
MILESTONE_5_MERGED_MEMBERS_SHA256 = (
    "e4eb49735d77544ab9c13a098db7b275df1778b58c977d8e1ec820eec9225c6e")


def run_pipeline(path):
    """Load and process a session exactly like main(); return everything."""
    scans, _ = load_point_cloud(path)
    preprocess_scans(scans)
    clusters = segment_scans(scans)
    records = fit_segments(clusters)
    merged, items = merge_segments(accepted_segments(records))
    return scans, clusters, records, merged, items


def visibility(viewer):
    """{layer: [visible flag of each of its artists]}."""
    return {key: [artist.get_visible() for artist in artists]
            for key, artists in viewer["layers"].items()}


def click_button(button):
    """What a mouse click on a Matplotlib Button does."""
    button._observers.process("clicked", None)


class LayerViewerTests(Part3TestCase):
    """A small 2-scan room where every layer has something to draw."""

    def setUp(self):
        super().setUp()
        room = [polyline_points([[0, 0], [3, 0]], 0.03),          # straight
                polyline_points([[0, 1], [0, 3], [2, 3]], 0.03)]   # corner
        scans_json = []
        for scan_id, pose in ((1, (1.5, 1.5, 0.0)), (2, (1.0, 1.2, 35.0))):
            local = [global_to_local(piece, pose) for piece in room]
            points = np.vstack([local[0], [[50.0, 50.0]], local[1]])
            scans_json.append(make_scan_json(scan_id, pose,
                                             points.round(6).tolist()))
        self.path = write_session(self.folder.name, scans_json)
        scans, clusters, records, merged, _ = run_pipeline(self.path)
        self.viewer = build_layer_viewer(scans, clusters, records, merged)

    def test_every_layer_has_artists_and_the_defaults_are_right(self):
        for key in LAYER_KEYS:
            self.assertTrue(self.viewer["layers"][key], key)
        self.assertEqual(self.viewer["visible_layers"](),
                         [key for key in LAYER_KEYS
                          if key in LAYERS_VISIBLE_AT_START])
        self.assertEqual(set(LAYERS_VISIBLE_AT_START),
                         {"points", "merged", "lidar"})
        for key, flags in visibility(self.viewer).items():
            self.assertEqual(set(flags), {key in LAYERS_VISIBLE_AT_START})

    def test_each_layer_toggles_independently(self):
        for index, key in enumerate(LAYER_KEYS):
            with self.subTest(layer=key):
                before = visibility(self.viewer)
                self.viewer["checkboxes"].set_active(index)  # = a click
                after = visibility(self.viewer)
                for other in LAYER_KEYS:
                    if other == key:
                        self.assertEqual(after[other],
                                         [not flag for flag in before[key]])
                    else:
                        self.assertEqual(after[other], before[other])
                self.viewer["checkboxes"].set_active(index)  # back
                self.assertEqual(visibility(self.viewer), before)

    def test_several_layers_visible_together(self):
        for key in ("clusters", "pca", "recovered", "endpoints"):
            self.viewer["set_layer"](key, True)
        self.assertEqual(self.viewer["visible_layers"](), list(LAYER_KEYS))
        self.viewer["set_layer"]("merged", False)
        self.assertNotIn("merged", self.viewer["visible_layers"]())
        self.assertTrue(all(flag for flag
                            in visibility(self.viewer)["recovered"]))

    def test_show_all_and_hide_all_buttons(self):
        show_button, hide_button, _ = self.viewer["buttons"]
        click_button(hide_button)
        self.assertEqual(self.viewer["visible_layers"](), [])
        self.assertFalse(any(flag for flags in visibility(self.viewer)
                             .values() for flag in flags))
        click_button(show_button)
        self.assertEqual(self.viewer["visible_layers"](), list(LAYER_KEYS))
        self.assertTrue(all(flag for flags in visibility(self.viewer)
                            .values() for flag in flags))

    def test_zoom_is_kept_and_reset_view_restores_it(self):
        axes = self.viewer["axes"]
        axes.set_xlim(0.5, 1.5)  # the user zooms in
        axes.set_ylim(2.5, 3.2)
        for index in range(len(LAYER_KEYS)):
            self.viewer["checkboxes"].set_active(index)
        click_button(self.viewer["buttons"][0])
        click_button(self.viewer["buttons"][1])
        self.assertEqual(axes.get_xlim(), (0.5, 1.5))
        self.assertEqual(axes.get_ylim(), (2.5, 3.2))
        click_button(self.viewer["buttons"][2])  # Reset view
        home = self.viewer["home_limits"]
        np.testing.assert_allclose(axes.get_xlim(), home[0])
        np.testing.assert_allclose(axes.get_ylim(), home[1])
        self.assertEqual(axes.get_aspect(), 1.0)
        self.assertEqual((axes.get_xlabel(), axes.get_ylabel()),
                         ("X (m)", "Y (m)"))

    def test_clicks_do_not_rerun_the_pipeline_or_redraw_artists(self):
        module = sys.modules[__name__]
        artists_before = {key: [id(a) for a in artists]
                          for key, artists in self.viewer["layers"].items()}
        children_before = len(self.viewer["axes"].get_children())
        forbidden = mock.Mock(side_effect=AssertionError("pipeline rerun"))
        with mock.patch.multiple(
                module, load_point_cloud=forbidden,
                preprocess_scans=forbidden, segment_scans=forbidden,
                fit_segments=forbidden, fit_line=forbidden,
                merge_segments=forbidden, build_layer_viewer=forbidden):
            for index in range(len(LAYER_KEYS)):
                self.viewer["checkboxes"].set_active(index)
            for button in self.viewer["buttons"]:
                click_button(button)
        forbidden.assert_not_called()
        self.assertEqual({key: [id(a) for a in artists] for key, artists
                          in self.viewer["layers"].items()}, artists_before)
        self.assertEqual(len(self.viewer["axes"].get_children()),
                         children_before)

    def test_inspection_style_and_line_styles(self):
        points = self.viewer["layers"]["points"][0]
        np.testing.assert_allclose(points.get_facecolor()[0][:3],
                                   matplotlib.colors.to_rgb("0.72"))
        merged = self.viewer["layers"]["merged"][0]
        pca = self.viewer["layers"]["pca"][0]
        recovered = self.viewer["layers"]["recovered"][0]
        self.assertGreater(merged.get_linewidth()[0],
                           pca.get_linewidth()[0])
        self.assertNotEqual(recovered.get_linestyle(), pca.get_linestyle())
        scans, clusters, records, merged_segments, _ = run_pipeline(self.path)
        coloured = build_layer_viewer(scans, clusters, records,
                                      merged_segments,
                                      inspection_style=False)
        colors = coloured["layers"]["points"][0].get_facecolor()
        self.assertGreater(len(np.unique(colors.round(3), axis=0)), 1)

    def test_main_opens_the_viewer_with_checkboxes(self):
        output, figure = self.run_main(str(self.path), view_mode="layers")
        viewer = sys.modules[__name__].ACTIVE_VIEWER
        self.assertIs(viewer["figure"], figure)
        self.assertEqual(len(viewer["checkboxes"].labels), len(LAYER_KEYS))
        self.assertIn("view 'layers'", output)


class TraceabilityTests(Part3TestCase):
    def test_no_hardcoded_geometry_the_same_map_anywhere(self):
        # Move and turn the WHOLE dataset (every saved pose) rigidly: the
        # measurements are unchanged, so every result must simply move with
        # it. Nothing depends on where the room is.
        if not CLASSROOM_FINAL.exists():
            self.skipTest("classroom_final.json absent")
        data = json.loads(CLASSROOM_FINAL.read_text(encoding="utf-8"))
        shift, turn = np.array([25.0, -40.0]), 37.0
        for entry in data["scans"]:
            pose = entry["pose"]
            moved = local_to_global(np.array([[pose["x_m"], pose["y_m"]]]),
                                    (shift[0], shift[1], turn))[0]
            pose["x_m"], pose["y_m"] = float(moved[0]), float(moved[1])
            pose["theta_deg"] = pose["theta_deg"] + turn
        moved_path = Path(self.folder.name) / "moved.json"
        moved_path.write_text(json.dumps(data), encoding="utf-8")
        *_, merged, _ = run_pipeline(CLASSROOM_FINAL)
        *_, moved_merged, _ = run_pipeline(moved_path)
        self.assertEqual(len(moved_merged), len(merged))
        self.assertAlmostEqual(sum(s["length"] for s in moved_merged),
                               sum(s["length"] for s in merged), places=6)
        self.assertEqual(sorted(s["support"] for s in moved_merged),
                         sorted(s["support"] for s in merged))

    def test_every_merged_segment_is_traceable_and_not_extended(self):
        if not CLASSROOM_FINAL.exists():
            self.skipTest("classroom_final.json absent")
        scans, _, _, merged, items = run_pipeline(CLASSROOM_FINAL)
        by_id = {item["id"]: item for item in items}
        scan_by_id = {scan["id"]: scan for scan in scans}
        for segment in merged:
            members = [by_id[member] for member in segment["member_ids"]]
            self.assertEqual(segment["scan_ids"],
                             sorted({item["scan_id"] for item in members}))
            support = []
            for item in members:  # back to the measured points of its scan
                cluster = item["segment"]["cluster"]
                scan = scan_by_id[item["scan_id"]]
                np.testing.assert_array_equal(
                    item["points"], scan["points_global"][cluster["indices"]])
                self.assertTrue(np.all(scan["valid"][cluster["indices"]]))
                support.append(item["points"])
            support = np.vstack(support)
            # Not extended: both endpoints lie within the extent of the
            # supporting measurements along the segment, up to a projection
            # effect: the merged ends are member ENDPOINTS projected on the
            # merged line; a member endpoint is at most LINE_MAX_ERROR_M
            # from its extreme point, across a line tilted at most
            # MERGE_MAX_ANGLE_DEG, so the end can pass the last measured
            # point by at most LINE_MAX_ERROR_M * sin(MERGE_MAX_ANGLE_DEG)
            # (6 mm; 1.8 mm at most in the classroom).
            bound = LINE_MAX_ERROR_M * math.sin(math.radians(
                MERGE_MAX_ANGLE_DEG)) + 1e-9
            along = (support - segment["start"]) @ segment["direction"]
            self.assertLessEqual(-along.min(), bound)
            self.assertLessEqual(segment["length"] - along.max(), bound)

    def test_classroom_merged_results_unchanged(self):
        if not CLASSROOM_FINAL.exists():
            self.skipTest("classroom_final.json absent")
        before = file_sha256(CLASSROOM_FINAL)
        *_, merged, _ = run_pipeline(CLASSROOM_FINAL)
        ends = np.array([np.concatenate([s["start"], s["end"]])
                         for s in merged])
        self.assertEqual(len(merged), 74)
        self.assertEqual(hashlib.sha256(ends.tobytes()).hexdigest(),
                         MILESTONE_5_MERGED_ENDPOINTS_SHA256)
        self.assertEqual(hashlib.sha256(json.dumps(
            [s["member_ids"] for s in merged]).encode()).hexdigest(),
            MILESTONE_5_MERGED_MEMBERS_SHA256)
        output, figure = self.run_main("classroom_final.json",
                                       view_mode="layers")
        self.assertIn("Session file unchanged: True", output)
        self.assertEqual(file_sha256(CLASSROOM_FINAL), before)


def cli(argv=None):
    global SESSION_FILE
    parser = argparse.ArgumentParser(description="Vector mapping")
    parser.add_argument("--session", type=Path, help="Session JSON; skips the picker")
    parser.add_argument("--non-interactive", action="store_true",
                        help="No picker or plot window; use --session or the default session")
    args = parser.parse_args(argv)
    selected = args.session or (resolve_session_path(SESSION_FILE) if args.non_interactive
                                else choose_scan_file())
    if selected is None:
        print("No file selected.")
        return
    SESSION_FILE = str(selected)
    if args.non_interactive:
        plt.switch_backend("Agg")
    main()
    if args.non_interactive:
        plt.close("all")


if __name__ == "__main__":
    cli()
