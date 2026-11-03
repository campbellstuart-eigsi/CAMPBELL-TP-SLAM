"""Part 2 - Occupancy grid mapping from the aligned classroom scans.

Step 1:
    - load the classroom session saved by Part 1 (READ-ONLY),
    - take each scan's ORIGINAL local Cartesian points and its saved pose,
    - transform the points into the global classroom frame,
    - choose the grid boundaries and resolution (10 cm cells),
    - create the occupancy grid with every cell UNKNOWN,
    - display the empty grid (with the scans on top, to check it fits).
Step 2:
    - Bresenham ray tracing from each scan's LiDAR position to each point,
    - basic BINARY update: crossed cells free (0), impact cell occupied
      (255), never-observed cells stay unknown (127); all scans combined.
Step 3:
    - PROBABILISTIC update with log-odds: each scan adds evidence (crossed
      cells towards free, impact cells towards occupied); conflicting
      observations are weighed against each other instead of "occupied
      always wins". The binary map of step 2 is kept unchanged.
Not yet: cleaning / time decay (later steps).

Follows the course document "Construction d'une grid map 2D a partir d'un
scan Lidar", sections 1 (define the grid; binary or probabilistic cells),
2 (LiDAR points -> grid), 3 (cells crossed by the beam: free, or lower
their probability) and 4 (impact cell: occupied, or raise its probability).

Automated tests, from the project folder:
    python -m unittest 02_grid_mapping -v
"""
import argparse
import contextlib
import hashlib
import io
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

import matplotlib.pyplot as plt
import numpy as np
# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------
# The Part 1 session to map. A bare file name is looked for in the "scans"
# folder NEXT TO THIS PROGRAM (whatever folder PyCharm runs it from), e.g.
#     SESSION_FILE = "classroom_final.json"
#     SESSION_FILE = "classroom_session.json"
#     SESSION_FILE = "new_room.json"
# An absolute path loads a file from anywhere else, e.g.
#     SESSION_FILE = r"C:\Users\me\Documents\other_room.json"
# The file is only ever READ. Any number of scans is accepted.
SESSION_FILE = "classroom_final.json"
SCANS_DIR = Path(__file__).resolve().parent / "scans"

def choose_scan_file():
    from tkinter import Tk, filedialog
    root = Tk()
    root.withdraw()
    try:
        filename = filedialog.askopenfilename(
            parent=root, title="Select LiDAR Scan File",
            initialdir=str(SCANS_DIR), filetypes=[("JSON Files", "*.json")])
    finally:
        root.destroy()
    return Path(filename) if filename else None

CELL_SIZE_M = 0.10
GRID_MARGIN_M = 0.50
# None = automatic: the grid holds every point and pose (+ margin).
# Or fix it by hand, e.g. (-6.5, -3.0, 5.0, 11.5) = (x_min, y_min, x_max,
# y_max) in metres to keep only the classroom; points outside are counted.
GRID_BOUNDS_M = None
# Which map to build and display:
#   "empty"         - the all-unknown grid of step 1
#   "occupancy"     - the BINARY map of step 2 (free / occupied / unknown)
#   "probabilistic" - the LOG-ODDS map of step 3 (degrees of belief)
GRID_TO_SHOW = "probabilistic"
VALID_GRIDS_TO_SHOW = ("empty", "occupancy", "probabilistic")
SHOW_SCANS_ON_GRID = False  # True: coloured LiDAR points on top of the grid
#                             (robot poses are always drawn)
MAX_RANGE_M = 12.0       # LD06 maximum range: farther points are not trusted

# Probabilistic (log-odds) update. Values = OctoMap's defaults (Hornung et
# al., 2013), a widely used LiDAR mapping library:
#   P_HIT  = 0.70: a beam ENDING in a cell says "occupied" with 70 % belief.
#   P_MISS = 0.40: a beam CROSSING a cell says "occupied" with only 40 %,
#                  i.e. weak evidence of free space.
# Hits are trusted more than misses: an echo is strong evidence of an
# object, while a beam can pass a thin chair leg or a cell corner without
# proving the whole 10 cm cell empty. With these values one hit outweighs
# two misses (0.85 > 2 x 0.41).
#   P_MIN = 0.12, P_MAX = 0.97: limits of belief (clamping). No cell ever
# becomes "certain", so later evidence can still change it, and a few
# hundred beams near the robot cannot make a cell infinitely free.
P_PRIOR = 0.5            # never observed: as likely free as occupied
P_HIT = 0.70
P_MISS = 0.40
P_MIN = 0.12
P_MAX = 0.97
# Classification thresholds (summary only; the map keeps the probability).
P_FREE_BELOW = 0.35      # p <= 0.35: free (e.g. 2 misses: p = 0.31)
P_OCCUPIED_ABOVE = 0.65  # p >= 0.65: occupied (e.g. 1 hit: p = 0.70)
# When to clamp to [P_MIN, P_MAX] (see section 6, "order"):
#   False: add ALL scans' evidence, clamp once at the end. The result does
#          not depend on the scan order (exact Bayes sum for saved scans).
#   True : clamp after every scan, L_new = clamp(L_old + L_measurement),
#          as a robot updating its map live would. Cells that reach a limit
#          then depend on the order (307 cells for the classroom).
CLAMP_EACH_SCAN = False

# Cell values (course document, section 1: a "probabilistic" uint8 map).
# One byte per cell, from 0 (certainly free) to 255 (certainly occupied).
FREE = 0
UNKNOWN = 127            # neutral starting value: nothing observed yet
OCCUPIED = 255

# The session format written by Part 1 (01_lidar_visualizer.py).
SESSION_FORMAT_NAME = "ld06_slam_session"
SESSION_FORMAT_VERSION = 2


# ---------------------------------------------------------------------------
# 1. Load the classroom session (read-only)
# ---------------------------------------------------------------------------
class SessionError(Exception):
    """The session file is missing, is not valid JSON or is not usable."""


def resolve_session_path(file_name, scans_dir=None):
    """Turn the SESSION_FILE setting into the full path of the file to read.

    A bare name ("classroom_final.json") or a relative path is taken inside
    scans_dir (default: the "scans" folder next to this program), so the
    result never depends on the folder the program is started from. A name
    without an extension gets ".json". An absolute path is used as it is.
    """
    path = Path(file_name)
    if not path.suffix:
        path = path.with_suffix(".json")
    if path.is_absolute():
        return path
    return Path(scans_dir if scans_dir is not None else SCANS_DIR) / path


def load_session_scans(path, visible_only=True):
    """Read a Part 1 session file and return its scans; never writes.

    path         - the .json file saved by Part 1
    visible_only - skip scans that were hidden in Part 1 (hiding a scan
                   there meant "do not use it")
    Returns a list of dictionaries, one per scan, in the file's order:
        "id"           - the scan number (unique, may have gaps: 1, 2, 5...)
        "name"         - e.g. "Scan 5"
        "pose"         - (x_m, y_m, theta_deg) of the LiDAR in the classroom
        "points_local" - (N, 2) array of ORIGINAL points in the LiDAR frame,
                         metres, +X = LiDAR front, +Y = LiDAR left
        "aligned"      - True if Part 1's automatic alignment was accepted
        "color"        - the scan's colour in Part 1 (None if not saved)
    """
    path = Path(path)
    try:
        with open(path, "r", encoding="utf-8") as file:  # "r": read only
            data = json.load(file)
    except OSError as error:
        raise SessionError(f"cannot read {path}: {error.strerror}") from error
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        raise SessionError(f"{path.name} is not valid JSON: {error}") from error

    if not isinstance(data, dict) or data.get("format") != SESSION_FORMAT_NAME:
        raise SessionError(f"{path.name} is not a Part 1 session file")
    if data.get("version") != SESSION_FORMAT_VERSION:
        raise SessionError(f"{path.name} has version {data.get('version')!r}; "
                           f"open it in Part 1 and save it again "
                           f"(version {SESSION_FORMAT_VERSION})")
    if not isinstance(data.get("scans"), list):
        raise SessionError(f"{path.name}: 'scans' must be a list")

    scans = []
    for index, entry in enumerate(data["scans"]):
        where = f"{path.name}, scans[{index}]"
        try:
            pose = (float(entry["pose"]["x_m"]), float(entry["pose"]["y_m"]),
                    float(entry["pose"]["theta_deg"]))
            points = np.array(entry["points_local_m"], dtype=float)
        except (KeyError, TypeError, ValueError) as error:
            raise SessionError(f"{where}: missing or invalid pose/points "
                               f"({error!r})") from error
        if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
            raise SessionError(f"{where}: points_local_m must be a non-empty "
                               f"list of [x, y]")
        if not (np.all(np.isfinite(points)) and np.all(np.isfinite(pose))):
            raise SessionError(f"{where}: contains NaN or infinite values")
        if visible_only and not entry.get("visible", True):
            continue
        scans.append({"id": entry.get("id", index + 1),
                      "name": entry.get("name", f"Scan {index + 1}"),
                      "pose": pose, "points_local": points,
                      "aligned": bool(entry.get("aligned", False)),
                      "color": entry.get("color")})
    return scans


# ---------------------------------------------------------------------------
# 2. Local LiDAR coordinates -> global classroom coordinates
# ---------------------------------------------------------------------------
def rotation_matrix(theta_deg):
    """2x2 matrix that turns a vector by theta (counter-clockwise).

        R(theta) = | cos(theta)  -sin(theta) |
                   | sin(theta)   cos(theta) |
    """
    theta_rad = math.radians(theta_deg)
    return np.array([[math.cos(theta_rad), -math.sin(theta_rad)],
                     [math.sin(theta_rad), math.cos(theta_rad)]])


def local_to_global(points_local, pose):
    """Move points from the LiDAR's own frame into the classroom frame.

    points_local - (N, 2) array, metres, in the LiDAR frame
    pose         - (x_r, y_r, theta_r): where the LiDAR stood in the
                   classroom and which way it faced (theta_r in degrees,
                   counter-clockwise from the classroom +X axis)

    Course formula:  P_global = R(theta_r) P_local + T(x_r, y_r)
    i.e. first ROTATE the point by the robot's heading, then SHIFT it by
    the robot's position:
        x_g = x_r + x_l cos(theta_r) - y_l sin(theta_r)
        y_g = y_r + x_l sin(theta_r) + y_l cos(theta_r)
    Each point is a row, so the whole array is done at once:
    (R @ P_local^T)^T = P_local @ R^T. Same formula as Part 1.
    """
    x_r, y_r, theta_deg = pose
    return points_local @ rotation_matrix(theta_deg).T + np.array([x_r, y_r])


def global_points_of_scans(scans):
    """Return one (N, 2) array of global points per scan, in scan order."""
    return [local_to_global(scan["points_local"], scan["pose"])
            for scan in scans]


# ---------------------------------------------------------------------------
# 3. The grid: boundaries, resolution, cell indices
# ---------------------------------------------------------------------------
# A grid is described by a small dictionary (its "spec"):
#     x_min, y_min - classroom coordinates of the grid's lower-left corner
#     cell_size    - side of one square cell, metres
#     width        - number of cells along X (columns, index i)
#     height       - number of cells along Y (rows, index j)
# The cells are stored in a NumPy array grid[j, i]: row j first, as in any
# matrix, so that the image is drawn with X to the right and Y upwards.
def create_grid_spec(x_min, y_min, x_max, y_max, cell_size):
    """Describe a grid covering [x_min, x_max] x [y_min, y_max].

    The number of cells is rounded UP, so the whole area is covered:
    a 2 m x 3 m map with 0.1 m cells is 20 x 30 cells (course example).
    """
    if cell_size <= 0 or x_max <= x_min or y_max <= y_min:
        raise ValueError("the grid needs a positive size and resolution")
    # round(..., 9) removes tiny float errors such as 2.0000000001 / 0.1.
    width = math.ceil(round((x_max - x_min) / cell_size, 9))
    height = math.ceil(round((y_max - y_min) / cell_size, 9))
    return {"x_min": x_min, "y_min": y_min, "cell_size": cell_size,
            "width": width, "height": height}


def choose_grid_bounds(global_point_arrays, poses, cell_size=CELL_SIZE_M,
                       margin=GRID_MARGIN_M):
    """Smallest grid that holds every point and every robot position.

    Adds `margin` all around, then moves each edge OUTWARDS to a multiple
    of the cell size, so cell borders fall on round coordinates (e.g. -5.8
    and not -5.7239...). Returns (x_min, y_min, x_max, y_max) in metres.
    """
    everything = np.vstack(list(global_point_arrays)
                           + [np.array(poses, dtype=float)[:, :2]])
    low = everything.min(axis=0) - margin
    high = everything.max(axis=0) + margin
    x_min, y_min = np.floor(low / cell_size) * cell_size
    x_max, y_max = np.ceil(high / cell_size) * cell_size
    return tuple(round(float(value), 6) for value in (x_min, y_min,
                                                      x_max, y_max))


def create_occupancy_grid(spec, initial_value=UNKNOWN):
    """Return the grid: a (height, width) uint8 array, all `initial_value`.

    uint8 = one byte per cell (0-255), as advised in the course for
    probability maps; UNKNOWN (127) = nothing observed yet.
    """
    return np.full((spec["height"], spec["width"]), initial_value,
                   dtype=np.uint8)


def world_to_grid(spec, x, y):
    """Classroom coordinates (m) -> cell indices (i = column, j = row).

        i = floor((x - x_min) / cell_size)
        j = floor((y - y_min) / cell_size)
    The course writes i = x / resolution; subtracting x_min first makes the
    lower-left cell (0, 0) even when the classroom has negative
    coordinates. floor() (not int()) matters for points LEFT of or BELOW
    the grid: (x - x_min) / cell_size = -0.5 must give -1 (outside), but
    int(-0.5) = 0 would wrongly put the point in the first cell.
    A point exactly on a border belongs to the cell above/right of it.
    Works on single numbers or NumPy arrays. Indices may be outside the
    grid; check them with is_inside_grid().
    """
    i = np.floor((np.asarray(x) - spec["x_min"]) / spec["cell_size"])
    j = np.floor((np.asarray(y) - spec["y_min"]) / spec["cell_size"])
    return i.astype(int), j.astype(int)


def grid_to_world(spec, i, j):
    """Cell indices -> classroom coordinates of the cell CENTRE (m).

        x = x_min + (i + 0.5) * cell_size,  y = y_min + (j + 0.5) * cell_size
    """
    return (spec["x_min"] + (np.asarray(i) + 0.5) * spec["cell_size"],
            spec["y_min"] + (np.asarray(j) + 0.5) * spec["cell_size"])


def is_inside_grid(spec, i, j):
    """True where the cell (i, j) exists: 0 <= i < width, 0 <= j < height."""
    i, j = np.asarray(i), np.asarray(j)
    return (i >= 0) & (i < spec["width"]) & (j >= 0) & (j < spec["height"])


def grid_extent(spec):
    """(left, right, bottom, top) of the whole grid in metres, for imshow."""
    return (spec["x_min"], spec["x_min"] + spec["width"] * spec["cell_size"],
            spec["y_min"], spec["y_min"] + spec["height"] * spec["cell_size"])


# ---------------------------------------------------------------------------
# 4. Ray tracing with Bresenham's algorithm
# ---------------------------------------------------------------------------
# A LiDAR beam is a straight segment from the sensor to the measured point.
# On a grid we need the list of CELLS that segment crosses. Bresenham's
# algorithm (1962) finds them using only integer additions and comparisons.
#
# Idea, for a "shallow" line (it moves more along i than along j):
#   - walk along i one cell at a time, from the start to the end;
#   - j must follow the ideal line j = j0 + (i - i0) * dj / di;
#   - instead of computing that fraction, keep an "error" = how far the
#     ideal line has drifted from the current cell row; when the drift
#     passes half a cell, step j by one too (a diagonal step).
# Multiplying everything by 2*di keeps the error an INTEGER, so there is
# no rounding at all. The version below handles both axes at once
# (it works for every direction without separate cases):
#     dx = |i1 - i0|,  dy = -|j1 - j0|,  err = dx + dy
#     at each step:  e2 = 2*err
#                    if e2 >= dy: err += dy, i += step_i   (move along i)
#                    if e2 <= dx: err += dx, j += step_j   (move along j)
# Special cases, all handled by the same loop:
#     horizontal (dy = 0): only i moves; vertical (dx = 0): only j moves;
#     diagonal (dx = |dy|): both move at every step;
#     steep (|dy| > dx): j moves every step, i only sometimes;
#     reverse direction (end before start): step_i / step_j = -1;
#     same cell: the start cell alone.
# The cells are "8-connected": consecutive cells touch by a side or a
# corner, and there are max(|di|, |dj|) + 1 of them, start and end included.
def bresenham_cells(i0, j0, i1, j1):
    """Return the list of cells (i, j) from (i0, j0) to (i1, j1), inclusive.

    Pure integer arithmetic; any direction; indices may be negative or
    outside the grid (the caller keeps only the cells that exist).
    """
    i0, j0, i1, j1 = int(i0), int(j0), int(i1), int(j1)
    dx = abs(i1 - i0)
    dy = -abs(j1 - j0)
    step_i = 1 if i0 < i1 else -1
    step_j = 1 if j0 < j1 else -1
    error = dx + dy
    cells = []
    i, j = i0, j0
    while True:
        cells.append((i, j))
        if i == i1 and j == j1:
            return cells
        doubled_error = 2 * error
        if doubled_error >= dy:   # the ideal line is far enough along i
            error += dy
            i += step_i
        if doubled_error <= dx:   # ... and/or along j
            error += dx
            j += step_j


# ---------------------------------------------------------------------------
# 5. Basic (binary) occupancy update
# ---------------------------------------------------------------------------
# For each measurement (course document, sections 3 and 4):
#     cells crossed by the beam BEFORE the impact -> seen FREE
#     the impact cell (the measured point)        -> seen OCCUPIED
#     cells no beam ever reached                  -> stay UNKNOWN
# Two yes/no layers record what was SEEN by all scans together; the final
# grid is built at the end, with OCCUPIED written last:
#     grid = UNKNOWN; grid[seen free] = FREE; grid[seen occupied] = OCCUPIED
# So an obstacle supported by at least one impact is never erased by a beam
# of another scan passing through that cell (e.g. a beam grazing a wall
# corner, or slightly misaligned scans), and the result does not depend on
# the order of the scans. The price: anything seen once (a person walking
# by) stays occupied - the probabilistic update of the next step fixes that.
def create_observation_layers(spec):
    """Two empty yes/no layers: cells seen free, cells seen occupied."""
    shape = (spec["height"], spec["width"])
    return {"free": np.zeros(shape, dtype=bool),
            "occupied": np.zeros(shape, dtype=bool)}


def mark_ray(layers, spec, start_cell, end_cell):
    """Record one beam from start_cell (the sensor) to end_cell (the impact).

    All cells of the Bresenham line EXCEPT the last are seen free; the last
    one is seen occupied. Cells outside the grid are skipped, so a beam that
    leaves the grid still frees the part that is inside, and an impact
    outside the grid marks nothing occupied. If start and end are the same
    cell, that cell is only marked occupied.
    Returns True if the impact cell is inside the grid.
    """
    cells = np.array(bresenham_cells(*start_cell, *end_cell))
    crossed, impact = cells[:-1], cells[-1]
    inside = is_inside_grid(spec, crossed[:, 0], crossed[:, 1])
    layers["free"][crossed[inside, 1], crossed[inside, 0]] = True  # [j, i]
    if is_inside_grid(spec, impact[0], impact[1]):
        layers["occupied"][impact[1], impact[0]] = True
        return True
    return False


def trace_scan(layers, spec, scan, points_global, max_range_m=MAX_RANGE_M):
    """Trace every beam of one scan, from ITS OWN saved LiDAR position.

    The beams start at the scan's pose (x_r, y_r), the place where the
    LiDAR actually stood, not at the classroom origin (0, 0).
    Points farther than max_range_m from the LiDAR (beyond the LD06's
    range) or not finite are ignored, as the course advises filtering
    aberrant points. Returns counts for the summary.
    """
    x_r, y_r, _ = scan["pose"]
    start_cell = tuple(int(value) for value in world_to_grid(spec, x_r, y_r))
    end_i, end_j = world_to_grid(spec, points_global[:, 0],
                                 points_global[:, 1])
    ranges = np.hypot(points_global[:, 0] - x_r, points_global[:, 1] - y_r)
    counts = {"rays": 0, "too_far": 0, "impact_outside": 0}
    for i, j, distance in zip(end_i, end_j, ranges):
        if not distance <= max_range_m:  # also catches NaN
            counts["too_far"] += 1
            continue
        counts["rays"] += 1
        if not mark_ray(layers, spec, start_cell, (i, j)):
            counts["impact_outside"] += 1
    return counts


def combine_layers(layers):
    """Turn the seen-free / seen-occupied layers into the uint8 grid."""
    grid = np.full(layers["free"].shape, UNKNOWN, dtype=np.uint8)
    grid[layers["free"]] = FREE
    grid[layers["occupied"]] = OCCUPIED  # last: an impact always wins
    return grid


def build_binary_occupancy_grid(spec, scans, global_point_arrays,
                                max_range_m=MAX_RANGE_M):
    """Trace all scans into one map; return (grid, counts).

    counts["rays"] / ["too_far"] / ["impact_outside"] are totals over all
    scans; counts["free_and_occupied"] is the number of cells that some
    beam crossed AND some beam ended in (they are shown as occupied).
    """
    layers = create_observation_layers(spec)
    totals = {"rays": 0, "too_far": 0, "impact_outside": 0}
    for scan, points in zip(scans, global_point_arrays):
        for key, value in trace_scan(layers, spec, scan, points,
                                     max_range_m).items():
            totals[key] += value
    totals["free_and_occupied"] = int(np.sum(layers["free"]
                                             & layers["occupied"]))
    return combine_layers(layers), totals


def count_cells(grid):
    """Return {"free": n, "occupied": n, "unknown": n} for a binary grid."""
    return {"free": int(np.sum(grid == FREE)),
            "occupied": int(np.sum(grid == OCCUPIED)),
            "unknown": int(np.sum(grid == UNKNOWN))}


# ---------------------------------------------------------------------------
# 6. Probabilistic occupancy update (log-odds)
# ---------------------------------------------------------------------------
# Each cell stores a degree of belief p = P(cell occupied), 0 to 1.
# Adding evidence to probabilities directly is awkward (0.7 + 0.7 is not a
# probability), so we store the LOG-ODDS instead:
#     L = log(p / (1 - p))       p = 0.5 -> L = 0;  p > 0.5 -> L > 0
#     p = 1 / (1 + exp(-L))      (the inverse: back to a probability)
# For independent measurements, Bayes' rule becomes a simple SUM:
#     L_new = L_old + L_measurement,  L_measurement = log(p_m / (1 - p_m))
# with p_m = P_HIT for the impact cell and P_MISS for crossed cells. The
# sum is clamped to [L_MIN, L_MAX] after every update (no certainty).
#
# Each SCAN updates a cell at most once: the cells of one scan are first
# collected with the binary step's trace_scan() (Bresenham, unchanged),
# then  hit cells: L += L_HIT;  crossed but not hit: L += L_MISS.
# Without this, hundreds of beams of ONE scan crossing the cells next to the
# LiDAR, or several points of one scan in the same wall cell, would count
# as hundreds of independent observations. A cell hit and crossed in the
# same scan counts as hit (the beam ended there).
#
# Conflicting scans are WEIGHED: a cell hit by 1 scan and crossed by 1
# other has L = 0.85 - 0.41 = 0.44, p = 0.61 (uncertain); crossed by 4
# others, p = 0.32 (free). In the binary map it would stay occupied.
#
# Order: the SUM does not depend on the order of the scans, but clamping
# after every update does: e.g. 5 hits then 1 miss gives 3.48 - 0.41 =
# 3.07, while 1 miss then 5 hits gives 3.48. So by default (CLAMP_EACH_SCAN
# = False) all evidence is added first and clamped ONCE at the end:
#     L = clamp(L_prior + h * L_HIT + m * L_MISS, L_MIN, L_MAX)
# with h / m = number of scans that hit / crossed the cell: order-free.
# CLAMP_EACH_SCAN = True applies L_new = clamp(L_old + L_measurement) after
# every scan instead, as a robot mapping live would (the clamp then keeps
# the map able to change quickly when the room changes).
def probability_to_log_odds(p):
    """L = log(p / (1 - p)); works on numbers and NumPy arrays."""
    p = np.asarray(p, dtype=float)
    return np.log(p / (1.0 - p))


def log_odds_to_probability(log_odds):
    """p = 1 / (1 + exp(-L)); works on numbers and NumPy arrays."""
    return 1.0 / (1.0 + np.exp(-np.asarray(log_odds, dtype=float)))


L_PRIOR = float(probability_to_log_odds(P_PRIOR))  # 0.0
L_HIT = float(probability_to_log_odds(P_HIT))      # +0.847
L_MISS = float(probability_to_log_odds(P_MISS))    # -0.405
L_MIN = float(probability_to_log_odds(P_MIN))      # -1.99
L_MAX = float(probability_to_log_odds(P_MAX))      # +3.48


def update_log_odds(log_odds, cells, measurement, l_min=L_MIN, l_max=L_MAX):
    """L_new = clamp(L_old + L_measurement, L_min, L_max), in place.

    log_odds    - the map's (height, width) float array
    cells       - boolean mask (same shape) of the cells to update
    measurement - L_HIT or L_MISS
    """
    log_odds[cells] = np.clip(log_odds[cells] + measurement, l_min, l_max)


def create_log_odds_map(spec):
    """A probabilistic map: every cell at the prior, nothing observed.

    "log_odds" - the belief of each cell (float, starts at L_PRIOR = 0)
    "hits"     - number of scans whose beam ENDED in the cell
    "misses"   - number of scans whose beam only CROSSED the cell
    (the two counters are kept to explain the result, e.g. conflicts)
    """
    shape = (spec["height"], spec["width"])
    return {"log_odds": np.full(shape, L_PRIOR),
            "hits": np.zeros(shape, dtype=np.int32),
            "misses": np.zeros(shape, dtype=np.int32)}


def integrate_scan_log_odds(log_odds_map, spec, scan, points_global,
                            max_range_m=MAX_RANGE_M, clamp=True):
    """Add ONE scan's evidence to the map (each cell at most once).

    Uses the binary step's trace_scan() to find, with Bresenham from the
    scan's own LiDAR position, which cells this scan saw hit or crossed.
    clamp=False adds the evidence without limits (clamped later).
    Returns the ray counts of trace_scan().
    """
    seen = create_observation_layers(spec)
    counts = trace_scan(seen, spec, scan, points_global, max_range_m)
    hit = seen["occupied"]
    missed = seen["free"] & ~hit  # hit wins within the same scan
    limits = (L_MIN, L_MAX) if clamp else (-np.inf, np.inf)
    update_log_odds(log_odds_map["log_odds"], hit, L_HIT, *limits)
    update_log_odds(log_odds_map["log_odds"], missed, L_MISS, *limits)
    log_odds_map["hits"] += hit
    log_odds_map["misses"] += missed
    return counts


def build_probabilistic_occupancy_map(spec, scans, global_point_arrays,
                                      max_range_m=MAX_RANGE_M,
                                      clamp_each_scan=CLAMP_EACH_SCAN):
    """Integrate all scans, in order; return (log_odds_map, counts).

    clamp_each_scan=False: sum all evidence, then clamp once (order-free);
    True: clamp after every scan (online update).
    """
    log_odds_map = create_log_odds_map(spec)
    totals = {"rays": 0, "too_far": 0, "impact_outside": 0}
    for scan, points in zip(scans, global_point_arrays):
        for key, value in integrate_scan_log_odds(
                log_odds_map, spec, scan, points, max_range_m,
                clamp=clamp_each_scan).items():
            totals[key] += value
    if not clamp_each_scan:
        np.clip(log_odds_map["log_odds"], L_MIN, L_MAX,
                out=log_odds_map["log_odds"])
    return log_odds_map, totals


def probabilities_to_uint8(probabilities):
    """p (0-1) -> the course's 0-255 cell value: floor(255 p).

    p = 0.5 gives 127, the course's "unknown" value; p = 0 gives 0, p = 1
    gives 255. Used for display, with the same grey scale as the binary map.
    """
    return np.floor(np.asarray(probabilities) * 255.0).astype(np.uint8)


def classify_probabilities(log_odds_map, free_below=P_FREE_BELOW,
                           occupied_above=P_OCCUPIED_ABOVE):
    """Count cells as free / occupied / uncertain / unobserved.

    unobserved - no scan ever reached the cell (still exactly the prior)
    free       - observed and p <= free_below
    occupied   - observed and p >= occupied_above
    uncertain  - observed, in between (e.g. conflicting evidence)
    Also counts the cells with CONFLICTING evidence (hit by some scans,
    crossed by others) and how they were classified.
    """
    probabilities = log_odds_to_probability(log_odds_map["log_odds"])
    observed = (log_odds_map["hits"] + log_odds_map["misses"]) > 0
    free = observed & (probabilities <= free_below)
    occupied = observed & (probabilities >= occupied_above)
    conflict = (log_odds_map["hits"] > 0) & (log_odds_map["misses"] > 0)
    return {"free": int(free.sum()), "occupied": int(occupied.sum()),
            "uncertain": int((observed & ~free & ~occupied).sum()),
            "unobserved": int((~observed).sum()),
            "conflicting": int(conflict.sum()),
            "conflicting_occupied": int((conflict & occupied).sum()),
            "conflicting_uncertain": int((conflict & ~free
                                          & ~occupied).sum()),
            "conflicting_free": int((conflict & free).sum())}


def check_grid_to_show(value):
    """Refuse an unknown GRID_TO_SHOW instead of silently showing another map."""
    if value not in VALID_GRIDS_TO_SHOW:
        raise ValueError(f"GRID_TO_SHOW = {value!r} is not valid; choose one "
                         f"of {', '.join(map(repr, VALID_GRIDS_TO_SHOW))}")


# ---------------------------------------------------------------------------
# 7. Display
# ---------------------------------------------------------------------------
BINARY_COLORBAR_LABELS = ("0 free", "127 unknown", "255 occupied")
PROBABILITY_COLORBAR_LABELS = ("p = 0 (free)", "p = 0.5 (unknown)",
                               "p = 1 (occupied)")


def show_grid(grid, spec, scans=None, global_point_arrays=None, title="",
              show_points=True, colorbar_labels=BINARY_COLORBAR_LABELS,
              figure=None):
    """Draw the grid as an image (and optionally the scans on top).

    Colour = cell value: 0 (free) white, 127 (unknown) grey, 255 (occupied)
    black: the usual look of occupancy maps. origin="lower" puts row j = 0
    at the BOTTOM, so the image has the same orientation as the classroom
    plot of Part 1 (X to the right, Y upwards).
    scans, global_point_arrays - draw each robot pose (dot + heading arrow)
    and, if show_points is True, the coloured LiDAR points.
    grid is a uint8 array: the binary map, or a probability map converted
    with probabilities_to_uint8() (same grey scale; other colour-bar text).
    Returns the figure.
    """
    if figure is None:
        figure, axes = plt.subplots(figsize=(11, 8))
    else:
        axes = figure.add_subplot(111)
    image = axes.imshow(grid, cmap="gray_r", vmin=FREE, vmax=OCCUPIED,
                        origin="lower", extent=grid_extent(spec),
                        interpolation="nearest")
    colorbar = figure.colorbar(image, ax=axes, shrink=0.8)
    colorbar.set_ticks([FREE, UNKNOWN, OCCUPIED])
    colorbar.set_ticklabels(list(colorbar_labels))

    if scans is not None and global_point_arrays is not None:
        colors = plt.get_cmap("tab10")
        for index, (scan, points) in enumerate(zip(scans,
                                                   global_point_arrays)):
            color = scan.get("color") or colors(index % 10)  # as in Part 1
            if show_points:
                axes.scatter(points[:, 0], points[:, 1], s=1, color=color)
            x_r, y_r, theta_deg = scan["pose"]
            axes.plot(x_r, y_r, "o", color=color, markeredgecolor="black",
                      markersize=6)
            axes.annotate("", xy=(x_r + 0.4 * math.cos(math.radians(theta_deg)),
                                  y_r + 0.4 * math.sin(math.radians(theta_deg))),
                          xytext=(x_r, y_r),
                          arrowprops={"arrowstyle": "->", "color": color})

    axes.set_xlabel("X (m)")
    axes.set_ylabel("Y (m)")
    axes.set_aspect("equal")
    axes.set_title(title or (f"Occupancy grid {spec['width']} x "
                             f"{spec['height']} cells of "
                             f"{spec['cell_size'] * 100:.0f} cm"))
    # Light lines every metre; zoom in (toolbar) to see the 10 cm cells.
    axes.set_xticks(np.arange(math.ceil(spec["x_min"]),
                              grid_extent(spec)[1] + 1e-9, 1.0))
    axes.set_yticks(np.arange(math.ceil(spec["y_min"]),
                              grid_extent(spec)[3] + 1e-9, 1.0))
    axes.tick_params(labelsize=8)
    axes.grid(True, color="white", linewidth=0.4, alpha=0.6)
    return figure


def file_sha256(path):
    """Fingerprint of a file: it changes if even one byte changes."""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def print_cell_summary(grid, spec, counts):
    """Print how many cells are free / occupied / unknown, and the rays."""
    cells = count_cells(grid)
    cell_area_m2 = spec["cell_size"] ** 2
    print(f"Rays traced: {counts['rays']} (ignored beyond "
          f"{MAX_RANGE_M:.0f} m: {counts['too_far']}; impact outside the "
          f"grid: {counts['impact_outside']})")
    for name in ("free", "occupied", "unknown"):
        print(f"  {name:8s} {cells[name]:6d} cells "
              f"({cells[name] / grid.size:5.1%}, "
              f"{cells[name] * cell_area_m2:6.1f} m2)")
    print(f"  ({counts['free_and_occupied']} cells were crossed by some "
          f"beams AND hit by others: kept occupied)")


def print_probabilistic_summary(log_odds_map, spec, counts):
    """Print the settings, the cell classes, conflicts and the rays."""
    classes = classify_probabilities(log_odds_map)
    total = log_odds_map["log_odds"].size
    cell_area_m2 = spec["cell_size"] ** 2
    print(f"Log-odds settings: P_HIT {P_HIT} (L {L_HIT:+.3f}), P_MISS "
          f"{P_MISS} (L {L_MISS:+.3f}), limits p {P_MIN}-{P_MAX} "
          f"(L {L_MIN:+.2f} to {L_MAX:+.2f}), prior p {P_PRIOR}, clamped "
          f"{'after every scan' if CLAMP_EACH_SCAN else 'once at the end'}")
    print(f"Rays traced: {counts['rays']} (ignored beyond "
          f"{MAX_RANGE_M:.0f} m: {counts['too_far']}; impact outside the "
          f"grid: {counts['impact_outside']})")
    print(f"Classes (free: p <= {P_FREE_BELOW}, occupied: p >= "
          f"{P_OCCUPIED_ABOVE}, uncertain: observed in between, "
          f"unobserved: p = {P_PRIOR} untouched):")
    for name in ("free", "occupied", "uncertain", "unobserved"):
        print(f"  {name:10s} {classes[name]:6d} cells "
              f"({classes[name] / total:5.1%}, "
              f"{classes[name] * cell_area_m2:6.1f} m2)")
    print(f"Conflicting cells (hit by some scans, crossed by others): "
          f"{classes['conflicting']} -> occupied "
          f"{classes['conflicting_occupied']}, uncertain "
          f"{classes['conflicting_uncertain']}, free "
          f"{classes['conflicting_free']}")


def report_scan_order_effect(spec, scans, global_points, log_odds_map):
    """Rebuild with the scans in REVERSE order and report the differences.

    With CLAMP_EACH_SCAN = False there must be none; with True, cells that
    reached a limit can differ (see section 6).
    """
    reversed_map, _ = build_probabilistic_occupancy_map(
        spec, scans[::-1], global_points[::-1])
    difference = np.abs(reversed_map["log_odds"] - log_odds_map["log_odds"])
    changed = difference > 1e-9
    classes_forward = classify_probabilities(log_odds_map)
    classes_reverse = classify_probabilities(reversed_map)
    same_classes = all(classes_forward[key] == classes_reverse[key]
                       for key in ("free", "occupied", "uncertain"))
    print(f"Scan order check (all scans in reverse order): "
          f"{int(changed.sum())} cells differ, by at most "
          f"{difference.max():.3f} in log-odds; class counts identical: "
          f"{same_classes}")
    if changed.any():
        print("  (clamping after every scan makes cells that reached a "
              "limit depend on the order; set CLAMP_EACH_SCAN = False for "
              "an order-free map)")


def main():
    check_grid_to_show(GRID_TO_SHOW)  # a clear error for a wrong value
    print(f"=== Part 2: occupancy grid ({GRID_TO_SHOW}) ===")
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
        scans = load_session_scans(session_path)
    except SessionError as error:
        print(f"Cannot load the session: {error}")
        return
    if not scans:
        print("The session has no visible scan: nothing to map.")
        return
    global_points = global_points_of_scans(scans)

    print(f"{len(scans)} scans, {sum(len(p) for p in global_points)} points:")
    for scan in scans:
        x_r, y_r, theta_deg = scan["pose"]
        note = "" if scan["aligned"] else "  (not auto-aligned in Part 1)"
        print(f"  {scan['name']:8s} pose ({x_r:6.2f} m, {y_r:6.2f} m, "
              f"{theta_deg:6.1f} deg)  {len(scan['points_local']):5d} "
              f"points{note}")

    if GRID_BOUNDS_M is None:
        x_min, y_min, x_max, y_max = choose_grid_bounds(
            global_points, [scan["pose"] for scan in scans])
        bounds_note = "automatic"
    else:
        x_min, y_min, x_max, y_max = GRID_BOUNDS_M
        bounds_note = "set by hand"
    spec = create_grid_spec(x_min, y_min, x_max, y_max, CELL_SIZE_M)
    grid = create_occupancy_grid(spec)
    all_points = np.vstack(global_points)
    outside = ~is_inside_grid(spec, *world_to_grid(spec, all_points[:, 0],
                                                   all_points[:, 1]))
    print(f"Grid ({bounds_note}): x {x_min} to {x_max} m, y {y_min} to "
          f"{y_max} m ({x_max - x_min:.1f} m x {y_max - y_min:.1f} m); "
          f"{outside.sum()} of {len(all_points)} points outside it")
    print(f"      {spec['width']} x {spec['height']} cells of "
          f"{CELL_SIZE_M * 100:.0f} cm = {grid.size} cells, "
          f"{grid.nbytes} bytes, every cell starts at {UNKNOWN} (unknown)")

    if GRID_TO_SHOW == "occupancy":
        start_time = time.perf_counter()
        grid, counts = build_binary_occupancy_grid(spec, scans, global_points)
        print(f"Binary occupancy map built in "
              f"{time.perf_counter() - start_time:.2f} s.")
        print_cell_summary(grid, spec, counts)
        title = (f"BINARY occupancy map: {len(scans)} scans, "
                 f"{spec['cell_size'] * 100:.0f} cm cells "
                 f"(white free, black occupied, grey unknown)")
        colorbar_labels = BINARY_COLORBAR_LABELS
    elif GRID_TO_SHOW == "probabilistic":
        start_time = time.perf_counter()
        log_odds_map, counts = build_probabilistic_occupancy_map(
            spec, scans, global_points)
        print(f"Probabilistic (log-odds) map built in "
              f"{time.perf_counter() - start_time:.2f} s.")
        print_probabilistic_summary(log_odds_map, spec, counts)
        report_scan_order_effect(spec, scans, global_points, log_odds_map)
        grid = probabilities_to_uint8(
            log_odds_to_probability(log_odds_map["log_odds"]))
        title = (f"PROBABILISTIC occupancy map (log-odds): {len(scans)} "
                 f"scans, {spec['cell_size'] * 100:.0f} cm cells\n"
                 f"white = surely free, black = surely occupied, "
                 f"grey = uncertain / unobserved")
        colorbar_labels = PROBABILITY_COLORBAR_LABELS
    else:  # "empty"
        title = ""
        colorbar_labels = BINARY_COLORBAR_LABELS
    print(f"Session file unchanged: "
          f"{file_sha256(session_path) == fingerprint_before}")

    show_grid(grid, spec, scans, global_points, title,
              show_points=SHOW_SCANS_ON_GRID, colorbar_labels=colorbar_labels)
    plt.show()


# ---------------------------------------------------------------------------
# Automated tests (no ESP32, no window): python -m unittest 02_grid_mapping -v
# ---------------------------------------------------------------------------
# The real classroom tests always use this file, whatever SESSION_FILE says.
CLASSROOM_FINAL = SCANS_DIR / "classroom_final.json"
# Fingerprints of the verified maps of classroom_final.json (computed before
# SESSION_FILE became selectable): the results must never change.
VERIFIED_BINARY_SHA256 = (
    "ba885e1790ca5504cd5a8a835e7a13804a46c897799d3c590933e6e1698f3382")
VERIFIED_LOG_ODDS_SHA256 = (
    "4c811b70cbed2cd9193a1b57abfdef3c8487555b1da5318accadb0e1e26244c6")


def write_test_session(folder, scans_json, **changes):
    """Write a small Part 1-style session file and return its path."""
    data = {"format": SESSION_FORMAT_NAME, "version": 2, "scans": scans_json}
    data.update(changes)
    path = Path(folder) / "test_session.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def make_scan_json(scan_id, pose, points, visible=True):
    return {"id": scan_id, "name": f"Scan {scan_id}", "visible": visible,
            "aligned": scan_id != 1,
            "pose": {"x_m": pose[0], "y_m": pose[1], "theta_deg": pose[2]},
            "points_local_m": points}


class LoadingTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.folder.cleanup()

    def test_reads_poses_and_original_points(self):
        path = write_test_session(self.folder.name, [
            make_scan_json(1, (0.0, 0.0, 0.0), [[1.0, 0.0], [0.0, 2.0]]),
            make_scan_json(4, (1.5, -2.0, 90.0), [[3.0, 4.0]])])
        scans = load_session_scans(path)
        self.assertEqual([scan["id"] for scan in scans], [1, 4])
        self.assertEqual(scans[1]["pose"], (1.5, -2.0, 90.0))
        np.testing.assert_array_equal(scans[0]["points_local"],
                                      [[1.0, 0.0], [0.0, 2.0]])
        self.assertEqual(scans[0]["points_local"].shape, (2, 2))

    def test_hidden_scans_are_skipped_unless_asked(self):
        path = write_test_session(self.folder.name, [
            make_scan_json(1, (0.0, 0.0, 0.0), [[1.0, 0.0]]),
            make_scan_json(2, (0.0, 0.0, 0.0), [[1.0, 0.0]], visible=False)])
        self.assertEqual(len(load_session_scans(path)), 1)
        self.assertEqual(len(load_session_scans(path, visible_only=False)), 2)

    def test_loading_never_modifies_the_file(self):
        path = write_test_session(self.folder.name, [
            make_scan_json(1, (0.0, 0.0, 0.0), [[1.0, 0.0]])])
        before = file_sha256(path)
        load_session_scans(path)
        self.assertEqual(file_sha256(path), before)

    def test_unusable_files_raise_session_error(self):
        good_scan = make_scan_json(1, (0.0, 0.0, 0.0), [[1.0, 0.0]])
        no_pose = dict(good_scan)
        del no_pose["pose"]
        cases = {
            "wrong format": ([good_scan], {"format": "other"}),
            "old version 1": ([good_scan], {"version": 1}),
            "pose missing": ([no_pose], {}),
            "no points": ([dict(good_scan, points_local_m=[])], {}),
            "bad point": ([dict(good_scan, points_local_m=[[1.0]])], {}),
            "NaN point": ([dict(good_scan,
                                points_local_m=[[float("nan"), 0.0]])], {}),
        }
        for name, (scans_json, changes) in cases.items():
            with self.subTest(name):
                path = write_test_session(self.folder.name, scans_json,
                                          **changes)
                with self.assertRaises(SessionError):
                    load_session_scans(path)
        with self.assertRaises(SessionError):
            load_session_scans(Path(self.folder.name) / "missing.json")
        broken = Path(self.folder.name) / "broken.json"
        broken.write_text("{not json", encoding="utf-8")
        with self.assertRaises(SessionError):
            load_session_scans(broken)

    @unittest.skipUnless(CLASSROOM_FINAL.exists(), "classroom_final.json absent")
    def test_real_classroom_session_loads_read_only(self):
        before = file_sha256(CLASSROOM_FINAL)
        scans = load_session_scans(CLASSROOM_FINAL)
        self.assertGreater(len(scans), 0)
        for scan in scans:
            self.assertEqual(scan["points_local"].shape[1], 2)
            self.assertEqual(len(scan["pose"]), 3)
        self.assertEqual(file_sha256(CLASSROOM_FINAL), before)


class TransformTests(unittest.TestCase):
    def test_identity_pose_changes_nothing(self):
        points = np.array([[1.0, 2.0], [-3.0, 0.5]])
        np.testing.assert_allclose(local_to_global(points, (0, 0, 0)), points)

    def test_translation_only(self):
        np.testing.assert_allclose(
            local_to_global(np.array([[1.0, 2.0]]), (3.0, -1.0, 0.0)),
            [[4.0, 1.0]])

    def test_rotations(self):
        front = np.array([[1.0, 0.0]])  # 1 m in front of the LiDAR
        expected = {90.0: [0.0, 1.0], 180.0: [-1.0, 0.0],
                    -90.0: [0.0, -1.0], 45.0: [math.sqrt(0.5)] * 2}
        for theta, result in expected.items():
            with self.subTest(theta=theta):
                np.testing.assert_allclose(
                    local_to_global(front, (0.0, 0.0, theta)), [result],
                    atol=1e-12)

    def test_rotation_then_translation(self):
        # LiDAR at (2, 1) facing +Y: a point 1 m to its LEFT (local +Y)
        # is at -X of the robot in the classroom: (2 - 1, 1).
        np.testing.assert_allclose(
            local_to_global(np.array([[0.0, 1.0]]), (2.0, 1.0, 90.0)),
            [[1.0, 1.0]], atol=1e-12)

    def test_distances_are_preserved(self):
        points = np.array([[0.0, 0.0], [3.0, 4.0]])
        moved = local_to_global(points, (5.0, -2.0, 37.0))
        self.assertAlmostEqual(np.linalg.norm(moved[1] - moved[0]), 5.0)

    def test_same_transform_as_part_1(self):
        import importlib
        try:
            part1 = importlib.import_module("01_lidar_visualizer")
        except ImportError as error:  # e.g. pyserial missing
            self.skipTest(f"Part 1 not importable: {error}")
        rng = np.random.default_rng(0)
        points = rng.uniform(-6, 6, (100, 2))
        for pose in ((0.1, 2.05, 1.7), (-3.59, -0.22, -88.2),
                     (-0.49, -0.25, 176.3)):
            x_part1, y_part1 = part1.local_to_global(points[:, 0],
                                                     points[:, 1], pose)
            np.testing.assert_allclose(local_to_global(points, pose),
                                       np.column_stack([x_part1, y_part1]),
                                       atol=1e-12)


class GridTests(unittest.TestCase):
    def setUp(self):
        # The course example: 2 m x 3 m, 0.1 m cells -> 20 x 30 cells.
        self.spec = create_grid_spec(0.0, 0.0, 2.0, 3.0, 0.1)

    def test_course_example_size(self):
        self.assertEqual((self.spec["width"], self.spec["height"]), (20, 30))
        grid = create_occupancy_grid(self.spec)
        self.assertEqual(grid.shape, (30, 20))  # (rows = y, columns = x)
        self.assertEqual(grid.dtype, np.uint8)
        self.assertTrue(np.all(grid == UNKNOWN))

    def test_world_to_grid(self):
        cases = {(0.0, 0.0): (0, 0), (0.05, 0.05): (0, 0),
                 (0.1, 0.0): (1, 0), (1.99, 2.99): (19, 29),
                 (0.35, 1.27): (3, 12)}
        for (x, y), expected in cases.items():
            with self.subTest(x=x, y=y):
                i, j = world_to_grid(self.spec, x, y)
                self.assertEqual((int(i), int(j)), expected)

    def test_negative_coordinates(self):
        spec = create_grid_spec(-1.0, -1.0, 1.0, 1.0, 0.1)
        i, j = world_to_grid(spec, -0.05, -0.95)  # classroom x, y < 0
        self.assertEqual((int(i), int(j)), (9, 0))

    def test_points_left_of_the_grid_use_floor(self):
        i, j = world_to_grid(self.spec, -0.05, 1.0)  # 5 cm left of x_min
        self.assertEqual(int(i), -1)  # int(-0.5) would give 0: wrong cell
        self.assertFalse(is_inside_grid(self.spec, i, j))

    def test_cell_centres_round_trip(self):
        i, j = np.meshgrid(np.arange(20), np.arange(30))
        x, y = grid_to_world(self.spec, i, j)
        np.testing.assert_allclose(x[0, :3], [0.05, 0.15, 0.25])
        back_i, back_j = world_to_grid(self.spec, x, y)
        np.testing.assert_array_equal(back_i, i)
        np.testing.assert_array_equal(back_j, j)

    def test_inside_grid(self):
        i, j = world_to_grid(self.spec, np.array([-0.01, 0.5, 2.0, 1.0]),
                             np.array([1.0, 1.0, 1.0, 3.5]))
        np.testing.assert_array_equal(is_inside_grid(self.spec, i, j),
                                      [False, True, False, False])

    def test_bounds_hold_every_point_and_pose_on_round_values(self):
        points = [np.array([[-5.7239, -2.47], [11.91, 10.75]])]
        poses = [(0.0, 0.0, 0.0), (-3.78, -0.31, 2.9)]
        x_min, y_min, x_max, y_max = choose_grid_bounds(points, poses, 0.1,
                                                        0.5)
        self.assertEqual((x_min, y_min, x_max, y_max),
                         (-6.3, -3.0, 12.5, 11.3))
        spec = create_grid_spec(x_min, y_min, x_max, y_max, 0.1)
        self.assertEqual((spec["width"], spec["height"]), (188, 143))
        i, j = world_to_grid(spec, points[0][:, 0], points[0][:, 1])
        self.assertTrue(np.all(is_inside_grid(spec, i, j)))

    def test_invalid_grid_is_refused(self):
        with self.assertRaises(ValueError):
            create_grid_spec(0.0, 0.0, 0.0, 1.0, 0.1)
        with self.assertRaises(ValueError):
            create_grid_spec(0.0, 0.0, 1.0, 1.0, 0.0)

    def test_display_draws_the_grid(self):
        plt.switch_backend("Agg")
        grid = create_occupancy_grid(self.spec)
        figure = show_grid(grid, self.spec)
        image = figure.axes[0].images[0]
        self.assertEqual(list(image.get_extent()), [0.0, 2.0, 0.0, 3.0])
        self.assertEqual(image.get_array().shape, (30, 20))
        plt.close(figure)


class BresenhamTests(unittest.TestCase):
    def test_horizontal_vertical_and_diagonal(self):
        self.assertEqual(bresenham_cells(0, 0, 4, 0),
                         [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)])
        self.assertEqual(bresenham_cells(2, 1, 2, 4),
                         [(2, 1), (2, 2), (2, 3), (2, 4)])
        self.assertEqual(bresenham_cells(0, 0, 3, 3),
                         [(0, 0), (1, 1), (2, 2), (3, 3)])
        self.assertEqual(bresenham_cells(3, 0, 0, 3),  # anti-diagonal
                         [(3, 0), (2, 1), (1, 2), (0, 3)])

    def test_reverse_directions(self):
        self.assertEqual(bresenham_cells(4, 0, 0, 0),
                         [(4, 0), (3, 0), (2, 0), (1, 0), (0, 0)])
        self.assertEqual(bresenham_cells(0, 3, 0, 0),
                         [(0, 3), (0, 2), (0, 1), (0, 0)])

    def test_shallow_and_steep_lines(self):
        self.assertEqual(bresenham_cells(0, 0, 5, 2),  # shallow
                         [(0, 0), (1, 0), (2, 1), (3, 1), (4, 2), (5, 2)])
        self.assertEqual(bresenham_cells(0, 0, 2, 5),  # steep: j every step
                         [(0, 0), (0, 1), (1, 2), (1, 3), (2, 4), (2, 5)])

    def test_same_cell_and_negative_indices(self):
        self.assertEqual(bresenham_cells(7, -3, 7, -3), [(7, -3)])
        self.assertEqual(bresenham_cells(-2, -1, 1, -1),
                         [(-2, -1), (-1, -1), (0, -1), (1, -1)])

    def test_properties_in_every_direction(self):
        rng = np.random.default_rng(1)
        for _ in range(300):
            i0, j0, i1, j1 = (int(value) for value in rng.integers(-40, 40, 4))
            cells = bresenham_cells(i0, j0, i1, j1)
            self.assertEqual(cells[0], (i0, j0))
            self.assertEqual(cells[-1], (i1, j1))
            self.assertEqual(len(cells),
                             max(abs(i1 - i0), abs(j1 - j0)) + 1)
            steps = np.diff(np.array(cells), axis=0)
            self.assertTrue(np.all(np.abs(steps) <= 1))  # 8-connected
            # Each cell stays within half a cell of the ideal straight
            # line, measured along the minor axis.
            di, dj = i1 - i0, j1 - j0
            for i, j in cells:
                if abs(di) >= abs(dj) and di != 0:
                    self.assertLessEqual(abs(j - (j0 + (i - i0) * dj / di)),
                                         0.5 + 1e-9)
                elif dj != 0:
                    self.assertLessEqual(abs(i - (i0 + (j - j0) * di / dj)),
                                         0.5 + 1e-9)


def make_spec_10x10():
    """A 1 m x 1 m grid of 10 cm cells, lower-left corner at (0, 0)."""
    return create_grid_spec(0.0, 0.0, 1.0, 1.0, 0.1)


def single_scan(pose, local_points):
    return {"id": 1, "name": "Scan 1", "pose": pose, "aligned": False,
            "points_local": np.array(local_points, dtype=float), "color": None}


class OccupancyUpdateTests(unittest.TestCase):
    def test_one_ray_classifies_cells(self):
        spec = make_spec_10x10()
        layers = create_observation_layers(spec)
        self.assertTrue(mark_ray(layers, spec, (1, 2), (6, 2)))
        grid = combine_layers(layers)
        np.testing.assert_array_equal(grid[2, 1:6], [FREE] * 5)   # crossed
        self.assertEqual(grid[2, 6], OCCUPIED)                     # impact
        self.assertEqual(count_cells(grid),
                         {"free": 5, "occupied": 1, "unknown": 94})

    def test_ray_inside_one_cell_is_only_occupied(self):
        spec = make_spec_10x10()
        layers = create_observation_layers(spec)
        mark_ray(layers, spec, (4, 4), (4, 4))
        self.assertEqual(count_cells(combine_layers(layers)),
                         {"free": 0, "occupied": 1, "unknown": 99})

    def test_rays_start_at_the_scans_own_pose(self):
        spec = make_spec_10x10()
        scan = single_scan((0.55, 0.35, 90.0), [[0.4, 0.0]])  # facing +Y
        grid, counts = build_binary_occupancy_grid(
            spec, [scan], global_points_of_scans([scan]))
        self.assertEqual(grid[3, 5], FREE)      # the LiDAR's own cell
        self.assertEqual(grid[7, 5], OCCUPIED)  # 0.4 m ahead = y 0.75
        np.testing.assert_array_equal(grid[3:7, 5], [FREE] * 4)
        self.assertEqual(grid[0, 0], UNKNOWN)   # classroom origin untouched
        self.assertEqual(counts["rays"], 1)

    def test_obstacles_win_over_later_free_rays_in_any_order(self):
        spec = make_spec_10x10()
        # Scan A hits a wall cell; scan B's beam passes through that cell.
        scan_a = single_scan((0.05, 0.55, 0.0), [[0.4, 0.0]])   # ends (4, 5)
        scan_b = single_scan((0.05, 0.55, 0.0), [[0.8, 0.0]])   # crosses it
        grids = []
        for order in ([scan_a, scan_b], [scan_b, scan_a]):
            grid, counts = build_binary_occupancy_grid(
                spec, order, global_points_of_scans(order))
            grids.append(grid)
            self.assertEqual(counts["free_and_occupied"], 1)
        np.testing.assert_array_equal(grids[0], grids[1])
        self.assertEqual(grids[0][5, 4], OCCUPIED)
        self.assertEqual(grids[0][5, 8], OCCUPIED)
        np.testing.assert_array_equal(grids[0][5, [0, 1, 2, 3, 5, 6, 7]],
                                      [FREE] * 7)

    def test_overlapping_free_rays_stay_free(self):
        spec = make_spec_10x10()
        layers = create_observation_layers(spec)
        mark_ray(layers, spec, (0, 0), (9, 9))
        mark_ray(layers, spec, (0, 0), (9, 9))  # same beam twice
        mark_ray(layers, spec, (9, 0), (0, 9))  # crosses it
        grid = combine_layers(layers)
        self.assertEqual(grid[5, 5], FREE)
        self.assertEqual(grid[4, 4], FREE)
        self.assertEqual(count_cells(grid)["occupied"], 2)

    def test_impact_outside_the_grid_frees_the_inside_part(self):
        spec = make_spec_10x10()
        layers = create_observation_layers(spec)
        self.assertFalse(mark_ray(layers, spec, (2, 5), (25, 5)))  # x 2.5 m
        grid = combine_layers(layers)
        np.testing.assert_array_equal(grid[5, 2:], [FREE] * 8)
        self.assertEqual(count_cells(grid)["occupied"], 0)

    def test_sensor_outside_the_grid_and_negative_coordinates(self):
        spec = create_grid_spec(-1.0, -1.0, 0.0, 0.0, 0.1)  # all negative
        layers = create_observation_layers(spec)
        start = world_to_grid(spec, -1.55, -0.45)  # 0.55 m LEFT of the grid
        end = world_to_grid(spec, -0.25, -0.45)
        self.assertTrue(mark_ray(layers, spec, start, end))
        grid = combine_layers(layers)
        np.testing.assert_array_equal(grid[5, 0:7], [FREE] * 7)
        self.assertEqual(grid[5, 7], OCCUPIED)

    def test_points_beyond_lidar_range_are_ignored(self):
        spec = create_grid_spec(0.0, 0.0, 20.0, 1.0, 0.1)
        scan = single_scan((0.05, 0.55, 0.0), [[5.0, 0.0], [15.0, 0.0]])
        grid, counts = build_binary_occupancy_grid(
            spec, [scan], global_points_of_scans([scan]), max_range_m=12.0)
        self.assertEqual((counts["rays"], counts["too_far"]), (1, 1))
        self.assertEqual(count_cells(grid)["occupied"], 1)
        self.assertEqual(grid[5, 100], UNKNOWN)  # the far beam left no trace

    def test_scan_rotation_is_used_for_the_rays(self):
        spec = make_spec_10x10()
        # LiDAR at the centre facing -X (180 deg): local "front" is -X.
        scan = single_scan((0.55, 0.55, 180.0), [[0.3, 0.0]])
        grid, _ = build_binary_occupancy_grid(
            spec, [scan], global_points_of_scans([scan]))
        self.assertEqual(grid[5, 2], OCCUPIED)       # x = 0.25 m
        np.testing.assert_array_equal(grid[5, 3:6], [FREE] * 3)
        self.assertEqual(grid[5, 8], UNKNOWN)

    def test_display_without_points_keeps_poses(self):
        plt.switch_backend("Agg")
        spec = make_spec_10x10()
        scan = single_scan((0.55, 0.55, 0.0), [[0.3, 0.0]])
        points = global_points_of_scans([scan])
        grid, _ = build_binary_occupancy_grid(spec, [scan], points)
        for show_points, expected_scatters in ((True, 1), (False, 0)):
            figure = show_grid(grid, spec, [scan], points,
                               show_points=show_points)
            axes = figure.axes[0]
            self.assertEqual(len(axes.collections), expected_scatters)
            self.assertEqual(len(axes.lines), 1)  # the pose dot
            plt.close(figure)

    @unittest.skipUnless(CLASSROOM_FINAL.exists(), "classroom_final.json absent")
    def test_real_classroom_map(self):
        before = file_sha256(CLASSROOM_FINAL)
        scans = load_session_scans(CLASSROOM_FINAL)
        points = global_points_of_scans(scans)
        spec = create_grid_spec(*choose_grid_bounds(
            points, [scan["pose"] for scan in scans]), CELL_SIZE_M)
        grid, counts = build_binary_occupancy_grid(spec, scans, points)
        cells = count_cells(grid)
        self.assertEqual(sum(cells.values()), grid.size)  # only 0/127/255
        self.assertGreater(cells["free"], cells["occupied"])
        self.assertGreater(cells["occupied"], 0)
        self.assertEqual(counts["impact_outside"], 0)     # automatic bounds
        for scan in scans:  # every LiDAR stood in a free (or hit) cell
            i, j = world_to_grid(spec, *scan["pose"][:2])
            self.assertIn(grid[j, i], (FREE, OCCUPIED))
        self.assertEqual(file_sha256(CLASSROOM_FINAL), before)


def build_map_from(scans, spec=None, **options):
    """Probabilistic map of test scans on a 1 m x 1 m grid by default."""
    spec = spec or make_spec_10x10()
    return build_probabilistic_occupancy_map(
        spec, scans, global_points_of_scans(scans), **options)[0]


# A scan at (0.05, 0.55) facing +X whose beam ends 0.4 m ahead: crosses
# cells (0..3, 5) and hits cell (4, 5). A longer one crosses (4, 5).
SHORT_BEAM = ((0.05, 0.55, 0.0), [[0.4, 0.0]])
LONG_BEAM = ((0.05, 0.55, 0.0), [[0.8, 0.0]])


class LogOddsTests(unittest.TestCase):
    def test_probability_log_odds_conversions(self):
        self.assertEqual(float(probability_to_log_odds(0.5)), 0.0)
        self.assertAlmostEqual(float(probability_to_log_odds(0.7)),
                               math.log(0.7 / 0.3))
        self.assertAlmostEqual(float(log_odds_to_probability(0.0)), 0.5)
        probabilities = np.array([0.01, 0.12, 0.4, 0.5, 0.7, 0.97, 0.99])
        np.testing.assert_allclose(
            log_odds_to_probability(probability_to_log_odds(probabilities)),
            probabilities)
        # Symmetry: "free with p" is the opposite of "occupied with p".
        self.assertAlmostEqual(float(probability_to_log_odds(0.3)),
                               -float(probability_to_log_odds(0.7)))
        self.assertAlmostEqual(L_HIT, 0.8473, places=4)
        self.assertAlmostEqual(L_MISS, -0.4055, places=4)
        self.assertEqual(L_PRIOR, 0.0)

    def test_update_is_clamped(self):
        log_odds = np.zeros((1, 3))
        cells = np.array([[True, True, False]])
        for _ in range(10):
            update_log_odds(log_odds, cells, L_HIT)
        np.testing.assert_allclose(log_odds, [[L_MAX, L_MAX, 0.0]])
        for _ in range(30):
            update_log_odds(log_odds, cells, L_MISS)
        np.testing.assert_allclose(log_odds, [[L_MIN, L_MIN, 0.0]])
        self.assertAlmostEqual(float(log_odds_to_probability(L_MAX)), P_MAX)
        self.assertAlmostEqual(float(log_odds_to_probability(L_MIN)), P_MIN)

    def test_probabilities_to_course_values(self):
        np.testing.assert_array_equal(
            probabilities_to_uint8(np.array([0.0, 0.12, 0.5, 0.97, 1.0])),
            [0, 30, 127, 247, 255])

    def test_single_ray_hit_and_miss(self):
        log_odds_map = build_map_from([single_scan(*SHORT_BEAM)])
        log_odds = log_odds_map["log_odds"]
        self.assertAlmostEqual(log_odds[5, 4], L_HIT)               # impact
        np.testing.assert_allclose(log_odds[5, 0:4], [L_MISS] * 4)  # crossed
        self.assertEqual(log_odds_map["hits"][5, 4], 1)
        np.testing.assert_array_equal(log_odds_map["misses"][5, 0:4], [1] * 4)

    def test_unobserved_cells_keep_the_prior(self):
        log_odds_map = build_map_from([single_scan(*SHORT_BEAM)])
        probabilities = log_odds_to_probability(log_odds_map["log_odds"])
        untouched = np.ones((10, 10), dtype=bool)
        untouched[5, 0:5] = False
        self.assertTrue(np.all(probabilities[untouched] == 0.5))
        self.assertTrue(np.all(probabilities_to_uint8(
            probabilities[untouched]) == UNKNOWN))
        self.assertEqual(classify_probabilities(log_odds_map)["unobserved"],
                         95)

    def test_repeated_hits_and_misses_accumulate_then_saturate(self):
        for repeats, expected_hit, expected_miss in (
                (1, L_HIT, L_MISS), (3, 3 * L_HIT, 3 * L_MISS),
                (5, L_MAX, L_MIN), (12, L_MAX, L_MIN)):
            with self.subTest(repeats=repeats):
                scans = [single_scan(*SHORT_BEAM)] * repeats
                log_odds = build_map_from(scans)["log_odds"]
                self.assertAlmostEqual(log_odds[5, 4], expected_hit)
                self.assertAlmostEqual(log_odds[5, 0], expected_miss)

    def test_one_scan_counts_once_per_cell(self):
        # Two points of ONE scan in the same cell, and many beams of ONE
        # scan through the LiDAR's cell: one hit, one miss, not several.
        scan = single_scan((0.05, 0.55, 0.0),
                           [[0.41, 0.0], [0.43, 0.01], [0.0, 0.4],
                            [0.0, -0.4], [-0.0, 0.3]])
        log_odds_map = build_map_from([scan])
        self.assertAlmostEqual(log_odds_map["log_odds"][5, 4], L_HIT)
        self.assertAlmostEqual(log_odds_map["log_odds"][5, 0], L_MISS)
        self.assertEqual(log_odds_map["misses"][5, 0], 1)

    def test_conflicting_evidence_is_weighed(self):
        # Scan A hits cell (4, 5); other scans' beams pass through it.
        hit = single_scan(*SHORT_BEAM)
        crossing = single_scan(*LONG_BEAM)
        expectations = {1: ("uncertain", L_HIT + L_MISS),
                        2: ("uncertain", L_HIT + 2 * L_MISS),
                        4: ("free", L_HIT + 4 * L_MISS)}
        for crossings, (expected_class, expected_log_odds) in \
                expectations.items():
            with self.subTest(crossings=crossings):
                log_odds_map = build_map_from([hit] + [crossing] * crossings)
                value = log_odds_map["log_odds"][5, 4]
                self.assertAlmostEqual(value, expected_log_odds)
                probability = float(log_odds_to_probability(value))
                if expected_class == "free":
                    self.assertLessEqual(probability, P_FREE_BELOW)
                else:
                    self.assertTrue(P_FREE_BELOW < probability
                                    < P_OCCUPIED_ABOVE)
                self.assertEqual(
                    classify_probabilities(log_odds_map)["conflicting"], 1)
        # The binary map keeps the same cell occupied whatever happens.
        spec = make_spec_10x10()
        scans = [hit] + [crossing] * 4
        binary, _ = build_binary_occupancy_grid(
            spec, scans, global_points_of_scans(scans))
        self.assertEqual(binary[5, 4], OCCUPIED)

    def test_two_hits_resist_one_miss(self):
        hit, crossing = single_scan(*SHORT_BEAM), single_scan(*LONG_BEAM)
        value = build_map_from([hit, hit, crossing])["log_odds"][5, 4]
        self.assertAlmostEqual(value, 2 * L_HIT + L_MISS)
        self.assertGreaterEqual(float(log_odds_to_probability(value)),
                                P_OCCUPIED_ABOVE)

    def test_scan_order_independence(self):
        hit, crossing = single_scan(*SHORT_BEAM), single_scan(*LONG_BEAM)
        scans = [hit] * 5 + [crossing]  # reaches the limit on the way
        forward = build_map_from(scans)["log_odds"]
        backward = build_map_from(scans[::-1])["log_odds"]
        np.testing.assert_allclose(forward, backward, atol=1e-12)
        self.assertAlmostEqual(forward[5, 4],
                               min(5 * L_HIT + L_MISS, L_MAX))

    def test_clamping_after_every_scan_depends_on_order(self):
        hit, crossing = single_scan(*SHORT_BEAM), single_scan(*LONG_BEAM)
        hits_first = build_map_from([hit] * 5 + [crossing],
                                    clamp_each_scan=True)["log_odds"][5, 4]
        miss_first = build_map_from([crossing] + [hit] * 5,
                                    clamp_each_scan=True)["log_odds"][5, 4]
        self.assertAlmostEqual(hits_first, L_MAX + L_MISS)
        self.assertAlmostEqual(miss_first, L_MAX)

    def test_display_labels_the_probabilistic_map(self):
        plt.switch_backend("Agg")
        log_odds_map = build_map_from([single_scan(*SHORT_BEAM)])
        grid = probabilities_to_uint8(
            log_odds_to_probability(log_odds_map["log_odds"]))
        figure = show_grid(grid, make_spec_10x10(), title="PROBABILISTIC",
                           colorbar_labels=PROBABILITY_COLORBAR_LABELS)
        labels = [label.get_text()
                  for label in figure.axes[1].get_yticklabels()]
        self.assertEqual(labels, list(PROBABILITY_COLORBAR_LABELS))
        plt.close(figure)


class GridModeTests(unittest.TestCase):
    def test_invalid_grid_to_show_is_a_clear_error(self):
        module = sys.modules[__name__]
        for value in ("probablistic", "binary", "", None):
            with self.subTest(value=value):
                with self.assertRaises(ValueError) as context:
                    check_grid_to_show(value)
                self.assertIn("probabilistic", str(context.exception))
                with mock.patch.object(module, "GRID_TO_SHOW", value):
                    with self.assertRaises(ValueError):
                        main()  # refused before loading or drawing anything
        for value in VALID_GRIDS_TO_SHOW:
            check_grid_to_show(value)  # no error

    @unittest.skipUnless(CLASSROOM_FINAL.exists(), "classroom_final.json absent")
    def test_every_mode_runs_and_titles_say_which_map(self):
        plt.switch_backend("Agg")
        module = sys.modules[__name__]
        expected_titles = {"empty": "Occupancy grid 188 x 143",
                           "occupancy": "BINARY",
                           "probabilistic": "PROBABILISTIC"}
        for mode, expected in expected_titles.items():
            with self.subTest(mode=mode), \
                    mock.patch.object(module, "GRID_TO_SHOW", mode), \
                    mock.patch.object(module, "SESSION_FILE",
                                      "classroom_final.json"), \
                    mock.patch.object(plt, "show"), \
                    contextlib.redirect_stdout(io.StringIO()):
                main()
                self.assertIn(expected, plt.gcf().axes[0].get_title())
                plt.close("all")


@unittest.skipUnless(CLASSROOM_FINAL.exists(), "classroom_final.json absent")
class ClassroomProbabilisticTests(unittest.TestCase):
    """All 16 real scans (read-only)."""

    @classmethod
    def setUpClass(cls):
        cls.fingerprint = file_sha256(CLASSROOM_FINAL)
        cls.scans = load_session_scans(CLASSROOM_FINAL)
        cls.points = global_points_of_scans(cls.scans)
        cls.spec = create_grid_spec(*choose_grid_bounds(
            cls.points, [scan["pose"] for scan in cls.scans]), CELL_SIZE_M)
        cls.log_odds_map, cls.counts = build_probabilistic_occupancy_map(
            cls.spec, cls.scans, cls.points)

    def test_all_scans_build_a_consistent_map(self):
        self.assertEqual(len(self.scans), 16)
        self.assertEqual(self.counts["rays"], 9712)
        classes = classify_probabilities(self.log_odds_map)
        self.assertEqual(sum(classes[key] for key in
                             ("free", "occupied", "uncertain", "unobserved")),
                         self.log_odds_map["log_odds"].size)
        self.assertGreater(classes["free"], classes["occupied"])
        log_odds = self.log_odds_map["log_odds"]
        self.assertTrue(np.all((log_odds >= L_MIN) & (log_odds <= L_MAX)))
        # The same cells are observed as in the binary map.
        binary, _ = build_binary_occupancy_grid(self.spec, self.scans,
                                                self.points)
        self.assertEqual(classes["unobserved"],
                         count_cells(binary)["unknown"])
        observed = (self.log_odds_map["hits"]
                    + self.log_odds_map["misses"]) > 0
        np.testing.assert_array_equal(observed, binary != UNKNOWN)

    def test_conflicts_no_longer_always_occupied(self):
        classes = classify_probabilities(self.log_odds_map)
        self.assertGreater(classes["conflicting"], 0)
        self.assertGreater(classes["conflicting_free"]
                           + classes["conflicting_uncertain"], 0)

    def test_any_scan_order_gives_the_same_map(self):
        rng = np.random.default_rng(7)
        for _ in range(3):
            order = rng.permutation(len(self.scans))
            shuffled, _ = build_probabilistic_occupancy_map(
                self.spec, [self.scans[k] for k in order],
                [self.points[k] for k in order])
            np.testing.assert_allclose(shuffled["log_odds"],
                                       self.log_odds_map["log_odds"],
                                       atol=1e-9)

    def test_binary_map_is_unchanged(self):
        binary, counts = build_binary_occupancy_grid(self.spec, self.scans,
                                                     self.points)
        self.assertEqual(count_cells(binary),
                         {"free": 9898, "occupied": 2179, "unknown": 14807})
        self.assertEqual(counts["rays"], 9712)
        self.assertEqual(counts["free_and_occupied"], 1661)

    def test_session_file_unchanged(self):
        self.assertEqual(file_sha256(CLASSROOM_FINAL), self.fingerprint)

    def test_results_identical_to_the_verified_maps(self):
        binary, _ = build_binary_occupancy_grid(self.spec, self.scans,
                                                self.points)
        self.assertEqual(hashlib.sha256(binary.tobytes()).hexdigest(),
                         VERIFIED_BINARY_SHA256)
        self.assertEqual(hashlib.sha256(
            self.log_odds_map["log_odds"].tobytes()).hexdigest(),
            VERIFIED_LOG_ODDS_SHA256)
        classes = classify_probabilities(self.log_odds_map)
        self.assertEqual(
            {key: classes[key] for key in ("free", "occupied", "uncertain",
                                           "unobserved", "conflicting")},
            {"free": 9143, "occupied": 1082, "uncertain": 1852,
             "unobserved": 14807, "conflicting": 1517})


def make_room_session_json(scan_count, seed=0):
    """A valid session with `scan_count` scans of a 6 m x 4 m room."""
    rng = np.random.default_rng(seed)
    scans_json = []
    for scan_id in range(1, scan_count + 1):
        x_r, y_r = rng.uniform(1.0, 5.0), rng.uniform(1.0, 3.0)
        theta = float(rng.uniform(-180.0, 180.0))
        # Points on the room's walls, expressed in this scan's own frame.
        angles = np.radians(np.arange(0.0, 360.0, 3.0))
        reach = np.minimum.reduce([
            np.where(np.cos(angles) > 0, (6.0 - x_r) / np.cos(angles), np.inf),
            np.where(np.cos(angles) < 0, -x_r / np.cos(angles), np.inf),
            np.where(np.sin(angles) > 0, (4.0 - y_r) / np.sin(angles), np.inf),
            np.where(np.sin(angles) < 0, -y_r / np.sin(angles), np.inf)])
        global_points = np.column_stack([x_r + reach * np.cos(angles),
                                         y_r + reach * np.sin(angles)])
        rotation = rotation_matrix(theta)
        local = (global_points - [x_r, y_r]) @ rotation  # inverse transform
        scans_json.append(make_scan_json(scan_id, (x_r, y_r, theta),
                                         local.round(6).tolist()))
    return scans_json


class SessionSelectionTests(unittest.TestCase):
    """SESSION_FILE: name in scans/, absolute path, missing file, any size."""

    def setUp(self):
        plt.switch_backend("Agg")
        self.folder = tempfile.TemporaryDirectory()
        self.module = sys.modules[__name__]

    def tearDown(self):
        plt.close("all")
        self.folder.cleanup()

    def run_main(self, session_file, mode="probabilistic", scans_dir=None):
        """Run main() off-screen; return (console text, map title or None)."""
        output = io.StringIO()
        with mock.patch.object(self.module, "SESSION_FILE", session_file), \
                mock.patch.object(self.module, "GRID_TO_SHOW", mode), \
                mock.patch.object(self.module, "SCANS_DIR",
                                  Path(scans_dir) if scans_dir else SCANS_DIR), \
                mock.patch.object(plt, "show"), \
                contextlib.redirect_stdout(output):
            main()
        title = (plt.gcf().axes[0].get_title() if plt.get_fignums()
                 else None)
        plt.close("all")
        return output.getvalue(), title

    def test_names_are_looked_for_next_to_the_program(self):
        self.assertEqual(resolve_session_path("classroom_final.json"),
                         SCANS_DIR / "classroom_final.json")
        self.assertEqual(resolve_session_path("new_room"),
                         SCANS_DIR / "new_room.json")
        self.assertTrue(SCANS_DIR.is_absolute())
        original_folder = Path.cwd()
        try:
            os.chdir(self.folder.name)  # like starting from another folder
            self.assertEqual(resolve_session_path("classroom_final.json"),
                             SCANS_DIR / "classroom_final.json")
        finally:
            os.chdir(original_folder)

    def test_absolute_paths_are_used_as_given(self):
        path = Path(self.folder.name) / "elsewhere.json"
        self.assertEqual(resolve_session_path(str(path)), path)
        path.write_text(json.dumps({"format": SESSION_FORMAT_NAME,
                                    "version": 2,
                                    "scans": make_room_session_json(3)}),
                        encoding="utf-8")
        output, title = self.run_main(str(path))
        self.assertIn("3 scans", output)
        self.assertIn("PROBABILISTIC", title)

    @unittest.skipUnless(CLASSROOM_FINAL.exists(), "classroom_final.json absent")
    def test_default_setting_loads_classroom_final(self):
        self.assertEqual(resolve_session_path(SESSION_FILE), CLASSROOM_FINAL)
        before = file_sha256(CLASSROOM_FINAL)
        output, title = self.run_main("classroom_final.json")
        self.assertIn(str(CLASSROOM_FINAL), output)
        self.assertIn("16 scans", output)
        self.assertIn("16 scans", title)
        self.assertEqual(file_sha256(CLASSROOM_FINAL), before)

    def test_another_file_name_in_scans_folder(self):
        write_test_session(self.folder.name, make_room_session_json(5))
        output, title = self.run_main("test_session.json",
                                      scans_dir=self.folder.name)
        self.assertIn("5 scans", output)
        self.assertIn("5 scans", title)

    @unittest.skipUnless((SCANS_DIR / "classroom_session.json").exists(),
                         "classroom_session.json absent")
    def test_the_other_real_session_can_be_selected(self):
        before = file_sha256(SCANS_DIR / "classroom_session.json")
        output, title = self.run_main("classroom_session.json")
        self.assertIn("classroom_session.json", output)
        self.assertIsNotNone(title)
        self.assertEqual(file_sha256(SCANS_DIR / "classroom_session.json"),
                         before)

    def test_missing_file_gives_a_clear_message(self):
        output, title = self.run_main("no_such_room.json")
        self.assertIn("Session file not found", output)
        self.assertIn(str(SCANS_DIR / "no_such_room.json"), output)
        self.assertIsNone(title)  # nothing drawn
        absolute = str(Path(self.folder.name) / "missing.json")
        output, _ = self.run_main(absolute)
        self.assertIn(absolute, output)

    def test_any_number_of_scans_in_every_mode(self):
        for scan_count in (1, 2, 7, 30):
            path = write_test_session(self.folder.name,
                                      make_room_session_json(scan_count))
            for mode in VALID_GRIDS_TO_SHOW:
                with self.subTest(scans=scan_count, mode=mode):
                    output, title = self.run_main(str(path), mode)
                    self.assertIn(f"{scan_count} scans", output)
                    self.assertIsNotNone(title)

    def test_session_without_visible_scans(self):
        hidden = [dict(scan, visible=False)
                  for scan in make_room_session_json(2)]
        path = write_test_session(self.folder.name, hidden)
        output, title = self.run_main(str(path))
        self.assertIn("no visible scan", output)
        self.assertIsNone(title)

    @unittest.skipUnless(CLASSROOM_FINAL.exists(), "classroom_final.json absent")
    def test_program_runs_as_a_script_from_another_folder(self):
        # What PyCharm does: start the file as a program. Here from an
        # unrelated working folder, with an off-screen Matplotlib backend.
        result = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), "--non-interactive"],
            cwd=self.folder.name, capture_output=True, text=True,
            env=dict(os.environ, MPLBACKEND="Agg"), timeout=120)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("16 scans", result.stdout)
        self.assertIn("Session file unchanged: True", result.stdout)

def cli(argv=None):
    global SESSION_FILE
    parser = argparse.ArgumentParser(description="Occupancy grid mapping")
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
