"""Day 2, Phase 2 - LiDAR localisation on the Code 03 VECTOR map.

Given the merged line segments of a room (Code 03, unchanged), one LiDAR
scan in its own sensor frame and NO initial pose, find the pose
(x, y, theta) at which the scan's points lie on the map's segments.

Different from Code 04 (occupancy grid): the reference geometry is a list
of FINITE segments [A, B]; a point's error is its Euclidean distance to the
closest point of the closest segment (perpendicular inside the segment,
distance to the endpoint beyond it). Nothing is rasterised from Code 02.

Method (section 3):
    1. global candidates - every plausible position and every heading,
       scored with a robust kernel of the point-to-segment distance read
       from a precomputed table (exact distances to the segments, sampled
       every 5 cm: an acceleration structure built from the vectors);
    2. refinement - for each distinct candidate, a finer table search, then
       Gauss-Newton point-to-finite-segment ICP on EXACT distances;
    3. verdict - robust score, inliers, residual, supporting segments,
       constraint in two directions, and the margin over the best clearly
       different pose (section 4). Local mode = steps 1-3 inside a window
       around a user guess.

Coordinates: the map frame of the session = Day 1 frame, Scan 1 at
(0, 0, 0 deg). Points stay in the sensor frame until a pose is applied;
a localised scan is never added to the map.

Run this file in PyCharm (RUN_MODE below):
    "application" - unified desktop app, Vector Localisation page
    "evaluation"  - real-data benchmark (leave-one-out, Code 04
                    comparison); results in results/phase2_vector_*
Tests: python -m unittest 05_vector_localisation -v
"""
import csv
import importlib
import json
import math
import sys
import time
import unittest
import unittest.mock
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np

PROJECT_DIR = Path(__file__).resolve().parent
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))
import localisation_common as common  # noqa: E402
from localisation_common import (  # noqa: E402
    LocalisationResult, angular_coverage_deg, filter_ranges, pose_from_day1,
    pose_to_day1, rotation, transform_points, wrap_angle)

grid_loc = importlib.import_module("04_grid_localisation")  # search helpers

# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------
RUN_MODE = "application"       # "application" or "evaluation"
SESSION_FILE = "classroom_final.json"
ASK_FOR_SESSION_FILE = True
RESULTS_FILE_STEM = "phase2_vector_localisation"


# ---------------------------------------------------------------------------
# 1. The vector map: finite segments + a distance table
# ---------------------------------------------------------------------------
def point_segment_distances(points, starts, ends):
    """Distances (N, M) from points (N, 2) to finite segments [starts, ends].

    t = clip(((p - A) . (B - A)) / |B - A|^2, 0, 1) is the closest point's
    position along the segment: 0 = endpoint A, 1 = endpoint B. Beyond an
    end the distance is to that endpoint, never to the infinite line.
    """
    ab = ends - starts
    length2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
    ap = points[:, None, :] - starts[None, :, :]
    t = np.clip((ap * ab[None]).sum(axis=2) / length2[None], 0.0, 1.0)
    closest = starts[None] + t[..., None] * ab[None]
    return np.hypot(points[:, None, 0] - closest[..., 0],
                    points[:, None, 1] - closest[..., 1])


def closest_on_segments(points, starts, ends):
    """For each point: (closest point (N, 2), distance (N,), segment (N,)).

    Chunked so that memory stays small for big point sets.
    """
    ab = ends - starts
    length2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
    closest = np.empty((len(points), 2))
    distance = np.empty(len(points))
    index = np.empty(len(points), dtype=np.int64)
    for first in range(0, len(points), 2048):
        chunk = points[first:first + 2048]
        ap = chunk[:, None, :] - starts[None]
        t = np.clip((ap * ab[None]).sum(axis=2) / length2[None], 0.0, 1.0)
        candidates = starts[None] + t[..., None] * ab[None]
        d = np.hypot(chunk[:, None, 0] - candidates[..., 0],
                     chunk[:, None, 1] - candidates[..., 1])
        best = d.argmin(axis=1)
        rows = np.arange(len(chunk))
        closest[first:first + len(chunk)] = candidates[rows, best]
        distance[first:first + len(chunk)] = d[rows, best]
        index[first:first + len(chunk)] = best
    return closest, distance, index


@dataclass
class VectorMap:
    """Finite segments of the map and a table of distances to them.

    starts, ends - (M, 2) segment endpoints in the map frame (metres)
    support      - (M,) number of scans that observed each segment
    distance     - (H, W) exact distance from each table cell CENTRE to the
                   nearest segment; cell (j, i) covers x_min + i*cell ...
    The table only speeds up candidate generation; refinement and every
    reported number use point_segment_distances() directly.
    """
    starts: np.ndarray
    ends: np.ndarray
    support: np.ndarray
    distance: np.ndarray
    x_min: float
    y_min: float
    cell: float

    @classmethod
    def from_merged(cls, merged, cell=0.05, margin=1.0):
        """Build from Code 03's merge_segments() output (unchanged)."""
        if not merged:
            raise ValueError("the vector map has no segments")
        starts = np.array([np.asarray(s["start"], float) for s in merged])
        ends = np.array([np.asarray(s["end"], float) for s in merged])
        support = np.array([int(s.get("support", 1)) for s in merged])
        low = np.minimum(starts, ends).min(axis=0) - margin
        high = np.maximum(starts, ends).max(axis=0) + margin
        low = np.floor(low / cell) * cell
        width, height = np.ceil((high - low) / cell).astype(int)
        xs = low[0] + (np.arange(width) + 0.5) * cell
        ys = low[1] + (np.arange(height) + 0.5) * cell
        centres = np.column_stack([np.repeat(xs[None], height, 0).ravel(),
                                   np.repeat(ys[:, None], width, 1).ravel()])
        distance = np.empty(len(centres))
        for first in range(0, len(centres), 4096):
            distance[first:first + 4096] = point_segment_distances(
                centres[first:first + 4096], starts, ends).min(axis=1)
        return cls(starts=starts, ends=ends, support=support,
                   distance=distance.reshape(height, width),
                   x_min=float(low[0]), y_min=float(low[1]), cell=cell)

    @property
    def extent(self):
        height, width = self.distance.shape
        return (self.x_min, self.x_min + width * self.cell,
                self.y_min, self.y_min + height * self.cell)

    @property
    def segment_extent(self):
        low = np.minimum(self.starts, self.ends).min(axis=0)
        high = np.maximum(self.starts, self.ends).max(axis=0)
        return float(low[0]), float(high[0]), float(low[1]), float(high[1])


def kernel(distance, tau):
    """Robust match value of a point: 1 on a segment, 0 beyond tau.

    k(d) = max(0, 1 - (d / tau)^2). Bounded: a point far from every
    segment (a chair the map does not hold) costs the same as any other
    unmatched point, so clutter cannot pull the pose.
    """
    return np.maximum(0.0, 1.0 - (np.asarray(distance) / tau) ** 2)


def exact_score(vmap, points, pose, tau):
    """Mean kernel of the EXACT point-to-segment distances at `pose`."""
    placed = transform_points(points, pose)
    return float(kernel(point_segment_distances(
        placed, vmap.starts, vmap.ends).min(axis=1), tau).mean())


# ---------------------------------------------------------------------------
# 2. Search
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class VectorSearchConfig:
    coarse_step_m: float = 0.20     # level-0 positions
    coarse_step_deg: float = 3.0    # level-0 headings (360 deg in global)
    coarse_tau_m: float = 0.30      # wide kernel: wide basin
    fine_step_m: float = 0.05       # level 1: +-1 coarse step around each
    fine_step_deg: float = 1.0      #   candidate
    fine_tau_m: float = 0.15
    tau_m: float = 0.10             # final score kernel
    inlier_m: float = 0.08          # "on a segment" (Code 01's 8 cm)
    icp_iterations: int = 30
    icp_start_m: float = 0.30       # association limit, shrinks to inlier_m
    candidates: int = 12
    distinct_m: float = 0.30        # two poses differ if position OR
    distinct_deg: float = 10.0      # heading differ more than this
    position_margin_m: float = 0.5  # positions within the map's extent
    min_wall_clearance_m: float = 0.10  # the sensor is not inside a wall
    xy_window_m: float = 1.0        # local mode window (as Code 04)
    theta_window_deg: float = 30.0
    coarse_voxel_m: float = 0.10
    voxel_m: float = 0.05
    min_range_m: float = common.MIN_RANGE_M
    max_range_m: float = common.MAX_RANGE_M
    min_points: int = 30
    min_coverage_deg: float = 90.0
    heading_batch: int = 8          # headings per progress / cancel check
    # Verdict (section 4). Calibrated on real classroom data: 21 true poses
    # (leave-one-out on rebuilt maps, other-day scans) against 74 wrong
    # results (deliberately wrong starts, mirror-image scans, occluded
    # scans): 0 wrong results reach "ok"; 12 of 21 true poses do, the
    # others are "weak" (check visually). Margins: the closest wrong result
    # passing every other test has a runner-up ratio of 0.82 (> ok_ratio);
    # true scores >= 0.364, unsupported wrong answers <= 0.269 (min_score).
    min_score: float = 0.30
    ok_score: float = 0.35
    ok_inliers: float = 0.45
    ok_constraint: float = 0.20
    ok_segments: int = 4
    ok_ratio: float = 0.75
    ambiguity_ratio: float = 0.92

    def to_dict(self):
        return asdict(self)


class SearchCancelled(Exception):
    """Raised inside a search when should_stop() returns True."""


def plausible_positions(vmap, step, margin, clearance):
    """Lattice positions inside the segments' extent (+ margin), not on a
    segment. Positions lie on table-cell borders (integer lookups)."""
    left, right, bottom, top = vmap.segment_extent
    ratio = int(round(step / vmap.cell))
    i0 = int(np.ceil((left - margin - vmap.x_min) / vmap.cell))
    i1 = int(np.floor((right + margin - vmap.x_min) / vmap.cell))
    j0 = int(np.ceil((bottom - margin - vmap.y_min) / vmap.cell))
    j1 = int(np.floor((top + margin - vmap.y_min) / vmap.cell))
    height, width = vmap.distance.shape
    i = np.arange(max(i0, 1), min(i1, width - 1) + 1, ratio)
    j = np.arange(max(j0, 1), min(j1, height - 1) + 1, ratio)
    grid_i, grid_j = np.meshgrid(i, j)
    grid_i, grid_j = grid_i.ravel(), grid_j.ravel()
    # A border is shared by four cells: clear of walls if all four are.
    near = np.minimum.reduce([vmap.distance[grid_j - a, grid_i - b]
                              for a in (0, 1) for b in (0, 1)])
    keep = near >= clearance
    return np.column_stack([vmap.x_min + grid_i[keep] * vmap.cell,
                            vmap.y_min + grid_j[keep] * vmap.cell])


def table_scores(points, table, vmap, positions, headings):
    """scores[h, p]: mean table value under the points (nearest cell).

    Positions lie on cell borders, so each point's cell is the position's
    cell plus the point's own offset (integer additions, as Code 04's
    coarse level). Beyond the table the value is 0 (no segment near).
    """
    pad = int(math.ceil(np.max(np.hypot(points[:, 0], points[:, 1]))
                        / vmap.cell)) + 2
    padded = np.pad(table, pad)
    pi = np.rint((positions[:, 0] - vmap.x_min) / vmap.cell).astype(np.int64) + pad
    pj = np.rint((positions[:, 1] - vmap.y_min) / vmap.cell).astype(np.int64) + pad
    scores = np.empty((len(headings), len(positions)))
    for index, theta in enumerate(headings):
        rotated = points @ rotation(theta).T
        oi = np.floor(rotated[:, 0] / vmap.cell).astype(np.int64)
        oj = np.floor(rotated[:, 1] / vmap.cell).astype(np.int64)
        scores[index] = padded[pj[:, None] + oj[None], pi[:, None] + oi[None]
                               ].mean(axis=1)
    return scores


def snap(value, origin, cell):
    """Nearest cell border: keeps integer lookups valid around any pose."""
    return origin + round((value - origin) / cell) * cell


def refine_icp(vmap, points, pose, config):
    """Gauss-Newton point-to-FINITE-segment ICP from `pose`.

    Each point is paired with its closest point c on the closest segment;
    the residual is the distance |q - c| and its direction u = (q - c)/|q-c|
    (the segment's normal inside it, the direction to the endpoint beyond
    it). Linearised: d(|q - c|) = u . (dt + dtheta * R'(theta) p). Only
    pairs closer than a limit shrinking from icp_start_m to inlier_m count.
    """
    x, y, theta = pose
    for iteration in range(config.icp_iterations):
        placed = transform_points(points, (x, y, theta))
        closest, distance, _ = closest_on_segments(placed, vmap.starts,
                                                   vmap.ends)
        limit = max(config.inlier_m, config.icp_start_m * 0.8 ** iteration)
        use = (distance < limit) & (distance > 1e-9)
        if use.sum() < 10:
            break
        u = (placed[use] - closest[use]) / distance[use, None]
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        p = points[use]
        d_rotated = np.column_stack([-sin_t * p[:, 0] - cos_t * p[:, 1],
                                     cos_t * p[:, 0] - sin_t * p[:, 1]])
        jacobian = np.column_stack([u, (u * d_rotated).sum(axis=1)])
        hessian = jacobian.T @ jacobian + 1e-6 * np.eye(3)
        step = -np.linalg.solve(hessian, jacobian.T @ distance[use])
        x, y, theta = x + step[0], y + step[1], wrap_angle(theta + step[2])
        if limit == config.inlier_m and abs(step[2]) < 1e-6 \
                and math.hypot(step[0], step[1]) < 1e-5:
            break
    return (float(x), float(y), float(wrap_angle(theta)))


def support_measures(vmap, points, pose, config):
    """Vector-specific evidence for a pose (all from exact distances).

    inliers     - share of points within inlier_m of a segment
    rms_m       - RMS distance of those inliers
    segments    - segments holding at least 5 inlier points
    constraint  - lambda_min / lambda_max of sum(u u^T) over inliers that
                  lie INSIDE a segment (u = its normal): near 0 when every
                  matched wall is parallel (position along them unknown)
    """
    placed = transform_points(points, pose)
    closest, distance, index = closest_on_segments(placed, vmap.starts,
                                                   vmap.ends)
    inlier = distance <= config.inlier_m
    rms = float(np.sqrt(np.mean(distance[inlier] ** 2))) if inlier.any() \
        else float("inf")
    counts = np.bincount(index[inlier], minlength=len(vmap.starts))
    ab = vmap.ends - vmap.starts
    normals = np.column_stack([-ab[:, 1], ab[:, 0]])
    normals /= np.linalg.norm(normals, axis=1, keepdims=True)
    n = normals[index[inlier]]
    information = n.T @ n
    eigen = np.linalg.eigvalsh(information) if inlier.sum() >= 2 \
        else np.zeros(2)
    return {"inliers": float(inlier.mean()), "rms_m": rms,
            "segments": int((counts >= 5).sum()),
            "constraint": float(eigen[0] / eigen[1]) if eigen[1] > 0 else 0.0,
            "score": float(kernel(distance, config.tau_m).mean())}


def localise_vector(vmap, scan_points, initial_pose=None, config=None,
                    true_pose=None, progress=None, should_stop=None):
    """Estimate a scan's pose on the vector map.

    initial_pose None  = GLOBAL search (whole map, 360 deg);
    initial_pose given = LOCAL search (+-xy_window_m, +-theta_window_deg).
    progress(fraction) / should_stop() are optional GUI callbacks; a
    cancelled search returns status "cancelled". Returns a
    LocalisationResult (method "vector_global" or "vector_local").
    """
    config = config or VectorSearchConfig()
    start = time.perf_counter()
    points, coarse = grid_loc.prepare_points(scan_points, config)
    method = "vector_global" if initial_pose is None else "vector_local"
    if initial_pose is not None:
        initial_pose = (float(initial_pose[0]), float(initial_pose[1]),
                        wrap_angle(float(initial_pose[2])))

    def finish(status, reason, **values):
        result = LocalisationResult(
            method=method, status=status, reason=reason,
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

    # Level 0 positions and headings.
    if initial_pose is None:
        positions = plausible_positions(vmap, config.coarse_step_m,
                                        config.position_margin_m,
                                        config.min_wall_clearance_m)
        step = math.radians(config.coarse_step_deg)
        headings = -math.pi + step * np.arange(int(round(2 * math.pi / step)))
    else:
        x0 = snap(initial_pose[0], vmap.x_min, vmap.cell)
        y0 = snap(initial_pose[1], vmap.y_min, vmap.cell)
        offsets = grid_loc.symmetric_offsets(config.xy_window_m,
                                             config.coarse_step_m)
        gx, gy = np.meshgrid(x0 + offsets, y0 + offsets)
        positions = np.column_stack([gx.ravel(), gy.ravel()])
        headings = initial_pose[2] + grid_loc.symmetric_offsets(
            math.radians(config.theta_window_deg),
            math.radians(config.coarse_step_deg))
    coarse_table = kernel(vmap.distance, config.coarse_tau_m)
    fine_table = kernel(vmap.distance, config.fine_tau_m)
    scores = np.empty((len(headings), len(positions)))
    try:
        for first in range(0, len(headings), config.heading_batch):
            if should_stop is not None and should_stop():
                raise SearchCancelled
            batch = headings[first:first + config.heading_batch]
            scores[first:first + len(batch)] = table_scores(
                coarse, coarse_table, vmap, positions, batch)
            if progress is not None:
                progress(0.7 * (first + len(batch)) / len(headings))
        evaluated = scores.size
        candidates = grid_loc.best_distinct(scores, positions, headings,
                                            config.candidates, config)
        refined = []
        fine_offsets = grid_loc.symmetric_offsets(config.coarse_step_m,
                                                  config.fine_step_m)
        fine_turns = grid_loc.symmetric_offsets(
            math.radians(config.coarse_step_deg),
            math.radians(config.fine_step_deg))
        for number, (coarse_pose, _) in enumerate(candidates):
            if should_stop is not None and should_stop():
                raise SearchCancelled
            gx, gy = np.meshgrid(coarse_pose[0] + fine_offsets,
                                 coarse_pose[1] + fine_offsets)
            fine_positions = np.column_stack([gx.ravel(), gy.ravel()])
            fine_headings = coarse_pose[2] + fine_turns
            fine = table_scores(points, fine_table, vmap, fine_positions,
                                fine_headings)
            evaluated += fine.size
            h, p = np.unravel_index(np.argmax(fine), fine.shape)
            pose = refine_icp(vmap, points, (fine_positions[p, 0],
                                             fine_positions[p, 1],
                                             fine_headings[h]), config)
            refined.append((pose, exact_score(vmap, points, pose,
                                              config.tau_m), coarse_pose))
            if progress is not None:
                progress(0.7 + 0.3 * (number + 1) / len(candidates))
    except SearchCancelled:
        return finish("cancelled", "search cancelled by the user")

    refined.sort(key=lambda item: -item[1])
    pose, score, coarse_pose = refined[0]
    runner = next(((other, other_score) for other, other_score, _
                   in refined[1:] if grid_loc.poses_distinct(other, pose,
                                                             config)),
                  (None, None))
    measures = support_measures(vmap, points, pose, config)
    status, reason = judge(score, runner[1], measures, config,
                           on_edge=initial_pose is not None and on_window_edge(
                               coarse_pose, initial_pose, config))
    return finish(status, reason, pose=pose, score=score,
                  candidates_evaluated=int(evaluated),
                  out_of_map_fraction=float("nan"),
                  runner_up_score=runner[1],
                  details={"coverage_deg": coverage,
                           "coarse_points": len(coarse),
                           "positions": len(positions),
                           "headings": len(headings),
                           **measures,
                           "runner_up_pose": None if runner[0] is None
                           else [round(v, 4) for v in pose_to_day1(runner[0])],
                           "candidates": [
                               [round(v, 4) for v in pose_to_day1(p)] + [s]
                               for p, s, _ in refined]})


def on_window_edge(coarse_pose, initial_pose, config):
    """Local mode: the coarse pose behind the answer is on the window's rim."""
    edge_xy = config.xy_window_m - config.coarse_step_m / 2
    edge_theta = math.radians(config.theta_window_deg
                              - config.coarse_step_deg / 2)
    return (abs(coarse_pose[0] - initial_pose[0]) > edge_xy
            or abs(coarse_pose[1] - initial_pose[1]) > edge_xy
            or abs(wrap_angle(coarse_pose[2] - initial_pose[2])) > edge_theta)


# ---------------------------------------------------------------------------
# 4. Verdict
# ---------------------------------------------------------------------------
def judge(score, runner_up, measures, config, on_edge=False):
    """(status, reason): ok, weak, ambiguous, at_search_limit, low_score.

    A high score alone never gives "ok": it also needs enough inliers,
    several supporting segments, walls in two directions (constraint) and
    a clear margin over the best clearly different pose.
    """
    ratio = runner_up / score if runner_up is not None and score > 0 else 0.0
    evidence = (f"score {score:.3f}, {measures['inliers']:.0%} of points "
                f"within {config.inlier_m * 100:.0f} cm of a segment "
                f"(RMS {measures['rms_m'] * 100:.1f} cm), "
                f"{measures['segments']} supporting segments, constraint "
                f"{measures['constraint']:.2f}"
                + (f", best different pose {runner_up:.3f}"
                   if runner_up is not None else ""))
    if score < config.min_score:
        return "low_score", (f"not localised: {evidence}; at least "
                             f"{config.min_score} needed")
    if on_edge:
        return "at_search_limit", (
            f"the best pose is on the edge of the local window: {evidence}")
    if ratio >= config.ambiguity_ratio:
        return "ambiguous", (f"a clearly different pose fits almost as well "
                             f"({ratio:.0%} of the best): {evidence}")
    failed = [text for ok, text in (
        (score >= config.ok_score, f"score >= {config.ok_score}"),
        (measures["inliers"] >= config.ok_inliers,
         f"inliers >= {config.ok_inliers:.0%}"),
        (measures["segments"] >= config.ok_segments,
         f">= {config.ok_segments} segments"),
        (measures["constraint"] >= config.ok_constraint,
         f"constraint >= {config.ok_constraint}"),
        (ratio <= config.ok_ratio,
         f"best different pose <= {config.ok_ratio:.0%} of best"))
        if not ok]
    if failed:
        return "weak", (f"plausible but NOT CONFIRMED (check visually): "
                        f"{evidence}. Missing: {', '.join(failed)}")
    return "ok", evidence


# ---------------------------------------------------------------------------
# 5. Real-data evaluation and comparison with Code 04 (reproducible)
# ---------------------------------------------------------------------------
# Leave-one-out: the held-out scan is removed BEFORE Code 03 runs, so no
# merged segment contains its points (rebuilding, not filtering the 74).
# Code 04 runs on the same scan with its own leave-one-out grid map. Local
# guesses use Code 04's REAL_INIT_CONDITIONS with the SAME seeds, so both
# methods start from identical guesses.
EVALUATION_NOTES = (
    "Real LD06 recordings only; session files are read, never written.",
    "Leave-one-out: the vector map is REBUILT by Code 03 from the other scans "
    "(the held-out scan contributes to no segment); the grid map likewise.",
    "Day 1 poses are estimates (scans 22 and 25 hand-placed): deviation is "
    "agreement with Day 1, not physical accuracy. No measured poses exist.",
    "Separate recordings have no reference pose: only the difference between "
    "the two methods is reported.",
    "Runtimes: one run each, algorithm only (no loading, no drawing).",
    "Statuses are each method's own verdict; the two scores are on different "
    "scales and are not compared numerically.")


def build_vector_map(scans):
    """Code 03 pipeline (app_session.build_vectors, unchanged) -> VectorMap."""
    from app_session import build_vectors
    return VectorMap.from_merged(build_vectors(scans)[3])


def loo_vector_map(scans, scan_id):
    """Map of every scan EXCEPT scan_id, rebuilt from the remaining points."""
    return build_vector_map([scan for scan in scans if scan["id"] != scan_id])


def evaluation_row(**values):
    """Flatten the two methods' results; add agreement and differences."""
    row = dict(values)
    for key in ("vector", "grid"):
        result = row.pop(key)
        row[f"{key}_status"] = result.status
        row[f"{key}_pose"] = (None if result.pose is None
                              else [round(v, 4) for v in pose_to_day1(result.pose)])
        row[f"{key}_runtime_ms"] = round(result.runtime_ms, 1)
        row[f"{key}_score"] = result.score
        reference = row.get("day1_pose")
        if reference is not None and result.pose is not None:
            d, a = common.pose_error(result.pose, pose_from_day1(reference))
            row[f"{key}_day1_deviation_m"] = round(d, 4)
            row[f"{key}_day1_deviation_deg"] = round(a, 3)
    if row["vector_pose"] and row["grid_pose"]:
        d, a = common.pose_error(pose_from_day1(row["vector_pose"]),
                                 pose_from_day1(row["grid_pose"]))
        row["methods_difference_m"] = round(d, 4)
        row["methods_difference_deg"] = round(a, 3)
    return row


def run_evaluation(path, separate_paths=(), local=True, log=print):
    """All experiments on real data; returns (rows, summary). Writes nothing."""
    scans = common.load_session_scans(path)
    spec = grid_loc.session_spec(scans)
    rows = []
    for scan in scans:
        vmap = loo_vector_map(scans, scan["id"])
        gmap = grid_loc.reference_map(scans, scan["id"], spec)[0]
        day1 = [round(v, 4) for v in pose_to_day1(scan["pose_rad"])]
        rows.append(evaluation_row(
            session=Path(path).name, scan=scan["id"], experiment="global LOO",
            condition="no initial pose", trial=0, day1_pose=day1,
            vector=localise_vector(vmap, scan["points_local"]),
            grid=grid_loc.localise_global(gmap, scan["points_local"])))
        log(f"  scan {scan['id']}: vector {rows[-1]['vector_status']} "
            f"({rows[-1].get('vector_day1_deviation_m')} m) | grid "
            f"{rows[-1]['grid_status']} ({rows[-1].get('grid_day1_deviation_m')} m)")
        if not local:
            continue
        for level, (name, error_m, error_deg, trials) in \
                enumerate(grid_loc.REAL_INIT_CONDITIONS):
            for trial in range(trials):
                rng = np.random.default_rng(1000 * scan["id"] + 10 * level + trial)
                initial = common.perturb_pose(scan["pose_rad"], error_m,
                                              error_deg, rng)
                rows.append(evaluation_row(
                    session=Path(path).name, scan=scan["id"],
                    experiment="local LOO", condition=name, trial=trial,
                    day1_pose=day1,
                    initial_pose=[round(v, 4) for v in pose_to_day1(initial)],
                    vector=localise_vector(vmap, scan["points_local"], initial),
                    grid=grid_loc.localise_scan(gmap, scan["points_local"],
                                                initial)))
    if separate_paths:
        full_vector = build_vector_map(scans)
        full_grid = grid_loc.build_grid_map(scans, spec)[0]
    for other in separate_paths:
        for target in grid_loc.load_recorded_targets(other):
            rows.append(evaluation_row(
                session=Path(other).name, scan=target["id"],
                experiment="global separate recording",
                condition="no initial pose", trial=0, day1_pose=None,
                vector=localise_vector(full_vector, target["points_local"]),
                grid=grid_loc.localise_global(full_grid, target["points_local"])))
            log(f"  {Path(other).name} scan {target['id']}: vector "
                f"{rows[-1]['vector_status']} | grid {rows[-1]['grid_status']}"
                f" | methods differ by {rows[-1].get('methods_difference_m')} m")
    return rows, summarise(rows)


def summarise(rows):
    """Per experiment and condition: statuses, agreement, runtimes."""
    groups = {}
    for row in rows:
        groups.setdefault((row["experiment"], row["condition"]), []).append(row)
    summary = []
    for (experiment, condition), group in groups.items():
        entry = {"experiment": experiment, "condition": condition,
                 "runs": len(group)}
        for key in ("vector", "grid"):
            statuses = {}
            for row in group:
                statuses[row[f"{key}_status"]] = \
                    statuses.get(row[f"{key}_status"], 0) + 1
            measured = [row for row in group
                        if row.get(f"{key}_day1_deviation_m") is not None]
            agree = [row for row in measured
                     if row[f"{key}_day1_deviation_m"] <= common.SUCCESS_POSITION_M
                     and row[f"{key}_day1_deviation_deg"] <= common.SUCCESS_ANGLE_DEG]
            entry[key] = {
                "statuses": statuses,
                "agree_with_day1": len(agree) if measured else None,
                "ok_but_disagree_with_day1": [
                    row["scan"] for row in measured
                    if row[f"{key}_status"] == "ok" and row not in agree],
                "median_day1_deviation_m": float(np.median(
                    [row[f"{key}_day1_deviation_m"] for row in measured]))
                if measured else None,
                "median_runtime_ms": float(np.median(
                    [row[f"{key}_runtime_ms"] for row in group])),
                "max_runtime_ms": float(np.max(
                    [row[f"{key}_runtime_ms"] for row in group]))}
        differences = [row["methods_difference_m"] for row in group
                       if row.get("methods_difference_m") is not None]
        entry["median_methods_difference_m"] = (
            float(np.median(differences)) if differences else None)
        summary.append(entry)
    return summary


COMPARISON_COLUMNS = (
    "session", "scan", "experiment", "condition", "trial", "day1_pose",
    "initial_pose", "grid_pose", "vector_pose", "methods_difference_m",
    "methods_difference_deg", "grid_day1_deviation_m",
    "grid_day1_deviation_deg", "vector_day1_deviation_m",
    "vector_day1_deviation_deg", "grid_runtime_ms", "vector_runtime_ms",
    "grid_status", "vector_status")


def save_evaluation(rows, summary, folder, stem, source):
    """Write <stem>.json (everything) and <stem>.csv (comparison table)."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    json_path, csv_path = folder / f"{stem}.json", folder / f"{stem}.csv"
    json_path.write_text(json.dumps({
        "phase": "Day 2 Phase 2 - vector localisation (Code 05) vs grid (Code 04)",
        "created": datetime.now().isoformat(timespec="seconds"),
        "session": str(source), "frame": "Scan 1 = (0, 0, 0 deg)",
        "vector_config": VectorSearchConfig().to_dict(),
        "grid_global_config": grid_loc.GlobalSearchConfig().to_dict(),
        "notes": EVALUATION_NOTES, "summary": summary, "runs": rows},
        indent=1, default=float), encoding="utf-8")
    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=COMPARISON_COLUMNS,
                                extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: json.dumps(value) if isinstance(value, list)
                             else value for key, value in row.items()})
    return json_path, csv_path


# ---------------------------------------------------------------------------
# Automated tests: python -m unittest 05_vector_localisation -v
# ---------------------------------------------------------------------------
# The MATHEMATICS is checked on a small hand-made segment map (an
# asymmetric room) where the true pose is known exactly; everything else
# uses the real LD06 recordings.
CLASSROOM_FINAL = common.SCANS_DIR / "classroom_final.json"
FIXTURE_ROOM = [((0, 0), (6, 0)), ((6, 0), (6, 4)), ((6, 4), (3, 4)),
                ((3, 4), (3, 6)), ((3, 6), (0, 6)), ((0, 6), (0, 0)),
                ((1.5, 1.5), (2.5, 1.5)), ((2.5, 1.5), (2.5, 2.2))]


def fixture_map():
    merged = [{"start": np.array(a, float), "end": np.array(b, float),
               "support": 1} for a, b in FIXTURE_ROOM]
    return VectorMap.from_merged(merged)


def fixture_scan(pose, spacing=0.03, noise=0.005, seed=0):
    """Points sampled on the fixture walls VISIBLE from pose (ray casting),
    returned in the sensor frame."""
    rng = np.random.default_rng(seed)
    vmap = fixture_map()
    angles = np.radians(np.arange(0.0, 360.0, 0.8))
    origin = np.array(pose[:2])
    hits = []
    for angle in angles:
        direction = np.array([math.cos(angle), math.sin(angle)])
        best = math.inf
        for a, b in zip(vmap.starts, vmap.ends):
            matrix = np.column_stack([direction, a - b])
            if abs(np.linalg.det(matrix)) < 1e-12:
                continue
            t, s = np.linalg.solve(matrix, a - origin)
            if t > 0 and 0 <= s <= 1:
                best = min(best, t)
        if math.isfinite(best):
            hits.append(origin + (best + rng.normal(0, noise)) * direction)
    return common.global_to_local(np.array(hits), pose)


class SegmentDistanceTests(unittest.TestCase):
    def test_inside_beyond_endpoints_and_degenerate(self):
        starts, ends = np.array([[0.0, 0.0]]), np.array([[2.0, 0.0]])
        points = np.array([[1.0, 0.5], [3.0, 0.0], [-1.0, -1.0], [2.0, 0.0]])
        distances = point_segment_distances(points, starts, ends)[:, 0]
        np.testing.assert_allclose(distances, [0.5, 1.0, math.sqrt(2), 0.0])
        # A point on the infinite line beyond the end is NOT on the segment.
        self.assertAlmostEqual(distances[1], 1.0)
        zero = point_segment_distances(np.array([[1.0, 1.0]]),
                                       np.array([[0.0, 0.0]]),
                                       np.array([[0.0, 0.0]]))
        self.assertAlmostEqual(zero[0, 0], math.sqrt(2))

    def test_closest_points_agree_with_distances(self):
        rng = np.random.default_rng(1)
        starts, ends = rng.uniform(-3, 3, (9, 2)), rng.uniform(-3, 3, (9, 2))
        points = rng.uniform(-4, 4, (500, 2))
        closest, distance, index = closest_on_segments(points, starts, ends)
        table = point_segment_distances(points, starts, ends)
        np.testing.assert_allclose(distance, table.min(axis=1))
        np.testing.assert_array_equal(index, table.argmin(axis=1))
        np.testing.assert_allclose(np.hypot(*(points - closest).T), distance)

    def test_distance_table_is_exact_at_cell_centres(self):
        vmap = fixture_map()
        j, i = 17, 23
        centre = np.array([[vmap.x_min + (i + 0.5) * vmap.cell,
                            vmap.y_min + (j + 0.5) * vmap.cell]])
        self.assertAlmostEqual(vmap.distance[j, i], point_segment_distances(
            centre, vmap.starts, vmap.ends).min())

    def test_kernel_is_bounded(self):
        np.testing.assert_allclose(kernel([0.0, 0.05, 0.1, 5.0], 0.1),
                                   [1.0, 0.75, 0.0, 0.0])


class FixtureSearchTests(unittest.TestCase):
    def test_global_search_recovers_any_heading(self):
        vmap = fixture_map()
        for pose_deg in ((1.2, 3.1, 137.0), (4.5, 1.0, -100.0),
                         (2.0, 5.0, 179.0)):
            truth = pose_from_day1(pose_deg)
            result = localise_vector(vmap, fixture_scan(truth), true_pose=truth)
            with self.subTest(pose=pose_deg):
                self.assertLess(result.position_error_m, 0.02)
                self.assertLess(result.angle_error_deg, 0.5)
                self.assertIsNone(result.initial_pose)
                self.assertEqual(result.details["headings"], 120)

    def test_icp_handles_endpoints_and_coordinates(self):
        vmap = fixture_map()
        truth = pose_from_day1((4.0, 2.5, 30.0))
        points = filter_ranges(fixture_scan(truth, noise=0.0))
        start = (truth[0] + 0.12, truth[1] - 0.08, truth[2] + 0.04)
        pose = refine_icp(vmap, points, start, VectorSearchConfig())
        distance, angle = common.pose_error(pose, truth)
        self.assertLess(distance, 0.005)
        self.assertLess(angle, 0.1)
        # The pose maps sensor-frame points onto the segments (map frame).
        placed = transform_points(points, pose)
        self.assertLess(point_segment_distances(
            placed, vmap.starts, vmap.ends).min(axis=1).max(), 0.01)

    def test_local_mode_window_edge_and_cancel(self):
        vmap = fixture_map()
        truth = pose_from_day1((1.2, 3.1, 40.0))
        points = fixture_scan(truth)
        good = localise_vector(vmap, points, (truth[0] + 0.3, truth[1], truth[2]),
                               true_pose=truth)
        self.assertLess(good.position_error_m, 0.02)
        self.assertEqual(good.method, "vector_local")
        progress = []
        cancelled = localise_vector(vmap, points, progress=progress.append,
                                    should_stop=lambda: len(progress) >= 2)
        self.assertEqual(cancelled.status, "cancelled")
        self.assertIsNone(cancelled.pose)


class VerdictTests(unittest.TestCase):
    def test_a_high_score_alone_is_not_ok(self):
        config = VectorSearchConfig()
        strong = {"inliers": 0.6, "rms_m": 0.02, "segments": 10,
                  "constraint": 0.6}
        self.assertEqual(judge(0.6, 0.2, strong, config)[0], "ok")
        cases = [(dict(strong, constraint=0.05), 0.2, "weak"),   # parallel walls
                 (dict(strong, segments=2), 0.2, "weak"),
                 (dict(strong, inliers=0.3), 0.2, "weak"),
                 (strong, 0.5, "weak"),                          # small margin
                 (strong, 0.58, "ambiguous"),
                 (strong, None, "ok")]
        for measures, runner_up, expected in cases:
            with self.subTest(measures=measures, runner_up=runner_up):
                self.assertEqual(judge(0.6, runner_up, measures, config)[0],
                                 expected)
        self.assertEqual(judge(0.25, None, strong, config)[0], "low_score")
        self.assertEqual(judge(0.6, None, strong, config, on_edge=True)[0],
                         "at_search_limit")


class RealDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not CLASSROOM_FINAL.exists():
            raise unittest.SkipTest("classroom_final.json absent")
        cls.fingerprint = grid_loc.part2.file_sha256(CLASSROOM_FINAL)
        cls.scans = common.load_session_scans(CLASSROOM_FINAL)
        from app_session import build_vectors
        cls.build_vectors = staticmethod(build_vectors)
        cls.full = build_vector_map(cls.scans)

    @classmethod
    def tearDownClass(cls):
        assert grid_loc.part2.file_sha256(CLASSROOM_FINAL) == cls.fingerprint

    def test_reference_is_the_74_segment_code03_map(self):
        merged = self.build_vectors(self.scans)[3]
        self.assertEqual(len(self.full.starts), 74)
        np.testing.assert_array_equal(self.full.starts,
                                      [s["start"] for s in merged])
        # Day 1 frame: Scan 1 sits at the origin.
        self.assertEqual(tuple(self.scans[0]["pose"]), (0.0, 0.0, 0.0))

    def test_leave_one_out_map_is_rebuilt_without_the_scan(self):
        held_out = self.scans[3]["id"]
        others = [s for s in self.scans if s["id"] != held_out]
        merged = self.build_vectors(others)[3]
        self.assertTrue(all(held_out not in s["scan_ids"] for s in merged))
        full_ids = set().union(*(s["scan_ids"] for s in
                                 self.build_vectors(self.scans)[3]))
        self.assertIn(held_out, full_ids)
        vmap = loo_vector_map(self.scans, held_out)
        np.testing.assert_array_equal(vmap.starts, [s["start"] for s in merged])

    def test_real_scans_localised_globally_against_day1(self):
        for scan in (self.scans[1], self.scans[7]):
            vmap = loo_vector_map(self.scans, scan["id"])
            before = (vmap.starts.copy(), vmap.ends.copy(), vmap.distance.copy())
            result = localise_vector(vmap, scan["points_local"],
                                     true_pose=scan["pose_rad"])
            with self.subTest(scan=scan["id"]):
                self.assertEqual(result.status, "ok", result.reason)
                # Agreement with the Day 1 ESTIMATE, not ground truth.
                self.assertLess(result.position_error_m, 0.05)
                self.assertLess(result.angle_error_deg, 1.0)
                for array, copy in zip((vmap.starts, vmap.ends, vmap.distance),
                                       before):
                    np.testing.assert_array_equal(array, copy)  # map unchanged

    def test_wrong_and_unsupported_inputs_are_not_ok(self):
        scan = self.scans[4]
        vmap = loo_vector_map(self.scans, scan["id"])
        wrong_start = (scan["pose_rad"][0], scan["pose_rad"][1],
                       wrap_angle(scan["pose_rad"][2] + math.pi))
        self.assertNotEqual(localise_vector(vmap, scan["points_local"],
                                            wrong_start).status, "ok")
        mirrored = scan["points_local"] * np.array([1.0, -1.0])  # SYNTHETIC
        self.assertNotEqual(localise_vector(vmap, mirrored).status, "ok")
        separate = common.SCANS_DIR / "classroom_session.json"
        if separate.exists():
            occluded = grid_loc.load_recorded_targets(separate)[0]["points_local"]
            self.assertEqual(localise_vector(self.full, occluded).status,
                             "low_score")


class ProgramTests(unittest.TestCase):
    def test_application_mode_opens_the_vector_page(self):
        import desktop_app
        with unittest.mock.patch.object(desktop_app, "launch") as launch:
            main(CLASSROOM_FINAL)
        launch.assert_called_once()
        self.assertEqual(launch.call_args.kwargs["initial_page"],
                         "Vector Localisation")

    def test_cancelled_picker_uses_the_default_session(self):
        module = sys.modules[__name__]
        with unittest.mock.patch.object(grid_loc, "choose_session_file",
                                        return_value=None), \
                unittest.mock.patch.object(module, "main") as fake_main:
            start_from_pycharm()
        fake_main.assert_called_once_with(None)

    def test_evaluation_rows_compare_both_methods(self):
        scans = common.load_session_scans(CLASSROOM_FINAL)[:6]
        with unittest.mock.patch.object(common, "load_session_scans",
                                        return_value=scans):
            rows, summary = run_evaluation(CLASSROOM_FINAL, local=False,
                                           log=lambda *_: None)
        self.assertEqual(len(rows), 6)
        for key in ("grid_pose", "vector_pose", "methods_difference_m",
                    "grid_runtime_ms", "vector_runtime_ms", "grid_status",
                    "vector_status", "vector_day1_deviation_m"):
            self.assertIn(key, rows[0])
        self.assertEqual(summary[0]["runs"], 6)

# ---------------------------------------------------------------------------
# 6. Program
# ---------------------------------------------------------------------------
def separate_recordings(path):
    """Other sessions in scans/ that are not byte copies of `path`."""
    digest = grid_loc.part2.file_sha256(path)
    return [other for other in sorted(common.SCANS_DIR.glob("*.json"))
            if grid_loc.part2.file_sha256(other) != digest]


def main(session_file=None):
    path = grid_loc.session_path(session_file or SESSION_FILE)
    if not path.is_file():
        print(f"Session file not found: {path}")
        return
    if RUN_MODE == "application":
        from desktop_app import launch
        launch(path, initial_page="Vector Localisation")
    elif RUN_MODE == "evaluation":
        separate = separate_recordings(path)
        print(f"=== Code 05 evaluation: {path.name}; separate recordings: "
              f"{', '.join(p.name for p in separate) or 'none'} ===")
        rows, summary = run_evaluation(path, separate)
        stem = f"{RESULTS_FILE_STEM}_{path.stem}_{datetime.now():%Y%m%d_%H%M%S}"
        for written in save_evaluation(rows, summary, common.RESULTS_DIR,
                                       stem, path):
            print(f"Saved {written}")
    else:
        raise ValueError(f"RUN_MODE = {RUN_MODE!r}: use 'application' or "
                         f"'evaluation'")


def start_from_pycharm():
    """Ask for the session (as Codes 02-04 do; Cancel = default), then run."""
    chosen = grid_loc.choose_session_file() if ASK_FOR_SESSION_FILE else None
    main(chosen)


if __name__ == "__main__":
    start_from_pycharm()
