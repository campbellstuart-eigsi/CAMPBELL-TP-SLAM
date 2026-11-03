"""Day 2, Phase 3 - scan-to-scan localisation (correlative search + ICP).

Given previously recorded LiDAR scans with their saved poses (the
reference) and one new scan in its own sensor frame, estimate the new
scan's pose (x, y, theta) in the session frame: Scan 1 = (0, 0, 0 deg).

The matching mathematics is Code 01's Auto Align, called unchanged:
    01_lidar_visualizer.align_scan(source_local, reference_scans, ...)
        correlative search over positions and headings on a likelihood
        image of the REFERENCE POINTS -> 10 distinct candidates -> fine
        search -> trimmed point-to-point ICP -> matched fraction, RMS,
        see-through conflicts in both directions -> verdict.
This file only chooses the reference, the search window and the meaning of
the verdict for LOCALISATION, and it never changes the session:
    - Day 1 Auto Align adds a scan to the map (Accept moves its pose);
      localisation reports a pose and keeps the reference fixed.
    - The reference is raw LiDAR points only (no grid, no vector map):
        "single"      - ONE positioned scan (strict scan-to-scan);
        "accumulated" - several positioned scans pooled into one point
                        cloud (scan-to-accumulated-point-cloud).
    - Global (whole reference area, 360 deg) or local (window around a
      guess) search.
    - Evaluation excludes the target scan from every reference.

Core call (no Tk, Matplotlib, serial port or file dialog; Day 3 can use it):
    result = localise_on_scans(scan_points, references, initial_pose=None)

Run this file in PyCharm (RUN_MODE below):
    "application" - unified desktop app, Scan Localisation page
    "evaluation"  - real-data benchmark and Day 2 three-method comparison
Tests: python -m unittest 06_scan_localisation -v
"""
import csv
import importlib
import json
import math
import sys
import time
import unittest
import unittest.mock
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

PROJECT_DIR = Path(__file__).resolve().parent
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))
import localisation_common as common  # noqa: E402
from localisation_common import (  # noqa: E402
    LocalisationResult, filter_ranges, pose_from_day1, pose_to_day1,
    transform_points, wrap_angle)

part1 = common.day1("part1")                       # the matcher (unchanged)
grid_loc = importlib.import_module("04_grid_localisation")
vector_loc = importlib.import_module("05_vector_localisation")

RUN_MODE = "application"       # "application" or "evaluation"
SESSION_FILE = "classroom_final.json"
ASK_FOR_SESSION_FILE = True
RESULTS_FILE_STEM = "phase3_scan_localisation"
COMPARISON_FILE_STEM = "day2_localisation_comparison"


# ---------------------------------------------------------------------------
# 1. Reference: positioned scans, as Code 01 expects them
# ---------------------------------------------------------------------------
def reference_scans(scans, mode="accumulated", reference_id=None,
                    exclude_ids=()):
    """[(local points (N, 2), pose (x_m, y_m, theta_deg))] for Code 01.

    scans        - dicts with "id", "points_local" and the saved "pose"
                   (Day 1 degrees) in the session frame
    mode         - "single": only scan reference_id; "accumulated": all
    exclude_ids  - never used (the scan being localised): no point of an
                   excluded scan can reach the reference
    The points are copied: localisation cannot modify the session. Code 01
    places each one with its pose (local -> Scan 1 frame) before matching.
    """
    excluded = set(exclude_ids)
    if mode == "single":
        if reference_id in excluded:
            raise ValueError("a scan cannot be its own reference")
        chosen = [scan for scan in scans if scan["id"] == reference_id]
        if not chosen:
            raise ValueError(f"no reference scan with id {reference_id}")
    elif mode == "accumulated":
        chosen = [scan for scan in scans if scan["id"] not in excluded]
    else:
        raise ValueError("mode must be 'single' or 'accumulated'")
    if not chosen:
        raise ValueError("the reference is empty")
    return [(np.array(scan["points_local"], dtype=float, copy=True),
             tuple(float(v) for v in scan["pose"])) for scan in chosen]


# ---------------------------------------------------------------------------
# 2. Localisation (Code 01 matching, localisation verdict)
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ScanSearchConfig:
    xy_window_m: float = 1.0       # local search: guess +- this (as 04, 05)
    theta_window_deg: float = 30.0

    def to_dict(self):
        return asdict(self)


# Code 01 verdict -> Day 2 status. "good" is Code 01's strongest verdict
# (matched >= 50 %, RMS <= 3 cm, <= 12 % see-through, no close rival); its
# "weak" = plausible, CHECK VISUALLY. Failures keep Code 01's reason.
FAILURE_STATUSES = (("ambiguous", "ambiguous"),
                    ("insufficient overlap", "insufficient_overlap"),
                    ("no consistent pose", "inconsistent"),
                    ("poor fit", "poor_fit"),
                    ("not enough points", "insufficient_points"))


def localisation_status(alignment, local_window=None):
    """Day 2 status of a Code 01 alignment result."""
    if alignment["status"] == "good":
        status = "ok"
    elif alignment["status"] == "weak":
        status = "weak"
    else:
        status = next((code for prefix, code in FAILURE_STATUSES
                       if alignment["reason"].startswith(prefix)), "failed")
    if local_window is not None and status in ("ok", "weak") \
            and alignment["coarse_pose"] is not None:
        x, y, theta, half_width, half_angle = local_window
        cx, cy, ctheta = alignment["coarse_pose"]
        edge_m = half_width - part1.ALIGN_COARSE_STEP_M / 2
        edge_deg = half_angle - part1.ALIGN_COARSE_STEP_DEG / 2
        if abs(cx - x) > edge_m or abs(cy - y) > edge_m \
                or part1.angle_difference_deg(ctheta, theta) > edge_deg:
            status = "at_search_limit"
    return status


def localise_on_scans(scan_points, references, initial_pose=None,
                      config=None, true_pose=None, progress=None,
                      should_stop=None, mode="accumulated"):
    """Estimate a new scan's pose against positioned reference scans.

    scan_points  - (N, 2) ORIGINAL sensor-frame points of the new scan
    references   - reference_scans(...) output; never modified
    initial_pose - None = GLOBAL search; (x, y, theta_rad) = LOCAL search
                   within +-xy_window_m and +-theta_window_deg
    Returns a LocalisationResult: pose (x, y, theta_rad) in the Scan 1
    frame; details hold Code 01's match fraction, RMS, conflicts and the
    best clearly different pose.
    """
    config = config or ScanSearchConfig()
    start = time.perf_counter()
    window = None
    if initial_pose is not None:
        initial_pose = (float(initial_pose[0]), float(initial_pose[1]),
                        wrap_angle(float(initial_pose[2])))
        x, y, theta_deg = pose_to_day1(initial_pose)
        window = (x, y, theta_deg, config.xy_window_m, config.theta_window_deg)
    method = f"scan_{mode}_{'global' if window is None else 'local'}"
    points = np.asarray(scan_points, dtype=float)  # as Code 01 receives them

    def finish(status, reason, **values):
        result = LocalisationResult(
            method=method, status=status, reason=reason,
            initial_pose=initial_pose, points_used=len(points),
            runtime_ms=(time.perf_counter() - start) * 1000.0, **values)
        return result.evaluate_against(true_pose) if true_pose else result

    try:
        alignment = part1.align_scan(points, references, search_window=window,
                                     progress=progress,
                                     should_stop=should_stop)
    except part1.AlignmentCancelled:
        return finish("cancelled", "search cancelled by the user")
    status = localisation_status(alignment, window)
    if alignment["pose"] is None:
        return finish(status, alignment["reason"])
    reason = alignment["reason"]
    if status == "at_search_limit":
        reason = ("the best pose lies on the edge of the local window: the "
                  "true pose may be outside it. " + reason)
    runner = alignment["runner_up"]
    return finish(
        status, reason, pose=pose_from_day1(alignment["pose"]),
        score=alignment["score"],
        runner_up_score=None if runner is None else runner["score"],
        details={"mode": mode, "reference_scans": len(references),
                 "match_fraction": alignment["match_fraction"],
                 "rms_m": alignment["rms_m"],
                 "conflict": alignment["conflict"],
                 "scan_sees_through": alignment["scan_sees_through"],
                 "reference_sees_through": alignment["reference_sees_through"],
                 "code01_status": alignment["status"],
                 "search_window": window,
                 "runner_up_pose": None if runner is None
                 else [round(v, 4) for v in runner["pose"]]})


# ---------------------------------------------------------------------------
# 3. Real-data evaluation and Day 2 comparison (reproducible)
# ---------------------------------------------------------------------------
EVALUATION_NOTES = (
    "Real LD06 recordings only; session files are read, never written.",
    "Self-matching excluded: the target scan is removed from every reference "
    "(single: a different scan; accumulated: all OTHER scans; Code 04/05: "
    "maps rebuilt without it).",
    "Day 1 poses are estimates (scans 22 and 25 hand-placed): deviation is "
    "agreement with Day 1, not physical accuracy. No measured poses exist; a "
    "<session>_measured_poses.json file would be evaluated as true error.",
    "Day 1 poses are used only AFTER each search, to measure agreement; the "
    "single-reference distance is an analysis label, never an input.",
    "Separate recordings have no reference pose: only differences between "
    "methods are reported.",
    "Runtimes: one run each, algorithm only. Statuses are each method's own "
    "verdict; the three scores are different quantities and not compared.")


def result_fields(prefix, result, day1=None, measured=None):
    row = {f"{prefix}_status": result.status,
           f"{prefix}_pose": None if result.pose is None
           else [round(v, 4) for v in pose_to_day1(result.pose)],
           f"{prefix}_runtime_ms": round(result.runtime_ms, 1)}
    if "match_fraction" in result.details:
        row[f"{prefix}_match_fraction"] = round(result.details["match_fraction"], 4)
        row[f"{prefix}_rms_m"] = (round(result.details["rms_m"], 5)
                                  if math.isfinite(result.details["rms_m"]) else None)
    for label, reference in (("day1", day1), ("measured", measured)):
        if reference is not None and result.pose is not None:
            d, a = common.pose_error(result.pose, reference)
            row[f"{prefix}_{label}_deviation_m"] = round(d, 4)
            row[f"{prefix}_{label}_deviation_deg"] = round(a, 3)
    return row


def run_evaluation(path, separate_paths=(), local=True, single=True,
                   log=print):
    """Code 06 experiments + Code 04/05 on identical inputs. Writes nothing."""
    scans, measured, _ = grid_loc.load_session(path)
    measured = {key: value["pose"] for key, value in measured.items()}
    spec = grid_loc.session_spec(scans)
    rows = []
    by_id = {scan["id"]: scan for scan in scans}
    for scan in scans:
        target, day1 = scan["points_local"], scan["pose_rad"]
        others = reference_scans(scans, exclude_ids=[scan["id"]])
        base = {"session": Path(path).name, "scan": scan["id"],
                "day1_pose": [round(v, 4) for v in pose_to_day1(day1)]}
        acc = localise_on_scans(target, others)
        grid = grid_loc.localise_global(
            grid_loc.reference_map(scans, scan["id"], spec)[0], target)
        vector = vector_loc.localise_vector(
            vector_loc.loo_vector_map(scans, scan["id"]), target)
        rows.append({**base, "experiment": "global LOO", "reference": "accumulated",
                     "condition": "no initial pose",
                     **result_fields("scan", acc, day1, measured.get(scan["id"])),
                     **result_fields("grid", grid, day1, measured.get(scan["id"])),
                     **result_fields("vector", vector, day1, measured.get(scan["id"]))})
        log(f"  scan {scan['id']}: scan(acc) {acc.status} "
            f"{rows[-1].get('scan_day1_deviation_m')} m | grid {grid.status} | "
            f"vector {vector.status}")
        if single:
            for reference in scans:
                if reference["id"] == scan["id"]:
                    continue
                result = localise_on_scans(
                    target, reference_scans(scans, "single", reference["id"]),
                    mode="single")
                rows.append({**base, "experiment": "global single reference",
                             "reference": f"scan {reference['id']}",
                             "condition": "no initial pose",
                             "reference_distance_m": round(math.dist(
                                 day1[:2], by_id[reference["id"]]["pose_rad"][:2]), 3),
                             **result_fields("scan", result, day1)})
        if local:
            for level, (name, error_m, error_deg, trials) in \
                    enumerate(grid_loc.REAL_INIT_CONDITIONS):
                for trial in range(trials):
                    rng = np.random.default_rng(1000 * scan["id"] + 10 * level + trial)
                    initial = common.perturb_pose(day1, error_m, error_deg, rng)
                    result = localise_on_scans(target, others, initial)
                    rows.append({**base, "experiment": "local LOO",
                                 "reference": "accumulated", "condition": name,
                                 "trial": trial,
                                 "initial_pose": [round(v, 4) for v in pose_to_day1(initial)],
                                 **result_fields("scan", result, day1)})
    if separate_paths:
        everything = reference_scans(scans)
        full_grid = grid_loc.build_grid_map(scans, spec)[0]
        full_vector = vector_loc.build_vector_map(scans)
    for other in separate_paths:
        for target in grid_loc.load_recorded_targets(other):
            base = {"session": Path(other).name, "scan": target["id"],
                    "day1_pose": None}
            points = target["points_local"]
            acc = localise_on_scans(points, everything)
            row = {**base, "experiment": "global separate recording",
                   "reference": "accumulated", "condition": "no initial pose",
                   **result_fields("scan", acc),
                   **result_fields("grid", grid_loc.localise_global(full_grid, points)),
                   **result_fields("vector", vector_loc.localise_vector(full_vector, points))}
            rows.append(row)
            log(f"  {Path(other).name} scan {target['id']}: scan(acc) {acc.status} | "
                f"grid {row['grid_status']} | vector {row['vector_status']}")
            if single:
                for reference in scans:
                    result = localise_on_scans(
                        points, reference_scans(scans, "single", reference["id"]),
                        mode="single")
                    rows.append({**base, "experiment": "global single reference (separate)",
                                 "reference": f"scan {reference['id']}",
                                 "condition": "no initial pose",
                                 **result_fields("scan", result)})
    for row in rows:
        poses = {key: row.get(f"{key}_pose") for key in ("scan", "grid", "vector")}
        for a, b in (("scan", "grid"), ("scan", "vector"), ("grid", "vector")):
            if poses[a] and poses[b]:
                d, angle = common.pose_error(pose_from_day1(poses[a]),
                                             pose_from_day1(poses[b]))
                row[f"{a}_vs_{b}_m"] = round(d, 4)
                row[f"{a}_vs_{b}_deg"] = round(angle, 3)
    return rows, summarise(rows)


def summarise(rows):
    """Per experiment/condition and method: statuses, agreement, runtime."""
    groups = {}
    for row in rows:
        groups.setdefault((row["experiment"], row["condition"]), []).append(row)
    summary = []
    for (experiment, condition), group in groups.items():
        entry = {"experiment": experiment, "condition": condition,
                 "runs": len(group)}
        for key in ("scan", "grid", "vector"):
            present = [row for row in group if f"{key}_status" in row]
            if not present:
                continue
            statuses = {}
            for row in present:
                statuses[row[f"{key}_status"]] = statuses.get(row[f"{key}_status"], 0) + 1
            measured = [row for row in present
                        if row.get(f"{key}_day1_deviation_m") is not None]
            agree = [row for row in measured
                     if row[f"{key}_day1_deviation_m"] <= common.SUCCESS_POSITION_M
                     and row[f"{key}_day1_deviation_deg"] <= common.SUCCESS_ANGLE_DEG]
            confident = [row for row in measured if row[f"{key}_status"] == "ok"]
            entry[key] = {
                "statuses": statuses,
                "agree_with_day1": len(agree) if measured else None,
                "ok_but_disagree_with_day1": [
                    (row["scan"], row["reference"]) for row in confident
                    if row not in agree],
                "median_day1_deviation_m": float(np.median(
                    [row[f"{key}_day1_deviation_m"] for row in measured]))
                if measured else None,
                "median_runtime_ms": float(np.median(
                    [row[f"{key}_runtime_ms"] for row in present])),
                "max_runtime_ms": float(np.max(
                    [row[f"{key}_runtime_ms"] for row in present]))}
        summary.append(entry)
    return summary


COMPARISON_COLUMNS = (
    "session", "scan", "experiment", "reference", "condition", "trial",
    "reference_distance_m", "day1_pose", "initial_pose",
    "grid_pose", "vector_pose", "scan_pose",
    "grid_status", "vector_status", "scan_status",
    "grid_day1_deviation_m", "grid_day1_deviation_deg",
    "vector_day1_deviation_m", "vector_day1_deviation_deg",
    "scan_day1_deviation_m", "scan_day1_deviation_deg",
    "scan_measured_deviation_m", "scan_measured_deviation_deg",
    "scan_match_fraction", "scan_rms_m",
    "scan_vs_grid_m", "scan_vs_vector_m", "grid_vs_vector_m",
    "grid_runtime_ms", "vector_runtime_ms", "scan_runtime_ms")


def save_evaluation(rows, summary, folder, stem, source):
    """<stem>.json (everything) and <stem>.csv (one row per localisation)."""
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    json_path, csv_path = folder / f"{stem}.json", folder / f"{stem}.csv"
    json_path.write_text(json.dumps({
        "phase": "Day 2 Phase 3 - scan-to-scan localisation (Code 06) and "
                 "comparison with Codes 04 and 05",
        "created": datetime.now().isoformat(timespec="seconds"),
        "session": str(source), "frame": "Scan 1 = (0, 0, 0 deg)",
        "scan_config": ScanSearchConfig().to_dict(),
        "code01_thresholds": {name: getattr(part1, name) for name in (
            "MATCH_DISTANCE_M", "GOOD_MATCH_FRACTION", "MIN_MATCH_FRACTION",
            "GOOD_RMS_M", "MAX_RMS_M", "MAX_CONFLICT_FRACTION",
            "GOOD_CONFLICT_FRACTION", "AMBIGUITY_RATIO")},
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


def comparison_summary(rows):
    """Method-level Day 2 table on the SAME targets (rows with all three)."""
    shared = [row for row in rows if all(f"{key}_status" in row
                                         for key in ("scan", "grid", "vector"))]
    table = {}
    for key, name in (("grid", "Grid (04)"), ("vector", "Vector (05)"),
                      ("scan", "Scan, accumulated (06)")):
        loo = [row for row in shared if row["experiment"] == "global LOO"]
        separate = [row for row in shared if row["day1_pose"] is None]
        deviations = [row[f"{key}_day1_deviation_m"] for row in loo
                      if row.get(f"{key}_day1_deviation_m") is not None]
        statuses = {}
        for row in shared:
            statuses[row[f"{key}_status"]] = statuses.get(row[f"{key}_status"], 0) + 1
        table[name] = {
            "targets": len(shared),
            "statuses": statuses,
            "loo_agree_with_day1_10cm_5deg": sum(
                row.get(f"{key}_day1_deviation_m") is not None
                and row[f"{key}_day1_deviation_m"] <= common.SUCCESS_POSITION_M
                and row[f"{key}_day1_deviation_deg"] <= common.SUCCESS_ANGLE_DEG
                for row in loo),
            "loo_runs": len(loo),
            "loo_median_day1_deviation_m": float(np.median(deviations)) if deviations else None,
            "ok_but_disagree_with_day1": [row["scan"] for row in loo
                                          if row[f"{key}_status"] == "ok"
                                          and (row.get(f"{key}_day1_deviation_m") or 0)
                                          > common.SUCCESS_POSITION_M],
            "separate_recordings": {f"{row['session']} {row['scan']}":
                                    row[f"{key}_status"] for row in separate},
            "median_runtime_ms": float(np.median([row[f"{key}_runtime_ms"] for row in shared])),
            "max_runtime_ms": float(np.max([row[f"{key}_runtime_ms"] for row in shared])),
            "needs_initial_guess": False}
    differences = {pair: float(np.median([row[f"{pair}_m"] for row in shared
                                          if row.get(f"{pair}_m") is not None]))
                   for pair in ("scan_vs_grid", "scan_vs_vector", "grid_vs_vector")}
    return {"methods": table, "median_pose_difference_m": differences,
            "targets": [(row["session"], row["scan"]) for row in shared]}


# ---------------------------------------------------------------------------
# Automated tests: python -m unittest 06_scan_localisation -v
# ---------------------------------------------------------------------------
# Mathematics on Code 01's simulated rooms (KNOWN poses: test fixtures
# only); real behaviour on the LD06 recordings.
CLASSROOM_FINAL = common.SCANS_DIR / "classroom_final.json"


def simulated_case(true_deg, reference_deg=(2.0, 2.5, 30.0), **options):
    """New-scan points (sensor frame) and a single positioned reference."""
    source, references = part1.make_alignment_case(
        part1.CLASSROOM_WALLS, reference_deg, true_deg, **options)
    scans = [{"id": 7, "points_local": references[0][0],
              "pose": references[0][1]}]
    return source, scans


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(0)
        self.scans = [{"id": i, "points_local": rng.normal(size=(40, 2)),
                       "pose": (float(i), 0.0, 10.0 * i)} for i in (1, 2, 3)]

    def test_single_accumulated_and_exclusion(self):
        single = reference_scans(self.scans, "single", 2)
        self.assertEqual(len(single), 1)
        self.assertEqual(single[0][1], (2.0, 0.0, 20.0))
        accumulated = reference_scans(self.scans, exclude_ids=[2])
        self.assertEqual([pose for _, pose in accumulated],
                         [(1.0, 0.0, 10.0), (3.0, 0.0, 30.0)])
        with self.assertRaises(ValueError):
            reference_scans(self.scans, "single", 2, exclude_ids=[2])
        with self.assertRaises(ValueError):
            reference_scans(self.scans, "single", 9)

    def test_references_are_copies(self):
        before = self.scans[0]["points_local"].copy()
        reference_scans(self.scans)[0][0][:] = 99.0
        np.testing.assert_array_equal(self.scans[0]["points_local"], before)


class SimulatedMatchingTests(unittest.TestCase):
    def test_reference_away_from_origin_gives_global_pose(self):
        # The reference scan was recorded at (2, 2.5, 30 deg), not at the
        # origin: the answer must be the GLOBAL pose, not a relative one.
        truth_deg = (5.0, 3.8, 123.0)
        source, scans = simulated_case(truth_deg)
        references = reference_scans(scans, "single", 7)
        before = references[0][0].copy()
        result = localise_on_scans(source, references, mode="single",
                                   true_pose=pose_from_day1(truth_deg))
        self.assertIn(result.status, ("ok", "weak"), result.reason)
        self.assertLess(result.position_error_m, 0.03)
        self.assertLess(result.angle_error_deg, 0.5)
        self.assertEqual(result.method, "scan_single_global")
        np.testing.assert_array_equal(references[0][0], before)  # unchanged

    def test_full_heading_and_local_window(self):
        truth_deg = (6.0, 3.5, -110.0)
        truth = pose_from_day1(truth_deg)
        source, scans = simulated_case(truth_deg)
        references = reference_scans(scans, "single", 7)
        global_result = localise_on_scans(source, references, true_pose=truth)
        self.assertLess(global_result.angle_error_deg, 0.5)
        guess = (truth[0] + 0.4, truth[1] - 0.3, truth[2] + math.radians(12))
        local = localise_on_scans(source, references, guess, true_pose=truth)
        self.assertEqual(local.method, "scan_accumulated_local")
        self.assertLess(local.position_error_m, 0.03)
        self.assertEqual(local.details["search_window"][3:], (1.0, 30.0))
        far = (truth[0] + 2.5, truth[1], truth[2] + math.radians(90))
        self.assertNotEqual(localise_on_scans(source, references, far).status, "ok")

    def test_no_overlap_and_ambiguity_are_not_ok(self):
        source, scans = simulated_case(
            (0.5, 0.3, 10.0), source_walls=part1.OCTAGON_ROOM_WALLS)
        other_room = localise_on_scans(source, reference_scans(scans))
        self.assertNotIn(other_room.status, ("ok", "weak"), other_room.reason)
        source, references = part1.make_alignment_case(
            part1.SYMMETRIC_ROOM_WALLS, (1.5, 1.0, 0.0), (4.0, 3.0, 120.0))
        symmetric = localise_on_scans(source, references)
        self.assertEqual(symmetric.status, "ambiguous", symmetric.reason)
        self.assertIsNotNone(symmetric.details["runner_up_pose"])

    def test_cancel_and_progress(self):
        source, scans = simulated_case((4.5, 2.5, 0.0))
        calls = []
        result = localise_on_scans(source, reference_scans(scans),
                                   progress=calls.append,
                                   should_stop=lambda: len(calls) >= 2)
        self.assertEqual(result.status, "cancelled")
        self.assertIsNone(result.pose)
        self.assertTrue(all(0.0 <= value <= 1.0 for value in calls))


class VerdictMappingTests(unittest.TestCase):
    def alignment(self, status, reason="", coarse=(1.0, 1.0, 0.0)):
        return {"status": status, "reason": reason, "coarse_pose": coarse}

    def test_code01_verdicts(self):
        self.assertEqual(localisation_status(self.alignment("good")), "ok")
        self.assertEqual(localisation_status(self.alignment("weak")), "weak")
        for reason, expected in (("ambiguous: x", "ambiguous"),
                                 ("insufficient overlap: x", "insufficient_overlap"),
                                 ("no consistent pose: x", "inconsistent"),
                                 ("poor fit: x", "poor_fit"),
                                 ("not enough points to align", "insufficient_points")):
            self.assertEqual(localisation_status(self.alignment("failed", reason)),
                             expected)

    def test_local_window_edge(self):
        window = (1.0, 1.0, 0.0, 1.0, 30.0)
        self.assertEqual(localisation_status(self.alignment("good"), window), "ok")
        self.assertEqual(localisation_status(
            self.alignment("good", coarse=(1.95, 1.0, 0.0)), window), "at_search_limit")
        self.assertEqual(localisation_status(
            self.alignment("weak", coarse=(1.0, 1.0, 29.5)), window), "at_search_limit")

    def test_code01_reasons_still_start_as_mapped(self):
        best = {"pose": (0.0, 0.0, 0.0), "match_fraction": 0.8, "rms_m": 0.02,
                "conflict": 0.0, "score": 0.8}
        cases = ((dict(best, match_fraction=0.2), None, "insufficient overlap"),
                 (dict(best, rms_m=0.09), None, "poor fit"),
                 (dict(best, conflict=0.5), None, "no consistent pose"),
                 (best, dict(best, pose=(3.0, 0.0, 180.0), score=0.79), "ambiguous"))
        for result, runner_up, prefix in cases:
            self.assertTrue(part1.judge_alignment(result, runner_up)[1].startswith(prefix))


class RealDataTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not CLASSROOM_FINAL.exists():
            raise unittest.SkipTest("classroom_final.json absent")
        cls.fingerprint = grid_loc.part2.file_sha256(CLASSROOM_FINAL)
        cls.scans = common.load_session_scans(CLASSROOM_FINAL)

    @classmethod
    def tearDownClass(cls):
        assert grid_loc.part2.file_sha256(CLASSROOM_FINAL) == cls.fingerprint

    def test_leave_one_out_accumulated_and_single(self):
        target = self.scans[3]                      # Scan 4
        others = reference_scans(self.scans, exclude_ids=[target["id"]])
        signature = grid_loc.measurement_signature(target["points_local"])
        self.assertTrue(all(grid_loc.measurement_signature(points) != signature
                            for points, _ in others))   # no target points
        accumulated = localise_on_scans(target["points_local"], others,
                                        true_pose=target["pose_rad"])
        single = localise_on_scans(target["points_local"],
                                   reference_scans(self.scans, "single", 2),
                                   mode="single", true_pose=target["pose_rad"])
        for result in (accumulated, single):
            with self.subTest(method=result.method):
                self.assertIn(result.status, ("ok", "weak"), result.reason)
                # Agreement with the Day 1 ESTIMATE, not ground truth.
                self.assertLess(result.position_error_m, 0.05)
                self.assertLess(result.angle_error_deg, 1.0)

    def test_occluded_recording_is_not_localised(self):
        separate = common.SCANS_DIR / "classroom_session.json"
        if not separate.exists():
            self.skipTest("classroom_session.json absent")
        occluded = grid_loc.load_recorded_targets(separate)[0]["points_local"]
        result = localise_on_scans(occluded, reference_scans(self.scans))
        self.assertNotIn(result.status, ("ok",), result.reason)


class ProgramTests(unittest.TestCase):
    def test_application_mode_opens_the_scan_page(self):
        import desktop_app
        with unittest.mock.patch.object(desktop_app, "launch") as launch:
            main(CLASSROOM_FINAL)
        self.assertEqual(launch.call_args.kwargs["initial_page"], "Scan Localisation")

    def test_cancelled_picker_uses_the_default_session(self):
        module = sys.modules[__name__]
        with unittest.mock.patch.object(grid_loc, "choose_session_file",
                                        return_value=None), \
                unittest.mock.patch.object(module, "main") as fake_main:
            start_from_pycharm()
        fake_main.assert_called_once_with(None)

    def test_core_has_no_gui_imports(self):
        source = Path(__file__).read_text(encoding="utf-8").split(
            "# Automated tests")[0]
        for word in ("tkinter", "matplotlib", "serial", "filedialog"):
            self.assertNotIn(f"import {word}", source)

# ---------------------------------------------------------------------------
# 4. Program
# ---------------------------------------------------------------------------
def main(session_file=None):
    path = grid_loc.session_path(session_file or SESSION_FILE)
    if not path.is_file():
        print(f"Session file not found: {path}")
        return
    if RUN_MODE == "application":
        from desktop_app import launch
        launch(path, initial_page="Scan Localisation")
    elif RUN_MODE == "evaluation":
        separate = vector_loc.separate_recordings(path)
        print(f"=== Code 06 evaluation: {path.name}; separate recordings: "
              f"{', '.join(p.name for p in separate) or 'none'} ===")
        rows, summary = run_evaluation(path, separate)
        stamp = f"{path.stem}_{datetime.now():%Y%m%d_%H%M%S}"
        for written in save_evaluation(rows, summary, common.RESULTS_DIR,
                                       f"{RESULTS_FILE_STEM}_{stamp}", path):
            print(f"Saved {written}")
        shared = [row for row in rows if "grid_status" in row]
        comparison = common.RESULTS_DIR / f"{COMPARISON_FILE_STEM}_{stamp}.json"
        comparison.write_text(json.dumps({
            "created": datetime.now().isoformat(timespec="seconds"),
            "session": str(path), "frame": "Scan 1 = (0, 0, 0 deg)",
            "notes": EVALUATION_NOTES, "comparison": comparison_summary(rows),
            "runs": shared}, indent=1, default=float), encoding="utf-8")
        with open(comparison.with_suffix(".csv"), "w", newline="",
                  encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=COMPARISON_COLUMNS,
                                    extrasaction="ignore")
            writer.writeheader()
            for row in shared:
                writer.writerow({key: json.dumps(value) if isinstance(value, list)
                                 else value for key, value in row.items()})
        print(f"Saved {comparison} and {comparison.with_suffix('.csv')}")
    else:
        raise ValueError(f"RUN_MODE = {RUN_MODE!r}: use 'application' or "
                         f"'evaluation'")


def start_from_pycharm():
    """Ask for the session (as Codes 02-05 do; Cancel = default), then run."""
    chosen = grid_loc.choose_session_file() if ASK_FOR_SESSION_FILE else None
    main(chosen)


if __name__ == "__main__":
    start_from_pycharm()
