"""Simulated scan sequences for the headless live localisation tracker.

World: Part 1's L-shaped CLASSROOM_WALLS (asymmetric, three obstacles).
Reference map: Code 04's own test fixture map, built by Part 2 from 8
simulated scans at fixed mapping poses (loc.fixture_mapping_scans). Live
scans come from OTHER poses along a route, with 10 mm range noise, through
localisation_common.simulate_scan (Part 1's ray caster, sensor frame).

The true pose of every scan is returned separately and is used ONLY to score
the tracker afterwards (live_localisation.evaluate); it is never an input.

Report (writes a new results/live_localisation_<timestamp>/ folder):
    python -m autonomy.localisation_sim [--step 0.10] [--turn 10]
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np

import localisation_common as common
from autonomy.live_localisation import LiveTracker, evaluate, loc, pose_error

WALLS = common.day1("part1").CLASSROOM_WALLS
# A loop through the free space of CLASSROOM_WALLS, clear of its three boxes (m).
ROUTE = ((1.0, 1.0), (5.5, 0.8), (7.2, 2.5), (6.0, 4.3), (3.2, 4.6), (2.5, 6.2),
         (1.0, 4.0), (1.0, 1.0))
RESULTS_DIR = Path(__file__).resolve().parent.parent / "results"


def reference_map():
    """Code 04's fixture GridMap of CLASSROOM_WALLS."""
    grid_map, _ = loc.build_grid_map(loc.fixture_mapping_scans())
    return grid_map


def route_poses(route=ROUTE, step_m=0.10, turn_deg=10.0):
    """True poses (x, y, theta_rad): turn in place at each waypoint in steps of
    at most turn_deg, then drive straight in steps of at most step_m."""
    poses = []
    heading = math.atan2(route[1][1] - route[0][1], route[1][0] - route[0][0])
    for (x0, y0), (x1, y1) in zip(route, route[1:]):
        target = math.atan2(y1 - y0, x1 - x0)
        turn = common.wrap_angle(target - heading)
        for k in range(1, int(math.ceil(abs(math.degrees(turn)) / turn_deg)) + 1):
            fraction = min(1.0, k * turn_deg / max(abs(math.degrees(turn)), 1e-9))
            poses.append((x0, y0, common.wrap_angle(heading + turn * fraction)))
        heading = target
        length = math.hypot(x1 - x0, y1 - y0)
        steps = int(math.ceil(length / step_m))
        for k in range(1, steps + 1):
            poses.append((x0 + (x1 - x0) * k / steps, y0 + (y1 - y0) * k / steps, heading))
    return poses


def scans_along(poses, noise_mm=10.0, seed=0):
    """Sensor-frame scans at the true poses (seeded noise)."""
    return [common.simulate_scan(WALLS, pose, noise_mm=noise_mm, seed=seed + i)
            for i, pose in enumerate(poses)]


def run(tracker, scans, start_seq=1):
    """Feed scans in order; returns the TrackUpdates."""
    return [tracker.update(points, seq=start_seq + i, timestamp_s=0.5 * i)
            for i, points in enumerate(scans)]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--step", type=float, default=0.10, help="metres between scans")
    parser.add_argument("--turn", type=float, default=10.0, help="degrees between scans")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)
    poses = route_poses(step_m=args.step, turn_deg=args.turn)
    scans = scans_along(poses, seed=args.seed)
    tracker = LiveTracker(reference_map())
    started = time.perf_counter()
    updates = run(tracker, scans)
    wall_s = time.perf_counter() - started
    summary = evaluate(updates, poses)
    summary.update(world="CLASSROOM_WALLS (Part 1)", reference="Code 04 fixture map",
                   start="unknown (global search)", step_m=args.step, turn_deg=args.turn,
                   noise_mm=10.0, seed=args.seed, wall_time_s=round(wall_s, 1),
                   truth_note="simulated ground truth; NOT a real-world accuracy figure")
    folder = RESULTS_DIR / time.strftime("live_localisation_%Y%m%d_%H%M%S")
    folder.mkdir(parents=True)
    (folder / "report.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    with (folder / "updates.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["seq", "search", "status", "state", "accepted", "est_x", "est_y",
                         "est_theta_deg", "true_x", "true_y", "true_theta_deg",
                         "position_error_m", "heading_error_deg", "runtime_ms"])
        for update, truth in zip(updates, poses):
            est = update.pose or (math.nan, math.nan, math.nan)
            err = pose_error(update.pose, truth) if update.accepted else (math.nan, math.nan)
            writer.writerow([update.seq, update.search, update.status, update.state,
                             int(update.accepted), *np.round(est[:2], 4),
                             round(math.degrees(est[2]), 2), *np.round(truth[:2], 4),
                             round(math.degrees(truth[2]), 2), round(err[0], 4),
                             round(err[1], 2), round(update.runtime_ms, 1)])
    print(json.dumps(summary, indent=2))
    print(f"wrote {folder}")


if __name__ == "__main__":
    main()
