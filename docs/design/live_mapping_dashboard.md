# Live autonomous mapping dashboard (four views)

Status: Draft (part of `docs/specs/autonomy_phase_b.md`). Written 2026-11-04.

**Goal:** press Start Mapping and watch the four Workspace views fill in as the robot explores:

| View | Content |
|---|---|
| Scan Map | registered points and the trajectory |
| Occupancy Grid | unknown, free and occupied cells |
| Vector Map | the walls extracted so far |
| Current Scan | the newest sweep, robot-local |

Every view also shows the robot pose, the planned path, the exploration target, the
localisation quality and the mapping status.

**This phase:**
- A standalone, simulation-only dashboard in the autonomy worktree.
- The existing Workspace and all Stage 3 files stay untouched. Section 6 describes the later
  integration.

## 1. One mapping state, one gate
`MappingSession` (`autonomy/mapping_state.py`) is the **only** owner of accumulated map data:
- the accepted scans: local points, accepted pose, scan id, source;
- the Code 02 log-odds map;
- the Code 03 per-scan segments and the merged vectors;
- the trajectory.

It changes only through one call:

```
accept(frame: ScanFrame, result: LocalisationResult, pose) -> bool
```

**The gate:** a scan is integrated only when the localisation status is `ok` (the project rule:
only `ok`/`weak` draw a pose, and here only `ok` changes the map). Consequences:
- `weak`, rejected, cancelled and failed scans are **never** integrated.
- They go to a bounded `rejected` log, and the Current Scan view shows them marked "rejected".
- A localisation failure therefore cannot contaminate the Scan Map, the grid or the vectors.

All three global layers derive from the same accepted list, with the same poses:
- **Scan Map:** each accepted scan, through `part2.local_to_global` with its accepted pose.
- **Occupancy Grid:** `part2.integrate_scan_log_odds` on one fixed, pre-allocated spec, then
  `GridMap.from_part2`.
- **Vector Map:** `part3.preprocess_scans` / `segment_scans` / `fit_segments` per accepted scan
  (cached), then `merge_segments` over all of them. This is the Code 03 pipeline of
  `app_session.build_vectors`.

**Tests will prove consistency:**
- the snapshot grid equals Code 02's batch map of the snapshot's accepted scans;
- the vectors equal the batch Code 03 pipeline on the same scans;
- a rejected scan leaves every layer's hash unchanged.

## 2. Snapshots (the interface the views read)
`MapSnapshot` is frozen, and its arrays are read-only. One snapshot corresponds to one state
revision.

| Field | Content |
|---|---|
| `revision` | increments on every accepted scan |
| `layers` | per-layer revision: `scan_map`, `grid`, `vectors`, `current`, `overlay`, so a view redraws only if its layer changed |
| `scan_map` | decimated global points (5 cm voxel for display), with a scan id per point for colouring |
| `trajectory` | the accepted poses, in order |
| `grid` | `probability_u8`, `classified` (Code 02 FREE/UNKNOWN/OCCUPIED values), `extent`, `cell_size` |
| `vectors` | merged segments; `vectors_revision` = the map revision they were built from (may lag, see section 4) |
| `current` | `points_local`, `scan_id`, `source`, `accepted` / `rejected` + reason, timestamp |
| `robot` | `estimate` (last accepted pose), `predicted` (estimate + commanded motion since; drawn differently), `link_state`, `faults` |
| `localisation` | method, status, score, runner-up, runtime, verdict level (as `workspace_localisation.Outcome`) |
| `plan` | frontier clusters, selected target, planned path, inflated-obstacle mask (optional layer) |
| `status` | explorer state, coverage %, accepted and rejected counts, elapsed time, last message |

Simulator ground truth is **not** in the snapshot. The dashboard has an optional "truth"
overlay, but it comes from the simulator handle and is labelled "simulation truth". The explorer
never reads it.

## 3. Sources: simulated now, ESP32/LD06 later
`ScanSource` produces `ScanFrame(points_local, scan_id, timestamp, source, meta)`:
- `points_local` is N × 2, in metres, in the sensor frame (+X front, +Y left).
- `SimScanSource` uses `MockRobotLink.request_scan()`.
- `Esp32ScanSource` (later) uses the real link: SCAN_CHUNK → `ScanAssembler` → Code 01's
  `polar_to_cartesian`.

`MappingSession` and the views see only `ScanFrame`, so switching the source does not rebuild or
change any view. This follows the M2 roadmap's `ScanFrame` idea. It is defined inside `autonomy/`
for now, because `workspace_inputs.py` is a Stage 2/3 file, and it should be unified in M2.

## 4. Threads and update rates
Three threads; the GUI thread never runs mapping or control.

| Thread | Work | Blocking allowed |
|---|---|---|
| **Engine** | explorer state machine, `RobotLink`, localisation (~0.3 s), Code 02 integration (~9 ms), Code 03 per-scan segmentation (~1 ms) | yes (it owns the robot) |
| **Vector worker** | `merge_segments` (quadratic: 0.15 s at 20 scans, 0.55 s at 40), latest-wins: a request arriving while a merge runs replaces the queued one | yes |
| **GUI (Tk)** | polls a single-slot, latest-wins `SnapshotChannel` every 80 ms (the Workspace's own tick); redraws only the layers whose revision changed, through `draw_idle` | **no**: tick work ≤ 5 ms plus the draw |

**Commands:**
- Start, Pause, Resume and Stop go from the GUI to the engine through a thread-safe queue.
- **Stop is also applied directly:** `link.stop()` is thread-safe and called from the button
  handler, so STOP never waits behind a localisation in progress.

**Measured costs** (2026-11-04, this PC, Agg backend, 40-scan simulated classroom; the baseline
suite was running at the same time, so these are upper bounds). The figures come from the
scratch benchmark `bench_dashboard.py`; it will be copied into `results/autonomy_<ts>/` with the
step 9 evidence.

| Operation | Cost |
|---|---|
| Code 02 integrate one scan | 9 ms |
| Code 04 `localise_scan` (local) | 305 ms |
| Code 03 segment + fit | 0.9 ms per scan |
| Code 03 merge, 5 / 20 / 40 scans | 10 / 153 / 551 ms |
| Redraw of one view | 14-21 ms |
| Redraw of all four views | 70 ms |

**Proposed update rates.** These are caps, not targets: a view redraws only when its data
changed.

| What | Data changes | Redraw cap | Reason |
|---|---|---|---|
| Robot pose, path, target, status text | on every engine event; predicted pose at 10 Hz while moving | 10 Hz | cheap overlay artists; smooth enough to follow motion |
| Current Scan | per scan: about 0.3-1 Hz stop-and-scan; later LD06 at ~10 Hz | 5 Hz | a 10 Hz sweep is unreadable to a person; 5 Hz keeps the GUI below 25 % busy |
| Scan Map | per accepted scan | 2 Hz | points grow without bound (18 000 at 40 scans): append and decimate, never re-plot everything |
| Occupancy Grid | per accepted scan (9 ms) | 2 Hz | `set_data` on one image; integration is cheap but redraw is not free |
| Vector Map | per finished merge (may lag one or more scans) | 1 Hz | merge cost grows quadratically; the view is labelled "vectors from scan N" while it lags |
| Telemetry / link health | 5 Hz (firmware TELEMETRY period) | 5 Hz | matches the source |

**Worst case:** all four views change in one tick, which costs about 70 ms, under the 80 ms tick.
With these caps, average GUI load stays below about 30 %.

**To verify in step 9:**
- GUI tick time while localisation and merging run (target ≤ 5 ms excluding the draw);
- STOP latency from the button handler to the firmware model state (target: one engine step,
  ≤ 50 ms);
- the TkAgg costs, not only Agg.

**Later improvement, not this phase:** an incremental vector merge (merge only the new scan's
segments into the existing merged set). It needs a parity check against the batch merge before
it can replace it.

## 5. Files (added to the Phase B structure)
```
autonomy/scan_source.py      ScanFrame, ScanSource, SimScanSource; Esp32ScanSource declared, raises
autonomy/mapping_state.py    MappingSession (the gate), MapSnapshot, SnapshotChannel
autonomy/engine.py           engine + vector worker threads, command queue, direct STOP
autonomy/dashboard_layers.py pure Matplotlib layers: ScanMapLayer, GridLayer, VectorLayer,
                             CurrentScanLayer, OverlayLayer. Each has render(axes, snapshot)
                             and update(snapshot) -> bool (changed). No Tk import.
autonomy/dashboard.py        standalone Tk window: the four views (same titles and order as the
                             Workspace), Start Mapping / Pause / Resume / Stop, status panel.
                             Colours from ui_theme.PLOT (imported read-only)
test_autonomy_mapping_state.py  gate, no contamination, layer consistency, immutability
test_autonomy_engine.py         non-blocking GUI poll, STOP latency during localisation,
                                latest-wins, clean shutdown
test_autonomy_dashboard.py      layers render snapshots on Agg; unchanged layers skip redraw
```
`visualise.py` from the original plan is replaced by `dashboard_layers.py` + `dashboard.py`.

## 6. Later integration into the Workspace (after Stage 3 is verified; not now)
The layers are the seam. They draw on any Matplotlib `Axes`, so the Workspace's
`Viewport.axes` can host them without rebuilding the views.
1. **Live mode:**
   - `WorkspacePage` gets a "Mapping" Properties group: Start Mapping, Pause, Resume, Stop, a
     Source selector (Simulation / ESP32), and a status summary.
   - While live, the three global views show the snapshot layers instead of the session-derived
     maps.
   - The two are never mixed: switching modes clears one set of artists and shows the other.
2. **Polling:** `WorkspacePage.tick()` (80 ms, must never raise) reads the `SnapshotChannel` and
   calls `layer.update(snapshot)`. It stays cheap: no mapping work in the tick.
3. **Engine lifetime:**
   - The engine runs outside `Page.run`. `Page.run` allows one job per page, and that slot is
     used by captures and Stage 3 localisation.
   - Stage 3's localisation controls and Capture are disabled while live mapping runs, so they
     never compete for the link or the port.
4. **Shutdown:** `WorkspacePage.close()` stops the engine (STOP, then DISARM, then join with a
   timeout) before `Viewport.close()`.
5. **Theme:**
   - Pose colours follow `PLOT_STYLES`: estimate blue, predicted orange (the "initial guess"
     colour), runner-up purple. The recorded-pose green is not used in live mode.
   - New artists are recoloured in `set_plot_style`, as the Stage 3 handoff requires.
6. **Saving:** "Save mapped session…" writes the accepted scans and poses as a **new** version-2
   session, through Code 01's atomic writer. It never overwrites a file in `scans/`.
7. **Stage 3 reuse:**
   - The live localiser policy stays in `autonomy/localiser.py`.
   - A manual Stage 3 localisation of a live scan can use `workspace_localisation.make_job` on
     the snapshot's `GridMap`. Its result never enters `MappingSession` unless it passes the
     same gate.

**Open questions for that integration:**
- whether live mode replaces the current session or creates a new one;
- the port: the Capture field shares COM7 with the robot link, so only one owner is allowed at a
  time.
