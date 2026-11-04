# M1 Stage 3 handoff: Workspace localisation of an independent scan

> **Status update (2026-11-04, later the same day):** Stage 3 has now been **implemented
> (uncommitted)**, but Gate 3 is **not yet accepted**. For the current state, read
> `docs/SESSION_HANDOFF.md` and `results/m1_gate3_20261104_092409/REPORT.md`. The brief below is
> kept unchanged as the original specification, so its "not started" line and line numbers are
> historical.

Written 2026-11-04 for a fresh Claude Code session. Stage 3 has **not** been started.
Line numbers refer to the working tree on 2026-11-04. Re-read each location before editing.

Project: `C:\Users\stuar\Desktop\SLAM`. Python 3.13, Tkinter, Matplotlib, NumPy, pyserial.
Interpreter: `.\.venv\Scripts\python.exe`.

Hardware: LD06 → ESP32 → CH340 USB serial, **COM7**, 115200 baud. The code's default port is
COM5 (`part1.SERIAL_PORT`); type COM7 in the Port field. A real 456-point capture has worked.
If a port reports "Access is denied", another program has it open: quit the Arduino IDE
completely (including its `serial-discovery`/`arduino-cli` helpers) and replug the USB cable.

---

## 1. Git state
- Branch `feature/ui-redesign`, pushed to `origin/feature/ui-redesign`. `main` =
  `origin/main` = bd8b842.
- **All Stage 0-2 code and tests are committed** in **e5f1a1d** "M1: UI polish, localisation
  adapter and Stage 3 handoff" (2026-11-04), on top of 8ed4457. Its 10 files:

  | File | Change |
  |---|---|
  | desktop_app.py | Stage 1 tokens |
  | ui_fonts.py | comment |
  | ui_theme.py | Stage 1 |
  | workspace_inputs.py | Stage 2 |
  | workspace_page.py | Stage 0 fix and Stage 1 |
  | workspace_localisation.py | new, Stage 2 |
  | test_gui_workspace.py | Stage 0 and Stage 1 tests |
  | test_ui_theme.py | new |
  | test_workspace_localisation.py | new |
  | this handoff | new |

  In total: +1183/−24 lines.
- A documentation-only commit follows e5f1a1d. It holds the template sync (`AGENTS.md`,
  `CLAUDE.md`, `.template-version`, `workflows/`, `docs/STATUS.md`, the `docs/` folder
  placeholders) and this Git-state update. Run `git log --oneline -3` to see the current HEAD.
  Nothing is staged and there are no stashes.
- **Untracked, not part of M1. Leave them alone:**
  - `results/` (evidence)
  - `.idea/`, `__pycache__/`, `hang.log`
  - `LD06.zip` (third-party library; MIT per its README, but no LICENSE file)
  - the three French TP `.md` files
  - `scans/classroom_extended.json` (byte-identical to `classroom_final.json`)
  - `assets/fonts/` (`README.md`, `.gitignore`, and an ignored `local/` that holds the font)
- **Open Git issue (the user decides):** 8ed4457 added, and the branch still tracks,
  `assets/engineering-plot-regular_freefontdownload_org.zip`, whose redistribution rights are
  unresolved. It is on `feature/ui-redesign` and its remote, not on `main`, and there is no root
  `.gitignore`. Proposed fix, without rewriting history:
  1. Add a root `.gitignore`: `__pycache__/`, `.idea/`, `.venv/`, `hang.log`,
     `assets/fonts/local/`, `assets/*.zip`.
  2. `git rm --cached` the zip.
  3. Squash-merge into `main`.
  4. Purging it from the remote history would need a force-push, which needs the user's
     explicit approval.

## 2. Work completed
Evidence and logs are under `results/m1_gate0_20261104_041714/`,
`results/m1_gate1_20261104_051315/` and `results/m1_gate2_20261104_080600/`. Each has a report.

**Stage 0: baseline and the close/resize fix.**
- Hashes of the five files in scans/, LD06.zip and the Day 2 comparison JSON/CSV are in
  `results/m1_gate0_20261104_041714/hashes_before.json`. They were unchanged at every gate since.
- Bug: closing the app within 250 ms of a viewport resize raised
  `TclError: can't delete Tcl command`.
  - Cause: `Viewport.on_configure` schedules its callback on the canvas widget, but
    `Viewport.close` cancelled it through the frame.
  - Fix: `workspace_page.py:184-189` cancels via `self.canvas.get_tk_widget()`.
  - Regression test: `test_gui_workspace.WorkspaceCloseResizeTests`. It fails 3 of 3 attempts
    on 8ed4457 and passes on the fix.
- The Day 2 evaluation was re-run in memory: 24 of 24 rows match
  `results/day2_localisation_comparison_classroom_final_20261103_172851.json` in every field
  other than runtimes.

**Stage 1: UI consistency (approved parts A, B and C).**
- `ui_theme.PlotPalette` / `PLOT` (`ui_theme.py:92-131`) names every plot colour.
  `Theme.danger_pressed` was added.
- The Workspace pose vocabulary matches pages 04-06. `PLOT_STYLES`
  (`workspace_page.py:42-54`) has these keys for both "Dark" and "Light paper":

  | Key | Meaning | Colour |
  |---|---|---|
  | `recorded` | recorded (Day 1) pose | green #2CA02C |
  | `initial` | initial guess | orange #FF7F0E |
  | `estimate` | localised estimate | #1F77B4 on paper / #6FA8DC on dark |
  | `alternative` | ambiguous runner-up | #9467BD / #B39DDB |
  | `axis` | +X cross on the Current Scan | grey |

  Rejected results draw no pose.
- The tab-group label went from 7 pt to 8 pt.
- Pages 01-06 are pixel-identical apart from the tab strip.
- Tests: `test_ui_theme.py` (headless) and `test_gui_workspace.WorkspacePoseColourTests`.
- Open observation: the green highlight of the selected scan's points can blend with scans
  3/13/23 (tab:green). A ring or a larger point size was suggested; this is not approved.

**Stage 2: the adapter** (`workspace_localisation.py`, pure: no tkinter or desktop_app import).
It is bit-identical to the page code paths (see section 8) and touches neither the pages nor
Codes 01-06.

## 3. Integration points
### workspace_page.py (`WorkspacePage`, line 215)
| Lines | Item | Notes |
|---|---|---|
| 20-31 | `GLOBAL_VIEWS = ("scan", "grid", "vector")`, `VIEW_TITLES`, `VIEW_HELP` | `"current"` is the 4th view |
| 42-54 | `PLOT_STYLES` | the pose colours above |
| 68-190 | `Viewport` | `.axes`, `.caption` (header label; `set_text`), `.empty` (centre text), `.draw()` (`draw_idle` only when mapped), `.fit()`, `.home` |
| 222-331 | `__init__` | Properties groups: `heading(self.controls, "Current scan")`, "Display", "View", "Capture (LD06)" (`capture_controls(self.captured, …)`; `capture_buttons[0]` is Primary.TButton), then `expander("Details")`. Map requests use `app.derived(MapRequest(self, "grid", "grid"), "grid", build_grid, self.grid_ready)` and the same for `"vectors"` |
| 447-457 | `update_busy` / `set_busy` | page busy = `capture_busy or pending map requests`. `Page.run` sets `capture_busy` for any job, so a capture and a localisation can never overlap on this page |
| 475-491 | `grid_ready(data)` | `data = (GridMap, log_odds)`, the `"grid"` cache entry. `data[0]` is the Code 04 reference |
| 521-532 | `vectors_ready(data)` | `data = (scans, clusters, records, merged, items)`. `data[3]` feeds `VectorMap.from_merged` |
| 533-548 | `create_selection_artists` | recorded-pose `pose_artists[key] = (marker, heading)` per global view; `highlight`, `current_points`, `axis_cross`, `axis_label` |
| 569-609 | `refresh_selection` | draws the Current Scan from `self.source`: `SOURCES = ("Selected recorded scan", "Latest capture")`, using `model.workspace_inputs.latest_capture`. Sets `selection_signature` |
| 613-632 | `set_plot_style` | recolours artists only and never rebuilds maps. New estimate artists must be recoloured here too |
| 633-637 | `captured(points)` | switches the source to "Latest capture" |
| 638-645 | `tick()` | runs every poll (80 ms); must stay cheap and must never raise. Returns early if `self.revision != model.revision` |
| 646-653 | `close()` | cancels `link_job`/`layout_job`, closes the viewports, then `Page.close` (sets `self.stop` if present, cancels capture) |

### desktop_app.py (shared machinery; do not restructure)
- **Helpers:** `tip` 97, `button` 102, `note` 113, `group` 133, `heading` 143,
  `expander` 162, `row` 185, `set_text` 197.
- **Verdicts:**
  - `verdict(result)` 203: ok → "Localised · ok"; weak → "Plausible, not confirmed · weak";
    otherwise "Not localised · <status>".
  - `VERDICT_STYLES` 214.
  - `show_verdict_chip(page, result)` 1021 needs a `page.verdict_chip` label.
  - `RESULT_ADVICE` 968: advice text per status.
  - `run_footer(page, command, tooltip)` 999 builds the Localise and Cancel Search row the pages
    use. It expects `page.cancel_search`.
- **`Page`:**
  - `set_busy` 491
  - `run(label, work, done, controls)` 512: one job per page; a second press while busy returns
    False
  - `capture_controls` 522
  - `capture(done)` 552: publishes to `model.workspace_inputs.publish_capture(points)` before
    calling `done`
  - `tick` 580
  - `close` 583
- **`PoseInputs(parent, get_pose, set_pose, apply_text, tooltip)`** 598: X m / Y m / heading °
  in Day 1 units. Its `sync()` 629 is popdown-safe. Reuse it; never use `focus_get()`.
- **Pages, for reference only:** `LocalisationPage` 1026 (Code 04), `NewScanLocalisationPage`
  1392 (`localise` 1589, `tick` 1624, `close` 1632, generation counter, progress in 5 % steps),
  `VectorLocalisationPage.vectors_ready` 1728 (the `"vector_reference"` cache with a revision
  check), `measure_lines` 1767 (05) and 1872 (06).
  `NewScanLocalisationPage.measure_lines(None, result)`-style unbound calls work for 05 and 06
  (they read only `result.details`).
- **`DesktopApp`:**
  - `last_runtime` 1921 is shown in the status bar ("Last localisation: …", 2433). Set it after
    a run, as the pages do (1309, 1613).
  - `show_page` 2258: a page whose `revision` is stale is closed and recreated on return.
  - `derived(page, key, build, done)` 2281: shared, revision-scoped builds.
  - `submit` 2314: a daemon thread.
  - `poll` 2328: every 80 ms; reschedules in `finally`. Do not change it.
  - `deliver_results` 2362: on the Tk thread; clears `owner.busy` before calling `done`.

### app_session.py
- `SharedSession` 47:
  - `scans` holds the **visible** mapping scans, the reference set for every method.
  - `cache` is the revision-scoped derived data: `"grid"`, `"vectors"`, `"vector_reference"`,
    `"global_points"`, `"binary"`.
  - `revision` changes and `cache.clear()` happens in `refresh()` (74-82) whenever the session
    digest changes.
  - `workspace_inputs = WorkspaceInputs()` (line 58) lives as long as the session object,
    across page re-creation.
- `build_grid` 96 and `build_vectors` 103.
- `CaptureAdapter.capture(port, baud)` 134: opens the port, `reset_input_buffer`, reads ONE scan,
  closes the port. It never writes. It returns an N×2 array in **sensor-frame metres**
  (`teleplot_to_sensor_frame`: (0,0) dropped, mm→m, Y negated so +Y is left).

### workspace_inputs.py
- `SCAN_SOURCES = ("usb_serial", "recorded", "wifi")` (13). Wifi is reserved for M3.
- `ScanFrame(points_local, timestamp_s, label="Fresh LD06 capture", source="usb_serial")` (17).
  Frozen, a read-only N×2 finite copy.
- `EstimatedRobotPose(x_m, y_m, heading_rad, timestamp_s, method="", status="",
  target_label="")` (36).
- `WorkspaceInputs` (54):
  - `latest_capture`, `latest_estimate`, and the inactive protocols `live_scan_source` and
    `robot_pose_source`.
  - `publish_capture(points)` (66).
  - `publish_estimate(pose_rad | None, method, status, target_label)` (70): only `ok` and
    `weak` poses are published; a rejected status raises; `None` clears. It never touches the
    session.

### workspace_localisation.py
| Lines | Item |
|---|---|
| 30-37 | `METHODS` (`grid`/`vector`/`scans`), `SEARCHES` (`global`/`local`, with labels), `SCAN_REFERENCES` (`accumulated`/`single`), `VECTOR_REFERENCE_KEY = "vector_reference"`, `SCORE_LABELS` |
| 46-83 | `Request(method, search, points_local, target_label, guess_day1=None, scan_reference="accumulated", reference_id=None, reference_revision=0)`. Frozen and validated. `signature` (76) covers every input except the label |
| 86-134 | `Outcome(request, result)`: see the list below |
| 137 | `check_target(points, reference_scans)` → None or a message (invalid array, or self-matching with the ids named) |
| 154 | `load_targets(path)` → list of `{name, id, points_local}`, poses dropped. Raises ValueError |
| 163 | `cancellable(method, search)`: False only for grid + local |
| 168 / 180 / 187 | `grid_initial_pose`, `grid_points`, `scan_references` (input preparation, as the pages) |
| 194 | `make_job(request, reference, progress=None, should_stop=None)` → a no-argument callable returning an `Outcome`. Run it through `Page.run` |

`Outcome` exposes:
- `.result`, the original `LocalisationResult`, unchanged
- `.status`, `.accepted` (ok), `.plausible` (weak), `.verdict_level` (`ok`/`weak`/`bad`)
- `.pose` (radians; None unless ok/weak), `.pose_day1`
- `.candidate_pose` (the method's best pose, even when rejected)
- `.runner_up_pose` (ambiguous only)
- `.score_label`

## 4. Fresh USB captures as independent targets
- Capturing from the Workspace, or from pages 04, 05 or 06:
  1. `Page.capture` runs `CaptureAdapter.capture` on a worker.
  2. On the Tk thread, `publish_capture` stores a `ScanFrame` (source `usb_serial`) in
     `model.workspace_inputs.latest_capture`.
  3. Then the page's `done` runs. The session is **not** changed, so the capture is independent
     of every reference map.
- **Capturing on 01 Scan is different.** `ScannerPage.add_capture` (desktop_app.py 768; wired at 701)
  adds the scan to the session. That changes the revision, rebuilds the maps, and makes the scan
  part of the reference. Because `Page.capture` publishes it as `latest_capture` too,
  `check_target` will then refuse it as self-matching. That refusal is correct; keep it.
- A capture lives only in memory. It is lost when another session is opened and is never written
  to `scans/`.
- **Recorded targets:** `load_targets(path)` reads a separate session file and keeps only its
  local points. For example, `scans/classroom_session.json` scan 3 is a good independent target
  on `classroom_final.json`.
  - Wrap the chosen scan in `ScanFrame(points, time.time(), label, source="recorded")` if it
    should share the capture path.
  - A file that is byte-identical to the open session (such as `classroom_extended.json`) gives
    only self-matching targets.
- **Always** call `check_target(points, model.scans)` before running, and show its message
  instead of running.

## 5. Calling Codes 04, 05 and 06 through the adapter
```python
import workspace_localisation as wl
request = wl.Request(method, search, frame.points_local, frame.label,
                     guess_day1=pose_inputs_value if search == "local" else None,
                     scan_reference=ref_mode, reference_id=ref_id,
                     reference_revision=self.model.revision)
reference = {"grid": grid_data[0],                  # cache "grid" (via app.derived)
             "vector": vector_map,                  # cache "vector_reference"
             "scans": self.model.scans}[method]
job = wl.make_job(request, reference, progress=report, should_stop=stop.is_set
                  if wl.cancellable(method, search) else None)
self.run(label, job, done, controls=(localise_button,))   # Page.run → app.submit
```
- **Grid:** obtain the GridMap through `app.derived(..., "grid", build_grid, cb)`. The Workspace
  already requests it, and `self.grid_arrays` holds display arrays, not the GridMap, so keep a
  reference to `data[0]` in `grid_ready`.
- **Vector:** reuse `model.cache["vector_reference"]` if present. Otherwise build
  `vloc.VectorMap.from_merged(vectors_data[3])` on a worker, and store it under that key only if
  `revision == model.revision` (copy the logic in desktop_app.py 1728-1743). Never build it on
  the Tk thread.
- **Scans:** `model.scans` (visible scans). For "single", pass a `reference_id` from a chooser.
- `progress(fraction)` is called on the worker. Store the fraction in an attribute and display it
  from `tick()` in 5 % steps, as the pages do. Never touch Tk from the worker.

## 6. Publishing and displaying one estimated pose
- In `done(outcome)` (Tk thread):
  1. **Drop stale results.** If `outcome.request.signature != current_request().signature`, or
     the page's generation changed, show "Target, method, guess or reference changed during the
     search. Press Localise again." and draw nothing.
  2. Store `self.outcome`.
  3. Call `model.workspace_inputs.publish_estimate(outcome.pose, METHODS[m], outcome.status,
     label)`. `outcome.pose` is None for rejected results, which clears the estimate.
  4. Set `app.last_runtime`.
  5. Redraw the overlays.
- **Artists:** create them once in `create_selection_artists`, one set per `GLOBAL_VIEWS` key:
  - estimate marker and heading
  - aligned target points (`transform_points(points, pose)` from `localisation_common`)
  - initial-guess marker (local search only)
  - runner-up marker (ambiguous only)

  Use `PLOT_STYLES[style]["estimate" | "initial" | "alternative"]`, and recolour them in
  `set_plot_style`. All three global views get the **same** pose tuple.
- **Weak results:** hollow marker and dashed heading; the caption says "plausible, not
  confirmed".
- **Rejected results:** no pose marker and no aligned points. Each global viewport caption (or
  the verdict chip) reads "Not localised · <status>". Showing `candidate_pose` is only allowed
  behind an explicit, clearly labelled option; it is not approved yet.
- **Current Scan (4th view):** show the target's local points exactly, with caption
  "Local · <target label>". Never apply the global pose there.
- **Clear Result:** remove the artists, `publish_estimate(None, …)`, reset the chip.
- **Never** rebuild maps, change `model.revision`, edit the session, write files, or touch the
  04/05/06 pages' state. Updating artists plus `viewport.draw()` is enough.

## 7. Workspace controls to add (compact "Localisation" group)
Place the group between "Current scan" and "Display", and put long text under "Details".
1. **Target:** "Latest capture" / "Scan from a separate recording". "Load Recorded Scan…" opens
   a file dialog in `common.SCANS_DIR`, followed by a scan chooser. The capture itself stays in
   the existing "Capture (LD06)" group.
2. **Method:** 04 Grid / 05 Vector / 06 Scan. For 06, add accumulated/single and a reference-scan
   chooser.
3. **Search:** Global / Local. In Local mode show `PoseInputs` (Day 1 units) and a "Use selected
   scan's recorded pose as guess" button.
4. **Actions:** **Localise** (Primary.TButton; make "Capture New Scan" a normal button so there
   is one primary action), **Cancel**, **Clear Result**, **Fit View** (`fit_all`).
   - Enable Cancel only when `wl.cancellable(...)`.
   - For 04 local, show "Code 04 local search cannot be stopped; the result will be discarded",
     then discard the result through the generation check. Never report it as "cancelled".
   - A second Localise press while busy is ignored by `Page.run`.
5. **Result:** a verdict chip (`show_verdict_chip`), then X m / Y m / heading ° (from
   `pose_day1`), `score_label` and score (with the runner-up score if present), runtime,
   method, target and the status reason. `RESULT_ADVICE[status]` and the 05/06
   `measure_lines` go under Details.
6. Under Details, a disabled "Tracking" note: "Continuous tracking is not implemented yet."
   Do not add any control that implies live or Wi-Fi operation.

## 8. Tests, baselines and known instability
**Test status when this handoff was written:**

| Suite | Command (from the project folder) | Result |
|---|---|---|
| a | `.\.venv\Scripts\python.exe -m unittest 01_lidar_visualizer 02_grid_mapping 03_vector_mapping localisation_common 04_grid_localisation 05_vector_localisation 06_scan_localisation test_application test_diagnose_lidar_serial test_workspace test_ui_theme test_workspace_localisation -v` | 431, OK (about 5.5 min) |
| b | `.\.venv\Scripts\python.exe -m unittest test_gui_map_display test_gui_workspace -v` | 12 tests; see the instability below |
| c | `.\.venv\Scripts\python.exe gui_smoke_test.py` | PASS |

**Regression tests already in place:**
- `test_gui_map_display`: the popdown and poll-survives tests, and `MissingFontStartupTests`.
- `test_gui_workspace`: `WorkspaceCloseResizeTests` and `WorkspacePoseColourTests`.
- `test_ui_theme`.
- `test_workspace_localisation` (19 tests):
  - page parity for 04 global/local via a real `AutoAlignViewer`, and 05/06 via the pages' own
    `compute`/`references`
  - self-matching, invalid and missing targets
  - signature staleness and cancel behaviour
  - verdict parity with `desktop_app.verdict`
  - `publish_estimate`

**Numerical baselines (do not re-run the full Day 2 evaluation unless a regression needs it; it
takes about 41 min):**
- Day 2: 24 of 24 rows match the 20261103 file (Gate 0).
- Direct calls (`results/m1_gate0_20261104_041714/baseline_direct_calls.json`), reference
  `classroom_final.json`, targets `classroom_session.json` scans 1-3:

  | Target | 04 | 05 | 06 accumulated | Pose |
  |---|---|---|---|---|
  | Scan 3 | ok | ok | weak | about (−4.48, 3.54, −102.5°) |
  | Scans 1-2 | low_score | low_score | inconsistent | — |

  Runtimes: 04 0.2-1.9 s, 05 0.2-2.3 s, 06 2.2-13.4 s.
- The adapter reproduces all 18 of those runs exactly
  (`results/m1_gate2_20261104_080600/adapter_vs_gate0.json`).

**Known GUI instability (unresolved):**
- At Gate 2, suite b failed in 5 of 5 runs on the current tree, and in 1 of 3 runs on a scratch
  copy of the Gate 1 code, in the same time window.
- Every failure was on-screen geometry or timing: map not yet visible, viewport not yet mapped,
  no pending resize, equal-aspect check, rendered size ≠ canvas size. One assertion was
  identical on both trees.
- Runs took up to 194 s instead of about 77 s while the desktop was in use.
- `test_workspace_1920x1080` has been intermittent since Phase 0.
- **Re-run suite b first, with the desktop idle**, to establish a clean baseline before Stage 3.
  If it still fails, investigate before adding GUI tests. Never weaken an existing test.
- **Other quirks:**
  - Suite b prints a harmless `invalid command name "…quit"`. This comes from an uncancelled
    watchdog at `test_gui_map_display.py:99`.
  - New GUI tests must wait until pages are idle before `app.close()`.
  - `test_gui_workspace.py` and `gui_smoke_test.py` overwrite `results/workspace/` and
    `results/gui_review/`. Copy them into a new timestamped folder before running.

## 9. Safety constraints
- **Git:** branch `feature/ui-redesign` only. No commit, push, reset, clean, stash, merge,
  rebase, or `git add .` / `-A`. Change the index only after the user approves an exact file
  list.
- **Recordings:** never modify `scans/` or `LD06.zip`. Verify against
  `results/m1_gate0_20261104_041714/hashes_before.json` at the end.
- **Algorithms:** do not edit Codes 01-06 or `localisation_common.py`. That includes their maths,
  thresholds, verdict rules, configs and output semantics. Localisation goes only through
  `workspace_localisation.make_job`.
- **Coordinate conventions:**
  - global frame = Scan 1 at (0, 0, 0°), X right, Y up, CCW positive
  - sensor frame: +X forward, +Y left, metres
  - the adapter takes guesses in Day 1 units (degrees) and `Outcome.pose` in radians
- **Caches:** never rebuild maps for a pose change. Use `app.derived` and the existing cache
  keys. Keep the reference fixed during a run, using `reference_revision` and the signature.
- **Tk polling fix:**
  - no `focus_get()`, `winfo_containing()` or unguarded `nametowidget()` on polled or event
    paths
  - `DesktopApp.poll` keeps its `finally`
  - `tick()` must be cheap and must never raise
  - no Tk calls from workers
  - one job manager only (`Page.run` / `app.submit`)
- **Pages:** keep all seven pages, Ctrl+1..7, the four-panel Workspace, maximise/restore and
  linked pan/zoom (global views only).
- **Hardware:** no new dependencies, no firmware changes. Open a serial port only with the
  user's approval and only through `CaptureAdapter` (read-only, never writes).
- **Workflow:** stop at each gate. Report the exact commands and results, and keep verified
  work separate from untested work.

## 10. Stage 3 checklist
**Implementation (workspace_page.py; small additions only):**
- [ ] Re-run suite b with the desktop idle and record a clean baseline, or report the failures.
- [ ] Back up `results/gui_review/` and `results/workspace/` into
      `results/m1_gate3_<timestamp>/`.
- [ ] `grid_ready`: keep `self.grid_map = data[0]`. Vector reference: get or build
      `cache["vector_reference"]` on a worker, with the revision check.
- [ ] Localisation group: target, method (with the 06 reference mode and chooser), search,
      PoseInputs, Localise / Cancel / Clear Result / Fit View, verdict chip, result text.
- [ ] `current_request()` builds a `wl.Request` from the controls, with
      `reference_revision=model.revision`. Clear the result whenever its signature changes
      (checked cheaply in `tick`, or on each control event).
- [ ] Localise:
  - `check_target`, then show the message or start `Page.run(make_job(...))`
  - the generation counter
  - a `threading.Event` for cancellable searches; extend `close()` so the event is set
- [ ] `tick()`: progress in 5 % steps, plus the staleness check, without raising.
- [ ] `done`: the staleness check, `publish_estimate`, `app.last_runtime`, overlay redraw, and
      Current Scan showing the target's local points.
- [ ] Overlay artists on Scan / Grid / Vector, recoloured in `set_plot_style`:
  - ok: filled marker
  - weak: hollow marker, dashed heading
  - rejected: nothing
  - local search: initial-guess marker
  - ambiguous: runner-up marker
- [ ] Update `docs/MULTI_VIEW_WORKSPACE.md` (estimate vs recorded pose; no tracking).

**Verification (new GUI tests in `test_gui_workspace.py` or a new
`test_gui_workspace_localisation.py`, using real Tk and the `drive()` pattern):**
- [ ] Workspace vs page parity: the same held-out target (`classroom_session.json` scan 3) on
      page 05 and in the Workspace (05, global) gives bit-identical poses. Repeat for 04 and 06
      accumulated.
- [ ] All three global views draw the estimate at identical coordinates. The Current Scan
      offsets equal the target's local points exactly.
- [ ] Global and local modes. A rejected target (scan 1 or 2) shows no pose. Self-matching
      (a `classroom_final` scan) is refused with a message.
- [ ] A missing or invalid recording file shows an error without a crash.
- [ ] Changing method, target or guess during a run discards the result. A session change
      (revision) discards it too.
- [ ] A second Localise while busy is ignored. Localise during a capture is refused.
- [ ] Cancel during 04/05/06 global gives "cancelled" with nothing drawn. 04 local is not
      cancellable and its result is discarded if stale.
- [ ] Switching pages, resizing, maximise/restore and closing during a run all work without
      TclError. The poll still delivers after an injected tick exception.
- [ ] Popdown safety: open the Method combobox popdown and poll for 1 s with no errors.
- [ ] Ctrl+1..7, four panels and suites a/b/c pass.
- [ ] Hashes are unchanged.
- [ ] No `*_measured_poses.json` file is created.
- [ ] The session digest and revision are unchanged.
- [ ] Screenshots at 1440×900 and 1920×1080, dark and light, before and after an ok, a weak and
      a rejected result.
- [ ] Gate 3 report: changed files with line ranges, test counts, parity evidence, screenshots,
      runtimes per method, limitations, and verified vs untested work. **Stop for approval.**
