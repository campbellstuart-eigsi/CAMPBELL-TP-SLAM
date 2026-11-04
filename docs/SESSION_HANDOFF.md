# Session handoff

_Last updated: 2026-11-04_ · _Session focus: M1 Stage 3 (Workspace localisation of an independent scan) up to the Gate 3 review_

## Project overview and current state
LiDAR SLAM Workbench (university project): an LD06 LiDAR read through an ESP32 over USB serial,
with Tkinter/Matplotlib programs for mapping (Codes 01-03) and localisation (Codes 04-06), plus a
seven-page desktop app with a four-view Workspace.

**M1 Stage 3 is implemented, but Gate 3 is NOT yet accepted.** The code and tests are written
and uncommitted. Most verification passed. The GUI suite b has not passed three times in a row,
because the desktop kept minimising the test windows (see "GUI minimisation investigation"). Two
things are still open: a clean GUI verification on an idle desktop, and your Gate 3 approval.

## Completed milestones
- Day 1 mapping (Codes 01-03) and Day 2 localisation (Codes 04-06), up to bd8b842 (`main`).
- M1 Stage 0 (baseline and the close/resize fix), Stage 1 (UI consistency) and Stage 2
  (`workspace_localisation.py` adapter), committed in e5f1a1d (2026-11-04).
- M1 Stage 3: implemented and verified as below, **not accepted, not committed**.

## Work performed this session
- **Stage 3 implementation**, all in `workspace_page.py` (line ranges in the Gate 3 report):
  - A "Localisation" group placed between "Current scan" and "Display", with:
    - a target: the latest capture, or one scan of a separate recording, loaded with "Load
      Recorded Scan…" and a scan chooser
    - a method: 04 Grid, 05 Vector, or 06 Scan (06 offers accumulated or single, with a
      reference-scan chooser)
    - a search: Global, or Local with `PoseInputs` and "Use Selected Scan's Recorded Pose"
    - the buttons Localise (the only Primary button; "Capture New Scan" is now a normal button),
      Cancel (enabled only for cancellable searches), Clear Result and Fit View
    - a verdict chip and the result text
    - under Details: the advice per status, and a disabled note "continuous tracking is not
      implemented yet"
  - Every run goes through `workspace_localisation`:
    - `check_target` refuses self-matching or invalid targets
    - `make_job` is run with `Page.run`
    - a generation counter plus a `Request.signature` check discards stale results (changed
      target, method, guess, reference or session revision)
    - `publish_estimate` is called only for `ok` and `weak` results
    - `app.last_runtime` is set
  - Overlays use the same pose on Scan, Grid and Vector:
    - `ok`: filled blue marker
    - `weak`: hollow marker with a dashed heading
    - rejected: nothing drawn
    - local search: orange guess marker
    - the overlays are recoloured by the plot style
  - The Current Scan has a third source, "Localisation target", which shows the target's own
    sensor-local points.
  - The 05 VectorMap is built once through `app.derived` under the shared `"vector_reference"`
    cache key, then the run starts automatically.
  - `close()` releases the page's Tk Variables (`self.stop` is set by `Page.close`).
- **New tests:** `test_gui_workspace_localisation.py` (6 real-Tk tests; see the Gate 3 report for
  coverage).
- **Approved GC teardown fix:**
  - `test_gui_map_display.collect_closed_app()` calls `gc.collect()` on the main thread after
    every app close.
  - It is called in `MapDisplayTests.tearDown`, `MissingFontStartupTests`,
    `WorkspaceGuiTests.tearDown` and `WorkspaceCloseResizeTests.close`.
  - No assertions or timeouts were changed.
  - Cause: Tk Variables of earlier tests' closed apps were freed by the cyclic GC on a worker
    thread, about 1 s each, which made `test_builds_shared…` time out.
- **Docs:**
  - `docs/MULTI_VIEW_WORKSPACE.md`: a one-shot localisation section, and estimate vs recorded
    pose.
  - `docs/STATUS.md` updated.
  - This file.
  - A status note at the top of `docs/M1_STAGE3_HANDOFF.md`.
- **Evidence:** `results/m1_gate3_20261104_092409/` holds:
  - `REPORT.md`
  - the logs
  - 16 screenshots
  - `geometry_investigation/`
  - `scripts/` (the diagnostic and screenshot scripts, copied from the session scratchpad)
  - `backup/` (`results/workspace` and `results/gui_review` before the test runs)

## Exact files modified (uncommitted)
| File | Change |
|------|--------|
| `workspace_page.py` | Stage 3: +425/−12 lines (`git diff --numstat`) |
| `test_gui_workspace_localisation.py` | new (untracked), 314 lines, 6 GUI tests |
| `test_gui_map_display.py` | +11: `collect_closed_app()` and 2 calls (approved) |
| `test_gui_workspace.py` | +2: calls to `collect_closed_app()` (approved) |
| `docs/MULTI_VIEW_WORKSPACE.md` | +30/−5: Stage 3 section; wording on estimate vs recorded pose |
| `docs/STATUS.md` | +43/−47: current state |
| `docs/SESSION_HANDOFF.md` | new, this file |
| `docs/M1_STAGE3_HANDOFF.md` | +6: a status note at the top (the original brief is unchanged) |

Not modified: Codes 01-06, `localisation_common.py`, `desktop_app.py`, `app_session.py`,
`workspace_localisation.py`, `workspace_inputs.py`, `gui_smoke_test.py`, `scans/`, `LD06.zip`.

## Important files and locations
| Path | Purpose |
|------|---------|
| `workspace_page.py` | Workspace page; the Stage 3 code is mostly `build_localisation_controls` … `tick_localisation` |
| `workspace_localisation.py` | the Stage 2 adapter; the only route to Codes 04/05/06 |
| `test_gui_workspace_localisation.py` | Stage 3 GUI tests |
| `docs/M1_STAGE3_HANDOFF.md` | the original Stage 3 brief, integration points and safety constraints |
| `results/m1_gate3_20261104_092409/REPORT.md` | Gate 3 report (changes, tests, parity, decisions) |
| `results/m1_gate3_20261104_092409/geometry_investigation/` | minimisation evidence |
| `results/m1_gate3_20261104_092409/scripts/` | `diagnose_mapping.py`, `bare_window_probe.py`, `trace_variables.py`, `trace_with_collect.py`, `page_refcount_probe.py`, `stage3_screenshots.py` |
| `results/m1_gate0_20261104_041714/hashes_before.json` | SHA-256 of the 8 baseline files |

## Tests and verification
All runs were on 2026-11-04, from the project folder, on the current working tree unless noted.

| Command or check | Result |
|------------------|--------|
| Suite b baseline before any Stage 3 edit: `.\.venv\Scripts\python.exe -m unittest test_gui_map_display test_gui_workspace -v` | 12 OK, 85.6 s |
| Suite a: `.\.venv\Scripts\python.exe -m unittest 01_lidar_visualizer 02_grid_mapping 03_vector_mapping localisation_common 04_grid_localisation 05_vector_localisation 06_scan_localisation test_application test_diagnose_lidar_serial test_workspace test_ui_theme test_workspace_localisation -v` | 431 OK, 235 s (`suite_a_final.log`, after all edits) |
| `.\.venv\Scripts\python.exe -m unittest test_gui_workspace_localisation -v` | 6 OK in 4 runs; the last (`stage3_gui_gcfix.log`, 165 s) after all edits. The first run had 2 test-helper bugs, since fixed |
| Suite b after Stage 3, before the GC fix | failed in 5 of 5 runs (1 × 1440x900 unmapped; 4 × `test_builds_shared…` timeout with GC errors) |
| Suite b after the GC fix (3 runs) | OK, FAILED (`test_workspace_1920x1080`: `[] != ['scan']`), OK; 0 GC errors |
| `.\.venv\Scripts\python.exe gui_smoke_test.py` | PASS twice, the last at 10:06, **before** the final `workspace_page.py` edit (10:10, which removed the duplicate result-text line) |
| Parity | `classroom_session.json` scan 3: Workspace = pages 04/05/06 bit for bit (status, pose, score, runner-up score), global and local |
| Poses | 05 ok (−4.475, 3.533, −102.4°); 06 accumulated weak (−4.467, 3.538, −102.5°); scan 1 on 05 low_score; all match Gate 0 |
| Baseline hashes (8 files in `hashes_before.json`) | unchanged (checked after suite a, at the end of the session) |
| `*_measured_poses.json` created | none |

Not verified this session:
- Suite b three times in a row on the final tree: blocked by the desktop minimising windows.
- The smoke test on the final tree: not re-run after the last edit; GUI runs were stopped.
- A real LD06 capture on COM7 localised in the Workspace: the tests mock the capture and no port
  was opened.

## GUI minimisation investigation
- **Symptom:** `test_workspace_1920x1080` failed intermittently: viewports unmapped at the first
  geometry check, `[] != ['scan']` after maximise, or a 100 s timeout. In isolation it failed in 4
  of 6 runs.
- **Finding:** at every failure the Tk root was `state=iconic` (minimised), while the layout was
  correct (`maximised=scan`, `rows panes=[1, 2]`, the scan frame packed at 693x491). The root
  changed from normal to iconic 6.8-17.3 s into each run, while the foreground window belonged to
  another process (window title "Claude").
- **Control:** a bare idle `tk.Tk()` window, with no project code, was minimised within
  1.7-9.2 s in 5 of 5 runs, at 1920x1080, 1440x900 and 800x600.
- **The project never minimises the window.** The only `withdraw()` calls are the standalone
  02-04 file pickers.
- **Conclusion:** external desktop interference. It is not an application layout defect and not
  a test-synchronisation defect.
- **Not done, by design:** deiconify or retry workarounds, longer timeouts, or removed
  assertions. The root cause in the desktop environment is unknown.
- **Unclassifiable:** the Gate 2 failures ("desktop in use") lack a record of the root state.
- **Evidence:** `geometry_investigation/` (`before_fix_iso1-6.log`, `diagnose_run1.log`,
  `diagnose_run2.log`, `bare_window_probe.log`) and `scripts/`.

## Outstanding Gate 3 verification
1. Suite b three times in a row, all OK, on an idle desktop.
2. `test_gui_workspace_localisation` and `gui_smoke_test.py` on the final tree.
3. The hash check, repeated at the end.
4. Your Gate 3 review, then approval of the exact commit file list.
5. Optional, needs your approval to open the port: a real COM7 capture localised in the
   Workspace.

## Known bugs, limitations and open questions
- **Desktop minimises new windows while tests run:** cause unknown. Is a window tool, focus
  setting, or the Claude desktop app responsible? This is an open question for you.
- **Suite b timing sensitivity:** older geometry failures recorded at Gate 2 under desktop load
  may come from the same interference; this is unconfirmed.
- **Optional diagnostic:** naming an iconic root in GUI test failure messages was proposed, not
  approved.
- **Rejected results:** the candidate pose of a rejected result is not shown; that option is not
  approved.
- **Code 04 local search:** cannot be cancelled; a late result is discarded if stale.
- **Font licence:** the font zip `assets/engineering-plot-regular_freefontdownload_org.zip` is
  tracked and pushed, with its licence unresolved. There is no root `.gitignore`. Both are your
  decisions; a fix is proposed in `docs/M1_STAGE3_HANDOFF.md` §1.
- **Selected-scan highlight:** the green highlight can blend with tab:green scans 3, 13 and 23
  (a fix is proposed, not approved).
- **Not implemented:** tracking, Wi-Fi and continuous localisation.

## Decisions and rationale
- **No runner-up marker for `ambiguous` results:** ambiguous is a rejected status, and "only
  ok/weak draw a pose" takes precedence over the handoff checklist. The alternative pose is given
  in the result text instead.
- **Verdict in the chip, not the viewport captions:** the captions keep their map details.
- **Lazy 05 VectorMap:** it is built on the first 05 Localise and shared with page 05, so Workspace
  start-up timing is unchanged.
- **Inline Localisation group, not a pinned footer:** this follows the handoff layout.
- **GC teardown in the GUI tests:** approved by you on 2026-11-04.
- **No code change for the minimisation:** the cause is external.

No `docs/decisions/` records exist yet. These decisions live here and in the Gate 3 report.

## Uncommitted changes and Git status
- Branch `feature/ui-redesign` at c65a78e, up to date with `origin/feature/ui-redesign`. Nothing
  is staged and there are no stashes.
- Modified: `docs/MULTI_VIEW_WORKSPACE.md`, `docs/M1_STAGE3_HANDOFF.md`, `docs/STATUS.md`,
  `test_gui_map_display.py`, `test_gui_workspace.py`, `workspace_page.py`.
- Untracked, part of Stage 3: `test_gui_workspace_localisation.py`, `docs/SESSION_HANDOFF.md`.
- Untracked, **leave untracked** (unrelated or generated):
  - `results/` (71 MB of evidence; never tracked)
  - `.idea/` (IDE)
  - `__pycache__/` (bytecode)
  - `hang.log`
  - `LD06.zip` (third-party library)
  - the three French TP `.md` files (course material)
  - `scans/classroom_extended.json` (byte-identical to `classroom_final.json`)
  - `assets/fonts/` (font README, `.gitignore` and the ignored `local/` font, pending the licence
    decision)
- A separate worktree `C:/Users/stuar/Desktop/SLAM-autonomy` (branch
  `feature/autonomy-simulator`, also at c65a78e) exists. It has its own untracked work
  (`autonomy/`, and Wi-Fi/dashboard notes under `docs/design/` and `docs/research/`), seen only
  read-only with `git status`. It was not touched in this session. Leave it alone; none of it
  belongs to this branch.

**Proposed commit (only after Gate 3 is accepted, and with your approval):**
- Files:
  - `workspace_page.py`
  - `test_gui_workspace_localisation.py`
  - `test_gui_map_display.py`
  - `test_gui_workspace.py`
  - `docs/MULTI_VIEW_WORKSPACE.md`
  - `docs/M1_STAGE3_HANDOFF.md`
  - `docs/STATUS.md`
  - `docs/SESSION_HANDOFF.md`
- Suggested message: "M1 Stage 3: Workspace one-shot localisation of an independent scan".

## Suggested spec status changes (awaiting approval)
None. `docs/specs/` is empty.

## Next recommended milestone
Recommendation only, not an agreed plan: finish and accept Gate 3 (above) before anything else.
After that, M2 (a common input layer for recorded and USB scans), as listed in `docs/STATUS.md`.

## How to resume
1. Read `AGENTS.md`, then this file and `docs/STATUS.md`. Run `git status` and confirm that the
   changes match "Uncommitted changes" above.
2. Setup: Windows, `.\.venv\Scripts\python.exe` (Python 3.13). No hardware is needed for Gate 3.
   Hardware (LD06 → ESP32 → CH340 on COM7, 115200) is needed only for the optional real-capture
   check, and only with your approval.
3. **Make the desktop idle first:** no clicking, no window switching, and no other app bringing
   itself forward during GUI runs. Optional check:
   `.\.venv\Scripts\python.exe -I results\m1_gate3_20261104_092409\scripts\bare_window_probe.py 1920 1080 30`.
   It must print only `None -> normal` and `done`. If it prints `-> iconic`, stop: the desktop is
   still minimising windows.
4. Back up `results/workspace/` and `results/gui_review/` into a new timestamped
   `results/m1_gate3_<timestamp>/`, because the GUI tests overwrite them.
5. Run, one after another, saving each log in that folder:
   1. `.\.venv\Scripts\python.exe -m unittest test_gui_map_display test_gui_workspace -v`, three
      times in a row. All three runs must be OK.
   2. `.\.venv\Scripts\python.exe -m unittest test_gui_workspace_localisation -v`
   3. `.\.venv\Scripts\python.exe gui_smoke_test.py`
   4. The hash check against `results/m1_gate0_20261104_041714/hashes_before.json`.
6. If a GUI run fails, run `scripts/diagnose_mapping.py` from the evidence folder to see whether
   the root was `iconic` before treating it as a code defect.
7. Update the Gate 3 report and `docs/STATUS.md`. Ask for the Gate 3 review and commit approval.
   Do not stage, commit or push before that. Do not start M2.

## Confirmed facts vs assumptions
- **Confirmed** by files, Git and commands seen in this session:
  - all the test results, timestamps and hashes above
  - the Git state and the worktree list
  - `classroom_extended.json` is byte-identical to `classroom_final.json`
  - `results/` is untracked (0 tracked files)
  - the root was `iconic` at each instrumented failure
  - bare windows were minimised
- **Assumed:**
  - The minimisation comes from the desktop environment or a user-side tool, not from Windows
    itself for a specific reason: it is unverified, and only the foreground window was recorded.
  - The Gate 2 geometry failures had the same cause: unverified.
  - The COM7 capture working is user-reported, not observed in this session.
