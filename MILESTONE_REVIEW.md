**Unified LiDAR application and new-scan localisation — review guide**

The application hosts the existing four tools in one Tk window. Existing scan
matching, occupancy integration and vector extraction mathematics are retained.
No commit or push was made. Codes 05/06 and continuous SLAM are outside this work.

**Run in PyCharm**

1. Use the project's existing Python 3.13 interpreter (`.venv/Scripts/python.exe`).
2. Open `00_main_app.py` and choose **Run**. No arguments are required.
3. Click **Open session…** and select `scans/classroom_final.json`.
4. Use the left sidebar to switch tools. The selected dataset and Scanner edits
   remain in memory. The map toolbar supplies pan, zoom, home and image export.
5. The right control panel scrolls on smaller windows. The window is resizable;
   the map gives up width before the controls do.

Optional command-line start:

```powershell
.\.venv\Scripts\python.exe 00_main_app.py --session scans/classroom_final.json --page "Grid Localisation"
```

The application starts empty when no session is specified. It does not connect to
the sensor automatically or load an artificial demonstration.

**Localise a new real scan**

1. Open the classroom reference session, then choose **Grid Localisation** and
   **Localise New Scan** (the default mode).
2. For live acquisition, enter the actual COM port and baud rate, keep the sensor
   still, then click **Capture New Scan**. Defaults come from Code 01: COM5,
   115200 baud. These are configuration defaults, not a detected connection.
3. Acquisition expects the existing ESP32 firmware's LD06 Teleplot XY stream.
   The port is opened read-only, with DTR/RTS inactive, for one fresh scan. It
   closes after completion, cancellation or failure. The read has a 10-second
   capture timeout. Close other serial monitors if the port is busy.
4. Alternatively, click **Load Recorded Scan**, select another version-2 session
   JSON and choose its scan in the dropdown. Only its local XY measurements are
   used; its saved pose is never a localisation answer or starting guess.
5. Left-click the map to set approximate position. Right-click to point the
   heading toward the cursor, or enter X/Y/heading and click **Set pose**.
6. Click **Auto Align**. Orange shows the initial overlay; blue shows the result.
   Read the estimated X/Y in metres, heading in degrees, score, status and
   algorithm runtime. A poor or ambiguous result is explicitly reported.

The coarse search is approximately ±1 m and ±30° around the initial guess,
followed by the existing fine refinement. This is not global localisation.
The reference map stays fixed: targets are neither added to it nor saved into
the reference session. Exact copied/reordered reference recordings are rejected
and directed to recorded-scan evaluation. This safeguard detects duplicate
measurements; it cannot detect every partial or edited copy of a scan.

**Recorded-scan evaluation**

Choose **Evaluate Recorded Scans** on the same page. Select a scan, keep
**Leave target out of reference map** enabled, and use the Day 1 starting pose
or a reproducibly perturbed guess. Disabling exclusion explicitly reports
self-matching. The original single-scan-session self-matching warning is retained.

**Evaluate all / export CSV + JSON** asks for an output folder, then runs the
existing initialisation conditions and repeated timing measurements on a worker.
Exports use unique timestamped filenames, preserving existing Phase 1 results.
You can navigate while evaluation runs; closing waits for the requested export.
Exports identify use of unsaved working edits when applicable.

Day 1 poses are estimates. Their deviations are agreement/self-consistency,
not physical ground-truth accuracy. Independently measured pose sidecars are
loaded through `localisation_common.py` when present.

**Working data and saving**

Scanner selection, visibility, poses, display modes, rays, automatic alignment
proposals, accept/reject, capture/add/replace and deletion use the existing editor.
Deletion retains its two-click confirmation. Scanner edits survive page changes.
They update the shared working dataset and invalidate dependent map caches.
Changing the reference dataset starts a fresh localisation page and clears its
old target/result; changing only pages preserves the page state.

**Save working copy…** explicitly writes a new session through Code 01's atomic
writer and makes that copy the active filename. The initially loaded source and
current active file cannot be overwritten by this action. Opening another session
or closing with unsaved Scanner edits offers Save / Discard / Cancel. Invalid JSON
does not replace the working session. No automatic saving occurs. Live targets
in the localisation page are temporary; capture in Scanner and save a separate
session if a persistent recording is needed.

Maps use an in-memory snapshot. External disk changes take effect when the file
is reopened; there is no file watcher. Measured-pose sidecars are not automatically
copied when saving a new session filename.

**Implementation and files**

| File | Change |
| --- | --- |
| `00_main_app.py` | New PyCharm entry point. |
| `desktop_app.py` | New Tk host, navigation, page-specific controls, worker result polling and resource cleanup. |
| `app_session.py` | New shared working-session adapter, revision caches and bounded acquisition adapter. |
| `01_lidar_visualizer.py` | Optional caller-supplied Figure, exposed map axes, and close-event timer cleanup. |
| `02_grid_mapping.py` | Optional supplied Figure; explicit CLI selection/non-interactive mode; consolidated the existing picker. |
| `03_vector_mapping.py` | Optional supplied Figure; explicit CLI selection/non-interactive mode. |
| `04_grid_localisation.py` | New target loading/validation, fixed-map viewer mode, asynchronous-result presentation hook; desktop application is the normal launch mode. Existing evaluation and search remain. |
| `test_application.py` | 15 headless integration regressions. |
| `gui_smoke_test.py` | Explicit real desktop checks, screenshots, real-data export and simulated acquisition checks. |
| `MILESTONE_REVIEW.md` | This run guide, architecture description and verification report. |
| `results/milestone_*.log`, `results/gui_review/` | Test evidence, rendered screenshots and machine-readable GUI results. |

`localisation_common.py` is reused unchanged. Code 04 and that helper were already
untracked at handover; the Code 02/03 file-picker edits were already uncommitted.
Existing recordings, Phase 1 result files and unrelated files were preserved.

Each page creates a normal Matplotlib `Figure` and attaches `FigureCanvasTkAgg`
before calling an existing builder. Existing callbacks therefore bind directly
to the embedded canvas. Tk controls call the editor/viewer's public actions;
old in-figure controls are hidden, with their direct canvas events disconnected
where needed. No private Matplotlib attributes are changed. Standalone callers
that omit the Figure retain their original pyplot windows.

The shell owns one Tk main loop. Numerical jobs and capture readers return data
through a queue; only the Tk poller updates controls and artists. A changed target
or starting pose invalidates an in-flight alignment result. Closed-page results
are discarded. Sensor cancellation is signalled before window destruction and
the connection is released by its owning worker. Code 01's existing alignment
worker is polled by the host. The `PAGES` dictionary is the extension point for
future tools.

Reused functions include `create_session_editor()`, `open_serial_readonly()`,
`capture_teleplot_scans()`, `teleplot_to_sensor_frame()`, Code 01's existing
alignment actions (correlative search/ICP), Code 02's probabilistic and binary
builders plus `show_grid()`, Code 03's preprocessing/segmentation/line-fit/merge
pipeline plus `build_layer_viewer()`, and Code 04's unchanged `localise_scan()`,
`reference_map()`, `session_rows()`, summaries and `save_results()`.

**Independent scripts and the picker regression**

Codes 01–04 still run independently. Code 01 retains its existing `RUN_MODE`;
Code 04 defaults to the desktop window on Grid Localisation. Its old
`RUN_MODE = "viewer"` and `"evaluation"` entry paths remain available.

Codes 02/03 still open a file picker on ordinary PyCharm Run. Cancelling exits
without a map. Imports create no picker. Automated runs explicitly bypass both
the picker and plot window:

```powershell
.\.venv\Scripts\python.exe 02_grid_mapping.py --non-interactive
.\.venv\Scripts\python.exe 03_vector_mapping.py --non-interactive
```

Use `--session classroom_final.json` or an absolute path to select a session.
`--session` alone skips the picker but retains the interactive plot. The three
existing subprocess tests now pass `--non-interactive`; their original numerical
and output assertions are unchanged. No pytest/environment test detection was
introduced.

**Verification**

Baseline: 319 tests, 316 passed and 3 failed. The failures were the known
Code 02/03 subprocess/file-dialog cases; in the restricted environment they
raised a Tcl initialisation error instead of timing out. Tk worked with desktop
access outside that sandbox. Baseline log: `results/milestone_baseline_tests.log`.

After implementation: all 334 tests passed in 120.9 seconds — 247 Day 1, 72 Phase 1
(19 shared helper tests + 53 Code 04 tests), and 15 new integration tests. The established
grid fingerprints and 74-segment vector baseline passed unchanged. See
`results/milestone_regression_tests.log`. Run them with:

```powershell
$env:MPLBACKEND = 'Agg'
.\.venv\Scripts\python.exe -m unittest 01_lidar_visualizer 02_grid_mapping 03_vector_mapping localisation_common 04_grid_localisation test_application -v
```

The desktop check rendered all four pages, dispatched real Matplotlib mouse
events, exercised toolbar pan, layer changes, scan selection, Code 01 alignment
proposal/reject, Code 04 alignment, navigation during work, session changes,
cache invalidation, save-copy/discard/cancel, invalid JSON, narrower-window
layout, simulated serial success/failure, and closing during capture. It also
rendered the independent scripts and exercised cancelled Code 02/03 pickers.
No duplicate pyplot windows were created by embedded pages. A 32-run real-data
evaluation (16 scans × two initialisations, one timing repeat) exported valid
CSV and JSON through the GUI worker. Source session hashes were checked unchanged.
From the Day 1 initial poses, 15 scans returned `ok` and one returned `low_score`;
median algorithm runtime was 348 ms in the final single-repeat
smoke run. This is not a repeat of the full default three-repeat benchmark.
An AST comparison against committed Day 1 code found changes only to its GUI
builders; the existing top-level mathematical functions were unchanged.

```powershell
.\.venv\Scripts\python.exe gui_smoke_test.py
```

This is an explicit desktop test, separate from headless unittest discovery.
It needs access to the Windows desktop. Pillow, already present with Matplotlib,
captures only the application window. Screenshots were inspected visually.

A real held-out Scan 2 was loaded from a temporary separate JSON after removing
it from the reference. Its source-file pose was deliberately replaced with an
incorrect pose, and a separate approximate guess was entered. The result was
`ok`, score 0.543, approximately 0.082 m / 2.047 m / 1.65°, with 23,971 candidate
poses. It agreed with the Day 1 estimate within 2.1 cm and essentially 0°; this
does not establish physical accuracy. Runtime was 357 ms in the final GUI run.

The older separate `classroom_session.json` Scan 1 loaded successfully but
returned `low_score` (0.043) from an origin guess against `classroom_final.json`.
This was displayed as a poor match, not a successful robot location. The current
machine-readable timings, verdicts and evaluation summary are in
`results/gui_review/verification.json`.

**Hardware status and remaining limits**

No physical LD06/ESP32 capture was performed. Acquisition was verified against
Code 01's known interface with simulated serial streams, missing-port errors,
timeouts, cancellation and connection cleanup. Actual wiring, COM-port choice,
baud rate, firmware and sensor operation still need testing in the lab. The
adapter expects the existing Teleplot stream; raw LD06 serial packets and
RobotControl integration were not added.

Matching requires an approximate pose and sufficient observed geometry. A high
score is not independent proof of robot position. There is no measured ground
truth in the supplied classroom data. Full default evaluation can take several
minutes; the GUI smoke export deliberately uses a reduced trial count. Long
numerical workers do not touch the UI; evaluation export must finish before exit.

**Rendered review images**

- [LiDAR Scanner](results/gui_review/01_lidar_scanner.png)
- [Grid Mapping](results/gui_review/02_grid_mapping.png)
- [Vector Mapping — 74 merged segments](results/gui_review/03_vector_mapping.png)
- [New-scan poor-match warning](results/gui_review/05_new_scan_aligned.png)
- [Recorded-scan evaluation](results/gui_review/06_recorded_evaluation.png)
- [Resized window and changed session](results/gui_review/07_resized_session_changed.png)
- [Held-out real scan aligned](results/gui_review/08_heldout_real_scan_aligned.png)
