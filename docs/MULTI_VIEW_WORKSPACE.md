# Multi-View Workspace specification

The additive **Workspace** page (Ctrl+7) opens with this default arrangement.
The six existing pages and Ctrl+1–6 remain available; the application's startup
page remains unchanged.

| Position | Viewport | Data and coordinate frame |
|---|---|---|
| Top-left | **Scan Map** | Code 01's visible scans accumulated using their current alignment poses; global metres. |
| Top-right | **Bitmap / Occupancy Grid** | Code 02's cached occupancy grid; Probability, Classified, Binary and Empty display modes; global metres. |
| Bottom-left | **Vector Map** | Code 03's cached extracted and merged finite line segments; global metres. |
| Bottom-right | **Current Scan / Live Robot** | Shared selected recorded scan or latest completed one-shot capture; original sensor-local metres. |

## Layout and navigation

- Drag the horizontal divider to resize the rows. Each row has its own vertical
  divider so every viewport is resizable.
- Each title bar offers maximise/restore; double-clicking the title does the same.
  Restore preserves divider proportions and all four navigation states.
- Click a viewport to activate it. The existing application toolbar's Fit, Reset,
  Back, Forward, Pan, Zoom and Export operate on that viewport. Mouse-wheel zoom
  is centred on the pointer. All plots have equal X/Y aspect, X right, Y up, and
  axis labels in metres. Resizing changes available pixels, never the geometry.
- Optional **Link global pan / zoom** shares world-coordinate limits among the
  first three views, including toolbar navigation. Different panel shapes may
  have unused margins to preserve equal aspect. The current scan never joins
  this link, even when maximised.

## Selection and data ownership

- One recorded scan selection is shared with Code 01 and all relevant Workspace
  views. Selection changes do not invalidate the mapping cache or dirty the file.
- Global markers show the selected scan's **recorded alignment pose**, not a
  live estimate. Hidden scans remain available in the local panel but have no
  global marker. Viewing a fresh capture does not place it on a global map.
- Workspace capture uses the existing bounded Code 01 capture adapter. The
  latest completed capture from any desktop page is available in the fourth
  panel. Workspace capture does not add that scan to the accumulated map.
- Use the session's revision-scoped map cache and shared in-flight jobs. Build
  missing grid/vector results once; Binary is built on first use. Cache display
  arrays and global point transforms. Resize, selection, pan, zoom and maximise
  must never rebuild maps. Session edits invalidate derived results as before.

## Future interfaces

`ScanFrame` and `LiveScanSource.latest_scan()` describe sensor-local samples.
`EstimatedRobotPose` and `RobotPoseSource.latest_pose()` separately describe
global estimated X/Y in metres, counterclockwise heading in radians, and a
timestamp in seconds. Both providers are optional and inactive in this version.
No scan-to-pose inference, continuous acquisition, tracking or localisation loop
is implemented. A future integration must explicitly supply and associate poses.

## Preservation and acceptance

Keep Tkinter/Matplotlib, the Rhino/QGIS-inspired chrome, all six original pages,
the robust GUI queue poller and Tk popdown focus fix. Do not change Codes 01–06,
serial parsing/acquisition, firmware or source recordings. Work stays uncommitted
on `feature/ui-redesign`; `main` remains untouched.

Verify at **1440×900** and **1920×1080** using the real Tk event loop: all four maps,
divider resizing, equal aspect and physical units, scan selection, independent
and linked navigation, all four maximise/restore controls, held combobox popdowns,
capture, cache reuse, and returning to each original page. Run the full numerical
suite and existing GUI smoke checks; report timings and remaining limitations.
