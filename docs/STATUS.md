# Project status

_Last updated: 2026-11-04_

Current project state only. Keep under ~60 lines; replace outdated items rather than appending.

## Progress
- Branch `feature/ui-redesign`, pushed to `origin/feature/ui-redesign`. The latest code commit is
  **e5f1a1d** "M1: UI polish, localisation adapter and Stage 3 handoff". A documentation-only
  commit follows it: the template 1.0.0 sync, this file and the handoff's Git section.
  `main` is at bd8b842.
- Codes 01-06 work: 01 capture and alignment, 02 occupancy grid, 03 vector map, 04 grid,
  05 vector and 06 scan localisation. The seven-page desktop app (`00_main_app.py`) and its
  four-view Workspace (Scan Map, Occupancy Grid, Vector Map, sensor-local Current Scan) work too.
- M1 Stage 0: baseline recorded, plus a fix for the Workspace close-during-resize TclError, with a
  regression test.
- M1 Stage 1:
  - plot colours are named in `ui_theme.PLOT`
  - Workspace pose colours match pages 04-06: green recorded pose, orange guess, blue estimate,
    purple alternative
  - tab-group labels are 8 pt
- M1 Stage 2: `workspace_localisation.py` calls Codes 04/05/06 through the pages' exact inputs.
  It is bit-identical to the page code paths and to the Gate 0 baseline (18 of 18 runs).
- LD06 USB capture through ESP32/CH340 on **COM7** at 115200 baud works: a real 456-point scan
  was captured (user-reported 2026-11-04). The app's default port field is COM5; type COM7.

## Unfinished work
- **M1 Stage 3 (not started):** localise an independent captured or recorded scan in the
  Workspace, and show the estimated pose on all three global maps. The Current Scan stays
  sensor-local. The full brief and checklist are in `docs/M1_STAGE3_HANDOFF.md`.

## Next steps
1. Re-run GUI suite b with the desktop idle, to get a clean baseline.
2. Implement Stage 3 following the handoff checklist. Stop at the Gate 3 review.
3. M2: a common input layer for recorded and USB scans. M3: ESP32 Wi-Fi transport, verified on
   hardware. M4: continuous localisation and trajectory tracking. Nothing claims live tracking
   until it has been verified.

## Issues
- GUI suite b is unstable: it failed in 5 of 5 runs at Gate 2, and in 1 of 3 runs on the Gate 1
  code. The failures are screen geometry and timing, never numbers. `test_workspace_1920x1080`
  has been intermittent since Phase 0.
- The font zip `assets/engineering-plot-regular_freefontdownload_org.zip` is tracked and pushed,
  but its licence is unresolved. There is no root `.gitignore`. Both need your decision.
- The highlighted points of the selected scan (green) can blend with tab:green scans 3, 13 and
  23. A fix is proposed, not approved.
- Code 04 local search cannot be cancelled; a late result can only be discarded as stale.
- COM "Access is denied" means another program has the port. Quit the Arduino IDE fully and
  replug the cable.

## Key decisions
- Codes 01-06, `localisation_common.py` and `scans/` are never modified. Localisation goes only
  through the adapter, and every result must match its page bit for bit.
- Only `ok` and `weak` results draw a pose. Rejected results draw none.
- Captures on 01 Scan join the session, so they are refused as self-matching targets. Workspace
  and 04-06 captures stay independent.

## Verified tests
| Date | Command | Result |
|------|---------|--------|
| 2026-11-04 | `.\.venv\Scripts\python.exe -m unittest 01_lidar_visualizer 02_grid_mapping 03_vector_mapping localisation_common 04_grid_localisation 05_vector_localisation 06_scan_localisation test_application test_diagnose_lidar_serial test_workspace test_ui_theme test_workspace_localisation -v` | 431 tests, OK |
| 2026-11-04 | `.\.venv\Scripts\python.exe gui_smoke_test.py` | PASS |
| 2026-11-04 | `.\.venv\Scripts\python.exe -m unittest test_gui_map_display test_gui_workspace -v` | 12 tests: OK at Gate 1 (2 of 2 runs); intermittent at Gate 2 |
| 2026-11-04 | Day 2 evaluation re-run, in memory | 24 of 24 rows identical to the 20261103 results |

## Untested / unverified
- A clean suite b run on e5f1a1d.
- The Workspace localisation UI (Stage 3 does not exist yet).
- Wi-Fi and continuous tracking (not implemented).
