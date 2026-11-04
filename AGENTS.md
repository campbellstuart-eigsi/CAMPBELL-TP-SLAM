# Project instructions

## Overview
LiDAR SLAM Workbench (university project): an LD06 LiDAR read through an ESP32 over USB serial,
with Tkinter/Matplotlib programs for scan capture, occupancy-grid and vector mapping, and grid,
vector and scan localisation, plus a seven-page desktop app with a four-view Workspace.

## Stack
Python 3.13, Tkinter, Matplotlib, NumPy, pyserial (tests also use Pillow). Hardware: LD06 → ESP32 → CH340 USB serial, 115200 baud.

## Commands
- Install: `<TBD>` (no requirements file in the project; uses `.venv`)
- Run: `.\.venv\Scripts\python.exe 00_main_app.py`
- Test: `.\.venv\Scripts\python.exe -m unittest 01_lidar_visualizer 02_grid_mapping 03_vector_mapping localisation_common 04_grid_localisation 05_vector_localisation 06_scan_localisation test_application test_diagnose_lidar_serial test_workspace test_ui_theme test_workspace_localisation test_robot_drive -v`; GUI: `... -m unittest test_gui_map_display test_gui_workspace test_gui_workspace_drive -v`; smoke: `... gui_smoke_test.py`
- Lint/format: `<TBD>`

## Conventions
Numbered programs: 01-03 Day 1 mapping, 04-06 Day 2 localisation, loaded by module name via importlib. Recordings live in `scans/` and are never modified. Evidence goes in new timestamped folders under `results/`. Branch naming: `<TBD>`.

## Documentation
- `docs/STATUS.md`: current project state and handover notes (updated with `/handover`)
- `docs/research/`: background research and notes
- `docs/design/`: architecture and design documents
- `docs/specs/`: feature specifications (each has a `Status:` line: Draft / Approved / Implemented / Superseded)
- `docs/decisions/`: decision records, one file per decision

## Project Documentation Workflow
Applies to any project type: software, electronics, mechanical or creative.
- **Starting a project:** fill in the sections above with information I have given you or that is in the folder. Leave unknowns as `<TBD>` and ask me.
- **Research:** save useful findings, with sources, in `docs/research/`.
- **Design:** organize UI inspiration, sketches, drawings and design notes in `docs/design/`.
- **Specs:** write feature requirements and plans in `docs/specs/`. New specs start as `Status: Draft`. Only I can approve a spec.
- **Decisions:** record significant decisions (what, why, alternatives) in `docs/decisions/`.
- **Status:** update `docs/STATUS.md` at meaningful milestones.

Rules:
- Update docs when something worth keeping changes, not after every conversation or small edit.
- Never invent information. Mark assumptions and gaps clearly.
- Never overwrite or delete existing work; add to it or propose edits. Never mark a spec Approved without my permission.
- If you can write to this folder, maintain the files directly. Otherwise, give me each document with its file path so I can save it.

## Session Handoff Workflow
- The workflow is in `workflows/08_handoff.md`. When I say "Run the session handoff workflow," follow that file.
- Use it before ending a significant development session. It writes `docs/SESSION_HANDOFF.md` (a fuller resume briefing) alongside `docs/STATUS.md`.
- At the start of a new session, read this file and `docs/SESSION_HANDOFF.md` if it exists.
- When resuming, summarize the current state and propose next steps. Do not start coding until I agree.

## Template Sync Workflow
- The workflow is in `workflows/10_template_sync.md`, run through the global `/project-sync` skill (`audit`, `apply`, `status`).
- It compares this project with the master development template. Only `apply` changes anything, and it only adds missing universal files; existing files change only with my approval.
- Never run it automatically at session start.
