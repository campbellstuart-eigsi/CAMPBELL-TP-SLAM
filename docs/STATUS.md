# Project status

_Last updated: 2026-11-04_

Current project state only. Keep under ~60 lines; replace outdated items rather than appending.
The fuller resume briefing is `docs/SESSION_HANDOFF.md`.

## Progress
- Branch `feature/ui-redesign`: Stage 3 checkpoint **536dba2**, plus a docs commit recording it.
  Both were pushed to `origin/feature/ui-redesign` on 2026-11-04 with your approval. `main` is at
  bd8b842; Stages 0-2 are in e5f1a1d.
- Codes 01-06 work, as do the seven-page desktop app (`00_main_app.py`) and its four-view
  Workspace. M1 Stages 0-2 are committed: the close/resize fix, UI consistency, and the
  bit-identical `workspace_localisation.py` adapter.
- **M1 Stage 3 is implemented and checkpointed in 536dba2 ("M1 Stage 3: one-shot localisation
  implemented, Gate 3 pending"). Gate 3 is NOT yet accepted.** The Workspace
  Localisation group localises one independent scan (04/05/06, global or local) and draws the
  estimate on all three global maps. Report: `results/m1_gate3_20261104_092409/REPORT.md`.
- LD06 USB capture through ESP32/CH340 on **COM7** at 115200 baud works (user-reported). The
  app's default port is COM5.

## Blocked / pending for Gate 3
- GUI suite b has not passed 3 times in a row since Stage 3 (latest: 2 of 3 OK). The failing
  runs coincide with the desktop minimising the test window: a bare idle `tk.Tk()` window was
  minimised in 5 of 5 runs. Re-run on an idle desktop.
- The smoke test has not been re-run since the last `workspace_page.py` edit.
- Your Gate 3 review and acceptance.

## Next steps
1. On an idle desktop: suite b three times in a row, then the Stage 3 GUI tests, the smoke test
   and the hash check (`docs/SESSION_HANDOFF.md`, "How to resume").
2. Gate 3 review and acceptance; commit and push the updated STATUS only with your approval.
3. Optional: a real COM7 capture localised in the Workspace.
4. Later (not started): M2 common input layer, M3 ESP32 Wi-Fi, M4 continuous tracking.

## Issues
- The desktop minimises new Tk windows while tests run (cause unknown; the foreground app was the
  Claude desktop app). This blocks GUI verification, not the app.
- Fixed this session with your approval: GC of closed test apps on worker threads made suite b
  time out. The GUI tests now `gc.collect()` on the main thread after each app close.
- The font zip `assets/engineering-plot-regular_freefontdownload_org.zip` is tracked and pushed,
  but its licence is unresolved. There is no root `.gitignore`. Both need your decision.
- The green selected-scan highlight can blend with tab:green scans 3, 13 and 23 (fix proposed).
- Code 04 local search cannot be cancelled; a late result is discarded as stale.
- COM "Access is denied": another program has the port. Quit the Arduino IDE and replug.

## Key decisions
- Codes 01-06, `localisation_common.py` and `scans/` are never modified. Localisation goes only
  through the adapter, and every result must match its page bit for bit.
- Only `ok` and `weak` draw a pose. Ambiguous counts as rejected, so no runner-up marker; the
  alternative pose is given as text.
- Captures on 01 Scan join the session (refused as self-matching targets). Workspace and 04-06
  captures stay independent.

## Verified tests (Gate 3, 2026-11-04, on the current working tree)
| Command | Result |
|---------|--------|
| Suite a (the 12 headless modules in `AGENTS.md`) | 431 OK (after all edits) |
| `.\.venv\Scripts\python.exe -m unittest test_gui_workspace_localisation -v` | 6 OK (4 runs; the last one after all edits) |
| `.\.venv\Scripts\python.exe -m unittest test_gui_map_display test_gui_workspace -v` | 2 of 3 OK; 1 × `test_workspace_1920x1080`, with the window minimised externally |
| `.\.venv\Scripts\python.exe gui_smoke_test.py` | PASS (before the last `workspace_page.py` edit) |
| SHA-256 of the 8 Gate 0 baseline files | unchanged |

## Untested / unverified
- Three consecutive clean suite b runs, and the smoke test, on the final tree.
- A real LD06 capture localised in the Workspace (the tests mock the capture).
- Wi-Fi and continuous tracking (not implemented).
