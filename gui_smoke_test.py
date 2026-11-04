"""Real desktop smoke check (opens windows); explicit, not unittest discovery.

Run: python gui_smoke_test.py
Screenshots show only this application, saved in results/gui_review/.
Hardware is simulated; no physical connection is claimed.
"""
import importlib
import ctypes
from collections import Counter
import json
import tempfile
import threading
import time
import tkinter as tk
from tkinter import ttk
from pathlib import Path
from unittest import mock

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseEvent
from PIL import ImageGrab
import numpy as np

from desktop_app import DesktopApp, PAGES
from app_session import SharedSession, part1, part2, part3, grid_localisation as loc
import localisation_common as common

ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT / "results" / "gui_review"


def pump(root, predicate=lambda: True, timeout=30):
    end = time.monotonic() + timeout
    while True:
        root.update()
        if predicate():
            root.update_idletasks()
            return
        if time.monotonic() >= end:
            raise AssertionError("GUI operation timed out")
        time.sleep(0.02)


def screenshot(root, name):
    root.lift()
    root.update()
    time.sleep(0.2)
    # Capture this HWND, not a desktop rectangle: works when another window
    # overlaps it and with Windows display scaling / multiple monitors.
    hwnd = ctypes.windll.user32.GetParent(root.winfo_id())
    ImageGrab.grab(window=hwnd).save(OUTPUT / f"{name}.png")


def main():
    OUTPUT.mkdir(exist_ok=True, parents=True)
    hashes = {p: part2.file_sha256(p) for p in common.SCANS_DIR.glob("*.json")}
    errors = []
    root = tk.Tk()
    root.report_callback_exception = lambda *exc: errors.append(str(exc))
    app = DesktopApp(root, common.SCANS_DIR / "classroom_final.json")
    visited = []
    try:
        for index, name in enumerate(PAGES):
            app.nav_buttons[name].invoke()
            page = app.pages[name]
            pump(root, lambda: not page.busy)
            if name == "Vector Mapping":
                assert "74" in page.info.cget("text")
                page.viewer["set_layer"]("points", True)
                assert "points" in page.viewer["visible_layers"]()
                page.viewer["reset_view"]()
            if name == "Grid Mapping":
                for mode in ("Binary", "Classified", "Empty", "Probability"):
                    page.view.set(mode)
                    page.draw_map()
                    pump(root, lambda: not page.busy)
                    assert page.figure.axes[0].images
            assert not plt.get_fignums(), "Embedded page created a pyplot window"
            assert not [w for w in root.winfo_children() if isinstance(w, tk.Toplevel)]
            screenshot(root, f"{index + 1:02d}_{name.lower().replace(' ', '_')}")
            visited.append(name)
        # Code 05: a separate real recording, global search, no initial pose.
        vpage = app.pages["Vector Localisation"]
        pump(root, lambda: vpage.vmap is not None, timeout=60)
        assert len(vpage.vmap.starts) == 74
        segments_before = vpage.vmap.starts.copy()
        reference_count = len(app.model.scans)
        assert vpage.load_recorded(common.SCANS_DIR / "classroom_extended.json") is False  # self-matching
        assert vpage.load_recorded(common.SCANS_DIR / "classroom_session.json")
        vpage.target_choice.current(2)
        vpage.choose_target()
        assert vpage.global_search() and vpage.localise_button.cget("text") == "Localise"
        vpage.localise()
        app.show_page("Grid Mapping")  # navigation during the search
        pump(root, lambda: not vpage.busy, timeout=120)
        app.show_page("Vector Localisation")
        vector_result = vpage.result
        assert vector_result.status == "ok" and vector_result.initial_pose is None, vector_result
        assert "Coordinate reference: Scan 1 (0, 0, 0°)" in [
            w.cget("text") for w in vpage.controls.winfo_children() if isinstance(w, ttk.Label)]
        np.testing.assert_array_equal(segments_before, vpage.vmap.starts)
        assert len(app.model.scans) == reference_count  # never added to the map
        screenshot(root, "09_vector_localisation_separate_scan")
        print("Vector localisation, separate real scan:", vector_result.to_dict())
        vpage.localise()
        vpage.cancel_search()
        pump(root, lambda: not vpage.busy, timeout=60)
        assert "cancelled" in vpage.info.cget("text") and vpage.result is vector_result
        vpage.search.current(1)
        vpage.change_search()
        vpage.set_pose((-4.3, 3.3, -95.0))
        vpage.localise()
        pump(root, lambda: not vpage.busy, timeout=60)
        assert vpage.result.method == "vector_local" and vpage.result.status == "ok", vpage.result
        # Code 06: the same separate real scan matched to recorded LiDAR points.
        spage = app.pages["Scan Localisation"]  # created by the page loop above
        pump(root, lambda: spage.reference_ready, timeout=30)
        assert spage.load_recorded(common.SCANS_DIR / "classroom_extended.json") is False
        assert spage.load_recorded(common.SCANS_DIR / "classroom_session.json")
        spage.target_choice.current(2)
        spage.choose_target()
        spage.reference_mode.current(1)  # accumulated LiDAR points
        spage.change_reference()
        spage.localise()
        spage.localise()                 # a repeated click is ignored
        app.show_page("Grid Mapping")    # navigation during the search
        pump(root, lambda: not spage.busy, timeout=180)
        app.show_page("Scan Localisation")
        scan_result = spage.result
        assert scan_result.method == "scan_accumulated_global", scan_result
        assert scan_result.status in ("ok", "weak"), scan_result
        assert common.pose_error(scan_result.pose, vector_result.pose)[0] < 0.1
        assert len(app.model.scans) == reference_count
        screenshot(root, "10_scan_localisation_separate_scan")
        print("Scan localisation, separate real scan:", scan_result.to_dict())
        spage.reference_mode.current(0)  # strict single reference: nearest scan 25
        spage.reference_choice.current([s["id"] for s in app.model.scans].index(25))
        spage.change_reference()
        assert spage.result is None
        spage.search.current(1)
        spage.change_search()
        spage.set_pose((-4.3, 3.3, -95.0))
        spage.localise()
        pump(root, lambda: not spage.busy, timeout=60)
        assert spage.result.method == "scan_single_local", spage.result
        spage.localise()
        spage.cancel_search()
        pump(root, lambda: not spage.busy, timeout=60)
        assert "cancelled" in spage.info.cget("text")
        page = app.pages["Grid Localisation"]
        assert page.viewer is not None
        grid = page.viewer.grid_map
        grid_before = grid.probability.copy()
        assert page.load_recorded(common.SCANS_DIR / "classroom_session.json")
        original = page.viewer.external_scan["points_local"].copy()
        assert page.align_button.cget("text") == "Localise" and page.global_search()
        # Guess handling below belongs to the optional LOCAL search.
        page.search.current(1)
        page.change_search()
        # Dispatch real Matplotlib mouse events via the embedded canvas.
        page.canvas.draw()
        for button, xy in ((1, (0.2, 0.3)), (3, (0.2, 1.3))):
            px, py = page.viewer.axes.transData.transform(xy)
            event = MouseEvent("button_press_event", page.canvas, px, py, button=button)
            page.canvas.callbacks.process("button_press_event", event)
        np.testing.assert_allclose(page.viewer.initial_pose(), (0.2, 0.3, np.pi / 2), atol=0.04)
        page.set_pose((0, 0, 0))
        page.align_button.invoke()
        # Navigation while Localise is running must stay responsive.
        app.show_page("Grid Mapping")
        pump(root, lambda: not page.busy)
        app.show_page("Grid Localisation")
        assert page.viewer.result is not None, page.info.cget("text")
        result = page.viewer.result
        print("Separate real scan:", result.to_dict())
        np.testing.assert_array_equal(original, page.viewer.external_scan["points_local"])
        np.testing.assert_array_equal(grid_before, grid.probability)
        assert page.viewer.grid_map is grid
        screenshot(root, "05_new_scan_aligned")
        # A result from an old guess must never overwrite the new one.
        page.align()
        page.set_pose((0.5, 0.5, 20))
        pump(root, lambda: not page.busy)
        assert page.viewer.result is None
        assert "changed" in page.info.cget("text")
        page.mode.set(page.MODES[1])
        page.change_mode()
        page.scan_choice.current(1)
        page.select_reference_scan()
        page.day1_pose()
        page.align()
        pump(root, lambda: not page.busy)
        assert "excluded" in page.viewer.map_description
        assert "Agreement with Day 1" in page.result_label.cget("text")
        screenshot(root, "06_recorded_evaluation")
        # Real CSV/JSON export through the UI worker, using two initialisations
        # and one timing repeat to keep this smoke check bounded.
        with tempfile.TemporaryDirectory() as folder, \
                mock.patch("desktop_app.filedialog.askdirectory", return_value=folder), \
                mock.patch.object(loc, "BENCHMARK_REPEATS", 1), \
                mock.patch.object(loc, "REAL_INIT_CONDITIONS", (
                    ("from Day 1 pose", 0.0, 0.0, 1), ("0.30 m / 10 deg", 0.30, 10.0, 1))):
            page.evaluate()
            app.show_page("Vector Mapping")
            pump(root, lambda: not page.busy, timeout=90)
            app.show_page("Grid Localisation")
            outputs = list(Path(folder).glob("*.json"))
            assert len(outputs) == 1, page.info.cget("text")
            exported = json.loads(outputs[0].read_text())
            assert len(exported["runs"]) == 32
            assert all("excluded" in run["reference_map"] for run in exported["runs"])
            assert len(list(Path(folder).glob("*.csv"))) == 1
            from_day1 = [run for run in exported["runs"] if run["condition"] == "from Day 1 pose"]
            evaluation_summary = {
                "runs": len(from_day1),
                "statuses": dict(Counter(run["status"] for run in from_day1)),
                "median_runtime_ms": float(np.median([run["runtime_ms_median"] for run in from_day1])),
                "note": "One repeat per scan in the GUI smoke check; Day 1 poses are estimates."}
            print("Recorded evaluation:", evaluation_summary)
        # Zoom/pan uses the actual Matplotlib toolbar and suppresses pose clicks.
        initial = page.viewer.initial_pose()
        page.toolbar.pan()
        px, py = page.viewer.axes.transData.transform((1, 1))
        page.canvas.callbacks.process("button_press_event", MouseEvent("button_press_event", page.canvas, px, py, button=1))
        assert page.viewer.initial_pose() == initial
        page.toolbar.pan()
        with mock.patch("desktop_app.filedialog.askopenfilename", return_value=""):
            page.choose_recorded()
            app.choose_session()
        assert app.model.path.name == "classroom_final.json"
        with tempfile.TemporaryDirectory() as folder:
            bad = Path(folder) / "invalid.json"
            bad.write_text("bad json")
            assert not page.load_recorded(bad)
            assert "not valid JSON" in page.info.cget("text")
            with mock.patch("desktop_app.messagebox.showerror") as error:
                assert not app.load_session(bad)
                error.assert_called_once()
        # Hardware errors and simulated success are delivered on the UI thread.
        with mock.patch.object(part1, "open_serial_readonly", side_effect=part1.serial.SerialException("test missing port")):
            page.capture(page.captured)
            pump(root, lambda: not page.busy)
            assert "Load Recorded Scan" in page.info.cget("text")
        connection = part1.FakeSerial([part1.make_teleplot_line(part1.make_test_points(60))])
        connection.close = mock.Mock()
        with mock.patch.object(part1, "open_serial_readonly", return_value=connection):
            page.capture(page.captured)
            pump(root, lambda: not page.busy)
        connection.close.assert_called_once()
        assert page.viewer.mode == "new"
        assert page.viewer.scan["name"] == "Fresh LD06 capture"
        # Scanner changes are retained through navigation; dependent maps rebuild.
        app.show_page("LiDAR Scanner")
        scanner = app.pages["LiDAR Scanner"]
        scanner.action("select", 2)
        scanner.action("auto_align")
        pump(root, lambda: not scanner.editor["is_aligning"](), timeout=60)
        assert scanner.editor["proposal"]() is not None
        scanner.action("reject")
        scanner.set_pose((0.15, 1.95, 2.5))
        assert app.model.dirty
        app.show_page("Grid Mapping")
        replacement = app.pages["Grid Mapping"]
        pump(root, lambda: not replacement.busy)
        assert replacement.grid is not grid
        app.show_page("LiDAR Scanner")
        assert app.pages["LiDAR Scanner"] is scanner
        assert scanner.get_pose() == (0.15, 1.95, 2.5)
        with tempfile.TemporaryDirectory() as folder:
            copy_path = Path(folder) / "working_copy.json"
            with mock.patch("desktop_app.filedialog.asksaveasfilename", return_value=str(copy_path)):
                assert app.save_copy()
            assert app.model.path == copy_path.resolve()
            assert not app.model.dirty
            assert copy_path.is_file()
        scanner.set_pose((0.16, 1.95, 2.5))
        with mock.patch("desktop_app.messagebox.askyesnocancel", return_value=None):
            assert not app.load_session(common.SCANS_DIR / "classroom_session.json")
        with mock.patch("desktop_app.messagebox.askyesnocancel", return_value=False):
            assert app.load_session(common.SCANS_DIR / "classroom_session.json")
        assert len(app.model.scans) == 3
        app.show_page("Grid Localisation")
        pump(root, lambda: not app.pages["Grid Localisation"].busy)
        assert len(app.pages["Grid Localisation"].viewer.scans) == 3
        root.geometry("1100x720")
        pump(root)
        resized_page = app.pages["Grid Localisation"]
        assert resized_page.controls.winfo_width() > 250
        assert resized_page.mode.winfo_rootx() + resized_page.mode.winfo_width() <= root.winfo_rootx() + root.winfo_width()
        screenshot(root, "07_resized_session_changed")
        # Positive new-target workflow using a REAL held-out classroom scan.
        # Its file pose is deliberately wrong; the approximate guess is entered
        # separately. Agreement remains a consistency check, not ground truth.
        root.geometry("1440x900")
        with tempfile.TemporaryDirectory() as folder:
            data = json.loads((common.SCANS_DIR / "classroom_final.json").read_text())
            target = next(s for s in data["scans"] if s["id"] == 2)
            pose = tuple(target["pose"][key] for key in ("x_m", "y_m", "theta_deg"))
            reference = dict(data, scans=[s for s in data["scans"] if s["id"] != 2])
            target = dict(target, pose={"x_m": 999, "y_m": 999, "theta_deg": 179})
            ref_path = Path(folder) / "reference_without_scan2.json"
            query_path = Path(folder) / "heldout_scan2.json"
            ref_path.write_text(json.dumps(reference))
            query_path.write_text(json.dumps(dict(data, scans=[target])))
            assert app.load_session(ref_path)
            page = app.pages["Grid Localisation"]
            pump(root, lambda: not page.busy)
            assert page.load_recorded(query_path)
            assert page.global_search()  # default: no initial pose given
            page.align()
            pump(root, lambda: not page.busy, timeout=120)
            heldout = page.viewer.result
            assert heldout.status == "ok" and heldout.initial_pose is None, heldout
            deviation = common.pose_error(heldout.pose, common.pose_from_day1(pose))
            assert deviation[0] < 0.1 and deviation[1] < 3, deviation
            screenshot(root, "08_heldout_real_scan_aligned")
            print("Held-out real Scan 2:", heldout.to_dict(), "Day 1 agreement:", deviation)
            page.align()  # a second global search, cancelled: nothing changes
            page.cancel_search()
            pump(root, lambda: not page.busy, timeout=60)
            assert "cancelled" in page.info.cget("text") and page.viewer.result is heldout
        # Close while a capture is waiting: the reader must finish and close
        # its port before the Tk event loop is destroyed.
        connection = part1.FakeSerial([])
        connection.close = mock.Mock()
        with mock.patch.object(part1, "open_serial_readonly", return_value=connection):
            page.capture(page.captured)
            pump(root, lambda: page.capture_adapter.active)
            app.close()
            end = time.monotonic() + 3
            while page.capture_adapter.active and time.monotonic() < end:
                time.sleep(0.02)
            root.update()
            assert not page.capture_adapter.active
            connection.close.assert_called_once()
        assert not errors, errors
    finally:
        with mock.patch("desktop_app.messagebox.askyesnocancel", return_value=False):
            app.close()
    assert app.closed
    assert not plt.get_fignums()
    for path, digest in hashes.items():
        assert part2.file_sha256(path) == digest
    print("PASS: all six pages (incl. Codes 05 and 06 localisation), real mouse events, toolbar, new scan, evaluation, background navigation,")
    print("session changes, cache invalidation, cancellation, errors, simulated capture, resize, close.")
    (OUTPUT / "verification.json").write_text(json.dumps({
        "pages": visited, "callback_errors": errors, "separate_scan_result": result.to_dict(),
        "heldout_scan_result": heldout.to_dict(), "heldout_day1_agreement": deviation,
        "gui_export_runs": 32,
        "evaluation_from_day1": evaluation_summary,
        "source_files_unchanged": True, "physical_hardware_tested": False}, indent=2))
    standalone()


def standalone():
    """Render independent Day 1 windows through their actual entry functions."""
    shown = []

    def show():
        figure = plt.gcf()
        manager = figure.canvas.manager
        manager.show()
        manager.window.update()
        figure.canvas.draw()
        assert manager.window.winfo_exists()
        shown.append(manager.window.title())
        plt.close(figure)

    with mock.patch.object(part1, "RUN_MODE", "session_viewer"), \
            mock.patch.object(part1, "SESSION_FILE_TO_OPEN", "classroom_final.json"), \
            mock.patch.object(plt, "show", side_effect=show):
        part1.main()
    for module in (part2, part3):
        with mock.patch("tkinter.filedialog.askopenfilename", return_value=str(common.SCANS_DIR / "classroom_final.json")), \
                mock.patch.object(plt, "show", side_effect=show):
            module.cli([])
        with mock.patch("tkinter.filedialog.askopenfilename", return_value=""):
            module.cli([])
    assert len(shown) == 3
    # Exercise Code 04's normal PyCharm entry and the main app's CLI entry,
    # stopping their real event loops after the requested page is ready.
    import desktop_app

    def launch_checked(session_path=None, initial_page="LiDAR Scanner"):
        print("Checking standalone Tk page:", initial_page, flush=True)
        root = tk.Tk()
        app = DesktopApp(root, session_path, initial_page)
        failures = []

        def finish():
            page = app.pages[initial_page]
            if page.busy:
                root.after(50, finish)
            else:
                root.update_idletasks()
                if not page.canvas.get_tk_widget().winfo_ismapped():
                    failures.append(f"Page not mapped: {initial_page}; root state {root.state()}")
                app.close()

        root.after(100, finish)
        root.mainloop()
        assert app.closed
        assert not failures, failures

    with mock.patch.object(desktop_app, "launch", side_effect=launch_checked), \
            mock.patch.object(loc, "choose_session_file", return_value=common.SCANS_DIR / "classroom_final.json"):
        loc.start_from_pycharm()
        importlib.import_module("00_main_app").main([])
    assert not plt.get_fignums()
    print("PASS: standalone Codes 01, 02, 03, 04 and 00 main entry; cancelled 02/03 pickers.")


if __name__ == "__main__":
    main()
