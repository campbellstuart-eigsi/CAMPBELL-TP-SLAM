"""Workspace acceptance checks in a real Tk mainloop at both requested sizes."""
from contextlib import ExitStack
import ctypes
import json
from pathlib import Path
import time
import threading
import tkinter as tk
import unittest
from unittest import mock

from matplotlib.backend_bases import MouseEvent
import numpy as np
from PIL import ImageGrab

import desktop_app
import workspace_page
from app_session import part1, part2
import localisation_common as common
import test_gui_map_display as gui


OUTPUT = Path(__file__).resolve().parent / "results" / "workspace"


class WorkspaceGuiTests(unittest.TestCase):
    hold_dropdown = gui.MapDisplayTests.hold_dropdown

    def drive(self, steps, timeout=60):
        """Run interaction steps without leaving watchdogs on destroyed roots."""
        outcome = {}
        step_job = None

        def fail(error):
            outcome["error"] = error
            self.root.quit()

        def schedule(delay, callback):
            nonlocal step_job
            step_job = self.root.after(delay, callback)

        def advance():
            try:
                command = next(steps)
            except StopIteration:
                outcome["done"] = True
                self.root.quit()
                return
            except Exception as error:
                fail(error)
                return
            if callable(command):
                def wait():
                    try:
                        if command():
                            advance()
                        else:
                            schedule(50, wait)
                    except Exception as error:
                        fail(error)
                schedule(50, wait)
            else:
                schedule(int(command * 1000), advance)

        watchdog = self.root.after(int(timeout * 1000), lambda: fail(AssertionError("GUI scenario timed out")))
        schedule(10, advance)
        try:
            self.root.mainloop()
        finally:
            self.root.after_cancel(watchdog)
            if step_job is not None:
                self.root.after_cancel(step_job)
        if "error" in outcome:
            raise outcome["error"]
        self.assertTrue(outcome.get("done"))

    def setUp(self):
        OUTPUT.mkdir(parents=True, exist_ok=True)
        self.hashes = {p: part2.file_sha256(p) for p in common.SCANS_DIR.glob("*.json")}
        self.root = tk.Tk()
        self.errors = []
        self.root.report_callback_exception = lambda t, v, tb: self.errors.append(repr(v))
        self.app = desktop_app.DesktopApp(self.root, gui.SESSION)

    def tearDown(self):
        with mock.patch("desktop_app.messagebox.askyesnocancel", return_value=False):
            self.app.close()
        for path, digest in self.hashes.items():
            self.assertEqual(part2.file_sha256(path), digest)

    def screenshot(self, name):
        self.root.update_idletasks()
        hwnd = ctypes.windll.user32.GetParent(self.root.winfo_id())
        ImageGrab.grab(window=hwnd).save(OUTPUT / f"{name}.png")

    def assert_geometry(self, view):
        self.assertTrue(view.frame.winfo_ismapped())
        image, size = gui.displayed_pixels(view)
        self.assertEqual((image.shape[1], image.shape[0]), size)
        axes = view.axes
        transform = axes.transData.transform
        origin, x, y = transform([[0, 0], [1, 0], [0, 1]])
        self.assertAlmostEqual(np.linalg.norm(x - origin), np.linalg.norm(y - origin), places=6)
        self.assertEqual(axes.get_xlabel(), "X (m)")
        self.assertEqual(axes.get_ylabel(), "Y (m)")
        self.assertGreater(view.canvas.get_tk_widget().winfo_width(), 150)

    def mouse(self, view, kind, xy, **kwargs):
        x, y = view.axes.transData.transform(xy)
        if kind == "motion_notify_event" and "button" in kwargs:
            kwargs["buttons"] = {kwargs.pop("button")}
        event = MouseEvent(kind, view.canvas, x, y, **kwargs)
        view.canvas.callbacks.process(kind, event)

    def exercise(self, width, height):
        app = self.app
        measurements = {}
        build_grid = mock.Mock(wraps=desktop_app.build_grid)
        build_vectors = mock.Mock(wraps=desktop_app.build_vectors)
        binary = mock.Mock(wraps=part2.build_binary_occupancy_grid)
        transforms = mock.Mock(wraps=part2.global_points_of_scans)
        with ExitStack() as stack:
            for module in (desktop_app, workspace_page):
                stack.enter_context(mock.patch.object(module, "build_grid", build_grid))
                stack.enter_context(mock.patch.object(module, "build_vectors", build_vectors))
            stack.enter_context(mock.patch.object(part2, "build_binary_occupancy_grid", binary))
            stack.enter_context(mock.patch.object(part2, "global_points_of_scans", transforms))

            def scenario():
                self.root.geometry(f"{width}x{height}+0+0")
                started = time.perf_counter()
                app.nav_buttons["Workspace"].invoke()
                page = app.pages["Workspace"]
                yield lambda: not page.busy
                measurements["maps_ready_s"] = time.perf_counter() - started
                yield 0.7
                measurements["cold_workspace_with_settle_s"] = time.perf_counter() - started
                self.assertTrue(hasattr(page, "grid"), page.info.cget("text"))
                self.assertTrue(hasattr(page, "segments"), page.info.cget("text"))
                self.assertEqual((self.root.winfo_width(), self.root.winfo_height()), (width, height))
                self.assertEqual(list(page.views), ["scan", "grid", "vector", "current"])
                np.testing.assert_allclose(page.divider_ratios(), [0.5, 0.5, 0.5], atol=0.01)
                self.assertEqual((build_grid.call_count, build_vectors.call_count), (1, 1))
                for view in page.views.values():
                    self.assert_geometry(view)
                scan, grid, vector, current = (page.views[k] for k in page.views)
                self.assertLess(scan.frame.winfo_rootx(), grid.frame.winfo_rootx())
                self.assertLess(scan.frame.winfo_rooty(), vector.frame.winfo_rooty())
                self.assertLess(vector.frame.winfo_rootx(), current.frame.winfo_rootx())
                self.screenshot(f"workspace_{width}x{height}")
                np.testing.assert_array_equal(page.current_points.get_offsets(),
                    np.column_stack(part1.scan_to_local(part1.selected_scan(app.model.session))))
                np.testing.assert_allclose(page.segments.get_segments(),
                    [[segment["start"], segment["end"]] for segment in app.model.cache["vectors"][3]])
                for artist, points in zip(page.scan_artists, part2.global_points_of_scans(app.model.scans)):
                    np.testing.assert_array_equal(artist.get_offsets(), points)
                self.assertIs(page.grid, app.model.cache["grid"][0])

                # Hold actual Tk popdowns over several poll cycles.
                for mode in ("Binary", "Classified", "Empty", "Probability"):
                    yield from self.hold_dropdown(page.view, GridModes.index(mode))
                    yield lambda: not page.busy
                    yield 0.15
                    self.assertIn(mode, grid.caption.cget("text"))
                    self.assertNotIn("Building", page.info.cget("text"))
                    expected = app.model.cache["binary"] if mode == "Binary" else page.grid_arrays[mode]
                    np.testing.assert_array_equal(page.grid_image.get_array(), expected)
                self.assertEqual(binary.call_count, 1)
                revision = app.model.revision
                cache = dict(app.model.cache)
                points_calls = transforms.call_count
                started = time.perf_counter()
                yield from self.hold_dropdown(page.scan_choice, 3)
                yield 0.2
                measurements["selection_with_held_dropdown_s"] = time.perf_counter() - started
                selected = app.model.session["scans"][3]
                self.assertEqual(app.model.session["selected_id"], selected["id"])
                self.assertEqual(app.pages["LiDAR Scanner"].scan_choice.current(), 3)
                np.testing.assert_array_equal(page.current_points.get_offsets(), np.column_stack(part1.scan_to_local(selected)))
                self.assertEqual(app.model.revision, revision)
                self.assertFalse(app.model.dirty)

                # Optional linking; wheel zoom and real toolbar Pan/Zoom drags.
                page.link.set(True)
                page.link_changed()
                yield 0.15
                local_limits = current.axes.get_xlim(), current.axes.get_ylim()
                self.mouse(scan, "scroll_event", (1, 1), button="up", step=1)
                yield 0.2
                for view in (grid, vector):
                    np.testing.assert_allclose(view.axes.get_xlim(), scan.axes.get_xlim())
                    np.testing.assert_allclose(view.axes.get_ylim(), scan.axes.get_ylim())
                before = scan.axes.get_xlim()
                app.toggle_pan()
                self.mouse(scan, "button_press_event", (1, 1), button=1)
                self.mouse(scan, "motion_notify_event", (1.5, 1.2), button=1)
                self.mouse(scan, "button_release_event", (1.5, 1.2), button=1)
                yield 0.2
                self.assertFalse(np.allclose(before, scan.axes.get_xlim()))
                np.testing.assert_allclose(grid.axes.get_xlim(), scan.axes.get_xlim())
                np.testing.assert_allclose((current.axes.get_xlim(), current.axes.get_ylim()), local_limits)
                app.toggle_pan()
                page.activate("vector")
                app.toggle_zoom()
                self.mouse(vector, "button_press_event", (0, 0), button=1)
                self.mouse(vector, "motion_notify_event", (2, 2), button=1)
                self.mouse(vector, "button_release_event", (2, 2), button=1)
                yield 0.2
                np.testing.assert_allclose(grid.axes.get_xlim(), vector.axes.get_xlim())
                self.assertLess(np.ptp(vector.axes.get_xlim()), np.ptp(before))
                app.toggle_zoom()
                page.toolbar.back()
                yield 0.2
                np.testing.assert_allclose(grid.axes.get_xlim(), vector.axes.get_xlim())
                global_limits = grid.axes.get_xlim(), grid.axes.get_ylim()
                page.activate("current")
                self.mouse(current, "scroll_event", (1, 1), button="up", step=1)
                yield 0.2
                np.testing.assert_allclose((grid.axes.get_xlim(), grid.axes.get_ylim()), global_limits)
                self.assertFalse(np.allclose(current.axes.get_xlim(), local_limits[0]))
                page.link.set(False)
                self.mouse(scan, "scroll_event", (1, 1), button="up", step=1)
                yield 0.2
                np.testing.assert_allclose((grid.axes.get_xlim(), grid.axes.get_ylim()), global_limits)

                # Move each sash, maximise/restore every panel, preserve cameras.
                page.split.sash_place(0, 0, int(page.split.winfo_height() * 0.58))
                page.rows[0].sash_place(0, int(page.rows[0].winfo_width() * 0.57), 0)
                page.rows[1].sash_place(0, int(page.rows[1].winfo_width() * 0.43), 0)
                yield 0.7
                ratios = page.divider_ratios()
                for key, view in page.views.items():
                    limits = view.axes.get_xlim(), view.axes.get_ylim()
                    view.max_button.invoke()
                    yield 0.5
                    self.assertEqual(page.maximised, key)
                    self.assertEqual([k for k, v in page.views.items() if v.frame.winfo_ismapped()], [key])
                    self.assert_geometry(view)
                    np.testing.assert_allclose((view.axes.get_xlim(), view.axes.get_ylim()), limits)
                    if key == "current":
                        self.screenshot(f"current_maximised_{width}x{height}")
                    view.max_button.invoke()
                    yield 0.5
                    np.testing.assert_allclose(page.divider_ratios(), ratios, atol=0.01)
                    for restored in page.views.values():
                        self.assert_geometry(restored)
                self.root.geometry(f"{width - 100}x{height - 80}")
                yield 0.6
                self.root.geometry(f"{width}x{height}")
                yield 0.6
                for view in page.views.values():
                    self.assert_geometry(view)
                self.assertEqual(transforms.call_count, points_calls)
                self.assertEqual((build_grid.call_count, build_vectors.call_count, binary.call_count), (1, 1, 1))
                for key, value in cache.items():
                    self.assertIs(app.model.cache[key], value)

                # One-shot capture: existing acquisition, no map insertion or pose.
                serial = part1.FakeSerial([part1.make_teleplot_line(part1.make_test_points(60))])
                serial.close = mock.Mock()
                with mock.patch.object(part1, "open_serial_readonly", return_value=serial):
                    page.start_capture()
                    yield lambda: not page.busy
                serial.close.assert_called_once()
                self.assertEqual(page.source.get(), page.SOURCES[1])
                self.assertEqual(app.model.revision, revision)
                self.assertEqual(len(app.model.scans), 16)
                np.testing.assert_array_equal(page.current_points.get_offsets(),
                                              app.model.workspace_inputs.latest_capture.points_local)
                self.assertIsNone(app.model.workspace_inputs.robot_pose_source)

                # Return to every original page and back with the same caches.
                for name in tuple(desktop_app.PAGES)[:6]:
                    app.nav_buttons[name].invoke()
                    yield lambda: (not app.pages[name].busy and
                                   app.pages[name].canvas.get_tk_widget().winfo_ismapped())
                    self.assertTrue(app.pages[name].canvas.get_tk_widget().winfo_ismapped(),
                                    f"{name}; active={app.current}; errors={self.errors}")
                self.assertIs(app.pages["Grid Mapping"].grid, page.grid)
                started = time.perf_counter()
                app.nav_buttons["Workspace"].invoke()
                measurements["warm_page_switch_ms"] = (time.perf_counter() - started) * 1000
                yield 0.5
                measurements["return_with_settle_s"] = time.perf_counter() - started
                self.assertIs(app.pages["Workspace"], page)
                self.assertEqual((build_grid.call_count, build_vectors.call_count, binary.call_count), (1, 1, 1))
                measurements["build_counts"] = {"grid": build_grid.call_count,
                                                 "vectors": build_vectors.call_count,
                                                 "binary": binary.call_count}
                for view in page.views.values():
                    started = time.perf_counter()
                    view.canvas.draw()
                    measurements[f"draw_{view.key}_ms"] = (time.perf_counter() - started) * 1000
                yield 0.4
                with ExitStack() as draws:
                    counters = [draws.enter_context(mock.patch.object(v.canvas, "draw", wraps=v.canvas.draw))
                                for v in page.views.values()]
                    yield 0.4
                    self.assertEqual([draw.call_count for draw in counters], [0, 0, 0, 0])
                self.assertIsNone(app.last_poll_error)

            self.drive(scenario(), timeout=100)
        self.assertEqual(self.errors, [])
        (OUTPUT / f"metrics_{width}x{height}.json").write_text(json.dumps(measurements, indent=2))

    def test_workspace_1440x900(self):
        self.exercise(1440, 900)

    def test_workspace_1920x1080(self):
        self.exercise(1920, 1080)

    def test_builds_shared_during_rapid_navigation_and_session_change(self):
        app = self.app
        gate, binary_gate = threading.Event(), threading.Event()
        real_grid, real_vectors = workspace_page.build_grid, workspace_page.build_vectors
        real_binary = part2.build_binary_occupancy_grid

        def delayed(build, event, *args):
            if not event.wait(15):
                raise RuntimeError("Test build gate timed out")
            return build(*args)

        with mock.patch.object(workspace_page, "build_grid", side_effect=lambda scans: delayed(real_grid, gate, scans)) as grid, \
                mock.patch.object(workspace_page, "build_vectors", side_effect=lambda scans: delayed(real_vectors, gate, scans)) as vectors, \
                mock.patch.object(part2, "build_binary_occupancy_grid", side_effect=lambda *args: delayed(real_binary, binary_gate, *args)) as binary:
            def scenario():
                app.show_page("Workspace")
                page = app.pages["Workspace"]
                for name in ("Grid Mapping", "Vector Mapping", "LiDAR Scanner", "Workspace"):
                    app.show_page(name)
                    yield 0.1
                self.assertTrue(page.busy)
                gate.set()
                yield lambda: not page.busy
                self.assertIs(app.pages["Grid Mapping"].grid, page.grid)
                self.assertEqual((grid.call_count, vectors.call_count), (1, 1))
                page.view.set("Binary")
                page.draw_grid()
                app.show_page("Grid Mapping")
                old_grid_page = app.pages["Grid Mapping"]
                old_grid_page.view.set("Binary")
                old_grid_page.draw_map()
                yield 0.2
                self.assertEqual(binary.call_count, 1)
                self.assertTrue(old_grid_page.busy)
                # Edit while the shared binary worker is still pending. Neither
                # waiting page may publish its obsolete map into the new cache.
                app.show_page("LiDAR Scanner")
                scanner = app.pages["LiDAR Scanner"]
                scanner.set_pose((0.1, 0.2, 3))
                app.show_page("Workspace")
                replacement = app.pages["Workspace"]
                self.assertIsNot(replacement, page)
                self.assertFalse(page.alive)
                yield lambda: not replacement.busy
                binary_gate.set()
                yield lambda: not app.jobs
                self.assertNotIn("binary", app.model.cache)
                self.assertIs(app.model.cache["grid"][0], replacement.grid)
                self.assertEqual((grid.call_count, vectors.call_count), (2, 2))
                self.assertIsNone(app.last_poll_error)
            try:
                self.drive(scenario(), timeout=60)
            finally:
                gate.set()
                binary_gate.set()
        self.assertEqual(self.errors, [])

    def test_hidden_scans_selection_on_return_and_empty_session(self):
        app = self.app

        def scenario():
            app.show_page("Workspace")
            page = app.pages["Workspace"]
            yield lambda: not page.busy
            yield 0.5
            before, _ = gui.displayed_pixels(page.views["current"])
            app.show_page("LiDAR Scanner")
            scanner = app.pages["LiDAR Scanner"]
            scanner.action("select", 2)
            yield 0.2  # Workspace observes selection while its canvases are hidden
            app.show_page("Workspace")
            yield 0.5
            after, _ = gui.displayed_pixels(page.views["current"])
            self.assertFalse(np.array_equal(before, after))
            self.assertEqual(page.scan_choice.current(), 1)
            app.show_page("LiDAR Scanner")
            for scan in app.model.session["scans"]:
                part1.set_scan_visible(app.model.session, scan["id"], False)
            app.show_page("Workspace")
            hidden = app.pages["Workspace"]
            yield 0.5
            self.assertFalse(hidden.busy)
            self.assertFalse(hasattr(hidden, "grid"))
            for marker, heading in hidden.pose_artists.values():
                self.assertFalse(marker.get_visible())
                self.assertFalse(heading.get_visible())
            np.testing.assert_array_equal(hidden.current_points.get_offsets(),
                np.column_stack(part1.scan_to_local(part1.selected_scan(app.model.session))))
            # Load a valid empty session through the existing Open Session path.
            import tempfile
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "empty.json"
                part1.save_session(part1.create_session(), path)
                with mock.patch("desktop_app.messagebox.askyesnocancel", return_value=False):
                    self.assertTrue(app.load_session(path))
                yield 0.5
                empty = app.pages["Workspace"]
                self.assertEqual(empty.scan_choice.get(), "")
                self.assertEqual(len(empty.current_points.get_offsets()), 0)
                self.assertIsNone(app.model.workspace_inputs.latest_capture)
                self.assertFalse(empty.busy)
            self.assertIsNone(app.last_poll_error)

        self.drive(scenario(), timeout=40)
        self.assertEqual(self.errors, [])


GridModes = list(desktop_app.GridPage.MODES)


class WorkspaceCloseResizeTests(unittest.TestCase):
    """Regression: closing while a viewport resize is pending raised
    TclError("can't delete Tcl command"). Viewport.on_configure schedules the
    settle callback on the canvas widget; Viewport.close cancelled it through
    the frame, which deleted the Tcl command but left it in the canvas's
    command list, so the canvas's destroy() failed."""

    def open_workspace(self, geometry="1440x900+0+0"):
        root = tk.Tk()
        root.geometry(geometry)
        errors = []
        root.report_callback_exception = lambda t, v, tb: errors.append(repr(v))
        app = desktop_app.DesktopApp(root, gui.SESSION, initial_page="Workspace")
        page = app.pages["Workspace"]
        self.pump(root, lambda: not page.busy and not any(
            v.resize_job for v in page.views.values()))
        return root, app, page, errors

    def pump(self, root, condition, timeout=60):
        deadline = time.monotonic() + timeout
        while not condition():
            self.assertLess(time.monotonic(), deadline, "GUI condition timed out")
            root.update()
            time.sleep(0.02)

    def close(self, app):
        with mock.patch("desktop_app.messagebox.askyesnocancel", return_value=False):
            app.close()

    def setUp(self):
        self.hashes = {p: part2.file_sha256(p) for p in common.SCANS_DIR.glob("*.json")}

    def tearDown(self):
        for path, digest in self.hashes.items():
            self.assertEqual(part2.file_sha256(path), digest)

    def test_close_while_resize_callbacks_are_pending(self):
        for attempt in range(3):
            with self.subTest(attempt=attempt):
                root, app, page, errors = self.open_workspace()
                root.geometry("1300x820")
                root.update()
                self.assertTrue(all(v.resize_job for v in page.views.values()),
                                "every viewport must have a pending resize callback")
                self.close(app)  # previously TclError: can't delete Tcl command
                self.assertTrue(app.closed)
                self.assertEqual(errors, [])

    def test_resize_still_settles_and_redraws_at_new_size(self):
        root, app, page, errors = self.open_workspace()
        try:
            before = {k: v.canvas.get_tk_widget().winfo_width() for k, v in page.views.items()}
            root.geometry("1200x780")
            root.update()
            self.assertTrue(any(v.resize_job for v in page.views.values()))
            self.pump(root, lambda: not any(v.resize_job for v in page.views.values()))
            root.update()
            for key, view in page.views.items():
                widget = view.canvas.get_tk_widget()
                self.assertNotEqual(widget.winfo_width(), before[key], key)
                width, height = view.figure.get_size_inches() * view.figure.dpi
                self.assertEqual((round(width), round(height)),
                                 (widget.winfo_width(), widget.winfo_height()), key)
            self.assertEqual(errors, [])
        finally:
            self.close(app)


class WorkspacePoseColourTests(unittest.TestCase):
    """The Workspace uses the approved pose vocabulary in both plot styles."""
    open_workspace = WorkspaceCloseResizeTests.open_workspace
    pump = WorkspaceCloseResizeTests.pump
    close = WorkspaceCloseResizeTests.close

    def test_recorded_pose_green_and_axis_cross_follows_style(self):
        from matplotlib.colors import to_hex
        root, app, page, errors = self.open_workspace()
        try:
            for style in workspace_page.PLOT_STYLES:
                page.plot_style.set(style)
                page.set_plot_style()
                root.update()
                expected = workspace_page.PLOT_STYLES[style]
                for marker, heading in page.pose_artists.values():
                    self.assertEqual(to_hex(marker.get_markeredgecolor()), to_hex(expected["recorded"]))
                    self.assertEqual(to_hex(heading.get_color()), to_hex(expected["recorded"]))
                self.assertEqual(to_hex(page.highlight.get_facecolor()[0]), to_hex(expected["recorded"]))
                self.assertEqual(to_hex(page.axis_cross.get_color()), to_hex(expected["axis"]), style)
                self.assertEqual(to_hex(page.axis_label.get_color()), to_hex(expected["axis"]), style)
            self.assertEqual(errors, [])
        finally:
            self.close(app)


if __name__ == "__main__":
    unittest.main()
