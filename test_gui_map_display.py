"""Regression: built maps must become VISIBLE in the real desktop window.

Run: python -m unittest test_gui_map_display -v   (opens a real Tk window)

The defect: while a combobox drop-down was open, PoseInputs.sync() called
focus_get(), which raises KeyError('popdown') for Tk's internal drop-down
listbox. The exception escaped the 80 ms poller, which then never ran again,
so every finished map stayed queued and 02 Grid / 03 Vector showed
"Building..." forever. These tests use the real event loop (mainloop) and
read the pixels Tk is actually displaying, not just Matplotlib state.
"""
import base64
import io
import time
import tkinter as tk
import unittest
from unittest import mock

import numpy as np
from PIL import Image

import desktop_app
import localisation_common as common
from app_session import part2

SESSION = common.SCANS_DIR / "classroom_final.json"
VIEW_BG = np.array([int(desktop_app.VIEW_BG[i:i + 2], 16) for i in (1, 3, 5)])


def displayed_pixels(page):
    """RGB pixels of the Tk image currently shown in the page's map canvas."""
    widget = page.canvas.get_tk_widget()
    name = [widget.itemcget(i, "image") for i in widget.find_all() if widget.type(i) == "image"][-1]
    data = widget.tk.call(name, "data", "-format", "png")
    raw = data if isinstance(data, bytes) else base64.b64decode(data)
    image = np.asarray(Image.open(io.BytesIO(raw)).convert("RGB")).astype(int)
    return image, (widget.winfo_width(), widget.winfo_height())


def map_is_visible(page):
    """Real map content on screen: light map area and dark walls/segments,
    not the plain viewport (the "Building..." placeholder is ~100 % VIEW_BG)."""
    image, size = displayed_pixels(page)
    pixels = image.reshape(-1, 3)
    light = np.mean(np.all(pixels > 200, axis=1))
    dark = np.mean(np.all(pixels < 40, axis=1))
    background = np.mean(np.all(np.abs(pixels - VIEW_BG) < 5, axis=1))
    fits = (image.shape[1], image.shape[0]) == size
    return bool(light > 0.04 and dark > 0.002 and background < 0.8 and fits)


class MapDisplayTests(unittest.TestCase):
    def setUp(self):
        self.hashes = {p: part2.file_sha256(p) for p in common.SCANS_DIR.glob("*.json")}
        self.root = tk.Tk()
        self.root.geometry("1440x900+0+0")
        self.errors = []
        self.root.report_callback_exception = lambda t, v, tb: self.errors.append(repr(v))
        self.app = desktop_app.DesktopApp(self.root, SESSION)

    def tearDown(self):
        if not self.app.closed:
            with mock.patch("desktop_app.messagebox.askyesnocancel", return_value=False):
                self.app.close()
        try:
            self.root.destroy()  # normally already destroyed by app.close()
        except tk.TclError:
            pass
        for path, digest in self.hashes.items():
            self.assertEqual(part2.file_sha256(path), digest)

    def drive(self, steps, timeout=60):
        """Run `steps` (a generator yielding seconds or predicates) in mainloop()."""
        outcome = {}
        deadline = time.monotonic() + timeout

        def advance():
            try:
                command = next(steps)
            except StopIteration:
                outcome["done"] = True
                self.root.quit()
                return
            except Exception as exc:  # assertion inside the scenario
                outcome["error"] = exc
                self.root.quit()
                return
            if callable(command):
                def wait():
                    if command() or time.monotonic() > deadline:
                        advance()
                    else:
                        self.root.after(50, wait)
                self.root.after(50, wait)
            else:
                self.root.after(int(command * 1000), advance)

        self.root.after(10, advance)
        self.root.after(int(timeout * 1000) + 5000, self.root.quit)  # watchdog
        self.root.mainloop()
        if "error" in outcome:
            raise outcome["error"]
        self.assertTrue(outcome.get("done"), "scenario did not finish (event loop stalled?)")

    def hold_dropdown(self, combo, index, held=0.5):
        """Open the real drop-down, keep it open across several polls, pick an item."""
        combo.tk.call("ttk::combobox::Post", combo)
        yield held
        listbox = combo.tk.eval(f"ttk::combobox::PopdownWindow {combo}") + ".f.l"
        combo.tk.call(listbox, "selection", "clear", 0, "end")
        combo.tk.call(listbox, "selection", "set", index)
        combo.tk.call("ttk::combobox::LBSelected", listbox)
        yield 0.05

    def visible_page(self, name, title=None):
        def ready():
            page = self.app.pages.get(name)
            return (page is not None and not page.busy and page.figure.axes
                    and (title is None or page.map_axes.get_title().startswith(title))
                    and map_is_visible(page))
        return ready

    def test_grid_and_vector_maps_appear_after_dropdown_use(self):
        app = self.app

        def scenario():
            yield 0.8
            scanner = app.pages["LiDAR Scanner"]
            yield from self.hold_dropdown(scanner.scan_choice, 3)
            app.nav_buttons["Grid Mapping"].invoke()
            yield self.visible_page("Grid Mapping", "Probability")
            yield 0.3
            grid = app.pages["Grid Mapping"]
            self.assertTrue(map_is_visible(grid), "Probability grid not visible")
            yield from self.hold_dropdown(grid.view, list(grid.view["values"]).index("Binary"))
            yield self.visible_page("Grid Mapping", "Binary")
            yield 0.3
            self.assertTrue(map_is_visible(grid), "Binary grid not visible")
            self.assertTrue(grid.map_axes.get_title().startswith("Binary"))
            app.nav_buttons["Vector Mapping"].invoke()
            yield self.visible_page("Vector Mapping")
            yield 0.3
            vector = app.pages["Vector Mapping"]
            self.assertTrue(map_is_visible(vector), "Vector map not visible")
            self.assertTrue(vector.map_axes.get_title().startswith("Vector map"))

        self.drive(scenario())
        self.assertEqual(self.errors, [])

    def test_rapid_page_switching_while_maps_build(self):
        app = self.app

        def scenario():
            yield 0.5
            for name in ("Grid Mapping", "LiDAR Scanner", "Vector Mapping", "Grid Mapping",
                         "Vector Mapping", "LiDAR Scanner", "Grid Mapping"):
                app.nav_buttons[name].invoke()
                yield 0.05
            yield self.visible_page("Grid Mapping", "Probability")
            yield 0.3
            self.assertTrue(map_is_visible(app.pages["Grid Mapping"]))
            app.nav_buttons["Vector Mapping"].invoke()
            yield self.visible_page("Vector Mapping")
            yield 0.3
            self.assertTrue(map_is_visible(app.pages["Vector Mapping"]))

        self.drive(scenario())
        self.assertEqual(self.errors, [])

    def test_pose_sync_tolerates_focus_in_a_dropdown(self):
        scanner = self.app.pages["LiDAR Scanner"]
        scanner.scan_choice.tk.call("ttk::combobox::Post", scanner.scan_choice)
        self.root.update()
        try:
            self.assertIn("popdown", str(self.root.tk.call("focus")))
            scanner.pose.sync()  # previously KeyError('popdown')
        finally:
            scanner.scan_choice.tk.call("ttk::combobox::Unpost", scanner.scan_choice)

    def test_poller_survives_a_failing_tick_and_still_delivers(self):
        app = self.app
        scanner = app.pages["LiDAR Scanner"]
        with mock.patch.object(scanner, "tick", side_effect=KeyError("popdown")), \
                mock.patch("traceback.print_exception"):
            delivered = []
            app.submit(scanner, "probe", lambda: 42, delivered.append)

            def scenario():
                yield lambda: delivered
                yield 0.3  # several more polls after the failure

            self.drive(scenario(), timeout=10)
        self.assertEqual(delivered, [42])
        self.assertIn("KeyError", scanner.info.cget("text"))


class MissingFontStartupTests(unittest.TestCase):
    """assets/fonts/local/ is git-ignored, so a fresh checkout has no font file."""

    def test_startup_without_local_font_uses_fallbacks(self):
        hashes = {p: part2.file_sha256(p) for p in common.SCANS_DIR.glob("*.json")}
        missing = common.SCANS_DIR.parent / "assets/fonts/local/absent-for-test.ttf"
        self.assertFalse(missing.exists())
        root = tk.Tk()
        errors = []
        root.report_callback_exception = lambda t, v, tb: errors.append(repr(v))
        app = None
        try:
            with mock.patch("ui_fonts.FONT_PATH", missing):
                app = desktop_app.DesktopApp(root, SESSION, initial_page="Workspace")
            fonts = app.fonts
            self.assertFalse(fonts.tk_available)
            self.assertFalse(fonts.mpl_available)
            self.assertEqual(fonts.heading, ("Segoe UI", 10, "bold"))
            self.assertEqual(fonts.page_title, ("Segoe UI", 10, "bold"))
            self.assertEqual(fonts.technical("SCAN 1"), ("Consolas", 10))
            self.assertEqual(fonts.plot("X (m)").get_family(), ["DejaVu Sans"])
            for name in desktop_app.PAGES:
                app.show_page(name)
                deadline = time.monotonic() + 60
                while app.pages[name].busy and time.monotonic() < deadline:
                    root.update()
                    time.sleep(0.02)
                self.assertFalse(app.pages[name].busy, name)
            self.assertEqual(errors, [])
        finally:
            if app is not None:
                with mock.patch("desktop_app.messagebox.askyesnocancel", return_value=False):
                    app.close()
            try:
                root.destroy()
            except tk.TclError:
                pass
        for path, digest in hashes.items():
            self.assertEqual(part2.file_sha256(path), digest)


if __name__ == "__main__":
    unittest.main()
