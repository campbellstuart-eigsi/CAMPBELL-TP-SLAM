"""Single Tk event loop hosting the existing Matplotlib tools.

Numerical work and serial reads run on workers. Only the Tk queue poller calls
completion handlers, creates artists, or touches widgets. No pyplot managers
are created by embedded pages.
"""
import argparse
import queue
import threading
import time
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.collections import LineCollection
from matplotlib.figure import Figure
from matplotlib.lines import Line2D

import localisation_common as common
from app_session import (SharedSession, CaptureAdapter, build_grid, build_vectors,
                         part1, part2, part3, grid_localisation as loc,
                         vector_localisation as vloc, scan_localisation as sloc)

BG, PANEL, ACCENT, TEXT, MUTED = "#0F1B2A", "#192B3D", "#39B5A5", "#EDF3F6", "#A4B8C9"
PROJECT = Path(__file__).resolve().parent


def button(parent, text, command):
    widget = ttk.Button(parent, text=text, command=command)
    widget.pack(fill="x", pady=3)
    return widget


def note(parent, text, **options):
    widget = ttk.Label(parent, text=text, wraplength=290, justify="left", **options)
    widget.pack(fill="x", pady=(5, 7))
    return widget


def section(parent, text):
    return note(parent, text, style="Section.TLabel")


class Page:
    def __init__(self, app):
        self.app = app
        self.model = app.model
        self.revision = self.model.revision
        self.alive = True
        self.busy = False
        self.frame = ttk.Frame(app.workspace, style="Background.TFrame")
        self.frame.columnconfigure(0, weight=1)
        self.frame.columnconfigure(1, minsize=330)
        self.frame.rowconfigure(0, weight=1)
        self.plot_frame = ttk.Frame(self.frame, style="Background.TFrame")
        self.plot_frame.grid(row=0, column=0, sticky="nsew")
        rail = ttk.Frame(self.frame, width=330)
        rail.grid(row=0, column=1, sticky="nsew")
        rail.pack_propagate(False)
        scroll = tk.Canvas(rail, bg=PANEL, highlightthickness=0, width=308)
        bar = ttk.Scrollbar(rail, orient="vertical", command=scroll.yview)
        bar.pack(side="right", fill="y")
        scroll.pack(side="left", fill="both", expand=True)
        scroll.configure(yscrollcommand=bar.set)
        self.controls = ttk.Frame(scroll, padding=14)
        window = scroll.create_window((0, 0), window=self.controls, anchor="nw")
        self.controls.bind("<Configure>", lambda _: scroll.configure(scrollregion=scroll.bbox("all")))
        scroll.bind("<Configure>", lambda e: scroll.itemconfigure(window, width=e.width))
        self.figure = Figure(figsize=(9, 7), facecolor=BG)
        # Attach BEFORE invoking builders: their widget callbacks and timers
        # belong to this actual Tk canvas, never an interim pyplot window.
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.plot_frame)
        toolbar_frame = ttk.Frame(self.plot_frame)
        toolbar_frame.pack(side="bottom", fill="x")
        self.toolbar = NavigationToolbar2Tk(self.canvas, toolbar_frame, pack_toolbar=False)
        self.toolbar.pack(fill="x")
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        # Lay the page out before builders draw: the canvas then has its final
        # size, so its first (100–200 ms) draw is not repeated after a resize.
        self.frame.pack(fill="both", expand=True)
        self.frame.update_idletasks()
        self.info = None
        self.capture_adapter = None
        self.placeholder = None
        self.locked = []

    def prepare_plot(self, axes, compact=True):
        if compact:
            for ax in self.figure.axes:
                if ax is not axes:
                    ax.set_visible(False)
                    # Check/RadioButtons can draw their artists directly on a
                    # canvas draw event, even when their Axes is hidden.
                    for artist in ax.get_children():
                        artist.set_visible(False)
            for text in self.figure.texts:
                text.set_visible(False)
            axes.set_position([0.12, 0.10, 0.84, 0.82])
        for ax in self.figure.axes:
            if not ax.get_visible():
                continue
            ax.set_facecolor("#F3F5F7")
            ax.tick_params(colors=MUTED)
            ax.xaxis.label.set_color(TEXT)
            ax.yaxis.label.set_color(TEXT)
            ax.title.set_color(TEXT)
            ax.title.set_fontsize(10)
            for spine in ax.spines.values():
                spine.set_color(MUTED)
        self.canvas.draw_idle()

    def set_info(self, text):
        if self.info is None:
            self.info = note(self.controls, text)
        else:
            self.info.configure(text=text)

    def set_busy(self, busy, label=None, controls=()):
        """Busy: wait cursor, locked buttons and, on a still-empty map, a note."""
        self.busy = busy
        self.frame.configure(cursor="watch" if busy else "")
        if busy:
            self.locked = list(controls)
            for widget in self.locked:
                widget.state(["disabled"])
            if label and not self.figure.axes and self.placeholder is None:
                self.placeholder = self.figure.text(0.5, 0.5, label, ha="center", va="center",
                                                    color=MUTED, fontsize=13)
                self.canvas.draw_idle()
            return
        for widget in self.locked:
            widget.state(["!disabled"])
        self.locked = []
        if self.placeholder is not None:
            if self.placeholder.figure is not None:
                self.placeholder.remove()
            self.placeholder = None

    def run(self, label, work, done, controls=()):
        """Start `work` on a worker; a second press while busy is ignored."""
        if self.busy:
            return False
        self.set_busy(True, label, controls)
        self.set_info(label)
        self.app.submit(self, label, work, done)
        return True

    def capture_controls(self, done, parent=None):
        parent = parent if parent is not None else self.controls
        section(parent, "LD06 capture")
        row = ttk.Frame(parent)
        row.pack(fill="x")
        self.port = tk.StringVar(value=part1.SERIAL_PORT)
        self.baud = tk.StringVar(value=str(part1.CAPTURE_BAUD_RATE))
        ttk.Label(row, text="Port").pack(side="left")
        ttk.Entry(row, textvariable=self.port, width=10).pack(side="left", padx=5)
        ttk.Label(row, text="Baud").pack(side="left")
        ttk.Entry(row, textvariable=self.baud, width=9).pack(side="left", padx=5)
        actions = ttk.Frame(parent)
        actions.pack(fill="x", pady=4)
        capture = ttk.Button(actions, text="Capture New Scan", command=lambda: self.capture(done))
        capture.pack(side="left", fill="x", expand=True)
        self.capture_buttons = [capture]
        ttk.Button(actions, text="Cancel", command=self.cancel_capture).pack(side="left", padx=(4, 0))
        note(parent, "Keep the LiDAR still. ESP32 Teleplot XY; 10 s timeout.")

    def capture(self, done):
        if self.busy:
            return
        if self.app.capture_owner is not None:
            self.set_info("Capture is already running on another page. Wait or cancel it there.")
            return
        try:
            baud = int(self.baud.get())
            if baud <= 0 or not self.port.get().strip():
                raise ValueError()
        except ValueError:
            self.set_info("Enter a COM port and a positive integer baud rate.")
            return
        port = self.port.get().strip()
        self.capture_adapter = CaptureAdapter()
        self.app.capture_owner = self
        self.run("Waiting for a fresh scan…", lambda: self.capture_adapter.capture(port, baud), done,
                 controls=self.capture_buttons)

    def cancel_capture(self):
        if self.capture_adapter is not None:
            self.capture_adapter.cancel()

    def tick(self):
        pass

    def close(self):
        self.alive = False
        self.cancel_capture()
        if hasattr(self, "stop"):
            self.stop.set()  # a closed page's global search stops early
        # Let any pending draw finish before removing its Tk widget. No private
        # Matplotlib manager/canvas attributes are accessed.
        self.canvas.flush_events()
        self.frame.destroy()


class PoseInputs:
    def __init__(self, parent, get_pose, set_pose):
        self.get_pose, self.set_pose = get_pose, set_pose
        self.variables, self.entries = [], []
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=5)
        for i, label in enumerate(("X (m)", "Y (m)", "Heading (°)")):
            col = ttk.Frame(row)
            col.pack(side="left", fill="x", expand=True)
            ttk.Label(col, text=label).pack(anchor="w")
            var = tk.StringVar()
            entry = ttk.Entry(col, textvariable=var, width=9)
            entry.pack(fill="x", padx=(0, 4))
            entry.bind("<Return>", lambda _: self.apply())
            self.variables.append(var)
            self.entries.append(entry)
        button(parent, "Set pose", self.apply)
        self.sync()

    def apply(self):
        try:
            pose = tuple(float(v.get()) for v in self.variables)
            if not np.all(np.isfinite(pose)):
                raise ValueError()
            self.set_pose(pose)
        except ValueError:
            messagebox.showerror("Invalid pose", "Enter finite numbers for X, Y and heading.",
                                 parent=self.entries[0].winfo_toplevel())
        self.sync()

    def sync(self):
        if self.entries[0].focus_get() in self.entries:
            return
        for var, value in zip(self.variables, self.get_pose()):
            var.set(f"{value:.3f}")


class ScannerPage(Page):
    def __init__(self, app):
        super().__init__(app)
        _, self.editor = part1.create_session_editor(self.model.session, figure=self.figure)
        self.editor["timer"].stop()  # the host's single poller owns polling
        self.editor["display_selector"].disconnect_events()
        # Hidden filename box: Matplotlib redraws the whole figure synchronously
        # on every resize for it. The host has its own Open/Save buttons.
        self.editor["file_box"].disconnect_events()
        self.prepare_plot(self.editor["axes"])
        section(self.controls, "LiDAR Scanner")
        note(self.controls, "Edits stay in memory across pages. Save a copy to keep them on disk.")
        self.scan_choice = ttk.Combobox(self.controls, state="readonly")
        self.scan_choice.pack(fill="x", pady=5)
        self.scan_choice.bind("<<ComboboxSelected>>", self.select)
        self.pose = PoseInputs(self.controls, self.get_pose, self.set_pose)
        button(self.controls, "Auto Align selected", lambda: self.action("auto_align"))
        row = ttk.Frame(self.controls)
        row.pack(fill="x")
        for label, action in (("Accept proposal", "accept"), ("Reject", "reject")):
            ttk.Button(row, text=label, command=lambda a=action: self.action(a)).pack(side="left", expand=True, fill="x", pady=3)
        section(self.controls, "View / edit")
        self.display = ttk.Combobox(self.controls, state="readonly", values=part1.DISPLAY_MODES)
        self.display.set(part1.DEFAULT_DISPLAY_MODE)
        self.display.pack(fill="x")
        self.display.bind("<<ComboboxSelected>>", lambda _: self.action("display_mode", self.display.get()))
        ttk.Scale(self.controls, from_=0, to=0.5, value=part1.DEFAULT_RAY_OPACITY,
                  command=lambda v: self.action("ray_opacity", float(v))).pack(fill="x")
        note(self.controls, "Ray opacity")
        for label, action in (("Fit view", "fit_view"), ("Hide / show selected", "toggle_visibility"),
                              ("Show all scans", "show_all"), ("Reset selected pose", "reset_pose"),
                              ("Delete selected (click twice)", "delete")):
            button(self.controls, label, lambda a=action: self.action(a))
        self.capture_controls(self.add_capture)
        self.capture_buttons.append(
            button(self.controls, "Capture replacement for selected", self.capture_replacement))
        self.set_info("Ready")
        self.refresh_selection()

    def action(self, name, *args):
        if self.busy and name in ("delete", "reset_pose", "accept", "auto_align"):
            return
        self.editor["actions"][name](*args)
        self.model.refresh()
        self.refresh_selection()
        self.set_info(self.editor["status_text"].get_text())
        self.canvas.draw_idle()

    def refresh_selection(self):
        scans = self.model.session["scans"]
        self.scan_choice["values"] = [f"{s['id']} — {s['label']}" for s in scans]
        selected = self.model.session["selected_id"]
        index = next((i for i, s in enumerate(scans) if s["id"] == selected), None)
        if index is not None:
            self.scan_choice.current(index)
        else:
            self.scan_choice.set("")
        if hasattr(self, "pose"):
            self.pose.sync()
        self.editor["axes"].set_title(
            f"LiDAR scans · selected Scan {selected}" if selected is not None else "LiDAR Scanner · capture or load a session")

    def select(self, _=None):
        index = self.scan_choice.current()
        if index >= 0:
            self.action("select", self.model.session["scans"][index]["id"])

    def get_pose(self):
        scan = part1.selected_scan(self.model.session)
        return scan["pose"] if scan is not None else (0, 0, 0)

    def set_pose(self, pose):
        for slider, value in zip(self.editor["sliders"], pose):
            part1.widen_slider_range(slider, value)
            slider.set_val(value)
        self.model.refresh()

    def add_capture(self, points):
        scan = part1.add_scan(self.model.session, points[:, 0], points[:, 1])
        self.action("select", scan["id"])
        self.action("fit_view")
        self.set_info(f"Captured {len(points)} local points. Added to the working session only.")

    def capture(self, done):
        if self.editor["is_aligning"]():
            self.set_info("Wait for the current alignment before capturing.")
            return
        super().capture(done)

    def capture_replacement(self):
        scan_id = self.model.session["selected_id"]
        if scan_id is None:
            self.set_info("Select a scan to replace first.")
            return

        def replace(points):
            part1.replace_scan_points(self.model.session, scan_id, points[:, 0], points[:, 1])
            self.action("select", scan_id)
            self.action("fit_view")
            self.set_info(f"Replaced Scan {scan_id} in memory; its pose was kept.")

        self.capture(replace)

    def tick(self):
        self.editor["actions"]["poll"]()
        self.pose.sync()
        if self.editor["is_aligning"]() or self.editor["proposal"]() is not None:
            self.set_info(self.editor["status_text"].get_text())

    def close(self):
        self.editor["timer"].stop()
        super().close()


class GridPage(Page):
    def __init__(self, app):
        super().__init__(app)
        section(self.controls, "Grid Mapping")
        note(self.controls, "Code 02 · log-odds occupancy\n0.10 m cells · global Y upward")
        self.view = ttk.Combobox(self.controls, state="readonly", values=("Probability", "Classified", "Binary", "Empty"))
        self.view.set("Probability")
        self.view.pack(fill="x", pady=8)
        self.view.bind("<<ComboboxSelected>>", lambda _: self.draw_map())
        self.points = tk.BooleanVar(value=True)
        ttk.Checkbutton(self.controls, text="Show measured points", variable=self.points,
                        command=self.draw_map).pack(anchor="w")
        note(self.controls, "Occupied: p ≥ 0.65\nFree: p ≤ 0.35\nUnknown cells remain separate.")
        button(self.controls, "Fit view", self.fit)
        self.set_info("Building map…")
        app.derived(self, "grid", build_grid, self.ready)

    def ready(self, data):
        self.grid, self.log_odds = data
        self.draw_map()
        self.set_info(f"{len(self.model.scans)} visible scans\n{self.grid.width} × {self.grid.height} cells\nMap cached for this working session.")

    def draw_map(self):
        if not hasattr(self, "grid"):
            return
        spec = self.grid.spec()
        arrays = part2.global_points_of_scans(self.model.scans)
        kind = self.view.get()
        if kind == "Classified":
            grid = np.full(self.grid.state.shape, part2.UNKNOWN, dtype=np.uint8)
            grid[self.grid.state == loc.FREE_STATE] = part2.FREE
            grid[self.grid.state == loc.OCCUPIED_STATE] = part2.OCCUPIED
        elif kind == "Binary":
            if "binary" not in self.model.cache:
                scans = self.model.scans
                self.run("Building binary grid…", lambda: part2.build_binary_occupancy_grid(
                    spec, scans, arrays)[0], self.binary_ready)
                return
            grid = self.model.cache["binary"]
        elif kind == "Empty":
            grid = part2.create_occupancy_grid(spec)
        else:
            grid = part2.probabilities_to_uint8(self.grid.probability)
        self.figure.clear()
        part2.show_grid(grid, spec, self.model.scans, arrays,
                       title=f"{kind} grid · {self.model.name}", show_points=self.points.get(),
                       colorbar_labels=part2.PROBABILITY_COLORBAR_LABELS if kind == "Probability"
                       else part2.BINARY_COLORBAR_LABELS, figure=self.figure)
        self.figure.subplots_adjust(left=0.12, right=0.94, bottom=0.10, top=0.92)
        self.prepare_plot(self.figure.axes[0], compact=False)
        self.toolbar.update()

    def binary_ready(self, data):
        if self.revision != self.model.revision:
            self.set_info("Working session changed. Reopen this page to rebuild its map.")
            return
        self.model.cache["binary"] = data
        self.draw_map()
        self.set_info("Binary grid ready (cached).")

    def fit(self):
        if not hasattr(self, "grid"):
            return
        left, right, bottom, top = self.grid.extent
        self.figure.axes[0].set(xlim=(left, right), ylim=(bottom, top))
        self.canvas.draw_idle()


class VectorPage(Page):
    def __init__(self, app):
        super().__init__(app)
        section(self.controls, "Vector Mapping")
        note(self.controls, "Code 03 · scan segmentation, PCA, Douglas–Peucker and merged lines")
        self.set_info("Building vector map…")
        app.derived(self, "vectors", build_vectors, self.ready)

    def ready(self, data):
        scans, clusters, records, merged, _ = data
        self.viewer = part3.build_layer_viewer(scans, clusters, records, merged, figure=self.figure)
        self.viewer["checkboxes"].disconnect_events()
        legend = next(ax.get_legend() for ax in self.figure.axes if ax.get_legend() is not None)
        self.viewer["axes"].legend(
            handles=legend.legend_handles, labels=[text.get_text() for text in legend.get_texts()],
            loc="lower right", fontsize=7, framealpha=0.9)
        self.prepare_plot(self.viewer["axes"])
        self.viewer["axes"].set_title(f"Vector map · {len(merged)} merged segments")
        self.info.destroy()
        self.info = None
        self.layers = {}
        for key in part3.LAYER_KEYS:
            variable = tk.BooleanVar(value=key in self.viewer["visible_layers"]())
            self.layers[key] = variable
            ttk.Checkbutton(self.controls, text=self.viewer["labels"][key], variable=variable,
                            command=lambda k=key, v=variable: self.viewer["set_layer"](k, v.get())).pack(anchor="w", pady=6)
        button(self.controls, "Show all", lambda: self.all_layers(True))
        button(self.controls, "Hide all", lambda: self.all_layers(False))
        button(self.controls, "Reset view", self.viewer["reset_view"])
        self.set_info(f"{len(merged)} merged segments\nGeometry uses the existing Code 03 pipeline.")

    def all_layers(self, visible):
        for key, var in self.layers.items():
            var.set(visible)
            self.viewer["set_layer"](key, visible)


# What the user can do about each Code 04 status (the statuses are unchanged).
RESULT_ADVICE = {
    "ok": "Check visually that the blue points lie on the black walls. Day 1 poses are estimates; "
          "this is a matching result, not measured accuracy.",
    "low_score": "Not localised: the scan does not fit the map well enough anywhere searched. Global: "
                 "is the LiDAR inside the mapped room with a clear view (nearby objects can hide the "
                 "walls)? Local: move the guess and point its arrow where the LiDAR front faced.",
    "at_search_limit": "Not confirmed: the best pose is on the local search edge. Press 'Use result as "
                       "next guess' and Localise again, or use the global search.",
    "ambiguous": "Not reliable: a clearly different pose (purple) matches almost as well, e.g. a "
                 "symmetric part of the room. Capture from a spot that sees distinctive features.",
    "out_of_map": "Not localised: most points fall outside the map at the best pose.",
    "cancelled": "Search cancelled.",
    "insufficient_overlap": "Not localised: too few points match the reference. A single reference "
                            "scan may see another part of the room: choose a nearer one or the "
                            "accumulated reference.",
    "inconsistent": "Not localised: even the best pose makes the scans see through each other's walls.",
    "poor_fit": "Not localised: the matched points are too far from the reference (RMS).",
    "failed": "Not localised.",
    "weak": "Plausible but not confirmed: check that the faded points lie on the walls, or capture "
            "again from a spot with more distinctive features. Do not treat it as a position fix.",
    "insufficient_points": "Capture again: too few usable points.",
    "insufficient_coverage": "Capture again with an unobstructed view: the scan covers too narrow an angle.",
}


class LocalisationPage(Page):
    MODES = ("Localise New Scan", "Evaluate Recorded Scans")
    SEARCHES = ("Global: whole map, 360°", "Local: ±1 m / ±30° around a guess")
    SEARCH_NOTES = (
        "No initial pose needed: every plausible position near the mapped room and "
        "every heading is searched on the occupancy grid. Several seconds; ambiguity "
        "is reported.",
        "Faster (under 1 s). Needs a guess within ±1 m and ±30°: left-click the "
        "position, right-click toward the LiDAR front (+X).")

    def __init__(self, app):
        super().__init__(app)
        self.viewer = None
        self.generation = 0
        self.targets = []
        self.stop = threading.Event()
        self.progress = None
        self.shown_progress = None
        section(self.controls, "Grid Localisation")
        self.mode = ttk.Combobox(self.controls, state="readonly", values=self.MODES)
        self.mode.set(self.MODES[0])
        self.mode.pack(fill="x", pady=6)
        self.mode.bind("<<ComboboxSelected>>", self.change_mode)
        self.new_controls = ttk.Frame(self.controls)
        self.new_controls.pack(fill="x")
        self.capture_controls(self.captured, parent=self.new_controls)
        button(self.new_controls, "Load Recorded Scan", self.choose_recorded)
        self.target_choice = ttk.Combobox(self.new_controls, state="readonly")
        self.target_choice.pack(fill="x", pady=4)
        self.target_choice.bind("<<ComboboxSelected>>", self.choose_target)
        self.eval_controls = ttk.Frame(self.controls)
        self.scan_choice = ttk.Combobox(self.eval_controls, state="readonly")
        self.scan_choice["values"] = [f"{s['id']} — {s['name']}" for s in self.model.scans]
        self.scan_choice.pack(fill="x", pady=4)
        self.scan_choice.bind("<<ComboboxSelected>>", self.select_reference_scan)
        self.exclude = tk.BooleanVar(value=True)
        ttk.Checkbutton(self.eval_controls, text="Leave target out of reference map", variable=self.exclude,
                        command=self.toggle_exclude).pack(anchor="w", pady=4)
        button(self.eval_controls, "Start at Day 1 estimate", self.day1_pose)
        button(self.eval_controls, "New perturbed guess", self.new_guess)
        button(self.eval_controls, "Evaluate all / export CSV + JSON", self.evaluate)
        note(self.eval_controls, "Agreement with Day 1 estimates is not physical ground-truth accuracy.")
        section(self.controls, "Search")
        self.search = ttk.Combobox(self.controls, state="readonly", values=self.SEARCHES)
        self.search.set(self.SEARCHES[0])
        self.search.pack(fill="x", pady=4)
        self.search.bind("<<ComboboxSelected>>", self.change_search)
        self.search_note = note(self.controls, self.SEARCH_NOTES[0])
        row = ttk.Frame(self.controls)
        row.pack(fill="x", pady=3)
        self.align_button = ttk.Button(row, text="Localise", command=self.align)
        self.align_button.pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="Cancel", command=self.cancel_search).pack(side="left", padx=(4, 0))
        self.local_controls = ttk.Frame(self.controls)
        section(self.local_controls, "Initial pose (local search)")
        self.pose = PoseInputs(self.local_controls, self.get_pose, self.set_pose)
        note(self.local_controls, "Left click on map: position\nRight click: point heading toward cursor")
        button(self.local_controls, "Use result as next guess", self.use_result)
        self.result_label = note(self.controls, "Load a separate scan or capture one to begin.")
        self.set_info("Building fixed reference map…")
        app.derived(self, "grid", build_grid, self.ready)

    def ready(self, data):
        self.viewer = loc.AutoAlignViewer(self.model.name, self.model.scans, self.model.measured,
                                         figure=self.figure, mode="new", full_grid=data[0])
        self.viewer.exclude_box.disconnect_events()
        self.viewer.scan_box.disconnect_events()  # hidden; avoids a full redraw per resize
        self.prepare_plot(self.viewer.axes)
        self.canvas.mpl_connect("button_press_event", lambda _: self.sync())
        self.verdict = self.viewer.axes.text(
            0.01, 0.99, "", transform=self.viewer.axes.transAxes, va="top", fontsize=10,
            weight="bold", zorder=10, bbox=dict(facecolor="white", alpha=0.85, edgecolor="none"))
        # The best clearly different pose, drawn only when it makes the answer ambiguous.
        self.runner_points = self.viewer.axes.scatter(np.empty(0), np.empty(0), s=5,
                                                      color="tab:purple", alpha=0.6, zorder=3)
        self.runner_marker = loc.pose_marker(self.viewer.axes, "tab:purple", "--")
        self.change_search()
        self.set_info("Reference ready. Load a separate scan or capture a fresh one.")
        self.sync()

    def show_verdict(self):
        """Only an "ok" result looks final; others stay visible but faded."""
        result = self.viewer.result
        reliable = result is None or result.status == "ok"
        self.viewer.aligned_points.set_alpha(1.0 if reliable else 0.3)
        for artist in self.viewer.aligned_marker:
            artist.set_alpha(1.0 if reliable else 0.3)
        runner = result.details.get("runner_up_pose") if result is not None else None
        if result is not None and result.status == "ambiguous" and runner is not None:
            pose = common.pose_from_day1(runner)
            self.runner_points.set_offsets(loc.transform_points(self.viewer.points, pose))
            loc.set_pose_marker(self.runner_marker, pose)
        else:
            self.runner_points.set_offsets(np.empty((0, 2)))
            loc.set_pose_marker(self.runner_marker, None)
        if result is None:
            self.verdict.set_text("")
        elif reliable:
            self.verdict.set_text("Localised (status ok)")
            self.verdict.set_color("#1B7F4B")
        else:
            self.verdict.set_text(f"NOT localised ({result.status}): faded pose is not an answer"
                                  + ("; purple = equally good alternative"
                                     if result.status == "ambiguous" and runner else ""))
            self.verdict.set_color("#B3261E")

    def sync(self):
        if self.viewer is None:
            return
        self.pose.sync()
        self.show_verdict()
        self.result_label.configure(text=self.viewer.info.get_text())
        self.scan_choice.current(self.viewer.index)
        self.canvas.draw_idle()

    def get_pose(self):
        return common.pose_to_day1(self.viewer.initial_pose()) if self.viewer else (0, 0, 0)

    def set_pose(self, pose):
        if self.viewer is not None:
            self.viewer.set_initial_pose(common.pose_from_day1(pose))
            self.generation += 1
            self.sync()

    def change_mode(self, _=None):
        if self.viewer is None:
            return
        self.generation += 1
        evaluation = self.mode.get() == self.MODES[1]
        self.viewer.set_mode("evaluation" if evaluation else "new")
        self.eval_controls.pack_forget()
        self.new_controls.pack_forget()
        (self.eval_controls if evaluation else self.new_controls).pack(fill="x", after=self.mode)
        self.set_info(self.viewer.map_description)
        self.sync()

    def choose_recorded(self):
        if self.viewer is None or self.busy:
            return
        path = filedialog.askopenfilename(parent=self.app.root, title="Load a separate recorded scan",
                                          initialdir=common.SCANS_DIR, filetypes=[("LiDAR session JSON", "*.json")])
        if path:
            self.load_recorded(path)

    def load_recorded(self, path):
        try:
            targets = loc.load_recorded_targets(path)
        except (ValueError, part2.SessionError) as error:
            self.set_info(str(error))
            return False
        self.targets = targets
        self.target_source = Path(path).name
        self.target_choice["values"] = [f"{s['id']} — {s['name']}" for s in targets]
        self.target_choice.current(0)
        return self.choose_target()

    def choose_target(self, _=None):
        if self.viewer is None or not self.targets:
            return False
        target = self.targets[self.target_choice.current()]
        return self.set_target(target["points_local"], f"{self.target_source}: {target['name']}")

    def set_target(self, points, name):
        try:
            self.viewer.set_new_scan(points, name)
        except ValueError as error:
            self.set_info(str(error))
            return False
        self.generation += 1
        self.mode.set(self.MODES[0])
        self.eval_controls.pack_forget()
        self.new_controls.pack(fill="x", after=self.mode)
        self.set_info(f"{name}\n{len(points)} original local points. Source pose ignored.\n"
                      "Next: press Localise. Global search needs no guess; local search "
                      "needs one within ±1 m and ±30°.")
        self.sync()
        return True

    def captured(self, points):
        self.set_target(points, "Fresh LD06 capture")

    def capture(self, done):
        if self.viewer is not None:
            super().capture(done)

    def select_reference_scan(self, _=None):
        if self.viewer is not None:
            self.generation += 1
            self.viewer.on_scan_number(str(self.model.scans[self.scan_choice.current()]["id"]))
            self.set_info(self.viewer.map_description)
            self.sync()

    def toggle_exclude(self):
        if self.viewer is not None:
            self.generation += 1
            self.viewer.exclude = self.exclude.get()
            self.viewer.load_scan(keep_guess=True)
            self.set_info(self.viewer.map_description)
            self.sync()

    def day1_pose(self):
        if self.viewer:
            self.viewer.day1_guess()
            self.sync()

    def new_guess(self):
        if self.viewer:
            self.viewer.new_guess()
            self.sync()

    def use_result(self):
        if self.viewer:
            self.viewer.use_result()
            self.sync()

    def align(self):
        viewer = self.viewer
        if viewer is None or self.busy:
            return
        if viewer.mode == "new" and viewer.external_scan is None:
            self.set_info("Capture or load a new scan first.")
            return
        generation, initial = self.generation, viewer.initial_pose()
        grid, points, config = viewer.grid_map, viewer.points.copy(), viewer.config
        if self.global_search():
            self.stop = stop = threading.Event()
            self.progress = 0.0

            def report(fraction):
                self.progress = fraction  # read by tick() on the Tk thread

            def work():
                return loc.localise_global(grid, points, progress=report, should_stop=stop.is_set)
            label = "Localising: global search…"
        else:
            self.progress = None

            def work():
                return loc.localise_scan(grid, points, initial, config)
            label = "Localising: local search…"

        def done(result):
            self.progress = self.shown_progress = None
            if result.status == "cancelled":
                self.set_info("Search cancelled. Nothing was changed.")
                return
            if generation != self.generation or (
                    result.initial_pose is not None and initial != viewer.initial_pose()):
                self.set_info("Pose or target changed during the search. Press Localise again.")
                return
            viewer.show_result(result)
            self.set_info(f"{result.status}: {result.reason}\n\n{RESULT_ADVICE.get(result.status, '')}")
            self.app.last_runtime = f"{result.runtime_ms:.0f} ms"
            self.sync()

        self.run(label, work, done, controls=(self.align_button,))

    def global_search(self):
        return self.search.current() == 0

    def change_search(self, _=None):
        """Global: no guess, so the guess overlay and pose inputs are hidden."""
        index = 0 if self.global_search() else 1
        self.search_note.configure(text=self.SEARCH_NOTES[index])
        if index:
            self.local_controls.pack(fill="x", after=self.search_note)
        else:
            self.local_controls.pack_forget()
        if self.viewer is not None:
            self.viewer.initial_points.set_visible(bool(index))
            for artist in self.viewer.initial_marker:
                artist.set_visible(bool(index))
            self.canvas.draw_idle()

    def cancel_search(self):
        self.stop.set()

    def tick(self):
        if self.busy and self.progress is not None:
            shown = int(self.progress * 20)  # 5 % steps: no label churn
            if shown != self.shown_progress:
                self.shown_progress = shown
                self.set_info(f"Localising: global search… {shown * 5} %\nCancel stops it; nothing is changed.")

    def evaluate(self):
        if self.busy or self.viewer is None:
            return
        folder = filedialog.askdirectory(parent=self.app.root, title="Choose folder for evaluation CSV and JSON",
                                         initialdir=common.RESULTS_DIR)
        if not folder:
            return
        scans, measured = self.model.scans, self.model.measured
        config = self.viewer.config
        stem = f"grid_evaluation_{datetime.now():%Y%m%d_%H%M%S_%f}"
        name, source, measured_path = self.model.name, self.model.path, self.model.measured_path
        dirty = self.model.dirty

        def work():
            rows = loc.session_rows(name, scans, measured, config, loc.BENCHMARK_REPEATS,
                                    loc.REAL_INIT_CONDITIONS, None)
            by_condition, by_scan = loc.summarise_conditions(rows), loc.summarise_scans(rows, config)
            return loc.save_results(rows, by_condition, by_scan, config, folder, stem,
                                    source, measured_path, loc.EVALUATION_NOTES +
                                    (f"Desktop working session; unsaved edits: {dirty}.",))

        self.run("Evaluating all scans and initialisations (several minutes). Navigation remains available.", work,
                 lambda paths: self.set_info(f"Evaluation saved:\n{paths[0]}\n{paths[1]}"),
                 controls=(self.align_button,))


class SharedBuild:
    """One derived-map computation; every page waiting for it gets the result."""
    def __init__(self, release):
        self.waiters = []  # (page, done)
        self.release = release
        self.busy = True

    @property
    def alive(self):
        return any(page.alive for page, _ in self.waiters)

    def set_busy(self, busy):
        self.busy = busy
        if not busy:
            self.release()
        for page, _ in self.waiters:
            if page.alive:
                page.set_busy(busy)

    def set_info(self, text):
        for page, _ in self.waiters:
            if page.alive:
                page.set_info(text)


class NewScanLocalisationPage(Page):
    """Shared page for localising ONE new scan on a fixed reference (Codes 05, 06).

    Subclasses give the reference: reference_controls(), draw_reference(),
    compute() and measure_lines(). The new scan keeps its sensor-frame
    points; it is never added to the reference or the session.
    """
    TITLE = DESCRIPTION = REFERENCE_NAME = ""
    SEARCHES = LocalisationPage.SEARCHES
    SEARCH_NOTES = ("", "")

    def __init__(self, app):
        super().__init__(app)
        self.reference_ready = False
        self.target = None          # (local points, name): never added to the map
        self.targets = []
        self.result = None
        self.guess = (0.0, 0.0, 0.0)
        self.generation = 0
        self.stop = threading.Event()
        self.progress = self.shown_progress = None
        section(self.controls, self.TITLE)
        note(self.controls, self.DESCRIPTION)
        note(self.controls, "Coordinate reference: Scan 1 (0, 0, 0°)", style="Section.TLabel")
        self.reference_controls(self.controls)
        self.capture_controls(self.captured)
        button(self.controls, "Load Recorded Scan", self.choose_recorded)
        self.target_choice = ttk.Combobox(self.controls, state="readonly")
        self.target_choice.pack(fill="x", pady=4)
        self.target_choice.bind("<<ComboboxSelected>>", self.choose_target)
        section(self.controls, "Search")
        self.search = ttk.Combobox(self.controls, state="readonly", values=self.SEARCHES)
        self.search.set(self.SEARCHES[0])
        self.search.pack(fill="x", pady=4)
        self.search.bind("<<ComboboxSelected>>", self.change_search)
        self.search_note = note(self.controls, self.SEARCH_NOTES[0])
        self.action_row = ttk.Frame(self.controls)
        self.action_row.pack(fill="x", pady=3)
        self.localise_button = ttk.Button(self.action_row, text="Localise", command=self.localise)
        self.localise_button.pack(side="left", fill="x", expand=True)
        ttk.Button(self.action_row, text="Cancel", command=self.cancel_search).pack(side="left", padx=(4, 0))
        self.local_controls = ttk.Frame(self.controls)
        section(self.local_controls, "Initial pose (local search)")
        self.pose = PoseInputs(self.local_controls, lambda: vloc.pose_to_day1(self.guess),
                               self.set_pose)
        note(self.local_controls, "Left click on map: position\nRight click: point heading toward cursor")
        section(self.controls, "Result")
        self.result_label = note(self.controls, "Load a separate scan or capture one to begin.")

    # -- hooks ------------------------------------------------------------------
    def reference_controls(self, parent):
        pass

    def draw_reference(self, ax):
        """Draw the reference; return its legend handles and (x, y) limits."""
        raise NotImplementedError

    def compute(self, points, initial, report, should_stop):
        raise NotImplementedError

    def measure_lines(self, result):
        return []

    # -- plot ---------------------------------------------------------------------
    def setup_plot(self):
        """Create the map once the reference is available."""
        ax = self.axes = self.figure.add_subplot(111)
        handles, (xlim, ylim) = self.draw_reference(ax)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.plot([0], [0], "+", color="tab:green", markersize=14, mew=2, zorder=6)
        ax.annotate("Scan 1 (0, 0, 0°)", (0, 0), (6, -14), textcoords="offset points",
                    fontsize=8, color="tab:green")
        self.initial_points = ax.scatter(np.empty(0), np.empty(0), s=5, color="tab:orange", zorder=3)
        self.aligned_points = ax.scatter(np.empty(0), np.empty(0), s=6, color="tab:blue", zorder=4)
        self.runner_points = ax.scatter(np.empty(0), np.empty(0), s=5, color="tab:purple",
                                        alpha=0.6, zorder=3)
        self.initial_marker = loc.pose_marker(ax, "tab:orange")
        self.aligned_marker = loc.pose_marker(ax, "tab:blue")
        self.runner_marker = loc.pose_marker(ax, "tab:purple", "--")
        self.verdict = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", fontsize=10,
                               weight="bold", zorder=10,
                               bbox=dict(facecolor="white", alpha=0.85, edgecolor="none"))
        ax.legend(handles=handles + [
            Line2D([], [], marker="o", ls="", color="tab:orange", label="new scan at initial guess (local)"),
            Line2D([], [], marker="o", ls="", color="tab:blue", label="localised new scan + pose"),
            Line2D([], [], marker="o", ls="", color="tab:purple", label="alternative pose (ambiguous)")],
            loc="upper right", fontsize=8, framealpha=0.9)
        self.canvas.mpl_connect("button_press_event", self.on_click)
        self.prepare_plot(ax, compact=False)
        self.figure.subplots_adjust(left=0.10, right=0.97, bottom=0.08, top=0.95)
        self.reference_ready = True
        self.change_search()

    # -- target scan ------------------------------------------------------------
    def choose_recorded(self):
        if not self.reference_ready or self.busy:
            return
        path = filedialog.askopenfilename(parent=self.app.root, title="Load a separate recorded scan",
                                          initialdir=common.SCANS_DIR, filetypes=[("LiDAR session JSON", "*.json")])
        if path:
            self.load_recorded(path)

    def load_recorded(self, path):
        try:
            targets = loc.load_recorded_targets(path)
        except (ValueError, part2.SessionError) as error:
            self.set_info(str(error))
            return False
        self.targets, self.target_source = targets, Path(path).name
        self.target_choice["values"] = [f"{t['id']} — {t['name']}" for t in targets]
        self.target_choice.current(0)
        return self.choose_target()

    def choose_target(self, _=None):
        if not self.reference_ready or not self.targets:
            return False
        target = self.targets[self.target_choice.current()]
        return self.set_target(target["points_local"], f"{self.target_source}: {target['name']}")

    def set_target(self, points, name):
        points = np.array(points, dtype=float, copy=True)
        if points.ndim != 2 or points.shape[1] != 2 or not len(points) or not np.all(np.isfinite(points)):
            self.set_info("The new scan must contain finite local [x, y] points in metres.")
            return False
        signature = loc.measurement_signature(points)
        duplicates = [s["id"] for s in self.model.scans
                      if loc.measurement_signature(s["points_local"]) == signature]
        if duplicates:  # its own points are in the reference: self-matching
            self.set_info(f"These measurements already belong to reference scan(s) {duplicates}; "
                          "localising them here would be self-matching. Use the leave-one-out "
                          "evaluation (RUN_MODE = \"evaluation\" in Codes 05 and 06).")
            return False
        points.setflags(write=False)
        self.target, self.result = (points, name), None
        self.generation += 1
        self.set_info(f"{name}\n{len(points)} original local points. Source pose ignored.\n"
                      "Next: press Localise.")
        self.redraw()
        return True

    def captured(self, points):
        self.set_target(points, "Fresh LD06 capture")

    def capture(self, done):
        if self.reference_ready:
            super().capture(done)

    # -- search ------------------------------------------------------------------
    def global_search(self):
        return self.search.current() == 0

    def change_search(self, _=None):
        index = 0 if self.global_search() else 1
        self.search_note.configure(text=self.SEARCH_NOTES[index])
        if index:
            self.local_controls.pack(fill="x", after=self.action_row)
        else:
            self.local_controls.pack_forget()
        self.redraw()

    def set_pose(self, pose):
        self.guess = vloc.pose_from_day1(pose)
        if self.result is not None and self.result.initial_pose is not None:
            self.result = None  # that answer was for another guess
        self.generation += 1
        self.redraw()

    def on_click(self, event):
        toolbar = getattr(self.canvas, "toolbar", None)
        if not self.reference_ready or self.global_search() or event.inaxes is not self.axes \
                or event.xdata is None or (toolbar is not None and toolbar.mode):
            return
        x, y, theta = self.guess
        if event.button == 1:
            x, y = event.xdata, event.ydata
        elif event.button == 3:
            theta = np.arctan2(event.ydata - y, event.xdata - x)
        self.set_pose(vloc.pose_to_day1((x, y, theta)))

    def localise(self):
        if not self.reference_ready or self.busy:
            return
        if self.target is None:
            self.set_info("Capture or load a new scan first.")
            return
        points, generation = self.target[0], self.generation
        initial = None if self.global_search() else self.guess
        self.stop = stop = threading.Event()
        self.progress = 0.0

        def report(fraction):
            self.progress = fraction  # read by tick() on the Tk thread

        def done(result):
            self.progress = self.shown_progress = None
            if result.status == "cancelled":
                self.set_info("Search cancelled. Nothing was changed.")
                return
            if generation != self.generation:
                self.set_info("Target, guess or reference changed during the search. "
                              "Press Localise again.")
                return
            self.result = result
            self.app.last_runtime = f"{result.runtime_ms:.0f} ms ({self.REFERENCE_NAME})"
            self.set_info(f"{result.status}: {result.reason}\n\n{RESULT_ADVICE.get(result.status, '')}")
            self.redraw()

        work = self.compute(points, initial, report, stop.is_set)
        self.run(f"Localising on the {self.REFERENCE_NAME}…", work, done,
                 controls=(self.localise_button,))

    def cancel_search(self):
        self.stop.set()

    def tick(self):
        if self.busy and self.progress is not None:
            shown = int(self.progress * 20)
            if shown != self.shown_progress:
                self.shown_progress = shown
                self.set_info(f"Localising on the {self.REFERENCE_NAME}… {shown * 5} %\n"
                              "Cancel stops it; nothing is changed.")

    def close(self):
        self.stop.set()
        super().close()

    # -- drawing ----------------------------------------------------------------
    def redraw(self):
        if not self.reference_ready:
            return
        empty = np.empty((0, 2))
        points = vloc.filter_ranges(self.target[0]) if self.target is not None else empty
        local = not self.global_search()
        self.initial_points.set_offsets(vloc.transform_points(points, self.guess)
                                        if local and len(points) else empty)
        loc.set_pose_marker(self.initial_marker, self.guess if local else None)
        result = self.result
        reliable = result is not None and result.status == "ok"
        alpha = 1.0 if reliable else 0.3
        if result is not None and result.pose is not None:
            self.aligned_points.set_offsets(vloc.transform_points(points, result.pose))
            loc.set_pose_marker(self.aligned_marker, result.pose)
        else:
            self.aligned_points.set_offsets(empty)
            loc.set_pose_marker(self.aligned_marker, None)
        self.aligned_points.set_alpha(alpha)
        for artist in self.aligned_marker:
            artist.set_alpha(alpha)
        runner = result.details.get("runner_up_pose") if result is not None else None
        if result is not None and result.status == "ambiguous" and runner is not None:
            pose = vloc.pose_from_day1(runner)
            self.runner_points.set_offsets(vloc.transform_points(points, pose))
            loc.set_pose_marker(self.runner_marker, pose)
        else:
            self.runner_points.set_offsets(empty)
            loc.set_pose_marker(self.runner_marker, None)
        if result is None:
            self.verdict.set_text("")
        elif reliable:
            self.verdict.set_text("Localised (status ok)")
            self.verdict.set_color("#1B7F4B")
        else:
            self.verdict.set_text(f"NOT localised ({result.status}): faded pose is not an answer"
                                  + ("; purple = equally good alternative"
                                     if result.status == "ambiguous" and runner else ""))
            self.verdict.set_color("#B3261E")
        self.axes.set_title(self.target[1] if self.target is not None else
                            f"{self.REFERENCE_NAME.capitalize()} · capture or load a scan",
                            fontsize=10, color=TEXT)
        self.result_label.configure(text=self.result_text())
        self.pose.sync()
        self.canvas.draw_idle()

    def result_text(self):
        result = self.result
        if result is None:
            return ("Press Localise." if self.target is not None
                    else "Load a separate scan or capture one to begin.")
        lines = [f"status: {result.status}",
                 "search: " + ("global (no initial pose)" if result.initial_pose is None
                               else "local (±1 m, ±30°)")]
        if result.pose is not None:
            x, y, heading = vloc.pose_to_day1(result.pose)
            lines += [f"X {x:.3f} m   Y {y:.3f} m", f"heading {heading:.2f}°",
                      f"score {result.score:.3f}"
                      + (f" (best different pose {result.runner_up_score:.3f})"
                         if result.runner_up_score is not None else "")]
            lines += self.measure_lines(result)
        lines += [f"time {result.runtime_ms:.0f} ms (algorithm only)",
                  f"Reference {self.REFERENCE_NAME} stays fixed. No measured pose:",
                  "this is a matching result, not physical accuracy."]
        return "\n".join(lines)


class VectorLocalisationPage(NewScanLocalisationPage):
    """Code 05: a new scan on the Code 03 vector map (finite segments)."""
    TITLE = "Vector Localisation"
    DESCRIPTION = ("Code 05 · scan points matched to the finite segments of the "
                   "Code 03 vector map (point-to-segment distance).")
    REFERENCE_NAME = "vector map"
    SEARCH_NOTES = (
        "No initial pose needed: every plausible position in the mapped room and every "
        "heading is searched against the vector segments. A few seconds; ambiguity is "
        "reported.",
        "Faster. Needs a guess within ±1 m and ±30°: left-click the position, right-click "
        "toward the LiDAR front (+X).")

    def __init__(self, app):
        super().__init__(app)
        self.vmap = None
        self.set_info("Extracting vector map…")
        app.derived(self, "vectors", build_vectors, self.vectors_ready)

    def vectors_ready(self, data):
        merged = data[3]
        if not merged:
            self.set_info("The vector map has no segments: nothing to localise against.")
            return
        cached = self.model.cache.get("vector_reference")
        if cached is not None:
            self.ready(cached)
            return
        revision = self.model.revision

        def keep(vmap):
            if revision == self.model.revision:
                self.model.cache["vector_reference"] = vmap
            self.ready(vmap)
        self.run("Preparing vector reference…", lambda: vloc.VectorMap.from_merged(merged), keep)

    def ready(self, vmap):
        self.vmap = vmap
        self.setup_plot()
        self.set_info(f"Vector map ready: {len(vmap.starts)} segments. Load a separate scan "
                      "or capture a fresh one.")

    def draw_reference(self, ax):
        vmap = self.vmap
        ax.add_collection(LineCollection(np.stack([vmap.starts, vmap.ends], axis=1),
                                         colors="#16232F", linewidths=2.2, zorder=2))
        left, right, bottom, top = vmap.segment_extent
        return ([Line2D([], [], color="#16232F", lw=2.2,
                        label=f"vector map ({len(vmap.starts)} segments)")],
                ((left - 0.5, right + 0.5), (bottom - 0.5, top + 0.5)))

    def compute(self, points, initial, report, should_stop):
        vmap = self.vmap
        return lambda: vloc.localise_vector(vmap, points, initial, progress=report,
                                            should_stop=should_stop)

    def measure_lines(self, result):
        d = result.details
        return [f"{d['inliers']:.0%} of points within 8 cm of a segment",
                f"RMS {d['rms_m'] * 100:.1f} cm · {d['segments']} supporting segments",
                f"constraint {d['constraint']:.2f} (walls in two directions)"]


class ScanLocalisationPage(NewScanLocalisationPage):
    """Code 06: a new scan matched to recorded LiDAR points (Code 01 + ICP)."""
    TITLE = "Scan Localisation"
    DESCRIPTION = ("Code 06 · new scan matched to previously recorded LiDAR POINTS with "
                   "Code 01's correlative search + ICP. No grid, no vector map.")
    REFERENCE_NAME = "reference scans"
    REFERENCE_MODES = ("Single scan (strict scan-to-scan)",
                       "Accumulated scans (point cloud of all scans)")
    SEARCH_NOTES = (
        "No initial pose needed: every position in the reference's area and every heading "
        "(2°) is searched, 10 distinct candidates refined by ICP. Single scan: a few "
        "seconds; accumulated: up to ~10 s.",
        "Faster. Searches ±1 m and ±30° around the guess (20 cm, 2° steps), then ICP: "
        "left-click the position, right-click toward the LiDAR front (+X).")

    def reference_controls(self, parent):
        section(parent, "Reference")
        self.reference_mode = ttk.Combobox(parent, state="readonly", values=self.REFERENCE_MODES)
        self.reference_mode.set(self.REFERENCE_MODES[0])
        self.reference_mode.pack(fill="x", pady=4)
        self.reference_mode.bind("<<ComboboxSelected>>", self.change_reference)
        self.reference_choice = ttk.Combobox(parent, state="readonly")
        self.reference_choice.pack(fill="x", pady=4)
        self.reference_choice.bind("<<ComboboxSelected>>", self.change_reference)
        self.reference_note = note(parent, "")

    def __init__(self, app):
        super().__init__(app)
        scans = self.model.scans
        self.reference_choice["values"] = [f"Scan {s['id']} — pose ({s['pose'][0]:.2f}, "
                                           f"{s['pose'][1]:.2f}, {s['pose'][2]:.0f}°)" for s in scans]
        if not scans:
            self.set_info("No visible positioned scans: open a session or show scans in "
                          "LiDAR Scanner.")
            return
        self.reference_choice.current(0)
        self.setup_plot()
        self.change_reference()
        self.set_info(f"Reference ready: {len(scans)} positioned scans. Load a separate scan or "
                      "capture a fresh one.")

    def single(self):
        return self.reference_mode.current() == 0

    def references(self):
        """Code 06 reference for the current choice (copied, never modified)."""
        scans = self.model.scans
        if self.single():
            return sloc.reference_scans(scans, "single",
                                        scans[self.reference_choice.current()]["id"])
        return sloc.reference_scans(scans, "accumulated")

    def change_reference(self, _=None):
        if self.single():
            self.reference_choice.pack(fill="x", pady=4, before=self.reference_note)
            self.reference_note.configure(
                text="Strict scan-to-scan: the new scan is matched to ONE recorded scan, "
                     "placed in the Scan 1 frame by its saved pose. Little overlap = failure.")
        else:
            self.reference_choice.pack_forget()
            self.reference_note.configure(
                text=f"Scan-to-accumulated-point-cloud: all {len(self.model.scans)} positioned "
                     "scans pooled (as Code 01's Auto Align reference).")
        if self.reference_ready:
            self.generation += 1
            self.result = None
            self.draw_reference_points()
            self.redraw()

    def draw_reference(self, ax):
        self.reference_points = ax.scatter(np.empty(0), np.empty(0), s=2, color="0.45", zorder=2)
        self.reference_poses = ax.scatter(np.empty(0), np.empty(0), s=36, marker="s",
                                          color="0.25", zorder=5)
        everything = np.vstack([vloc.transform_points(s["points_local"],
                                                      vloc.pose_from_day1(s["pose"]))
                                for s in self.model.scans])
        everything = everything[np.all(np.isfinite(everything), axis=1)]
        low, high = everything.min(axis=0) - 0.5, everything.max(axis=0) + 0.5
        return ([Line2D([], [], marker="o", ls="", color="0.45", label="reference LiDAR points"),
                 Line2D([], [], marker="s", ls="", color="0.25", label="reference scan pose(s)")],
                ((low[0], high[0]), (low[1], high[1])))

    def draw_reference_points(self):
        references = self.references()
        points = np.vstack([part1.transform_points(local, pose) for local, pose in references])
        self.reference_points.set_offsets(points)
        self.reference_poses.set_offsets(np.array([pose[:2] for _, pose in references]))

    def compute(self, points, initial, report, should_stop):
        references = self.references()
        mode = "single" if self.single() else "accumulated"
        return lambda: sloc.localise_on_scans(points, references, initial, progress=report,
                                              should_stop=should_stop, mode=mode)

    def measure_lines(self, result):
        d = result.details
        reference = ("single scan" if d["mode"] == "single"
                     else f"{d['reference_scans']} scans accumulated")
        return [f"matched {d['match_fraction']:.0%} (within 8 cm) · RMS {d['rms_m'] * 100:.1f} cm",
                f"see-through {d['conflict']:.0%} (scan past reference "
                f"{d['scan_sees_through']:.0%}, reference through scan "
                f"{d['reference_sees_through']:.0%})",
                f"reference: {reference} · Code 01 verdict: {d['code01_status']}"]


BUILD_LABELS = {"grid": "Building occupancy grid…", "vectors": "Extracting vector map…"}

# Add future pages here when their algorithms are implemented.
PAGES = {"LiDAR Scanner": ScannerPage, "Grid Mapping": GridPage,
         "Vector Mapping": VectorPage, "Grid Localisation": LocalisationPage,
         "Vector Localisation": VectorLocalisationPage,
         "Scan Localisation": ScanLocalisationPage}


class DesktopApp:
    def __init__(self, root, session_path=None, initial_page="LiDAR Scanner"):
        self.root = root
        root.title("LiDAR SLAM · Engineering lab")
        root.geometry("1440x900")
        root.minsize(1080, 700)
        root.configure(bg=BG)
        self.closed = False
        self.results = queue.Queue()
        self.jobs = {}
        self.building = {}  # (model, revision, key) -> SharedBuild in progress
        self.pages = {}
        self.current = None
        self.capture_owner = None
        self.last_runtime = "—"
        self.model = SharedSession(session_path)
        self.style()
        top = ttk.Frame(root, style="Background.TFrame", padding=(18, 10))
        top.pack(fill="x")
        ttk.Label(top, text="LiDAR / SLAM TP", style="Title.TLabel").pack(side="left")
        self.session_label = ttk.Label(top, style="Background.TLabel")
        self.session_label.pack(side="left", padx=28)
        ttk.Button(top, text="Save working copy…", command=self.save_copy).pack(side="right", padx=4)
        ttk.Button(top, text="Open session…", command=self.choose_session).pack(side="right", padx=4)
        self.status = ttk.Label(root, style="Status.TLabel", padding=(16, 8))
        self.status.pack(side="bottom", fill="x")
        body = ttk.Frame(root, style="Background.TFrame")
        body.pack(fill="both", expand=True)
        nav = ttk.Frame(body, width=184, padding=(10, 16))
        nav.pack(side="left", fill="y", padx=(0, 4))
        nav.pack_propagate(False)
        self.nav_buttons = {}
        for name in PAGES:
            self.nav_buttons[name] = button(nav, name, lambda n=name: self.show_page(n))
        note(nav, "Day 1 · Mapping\nDay 2 · Grid localisation").configure(wraplength=160)
        note(nav, "Session edits stay in memory until you save a copy.").configure(wraplength=160)
        self.workspace = ttk.Frame(body, style="Background.TFrame")
        self.workspace.pack(side="left", fill="both", expand=True)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.update_idletasks()  # real workspace size before the first page draws
        self.show_page(initial_page)
        self.after_id = root.after(80, self.poll)

    def style(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(".", background=PANEL, foreground=TEXT, font=("Segoe UI", 10))
        style.configure("TButton", padding=(8, 7), background="#244054", borderwidth=0)
        style.map("TButton", background=[("active", "#31556B")])
        style.configure("Selected.TButton", background=ACCENT, foreground=BG)
        style.configure("TEntry", fieldbackground=BG, foreground=TEXT, insertcolor=TEXT)
        style.configure("TCombobox", fieldbackground=BG, background=PANEL, foreground=TEXT)
        style.map("TCombobox", fieldbackground=[("readonly", BG)], foreground=[("readonly", TEXT)])
        style.configure("TCheckbutton", background=PANEL)
        style.map("TCheckbutton", background=[("active", PANEL)])
        style.configure("Background.TFrame", background=BG)
        style.configure("Background.TLabel", background=BG, foreground=MUTED)
        style.configure("Title.TLabel", background=BG, foreground=TEXT, font=("Segoe UI", 16, "bold"))
        style.configure("Section.TLabel", foreground="#7CD8D0", font=("Segoe UI", 11, "bold"))
        style.configure("Status.TLabel", background=PANEL, foreground=MUTED, font=("Segoe UI", 9))
        self.root.option_add("*TCombobox*Listbox.background", PANEL)
        self.root.option_add("*TCombobox*Listbox.foreground", TEXT)

    def show_page(self, name):
        self.model.refresh()
        if self.current:
            self.pages[self.current].frame.pack_forget()
        page = self.pages.get(name)
        if page and name != "LiDAR Scanner" and page.revision != self.model.revision:
            page.close()
            del self.pages[name]
            page = None
        if page is None:
            page = PAGES[name](self)
            self.pages[name] = page
        page.frame.pack(fill="both", expand=True)
        self.current = name
        for label, widget in self.nav_buttons.items():
            widget.configure(style="Selected.TButton" if label == name else "TButton")
        page.canvas.draw_idle()

    def derived(self, page, key, build, done):
        """Cached map of the current revision; one build shared by all pages."""
        model, revision, scans = self.model, self.model.revision, self.model.scans
        if key in model.cache:
            done(model.cache[key])
            return
        label = BUILD_LABELS.get(key, "Building map…")
        page.set_busy(True, label)
        page.set_info(label)
        slot = (model, revision, key)
        job = self.building.get(slot)
        if job is not None:  # already running (e.g. the grid for two pages)
            job.waiters.append((page, done))
            return
        job = self.building[slot] = SharedBuild(lambda: self.building.pop(slot, None))
        job.waiters.append((page, done))

        def ready(data):
            if model is not self.model or revision != model.revision:
                job.set_info("Working session changed. Reopen this page to rebuild its map.")
                return
            model.cache[key] = data
            for waiter, callback in job.waiters:
                if not waiter.alive:
                    continue
                try:
                    callback(data)
                except Exception as exc:  # one page's failure must not block the others
                    waiter.set_info(f"Operation failed: {exc}")
                    print(f"Operation failed: {exc}")

        self.submit(job, label, lambda: build(scans), ready)

    def submit(self, owner, label, work, done):
        token = object()
        self.jobs[token] = (owner, label, time.monotonic())

        def worker():
            try:
                result, error = work(), None
            except Exception as exc:
                result, error = None, exc
            self.results.put((token, owner, done, result, error))

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()

    def poll(self):
        if self.closed:
            return
        while True:
            try:
                token, owner, done, result, error = self.results.get_nowait()
            except queue.Empty:
                break
            self.jobs.pop(token, None)
            if owner.alive:
                owner.set_busy(False)
            else:
                owner.busy = False
            if self.capture_owner is owner:
                self.capture_owner = None
            if owner.alive:
                try:
                    if error:
                        raise error
                    done(result)
                except Exception as exc:
                    owner.set_info(f"Operation failed: {exc}")
                    print(f"Operation failed: {exc}")
        for page in list(self.pages.values()):
            page.tick()
        dirty = self.model.dirty
        self.session_label.configure(text=f"{self.model.name}{'  ·  unsaved edits' if dirty else ''}")
        connection = self.capture_owner.capture_adapter.state if self.capture_owner else "Port closed"
        operation = next((f"{label} {time.monotonic() - start:.1f} s"
                          for owner, label, start in self.jobs.values() if owner.alive), "Ready")
        if self.current == "LiDAR Scanner" and self.pages[self.current].editor["is_aligning"]():
            operation = "Aligning selected scan (Code 01)"
        self.status.configure(text=f"{self.model.name}   |   {len(self.model.session['scans'])} scans · {len(self.model.scans)} visible   |   "
                                   f"{connection}   |   {operation}   |   Last localisation: {self.last_runtime}")
        self.after_id = self.root.after(80, self.poll)

    def save_copy(self):
        path = filedialog.asksaveasfilename(parent=self.root, title="Save working session as a copy",
                                           initialdir=common.SCANS_DIR,
                                           initialfile=f"{Path(self.model.name).stem}_edited.json",
                                           defaultextension=".json", filetypes=[("LiDAR session JSON", "*.json")])
        if not path:
            return False
        try:
            self.model.save_copy(path)
            for page in self.pages.values():
                if isinstance(page, LocalisationPage) and page.viewer is not None:
                    page.viewer.session_name = self.model.name
                if isinstance(page, GridPage) and hasattr(page, "grid") \
                        and page.revision == self.model.revision:
                    page.draw_map()
            self.pages[self.current].set_info(f"Working copy saved: {path}")
            return True
        except (OSError, ValueError) as error:
            messagebox.showerror("Cannot save copy", str(error), parent=self.root)
            return False

    def may_discard(self):
        self.model.refresh()
        if not self.model.dirty:
            return True
        answer = messagebox.askyesnocancel("Unsaved session edits",
                                           "Save a copy before leaving this session?\nYes: save copy. No: discard edits. Cancel: keep working.",
                                           parent=self.root)
        return self.save_copy() if answer else answer is False

    def choose_session(self):
        path = filedialog.askopenfilename(parent=self.root, title="Open shared LiDAR session",
                                          initialdir=common.SCANS_DIR, filetypes=[("LiDAR session JSON", "*.json")])
        if path:
            self.load_session(path)

    def load_session(self, path):
        try:
            model = SharedSession(path)  # validate before replacing any state
        except (part1.SessionFileError, common.MeasuredPoseError) as error:
            messagebox.showerror("Cannot open session", str(error), parent=self.root)
            return False
        if not self.may_discard():
            return False
        previous = self.current
        for page in self.pages.values():
            page.close()
        self.pages.clear()
        self.model, self.current = model, None
        self.last_runtime = "—"
        self.show_page(previous)
        return True

    def close(self):
        if self.closed or not self.may_discard():
            return
        # Exports are explicitly requested writes; don't abandon them silently.
        if any("Evaluating" in label for _, label, _ in self.jobs.values()):
            messagebox.showinfo("Evaluation running", "Wait for the evaluation export to finish before closing.", parent=self.root)
            return
        self.closed = True
        self.root.after_cancel(self.after_id)
        for page in self.pages.values():
            page.cancel_capture()
        self.finish_close()

    def finish_close(self):
        # The serial reader checks cancellation every 0.1 s; give its finally
        # block time to release the port before ending Python's event loop.
        if self.capture_owner and self.capture_owner.capture_adapter.active:
            self.root.after(50, self.finish_close)
            return
        for page in self.pages.values():
            page.close()
        self.root.destroy()


def launch(session_path=None, initial_page="LiDAR Scanner"):
    root = tk.Tk()
    try:
        app = DesktopApp(root, session_path, initial_page)
    except Exception:
        root.destroy()
        raise
    root.mainloop()
    return app


def main(argv=None):
    parser = argparse.ArgumentParser(description="Unified LiDAR mapping and grid localisation")
    parser.add_argument("--session", type=Path, help="Open this session (otherwise open one in the app)")
    parser.add_argument("--page", choices=PAGES, default="LiDAR Scanner")
    args = parser.parse_args(argv)
    launch(args.session, args.page)
