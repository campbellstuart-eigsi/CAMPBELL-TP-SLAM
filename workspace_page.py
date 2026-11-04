"""Four cached map views in the existing Tk desktop; no mapping algorithms."""
from pathlib import Path
import threading
import time
import tkinter as tk
from tkinter import filedialog, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.collections import LineCollection
from matplotlib import colormaps
from matplotlib.colors import LinearSegmentedColormap, to_hex
from matplotlib.figure import Figure

import localisation_common as common
from app_session import build_grid, build_vectors, part1, part2, grid_localisation as loc, \
    vector_localisation as vloc
from desktop_app import (Page, GridPage, PoseInputs, VectorLocalisationPage, ScanLocalisationPage,
                         CHROME, CHROME_2, SELECT, VIEW_BG, VIEW_CAPTION, VIEW_TEXT, VIEW_MUTED,
                         RESIZE_SETTLE_MS, RESULT_ADVICE, VERDICT_STYLES, button, expander,
                         field_label, heading, note, row, tip, set_text, show_verdict_chip)
from ui_fonts import FontSet, TechnicalLabel
from ui_theme import PLOT, THEME
import robot_drive
import workspace_localisation as wl
from workspace_inputs import ScanFrame


GLOBAL_VIEWS = ("scan", "grid", "vector")
VIEW_TITLES = {"scan": "Scan Map", "grid": "Bitmap / Occupancy Grid",
               "vector": "Vector Map", "current": "Current Scan / Live Robot"}
VIEW_HELP = {
    "scan": "Code 01 scans accumulated at their recorded alignment poses. Global frame "
            "(Scan 1), metres.",
    "grid": "Code 02 occupancy grid, global frame, metres. Display mode in Properties.",
    "vector": "Code 03 extracted and merged finite line segments. Global frame, metres.",
    "current": "Selected recorded scan, latest capture or localisation target in the LiDAR's "
               "own frame: origin (0, 0), heading along +X, metres. The global estimate is "
               "never applied here; no live tracking.",
}
TARGETS = ("Latest capture", "Scan from a separate recording")
MEASURE_LINES = {"vector": VectorLocalisationPage.measure_lines,
                 "scans": ScanLocalisationPage.measure_lines}  # read only result.details
NAV_HELP = "\n\nClick a viewport to make the tool bar act on it; double-click a title to maximise."

# Plot styles: only presentation changes. Cell VALUES are never altered; the
# dark map keeps three distinct classes ordered by salience: unknown recedes
# into the background, observed free floor is a visible graphite, occupied
# walls are the brightest element (light paper: free white, occupied black).
DARK_GRID = LinearSegmentedColormap.from_list("occupancy_dark", [
    (0.0, PLOT.dark_free),                     # free (0) — observed floor
    (part2.UNKNOWN / 255, PLOT.dark_unknown),  # unknown (127) — background
    (1.0, PLOT.dark_occupied),                 # occupied (255) — walls
])
PLOT_STYLES = {
    "Dark": dict(face=PLOT.dark_face, tick=THEME.secondary, label=THEME.text, spine=PLOT.dark_spine,
                 grid=(PLOT.dark_grid, 0.5, 1.0), cmap=DARK_GRID, vector=PLOT.dark_vector,
                 current=PLOT.dark_current, empty=THEME.secondary,
                 recorded=PLOT.recorded_pose, axis=PLOT.axis_dark, initial=PLOT.initial_guess,
                 estimate=PLOT.estimate_dark, alternative=PLOT.alternative_dark),
    "Light paper": dict(face=THEME.paper, tick=VIEW_MUTED, label=VIEW_TEXT, spine=THEME.plot_spine,
                        grid=(PLOT.paper_grid, 0.4, 0.35), cmap="gray_r", vector=PLOT.paper_vector,
                        current=PLOT.paper_current, empty=PLOT.paper_empty,
                        recorded=PLOT.recorded_pose, axis=PLOT.axis_paper,
                        initial=PLOT.initial_guess, estimate=PLOT.estimate_paper,
                        alternative=PLOT.alternative_paper),
}
LEGEND = (("Occupied", part2.OCCUPIED), ("Free", part2.FREE), ("Unknown", part2.UNKNOWN))


def point_bounds(points):
    """Display bounds only, with room for the sensor origin and axis ticks."""
    points = np.asarray(points).reshape(-1, 2)
    if not len(points):
        return (-1, 1), (-1, 1)
    low, high = points.min(axis=0), points.max(axis=0)
    margin = np.maximum((high - low) * 0.05, 0.2)
    return tuple(zip(low - margin, high + margin))


class Viewport:
    def __init__(self, page, parent, key):
        self.page, self.key = page, key
        self.resize_job = None
        self.sized = False
        self.home = ((-1, 1), (-1, 1))
        # The 1-pixel frame border and the header's top accent turn purple
        # on the active viewport (the one the tool bar navigates).
        self.frame = tk.Frame(parent, background=VIEW_BG, borderwidth=1)
        self.accent = tk.Frame(self.frame, height=2, background=VIEW_CAPTION)
        self.accent.pack(fill="x")
        self.header = tk.Frame(self.frame, background=VIEW_CAPTION)
        self.header.pack(fill="x")
        self.max_button = ttk.Button(self.header, text="Maximise", style="Viewport.TButton",
                                     command=lambda: page.toggle_maximise(key))
        self.max_button.pack(side="right", padx=2, pady=1)
        tip(self.max_button, "Maximise / restore this viewport; double-click its title also works.")
        # Compact status of the view (frame, count, mode), right of the title.
        self.caption = tk.Label(self.header, text="Local" if key == "current" else "Global",
                                anchor="e", padx=6, bg=VIEW_CAPTION, fg=VIEW_MUTED,
                                font=("Segoe UI", 8))
        fonts = FontSet.current
        if fonts is not None and fonts.tk_available:
            self.title = TechnicalLabel(self.header, VIEW_TITLES[key], fonts, size=10,
                                        background=VIEW_CAPTION, foreground=VIEW_TEXT, padx=7, pady=3)
        else:
            self.title = tk.Label(self.header, text=VIEW_TITLES[key], anchor="w", padx=6,
                                  bg=VIEW_CAPTION, fg=VIEW_TEXT, font=("Segoe UI", 9, "bold"))
        self.title.pack(side="left")
        self.caption.pack(side="right")  # packed after the title: the caption shrinks first
        for widget in (self.title, self.header):
            widget.bind("<Button-1>", lambda _: page.activate(key))
            widget.bind("<Double-Button-1>", lambda _: page.toggle_maximise(key))
        tip(self.title, VIEW_HELP[key] + NAV_HELP)
        tip(self.caption, VIEW_HELP[key])
        self.figure = Figure(figsize=(4.5, 3.5), facecolor=VIEW_BG)
        # Tight margins: the map, not the frame, takes the viewport.
        self.axes = self.figure.add_axes((0.11, 0.11, 0.86, 0.86))
        self.axes.set(aspect="equal")
        self.axes.set_xlabel("X (m)", fontsize=8, labelpad=2)
        self.axes.set_ylabel("Y (m)", fontsize=8, labelpad=2)
        self.axes.tick_params(labelsize=7, length=3, pad=2)
        self.axes.set_autoscale_on(False)
        self.empty = self.axes.text(0.5, 0.5, "", transform=self.axes.transAxes,
                                    ha="center", va="center", fontsize=10)
        self.apply_style(PLOT_STYLES["Dark"])
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.frame)
        # Activation runs before toolbar press handling, so the first drag in
        # another viewport inherits the active Pan/Zoom tool correctly.
        self.canvas.mpl_connect("button_press_event", lambda _: page.activate(key))
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.frame, pack_toolbar=False)
        widget = self.canvas.get_tk_widget()
        widget.configure(background=VIEW_BG)
        widget.pack(fill="both", expand=True)
        widget.bind("<Configure>", self.on_configure)
        widget.bind("<Map>", lambda _: self.draw(), add="+")
        self.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.canvas.mpl_connect("figure_leave_event", lambda _: page.app.show_cursor(None))
        self.canvas.mpl_connect("scroll_event", self.on_scroll)
        self.axes.callbacks.connect("xlim_changed", lambda _: page.queue_link(key))
        self.axes.callbacks.connect("ylim_changed", lambda _: page.queue_link(key))

    def apply_style(self, style):
        axes = self.axes
        axes.set_facecolor(style["face"])
        axes.tick_params(colors=style["tick"])
        axes.xaxis.label.set_color(style["label"])
        axes.yaxis.label.set_color(style["label"])
        for spine in axes.spines.values():
            spine.set_color(style["spine"])
        colour, width, alpha = style["grid"]
        axes.grid(True, color=colour, linewidth=width, alpha=alpha)
        self.empty.set_color(style["empty"])

    def draw(self):
        if self.frame.winfo_ismapped():
            self.canvas.draw_idle()

    def on_configure(self, event):
        if not self.page.alive:
            return
        if self.resize_job is not None:
            event.widget.after_cancel(self.resize_job)
        if not self.sized:
            self.sized = True
            self.canvas.resize(event)
        else:
            self.resize_job = event.widget.after(RESIZE_SETTLE_MS, lambda: self.resize(event))

    def resize(self, event):
        self.resize_job = None
        if self.page.alive:
            self.canvas.resize(event)

    def on_motion(self, event):
        self.page.app.show_cursor((event.xdata, event.ydata)
                                  if event.inaxes is self.axes else None)

    def on_scroll(self, event):
        if event.inaxes is not self.axes or event.xdata is None:
            return
        self.page.activate(self.key)
        scale = 1.2 ** -event.step
        self.toolbar.push_current()
        for anchor, limits, setter in ((event.xdata, self.axes.get_xlim(), self.axes.set_xlim),
                                       (event.ydata, self.axes.get_ylim(), self.axes.set_ylim)):
            setter([anchor + (value - anchor) * scale for value in limits])
        self.toolbar.push_current()
        self.draw()

    def fit(self):
        self.toolbar.push_current()
        self.axes.set(xlim=self.home[0], ylim=self.home[1])
        self.toolbar.push_current()
        self.draw()

    def close(self):
        if self.resize_job is not None:
            # Cancel through the widget that scheduled it (on_configure), so
            # Tk's command list stays consistent for its later destroy().
            self.canvas.get_tk_widget().after_cancel(self.resize_job)
            self.resize_job = None


class MapRequest:
    """One map's busy state, independent of the other Workspace requests."""
    def __init__(self, page, key, viewport):
        self.page, self.key, self.viewport = page, key, viewport
        self.busy = False

    @property
    def alive(self):
        return self.page.alive

    def set_busy(self, busy, label=None):
        self.busy = busy
        if busy:
            self.page.pending.add(self.key)
        else:
            self.page.pending.discard(self.key)
        self.page.update_busy()

    def set_info(self, text):
        set_text(self.page.views[self.viewport].caption, text)
        self.page.set_info(text)


class WorkspacePage(Page):
    INTERACTIVE_SCANS = True
    SCAN_LIST_NOTE = False
    SCAN_LIST_HINT = ("Select a recorded scan for all Workspace views and 01 Scan. "
                      "Visibility and alignment are edited on 01 Scan.")
    SOURCES = ("Selected recorded scan", "Latest capture", "Localisation target")

    def __init__(self, app):
        self.pending = set()
        self.capture_busy = False  # any Page.run job: a capture or a localisation
        # One-shot localisation of an independent scan (never added to the session).
        self.targets, self.target_source, self.recorded_target = [], "", None
        self.guess = (0.0, 0.0, 0.0)  # Day 1 units: x m, y m, heading deg
        self.outcome = self.outcome_key = None
        self.loc_generation = 0
        self.stop = threading.Event()  # Page.close() sets it: a closed page's search stops
        self.loc_label = None
        self.loc_progress = self.shown_progress = None
        self.vmap = None
        self.link_job = self.layout_job = None
        self.maximised = None
        self.active = "scan"
        self.selection_signature = None
        self.local_fitted = False
        self.grid_arrays = {}
        self.grid_image = None
        self.link = tk.BooleanVar(master=app.root, value=False)
        self.source = tk.StringVar(master=app.root, value=self.SOURCES[0])
        super().__init__(app)
        self.caption.master.pack_forget()  # viewport headers carry their own status
        self.page_header("Workspace", None, "Four views of the working session. Global maps use "
                                            "recorded alignment poses." + NAV_HELP)
        # -- frequent controls ------------------------------------------------------
        current = heading(self.controls, "Current scan")
        self.scan_choice = ttk.Combobox(current, state="readonly")
        self.scan_choice.pack(fill="x", pady=(0, 3))
        tip(self.scan_choice, "Recorded scan shown in every Workspace view; shared with 01 Scan.")
        self.scan_choice.bind("<<ComboboxSelected>>", self.select)
        self.scan_tree.bind("<<TreeviewSelect>>", self.select_tree)
        self.source_choice = ttk.Combobox(current, state="readonly", values=self.SOURCES,
                                          textvariable=self.source)
        self.source_choice.pack(fill="x")
        tip(self.source_choice, "Source of the Current Scan view: the selected recorded scan, "
                                "or the latest completed LD06 capture.")
        self.source_choice.bind("<<ComboboxSelected>>", lambda _: self.refresh_selection())
        fonts = FontSet.current
        if fonts is not None and fonts.tk_available:
            # Technical readout: selected scan and its point count.
            self.current_info = TechnicalLabel(current, "", fonts, size=10, background=CHROME,
                                               foreground=THEME.text, padx=0, pady=3)
            self.current_info.pack(anchor="w", pady=(5, 0))
        else:
            self.current_info = note(current, "")
        tip(self.current_info, VIEW_HELP["current"])
        self.build_localisation_controls()

        display = heading(self.controls, "Display")
        line = row(display)
        ttk.Label(line, text="Grid", style="Field.TLabel", width=6).pack(side="left")
        self.view = ttk.Combobox(line, state="readonly", values=GridPage.MODES)
        self.view.set("Probability")
        self.view.pack(side="left", fill="x", expand=True)
        self.view.bind("<<ComboboxSelected>>", lambda _: self.draw_grid())
        tip(self.view, lambda: GridPage.MODE_HELP[self.view.get()])
        line = row(display)
        ttk.Label(line, text="Plots", style="Field.TLabel", width=6).pack(side="left")
        self.plot_style = ttk.Combobox(line, state="readonly", values=tuple(PLOT_STYLES))
        self.plot_style.set("Dark")
        self.plot_style.pack(side="left", fill="x", expand=True)
        self.plot_style.bind("<<ComboboxSelected>>", lambda _: self.set_plot_style())
        tip(self.plot_style, "Plot background for the four views. Colours only; map values, "
                             "scan colours and coordinates are unchanged.")
        self.legend = tk.Frame(display, background=CHROME)
        self.legend.pack(fill="x", pady=(5, 0))
        self.legend_swatches = []
        for text, _ in LEGEND:
            # Wide flat bars, so the key cannot be mistaken for checkboxes.
            swatch = tk.Frame(self.legend, width=18, height=8, highlightthickness=1,
                              highlightbackground=THEME.border)
            swatch.pack(side="left", padx=(0, 5))
            ttk.Label(self.legend, text=text, style="Field.TLabel").pack(side="left", padx=(0, 10))
            self.legend_swatches.append(swatch)
        tip(self.legend, "Occupancy grid cell classes in the current plot style.")

        view = heading(self.controls, "View")
        link = ttk.Checkbutton(view, text="Link global pan / zoom", variable=self.link,
                               command=self.link_changed)
        link.pack(anchor="w")
        tip(link, "Links Scan Map, Bitmap and Vector Map. Current Scan always keeps its own "
                  "sensor-local view.")
        line = row(view)
        button(line, "Fit All Views", self.fit_all, side="left",
               tooltip="Fit every viewport to its map extent.")
        button(line, "Restore Panels", self.restore, side="left",
               tooltip="Return a maximised viewport to the four-panel layout.")

        capture = heading(self.controls, "Capture (LD06)")
        self.capture_controls(self.captured, parent=capture, heading=False, explain=False)
        self.build_drive_controls()

        # -- rarely needed details ---------------------------------------------------
        details = expander(self.controls, "Details", tooltip="Map sizes, cache state and data notes.")
        self.map_stats = note(details, "")
        note(details, "Global maps use recorded alignment poses. Green markers show the selected "
                      "scan's recorded pose; blue shows a one-shot localisation estimate of an "
                      "independent scan (hollow and dashed when only plausible).")
        note(details, "A Workspace capture stays separate from the global map. Use 01 Scan to "
                      "add and align scans.")
        self.loc_details = note(details, "")
        tracking = ttk.Label(details, text="Tracking: continuous tracking is not implemented yet.",
                             style="Note.TLabel")
        tracking.state(["disabled"])
        tracking.pack(fill="x", pady=(2, 4))
        self.global_points = self.model.cache.get("global_points")
        if self.global_points is None:
            self.global_points = part2.global_points_of_scans(self.model.scans)
            self.model.cache["global_points"] = self.global_points
        self.draw_scans()
        self.create_selection_artists()
        self.refresh_selection()
        self.update_localisation_controls()
        if self.model.scans:
            app.derived(MapRequest(self, "grid", "grid"), "grid", build_grid, self.grid_ready)
            app.derived(MapRequest(self, "vectors", "vector"), "vectors", build_vectors, self.vectors_ready)
        else:
            for key in GLOBAL_VIEWS:
                self.views[key].empty.set_text("No visible scans\nOpen a session or use 01 Scan")
                self.views[key].draw()
            self.set_info("Open a recorded session or capture and align scans on 01 Scan.")
        # Page has now mapped the real frame; earlier idle callbacks can see a
        # 1-pixel paned window and clamp every sash to its minimum position.
        self.place_dividers((0.5, 0.5, 0.5))
        self.set_plot_style()

    def build_viewport(self):
        options = dict(background=CHROME_2, sashwidth=7, sashrelief="flat",
                       borderwidth=0, showhandle=False, opaqueresize=True)
        self.split = tk.PanedWindow(self.plot_frame, orient="vertical", **options)
        self.split.pack(fill="both", expand=True)
        self.rows = [tk.PanedWindow(self.split, orient="horizontal", **options) for _ in range(2)]
        self.views = {}
        for index, keys in enumerate((("scan", "grid"), ("vector", "current"))):
            self.split.add(self.rows[index], minsize=130, stretch="always")
            for key in keys:
                viewport = self.views[key] = Viewport(self, self.rows[index], key)
                self.rows[index].add(viewport.frame, minsize=170, stretch="always")
        self.resize_job = None  # the four viewports own their resize timers
        self.activate("scan")

    def place_dividers(self, ratios):
        self.layout_job = None
        if not self.alive or self.maximised:
            return
        self.split.sash_place(0, 0, int(self.split.winfo_height() * ratios[0]))
        for row, ratio in zip(self.rows, ratios[1:]):
            row.sash_place(0, int(row.winfo_width() * ratio), 0)

    def divider_ratios(self):
        return (self.split.sash_coord(0)[1] / max(1, self.split.winfo_height()),
                *(row.sash_coord(0)[0] / max(1, row.winfo_width()) for row in self.rows))

    def activate(self, key):
        previous = self.views.get(self.active)
        mode = previous.toolbar.mode if previous is not None and hasattr(previous, "toolbar") else ""
        if previous is not None and key != self.active and mode:
            getattr(previous.toolbar, "pan" if mode == "pan/zoom" else "zoom")()
        self.active = key
        viewport = self.views[key]
        self.figure, self.canvas, self.toolbar = viewport.figure, viewport.canvas, viewport.toolbar
        self.map_axes = viewport.axes
        if mode and not self.toolbar.mode:
            getattr(self.toolbar, "pan" if mode == "pan/zoom" else "zoom")()
        for name, view in self.views.items():
            active = name == key
            view.title.configure(foreground=THEME.lavender if active else VIEW_TEXT)
            view.accent.configure(background=SELECT if active else VIEW_CAPTION)
            view.frame.configure(background=SELECT if active else VIEW_BG)
        if self.app.page() is self:
            self.app.update_tool_states()

    def toggle_maximise(self, key):
        if self.maximised == key:
            self.restore()
            return
        if self.maximised:
            self.restore()
            self.frame.update_idletasks()
        self.saved_ratios = self.divider_ratios()
        if self.layout_job is not None:
            self.frame.after_cancel(self.layout_job)
            self.layout_job = None
        self.activate(key)
        self.maximised = key
        index = 0 if key in ("scan", "grid") else 1
        self.split.forget(self.rows[1 - index])
        for name, view in self.views.items():
            if name != key and view.frame.master is self.rows[index]:
                self.rows[index].forget(view.frame)
        self.views[key].max_button.configure(text="Restore")

    def restore(self):
        if self.maximised is None:
            return
        self.views[self.maximised].max_button.configure(text="Maximise")
        self.maximised = None
        for pane in self.split.panes():
            self.split.forget(pane)
        for row in self.rows:
            for pane in row.panes():
                row.forget(pane)
        for index, keys in enumerate((("scan", "grid"), ("vector", "current"))):
            self.split.add(self.rows[index], minsize=130, stretch="always")
            for key in keys:
                self.rows[index].add(self.views[key].frame, minsize=170, stretch="always")
        self.frame.update_idletasks()
        self.place_dividers(self.saved_ratios)
        for view in self.views.values():
            view.canvas.draw_idle()

    def queue_link(self, key):
        if not self.link.get() or key not in GLOBAL_VIEWS or not self.alive:
            return
        if self.link_job is not None:
            self.frame.after_cancel(self.link_job)
        self.link_job = self.frame.after_idle(lambda: self.sync_limits(key))

    def sync_limits(self, key):
        self.link_job = None
        if not self.link.get() or not self.alive:
            return
        axes = self.views[key].axes
        for name in GLOBAL_VIEWS:
            if name != key:
                other = self.views[name]
                other.axes.set_xlim(axes.get_xlim(), emit=False)
                other.axes.set_ylim(axes.get_ylim(), emit=False)
                other.draw()

    def link_changed(self):
        if self.link.get():
            self.queue_link(self.active if self.active in GLOBAL_VIEWS else "scan")

    def fit_map(self):
        self.views[self.active].fit()

    def fit_all(self):
        for view in self.views.values():
            view.fit()

    def update_busy(self):
        # Map requests can overlap a capture; they must not replace the capture
        # button lock list maintained by Page.set_busy().
        self.busy = self.capture_busy or bool(self.pending)
        self.frame.configure(cursor="watch" if self.busy else "")

    def set_busy(self, busy, label=None, controls=()):
        self.capture_busy = busy
        Page.set_busy(self, busy, label, controls)
        self.update_busy()

    def draw_scans(self):
        axes = self.views["scan"].axes
        self.scan_artists = []
        for scan, points in zip(self.model.scans, self.global_points):
            artist = axes.scatter(points[:, 0], points[:, 1], s=1, color=scan["color"])
            self.scan_artists.append(artist)
        all_points = np.vstack([np.zeros((1, 2)), *self.global_points,
                                np.array([s["pose"][:2] for s in self.model.scans]).reshape(-1, 2)])
        bounds = point_bounds(all_points)
        for key in GLOBAL_VIEWS:
            view = self.views[key]
            view.home = bounds
            view.axes.set(xlim=bounds[0], ylim=bounds[1])
            view.toolbar.push_current()
        set_text(self.views["scan"].caption, f"Global · {len(self.model.scans)} scans")
        self.views["scan"].draw()

    def grid_ready(self, data):
        self.grid, self.log_odds = data
        classified = np.full(self.grid.state.shape, part2.UNKNOWN, dtype=np.uint8)
        classified[self.grid.state == loc.FREE_STATE] = part2.FREE
        classified[self.grid.state == loc.OCCUPIED_STATE] = part2.OCCUPIED
        self.grid_arrays.update(Probability=part2.probabilities_to_uint8(self.grid.probability),
                                Classified=classified, Empty=part2.create_occupancy_grid(self.grid.spec()))
        left, right, bottom, top = self.grid.extent
        for key in GLOBAL_VIEWS:
            view = self.views[key]
            # Set the fit extent without moving an already navigated viewport.
            view.home = ((left, right), (bottom, top))
        self.draw_grid()
        set_text(self.map_stats, f"Grid: {self.grid.width} × {self.grid.height} cells\n"
                                 f"Cell size: {part2.CELL_SIZE_M:.2f} m\nMaps cached for this session revision.")
        self.set_info("Maps ready (cached for this session).")

    def draw_grid(self):
        kind = self.view.get()
        if not hasattr(self, "grid"):
            return
        if kind == "Binary":
            if "binary" not in self.model.cache:
                if "binary" not in self.pending:
                    spec, arrays = self.grid.spec(), self.global_points
                    self.app.derived(MapRequest(self, "binary", "grid"), "binary",
                                     lambda scans: part2.build_binary_occupancy_grid(spec, scans, arrays)[0],
                                     self.binary_ready)
                return
            data = self.model.cache["binary"]
        else:
            data = self.grid_arrays[kind]
        viewport = self.views["grid"]
        if self.grid_image is None:
            self.grid_image = viewport.axes.imshow(data, cmap=self.style()["cmap"], vmin=part2.FREE,
                                                   vmax=part2.OCCUPIED, origin="lower",
                                                   extent=self.grid.extent, interpolation="nearest")
        else:
            self.grid_image.set_data(data)
        set_text(viewport.caption, f"{part2.CELL_SIZE_M:.2f} m cells · {kind}")
        viewport.draw()

    def binary_ready(self, _):
        self.draw_grid()
        self.set_info("Binary grid ready (cached).")

    def vectors_ready(self, data):
        self.vector_data = data
        merged = data[3]
        viewport = self.views["vector"]
        self.segments = LineCollection([[s["start"], s["end"]] for s in merged],
                                       colors=self.style()["vector"], linewidths=1.6)
        viewport.axes.add_collection(self.segments)
        set_text(viewport.caption, f"Global · {len(merged)} segments")
        viewport.empty.set_text("" if merged else "No finite segments extracted")
        viewport.draw()
        self.set_info("Maps ready (cached for this session).")

    def create_selection_artists(self):
        style = self.style()
        self.pose_artists = {}
        for key in GLOBAL_VIEWS:
            axes = self.views[key].axes
            marker, = axes.plot([], [], "o", ms=7, mfc="none", mec=style["recorded"], mew=1.7, zorder=5)
            heading, = axes.plot([], [], color=style["recorded"], linewidth=1.7, zorder=5)
            self.pose_artists[key] = marker, heading
        self.highlight = self.views["scan"].axes.scatter([], [], s=3, color=style["recorded"], zorder=4)
        axes = self.views["current"].axes
        self.current_points = axes.scatter([], [], s=3, color=style["current"])
        self.axis_cross, = axes.plot(0, 0, "+", ms=9, color=style["axis"])
        self.axis_label = axes.annotate("+X", xy=(0.4, 0), xytext=(0.65, 0), color=style["axis"],
                                        arrowprops={"arrowstyle": "<-", "color": style["axis"]},
                                        fontsize=8)
        # Localisation overlays: the SAME pose on every global view, never on Current Scan.
        self.estimate_artists = {}
        for key in GLOBAL_VIEWS:
            axes = self.views[key].axes
            points = axes.scatter([], [], s=3, color=style["estimate"], zorder=6)
            marker, = axes.plot([], [], "o", ms=8, color=style["estimate"], mew=1.8, zorder=7)
            heading, = axes.plot([], [], color=style["estimate"], linewidth=1.8, zorder=7)
            guess, = axes.plot([], [], "o", ms=7, mfc="none", mec=style["initial"], mew=1.5, zorder=6)
            guess_heading, = axes.plot([], [], color=style["initial"], linewidth=1.5, zorder=6)
            self.estimate_artists[key] = points, marker, heading, guess, guess_heading

    def select_id(self, scan_id):
        if scan_id != self.model.session["selected_id"]:
            scanner = self.app.pages.get("LiDAR Scanner")
            if scanner is not None:
                scanner.action("select", scan_id)
            else:
                self.model.session["selected_id"] = scan_id
        self.source.set(self.SOURCES[0])
        self.refresh_selection()

    def select(self, _=None):
        index = self.scan_choice.current()
        if index >= 0:
            self.select_id(self.model.session["scans"][index]["id"])

    def select_tree(self, _=None):
        selection = self.scan_tree.selection()
        if selection and selection[0] != str(self.model.session["selected_id"]):
            self.select_id(int(selection[0]))

    def refresh_selection(self):
        scans = self.model.session["scans"]
        selected = part1.selected_scan(self.model.session)
        self.scan_choice["values"] = [f"{s['id']} — {s['label']}" for s in scans]
        if selected is not None:
            self.scan_choice.current(next(i for i, scan in enumerate(scans) if scan["id"] == selected["id"]))
        else:
            self.scan_choice.set("")
        self.refresh_scan_tree()
        visible = selected is not None and selected["visible"]
        for marker, heading in self.pose_artists.values():
            if visible:
                x, y, degrees = selected["pose"]
                angle = np.deg2rad(degrees)
                marker.set_data([x], [y])
                heading.set_data([x, x + 0.45 * np.cos(angle)], [y, y + 0.45 * np.sin(angle)])
            marker.set_visible(visible)
            heading.set_visible(visible)
        points = next((p for s, p in zip(self.model.scans, self.global_points)
                       if selected is not None and s["id"] == selected["id"]), np.empty((0, 2)))
        self.highlight.set_offsets(points)
        capture = self.model.workspace_inputs.latest_capture
        if self.source.get() == self.SOURCES[1]:
            local = capture.points_local if capture is not None else np.empty((0, 2))
            label = capture.label if capture is not None else "No completed capture"
        elif self.source.get() == self.SOURCES[2]:
            target = self.current_target()  # its own local points, exactly
            local = target.points_local if target is not None else np.empty((0, 2))
            label = target.label if target is not None else "No localisation target"
        else:
            local = np.column_stack(part1.scan_to_local(selected)) if selected is not None else np.empty((0, 2))
            label = selected["label"] if selected is not None else "No selected scan"
        self.current_points.set_offsets(local)
        viewport = self.views["current"]
        viewport.empty.set_text("" if len(local) else label)
        set_text(viewport.caption, f"Local · {label}")
        set_text(self.current_info, f"{label} · {len(local):,} points")
        viewport.home = point_bounds(np.vstack([np.zeros((1, 2)), local]))
        if len(local) and not self.local_fitted:
            viewport.fit()
            self.local_fitted = True
        for view in self.views.values():
            view.draw()
        self.selection_signature = (self.model.session["selected_id"], id(capture), self.source.get())

    # -- one-shot localisation (Codes 04/05/06 through workspace_localisation) -------
    # -- manual driving (docs/specs/workspace_drive.md) ------------------------------
    def build_drive_controls(self):
        body = heading(self.controls, "Drive (robot)",
                       "Manual driving over Wi-Fi with the professor's HTTP routes. Keys act only "
                       "while the drive pad has focus; releasing a key or leaving the pad stops.")
        self.drive = robot_drive.DriveController()
        line = row(body)
        ttk.Label(line, text="URL", style="Field.TLabel", width=7).pack(side="left")
        self.drive_url = ttk.Entry(line)
        self.drive_url.insert(0, robot_drive.DEFAULT_URL)
        self.drive_url.pack(side="left", fill="x", expand=True)
        line = row(body)
        button(line, "Connect", self.drive_connect, side="left",
               tooltip="Join the robot's Wi-Fi first. Sends /ping.")
        button(line, "Disconnect", self.drive.disconnect, side="left",
               tooltip="Sends /stop, then stops sending commands.")
        line = row(body)
        ttk.Label(line, text="Speed", style="Field.TLabel", width=7).pack(side="left")
        self.drive_speed_label = ttk.Label(line, text=str(robot_drive.DEFAULT_SPEED), width=4)
        self.drive_speed_label.pack(side="right")
        self.drive_speed = ttk.Scale(line, from_=0, to=255, command=self.drive_speed_changed)
        self.drive_speed.set(robot_drive.DEFAULT_SPEED)
        self.drive_speed.pack(side="left", fill="x", expand=True)
        tip(self.drive_speed, "Motor PWM 0-255 sent with each move (MecanumCar_v2 default 80).")
        self.drive_status = note(body, "Not connected.")
        self.drive_pad = tk.Frame(body, takefocus=1, background=CHROME_2, highlightthickness=2,
                                  highlightbackground=THEME.border, highlightcolor=SELECT)
        self.drive_pad.pack(fill="x", pady=(2, 4))
        hint = tk.Label(self.drive_pad, text="Click here, then hold\nZ S forward/back · Q D strafe · "
                                             "A E rotate\nSpace = STOP",
                        background=CHROME_2, foreground=THEME.text, justify="center", pady=6)
        hint.pack(fill="x")
        for widget in (self.drive_pad, hint):
            widget.bind("<Button-1>", lambda _: self.drive_pad.focus_set())
        self.drive_pad.bind("<KeyPress>", self.drive_key)
        self.drive_pad.bind("<KeyRelease>", self.drive_key_up)
        self.drive_pad.bind("<FocusOut>", lambda _: self.drive.stop())

    def drive_connect(self):
        self.drive.set_speed(self.drive_speed.get())
        self.drive.connect(self.drive_url.get())

    def drive_speed_changed(self, value):
        speed = int(float(value))
        self.drive.set_speed(speed)
        set_text(self.drive_speed_label, str(speed))

    def drive_key(self, event):
        key = event.keysym.lower()
        if key in ("space", "escape"):
            self.drive.stop()
            return "break"
        if key in robot_drive.KEYS:
            self.drive.hold(key)  # Windows auto-repeat only renews the same hold
            return "break"
        return None

    def drive_key_up(self, event):
        key = event.keysym.lower()
        if key in robot_drive.KEYS:
            self.drive.release(key)
            return "break"
        return None

    def tick_drive(self):
        if not self.alive:  # a closed page can still be ticked while others close
            return
        try:
            focused = self.frame.focus_get() is self.drive_pad
        except (KeyError, tk.TclError):  # focus inside a combobox popdown
            focused = False
        if focused and self.drive_pad.winfo_viewable():
            self.drive.keepalive()  # without it the worker drops a held key in 0.5 s
        set_text(self.drive_status, self.drive.status()[1])

    def build_localisation_controls(self):
        body = heading(self.controls, "Localisation",
                       "Localise ONE independent scan on the fixed global maps. The scan is "
                       "never added to the session and no map is rebuilt.")
        line = row(body)
        ttk.Label(line, text="Target", style="Field.TLabel", width=7).pack(side="left")
        self.target_choice = ttk.Combobox(line, state="readonly", values=TARGETS)
        self.target_choice.current(0)
        self.target_choice.pack(side="left", fill="x", expand=True)
        self.target_choice.bind("<<ComboboxSelected>>", lambda _: self.target_changed())
        tip(self.target_choice, "Latest capture: the last completed LD06 capture (Capture group).\n\n"
                                "Separate recording: one scan of another session file; its saved "
                                "pose is ignored. Scans of this session are refused.")
        self.recording_controls = ttk.Frame(body)
        button(self.recording_controls, "Load Recorded Scan…", self.choose_recording,
               tooltip="Open a separate session JSON and pick one of its scans as the target.")
        self.recorded_choice = ttk.Combobox(self.recording_controls, state="readonly")
        self.recorded_choice.pack(fill="x", pady=(0, 2))
        self.recorded_choice.bind("<<ComboboxSelected>>", lambda _: self.choose_recorded_target())
        self.method_line = line = row(body)
        ttk.Label(line, text="Method", style="Field.TLabel", width=7).pack(side="left")
        self.method_choice = ttk.Combobox(line, state="readonly", values=tuple(wl.METHODS.values()))
        self.method_choice.current(0)
        self.method_choice.pack(side="left", fill="x", expand=True)
        self.method_choice.bind("<<ComboboxSelected>>", lambda _: self.inputs_changed())
        tip(self.method_choice, "04 Grid: occupancy grid. 05 Vector: finite segments. 06 Scan: "
                                "recorded scans. Each score has its own thresholds; never "
                                "compare scores across methods.")
        self.scan_reference_controls = ttk.Frame(body)
        self.reference_mode = ttk.Combobox(self.scan_reference_controls, state="readonly",
                                           values=tuple(wl.SCAN_REFERENCES.values()))
        self.reference_mode.current(0)
        self.reference_mode.pack(fill="x", pady=(0, 2))
        self.reference_mode.bind("<<ComboboxSelected>>", lambda _: self.inputs_changed())
        tip(self.reference_mode, "Code 06 reference: all positioned scans pooled, or ONE recorded "
                                 "scan at its saved pose (needs real overlap).")
        self.reference_choice = ttk.Combobox(self.scan_reference_controls, state="readonly",
                                             values=[f"Scan {s['id']}" for s in self.model.scans])
        if self.model.scans:
            self.reference_choice.current(0)
        self.reference_choice.bind("<<ComboboxSelected>>", lambda _: self.inputs_changed())
        self.search_line = line = row(body)
        ttk.Label(line, text="Search", style="Field.TLabel", width=7).pack(side="left")
        self.search_choice = ttk.Combobox(line, state="readonly", values=tuple(wl.SEARCHES.values()))
        self.search_choice.current(0)
        self.search_choice.pack(side="left", fill="x", expand=True)
        self.search_choice.bind("<<ComboboxSelected>>", lambda _: self.inputs_changed())
        self.local_controls = ttk.Frame(body)
        field_label(self.local_controls, "Initial guess (Day 1 units)")
        self.pose_inputs = PoseInputs(self.local_controls, lambda: self.guess, self.set_guess,
                                      "Set Guess", "Use the typed X, Y and heading as the local "
                                                   "search's initial guess (orange).")
        button(self.local_controls, "Use Selected Scan's Recorded Pose", self.guess_from_selected,
               tooltip="Copy the selected recorded scan's Day 1 pose into the guess.")
        self.action_line = line = row(body)
        self.localise_button = button(line, "Localise", self.localise, style="Primary.TButton",
                                      side="left", tooltip="Run the selected method once on the "
                                                           "target. The maps never change.")
        self.cancel_button = button(line, "Cancel", self.cancel_localisation, side="left",
                                    tooltip="Stop a running search; nothing is changed. Code 04 "
                                            "local search cannot be stopped.")
        line = row(body)
        button(line, "Clear Result", self.clear_result, side="left",
               tooltip="Remove the estimate from the global views.")
        button(line, "Fit View", self.fit_all, side="left", tooltip="Fit every viewport.")
        self.verdict_chip = ttk.Label(body, text="No result yet", style=VERDICT_STYLES["none"])
        self.verdict_chip.pack(anchor="w", pady=(4, 0))
        tip(self.verdict_chip, "Only 'ok' is a position fix; 'weak' is plausible, not confirmed. "
                               "Rejected results draw no pose.")
        self.result_text = note(body, "Capture a scan or load a separate recording, then Localise.",
                                style="Mono.TLabel")

    def method_key(self):
        return tuple(wl.METHODS)[max(0, self.method_choice.current())]

    def search_key(self):
        return tuple(wl.SEARCHES)[max(0, self.search_choice.current())]

    def scan_reference_key(self):
        return tuple(wl.SCAN_REFERENCES)[max(0, self.reference_mode.current())]

    def reference_id(self):
        index = self.reference_choice.current()
        return self.model.scans[index]["id"] if 0 <= index < len(self.model.scans) else None

    def current_target(self):
        if self.target_choice.current() == 1:
            return self.recorded_target
        return self.model.workspace_inputs.latest_capture

    def current_request(self):
        """The wl.Request the controls describe now, or None if incomplete."""
        target = self.current_target()
        if target is None:
            return None
        local = self.search_key() == "local"
        try:
            return wl.Request(self.method_key(), self.search_key(), target.points_local,
                              target.label, guess_day1=self.guess if local else None,
                              scan_reference=self.scan_reference_key(),
                              reference_id=self.reference_id(),
                              reference_revision=self.model.revision)
        except ValueError:
            return None

    def inputs_key(self):
        """Cheap identity of the inputs (polled by tick); see Request.signature."""
        local = self.search_key() == "local"
        scans = self.method_key() == "scans"
        return (id(self.current_target()), self.method_key(), self.search_key(),
                self.guess if local else None, self.scan_reference_key() if scans else None,
                self.reference_id() if scans else None, self.model.revision)

    def update_localisation_controls(self):
        if self.method_key() == "scans":
            self.scan_reference_controls.pack(fill="x", after=self.method_line)
            if self.scan_reference_key() == "single":
                self.reference_choice.pack(fill="x", pady=(0, 2))
            else:
                self.reference_choice.pack_forget()
        else:
            self.scan_reference_controls.pack_forget()
        if self.search_key() == "local":
            self.local_controls.pack(fill="x", after=self.search_line)
        else:
            self.local_controls.pack_forget()
        if self.target_choice.current() == 1:
            self.recording_controls.pack(fill="x", after=self.target_choice.master)
        else:
            self.recording_controls.pack_forget()
        cancellable = wl.cancellable(self.method_key(), self.search_key())
        self.cancel_button.state(["!disabled"] if cancellable else ["disabled"])

    def inputs_changed(self):
        """Any target, method, search, guess or reference change: a running
        search's result becomes stale and a shown result is cleared."""
        self.loc_generation += 1
        self.update_localisation_controls()
        if self.outcome is not None and self.inputs_key() != self.outcome_key:
            self.clear_result("Inputs changed: press Localise again.")
        else:
            self.draw_estimate()
            self.draw_global()

    def target_changed(self):
        self.inputs_changed()
        self.source.set(self.SOURCES[2])
        self.refresh_selection()

    def set_guess(self, pose):
        self.guess = tuple(float(value) for value in pose)
        self.inputs_changed()

    def guess_from_selected(self):
        selected = part1.selected_scan(self.model.session)
        if selected is None:
            self.set_info("Select a recorded scan first.")
            return
        self.set_guess(selected["pose"])
        self.pose_inputs.sync()

    def choose_recording(self):
        path = filedialog.askopenfilename(parent=self.app.root, title="Load a separate recorded scan",
                                          initialdir=common.SCANS_DIR,
                                          filetypes=[("LiDAR session JSON", "*.json")])
        if path:
            self.load_recording(path)

    def load_recording(self, path):
        try:
            targets = wl.load_targets(path)
        except (ValueError, OSError) as error:
            self.set_info(f"Could not load {Path(path).name}: {error}")
            return False
        self.targets, self.target_source = targets, Path(path).name
        self.recorded_choice["values"] = [f"{t['id']} — {t['name']}" for t in targets]
        self.recorded_choice.current(0)
        self.target_choice.current(1)
        return self.choose_recorded_target()

    def choose_recorded_target(self):
        index = self.recorded_choice.current()
        if not 0 <= index < len(self.targets):
            return False
        target = self.targets[index]
        try:
            self.recorded_target = ScanFrame(target["points_local"], time.time(),
                                             f"{self.target_source}: {target['name']}",
                                             source="recorded")
        except ValueError as error:
            self.recorded_target = None
            self.set_info(str(error))
            self.target_changed()
            return False
        self.target_changed()
        message = wl.check_target(self.recorded_target.points_local, self.model.scans)
        self.set_info(message or f"{self.recorded_target.label}: "
                                 f"{len(self.recorded_target.points_local)} local points; source "
                                 "pose ignored. Press Localise.")
        return message is None

    def reference_for(self, method):
        """The method's fixed reference from the shared caches, or None if not ready."""
        if method == "grid":
            return getattr(self, "grid", None)
        if method == "vector":
            return self.vmap
        return self.model.scans

    def prepare_vector_reference(self):
        """Page 05's VectorMap for this revision, built once on a worker."""
        merged = self.vector_data[3]
        if not merged:
            self.set_info("The vector map has no segments: nothing to localise against.")
            return

        def ready(vmap):  # cached under the revision it was built for, as page 05
            self.vmap = vmap
            set_text(self.views["vector"].caption, f"Global · {len(merged)} segments")
            self.localise()
        self.app.derived(MapRequest(self, wl.VECTOR_REFERENCE_KEY, "vector"), wl.VECTOR_REFERENCE_KEY,
                         lambda _: vloc.VectorMap.from_merged(merged), ready)

    def localise(self):
        if self.busy:
            self.set_info("A capture is running: wait for it or cancel it."
                          if self.app.capture_owner is self else
                          "Maps are still building: wait, then press Localise."
                          if self.pending else "A localisation is already running.")
            return False
        target = self.current_target()
        if target is None:
            self.set_info("Capture a scan or load a separate recording first.")
            return False
        message = wl.check_target(target.points_local, self.model.scans)
        if message:
            self.set_info(message)
            return False
        method, search = self.method_key(), self.search_key()
        reference = self.reference_for(method)
        if reference is None:
            if method == "vector" and hasattr(self, "vector_data"):
                self.prepare_vector_reference()
            else:
                self.set_info("Maps are still building: wait, then press Localise.")
            return False
        request = self.current_request()
        if request is None:
            self.set_info("Choose a reference scan for the single-scan reference.")
            return False
        generation = self.loc_generation
        cancellable = wl.cancellable(method, search)
        self.stop = stop = threading.Event()
        self.loc_progress = 0.0 if cancellable else None  # Code 04 local reports no progress
        self.shown_progress = None
        self.loc_label = f"Localising with {wl.METHODS[method]} ({search} search)…"
        if not cancellable:
            self.loc_label += ("\nCode 04 local search cannot be stopped; a stale result "
                               "will be discarded.")

        def report(fraction):
            self.loc_progress = fraction  # read by tick() on the Tk thread

        def done(outcome):
            self.loc_progress = self.shown_progress = None
            if outcome.status == "cancelled":
                self.set_info("Search cancelled. Nothing was changed.")
                return
            current = self.current_request()
            if generation != self.loc_generation or current is None \
                    or outcome.request.signature != current.signature:
                self.set_info("Target, method, guess or reference changed during the search. "
                              "Press Localise again.")
                return
            self.outcome, self.outcome_key = outcome, self.inputs_key()
            self.model.workspace_inputs.publish_estimate(outcome.pose, wl.METHODS[method],
                                                         outcome.status, request.target_label)
            self.app.last_runtime = f"{outcome.result.runtime_ms:.0f} ms ({wl.METHODS[method]})"
            self.set_info(f"{outcome.status}: {outcome.result.reason}")
            self.source.set(self.SOURCES[2])
            self.refresh_selection()
            self.show_result()

        job = wl.make_job(request, reference, progress=report,
                          should_stop=stop.is_set if cancellable else None)
        return self.run(self.loc_label, job, done, controls=(self.localise_button,))

    def cancel_localisation(self):
        if wl.cancellable(self.method_key(), self.search_key()):
            self.stop.set()

    def clear_result(self, message="Result cleared."):
        self.loc_generation += 1  # a running search's result is discarded too
        self.outcome = self.outcome_key = None
        self.model.workspace_inputs.publish_estimate(None, "", "", "")
        self.set_info(message)
        self.show_result()

    def show_result(self):
        outcome = self.outcome
        show_verdict_chip(self, outcome.result if outcome is not None else None)
        set_text(self.result_text, self.result_lines())
        set_text(self.loc_details, "" if outcome is None else
                 RESULT_ADVICE.get(outcome.status, ""))
        self.draw_estimate()
        self.draw_global()

    def result_lines(self):
        outcome = self.outcome
        if outcome is None:
            return "Capture a scan or load a separate recording, then Localise."
        request, result = outcome.request, outcome.result
        lines = [f"{wl.METHODS[request.method]} · {request.search} search",
                 f"target: {request.target_label}"]
        if outcome.pose_day1 is not None:
            x, y, degrees = outcome.pose_day1
            lines += [f"X {x:.3f} m   Y {y:.3f} m", f"heading {degrees:.2f}°"]
        if result.score is not None:
            lines.append(f"{outcome.score_label} {result.score:.3f}"
                         + (f" (runner-up {result.runner_up_score:.3f})"
                            if result.runner_up_score is not None else ""))
        if outcome.runner_up_pose is not None:
            x, y, degrees = common.pose_to_day1(outcome.runner_up_pose)
            lines.append(f"equally good alternative ({x:.2f}, {y:.2f}, {degrees:.0f}°)")
        if outcome.pose is not None and request.method in MEASURE_LINES:
            lines += MEASURE_LINES[request.method](None, result)
        lines.append(f"time {result.runtime_ms:.0f} ms (algorithm only)")  # reason: Messages
        return "\n".join(lines)

    def draw_estimate(self):
        """Overlay artists only: ok filled, weak hollow + dashed, rejected nothing."""
        style, outcome = self.style(), self.outcome
        pose = outcome.pose if outcome is not None else None
        weak = outcome is not None and outcome.plausible
        aligned = (common.transform_points(outcome.request.points_local, pose)
                   if pose is not None else np.empty((0, 2)))
        guess = common.pose_from_day1(self.guess) if self.search_key() == "local" else None
        for points, marker, heading, guess_marker, guess_heading in self.estimate_artists.values():
            points.set_offsets(aligned)
            points.set_color(style["estimate"])
            points.set_alpha(0.45 if weak else 0.9)
            loc.set_pose_marker((marker, heading), pose, 0.45)
            marker.set_color(style["estimate"])
            marker.set_markerfacecolor("none" if weak else style["estimate"])
            heading.set_color(style["estimate"])
            heading.set_linestyle("--" if weak else "-")
            loc.set_pose_marker((guess_marker, guess_heading), guess, 0.45)
            guess_marker.set_markeredgecolor(style["initial"])
            guess_heading.set_color(style["initial"])

    def draw_global(self):
        for key in GLOBAL_VIEWS:
            self.views[key].draw()

    def tick_localisation(self):
        if self.busy and self.loc_progress is not None and self.loc_label is not None:
            shown = int(self.loc_progress * 20)  # 5 % steps: no label churn
            if shown != self.shown_progress:
                self.shown_progress = shown
                self.set_info(f"{self.loc_label} {shown * 5} %")
        if self.outcome is not None and self.inputs_key() != self.outcome_key:
            self.clear_result("Target or session changed: press Localise again.")

    def style(self):
        return PLOT_STYLES[self.plot_style.get()]

    def set_plot_style(self):
        """Recolour artists only; no map is rebuilt and no value changes."""
        style = self.style()
        for view in self.views.values():
            view.apply_style(style)
        if self.grid_image is not None:
            self.grid_image.set_cmap(style["cmap"])
        if hasattr(self, "segments"):
            self.segments.set_color(style["vector"])
        self.current_points.set_color(style["current"])
        self.axis_cross.set_color(style["axis"])
        self.axis_label.set_color(style["axis"])
        self.axis_label.arrow_patch.set_color(style["axis"])
        cmap = colormaps[style["cmap"]] if isinstance(style["cmap"], str) else style["cmap"]
        for swatch, (_, value) in zip(self.legend_swatches, LEGEND):
            fraction = (value - part2.FREE) / (part2.OCCUPIED - part2.FREE)
            swatch.configure(background=to_hex(cmap(fraction)))
        self.draw_estimate()
        for view in self.views.values():
            view.draw()

    def captured(self, points):
        self.source.set(self.SOURCES[1])
        self.refresh_selection()
        self.set_info(f"Captured {len(points)} points (sensor frame; no global pose).")

    def tick(self):
        self.tick_drive()
        if self.revision != self.model.revision:
            return  # show_page recreates stale derived views on returning
        signature = (self.model.session["selected_id"],
                     id(self.model.workspace_inputs.latest_capture), self.source.get())
        if signature != self.selection_signature:
            self.refresh_selection()
        self.tick_localisation()

    def close(self):
        self.drive.close()  # sends /stop if connected; ends the worker thread
        for job in (self.link_job, self.layout_job):
            if job is not None:
                self.frame.after_cancel(job)
        for viewport in self.views.values():
            viewport.close()
        super().close()
        # A closed page is freed by the cyclic GC, which may run on a worker
        # thread; a Tk Variable deleted there blocks it ("main thread is not in
        # main loop"). Drop the page's Variables now, on the Tk thread.
        self.pose_inputs.variables.clear()
        self.source = self.link = self.port = self.baud = None
