"""Four cached map views in the existing Tk desktop; no mapping algorithms."""
import tkinter as tk
from tkinter import ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.collections import LineCollection
from matplotlib import colormaps
from matplotlib.colors import LinearSegmentedColormap, to_hex
from matplotlib.figure import Figure

from app_session import build_grid, build_vectors, part1, part2, grid_localisation as loc
from desktop_app import (Page, GridPage, CHROME, CHROME_2, SELECT, VIEW_BG, VIEW_CAPTION,
                         VIEW_TEXT, VIEW_MUTED, RESIZE_SETTLE_MS,
                         button, expander, heading, note, row, tip, set_text)
from ui_fonts import FontSet, TechnicalLabel
from ui_theme import THEME


GLOBAL_VIEWS = ("scan", "grid", "vector")
VIEW_TITLES = {"scan": "Scan Map", "grid": "Bitmap / Occupancy Grid",
               "vector": "Vector Map", "current": "Current Scan / Live Robot"}
VIEW_HELP = {
    "scan": "Code 01 scans accumulated at their recorded alignment poses. Global frame "
            "(Scan 1), metres.",
    "grid": "Code 02 occupancy grid, global frame, metres. Display mode in Properties.",
    "vector": "Code 03 extracted and merged finite line segments. Global frame, metres.",
    "current": "Selected recorded scan or latest capture in the LiDAR's own frame: origin "
               "(0, 0), heading along +X, metres. No live pose estimate or tracking.",
}
NAV_HELP = "\n\nClick a viewport to make the tool bar act on it; double-click a title to maximise."

# Plot styles: only presentation changes. Cell VALUES are never altered; the
# dark map keeps three distinct classes ordered by salience: unknown recedes
# into the background, observed free floor is a visible graphite, occupied
# walls are the brightest element (light paper: free white, occupied black).
DARK_GRID = LinearSegmentedColormap.from_list("occupancy_dark", [
    (0.0, "#4B4D56"),                     # free (0) — observed floor
    (part2.UNKNOWN / 255, "#1A1B1F"),     # unknown (127) — background
    (1.0, "#EEE8E0"),                     # occupied (255) — walls
])
PLOT_STYLES = {
    "Dark": dict(face="#1A1B1F", tick=THEME.secondary, label=THEME.text, spine="#3F3D46",
                 grid=("#3A3940", 0.5, 1.0), cmap=DARK_GRID, vector="#E6E1DA",
                 current="#8DB4D2", empty=THEME.secondary),
    "Light paper": dict(face=THEME.paper, tick=VIEW_MUTED, label=VIEW_TEXT, spine=THEME.plot_spine,
                        grid=("#B0B0B0", 0.4, 0.35), cmap="gray_r", vector="#153D71",
                        current="#246F9D", empty="#555555"),
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
            self.frame.after_cancel(self.resize_job)
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
    SOURCES = ("Selected recorded scan", "Latest capture")

    def __init__(self, app):
        self.pending = set()
        self.capture_busy = False
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
        self.capture_buttons[0].configure(style="Primary.TButton")

        # -- rarely needed details ---------------------------------------------------
        details = expander(self.controls, "Details", tooltip="Map sizes, cache state and data notes.")
        self.map_stats = note(details, "")
        note(details, "Global maps use recorded alignment poses; markers show the selected "
                      "scan's recorded pose, not a live estimate.")
        note(details, "A Workspace capture stays separate from the global map. Use 01 Scan to "
                      "add and align scans.")
        self.global_points = self.model.cache.get("global_points")
        if self.global_points is None:
            self.global_points = part2.global_points_of_scans(self.model.scans)
            self.model.cache["global_points"] = self.global_points
        self.draw_scans()
        self.create_selection_artists()
        self.refresh_selection()
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
        self.pose_artists = {}
        for key in GLOBAL_VIEWS:
            axes = self.views[key].axes
            marker, = axes.plot([], [], "o", ms=7, mfc="none", mec="#DC7824", mew=1.7, zorder=5)
            heading, = axes.plot([], [], color="#DC7824", linewidth=1.7, zorder=5)
            self.pose_artists[key] = marker, heading
        self.highlight = self.views["scan"].axes.scatter([], [], s=3, color="#DC7824", zorder=4)
        axes = self.views["current"].axes
        self.current_points = axes.scatter([], [], s=3, color=self.style()["current"])
        axes.plot(0, 0, "+", ms=9, color="#DC7824")
        axes.annotate("+X", xy=(0.4, 0), xytext=(0.65, 0), color="#9A4D10",
                      arrowprops={"arrowstyle": "<-", "color": "#DC7824"}, fontsize=8)

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
        cmap = colormaps[style["cmap"]] if isinstance(style["cmap"], str) else style["cmap"]
        for swatch, (_, value) in zip(self.legend_swatches, LEGEND):
            fraction = (value - part2.FREE) / (part2.OCCUPIED - part2.FREE)
            swatch.configure(background=to_hex(cmap(fraction)))
        for view in self.views.values():
            view.draw()

    def captured(self, points):
        self.source.set(self.SOURCES[1])
        self.refresh_selection()
        self.set_info(f"Captured {len(points)} points (sensor frame; no global pose).")

    def tick(self):
        if self.revision != self.model.revision:
            return  # show_page recreates stale derived views on returning
        signature = (self.model.session["selected_id"],
                     id(self.model.workspace_inputs.latest_capture), self.source.get())
        if signature != self.selection_signature:
            self.refresh_selection()

    def close(self):
        for job in (self.link_job, self.layout_job):
            if job is not None:
                self.frame.after_cancel(job)
        for viewport in self.views.values():
            viewport.close()
        super().close()
