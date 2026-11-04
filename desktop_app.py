"""Single Tk event loop hosting the existing Matplotlib tools.

Numerical work and serial reads run on workers. Only the Tk queue poller calls
completion handlers, creates artists, or touches widgets. No pyplot managers
are created by embedded pages.

Layout (desktop CAD/GIS style): menu bar, tool bar, Day 1 / Day 2 page tabs;
per page a Layers dock (left), the map viewport (centre) and a Properties dock
(right, controls grouped in workflow order, messages at its foot); status bar.
"""
import argparse
import queue
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.collections import LineCollection
from matplotlib.colors import to_hex
from matplotlib.figure import Figure
from matplotlib.lines import Line2D

import localisation_common as common
from app_session import (SharedSession, CaptureAdapter, build_grid, build_vectors,
                         part1, part2, part3, grid_localisation as loc,
                         vector_localisation as vloc, scan_localisation as sloc)
from ui_fonts import FontSet, TechnicalLabel
from ui_theme import PLOT, THEME, apply_styles

# -- Theme: graphite chrome (ui_theme.THEME); purple = active, green = confirmed.
CHROME = THEME.panel              # docks and panels
CHROME_2 = THEME.background       # tool bar, tab strip, status bar
DOCK_TITLE = THEME.raised
HEADER = THEME.raised             # group headings
BORDER = THEME.border
FIELD = THEME.field
TEXT = THEME.text
MUTED = THEME.secondary
SELECT = THEME.purple             # active tool, active tab, selection
# Drawn on the light plot paper, so these stay the darker data-legible tones.
OK_COLOUR, WEAK_COLOUR, BAD_COLOUR = PLOT.verdict_ok, PLOT.verdict_weak, PLOT.verdict_bad
VIEW_BG = THEME.viewport          # viewport surround (the plot area keeps its data colours)
VIEW_CAPTION = THEME.viewport_title
VIEW_TEXT = THEME.text
VIEW_MUTED = THEME.secondary
UI_FONT = ("Segoe UI", 9)
MONO = ("Consolas", 9)
LEFT_W, RIGHT_W = 214, 318
WRAP = 266                # text width inside a Properties group
RESIZE_SETTLE_MS = 250    # redraw the map once the window stops resizing (> one Tk relayout)
PROJECT = Path(__file__).resolve().parent
ICONS = Path(matplotlib.get_data_path()) / "images"


class Tooltip:
    """Short explanation shown after hovering; created only while visible."""
    def __init__(self, widget, text, delay=450):
        self.widget, self.text, self.delay = widget, text, delay
        self.window = self.after_id = None
        widget.bind("<Enter>", self.schedule, add="+")
        widget.bind("<Leave>", self.hide, add="+")
        widget.bind("<ButtonPress>", self.hide, add="+")

    def schedule(self, _=None):
        self.hide()
        self.after_id = self.widget.after(self.delay, self.show)

    def show(self):
        self.after_id = None
        text = self.text() if callable(self.text) else self.text
        if not text or not self.widget.winfo_exists():
            return
        x = self.widget.winfo_rootx() + 10
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 3
        self.window = tk.Toplevel(self.widget)
        self.window.wm_overrideredirect(True)
        self.window.wm_geometry(f"+{x}+{y}")
        tk.Label(self.window, text=text, justify="left", wraplength=330, background=THEME.tooltip,
                 foreground=TEXT, relief="flat", borderwidth=0, highlightthickness=1,
                 highlightbackground=BORDER, font=UI_FONT, padx=7, pady=5).pack()

    def hide(self, _=None):
        if self.after_id is not None:
            self.widget.after_cancel(self.after_id)
            self.after_id = None
        if self.window is not None:
            self.window.destroy()
            self.window = None


def tip(widget, text):
    Tooltip(widget, text)
    return widget


def button(parent, text, command, style="TButton", tooltip=None, side=None):
    widget = ttk.Button(parent, text=text, command=command, style=style)
    if side:
        widget.pack(side=side, fill="x", expand=True, padx=(0, 3), pady=2)
    else:
        widget.pack(fill="x", pady=2)
    if tooltip:
        tip(widget, tooltip)
    return widget


def note(parent, text, **options):
    options.setdefault("style", "Note.TLabel")
    widget = ttk.Label(parent, text=text, wraplength=WRAP, justify="left", **options)
    widget.pack(fill="x", pady=(2, 4))
    return widget


def section(parent, text):
    """A group heading bar (Properties / Layers docks)."""
    widget = ttk.Label(parent, text=text, style="Section.TLabel")
    widget.pack(fill="x", pady=(6, 0))
    return widget


def subheading(parent, text):
    widget = ttk.Label(parent, text=text, style="Sub.TLabel")
    widget.pack(fill="x", pady=(6, 1))
    return widget


def group(parent, title, tooltip=None):
    """Heading + padded body; returns the body frame."""
    header = section(parent, title)
    if tooltip:
        tip(header, tooltip)
    body = ttk.Frame(parent, padding=(8, 4, 8, 6))
    body.pack(fill="x")
    return body


def heading(parent, text, tooltip=None):
    """Quiet section title: identity face on the panel, a hairline above.
    Lighter than section()'s filled bar; used by the simplified Workspace."""
    tk.Frame(parent, height=1, background=BORDER).pack(fill="x", padx=8, pady=(10, 0))
    fonts = FontSet.current
    if fonts is not None and fonts.tk_available:
        label = TechnicalLabel(parent, text, fonts, size=8, background=CHROME,
                               foreground=MUTED, padx=8, pady=4)
        label.pack(anchor="w")
    else:
        label = ttk.Label(parent, text=text, style="Sub.TLabel", padding=(8, 4, 8, 2))
        label.pack(fill="x")
    if tooltip:
        tip(label, tooltip)
    body = ttk.Frame(parent, padding=(8, 0, 8, 2))
    body.pack(fill="x")
    return body


def expander(parent, text, opened=False, tooltip=None):
    """A heading that shows/hides its body: rarely used details and diagnostics."""
    tk.Frame(parent, height=1, background=BORDER).pack(fill="x", padx=8, pady=(10, 0))
    state = tk.BooleanVar(master=parent, value=opened)
    toggle = ttk.Button(parent, style="Expander.TButton")
    toggle.pack(fill="x", padx=2)
    body = ttk.Frame(parent, padding=(8, 0, 8, 4))
    marker = toggle  # body is re-packed directly after the toggle

    def show():
        toggle.configure(text=("▾  " if state.get() else "▸  ") + text)
        if state.get():
            body.pack(fill="x", after=marker)
        else:
            body.pack_forget()

    toggle.configure(command=lambda: (state.set(not state.get()), show()))
    if tooltip:
        tip(toggle, tooltip)
    show()
    return body


def row(parent):
    frame = ttk.Frame(parent)
    frame.pack(fill="x", pady=1)
    return frame


def field_label(parent, text):
    widget = ttk.Label(parent, text=text, style="Field.TLabel")
    widget.pack(anchor="w", pady=(3, 0))
    return widget


def set_text(widget, text):
    """Configure a label only when its text changes (avoids relayout churn)."""
    if widget.cget("text") != text:
        widget.configure(text=text)


def verdict(result):
    """Status-bar / Result-group wording for a localisation result."""
    if result is None:
        return "No result yet", "none"
    if result.status == "ok":
        return "Localised · ok", "ok"
    if result.status == "weak":
        return "Plausible, not confirmed · weak", "weak"
    return f"Not localised · {result.status}", "bad"


VERDICT_STYLES = {"ok": "VerdictOK.TLabel", "weak": "VerdictWeak.TLabel",
                  "bad": "VerdictBad.TLabel", "none": "VerdictNone.TLabel"}


class Page:
    CAPTION = "Plan view   ·   X right, Y up   ·   metres"

    def __init__(self, app):
        self.app = app
        self.model = app.model
        self.revision = self.model.revision
        self.alive = True
        self.busy = False
        self.frame = ttk.Frame(app.workspace, style="Chrome.TFrame")
        self.frame.columnconfigure(1, weight=1)
        self.frame.rowconfigure(0, weight=1)
        # Left dock: layers / session inspector.
        self.layers_panel = ttk.Frame(self.frame, width=LEFT_W)
        self.layers_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 1))
        self.layers_panel.pack_propagate(False)
        self.dock_title(self.layers_panel, "Layers", app.toggle_layers)
        self.layer_body = ttk.Frame(self.layers_panel, padding=(6, 0, 6, 6))
        self.layer_body.pack(fill="both", expand=True)
        self.layer_anchor = None  # map-layer groups are inserted above the scan list
        self.build_layers_rail()
        # Centre: the map viewport.
        self.plot_frame = ttk.Frame(self.frame, style="View.TFrame")
        self.plot_frame.grid(row=0, column=1, sticky="nsew")
        caption = ttk.Frame(self.plot_frame, style="Caption.TFrame")
        caption.pack(side="top", fill="x")
        self.caption = ttk.Label(caption, text=self.CAPTION, style="Caption.TLabel")
        self.caption.pack(side="left")
        # Right dock: properties (scrolling) and messages (always visible).
        rail = self.rail = ttk.Frame(self.frame, width=RIGHT_W)
        rail.grid(row=0, column=2, sticky="nsew", padx=(1, 0))
        rail.pack_propagate(False)
        self.dock_title(rail, "Properties")
        messages = ttk.Frame(rail)
        messages.pack(side="bottom", fill="x")
        section(messages, "Messages")
        self.message_body = ttk.Frame(messages, padding=(8, 3, 8, 8))
        self.message_body.pack(fill="x")
        scroll = tk.Canvas(rail, bg=CHROME, highlightthickness=0, width=RIGHT_W - 14)
        bar = self.rail_bar = ttk.Scrollbar(rail, orient="vertical", command=scroll.yview)
        bar.pack(side="right", fill="y")
        scroll.pack(side="left", fill="both", expand=True)
        scroll.configure(yscrollcommand=bar.set)
        self.controls = ttk.Frame(scroll, padding=(0, 0, 0, 8))
        window = scroll.create_window((0, 0), window=self.controls, anchor="nw")
        self.controls.bind("<Configure>", lambda _: scroll.configure(scrollregion=scroll.bbox("all")))
        scroll.bind("<Configure>", lambda e: scroll.itemconfigure(window, width=e.width))
        app.scrollers[scroll] = self.controls
        self.build_viewport()
        # Lay the page out before builders draw: the canvas then has its final
        # size, so its first (100–200 ms) draw is not repeated after a resize.
        self.frame.pack(fill="both", expand=True)
        self.frame.update_idletasks()
        self.info = None
        self.capture_adapter = None
        self.placeholder = None
        self.locked = []
        self.swatches = {}
        self.tree_signature = None
        self.build_scan_list()

    def build_viewport(self):
        """Single map by default; Workspace supplies four independent canvases."""
        self.figure = Figure(figsize=(9, 7), facecolor=VIEW_BG)
        # Attach BEFORE invoking builders: their widget callbacks and timers
        # belong to this actual Tk canvas, never an interim pyplot window.
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.plot_frame)
        # Matplotlib's navigation (pan, zoom, home, back, forward, save) is
        # driven from the application tool bar and View menu; its own bar
        # stays unpacked so the map gets the full height.
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.plot_frame, pack_toolbar=False)
        widget = self.canvas.get_tk_widget()
        widget.configure(background=VIEW_BG)
        widget.pack(fill="both", expand=True)
        # Resizing: Matplotlib redraws the whole figure on EVERY <Configure>.
        # The first size is applied at once; later ones once resizing pauses.
        self.resize_job = None
        self.sized = False
        widget.bind("<Configure>", self.on_configure)
        self.map_axes = None
        self.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.canvas.mpl_connect("figure_leave_event", lambda _: self.app.show_cursor(None))

    # -- docks -------------------------------------------------------------------
    @staticmethod
    def dock_title(parent, text, collapse=None):
        bar = ttk.Frame(parent, style="DockTitle.TFrame")
        bar.pack(side="top", fill="x")
        fonts = FontSet.current
        if fonts is not None and fonts.tk_available:
            TechnicalLabel(bar, text, fonts, size=fonts.dock_title[1], background=DOCK_TITLE,
                           foreground=MUTED, padx=8, pady=4).pack(side="left")
        else:
            ttk.Label(bar, text=text, style="DockTitle.TLabel").pack(side="left")
        if collapse is not None:
            tip(ttk.Button(bar, text="«", width=2, style="DockButton.TButton", command=collapse),
                "Hide the Layers panel (View ▸ Layers Panel shows it again).").pack(side="right", padx=2)

    def action_footer(self):
        """A strip pinned above Messages: the page's main action never scrolls away."""
        footer = ttk.Frame(self.rail, style="Footer.TFrame", padding=(8, 5, 8, 7))
        footer.pack(side="bottom", fill="x", before=self.rail_bar)
        tk.Frame(footer, height=1, background=BORDER).pack(fill="x", pady=(0, 5))
        return footer

    def page_header(self, title, subtitle, tooltip=None):
        fonts = FontSet.current
        if fonts is not None and fonts.tk_available:
            label = TechnicalLabel(self.controls, title, fonts, size=fonts.page_title[1],
                                   background=CHROME, foreground=TEXT, padx=0, pady=2)
            label.pack(anchor="w", padx=8, pady=(8, 0))
        else:
            label = ttk.Label(self.controls, text=title, style="PageTitle.TLabel")
            label.pack(fill="x", padx=8, pady=(6, 0))
        if tooltip:
            tip(label, tooltip)
        if not subtitle:
            return
        ttk.Label(self.controls, text=subtitle, style="Note.TLabel", wraplength=RIGHT_W - 34,
                  justify="left").pack(fill="x", padx=8, pady=(0, 2))

    def layer_group(self, title):
        """A group in the Layers dock, above the session scan list."""
        header = section(self.layer_body, title)
        body = ttk.Frame(self.layer_body, padding=(4, 2, 2, 4))
        header.pack_configure(before=self.layer_anchor)
        body.pack(fill="x", before=self.layer_anchor)
        return body

    INTERACTIVE_SCANS = False
    SCAN_LIST_NOTE = True
    SCAN_LIST_HINT = ("Read-only here. Show, hide or move scans on 01 Scan; this page's map "
                      "is rebuilt when you come back.")

    def build_scan_list(self):
        """Session scans with their data colour and visibility."""
        self.layer_anchor = section(self.layer_body, "Session scans")
        frame = ttk.Frame(self.layer_body)
        frame.pack(fill="both", expand=True, pady=(2, 0))
        self.scan_tree = ttk.Treeview(frame, columns=("visible",), show="tree headings",
                                      selectmode="browse" if self.INTERACTIVE_SCANS else "none")
        self.scan_tree.heading("#0", text="Scan", anchor="w")
        self.scan_tree.column("#0", width=120, stretch=True)
        self.scan_tree.heading("visible", text="Visible")
        self.scan_tree.column("visible", width=52, stretch=False, anchor="center")
        self.scan_tree.tag_configure("hidden", foreground=THEME.disabled)
        bar = ttk.Scrollbar(frame, command=self.scan_tree.yview)
        self.scan_tree.configure(yscrollcommand=bar.set)
        bar.pack(side="right", fill="y")
        self.scan_tree.pack(side="left", fill="both", expand=True)
        if self.SCAN_LIST_NOTE:
            self.scan_list_note = note(self.layer_body, self.SCAN_LIST_HINT)
            self.scan_list_note.configure(wraplength=LEFT_W - 16)
        else:  # simplified pages: the same guidance on hover
            tip(self.scan_tree, self.SCAN_LIST_HINT)
        self.refresh_scan_tree()

    def swatch(self, colour):
        try:
            colour = to_hex(colour)
        except (ValueError, TypeError):
            colour = THEME.secondary
        if colour not in self.swatches:
            image = tk.PhotoImage(master=self.frame, width=11, height=11)
            image.put(THEME.border, to=(0, 0, 11, 11))
            image.put(colour, to=(1, 1, 10, 10))
            self.swatches[colour] = image
        return self.swatches[colour]

    def refresh_scan_tree(self):
        scans = self.model.session["scans"]
        selected = self.model.session["selected_id"]
        signature = (tuple((s["id"], s["label"], s["visible"], str(s["color"])) for s in scans),
                     selected if self.INTERACTIVE_SCANS else None)
        if signature == self.tree_signature:
            return
        self.tree_signature = signature
        tree = self.scan_tree
        tree.delete(*tree.get_children())
        for scan in scans:
            tree.insert("", "end", iid=str(scan["id"]), text=f" {scan['label']}",
                        image=self.swatch(scan["color"]),
                        values=("☑" if scan["visible"] else "☐",),
                        tags=() if scan["visible"] else ("hidden",))
        if self.INTERACTIVE_SCANS and selected is not None and tree.exists(str(selected)):
            tree.selection_set(str(selected))
            tree.see(str(selected))

    def set_layers_visible(self, visible):
        if visible:
            self.layers_rail.grid_remove()
            self.layers_panel.grid()
        else:
            self.layers_panel.grid_remove()
            self.layers_rail.grid()

    def build_layers_rail(self):
        """Collapsed Layers panel: a narrow strip that reopens it on click."""
        rail = self.layers_rail = tk.Canvas(self.frame, width=22, background=DOCK_TITLE,
                                            highlightthickness=0, cursor="hand2")
        rail.grid(row=0, column=0, sticky="ns", padx=(0, 1))
        rail.create_text(11, 10, text="»", fill=MUTED, font=("Segoe UI", 10), anchor="n")
        fonts = FontSet.current
        family = ("Engineering Plot", 10) if fonts is not None and fonts.tk_available else ("Segoe UI", 9)
        rail.create_text(11, 34, text="LAYERS", angle=90, fill=MUTED, font=family, anchor="e")
        rail.bind("<Button-1>", lambda _: self.app.toggle_layers())
        tip(rail, "Show the Layers panel (scan list and visibility).")
        rail.grid_remove()

    # -- viewport ----------------------------------------------------------------
    def on_configure(self, event):
        if self.resize_job is not None:
            event.widget.after_cancel(self.resize_job)
            self.resize_job = None
        if not self.sized:
            self.sized = True
            self.canvas.resize(event)
            return
        self.resize_job = event.widget.after(RESIZE_SETTLE_MS, lambda: self.apply_resize(event))

    def apply_resize(self, event):
        self.resize_job = None
        if self.alive:
            self.canvas.resize(event)

    def on_motion(self, event):
        if self.map_axes is not None and event.inaxes is self.map_axes and event.xdata is not None:
            self.app.show_cursor((event.xdata, event.ydata))
        else:
            self.app.show_cursor(None)

    def fit_map(self):
        """Fit Map to Window: the page's own fit where it has one, else the
        Matplotlib home view (the map's initial extent)."""
        self.toolbar.home()

    def localisation_status(self):
        return None

    def prepare_plot(self, axes, compact=True):
        self.map_axes = axes
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
            axes.set_position([0.08, 0.07, 0.89, 0.88])
        for ax in self.figure.axes:
            if not ax.get_visible():
                continue
            ax.set_facecolor(THEME.paper)
            ax.tick_params(colors=VIEW_MUTED, labelsize=8)
            ax.xaxis.label.set_color(VIEW_TEXT)
            ax.yaxis.label.set_color(VIEW_TEXT)
            ax.title.set_color(VIEW_TEXT)
            ax.title.set_fontsize(10)
            for spine in ax.spines.values():
                spine.set_color(THEME.plot_spine)
        self.canvas.draw_idle()

    # -- messages and busy state -----------------------------------------------------
    def set_info(self, text):
        if self.info is None:
            self.info = note(self.message_body, text, style="Message.TLabel")
            self.info.configure(wraplength=RIGHT_W - 30)
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
                                                    color=VIEW_MUTED, fontsize=13)
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

    # -- LD06 capture ------------------------------------------------------------------
    def capture_controls(self, done, parent=None, heading=True, explain=True):
        parent = parent if parent is not None else self.controls
        if heading:
            subheading(parent, "LD06 serial capture")
        line = row(parent)
        self.port = tk.StringVar(value=part1.SERIAL_PORT)
        self.baud = tk.StringVar(value=str(part1.CAPTURE_BAUD_RATE))
        ttk.Label(line, text="Port").pack(side="left")
        tip(ttk.Entry(line, textvariable=self.port, width=8, font=MONO),
            "Serial port of the ESP32 (e.g. COM5).").pack(side="left", padx=(4, 10))
        ttk.Label(line, text="Baud").pack(side="left")
        tip(ttk.Entry(line, textvariable=self.baud, width=8, font=MONO),
            "Serial speed set in the ESP32 firmware.").pack(side="left", padx=4)
        actions = row(parent)
        self.capture_done = done
        capture = button(actions, "Capture New Scan", self.start_capture, side="left",
                         tooltip="Reads ONE fresh LD06 scan from the ESP32 (read-only: nothing is "
                                 "sent to the device). Keep the LiDAR still. Accepts ESP32 "
                                 f"Teleplot XY or CSV; {part1.CAPTURE_TIMEOUT_S:.0f} s timeout.")
        self.capture_buttons = [capture]
        button(actions, "Cancel Capture", self.cancel_capture, side="left",
               tooltip="Stops waiting for a scan and closes the serial port.")
        if explain:
            note(parent, f"Keep the LiDAR still. Accepts ESP32 Teleplot XY or CSV; "
                         f"{part1.CAPTURE_TIMEOUT_S:.0f} s timeout.")
        return parent

    def start_capture(self):
        self.capture(self.capture_done)

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

        def captured(points):
            self.model.workspace_inputs.publish_capture(points)
            done(points)

        self.run("Waiting for a fresh scan…", lambda: self.capture_adapter.capture(port, baud), captured,
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
        if self.resize_job is not None:
            self.canvas.get_tk_widget().after_cancel(self.resize_job)
            self.resize_job = None
        self.app.scrollers = {c: f for c, f in self.app.scrollers.items() if f is not self.controls}
        # Let any pending draw finish before removing its Tk widget. No private
        # Matplotlib manager/canvas attributes are accessed.
        self.canvas.flush_events()
        self.frame.destroy()


class PoseInputs:
    def __init__(self, parent, get_pose, set_pose, apply_text="Set Pose", tooltip=None):
        self.get_pose, self.set_pose = get_pose, set_pose
        self.variables, self.entries = [], []
        line = ttk.Frame(parent)
        line.pack(fill="x", pady=(2, 2))
        for i, label in enumerate(("X (m)", "Y (m)", "Heading (°)")):
            col = ttk.Frame(line)
            col.pack(side="left", fill="x", expand=True)
            ttk.Label(col, text=label, style="Field.TLabel").pack(anchor="w")
            var = tk.StringVar()
            entry = ttk.Entry(col, textvariable=var, width=9, font=MONO, justify="right")
            entry.pack(fill="x", padx=(0, 4))
            entry.bind("<Return>", lambda _: self.apply())
            self.variables.append(var)
            self.entries.append(entry)
        self.apply_button = button(parent, apply_text, self.apply,
                                   tooltip=tooltip or "Apply the typed X, Y and heading (Enter also applies).")
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
        # Compare Tk path names, not focus_get(): while a combobox drop-down is
        # open, focus is on Tk's internal popdown listbox, which has no tkinter
        # object, and focus_get() raises KeyError('popdown'). Called from the
        # host poller, that exception stopped all result delivery.
        if str(self.entries[0].tk.call("focus")) in {str(entry) for entry in self.entries}:
            return
        for var, value in zip(self.variables, self.get_pose()):
            var.set(f"{value:.3f}")


class ScannerPage(Page):
    INTERACTIVE_SCANS = True
    SCAN_LIST_HINT = ("Click a row to select that scan. Click its ☑ / ☐ to hide or show it: "
                      "hidden scans are left out of every map.")

    def __init__(self, app):
        super().__init__(app)
        _, self.editor = part1.create_session_editor(self.model.session, figure=self.figure)
        self.editor["timer"].stop()  # the host's single poller owns polling
        self.editor["display_selector"].disconnect_events()
        # Hidden filename box: Matplotlib redraws the whole figure synchronously
        # on every resize for it. The host has its own Open/Save buttons.
        self.editor["file_box"].disconnect_events()
        self.prepare_plot(self.editor["axes"])
        self.scan_tree.bind("<ButtonRelease-1>", self.on_tree_click)
        tip(button(self.layer_body, "Show All Scans", lambda: self.action("show_all")),
            "Make every scan in the session visible again.").pack_configure(before=self.layer_anchor)
        self.page_header("LiDAR Scanner", "Code 01 · capture, position and align scans. Edits stay "
                                          "in memory across pages; File ▸ Save Working Copy keeps them.")
        selected = group(self.controls, "Selected scan")
        self.scan_choice = ttk.Combobox(selected, state="readonly")
        self.scan_choice.pack(fill="x", pady=(0, 2))
        self.scan_choice.bind("<<ComboboxSelected>>", self.select)
        self.pose = PoseInputs(selected, self.get_pose, self.set_pose, "Apply Pose to Selected Scan",
                               "Place the selected scan at this global X, Y (metres) and heading "
                               "(degrees). Enter also applies.")
        line = row(selected)
        button(line, "Hide / Show", lambda: self.action("toggle_visibility"), side="left",
               tooltip="Toggle the selected scan's visibility. Hidden scans are left out of the "
                       "maps and of the Auto-Align reference.")
        button(line, "Reset Pose", lambda: self.action("reset_pose"), side="left",
               tooltip="Set the selected scan's pose back to (0, 0, 0°). Its points are unchanged.")
        align = group(self.controls, "Alignment",
                      "Code 01 automatic alignment: correlative search + ICP.")
        button(align, "Auto-Align Selected Scan", lambda: self.action("auto_align"),
               style="Primary.TButton",
               tooltip="Matches the selected scan to the reference scan and the scans already "
                       "accepted (visible ones only). The answer is drawn in black as a PROPOSAL; "
                       "nothing changes until you apply it.")
        field_label(align, "Proposed alignment (drawn in black)")
        line = row(align)
        button(line, "Apply", lambda: self.action("accept"), side="left",
               tooltip="Apply Proposed Alignment: move the selected scan to the proposed pose and "
                       "mark it as aligned.")
        button(line, "Discard", lambda: self.action("reject"), side="left",
               tooltip="Discard Proposed Alignment: the scan keeps its current pose.")
        display = group(self.controls, "Display")
        field_label(display, "Scan drawing")
        self.display = ttk.Combobox(display, state="readonly", values=part1.DISPLAY_MODES)
        self.display.set(part1.DEFAULT_DISPLAY_MODE)
        self.display.pack(fill="x")
        self.display.bind("<<ComboboxSelected>>", lambda _: self.action("display_mode", self.display.get()))
        tip(self.display, "Points: measured impacts. Rays: lines from each scan's LiDAR position "
                          "to its points (show what each scan saw through).")
        line = row(display)
        ttk.Label(line, text="Ray opacity", style="Field.TLabel").pack(side="left")
        self.opacity_value = ttk.Label(line, text=f"{part1.DEFAULT_RAY_OPACITY:.2f}", style="Mono.TLabel")
        self.opacity_value.pack(side="right")
        ttk.Scale(display, from_=0, to=0.5, value=part1.DEFAULT_RAY_OPACITY,
                  command=self.set_opacity).pack(fill="x")
        capture = group(self.controls, "Capture (LD06)")
        self.capture_controls(self.add_capture, parent=capture, heading=False)
        self.capture_buttons.append(
            button(capture, "Capture Replacement for Selected Scan", self.capture_replacement,
                   tooltip="Replace the selected scan's points with a fresh capture; its pose is kept."))
        danger = group(self.controls, "Delete")
        button(danger, "Delete Selected Scan (click twice)", lambda: self.action("delete"),
               style="Danger.TButton",
               tooltip="Removes the selected scan from the working session. The first click asks "
                       "for confirmation in Messages; click again to delete. Files on disk are not "
                       "changed until you save a copy.")
        self.set_info("Ready")
        self.refresh_selection()

    def set_opacity(self, value):
        self.opacity_value.configure(text=f"{float(value):.2f}")
        self.action("ray_opacity", float(value))

    def on_tree_click(self, event):
        item = self.scan_tree.identify_row(event.y)
        if not item:
            return
        self.action("select", int(item))
        if self.scan_tree.identify_column(event.x) == "#1":  # the Visible column
            self.action("toggle_visibility")

    def fit_map(self):
        self.action("fit_view")

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
        self.refresh_scan_tree()
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
        self.refresh_scan_tree()
        if self.editor["is_aligning"]() or self.editor["proposal"]() is not None:
            self.set_info(self.editor["status_text"].get_text())

    def close(self):
        self.editor["timer"].stop()
        super().close()


class GridPage(Page):
    MODES = ("Probability", "Classified", "Binary", "Empty")
    MODE_HELP = {
        "Probability": "Log-odds occupancy probability of each cell: white free (p → 0), "
                       "black occupied (p → 1), mid-grey unknown (p = 0.5).",
        "Classified": f"The probability map in three classes: occupied p ≥ {part2.P_OCCUPIED_ABOVE}, "
                      f"free p ≤ {part2.P_FREE_BELOW}, everything else unknown.",
        "Binary": "Code 02 basic update, no probabilities: cells crossed by any beam are free, "
                  "impact cells occupied (an impact always wins), unreached cells unknown. "
                  "Built on first use, then cached.",
        "Empty": "The grid before any scan is added: every cell unknown (127). Shows the grid "
                 "extent and its cells."}

    def __init__(self, app):
        super().__init__(app)
        self.page_header("Grid Mapping", "Code 02 · occupancy grid from all visible scans · "
                                         f"{part2.CELL_SIZE_M:.2f} m cells · global Y upward")
        display = group(self.controls, "Display")
        field_label(display, "Grid display mode")
        self.view = ttk.Combobox(display, state="readonly", values=self.MODES)
        self.view.set("Probability")
        self.view.pack(fill="x")
        self.view.bind("<<ComboboxSelected>>", lambda _: self.draw_map())
        tip(self.view, "\n\n".join(f"{mode}: {text}" for mode, text in self.MODE_HELP.items()))
        self.mode_help = note(display, self.MODE_HELP["Probability"])
        self.points = tk.BooleanVar(value=True)
        layers = self.layer_group("Map layers")
        tip(ttk.Checkbutton(layers, text="Measured LiDAR points", variable=self.points,
                            command=self.draw_map),
            "Draw each scan's points (in its data colour) over the grid.").pack(anchor="w")
        note(layers, "Grid and scan poses are always drawn.").configure(wraplength=LEFT_W - 20)
        thresholds = group(self.controls, "Classification thresholds")
        note(thresholds, f"Occupied: p ≥ {part2.P_OCCUPIED_ABOVE}\nFree: p ≤ {part2.P_FREE_BELOW}\n"
                         "Unknown cells remain separate.", style="Mono.TLabel")
        stats = group(self.controls, "Map information")
        self.map_stats = note(stats, "Building…", style="Mono.TLabel")
        self.set_info("Building map…")
        app.derived(self, "grid", build_grid, self.ready)

    def ready(self, data):
        self.grid, self.log_odds = data
        self.draw_map()
        left, right, bottom, top = self.grid.extent
        self.map_stats.configure(
            text=f"Scans in map  {len(self.model.scans)}\nGrid size     {self.grid.width} × "
                 f"{self.grid.height} cells\nCell size     {part2.CELL_SIZE_M:.2f} m\n"
                 f"X extent      {left:.1f} … {right:.1f} m\nY extent      {bottom:.1f} … {top:.1f} m")
        self.set_info(f"{len(self.model.scans)} visible scans\n{self.grid.width} × {self.grid.height} cells\nMap cached for this working session.")

    def fit_map(self):
        self.fit()

    def draw_map(self):
        if not hasattr(self, "grid"):
            return
        spec = self.grid.spec()
        arrays = part2.global_points_of_scans(self.model.scans)
        kind = self.view.get()
        self.mode_help.configure(text=self.MODE_HELP.get(kind, ""))
        if kind == "Classified":
            grid = np.full(self.grid.state.shape, part2.UNKNOWN, dtype=np.uint8)
            grid[self.grid.state == loc.FREE_STATE] = part2.FREE
            grid[self.grid.state == loc.OCCUPIED_STATE] = part2.OCCUPIED
        elif kind == "Binary":
            if "binary" not in self.model.cache:
                # Rendering/feedback fix (separate from the visual redesign; see
                # the Binary investigation in the redesign report). The build is
                # ~0.2 s on a worker and is shown on completion by binary_ready().
                # Previously the old map stayed on screen with no sign of the
                # pending build, and a repeated request while busy was dropped
                # silently, which looked like a freeze. Grid maths is untouched.
                if self.busy:
                    self.set_info("Binary grid is still being built; it is shown as soon as it is ready.")
                    return
                self.app.derived(self, "binary", lambda scans: part2.build_binary_occupancy_grid(
                    spec, scans, arrays)[0], self.binary_ready)
                # Feedback in the Tk caption bar: no Matplotlib redraw competes
                # with the worker (a title change cost a ~250 ms stall).
                self.caption.configure(text=f"{self.CAPTION}   ·   Building binary grid… "
                                            "(previous display still shown)")
                return
            grid = self.model.cache["binary"]
        elif kind == "Empty":
            grid = part2.create_occupancy_grid(spec)
        else:
            grid = part2.probabilities_to_uint8(self.grid.probability)
        self.caption.configure(text=f"{self.CAPTION}   ·   {kind} grid")
        self.figure.clear()
        part2.show_grid(grid, spec, self.model.scans, arrays,
                       title=f"{kind} grid · {self.model.name}", show_points=self.points.get(),
                       colorbar_labels=part2.PROBABILITY_COLORBAR_LABELS if kind == "Probability"
                       else part2.BINARY_COLORBAR_LABELS, figure=self.figure)
        self.figure.subplots_adjust(left=0.07, right=0.93, bottom=0.07, top=0.95)
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
        self.page_header("Vector Mapping", "Code 03 · line segments extracted from the visible scans")
        pipeline = group(self.controls, "Pipeline")
        note(pipeline, "Scan segmentation → PCA line fit → Douglas–Peucker split → recovered "
                       "segments → merged lines. Geometry uses the existing Code 03 pipeline.")
        stats = group(self.controls, "Map information")
        self.map_stats = note(stats, "Building…", style="Mono.TLabel")
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
        layers = self.layer_group("Vector layers")
        self.layers = {}
        for key in part3.LAYER_KEYS:
            variable = tk.BooleanVar(value=key in self.viewer["visible_layers"]())
            self.layers[key] = variable
            ttk.Checkbutton(layers, text=self.viewer["labels"][key], variable=variable,
                            command=lambda k=key, v=variable: self.viewer["set_layer"](k, v.get())).pack(anchor="w", pady=1)
        line = row(layers)
        button(line, "Show All", lambda: self.all_layers(True), side="left",
               tooltip="Show every vector layer.")
        button(line, "Hide All", lambda: self.all_layers(False), side="left",
               tooltip="Hide every vector layer.")
        self.map_stats.configure(text=f"Merged segments  {len(merged)}\nScans used       {len(scans)}")
        self.set_info(f"{len(merged)} merged segments\nGeometry uses the existing Code 03 pipeline.")

    def fit_map(self):
        if hasattr(self, "viewer"):
            self.viewer["reset_view"]()

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
    "at_search_limit": "Not confirmed: the best pose is on the local search edge. Press 'Use Result "
                       "as Starting Pose' and Localise again, or use the global search.",
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

SEARCH_TIP = ("Global search: needs no starting pose; every plausible position and every heading "
              "is tried, then ambiguity is checked. Slower (seconds).\n\n"
              "Local search: refines a starting pose you give (within ±1 m and ±30°). Faster, but "
              "fails if the starting pose is too far off.")
CLICK_HELP = "On the map: left-click = position, right-click = point the heading toward the cursor."


def run_footer(page, command, tooltip):
    """Pinned footer: verdict chip, Localise (primary) and Cancel Search.

    Returns (Localise button, its row frame)."""
    footer = page.action_footer()
    top = ttk.Frame(footer, style="Footer.TFrame")
    top.pack(fill="x")
    ttk.Label(top, text="Run", style="FooterTitle.TLabel").pack(side="left")
    page.verdict_chip = ttk.Label(top, text="No result yet", style=VERDICT_STYLES["none"])
    page.verdict_chip.pack(side="right")
    tip(page.verdict_chip, "Verdict of the last localisation on this page. Only 'ok' is a position "
                           "fix; other statuses are drawn faded on the map.")
    line = ttk.Frame(footer, style="Footer.TFrame")
    line.pack(fill="x", pady=(5, 0))
    run = ttk.Button(line, text="Localise", command=command, style="Primary.TButton")
    run.pack(side="left", fill="x", expand=True)
    tip(run, tooltip)
    tip(ttk.Button(line, text="Cancel Search", command=page.cancel_search),
        "Stop a running search; nothing is changed.").pack(side="left", padx=(4, 0))
    return run, line


def show_verdict_chip(page, result):
    text, level = verdict(result)
    page.verdict_chip.configure(text=text, style=VERDICT_STYLES[level])


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
        self.page_header("Grid Localisation", "Code 04 · a scan matched to the fixed Code 02 "
                                              "occupancy grid")
        task = group(self.controls, "Task")
        note(task, "Localise a separate new scan, or evaluate the session's own scans "
                   "(each left out of its reference map).")
        self.mode = ttk.Combobox(task, state="readonly", values=self.MODES)
        self.mode.set(self.MODES[0])
        self.mode.pack(fill="x", pady=(0, 2))
        self.mode.bind("<<ComboboxSelected>>", self.change_mode)
        tip(self.mode, "Localise New Scan: capture or load a scan that is NOT part of this map.\n\n"
                       "Evaluate Recorded Scans: localise each session scan on a map built from "
                       "the others and compare with its Day 1 pose estimate.")
        self.new_controls = ttk.Frame(task)
        self.new_controls.pack(fill="x")
        self.capture_controls(self.captured, parent=self.new_controls)
        subheading(self.new_controls, "Recorded scan")
        button(self.new_controls, "Load Recorded Scan…", self.choose_recorded,
               tooltip="Open a session JSON and use one of its scans as the new scan. Its saved "
                       "pose is ignored.")
        field_label(self.new_controls, "Scan from the loaded recording")
        self.target_choice = ttk.Combobox(self.new_controls, state="readonly")
        self.target_choice.pack(fill="x", pady=(0, 2))
        self.target_choice.bind("<<ComboboxSelected>>", self.choose_target)
        self.eval_controls = ttk.Frame(task)
        field_label(self.eval_controls, "Session scan to evaluate")
        self.scan_choice = ttk.Combobox(self.eval_controls, state="readonly")
        self.scan_choice["values"] = [f"{s['id']} — {s['name']}" for s in self.model.scans]
        self.scan_choice.pack(fill="x", pady=(0, 2))
        self.scan_choice.bind("<<ComboboxSelected>>", self.select_reference_scan)
        self.exclude = tk.BooleanVar(value=True)
        tip(ttk.Checkbutton(self.eval_controls, text="Leave this scan out of the reference map",
                            variable=self.exclude, command=self.toggle_exclude),
            "Leave-one-out: the reference map is rebuilt from the OTHER scans, so the scan is "
            "never matched against its own measurements.").pack(anchor="w", pady=3)
        line = row(self.eval_controls)
        button(line, "Day 1 Pose", self.day1_pose, side="left",
               tooltip="Start From Day 1 Pose Estimate: set the starting pose to this scan's saved "
                       "Day 1 pose.")
        button(line, "Perturbed Guess", self.new_guess, side="left",
               tooltip="New Perturbed Guess: a reproducible starting pose about 0.3 m / 10° away "
                       "from the Day 1 pose.")
        button(self.eval_controls, "Evaluate All Scans → CSV + JSON…", self.evaluate,
               tooltip="Runs every scan from several starting guesses (several minutes) and "
                       "exports the results to a folder you choose. Navigation stays available.")
        note(self.eval_controls, "Agreement with Day 1 estimates is not physical ground-truth accuracy.")
        search = group(self.controls, "Search", SEARCH_TIP)
        self.search = ttk.Combobox(search, state="readonly", values=self.SEARCHES)
        self.search.set(self.SEARCHES[0])
        self.search.pack(fill="x", pady=(0, 2))
        self.search.bind("<<ComboboxSelected>>", self.change_search)
        tip(self.search, SEARCH_TIP)
        self.search_note = note(search, self.SEARCH_NOTES[0])
        self.local_controls = ttk.Frame(search)
        subheading(self.local_controls, "Starting pose (local search)")
        self.pose = PoseInputs(self.local_controls, self.get_pose, self.set_pose, "Set Starting Pose",
                               "Use the typed X, Y and heading as the local search's starting guess.")
        note(self.local_controls, CLICK_HELP)
        button(self.local_controls, "Use Result as Starting Pose", self.use_result,
               tooltip="Copy the last localised pose into the starting pose, e.g. to search again "
                       "around it.")
        self.align_button, _ = run_footer(
            self, self.align, "Run the selected search. The reference map never changes.")
        results = group(self.controls, "Result")
        self.result_label = note(results, "Load a separate scan or capture one to begin.",
                                 style="Mono.TLabel")
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

    def localisation_status(self):
        return verdict(self.viewer.result if self.viewer is not None else None)

    def run_localisation(self):
        self.align()

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
            self.verdict.set_color(OK_COLOUR)
        else:
            self.verdict.set_text(f"NOT localised ({result.status}): faded pose is not an answer"
                                  + ("; purple = equally good alternative"
                                     if result.status == "ambiguous" and runner else ""))
            self.verdict.set_color(BAD_COLOUR)
        show_verdict_chip(self, result)

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
        self.page_header(self.TITLE, self.DESCRIPTION)
        source = group(self.controls, "New scan (input)")
        self.capture_controls(self.captured, parent=source)
        subheading(source, "Recorded scan")
        button(source, "Load Recorded Scan…", self.choose_recorded,
               tooltip="Open a session JSON and use one of its scans as the new scan. Its saved "
                       "pose is ignored; scans already in the reference are refused.")
        field_label(source, "Scan from the loaded recording")
        self.target_choice = ttk.Combobox(source, state="readonly")
        self.target_choice.pack(fill="x", pady=(0, 2))
        self.target_choice.bind("<<ComboboxSelected>>", self.choose_target)
        self.reference_body = group(self.controls, "Reference")
        self.reference_controls(self.reference_body)
        ttk.Label(self.controls, text="Coordinate reference: Scan 1 (0, 0, 0°)",
                  style="Coord.TLabel").pack(fill="x", padx=8, pady=(0, 4))
        search = group(self.controls, "Search", SEARCH_TIP)
        self.search = ttk.Combobox(search, state="readonly", values=self.SEARCHES)
        self.search.set(self.SEARCHES[0])
        self.search.pack(fill="x", pady=(0, 2))
        self.search.bind("<<ComboboxSelected>>", self.change_search)
        tip(self.search, SEARCH_TIP)
        self.search_note = note(search, self.SEARCH_NOTES[0])
        self.local_controls = ttk.Frame(search)
        subheading(self.local_controls, "Starting pose (local search)")
        self.pose = PoseInputs(self.local_controls, lambda: vloc.pose_to_day1(self.guess),
                               self.set_pose, "Set Starting Pose",
                               "Use the typed X, Y and heading as the local search's starting guess.")
        note(self.local_controls, CLICK_HELP)
        self.localise_button, self.action_row = run_footer(
            self, self.localise, f"Run the selected search on the {self.REFERENCE_NAME}. The "
                                 "reference never changes.")
        results = group(self.controls, "Result")
        self.result_label = note(results, "Load a separate scan or capture one to begin.",
                                 style="Mono.TLabel")

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

    def localisation_status(self):
        return verdict(self.result)

    def run_localisation(self):
        self.localise()

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
        self.figure.subplots_adjust(left=0.07, right=0.98, bottom=0.07, top=0.95)
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
            self.local_controls.pack(fill="x", after=self.search_note)
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
            self.verdict.set_color(OK_COLOUR)
        else:
            self.verdict.set_text(f"NOT localised ({result.status}): faded pose is not an answer"
                                  + ("; purple = equally good alternative"
                                     if result.status == "ambiguous" and runner else ""))
            self.verdict.set_color(BAD_COLOUR)
        self.axes.set_title(self.target[1] if self.target is not None else
                            f"{self.REFERENCE_NAME.capitalize()} · capture or load a scan",
                            fontsize=10, color=VIEW_TEXT)
        show_verdict_chip(self, result)
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

    def reference_controls(self, parent):
        self.reference_summary = note(parent, "Code 03 vector map of the visible scans (fixed). "
                                              "Extracting…")

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
        self.reference_summary.configure(text=f"Code 03 vector map of the visible scans (fixed): "
                                              f"{len(vmap.starts)} segments.")
        self.set_info(f"Vector map ready: {len(vmap.starts)} segments. Load a separate scan "
                      "or capture a fresh one.")

    def draw_reference(self, ax):
        vmap = self.vmap
        ax.add_collection(LineCollection(np.stack([vmap.starts, vmap.ends], axis=1),
                                         colors=PLOT.vector_reference, linewidths=2.2, zorder=2))
        left, right, bottom, top = vmap.segment_extent
        return ([Line2D([], [], color=PLOT.vector_reference, lw=2.2,
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
        field_label(parent, "Reference mode")
        self.reference_mode = ttk.Combobox(parent, state="readonly", values=self.REFERENCE_MODES)
        self.reference_mode.set(self.REFERENCE_MODES[0])
        self.reference_mode.pack(fill="x", pady=(0, 2))
        self.reference_mode.bind("<<ComboboxSelected>>", self.change_reference)
        tip(self.reference_mode, "Single scan: match the new scan to ONE recorded scan (needs real "
                                 "overlap with it).\n\nAccumulated scans: match it to the points of "
                                 "all positioned scans pooled together.")
        self.reference_choice = ttk.Combobox(parent, state="readonly")
        self.reference_choice.pack(fill="x", pady=(0, 2))
        self.reference_choice.bind("<<ComboboxSelected>>", self.change_reference)
        tip(self.reference_choice, "The recorded scan used as the single reference, at its saved pose.")
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
            self.reference_choice.pack(fill="x", pady=(0, 2), before=self.reference_note)
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


BUILD_LABELS = {"grid": "Building occupancy grid…", "vectors": "Extracting vector map…",
                "binary": "Building binary grid…"}


def create_workspace_page(app):
    # Lazy import keeps the additional UI separate from the six original pages.
    from workspace_page import WorkspacePage
    return WorkspacePage(app)

# Add future pages here when their algorithms are implemented.
PAGES = {"LiDAR Scanner": ScannerPage, "Grid Mapping": GridPage,
         "Vector Mapping": VectorPage, "Grid Localisation": LocalisationPage,
         "Vector Localisation": VectorLocalisationPage,
         "Scan Localisation": ScanLocalisationPage, "Workspace": create_workspace_page}
TAB_LABELS = {"LiDAR Scanner": "01  Scan", "Grid Mapping": "02  Grid", "Vector Mapping": "03  Vector",
              "Grid Localisation": "04  Grid Localisation",
              "Vector Localisation": "05  Vector Localisation",
              "Scan Localisation": "06  Scan Localisation", "Workspace": "Workspace"}
DAYS = (("DAY 1 · MAPPING", ("LiDAR Scanner", "Grid Mapping", "Vector Mapping")),
        ("DAY 2 · LOCALISATION", ("Grid Localisation", "Vector Localisation", "Scan Localisation")),
        ("", ("Workspace",)))
LOCALISATION_PAGES = (LocalisationPage, NewScanLocalisationPage)


class DesktopApp:
    def __init__(self, root, session_path=None, initial_page="LiDAR Scanner"):
        self.root = root
        root.title("LiDAR SLAM Workbench")
        root.geometry("1440x900")
        root.minsize(1080, 700)
        root.configure(bg=CHROME_2)
        self.closed = False
        self.results = queue.Queue()
        self.jobs = {}
        self.building = {}  # (model, revision, key) -> SharedBuild in progress
        self.pages = {}
        self.current = None
        self.capture_owner = None
        self.last_runtime = "—"
        self.scrollers = {}  # Properties canvas -> its content frame (mouse wheel)
        self.cursor = None
        self.progress_running = False
        self.last_poll_error = None
        self.model = SharedSession(session_path)
        self.style()
        self.icons = {name: self.tool_icon(name)
                      for name in ("home", "back", "forward", "move", "zoom_to_rect", "filesave")}
        self.layers_visible = tk.BooleanVar(master=root, value=True)
        self.build_menus()
        self.build_toolbar()
        self.build_tabs()
        self.build_status()
        self.workspace = ttk.Frame(root, style="Chrome.TFrame")
        self.workspace.pack(fill="both", expand=True)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind_all("<MouseWheel>", self.on_wheel, add="+")
        root.bind_all("<Control-o>", lambda _: self.choose_session())
        root.bind_all("<Control-s>", lambda _: self.save_copy())
        for index, name in enumerate(PAGES):
            root.bind_all(f"<Control-Key-{index + 1}>", lambda _, n=name: self.show_page(n))
        root.update_idletasks()  # real workspace size before the first page draws
        self.show_page(initial_page)
        self.after_id = root.after(80, self.poll)

    # -- look ------------------------------------------------------------------------
    def style(self):
        """Graphite theme (ui_theme) and the optional Engineering Plot face."""
        self.fonts = FontSet.current = FontSet(self.root)
        apply_styles(self.root, THEME, self.fonts)

    def tool_icon(self, name):
        """Matplotlib's black tool icons, recoloured for the dark tool bar."""
        try:
            from PIL import Image, ImageTk
            shape = Image.open(ICONS / f"{name}.png").convert("RGBA")
            icon = Image.new("RGBA", shape.size, TEXT)
            icon.putalpha(shape.getchannel("A"))
            return ImageTk.PhotoImage(icon, master=self.root)
        except (ImportError, OSError):
            return tk.PhotoImage(master=self.root, file=str(ICONS / f"{name}.png"))

    # -- menus, tool bar, tabs, status bar ---------------------------------------------
    def page(self):
        return self.pages.get(self.current)

    def on_page(self, method, *args):
        """Menu/tool-bar command: call `method` on the current page if it has it."""
        def command():
            page = self.page()
            if page is not None and hasattr(page, method):
                getattr(page, method)(*args)
        return command

    def build_menus(self):
        bar = tk.Menu(self.root)
        self.root.configure(menu=bar)
        self.menu_rules = []  # (menu, index, predicate(page))
        is_scanner = lambda p: isinstance(p, ScannerPage)
        captures = lambda p: hasattr(p, "capture_buttons")
        is_loc = lambda p: isinstance(p, LOCALISATION_PAGES)
        is_grid = lambda p: isinstance(p, GridPage)

        def menu(title):
            m = tk.Menu(bar, tearoff=0)
            m.configure(postcommand=lambda: self.refresh_menu(m))
            bar.add_cascade(label=title, menu=m)
            return m

        def item(m, label, command, when=None, accelerator=None):
            m.add_command(label=label, command=command, accelerator=accelerator)
            if when is not None:
                self.menu_rules.append((m, m.index("end"), when))

        def radio(m, label, variable, value, command, when):
            m.add_radiobutton(label=label, variable=variable, value=value, command=command)
            self.menu_rules.append((m, m.index("end"), when))

        def check(m, label, variable, command, when=None):
            m.add_checkbutton(label=label, variable=variable, command=command)
            if when is not None:
                self.menu_rules.append((m, m.index("end"), when))

        file = menu("File")
        item(file, "Open Session…", self.choose_session, accelerator="Ctrl+O")
        item(file, "Save Working Copy As…", self.save_copy, accelerator="Ctrl+S")
        file.add_separator()
        item(file, "Export Map Image…", lambda: self.page() and self.page().toolbar.save_figure(),
             lambda p: True)
        file.add_separator()
        item(file, "Exit", self.close)

        view = menu("View")
        item(view, "Fit Map to Window", self.on_page("fit_map"))
        item(view, "Reset View", lambda: self.page() and self.page().toolbar.home())
        item(view, "Previous View", lambda: self.page() and self.page().toolbar.back())
        item(view, "Next View", lambda: self.page() and self.page().toolbar.forward())
        view.add_separator()
        self.menu_pan = tk.BooleanVar(master=self.root)
        self.menu_zoom = tk.BooleanVar(master=self.root)
        check(view, "Pan", self.menu_pan, self.toggle_pan)
        check(view, "Zoom to Rectangle", self.menu_zoom, self.toggle_zoom)
        view.add_separator()
        check(view, "Layers Panel", self.layers_visible, self.apply_layers)
        view.add_separator()
        self.menu_page = tk.StringVar(master=self.root)
        for index, name in enumerate(PAGES):
            view.add_radiobutton(label=f"{TAB_LABELS[name]}", variable=self.menu_page, value=name,
                                 accelerator=f"Ctrl+{index + 1}", command=lambda n=name: self.show_page(n))

        scan = menu("Scan")
        item(scan, "Capture New Scan", self.on_page("start_capture"), captures)
        item(scan, "Capture Replacement for Selected Scan", self.on_page("capture_replacement"), is_scanner)
        item(scan, "Cancel Capture", self.on_page("cancel_capture"), captures)
        scan.add_separator()
        item(scan, "Load Recorded Scan…", self.on_page("choose_recorded"), is_loc)
        scan.add_separator()
        item(scan, "Auto-Align Selected Scan", self.on_page("action", "auto_align"), is_scanner)
        item(scan, "Apply Proposed Alignment", self.on_page("action", "accept"), is_scanner)
        item(scan, "Discard Proposed Alignment", self.on_page("action", "reject"), is_scanner)
        scan.add_separator()
        item(scan, "Hide / Show Selected Scan", self.on_page("action", "toggle_visibility"), is_scanner)
        item(scan, "Show All Scans", self.on_page("action", "show_all"), is_scanner)
        item(scan, "Reset Selected Scan Pose to (0, 0, 0°)", self.on_page("action", "reset_pose"), is_scanner)
        scan.add_separator()
        item(scan, "Delete Selected Scan (asks to confirm)", self.on_page("action", "delete"), is_scanner)

        maps = menu("Map")
        self.menu_grid_view = tk.StringVar(master=self.root)
        for mode in GridPage.MODES:
            radio(maps, f"Grid: {mode}", self.menu_grid_view, mode, self.menu_set_grid_view, is_grid)
        self.menu_points = tk.BooleanVar(master=self.root)
        check(maps, "Grid: Show Measured Points", self.menu_points, self.menu_set_points, is_grid)
        maps.add_separator()
        self.menu_display = tk.StringVar(master=self.root)
        for mode in part1.DISPLAY_MODES:
            radio(maps, f"Scans: {mode}", self.menu_display, mode, self.menu_set_display, is_scanner)
        maps.add_separator()
        has_layers = lambda p: isinstance(p, VectorPage) and hasattr(p, "layers")
        item(maps, "Show All Vector Layers", self.on_page("all_layers", True), has_layers)
        item(maps, "Hide All Vector Layers", self.on_page("all_layers", False), has_layers)

        locm = menu("Localisation")
        item(locm, "Localise", self.on_page("run_localisation"), is_loc)
        item(locm, "Cancel Search", self.on_page("cancel_search"), is_loc)
        locm.add_separator()
        self.menu_search = tk.IntVar(master=self.root)
        radio(locm, "Global Search (no starting pose)", self.menu_search, 0, self.menu_set_search, is_loc)
        radio(locm, "Local Search (±1 m / ±30°)", self.menu_search, 1, self.menu_set_search, is_loc)
        locm.add_separator()
        is_grid_loc = lambda p: isinstance(p, LocalisationPage)
        item(locm, "Use Result as Starting Pose", self.on_page("use_result"), is_grid_loc)
        item(locm, "Evaluate All Recorded Scans → CSV + JSON…", self.on_page("evaluate"), is_grid_loc)

    def refresh_menu(self, menu):
        page = self.page()
        for owner, index, rule in self.menu_rules:
            if owner is menu:
                menu.entryconfigure(index, state="normal" if page is not None and rule(page) else "disabled")
        if page is None:
            return
        self.menu_page.set(self.current)
        self.menu_pan.set(page.toolbar.mode == "pan/zoom")
        self.menu_zoom.set(page.toolbar.mode == "zoom rect")
        if isinstance(page, GridPage):
            self.menu_grid_view.set(page.view.get())
            self.menu_points.set(page.points.get())
        if isinstance(page, ScannerPage):
            self.menu_display.set(page.display.get())
        if isinstance(page, LOCALISATION_PAGES):
            self.menu_search.set(0 if page.global_search() else 1)

    def menu_set_grid_view(self):
        page = self.page()
        page.view.set(self.menu_grid_view.get())
        page.draw_map()

    def menu_set_points(self):
        page = self.page()
        page.points.set(self.menu_points.get())
        page.draw_map()

    def menu_set_display(self):
        page = self.page()
        page.display.set(self.menu_display.get())
        page.action("display_mode", page.display.get())

    def menu_set_search(self):
        page = self.page()
        page.search.current(self.menu_search.get())
        page.change_search()

    def build_toolbar(self):
        bar = ttk.Frame(self.root, style="Chrome.TFrame", padding=(4, 2))
        bar.pack(side="top", fill="x")

        def tool(text, command, tooltip, icon=None):
            widget = ttk.Button(bar, text=text, command=command, style="Tool.TButton",
                                image=self.icons[icon] if icon else "", compound="left")
            widget.pack(side="left", padx=1)
            return tip(widget, tooltip)

        def separator():
            ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=5, pady=3)

        tool("Open…", self.choose_session, "Open Session: load a LiDAR session JSON (Ctrl+O).")
        tool("Save Copy…", self.save_copy, "Save Working Copy As: write the working session to a NEW "
                                           "JSON file; the source file is never overwritten (Ctrl+S).",
             "filesave")
        separator()
        tool("Fit Map to Window", self.on_page("fit_map"), "Show the whole map in the viewport.")
        tool("Reset View", lambda: self.page().toolbar.home(), "Back to the first view of this map.", "home")
        tool("", lambda: self.page().toolbar.back(), "Previous view.", "back")
        tool("", lambda: self.page().toolbar.forward(), "Next view.", "forward")
        separator()
        self.pan_button = tool("Pan", self.toggle_pan, "Pan: drag with the left button; right-drag "
                                                       "zooms. Map clicks set no poses while on.", "move")
        self.zoom_button = tool("Zoom", self.toggle_zoom, "Zoom to Rectangle: drag a box to zoom in "
                                                          "(right-drag zooms out).", "zoom_to_rect")
        separator()
        tool("Export Image…", lambda: self.page().toolbar.save_figure(), "Save the current map as an image.")
        separator()
        self.layers_button = tool("Layers", self.toggle_layers, "Show or hide the Layers panel.")
        session = ttk.Frame(bar, style="Chrome.TFrame")
        session.pack(side="right", padx=6)
        ttk.Label(session, text="Session:", style="Bar.TLabel").pack(side="left")
        self.session_label = ttk.Label(session, style="Session.TLabel")
        self.session_label.pack(side="left", padx=(4, 0))
        self.dirty_label = ttk.Label(session, style="Dirty.TLabel")
        self.dirty_label.pack(side="left", padx=(6, 0))
        tip(self.dirty_label, "Edits are in memory only. Save Working Copy keeps them.")

    def build_tabs(self):
        strip = ttk.Frame(self.root, style="Chrome.TFrame")
        strip.pack(side="top", fill="x")
        tk.Frame(strip, height=1, background=BORDER).pack(side="bottom", fill="x")
        self.nav_buttons, self.tab_bars = {}, {}
        for index, (caption, names) in enumerate(DAYS):
            if index:
                ttk.Separator(strip, orient="vertical").pack(side="left", fill="y", padx=(8, 0), pady=4)
            ttk.Label(strip, text=caption, style="TabGroup.TLabel").pack(side="left")
            for name in names:
                cell = tk.Frame(strip, background=CHROME_2)
                cell.pack(side="left", padx=(0, 1))
                bar = tk.Frame(cell, height=2, background=CHROME_2)
                bar.pack(side="top", fill="x")
                widget = ttk.Button(cell, text=TAB_LABELS[name], style="Tab.TButton",
                                    command=lambda n=name: self.show_page(n))
                widget.pack(side="top")
                tip(widget, f"{name} (Ctrl+{list(PAGES).index(name) + 1})")
                self.nav_buttons[name], self.tab_bars[name] = widget, bar

    def build_status(self):
        bar = ttk.Frame(self.root, style="Status.TFrame")
        bar.pack(side="bottom", fill="x")
        tk.Frame(bar, height=1, background=BORDER).pack(side="top", fill="x")
        self.status = bar

        def field(side="left", style="Status.TLabel", width=None, tooltip=None):
            label = ttk.Label(bar, style=style, width=width)
            label.pack(side=side)
            ttk.Separator(bar, orient="vertical").pack(side=side, fill="y", pady=3)
            if tooltip:
                tip(label, tooltip)
            return label

        self.status_session = field(tooltip="Working session file.")
        self.status_scans = field(tooltip="Scans in the session · visible scans used by the maps.")
        self.status_serial = field(tooltip="LD06 serial connection (state and data format when known).")
        if self.fonts.tk_available:
            # Measurement readout in the identity face; equal digit cells, fixed width.
            self.status_cursor = TechnicalLabel(bar, "", self.fonts, size=10, background=CHROME_2,
                                                foreground=TEXT, padx=8, pady=3, width=256, tabular=True)
            self.status_cursor.pack(side="right")
            ttk.Separator(bar, orient="vertical").pack(side="right", fill="y", pady=3)
            tip(self.status_cursor, "Map coordinates under the cursor (metres).")
        else:
            self.status_cursor = field("right", "StatusMono.TLabel", 24, "Map coordinates under the cursor.")
        self.status_loc = field("right", "StatusVerdictNone.TLabel",
                                tooltip="Result of this page's last localisation, and the runtime of "
                                        "the most recent one on any page.")
        self.status_operation = ttk.Label(bar, style="Status.TLabel")
        self.status_operation.pack(side="left")
        self.progress = ttk.Progressbar(bar, length=130, maximum=1.0, mode="determinate")
        self.progress.pack(side="left", padx=4)

    # -- commands ------------------------------------------------------------------------
    def toggle_pan(self):
        self.page().toolbar.pan()
        self.update_tool_states()

    def toggle_zoom(self):
        self.page().toolbar.zoom()
        self.update_tool_states()

    def update_tool_states(self):
        page = self.page()
        mode = page.toolbar.mode if page is not None else ""
        for widget, active in ((self.pan_button, mode == "pan/zoom"),
                               (self.zoom_button, mode == "zoom rect"),
                               (self.layers_button, self.layers_visible.get())):
            style = "ToolOn.TButton" if active else "Tool.TButton"
            if widget.cget("style") != style:
                widget.configure(style=style)

    def toggle_layers(self):
        self.layers_visible.set(not self.layers_visible.get())
        self.apply_layers()

    def apply_layers(self):
        page = self.page()
        if page is not None:
            page.set_layers_visible(self.layers_visible.get())
        self.update_tool_states()

    def show_cursor(self, xy):
        self.cursor = xy

    def on_wheel(self, event):
        """Scroll a Properties dock under the pointer (only when it overflows)."""
        # Tk path first: over a combobox drop-down, winfo_containing() raises
        # KeyError (Tk-internal popdown widget with no tkinter object).
        path = str(self.root.tk.call("winfo", "containing", event.x_root, event.y_root))
        try:
            widget = self.root.nametowidget(path) if path else None
        except KeyError:
            return
        while widget is not None:
            if widget in self.scrollers:
                if self.scrollers[widget].winfo_reqheight() > widget.winfo_height():
                    widget.yview_scroll(int(-event.delta / 120), "units")
                return
            if isinstance(widget, (ttk.Combobox, ttk.Treeview)):
                return
            widget = widget.master

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
        page.set_layers_visible(self.layers_visible.get())
        self.current = name
        for label, widget in self.nav_buttons.items():
            active = label == name
            widget.configure(style="TabOn.TButton" if active else "Tab.TButton")
            self.tab_bars[label].configure(background=SELECT if active else CHROME_2)
        self.cursor = None
        self.update_tool_states()
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
        """Deliver worker results and refresh status, every 80 ms.

        This is the ONLY path by which built maps reach the UI, so it always
        reschedules itself: an exception in a page tick or the status bar is
        reported, never allowed to stop result delivery (previously one
        KeyError left every later map stuck on "Building…").
        """
        if self.closed:
            return
        try:
            self.deliver_results()
            for page in list(self.pages.values()):
                try:
                    page.tick()
                except Exception as exc:
                    self.report_poll_error(page, exc)
            try:
                self.update_status()
            except Exception as exc:
                self.report_poll_error(self.page(), exc)
        finally:
            if not self.closed:
                self.after_id = self.root.after(80, self.poll)

    def report_poll_error(self, page, exc):
        message = f"Background update error: {exc!r}"
        if message == self.last_poll_error:
            return  # once per distinct error, not every 80 ms
        self.last_poll_error = message
        traceback.print_exception(exc)
        if page is not None and page.alive:
            page.set_info(message)

    def deliver_results(self):
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

    def update_status(self):
        dirty = self.model.dirty
        name = self.model.name
        set_text(self.session_label, name)
        set_text(self.dirty_label, "● unsaved edits" if dirty else "")
        title = f"{name}{' *' if dirty else ''} — LiDAR SLAM Workbench"
        if self.root.title() != title:
            self.root.title(title)
        set_text(self.status_session, f"{name}{'  (unsaved edits)' if dirty else ''}")
        set_text(self.status_scans, f"{len(self.model.session['scans'])} scans · "
                                    f"{len(self.model.scans)} visible")
        connection = self.capture_owner.capture_adapter.state if self.capture_owner else "Port closed"
        set_text(self.status_serial, f"Serial: {connection}")
        # Green only while the LD06 port is actually open and reading.
        live = "Status" + ("Live" if connection.startswith("Reading") else "") + ".TLabel"
        if str(self.status_serial.cget("style")) != live:
            self.status_serial.configure(style=live)
        operation = next((f"{label} {time.monotonic() - start:.1f} s"
                          for owner, label, start in self.jobs.values() if owner.alive), "Ready")
        page = self.page()
        if self.current == "LiDAR Scanner" and page.editor["is_aligning"]():
            operation = "Aligning selected scan (Code 01)"
        set_text(self.status_operation, operation)
        busy = "StatusReady.TLabel" if operation == "Ready" else "StatusBusy.TLabel"
        if str(self.status_operation.cget("style")) != busy:
            self.status_operation.configure(style=busy)
        progress = getattr(page, "progress", None) if page is not None and page.busy else None
        working = operation != "Ready"
        if progress is not None:
            if self.progress_running:
                self.progress.stop()
                self.progress_running = False
            self.progress.configure(mode="determinate")
            self.progress["value"] = progress
        elif working:
            if not self.progress_running:
                self.progress.configure(mode="indeterminate")
                self.progress.start(15)
                self.progress_running = True
        else:
            if self.progress_running:
                self.progress.stop()
                self.progress_running = False
            self.progress.configure(mode="determinate")
            self.progress["value"] = 0
        set_text(self.status_cursor, "X       —      Y       —" if self.cursor is None else
                 f"X {self.cursor[0]:8.3f} m  Y {self.cursor[1]:8.3f} m")
        state = page.localisation_status() if page is not None else None
        if state is None:
            text, level = f"Last localisation: {self.last_runtime}", "none"
        else:
            text, level = f"{state[0]}   ·   last run {self.last_runtime}", state[1]
        set_text(self.status_loc, text)
        style = "StatusVerdict" + {"ok": "OK", "weak": "Weak", "bad": "Bad"}.get(level, "None") + ".TLabel"
        if self.status_loc.cget("style") != style:
            self.status_loc.configure(style=style)
        self.update_tool_states()

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
        if self.progress_running:
            self.progress.stop()
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
        self.root.unbind_all("<MouseWheel>")
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
