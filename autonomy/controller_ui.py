"""Robot controller panel: connect, drive with the keyboard, stop; status and a LiDAR preview.

RobotControllerPanel is a ttk.Frame styled with the SLAM application's
ui_theme, so a Workspace page can host it later; robot_controller.py opens it
in its own window. All networking is in autonomy.teleop (worker threads);
this Tk side only calls its non-blocking methods and redraws from
Teleop.snapshot() every POLL_MS.

Use: Connect, hold a movement key to drive (the first press arms the robot;
the firmware may refuse and says why), release to stop, Space or Esc = STOP.

Modes:
  REAL ROBOT  (default) the ESP32SlamRobot firmware (http://192.168.4.1)
  SIMULATOR   a FakeHttpRobot in a simulated room on 127.0.0.1; no hardware.
              Its fault buttons exercise the safety paths.
Launching never connects or arms anything.
"""
import tkinter as tk
from tkinter import ttk

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from autonomy import protocol
from autonomy.fake_http_robot import FakeHttpRobot
from autonomy.http_robot_link import DEFAULT_URL
from autonomy.teleop import (ACTION_LABELS, DEFAULT_LAYOUT, KEY_LAYOUTS, MAX_MANUAL_PCT,
                             Teleop)
from ui_fonts import FontSet
from ui_theme import PLOT, THEME

POLL_MS = 100
RAIL_W = 340
PREVIEW_RANGE_M = 4.0
KEY_NAMES = {"space": "Space"}
PAD = [("rotate_left", "forward", "rotate_right"), ("left", "backward", "right")]


def key_name(keysym):
    return KEY_NAMES.get(keysym, keysym.upper())


class RobotControllerPanel(ttk.Frame):
    def __init__(self, master, teleop=None, fonts=None):
        super().__init__(master, style="Chrome.TFrame")
        self.fonts = fonts or FontSet.current or FontSet(self)
        self.teleop = teleop or Teleop()
        self.fake = None
        self.layout_name = tk.StringVar(value=DEFAULT_LAYOUT)
        self.mode = tk.StringVar(value="real")
        self.teleop.set_mode(True)
        self.url = tk.StringVar(value=DEFAULT_URL)
        self.drive_pct = tk.IntVar(value=self.teleop.drive_pct)
        self.turn_pct = tk.IntVar(value=self.teleop.turn_pct)
        self.fault_vars = {name: tk.BooleanVar(value=False)
                           for name in ("lidar_off", "silent", "stuck")}
        self.keyboard_active = False
        self.drawn_scan = None
        self.shown_real = None
        self._configure_styles()
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)
        self._build_rail()
        self._build_preview()
        self._apply_layout()
        self._bind_keys()
        self._poll_job = self.after(POLL_MS, self._poll)

    # -- construction ---------------------------------------------------------
    def _configure_styles(self):
        t, f = THEME, self.fonts
        style = ttk.Style(self)
        style.configure("Big.Danger.TButton", font=(f.bold[0], 16, "bold"), padding=(10, 12))
        style.configure("Key.TLabel", background=t.raised, foreground=t.text, font=f.bold,
                        padding=(4, 6), anchor="center")
        style.configure("KeyOn.TLabel", background=t.purple, foreground=t.primary_text,
                        font=f.bold, padding=(4, 6), anchor="center")
        style.configure("BannerSim.TLabel", background=t.success_bg, foreground=t.sage,
                        font=f.bold, padding=(8, 6), anchor="center")
        style.configure("BannerReal.TLabel", background=t.danger_bg, foreground=t.danger,
                        font=f.bold, padding=(8, 6), anchor="center")
        style.configure("Good.TLabel", background=t.panel, foreground=t.sage, font=f.numeric)
        style.configure("Warn.TLabel", background=t.panel, foreground=t.warning, font=f.numeric)
        style.configure("Bad.TLabel", background=t.panel, foreground=t.danger, font=f.numeric)

    def _section(self, parent, title):
        """A titled section; returns its body. Its container is body.master."""
        container = ttk.Frame(parent)
        container.pack(fill="x")
        ttk.Label(container, text=title, style="Section.TLabel").pack(fill="x", pady=(6, 0))
        body = ttk.Frame(container, padding=(8, 4, 8, 4))
        body.pack(fill="x")
        return body


    def _build_rail(self):
        rail = self.rail = ttk.Frame(self, width=RAIL_W)
        rail.grid(row=0, column=0, sticky="nsew", padx=(0, 1))
        rail.pack_propagate(False)
        # Always visible: the mode banner and STOP on top, the messages at the bottom.
        ttk.Label(rail, text="ROBOT CONTROLLER", style="DockTitle.TLabel").pack(fill="x")
        top = ttk.Frame(rail, padding=(8, 6, 8, 2))
        top.pack(fill="x")
        self.banner = ttk.Label(top, style="BannerSim.TLabel")
        self.banner.pack(fill="x")
        self.stop_button = ttk.Button(top, text="STOP", style="Big.Danger.TButton",
                                      command=self.teleop.stop)
        self.stop_button.pack(fill="x", pady=(6, 0))
        messages = ttk.Frame(rail)
        messages.pack(side="bottom", fill="x")
        ttk.Label(messages, text="Messages", style="Section.TLabel").pack(fill="x")
        self.messages = ttk.Label(messages, style="Message.TLabel", wraplength=RAIL_W - 20,
                                  justify="left", padding=(8, 3, 8, 8))
        self.messages.pack(fill="x")
        # Everything else scrolls (small screens).
        scroll = tk.Canvas(rail, bg=THEME.panel, highlightthickness=0)
        bar = ttk.Scrollbar(rail, orient="vertical", command=scroll.yview)
        bar.pack(side="right", fill="y")
        scroll.pack(side="left", fill="both", expand=True)
        scroll.configure(yscrollcommand=bar.set)
        inner = ttk.Frame(scroll)
        window = scroll.create_window((0, 0), window=inner, anchor="nw")
        inner.bind("<Configure>", lambda _: scroll.configure(scrollregion=scroll.bbox("all")))
        scroll.bind("<Configure>", lambda e: scroll.itemconfigure(window, width=e.width))
        scroll.bind("<Enter>", lambda _: scroll.bind_all(
            "<MouseWheel>", lambda e: scroll.yview_scroll(int(-e.delta / 120), "units")))
        scroll.bind("<Leave>", lambda _: scroll.unbind_all("<MouseWheel>"))
        rail = inner

        body = self._section(rail, "Mode")
        row = ttk.Frame(body)
        row.pack(fill="x")
        self.mode_buttons = {}
        for value, text in (("sim", "SIMULATOR"), ("real", "REAL ROBOT")):
            button = ttk.Button(row, text=text, command=lambda v=value: self._set_mode(v))
            button.pack(side="left", fill="x", expand=True, padx=(0, 2))
            self.mode_buttons[value] = button

        body = self._section(rail, "Connection")
        self.url_row = ttk.Frame(body)
        ttk.Label(self.url_row, text="URL", style="Field.TLabel").pack(side="left", padx=(0, 6))
        self.url_entry = ttk.Entry(self.url_row, textvariable=self.url)
        self.url_entry.pack(side="left", fill="x", expand=True)
        row = self.connect_row = ttk.Frame(body)
        row.pack(fill="x", pady=(4, 0))
        self.connect_button = ttk.Button(row, text="Connect", command=self._connect)
        self.connect_button.pack(side="left", fill="x", expand=True, padx=(0, 2))
        self.disconnect_button = ttk.Button(row, text="Disconnect",
                                            command=self.teleop.request_disconnect)
        self.disconnect_button.pack(side="left", fill="x", expand=True)

        body = self._section(rail, "Drive")
        row = ttk.Frame(body)
        row.pack(fill="x")
        ttk.Label(row, text="Keyboard", style="Field.TLabel").pack(side="left", padx=(0, 6))
        layout = ttk.Combobox(row, textvariable=self.layout_name, values=list(KEY_LAYOUTS),
                              state="readonly", width=9)
        layout.pack(side="left")
        layout.bind("<<ComboboxSelected>>", lambda _: (self._apply_layout(), self.focus_set()))
        self.keyboard_label = ttk.Label(row, style="Note.TLabel")
        self.keyboard_label.pack(side="right")
        pad = ttk.Frame(body)
        pad.pack(fill="x", pady=(4, 2))
        self.key_labels = {}
        for r, actions in enumerate(PAD):
            for c, action in enumerate(actions):
                label = ttk.Label(pad, style="Key.TLabel", width=12, justify="center")
                label.grid(row=r, column=c, padx=1, pady=1, sticky="ew")
                self.key_labels[action] = label
        for column in range(3):
            pad.columnconfigure(column, weight=1)
        self.stop_key_label = ttk.Label(pad, style="Key.TLabel", justify="center")
        self.stop_key_label.grid(row=2, column=0, columnspan=3, padx=1, pady=1, sticky="ew")
        for text, var in (("Drive %", self.drive_pct), ("Turn %", self.turn_pct)):
            row = ttk.Frame(body)
            row.pack(fill="x", pady=(2, 0))
            ttk.Label(row, text=text, style="Field.TLabel", width=8).pack(side="left")
            value = ttk.Label(row, style="Mono.TLabel", width=4)
            value.pack(side="right")
            scale = ttk.Scale(row, from_=0, to=MAX_MANUAL_PCT, variable=var,
                              command=lambda _, v=var, l=value: self._speeds_changed(v, l))
            scale.pack(side="left", fill="x", expand=True)
            value.configure(text=f"{var.get()}")
        self.calibration_note = ttk.Label(body, style="Note.TLabel", wraplength=RAIL_W - 30,
                                          justify="left")
        self.calibration_note.pack(fill="x", pady=(4, 0))

        body = self._section(rail, "Status")
        self.status_values = {}
        for name in ("Link", "Firmware", "LiDAR", "I²C", "Motors", "Last stop", "Refused",
                     "Command", "Reply"):
            row = ttk.Frame(body)
            row.pack(fill="x")
            ttk.Label(row, text=name, style="Field.TLabel", width=9).pack(side="left")
            value = ttk.Label(row, style="Mono.TLabel", anchor="w")
            value.pack(side="left", fill="x", expand=True)
            self.status_values[name] = value

        self.status_section = body.master
        self.sim_body = self._section(rail, "Simulator faults")
        for name, text in (("lidar_off", "LiDAR off"), ("silent", "Drop Wi-Fi link"),
                           ("stuck", "Stuck motor board")):
            ttk.Checkbutton(self.sim_body, text=text, variable=self.fault_vars[name],
                            command=self._faults_changed).pack(anchor="w")
        ttk.Button(self.sim_body, text="Restart ESP32",
                   command=self._sim_restart).pack(anchor="w", pady=(2, 0))

    def _build_preview(self):
        view = ttk.Frame(self, style="View.TFrame")
        view.grid(row=0, column=1, sticky="nsew")
        caption = ttk.Frame(view, style="Caption.TFrame")
        caption.pack(side="top", fill="x")
        ttk.Label(caption, text="LiDAR preview   ·   robot frame, front up   ·   metres",
                  style="Caption.TLabel").pack(side="left")
        self.scan_caption = ttk.Label(caption, style="Caption.TLabel")
        self.scan_caption.pack(side="right")
        self.figure = Figure(figsize=(6, 6), facecolor=THEME.viewport)
        self.axes = self.figure.add_subplot(111)
        self.axes.set_facecolor(PLOT.dark_face)
        self.axes.set_aspect("equal")
        self.axes.set_xlim(-PREVIEW_RANGE_M, PREVIEW_RANGE_M)
        self.axes.set_ylim(-PREVIEW_RANGE_M, PREVIEW_RANGE_M)
        self.axes.grid(True, color=PLOT.dark_grid, linewidth=0.6)
        self.axes.tick_params(colors=THEME.secondary, labelsize=8)
        for spine in self.axes.spines.values():
            spine.set_color(PLOT.dark_spine)
        self.points_artist = self.axes.scatter([], [], s=4, color=PLOT.dark_current)
        self.axes.plot([0], [0], marker="^", markersize=12, color=PLOT.estimate_dark)
        self.axes.annotate("front", (0, 0.35), color=THEME.secondary, ha="center", fontsize=8)
        self.figure.subplots_adjust(left=0.08, right=0.97, bottom=0.06, top=0.97)
        self.canvas = FigureCanvasTkAgg(self.figure, master=view)
        widget = self.canvas.get_tk_widget()
        widget.configure(background=THEME.viewport, highlightthickness=0)
        widget.pack(fill="both", expand=True)
        widget.bind("<Button-1>", lambda _: self.focus_set())

    # -- keyboard -------------------------------------------------------------
    def _bind_keys(self):
        top = self.winfo_toplevel()
        top.bind("<KeyPress>", self._on_key_press, add="+")
        top.bind("<KeyRelease>", self._on_key_release, add="+")
        top.bind("<FocusOut>", lambda _: self.after(1, self._check_focus), add="+")
        top.bind("<FocusIn>", lambda _: self._check_focus(), add="+")
        # Minimised: only the window's own Unmap (children are hidden by mode changes).
        top.bind("<Unmap>", lambda e: self._focus_lost() if e.widget is top else None, add="+")
        top.bind("<Escape>", lambda _: self.teleop.stop(), add="+")
        self.focus_set()

    def _bindings(self):
        """keysym -> action, for the selected layout."""
        return {key: action for action, key in KEY_LAYOUTS[self.layout_name.get()].items()}

    def _typing(self):
        try:
            widget = self.focus_get()
        except (KeyError, tk.TclError):
            return False
        return isinstance(widget, (ttk.Entry, ttk.Combobox, tk.Entry))

    def _on_key_press(self, event):
        action = self._bindings().get(event.keysym.lower())
        if action is None or self._typing():
            return None
        if action == "stop":
            self.teleop.stop()
        else:
            self.teleop.press(action)
        return "break"

    def _on_key_release(self, event):
        action = self._bindings().get(event.keysym.lower())
        if action is not None and action != "stop":
            self.teleop.release(action)

    def _check_focus(self):
        try:
            focused = self.focus_get() is not None
        except (KeyError, tk.TclError):
            focused = True       # a popdown list of this window has the focus
        if not focused:
            self._focus_lost()
        self.keyboard_active = focused

    def _focus_lost(self):
        self.keyboard_active = False
        self.teleop.release_all()

    def _apply_layout(self):
        keys = KEY_LAYOUTS[self.layout_name.get()]
        for action, label in self.key_labels.items():
            label.configure(text=f"{key_name(keys[action])}\n{ACTION_LABELS[action]}")
        self.stop_key_label.configure(text=f"{key_name(keys['stop'])}   STOP   (or Esc)")

    # -- actions --------------------------------------------------------------
    def _set_mode(self, value):
        if self.teleop.set_mode(value == "real"):
            self.mode.set(value)

    def _connect(self):
        if self.mode.get() == "sim":
            if self.fake is None:
                self.fake = FakeHttpRobot(room=True).start()
            self.teleop.request_connect(self.fake.url)
        else:
            self.teleop.request_connect(self.url.get().strip())
        self.focus_set()

    def _speeds_changed(self, var, label):
        var.set(int(round(float(var.get()))))
        label.configure(text=f"{var.get()}")
        self.teleop.set_speeds(self.drive_pct.get(), self.turn_pct.get())

    def _faults_changed(self):
        if self.fake is not None:
            self.fake.lidar_ok = not self.fault_vars["lidar_off"].get()
            self.fake.silent = self.fault_vars["silent"].get()
            self.fake.stuck = self.fault_vars["stuck"].get()

    def _sim_restart(self):
        if self.fake is not None:
            self.fake.restart()

    # -- refresh --------------------------------------------------------------
    def _poll(self):
        self._poll_job = None
        try:
            self.refresh(self.teleop.snapshot())
        finally:
            self._poll_job = self.after(POLL_MS, self._poll)

    def refresh(self, snap):
        real = snap.real_robot
        connected = snap.link in ("ok", "lost")
        idle = not snap.busy
        for value, button in self.mode_buttons.items():
            on = (value == "real") == real
            button.configure(style="TabOn.TButton" if on else "Tab.TButton",
                             state="normal" if not connected and idle else "disabled")
        self.banner.configure(
            style="BannerReal.TLabel" if real else "BannerSim.TLabel",
            text="REAL ROBOT  ·  physical motors" if real
            else "SIMULATOR  ·  fake robot on this PC, no hardware")
        if real != self.shown_real:
            self.shown_real = real
            # Hidden sections are unpacked, then re-packed next to a fixed neighbour.
            if real:
                self.sim_body.master.pack_forget()
                self.url_row.pack(fill="x", before=self.connect_row)
            else:
                self.url_row.pack_forget()
                self.sim_body.master.pack(fill="x", after=self.status_section)
        self.url_entry.configure(state="disabled" if connected else "normal")
        self.connect_button.configure(state="normal" if not connected and not snap.busy
                                      else "disabled")
        self.disconnect_button.configure(state="normal" if connected and idle else "disabled")
        held = set(snap.held) if snap.armed else set()
        for action, label in self.key_labels.items():
            label.configure(style="KeyOn.TLabel" if action in held else "Key.TLabel")
        self.keyboard_label.configure(
            text="keys active" if self.keyboard_active else "click here to drive")
        self.calibration_note.configure(
            text="Hold a key to drive, release to stop. Space or Esc = STOP. Speeds are raw "
                 "PWM %: distance moves stay blocked until calibrated.")
        self._refresh_status(snap)
        self._refresh_scan(snap)
        lines = [f"{when}  {text}" for when, text in snap.errors[-5:]]
        self.messages.configure(text="\n".join(lines) or "No messages.")

    def _set_value(self, name, text, level="Mono"):
        self.status_values[name].configure(text=text, style=f"{level}.TLabel")

    def _refresh_status(self, snap):
        link_level = {"ok": "Good", "lost": "Bad", "connecting": "Warn"}.get(snap.link, "Mono")
        busy = f"  ({snap.busy}…)" if snap.busy else ""
        self._set_value("Link", f"{snap.link}{busy}", link_level)
        status = snap.status
        if status is None:
            for name in ("Firmware", "LiDAR", "I²C", "Motors", "Last stop", "Refused"):
                self._set_value(name, "—")
        else:
            if status["fault"]:
                text, level = "FAULT", "Bad"
            elif snap.armed:
                text, level = ("ARMED, moving" if status["moving"] else "ARMED"), "Warn"
            elif status.get("motors_locked"):
                text, level = "MOTORS LOCKED build", "Warn"
            else:
                text, level = "DISARMED", "Good"
            self._set_value("Firmware", f"{text} · {status['boot_id']:08X}", level)
            age = status.get("scan_age_ms", 0)
            level = "Good" if age < 500 else "Bad"
            self._set_value("LiDAR", f"{status.get('scan_points', 0)} pts · "
                                     f"{age} ms · {status.get('lidar_dps', 0)}°/s", level)
            self._refresh_motor_diagnostics(status)
        vx, vy, wz = snap.command
        state = ("stopped · press a key to drive" if snap.stop_latched
                 else f"vx {vx:+d} vy {vy:+d} wz {wz:+d} %")
        self._set_value("Command", state, "Warn" if snap.command != (0, 0, 0) else "Mono")
        self._set_value("Reply", f"{snap.last_reply or '—'}  · {snap.drives_sent} sent")

    def _refresh_motor_diagnostics(self, status):
        """The firmware's own motor and command diagnostics (absent on older builds)."""
        if "i2c_ack_0x30" not in status:
            self._set_value("I²C", "— (older firmware)")
            for name in ("Motors", "Last stop", "Refused"):
                self._set_value(name, "—")
            return
        ack = status["i2c_ack_0x30"] and not status.get("i2c_ack_0x31")
        pwm = status.get("pwm", [0, 0, 0, 0])
        self._set_value("I²C", f"0x30 {'ACK' if ack else 'NO ACK'} · {status.get('i2c_nacks', 0)}"
                               f" NACK", "Good" if ack else "Bad")
        self._set_value("Motors", f"PWM {' '.join(f'{v:+d}' for v in pwm)}",
                        "Warn" if any(pwm) else "Mono")
        self._set_value("Last stop", status.get("last_stop", "—"))
        rejects = status.get("rejects", 0)
        self._set_value("Refused", f"{status.get('last_reject', 'none')} · {rejects} since boot",
                        "Warn" if rejects else "Mono")

    def _refresh_scan(self, snap):
        if snap.scan_seq == self.drawn_scan:
            return
        self.drawn_scan = snap.scan_seq
        xy = protocol.points_to_sensor_xy(snap.scan_points)
        # Robot frame, front up: screen x = -y (right), screen y = x (front).
        self.points_artist.set_offsets([(-y, x) for x, y in xy] or [(float("nan"),) * 2])
        self.scan_caption.configure(text=f"scan {snap.scan_seq}  ·  {len(xy)} points")
        self.canvas.draw_idle()

    # -- lifecycle -----------------------------------------------------------
    def close(self):
        if self._poll_job is not None:
            self.after_cancel(self._poll_job)
            self._poll_job = None
        self.teleop.close()
        if self.fake is not None:
            self.fake.close()
            self.fake = None
