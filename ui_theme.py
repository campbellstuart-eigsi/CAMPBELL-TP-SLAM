"""Workstation UI tokens. Scientific colour maps and scan colours are separate.

Palette provenance: user-supplied graphite / purple / green values, 2026-11-03.
Semantics: purple = selected / active / focused; green = connected, live,
confirmed; greys = every other surface. Plot paper, scan colours, occupancy
grey levels and localisation markers are data colours and are not themed.
"""
from dataclasses import dataclass, replace
from tkinter import ttk

from ui_fonts import FontSet


@dataclass(frozen=True)
class Spacing:
    left_panel: int = 214
    right_panel: int = 318
    description_wrap: int = 266
    button: tuple = (7, 3)
    toolbar: tuple = (5, 2)
    tab: tuple = (11, 4)
    heading: tuple = (8, 3)
    field: tuple = (4, 2)
    status: tuple = (6, 3)
    tree_row: int = 20
    divider: int = 7
    border: int = 1


@dataclass(frozen=True)
class Theme:
    name: str = "Graphite / restrained purple"
    background: str = "#1F2024"
    panel: str = "#2B2D33"
    raised: str = "#3A3841"
    border: str = "#55515D"
    field: str = "#24252A"
    text: str = "#DDD9D6"
    secondary: str = "#AAA6A3"
    purple: str = "#8C7E99"
    lavender: str = "#B2A6BD"
    green: str = "#6F8B78"
    sage: str = "#98B09E"
    hover: str = "#45414C"
    pressed: str = "#4D4456"
    selection: str = "#48404F"
    selection_text: str = "#DDD9D6"
    viewport: str = "#25262B"
    viewport_title: str = "#303037"
    active_title: str = "#3A3841"
    active_title_text: str = "#B2A6BD"
    primary: str = "#4A4153"       # restrained purple fill for the one main action
    primary_hover: str = "#544A5F"
    primary_pressed: str = "#5E536B"
    primary_text: str = "#EAE5EF"
    paper: str = "#F3F5F7"         # plot area; kept light so data colours keep their meaning
    plot_spine: str = "#6E6A74"
    disabled: str = "#85818A"
    disabled_bg: str = "#292A2F"
    danger: str = "#D9A29C"
    danger_bg: str = "#4A3439"
    danger_edge: str = "#9C6B71"
    danger_pressed: str = "#593C42"
    warning: str = "#D0B687"
    success_bg: str = "#293A30"
    warning_bg: str = "#463E2F"
    tooltip: str = "#39373F"
    spacing: Spacing = Spacing()

    def aliases(self):
        """Compatibility tokens for the existing compact desktop shell."""
        return dict(CHROME=self.panel, CHROME_2=self.background,
                    DOCK_TITLE=self.raised, HEADER=self.raised, BORDER=self.border,
                    FIELD=self.field, TEXT=self.text, MUTED=self.secondary,
                    SELECT=self.purple, OK_COLOUR=self.sage, WEAK_COLOUR=self.warning,
                    BAD_COLOUR=self.danger, VIEW_BG=self.viewport,
                    VIEW_CAPTION=self.viewport_title, VIEW_TEXT=self.text,
                    VIEW_MUTED=self.secondary)


GRAPHITE = Theme()
LAVENDER = replace(GRAPHITE, name="Graphite / stronger lavender selection",
                   panel="#292A30", field="#303037", viewport="#1F2024",
                   active_title="#B2A6BD", active_title_text="#1F2024",
                   selection="#B2A6BD", selection_text="#1F2024")
PROTOTYPES = {"graphite": GRAPHITE, "lavender": LAVENDER}
# Production theme (2026-11-04). LAVENDER and render_prototypes.py remain as
# the Stage A comparison; the stronger lavender fill was judged too loud.
THEME = GRAPHITE


@dataclass(frozen=True)
class PlotPalette:
    """Named colours drawn INSIDE plots (not chrome). Values are the ones the
    pages already used; Codes 01-06 keep their own scan and pose colours."""
    # In-plot verdict text on the light paper of pages 04/05 (>= 4.4:1 on paper).
    verdict_ok: str = "#1B7F4B"
    verdict_weak: str = "#9A6700"
    verdict_bad: str = "#B3261E"
    vector_reference: str = "#16232F"   # page 05 reference segments
    # Workspace "Dark" plot style: occupancy stops free / unknown / occupied.
    dark_free: str = "#4B4D56"
    dark_unknown: str = "#1A1B1F"
    dark_occupied: str = "#EEE8E0"
    dark_face: str = "#1A1B1F"
    dark_spine: str = "#3F3D46"
    dark_grid: str = "#3A3940"
    dark_vector: str = "#E6E1DA"
    dark_current: str = "#8DB4D2"
    # Workspace "Light paper" plot style (face = THEME.paper).
    paper_grid: str = "#B0B0B0"
    paper_vector: str = "#153D71"
    paper_current: str = "#246F9D"
    paper_empty: str = "#555555"
    # Workspace pose vocabulary, the same meanings as pages 04-06 (approved
    # 2026-11-04): green = recorded (Day 1) pose, orange = initial guess,
    # blue = localised estimate, purple = alternative pose. Rejected results
    # draw no pose. The blue/purple pairs keep >= 3.5:1 on each plot face.
    recorded_pose: str = "#2CA02C"      # tab:green
    initial_guess: str = "#FF7F0E"      # tab:orange
    estimate_paper: str = "#1F77B4"     # tab:blue
    estimate_dark: str = "#6FA8DC"
    alternative_paper: str = "#9467BD"  # tab:purple
    alternative_dark: str = "#B39DDB"
    axis_paper: str = "#6E6A74"         # sensor +X cross on Current Scan
    axis_dark: str = "#8A8790"


PLOT = PlotPalette()


def apply_styles(root, theme=THEME, fonts=None):
    """Configure compact rectangular ttk controls and their interaction states."""
    t, s = theme, theme.spacing
    fonts = fonts or FontSet(root)
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure(".", background=t.panel, foreground=t.text, font=fonts.body,
                    bordercolor=t.border, lightcolor=t.panel, darkcolor=t.panel,
                    troughcolor=t.background, selectbackground=t.selection,
                    selectforeground=t.selection_text, focuscolor=t.lavender, insertcolor=t.text)
    for name, colour in {"Chrome": t.background, "View": t.viewport,
                         "Caption": t.viewport_title, "DockTitle": t.raised,
                         "Footer": t.raised, "Status": t.background}.items():
        style.configure(f"{name}.TFrame", background=colour)
    labels = {
        "Caption": (t.viewport_title, t.secondary, fonts.small, (8, 3)),
        "Note": (t.panel, t.secondary, fonts.body, 0),
        "Message": (t.panel, t.text, fonts.body, 0),
        "Field": (t.panel, t.secondary, fonts.small, 0),
        "Mono": (t.panel, t.text, fonts.numeric, 0),
        "Coord": (t.panel, t.secondary, (fonts.small[0], fonts.small[1], "italic"), 0),
        "PageTitle": (t.panel, t.text, fonts.page_title, 0),
        "DockTitle": (t.raised, t.secondary, fonts.dock_title, s.heading),
        "Section": (t.raised, t.text, fonts.bold, s.heading),
        "Sub": (t.panel, t.secondary, fonts.bold, 0),
        "TabGroup": (t.background, t.secondary, (fonts.small[0], 8, "bold"), (8, 0, 3, 0)),
        "Bar": (t.background, t.secondary, fonts.body, 0),
        "Session": (t.background, t.text, fonts.bold, 0),
        "Dirty": (t.background, t.warning, fonts.bold, 0),
        "FooterTitle": (t.raised, t.text, fonts.bold, 0),
        "Status": (t.background, t.secondary, fonts.body, s.status),
        "StatusMono": (t.background, t.text, fonts.numeric, s.status),
        "StatusReady": (t.background, t.sage, fonts.body, s.status),
        "StatusBusy": (t.background, t.lavender, fonts.body, s.status),
        "StatusLive": (t.background, t.sage, fonts.bold, s.status),
        "VerdictNone": (t.raised, t.secondary, fonts.body, (6, 2)),
        "StatusVerdictNone": (t.background, t.secondary, fonts.body, s.status),
    }
    for name, (bg, fg, font, padding) in labels.items():
        style.configure(f"{name}.TLabel", background=bg, foreground=fg, font=font, padding=padding)
    for level, fg, bg in (("OK", t.sage, t.success_bg), ("Weak", t.warning, t.warning_bg),
                          ("Bad", t.danger, t.danger_bg)):
        style.configure(f"Verdict{level}.TLabel", background=bg, foreground=fg,
                        font=fonts.bold, padding=(6, 2))
        style.configure(f"StatusVerdict{level}.TLabel", background=t.background,
                        foreground=fg, font=fonts.bold, padding=s.status)

    def control(name, bg, fg=t.text, padding=s.button, border=None, hover=None, pressed=None):
        border = border or t.border
        style.configure(name, background=bg, foreground=fg, padding=padding, relief="flat",
                        borderwidth=1, bordercolor=border, lightcolor=bg, darkcolor=bg,
                        focuscolor=t.lavender, font=fonts.body)
        states = [("disabled", t.disabled_bg), ("pressed", pressed or t.pressed),
                  ("active", hover or t.hover)]
        for element in ("background", "lightcolor", "darkcolor"):
            style.map(name, **{element: states})
        style.map(name, foreground=[("disabled", t.disabled), ("!disabled", fg)],
                  bordercolor=[("disabled", t.disabled_bg), ("focus", t.lavender),
                               ("active", t.purple), ("!disabled", border)])

    control("TButton", t.raised)
    control("Tool.TButton", t.background, padding=s.toolbar, border=t.background)
    control("ToolOn.TButton", t.selection, fg=t.selection_text, padding=s.toolbar, border=t.purple)
    # Tabs: inactive ones sit on the strip in secondary text; the active tab
    # takes the page colour so it joins the page, with a purple bar above it.
    control("Tab.TButton", t.background, fg=t.secondary, padding=s.tab, border=t.background)
    control("TabOn.TButton", t.panel, fg=t.lavender, padding=s.tab, border=t.panel,
            hover=t.panel, pressed=t.panel)
    style.configure("TabOn.TButton", font=fonts.bold)
    control("DockButton.TButton", t.raised, padding=(2, 0), border=t.raised)
    control("Expander.TButton", t.panel, fg=t.secondary, padding=(6, 3), border=t.panel)
    style.configure("Expander.TButton", anchor="w", font=fonts.bold)
    control("Viewport.TButton", t.viewport_title, fg=t.secondary, padding=(6, 0),
            border=t.viewport_title)
    style.configure("Viewport.TButton", font=fonts.small)
    control("ViewportOn.TButton", t.active_title, fg=t.active_title_text,
            padding=(2, 0), border=t.purple)
    # One main action per group: dark purple fill, lavender edge, bold text.
    # Hierarchy comes from weight and edge, not a large saturated fill.
    control("Primary.TButton", t.primary, fg=t.primary_text, border=t.purple,
            hover=t.primary_hover, pressed=t.primary_pressed)
    style.configure("Primary.TButton", font=fonts.bold)
    style.map("Primary.TButton", bordercolor=[("disabled", t.disabled_bg), ("focus", t.lavender),
                                              ("active", t.lavender), ("!disabled", t.purple)])
    # Destructive: muted red outline and text, never a filled call to action.
    control("Danger.TButton", t.panel, fg=t.danger, border=t.danger_edge,
            hover=t.danger_bg, pressed=t.danger_pressed)
    style.map("Danger.TButton", bordercolor=[("disabled", t.disabled_bg), ("focus", t.danger),
                                             ("active", t.danger), ("!disabled", t.danger_edge)])

    for name in ("TEntry", "TCombobox"):
        style.configure(name, fieldbackground=t.field, foreground=t.text,
                        background=t.raised, bordercolor=t.border, lightcolor=t.field,
                        darkcolor=t.field, insertcolor=t.text, padding=s.field, arrowsize=12,
                        arrowcolor=t.secondary)
        style.map(name, fieldbackground=[("disabled", t.disabled_bg), ("readonly", t.field)],
                  foreground=[("disabled", t.disabled), ("!disabled", t.text)],
                  selectbackground=[("readonly", t.field), ("!readonly", t.selection)],
                  selectforeground=[("readonly", t.text), ("!readonly", t.selection_text)],
                  bordercolor=[("focus", t.lavender), ("active", t.purple)])
    style.configure("TCheckbutton", background=t.panel, foreground=t.text,
                    indicatorbackground=t.field, indicatorforeground=t.text,
                    indicatorcolor=t.field, bordercolor=t.border, focuscolor=t.lavender)
    style.map("TCheckbutton", background=[("active", t.panel)],
              foreground=[("disabled", t.disabled)],
              # A disabled checkbox must still show whether it is ticked.
              indicatorbackground=[("disabled", "selected", t.disabled), ("disabled", t.disabled_bg),
                                   ("selected", t.lavender), ("active", t.hover)],
              indicatorforeground=[("disabled", "selected", t.disabled_bg), ("selected", t.background)],
              bordercolor=[("focus", t.lavender), ("active", t.purple)])
    style.configure("Treeview", background=t.field, fieldbackground=t.field,
                    foreground=t.text, rowheight=s.tree_row, bordercolor=t.border,
                    lightcolor=t.field, darkcolor=t.field, font=fonts.body)
    style.map("Treeview", background=[("selected", t.selection)],
              foreground=[("selected", t.selection_text)])
    style.configure("Treeview.Heading", background=t.raised, foreground=t.secondary,
                    font=fonts.bold, padding=(4, 2), bordercolor=t.border,
                    lightcolor=t.raised, darkcolor=t.raised, relief="flat")
    style.map("Treeview.Heading", background=[("active", t.hover)])
    # Work in progress is "active" (purple); green is kept for confirmed states.
    style.configure("Horizontal.TProgressbar", background=t.purple, troughcolor=t.field,
                    bordercolor=t.border, lightcolor=t.purple, darkcolor=t.purple, thickness=8)
    for orient in ("Horizontal", "Vertical"):
        style.configure(f"{orient}.TScrollbar", background=t.raised, troughcolor=t.background,
                        arrowcolor=t.secondary, bordercolor=t.background, lightcolor=t.raised,
                        darkcolor=t.raised, gripcount=0, arrowsize=12)
        style.map(f"{orient}.TScrollbar", background=[("active", t.hover), ("pressed", t.purple)])
        style.configure(f"{orient}.TSeparator", background=t.border)
    style.configure("TScale", background=t.raised, troughcolor=t.field, bordercolor=t.border,
                    lightcolor=t.raised, darkcolor=t.raised)
    style.map("TScale", background=[("active", t.lavender), ("pressed", t.lavender)])
    style.configure("TSeparator", background=t.border)
    for option, value in {"background": t.field, "foreground": t.text,
                          "selectBackground": t.selection, "selectForeground": t.selection_text,
                          "font": fonts.body}.items():
        root.option_add(f"*TCombobox*Listbox.{option}", value)
    for option, value in {"font": fonts.body, "background": t.panel, "foreground": t.text,
                          "activeBackground": t.selection, "activeForeground": t.selection_text,
                          "disabledForeground": t.disabled}.items():
        root.option_add(f"*Menu.{option}", value)
    root.configure(background=t.background)
    return fonts


def contrast(foreground, background):
    def luminance(hex_colour):
        c = [int(hex_colour[i:i + 2], 16) / 255 for i in (1, 3, 5)]
        linear = [v / 12.92 if v <= .04045 else ((v + .055) / 1.055) ** 2.4 for v in c]
        return sum(v * weight for v, weight in zip(linear, (.2126, .7152, .0722)))
    low, high = sorted((luminance(foreground), luminance(background)))
    return (high + .05) / (low + .05)
