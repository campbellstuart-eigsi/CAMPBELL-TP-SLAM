"""Optional local typography; Windows fonts are private to this process."""
import atexit
import ctypes
import os
from pathlib import Path
import tkinter.font as tkfont
import tkinter as tk

from matplotlib import font_manager
from matplotlib.ft2font import FT2Font

FONT_PATH = Path(__file__).resolve().parent / "assets/fonts/local/engineering-plot-regular.ttf"
FAMILY = "Engineering Plot"
BODY_FAMILY = "Segoe UI"
NUMBER_FAMILY = "Consolas"
_private_path = None
_mpl_loaded = False


def register_private_font(path=FONT_PATH):
    """FR_PRIVATE (0x10): no global installation, registry edits or broadcasts."""
    global _private_path
    if os.name != "nt" or not path.is_file():
        return False
    if _private_path is not None:
        return True
    add = ctypes.windll.gdi32.AddFontResourceExW
    add.argtypes = [ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_void_p]
    add.restype = ctypes.c_int
    if not add(str(path.resolve()), 0x10, None):
        return False
    _private_path = str(path.resolve())
    atexit.register(release_private_font)
    return True


def release_private_font():
    global _private_path
    if _private_path is not None:
        remove = ctypes.windll.gdi32.RemoveFontResourceExW
        remove.argtypes = [ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_void_p]
        remove.restype = ctypes.c_int
        remove(_private_path, 0x10, None)
        _private_path = None


class FontSet:
    """Use Engineering Plot for short technical labels, with full-text fallback."""
    current = None  # the running application's set, for widget helpers without an app

    def __init__(self, root, use_custom=True):
        global _mpl_loaded
        self.codepoints = set()
        self.cap_ratio = 0.7  # cap height / em; measured from the face when present
        self.tk_available = self.mpl_available = False
        if use_custom and FONT_PATH.is_file():
            try:
                face = FT2Font(str(FONT_PATH))
                self.codepoints = set(face.get_charmap())
                face.set_size(100, 72)
                # This face draws capitals a full em tall (Segoe UI: ~0.7 em).
                self.cap_ratio = face.load_char(ord("H")).height / 64 / 100
                if not _mpl_loaded:
                    font_manager.fontManager.addfont(str(FONT_PATH))
                    _mpl_loaded = True
                self.mpl_available = True
                if register_private_font():
                    self.tk_available = FAMILY in tkfont.families(root)
            except (OSError, RuntimeError, ValueError):
                pass  # the UI must remain usable without the optional asset
        self.body = (BODY_FAMILY, 9)
        self.small = (BODY_FAMILY, 8)
        self.bold = (BODY_FAMILY, 9, "bold")
        self.numeric = (NUMBER_FAMILY, 9)
        self.heading = (FAMILY, 12) if self.tk_available else (BODY_FAMILY, 10, "bold")
        # Identity roles: short all-caps technical labels only. Body text,
        # settings and explanations stay in Segoe UI; editable numbers in Consolas.
        custom = self.tk_available
        self.dock_title = (FAMILY, 10) if custom else (BODY_FAMILY, 9, "bold")
        self.page_title = (FAMILY, 13) if custom else (BODY_FAMILY, 10, "bold")

    def supports(self, text):
        return bool(self.codepoints) and all(ord(char) in self.codepoints for char in text)

    def technical(self, text, size=11):
        if self.tk_available and self.supports(text):
            return (FAMILY, size)
        return (NUMBER_FAMILY, size - 1)

    def plot(self, text, size=9):
        # Explicit paths avoid the legacy 'New' name records and font-cache
        # ambiguity. Strings containing unsupported glyphs use DejaVu Sans.
        if self.mpl_available and self.supports(text):
            return font_manager.FontProperties(fname=str(FONT_PATH), size=size)
        return font_manager.FontProperties(family="DejaVu Sans", size=size)


class TechnicalLabel(tk.Canvas):
    """Single-line chrome label with explicit glyph fallback.

    Tk wrongly treats some missing ASCII glyphs in this old font as supported.
    Split the text by its real cmap, instead of trusting automatic fallback.
    This is a label only; it owns no plot, data or navigation state.
    """
    # Defaults equal THEME.background / THEME.text; ui_theme imports this
    # module, so the tokens cannot be imported here. Callers pass their colours.
    def __init__(self, parent, text, fonts, size=11, background="#1F2024",
                 foreground="#DDD9D6", padx=6, pady=3, width=None, tabular=False):
        """width: fixed pixel width (readouts must not resize their bar).
        tabular: digits on equal advances, as the font's own digits are proportional."""
        self._text, self._foreground = text, foreground
        self._fonts = fonts
        self._fixed_width, self._tabular = width, tabular
        self._technical = tkfont.Font(parent, family=FAMILY, size=size)
        # Segoe UI capitals are ~0.7 em, so the fallback is enlarged until its
        # capitals match this face's; regular weight suits its light strokes.
        self._fallback = tkfont.Font(parent, family=BODY_FAMILY,
                                     size=max(8, round(size * fonts.cap_ratio / 0.7)))
        self._cap = round(parent.winfo_fpixels(f"{size}p") * fonts.cap_ratio)
        self._padx, self._pady = padx, pady
        super().__init__(parent, background=background, highlightthickness=0,
                         borderwidth=0, takefocus=False, width=1, height=1)
        self._draw_text()

    def _draw_text(self):
        self.delete("label")
        runs = []
        for char in self._text:
            # The face has capitals only; in readouts a lowercase unit ("m")
            # must not become "M" (mega), so it uses the fallback face.
            custom = (self._fonts.tk_available and ord(char) in self._fonts.codepoints
                      and not (self._tabular and char.islower()))
            if runs and runs[-1][0] == custom:
                runs[-1][1] += char
            else:
                runs.append([custom, char])
        # Lay out on the measured cap height, not Tk's padded line metrics (the
        # face reports a 2-em line): one shared baseline, room for a slash's tail.
        baseline = self._pady + self._cap + 1
        tail = max(3, self._cap // 4)
        x = self._padx
        digit = max(self._technical.measure(d) for d in "0123456789") if self._tabular else 0
        for custom, text in runs:
            font = self._technical if custom else self._fallback
            y = baseline - font.metrics("ascent")
            if custom and self._tabular:
                for char in text:  # centre each glyph in an equal cell
                    advance = digit if char.isdigit() else font.measure(char)
                    self.create_text(x + advance / 2, y, text=char, font=font, anchor="n",
                                     fill=self._foreground, tags="label")
                    x += advance
                continue
            self.create_text(x, y, text=text, font=font, anchor="nw",
                             fill=self._foreground, tags="label")
            x += font.measure(text)
        width = self._fixed_width or x + self._padx
        super().configure(width=width, height=baseline + tail + self._pady)

    def configure(self, cnf=None, **kwargs):
        if cnf is not None:
            return super().configure(cnf, **kwargs)
        text = kwargs.pop("text", None)
        colour = kwargs.pop("foreground", kwargs.pop("fg", None))
        if colour is not None:
            self._foreground = colour
            self.itemconfigure("label", fill=colour)
        if text is not None and text != self._text:
            self._text = text
            self._draw_text()
        return super().configure(**kwargs) if kwargs else None

    config = configure

    def cget(self, key):
        if key == "text":
            return self._text
        if key in ("foreground", "fg"):
            return self._foreground
        return super().cget(key)
