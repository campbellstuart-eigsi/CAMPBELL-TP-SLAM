"""Headless checks of the UI theme and the plot-overlay palette (no Tk window).

Run: python -m unittest test_ui_theme -v
"""
import unittest

from matplotlib.colors import to_hex

from app_session import grid_localisation as loc
from ui_theme import PLOT, THEME, contrast


class PoseVocabularyTests(unittest.TestCase):
    """The Workspace pose colours mean what they mean on pages 04-06."""

    def same(self, a, b):
        self.assertEqual(to_hex(a).lower(), to_hex(b).lower())

    def test_matches_code04_and_page_colours(self):
        self.same(PLOT.recorded_pose, loc.DAY1_COLOR)        # recorded (Day 1) pose
        self.same(PLOT.initial_guess, loc.INITIAL_COLOR)     # initial guess
        self.same(PLOT.estimate_paper, loc.ALIGNED_COLOR)    # localised estimate
        self.same(PLOT.alternative_paper, "tab:purple")      # pages 05/06 alternative pose

    def test_each_meaning_has_its_own_colour(self):
        for face in ("paper", "dark"):
            colours = [PLOT.recorded_pose, PLOT.initial_guess, getattr(PLOT, f"estimate_{face}"),
                       getattr(PLOT, f"alternative_{face}"), getattr(PLOT, f"axis_{face}")]
            self.assertEqual(len({c.lower() for c in colours}), len(colours), face)


class ContrastTests(unittest.TestCase):
    def test_overlays_are_legible_on_their_plot_face(self):
        for face, background in (("paper", THEME.paper), ("dark", PLOT.dark_face)):
            for name in ("estimate", "alternative", "axis"):
                colour = getattr(PLOT, f"{name}_{face}")
                self.assertGreaterEqual(contrast(colour, background), 3.0, f"{name} on {face}")
        for colour in (PLOT.recorded_pose, PLOT.initial_guess):
            self.assertGreaterEqual(contrast(colour, PLOT.dark_face), 3.0, colour)

    def test_verdict_text_on_paper(self):
        for colour in (PLOT.verdict_ok, PLOT.verdict_weak, PLOT.verdict_bad):
            self.assertGreaterEqual(contrast(colour, THEME.paper), 4.4, colour)

    def test_chrome_text_pairs(self):
        t = THEME
        for fg, bg in ((t.text, t.panel), (t.text, t.background), (t.secondary, t.panel),
                       (t.secondary, t.raised), (t.lavender, t.panel), (t.sage, t.success_bg),
                       (t.warning, t.warning_bg), (t.danger, t.danger_bg),
                       (t.primary_text, t.primary)):
            self.assertGreaterEqual(contrast(fg, bg), 4.5, (fg, bg))


if __name__ == "__main__":
    unittest.main()
