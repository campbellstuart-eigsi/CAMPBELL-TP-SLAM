"""Robot controller window: run this file (PyCharm: right-click > Run 'robot_controller').

1. Join the robot's Wi-Fi.  2. Run this file, click Connect.  3. Hold a key to
drive, release to stop; Space or Esc = STOP. Keys (AZERTY by default,
changeable in the window): Z/S forward/back, Q/D strafe, A/E rotate.
Opens in REAL ROBOT mode (SIMULATOR is one click away) and never connects or
arms anything by itself.
See autonomy/controller_ui.py and autonomy/teleop.py.
"""
import tkinter as tk

from autonomy.controller_ui import RobotControllerPanel
from ui_fonts import FontSet
from ui_theme import THEME, apply_styles


def main():
    root = tk.Tk()
    root.title("SLAM Robot Controller")
    root.geometry("1280x900")
    root.minsize(980, 820)
    root.configure(background=THEME.background)
    fonts = FontSet(root)
    FontSet.current = fonts
    apply_styles(root, fonts=fonts)
    panel = RobotControllerPanel(root, fonts=fonts)
    panel.pack(fill="both", expand=True)

    def on_close():
        panel.close()               # STOP / disarm, end the worker threads, stop the simulator
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == "__main__":
    main()
