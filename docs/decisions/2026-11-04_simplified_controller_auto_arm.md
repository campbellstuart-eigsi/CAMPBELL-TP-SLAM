# Decision: a simplified robot controller (connect, drive, stop) with automatic arming

Date: 2026-11-04. Status: Requested by the user ("connect, drive, stop"; no ARM/DISARM buttons,
no manual motor-lock settings in the normal interface). Implemented, uncommitted at the time of
writing.

**What:**
- `robot_controller.py` opens in **REAL ROBOT** mode (SIMULATOR is one click away). Launching
  never connects or arms anything.
- The user clicks Connect, then holds Z/Q/S/D or A/E to drive. Releasing stops, and Space or
  Esc is STOP.
- **No ARM or DISARM buttons, no bench-safety checkboxes.** The first *new* movement key press
  while stopped sends `/arm` on the worker thread, then drives. If the firmware refuses (for
  example `MOTORS_LOCKED`), the controller shows `cannot drive: MOTORS_LOCKED (HTTP 423)`.
- A key held through a STOP (or through a link loss or robot restart) must be **released**
  before it can drive again, so keyboard auto-repeat never restarts the robot.
- A refused automatic ARM is not retried by auto-repeat, and at most once per second.
- A STOP pressed while an automatic `/arm` is in flight ends with the robot disarmed: the
  controller sends `/stop` again.

**Safety stays in the firmware, unchanged:**
- disarmed at boot, and `/stop` disarms;
- every move is a lease of at most 1 s (the controller sends 300 ms leases);
- stale or old-seq commands are refused, and the seq is kept across `/arm`;
- moves stop on a stale LiDAR or a Wi-Fi client drop;
- the 300 ms motor-task stall check and the 5 s task watchdog;
- **`MOTORS_LOCKED 1`**: physical driving stays disabled until the unexplained S1 wheel spin
  and the motor-power cut-off are checked (`docs/research/2026-11-04_s1_reboot_investigation.md`).

**Why:** the user wants the simplest possible experience. Arming is an internal state the
firmware already enforces; the button and the checkboxes added clicks without adding a check
the firmware does not already make.

**Alternatives:**
- Keep the ARM button (the previous design): one more step for every drive.
- Auto-connect at launch: rejected. Launching must never connect to or arm the physical robot.
- Remove the firmware lock: rejected until the S1 hardware safety gate passes.

**Removed from the Python side:** the "bench safety" checkboxes that gated ARM in REAL ROBOT
mode. The physical checks they named (wheels raised, cut-off in reach, correct firmware) are
in the firmware README's S1 checklist and are enforced by `MOTORS_LOCKED` until done.
