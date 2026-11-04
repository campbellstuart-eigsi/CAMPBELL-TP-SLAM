# Decision: reuse the Robot-Maker LD06 library for acquisition

Date: 2026-11-04. Status: Approved by the user (findings report, recommendation 3).
**Open point:** the licence. The README says MIT, but there is no LICENSE file.

**What:**
- The new firmware acquires scans with the Robot-Maker LD06 library. The copy in Downloads is
  byte-identical to `SLAM\LD06.zip`.
- It uses the robot's proven settings: Serial2 at 230400 baud, RX = GPIO33, PWM pin GPIO32 held
  HIGH, full-scan mode.
- The library is an **external dependency**, installed in the Arduino libraries folder. **It is
  not copied into this repository** until the licence is confirmed.
- The earlier plan for our own LD06 parser (`ld06_parser.h/.cpp`) is dropped.

**Why:** it is proven on this robot and avoids re-implementing tested code.

**Alternative:** our own parser, ported from Code 01. It stays possible if the licence turns out
to be unsuitable.
