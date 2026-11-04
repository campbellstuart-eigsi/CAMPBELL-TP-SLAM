# Decision: MVP on the HTTP firmware (approach B); protocol v2 firmware parked

Date: 2026-11-04. Status: Approved by the user ("Approve B, start S0").

**What:**
- The TP demonstration uses `firmware/ESP32SlamRobot/ESP32SlamRobot.ino`: the course sketch
  `ESP32RobotControl.ino` extended with ARM/STOP, one motor task, `/drive` leases of at most
  1 s, `/scan2` and `/status`. Python drives it through `autonomy/http_robot_link.py`
  (`HttpRobotLink`, a `RobotLink`), so the future explorer runs unchanged on `MockRobotLink`
  (simulation) or on the robot.
- Path and gates: S0 desk (compile, fake robot tests), S1 bench (wheels off the ground),
  S2 supervised manual driving and calibration, S3 drive-stop-scan-localise, S4 frontier
  exploration, S5 loading the run into the existing Workspace.
- The protocol v2 work stays as it is (commit 364982a on `feature/autonomy-simulator`). The
  untracked Step 3 C++ (`firmware/autonomy_robot/src/crc.*`, `protocol.*`) is parked,
  uncompiled.

**Why:** a practical TP demonstration, not a production platform. The sketch reuses the
robot's working motor and LiDAR code; the 1 s lease gives a dead-man stop without a custom
protocol. Approach A needed the rest of Step 3, a physical motor driver and H1-H3 before any
motion.

**Deferred:** protocol v2 framing, UDP, session binding, heartbeat echo, MOVE expiry, the C++
trace tests, the live dashboard, strafing, scanning while moving, LiDAR stop zones (until the
LiDAR front direction is measured), global relocalisation beyond a retry.

**Accepted risks (supervised use only):**
- A `/drive` delayed in TCP can still start a lease (at most 1 s) late.
- Anyone on the access point can drive the robot (CORS `*`, no authentication): use a unique
  password per robot.
- Whether the motor board holds its last PWM when the ESP32 resets or hangs is unknown: S1 must
  observe it before any floor test.

**Alternative:** approach A (custom protocol v2 firmware). Kept on its branch, not deleted.
