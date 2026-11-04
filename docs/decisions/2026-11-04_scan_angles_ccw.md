# Decision: scan angles in SCAN_CHUNK are counter-clockwise (LD06 library convention)

Date: 2026-11-04. Status: Approved by the user (findings report, recommendation 3, option A).

**What:**
- `SCAN_CHUNK` point angles are in 0.01° units, **counter-clockwise** from the sensor's +X
  (front), within [0, 36000). This is the Robot-Maker LD06 library's `DataPoint.angle`:
  `360 − LD06 physical angle`.
- The host converts with `x = d·cos(a)`, `y = d·sin(a)` (+X front, +Y left), in
  `autonomy.protocol.points_to_sensor_xy`.

**Why:**
- The original TP firmware already sends exactly this, and the robot's existing data uses it.
- It is the convention of Code 01's CSV path.
- The host conversion is trivial.

**Alternative:** raw clockwise packet angles with our own parser. Not chosen: it would diverge
from the proven robot data.

**Note:** the library spreads points as `start + (i + 0.5)·step`, while Code 01's packet decoder
uses `start + i·step`. The angles therefore differ by half a step (about 0.4°) from Code 01's
recordings. This is negligible for mapping, but it is recorded here.
