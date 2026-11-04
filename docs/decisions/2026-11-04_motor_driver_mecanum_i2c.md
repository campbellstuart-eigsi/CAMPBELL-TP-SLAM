# Decision: the physical motor driver targets the MecanumCar_v2 I2C coprocessor; mock until H2

Date: 2026-11-04. Status: Approved by the user (findings report, recommendations 1, 2 and 6).

**What:**
- A future `MecanumI2cDriver` sits behind the existing `MotorDriver` interface. It uses the
  register map read from the MecanumCar_v2 source:
  - I2C address 0x30 on SDA = GPIO19, SCL = GPIO21;
  - PWM registers 1-8, two per wheel;
  - LED registers 0x09 and 0x0A.
- Unlike the original library, it must:
  - check every ACK, latching DRIVER_FAULT on a NACK or bus error;
  - periodically re-write zero PWM while the robot is not moving;
  - refuse to run without a measured PWM → speed calibration table.
- Body velocity is mapped to the wheel patterns of `Advance`, `Back`, `Turn_Left` and
  `Turn_Right`. Strafing is not in protocol v2.
- It stays compiled out (`MOTOR_OUTPUTS_ENABLED 0`, `HARDWARE_VERIFIED 0`). The mock driver
  remains the only driver until H2 is approved.

**H2 prerequisite (needs the user, wheels off the ground):** observe what the coprocessor does
when the ESP32 resets or is held in reset during a move. If it keeps the last PWM, a hardware stop
path is required before any autonomous motion.

**Why:** this is the robot's actual motor interface, and the original library has no failure
detection.

**Alternative:** reuse the library verbatim. Rejected: it ignores ACKs, uses blocking bit-bang
I2C, and its member names shadow the ESP32 core's `SDA`/`SCL`.
