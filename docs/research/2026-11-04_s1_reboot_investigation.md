# S1 reboot loop and wheel incident: investigation (offline)

Date: 2026-11-04. Branch `feature/http-robot-mvp`. Nothing was flashed, no port was opened, no
motor was powered. Firmware under investigation: `ESP32SlamRobot.ino` as committed in
**221c778** (what was uploaded at S1, as reported).

## 1. Observations (user-reported, S1)
- The upload succeeded. The Serial Monitor at 115200 showed repeated boots with
  `rst:0xc (SW_CPU_RESET), boot:0x13 (SPI_FAST_FLASH_BOOT)`, then the flash loading messages,
  repeating.
- The access point (SSID set in `wifi_secrets.h`) never appeared.
- One wheel spun **before and after** the upload, then stopped.
- The Python controller was never connected to the robot.
- **Not captured:** the serial lines printed just before each `rst:` line. These would name
  the cause.

## 2. Hardware facts
| Fact | Source | Status |
|---|---|---|
| The motors are driven by an I²C motor controller, not by direct PWM from the ESP32 | user, from the professor (2026-11-04) | **confirmed** |
| Address 0x30, SDA GPIO19, SCL GPIO21, PWM registers 1-8 | `MecanumCar_v2.cpp` (course library) | code only, **not checked on the robot** |
| Register pairs {reverse, forward}: front-left 3/4, front-right 1/2, rear-left 7/8, rear-right 5/6 | `Motor_Upper_L/R`, `Motor_Lower_L/R` in the library; the sketch's `WHEEL_REG` matches | code only; which motor is "upper" = front is an **assumption** |
| LD06 on Serial2 RX GPIO33 at 230400, PWM GPIO32 HIGH | course sketch and notes | code only |
| What the motor controller does when the ESP32 resets, hangs or stops writing | — | **unknown** |
| Whether a physical cut-off removes motor power independently of the ESP32 | — | **unknown** |

The copies of `MecanumCar_v2.cpp/.h` in the sketch folder are byte-identical to
`MecanumCar_v2.zip` (SHA-256 in `firmware/ESP32SlamRobot/DEPENDENCIES.md`).

## 3. Reboot loop: findings
`rst:0xc (SW_CPU_RESET)` is the ROM's line for a **software** reset. On this core
(sdkconfig of esp32 3.3.12: `CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT`,
`CONFIG_ESP_TASK_WDT_PANIC`, `CONFIG_ESP_BROWNOUT_USE_INTR`), these all produce it:
- a panic or crash;
- a task-watchdog timeout;
- **a brownout**, which here is handled by an interrupt that restarts the CPU;
- `esp_restart()`.

So the reset line alone cannot tell them apart.

**Most likely cause (code evidence; to be confirmed by the serial log).** A watchdog conflict
the sketch introduced:
1. `setup()` called `esp_task_wdt_reconfigure({1000 ms, panic})` on the **shared** task
   watchdog.
2. AsyncTCP 3.5.0 subscribes its own `async_tcp` task to that same watchdog
   (`CONFIG_ASYNC_TCP_USE_WDT` defaults to 1). When idle, it sleeps for up to
   `ASYNC_TCP_MAX_TASK_SLEEP = 1000 * CONFIG_ESP_TASK_WDT_TIMEOUT_S / 4` ms between resets.
   The core's `CONFIG_ESP_TASK_WDT_TIMEOUT_S` is 5, which gives **1250 ms**
   (`AsyncTCP.cpp` lines 62-66 and 331-351).
3. `server.begin()` (the last step of `setup()`) starts that task. With no network traffic it
   would sleep for 1250 ms, longer than the 1000 ms timeout, so the watchdog panics. The CPU
   restarts and the loop repeats.
4. This is consistent with the AP "never appearing": it would live about 1 s per boot.

**Competing cause:** a brownout. A spinning motor and Wi-Fi start-up both draw current. Only
the serial log can tell: `Brownout detector was triggered` versus
`task_wdt: Task watchdog got triggered … async_tcp`.

**Checked and judged not the cause, from the code:**
- `car.Stop()` before `Serial.begin()`: bit-banged I²C writes of microseconds, no waits.
  Intended: the motors are zeroed first.
- LiDAR start-up: calling `Serial2.begin()` twice keeps RX on GPIO33. The core's
  `HardwareSerial::begin` reuses pins it has already set (core 3.3.12, `HardwareSerial.cpp`
  line 372).
- The loop task: `enableLoopWDT()` is called last, and the core resets that watchdog before
  every `loop()`.

**Static regression check:** `test_firmware_static.py`. It passes on the new sketch and
fails on 221c778 (the watchdog check, among others).

## 4. Changes in the diagnostic build (COMPILE-VERIFIED / HARDWARE-UNTESTED)
1. **The shared task watchdog keeps the core's 5 s** (no reconfigure). The fast motor check is
   now its own: the motor task writes a heartbeat every 10 ms. `loop()` reboots the ESP32 if
   the heartbeat is older than **300 ms** (it was 1 s), and `setup()` then zeroes the motors
   first. The motor task is still on the shared watchdog as a 5 s backstop. **No safety
   check was removed.**
2. **Boot diagnostics:**
   - numbered `[BOOT]` checkpoints (motors zeroed, serial, motor task, LiDAR UART, Wi-Fi AP,
     HTTP server, setup complete, first LiDAR scan);
   - the reset reason by name (`esp_reset_reason()`: PANIC, TASK_WDT, BROWNOUT, SW, …);
   - the boot count since power-on, and the **last checkpoint the previous boot reached**
     (RTC memory, survives a software reset). A count of 3 or more prints
     `RESET LOOP`.
3. **Motor ACK at boot:** the zero-PWM write is repeated with its I²C ACK bits checked. A
   missing ACK prints "the motors may not be stopped - cut motor power". `/status` has
   `motor_ack`.
4. **Safe failures:**
   - motor task not created: `/arm` answers 503;
   - access point failed: logged, the robot stays disarmed;
   - no LiDAR scan within 3 s: warning (moves were already refused with NO_LIDAR).
5. **`MOTORS_LOCKED 1`:** `/arm` answers 423 `MOTORS_LOCKED`, so no route can move a wheel.
   Zero-PWM writes still run. Set it to 0 only after the S1 cut-off and stopping checks.

## 5. Reading the serial log at the next session
- **Healthy boot:**
  - `=== ESP32SlamRobot: COMPILE-VERIFIED, HARDWARE-UNTESTED ===`;
  - `[BOOT] reset reason: POWERON …`;
  - checkpoints 1 to 7;
  - `AP IP: 192.168.4.1`;
  - within seconds, `8 running: first LiDAR scan: N points`.

  Nothing repeats.
- **Reset loop:** `rst:0xc` lines keep coming, `boot N since power-on` climbs, and the
  `RESET LOOP` warning appears from boot 3. The cause is **in the lines just before each
  `rst:`**, and in the next boot's `reset reason` and `previous boot reached checkpoint`:

  | You see | Meaning |
  |---|---|
  | `Brownout detector was triggered`; reason BROWNOUT | supply sag: battery, motor current, wiring |
  | `task_wdt: Task watchdog got triggered` + task names; reason TASK_WDT | a named task blocked |
  | `Guru Meditation Error` / `abort()` + backtrace; reason PANIC | a crash: keep the backtrace |
  | `[FATAL] motor task silent`; reason SW | the motor task stalled (I²C) |

  "Previous boot reached checkpoint 4" means it died during Wi-Fi start-up; "6" means right
  after the HTTP server started; and so on.
- For more detail, set Tools > Core Debug Level to **Info** for this session only.

## 6. Wheel incident: separate hardware safety issue
- The wheel spun **before** the upload too, so it was not caused by `ESP32SlamRobot.ino`.
  Every boot of this sketch writes zero PWM before anything else.
- It kept spinning **while** the new sketch was looping. Possible explanations, none
  confirmed:
  - the controller did not take the zero writes (wrong bus or pins, no ACK, controller not
    powered from the same supply, or a fault);
  - the motor or driver channel is faulty;
  - the controller holds its last PWM, and the writes were not reaching it.
- **Code cannot prove that motor power is off during an ESP32 reset.** The diagnostic build
  only reports whether the controller *ACKs* the zero write.
- **Required before any physical movement:**
  1. a physical cut-off that removes motor power independently of the ESP32;
  2. a test that it stops a spinning wheel;
  3. with the wheels in the air and motor power on, reset the ESP32 and observe whether the
     controller holds PWM.

  Until then the firmware stays `MOTORS_LOCKED 1`.
