"""Static checks of firmware/ESP32SlamRobot/ESP32SlamRobot.ino (no compiler, no hardware).

Guards the S1 reboot-loop finding (docs/research/2026-11-04_s1_reboot_investigation.md):
AsyncTCP's task sleeps up to 1000 * CONFIG_ESP_TASK_WDT_TIMEOUT_S / 4 ms between
watchdog resets, so the sketch must not shorten the SHARED task watchdog below that.
Run: .\\.venv\\Scripts\\python.exe -m unittest test_firmware_static -v
"""
import os
import re
import unittest
from pathlib import Path

SKETCH = Path(__file__).resolve().parent / "firmware" / "ESP32SlamRobot" / "ESP32SlamRobot.ino"
SDKCONFIG = (Path(os.environ.get("LOCALAPPDATA", "")) / "Arduino15" / "packages" / "esp32"
             / "tools" / "esp32-libs" / "3.3.12" / "qio_qspi" / "include" / "sdkconfig.h")


def source():
    return SKETCH.read_text(encoding="utf-8")


def function_body(text, signature):
    start = text.index(signature)
    depth, i = 0, text.index("{", start)
    for j in range(i, len(text)):
        depth += {"{": 1, "}": -1}.get(text[j], 0)
        if depth == 0:
            return text[i + 1:j]
    raise ValueError(signature)


class WatchdogTests(unittest.TestCase):
    def test_shared_task_watchdog_is_not_shortened(self):
        text = source()
        self.assertNotIn("esp_task_wdt_reconfigure", text)
        self.assertNotIn("esp_task_wdt_init", text)

    @unittest.skipUnless(SDKCONFIG.is_file(), "ESP32 core 3.3.12 not installed")
    def test_core_watchdog_outlasts_asynctcp_sleep(self):
        timeout_s = int(re.search(r"#define CONFIG_ESP_TASK_WDT_TIMEOUT_S (\d+)",
                                  SDKCONFIG.read_text()).group(1))
        asynctcp_sleep_ms = 1000 * timeout_s / 4
        self.assertGreater(timeout_s * 1000, asynctcp_sleep_ms)

    def test_motor_stall_check_is_fast_and_reboots(self):
        text = source()
        stall = int(re.search(r"MOTOR_STALL_MS = (\d+);", text).group(1))
        self.assertLessEqual(stall, 500)
        loop = function_body(text, "void loop()")
        self.assertIn("MOTOR_STALL_MS", loop)
        self.assertIn("esp_restart()", loop)
        self.assertIn("motorHeartbeatMs = millis()", function_body(text, "void motorTask("))


class BootSafetyTests(unittest.TestCase):
    def test_setup_zeroes_motors_first(self):
        body = function_body(source(), "void setup()")
        statements = [line.strip() for line in body.splitlines()
                      if line.strip() and not line.strip().startswith("//")]
        self.assertEqual(statements[0], "car.Stop();")
        self.assertLess(body.index("car.Stop()"), body.index("Serial.begin"))
        self.assertLess(body.index("writeWheels(zero)"), body.index("Serial.begin"))

    def test_diagnostic_build_locks_the_motors(self):
        text = source()
        self.assertRegex(text, r"#define MOTORS_LOCKED 1\b")
        arm = text[text.index('server.on("/arm"'):]
        self.assertLess(arm.index("MOTORS_LOCKED"), arm.index("armed = true"))

    def test_boot_checkpoints_cover_each_subsystem(self):
        text = source()
        for step in range(1, 9):
            self.assertRegex(text, rf"checkpoint\({step},")
        self.assertIn("esp_reset_reason()", function_body(text, "void setup()"))
        self.assertIn("RTC_NOINIT_ATTR uint32_t diagCheckpoint", text)


if __name__ == "__main__":
    unittest.main()
