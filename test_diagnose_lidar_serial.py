"""Offline tests for diagnose_lidar_serial.py. Run: python -m unittest test_diagnose_lidar_serial -v"""
import contextlib
import io
import unittest
from unittest import mock

import serial

import diagnose_lidar_serial as diag

BOOT_LOG = (b"ets Jul 29 2019 12:21:46\r\n\r\n"
            b"rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)\r\n"
            b"configsip: 0, SPIWP:0xee\r\nmode:DIO, clock div:1\r\n"
            b"load:0x3fff0030,len:1184\r\nentry 0x400805e4\r\n"
            b"AP IP: 192.168.4.1\r\n")
CSV_HEADER = "N,Angle(°),Distance(mm),Intensity,x(mm),y(mm)\r\n".encode("utf-8")
CSV_SCAN = b"".join(b"%d,%.2f,%d,229,%d,%d\r\n" % (i, i * 0.9, 1000 + i, 1000 + i, -i)
                    for i in range(400)) + b"\r\n"
SCAN_LINE = b">lidar:" + b"".join(b"%d:%d;" % (1000 + i, i) for i in range(400)) + b"|xy\r\n"


class FakePort:
    def __init__(self, chunks):
        self.chunks, self.is_open, self.written = list(chunks), True, []

    def read(self, size):
        return self.chunks.pop(0) if self.chunks else b""

    def write(self, data):  # must never be called
        self.written.append(data)

    def close(self):
        self.is_open = False


def run(chunks=(), error=None, seconds=0.3):
    port = FakePort(chunks)
    opener = mock.patch.object(diag, "open_port",
                               side_effect=error) if error else \
        mock.patch.object(diag, "open_port", return_value=port)
    with opener, contextlib.redirect_stdout(io.StringIO()) as out:
        verdict = diag.observe("COM9", 115200, seconds, raw=True, reset_on_open=False)
    return verdict, out.getvalue(), port


class ClassifierTests(unittest.TestCase):
    def test_line_kinds(self):
        self.assertEqual(diag.classify_line("rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)"), "esp32_boot")
        self.assertEqual(diag.classify_line("AP IP: 192.168.4.1"), "firmware_wifi")
        self.assertEqual(diag.classify_line(">lidar:1:2;3:4;|xy"), "lidar_xy")
        self.assertEqual(diag.classify_line(">lidar:1:2;3:"), "lidar_partial")
        self.assertEqual(diag.classify_line(">speed:12"), "teleplot_other")
        self.assertEqual(diag.classify_line("404,40.08,775,229,592,-499"), "lidar_csv6")
        self.assertEqual(diag.classify_line("404,40.08,775,229\r"), "lidar_csv4")
        self.assertEqual(diag.classify_line("N,Angle(°),Distance(mm),Intensity,x(mm),y(mm)"),
                         "lidar_csv_header")
        self.assertEqual(diag.classify_line("404,40.08,7x5,229,592,-499"), "firmware_text")
        self.assertEqual(diag.classify_line("1,2,nan,4"), "firmware_text")

    def test_ld06_crc_and_packets(self):
        body = bytes([0x54, 0x2C]) + bytes(range(44))
        packet = body + bytes([diag.ld06_crc8(body)])
        self.assertEqual(diag.count_ld06_packets(b"\x00\x54" + packet * 3), 3)
        self.assertEqual(diag.count_ld06_packets(packet[:-1] + b"\x00"), 0)


class ObserveTests(unittest.TestCase):
    def test_silence_is_not_a_parse_failure(self):
        verdict, out, port = run([])
        self.assertEqual(verdict, "SILENCE")
        self.assertIn("0 bytes", out)
        self.assertFalse(port.is_open)

    def test_boot_text_only(self):
        verdict, out, port = run([BOOT_LOG[:40], BOOT_LOG[40:]])
        self.assertEqual(verdict, "ESP32 TEXT, NO LIDAR DATA")
        self.assertIn("Wi-Fi access point", out)
        self.assertFalse(port.is_open)

    def test_lidar_telemetry_split_across_reads(self):
        stream = BOOT_LOG + SCAN_LINE * 2
        verdict, out, port = run([stream[i:i + 64] for i in range(0, len(stream), 64)])
        self.assertEqual(verdict, "LIDAR TELEMETRY")
        self.assertIn("(400 points)", out)

    def test_csv_scans_are_lidar_telemetry(self):
        verdict, out, _ = run([BOOT_LOG + CSV_HEADER + CSV_SCAN[:500], CSV_SCAN[500:] + CSV_SCAN])
        self.assertEqual(verdict, "LIDAR TELEMETRY")
        self.assertIn("2 complete CSV scans", out)
        self.assertIn("6 columns", out)
        self.assertIn("(400 points)", out)
        self.assertLess(out.count("\n"), 60)  # one line per scan, not per point

    def test_csv_rows_without_scan_end_are_partial(self):
        verdict, _, _ = run([CSV_SCAN[:-2]])
        self.assertEqual(verdict, "PARTIAL LIDAR LINES")

    def test_open_errors_are_distinct(self):
        denied = serial.SerialException(
            "could not open port 'COM9': PermissionError(13, 'Access is denied.', None, 5)")
        missing = serial.SerialException(
            "could not open port 'COM9': FileNotFoundError(2, 'The system cannot find the file specified.', None, 2)")
        self.assertEqual(run(error=denied)[0], "ACCESS DENIED")
        self.assertEqual(run(error=missing)[0], "PORT NOT FOUND")

    def test_disconnect_during_read_closes_port(self):
        port = FakePort([])
        port.read = mock.Mock(side_effect=serial.SerialException("ClearCommError failed"))
        with mock.patch.object(diag, "open_port", return_value=port), \
                contextlib.redirect_stdout(io.StringIO()):
            verdict = diag.observe("COM9", 115200, 0.3, False, False)
        self.assertEqual(verdict, "DISCONNECTED")
        self.assertFalse(port.is_open)

    def test_nothing_is_ever_written(self):
        _, _, port = run([BOOT_LOG, SCAN_LINE])
        self.assertEqual(port.written, [])
        with open(diag.__file__, encoding="utf-8") as handle:
            source = handle.read()
        self.assertNotIn(".write(", source)


if __name__ == "__main__":
    unittest.main()
