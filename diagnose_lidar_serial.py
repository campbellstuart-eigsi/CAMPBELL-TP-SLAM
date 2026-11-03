"""Independent, READ-ONLY serial diagnostic for the ESP32 + LD06 robot.

Not part of the GUI and imports nothing from Codes 01-04 (only pyserial).
It answers ONE question: what does the ESP32 actually send over USB?

    python diagnose_lidar_serial.py                 # list ports, choose one
    python diagnose_lidar_serial.py COM7            # observe COM7 for 15 s
    python diagnose_lidar_serial.py COM7 --seconds 30 --raw
    python diagnose_lidar_serial.py --list          # only list the ports
    python diagnose_lidar_serial.py COM7 --reset-on-open

SAFE MODE (the default): the port is opened with DTR and RTS held inactive,
exactly like Code 01, so the ESP32 is NOT restarted, and NOTHING is ever
written to the port in any mode (no write() call exists in this file).

--reset-on-open is an explicit opt-in: it pulses RTS (the ESP32 EN/reset
line on CH340 auto-reset boards) the same way the Arduino IDE or Teleplot
do, to show the complete boot log and what the firmware prints just after
booting. This is the equivalent of pressing the board's EN/RST button; it
does not program the ESP32 (DTR/GPIO0 stays inactive, so it boots normally).
"""
import argparse
import math
import sys
import time

import serial
from serial.tools import list_ports

DEFAULT_BAUD = 115200        # Serial.begin(115200) in the LD06 example sketch
DEFAULT_SECONDS = 15.0
READ_TIMEOUT_S = 0.1
LD06_PACKET_BYTES = 47       # 0x54 0x2C + 44 bytes + CRC8 (ld06.h)
LD06_CRC_POLYNOMIAL = 0x4D   # generates CrcTable[] in ld06crc.h

KNOWN_USB_CHIPS = {
    "10C4:EA60": "Silicon Labs CP210x (ESP32 board)",
    "1A86:7523": "WCH CH340 (ESP32 board - the robot used so far)",
    "1A86:55D4": "WCH CH9102 (ESP32 board)",
    "0403:6001": "FTDI USB-UART",
    "303A:1001": "Espressif native USB",
}

# Text printed by the ESP32 ROM bootloader / ESP-IDF, never by the LiDAR.
BOOT_PREFIXES = ("rst:", "ets ", "configsip", "clk_drv", "mode:", "load:",
                 "entry ", "boot:", "ho ", "Brownout", "Guru Meditation",
                 "Rebooting", "Backtrace", "E (", "W (", "I (")
BOOT_WORDS = ("POWERON_RESET", "SPI_FAST_FLASH_BOOT", "DOWNLOAD_BOOT",
              "SW_CPU_RESET", "RTCWDT", "TG1WDT", "TG0WDT", "BROWNOUT")


# ---------------------------------------------------------------------------
# Classification (pure functions, tested at the bottom of the file)
# ---------------------------------------------------------------------------
def ld06_crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ LD06_CRC_POLYNOMIAL) & 0xFF if crc & 0x80 \
                else (crc << 1) & 0xFF
    return crc


def count_ld06_packets(data):
    """Count raw LD06 binary packets with a valid CRC (LiDAR wired to USB)."""
    valid = 0
    start = data.find(b"\x54\x2c")
    while start != -1 and start + LD06_PACKET_BYTES <= len(data):
        packet = data[start:start + LD06_PACKET_BYTES]
        if ld06_crc8(packet[:-1]) == packet[-1]:
            valid += 1
            start = data.find(b"\x54\x2c", start + LD06_PACKET_BYTES)
        else:
            start = data.find(b"\x54\x2c", start + 1)
    return valid


def csv_column_count(text):
    """4 or 6 if `text` is a numeric ld06.printScanCSV point row, else 0."""
    fields = text.split(",")
    if len(fields) not in (4, 6):
        return 0
    try:
        values = [float(field) for field in fields]
    except ValueError:
        return 0
    return len(fields) if all(math.isfinite(v) for v in values) else 0


def classify_line(line):
    """Return the kind of one decoded text line (no line ending)."""
    text = line.strip()
    if not text:
        return "empty"           # also the end of a CSV scan (printScanCSV)
    if text.startswith(">lidar:"):
        return "lidar_xy" if text.rstrip().endswith("|xy") else "lidar_partial"
    if text.startswith(">"):
        return "teleplot_other"
    if text.startswith("N,Angle"):
        return "lidar_csv_header"
    columns = csv_column_count(text)
    if columns:
        return f"lidar_csv{columns}"
    if text.startswith(BOOT_PREFIXES) or any(w in text for w in BOOT_WORDS):
        return "esp32_boot"
    if "AP IP" in text or "WiFi" in text or "192.168." in text:
        return "firmware_wifi"
    printable = sum(1 for ch in text if " " <= ch <= "~")
    if printable < 0.9 * len(text):
        return "garbled"
    return "firmware_text"


def new_stats():
    return {"bytes": 0, "lines": 0, "kinds": {}, "first_byte_s": None,
            "last_byte_s": None, "ld06_packets": 0, "csv_scans": 0}


def describe(stats, duration_s, baud):
    """Return (verdict, explanation) from the collected statistics."""
    kinds = stats["kinds"]
    if stats["bytes"] == 0:
        return ("SILENCE", f"0 bytes in {duration_s:.0f} s. The port opened, so "
                "the COM number exists and is free, but the running firmware "
                "sent nothing. With DTR/RTS held inactive the ESP32 was not "
                "restarted, so no boot text is expected either. Either the "
                "firmware does not stream over USB serial, or it only prints "
                "when a LiDAR scan completes (LD06 not powered/spinning). "
                "Try --reset-on-open to see what the firmware prints at boot.")
    if kinds.get("lidar_xy"):
        return ("LIDAR TELEMETRY", f"{kinds['lidar_xy']} complete '>lidar:...|xy' "
                "scan lines (Teleplot XY, ld06.printScanTeleplot): Codes 01-06 "
                "accept this format. If they still fail, the problem is in "
                "Python, not the robot.")
    if stats["csv_scans"]:
        columns = ("6 columns with x/y" if kinds.get("lidar_csv6", 0) >= kinds.get("lidar_csv4", 0)
                   else "4 columns without x/y")
        return ("LIDAR TELEMETRY", f"{stats['csv_scans']} complete CSV scans "
                f"(ld06.printScanCSV, {columns}; an empty line ends each scan): "
                "Codes 01-06 accept this format. If they still fail, the "
                "problem is in Python, not the robot.")
    if stats["ld06_packets"] >= 2:
        return ("RAW LD06 PACKETS", "Binary LD06 packets (0x54 0x2C, valid CRC) "
                "are arriving: the LiDAR is wired straight to USB or the "
                "firmware forwards raw bytes. Codes 01-06 expect Teleplot "
                "or CSV text, not raw packets (and normally at 230400 baud).")
    if kinds.get("lidar_partial"):
        return ("PARTIAL LIDAR LINES", "'>lidar:' lines arrive but none ends "
                "with '|xy' - lines are being cut (USB/cable/buffer issue).")
    if kinds.get("lidar_csv6") or kinds.get("lidar_csv4"):
        return ("PARTIAL LIDAR LINES", "LD06 CSV rows arrive but no empty "
                "line ends a scan - observe longer, or data is being lost.")
    if kinds.get("garbled") and kinds["garbled"] >= stats["lines"] / 2:
        return ("GARBLED", f"Mostly unreadable bytes at {baud} baud: wrong baud "
                "rate, or electrical noise.")
    if kinds.get("esp32_boot") or kinds.get("firmware_wifi") or kinds.get("firmware_text"):
        quiet = (stats["last_byte_s"] is not None
                 and duration_s - stats["last_byte_s"] > 3.0)
        tail = (f" The stream then went SILENT for the last "
                f"{duration_s - stats['last_byte_s']:.0f} s." if quiet else "")
        wifi = (" The firmware starts a Wi-Fi access point; the LD06 example "
                "sketch (Basic.ino) contains no Wi-Fi code, so this is a "
                "DIFFERENT program from the LiDAR firmware Codes 01-06 "
                "expect. Its data may go over Wi-Fi instead of USB."
                if kinds.get("firmware_wifi") else "")
        return ("ESP32 TEXT, NO LIDAR DATA", "Only boot/firmware messages, no "
                f"'>lidar:' line or CSV scan.{tail}{wifi} This is a firmware/robot-state "
                "issue, not a Python parser issue.")
    return ("OTHER TEXT", "Text that is neither boot messages nor Teleplot.")


# ---------------------------------------------------------------------------
# Serial access
# ---------------------------------------------------------------------------
def list_serial_ports():
    ports = sorted(list_ports.comports(), key=lambda p: p.device)
    if not ports:
        print("  No serial port detected. Plug the ESP32 in with a DATA cable "
              "(some USB-C cables are charge-only) and check Device Manager "
              "> Ports (COM & LPT) for 'USB-SERIAL CH340'.")
    for port in ports:
        usb_id = f"{port.vid:04X}:{port.pid:04X}" if port.vid is not None else "not USB"
        print(f"  {port.device:6s} {port.description} [{usb_id}] "
              f"-> {KNOWN_USB_CHIPS.get(usb_id, 'unknown chip')}"
              f"{'  location ' + port.location if port.location else ''}")
    return [port.device for port in ports]


def choose_port():
    likely = [p.device for p in list_ports.comports()
              if p.vid is not None and f"{p.vid:04X}:{p.pid:04X}" in KNOWN_USB_CHIPS]
    default = likely[0] if len(likely) == 1 else None
    prompt = f"Port to observe [{default}]: " if default else "Port to observe (e.g. COM7): "
    try:
        answer = input(prompt).strip()
    except EOFError:
        answer = ""
    return answer or default


def open_port(port, baud, reset_on_open):
    connection = serial.Serial()          # not opened yet
    connection.port = port
    connection.baudrate = baud
    connection.bytesize, connection.parity, connection.stopbits = 8, "N", 1
    connection.timeout = READ_TIMEOUT_S
    connection.xonxoff = connection.rtscts = connection.dsrdtr = False
    connection.dtr = False                # set BEFORE open(): never asserted
    connection.rts = False
    connection.open()
    if reset_on_open:                     # same pulse as Arduino IDE "reset"
        connection.rts = True             # EN low -> ESP32 held in reset
        time.sleep(0.1)
        connection.rts = False            # EN high -> normal boot (GPIO0 untouched)
    return connection


def show_chunk(chunk, elapsed, raw):
    if raw:
        print(f"[{elapsed:6.2f} s] {len(chunk):4d} B  hex: {chunk[:48].hex(' ')}"
              f"{' ...' if len(chunk) > 48 else ''}")


def observe(port, baud, seconds, raw, reset_on_open, max_line_chars=160):
    stats = new_stats()
    buffer = bytearray()
    csv_rows, csv_first = 0, None         # CSV rows of the scan in progress
    connection = None
    print(f"\nOpening {port}: {baud} baud, 8N1, no flow control, DTR=off, RTS=off, "
          f"{'RESET PULSE ON OPEN' if reset_on_open else 'SAFE MODE (no reset)'}; "
          "nothing will be transmitted.")
    start = time.monotonic()
    try:
        connection = open_port(port, baud, reset_on_open)
        print(f"Opened. Listening for {seconds:.0f} s (Ctrl+C to stop early)...\n")
        start = time.monotonic()
        while (elapsed := time.monotonic() - start) < seconds:
            chunk = connection.read(4096)
            if not chunk:
                continue
            if stats["first_byte_s"] is None:
                stats["first_byte_s"] = elapsed
            stats["last_byte_s"] = elapsed
            stats["bytes"] += len(chunk)
            stats["ld06_packets"] += count_ld06_packets(chunk)
            show_chunk(chunk, elapsed, raw)
            buffer.extend(chunk)
            while (end := buffer.find(b"\n")) != -1:
                line = bytes(buffer[:end]).decode("utf-8", errors="backslashreplace")
                del buffer[:end + 1]
                kind = classify_line(line)
                if kind == "empty":
                    if csv_rows:          # printScanCSV: empty line = scan end
                        stats["csv_scans"] += 1
                        print(f"[{elapsed:6.2f} s] {'lidar_csv_scan':15s}| "
                              f"first row {csv_first!r} ... ({csv_rows} points)")
                    csv_rows, csv_first = 0, None
                    continue
                stats["lines"] += 1
                stats["kinds"][kind] = stats["kinds"].get(kind, 0) + 1
                shown = line.strip()
                if kind in ("lidar_csv6", "lidar_csv4"):
                    csv_rows += 1         # one line per point: not printed
                    csv_first = csv_first or shown
                    continue
                if kind == "lidar_xy":
                    shown = f"{shown[:60]} ... ({shown.count(';')} points)"
                print(f"[{elapsed:6.2f} s] {kind:15s}| {shown[:max_line_chars]}")
            if len(buffer) > 65536:       # binary stream: no line endings
                buffer.clear()
        duration = seconds
    except KeyboardInterrupt:
        duration = time.monotonic() - start
        print("\nStopped by user.")
    except serial.SerialException as error:
        message = str(error)
        if "PermissionError" in message or "Access is denied" in message:
            print(f"ACCESS DENIED: {port} is used by another program (Teleplot, "
                  "Arduino Serial Monitor, PyCharm run of Code 01/00, another "
                  f"Python). Close it and retry.\n  ({message})")
            return "ACCESS DENIED"
        if "FileNotFoundError" in message or "cannot find" in message.lower():
            print(f"PORT NOT FOUND: {port} does not exist right now. Windows gives "
                  "the CH340 a different COM number for each USB socket - use "
                  f"the list above.\n  ({message})")
            return "PORT NOT FOUND"
        if connection is not None and connection.is_open:
            print(f"DISCONNECTED during reading (cable, power, or ESP32 brown-out)."
                  f"\n  ({message})")
            return "DISCONNECTED"
        print(f"CONNECTION ERROR opening {port}: {message}")
        return "CONNECTION ERROR"
    finally:
        if connection is not None:
            connection.close()
            print(f"\n{port} closed.")

    if buffer:
        print(f"(+{len(buffer)} bytes after the last line ending, not shown)")
    verdict, explanation = describe(stats, max(duration, 0.1), baud)
    first = "-" if stats["first_byte_s"] is None else f"{stats['first_byte_s']:.2f} s"
    last = "-" if stats["last_byte_s"] is None else f"{stats['last_byte_s']:.2f} s"
    print("\n=== Summary ===")
    print(f"  bytes received : {stats['bytes']} ({stats['bytes'] / max(duration, 0.1):.0f} B/s)")
    print(f"  first / last   : {first} / {last}")
    print(f"  text lines     : {stats['lines']} {stats['kinds']}")
    print(f"  CSV scans      : {stats['csv_scans']} (ld06.printScanCSV, ended by an empty line)")
    print(f"  LD06 packets   : {stats['ld06_packets']} (raw binary, valid CRC)")
    print(f"\n  VERDICT: {verdict}\n  {explanation}")
    return verdict


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("port", nargs="?", help="e.g. COM7 (asked if omitted)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--seconds", type=float, default=DEFAULT_SECONDS)
    parser.add_argument("--raw", action="store_true", help="also print every chunk in hex")
    parser.add_argument("--list", action="store_true", help="only list the ports")
    parser.add_argument("--reset-on-open", action="store_true",
                        help="opt-in: pulse RTS to restart the ESP32 (like the EN button)")
    args = parser.parse_args(argv)

    print(f"pyserial {serial.__version__}, Python {sys.version.split()[0]}")
    print("Detected serial ports:")
    list_serial_ports()
    if args.list:
        return 0
    port = args.port or choose_port()
    if not port:
        print("No port chosen.")
        return 1
    verdict = observe(port, args.baud, args.seconds, args.raw, args.reset_on_open)
    return 0 if verdict == "LIDAR TELEMETRY" else 2


if __name__ == "__main__":
    sys.exit(main())
