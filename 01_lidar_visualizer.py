"""LD06 LiDAR acquisition and Cartesian visualization.

Current features:
    - simulate LD06 scans taken from any pose inside a rectangular room,
    - convert them to Cartesian coordinates (sensor frame),
    - transform them into one global map frame and display them together,
    - adjust the pose of each scan interactively with sliders,
    - decode raw LD06 binary packets offline (CRC, fields, angles, scans),
    - READ-ONLY serial diagnostic to identify what the ESP32 sends
      (set RUN_MODE = "serial_diagnostic"),
    - READ-ONLY manual capture of any number of real Teleplot XY scans
      (Scan 1, 2, 3, ...) from different robot positions, manual alignment,
      and save/load of the session as JSON in scans/
      (set RUN_MODE = "serial_capture"),
    - offline viewing/alignment of a saved session, no ESP32 needed
      (set RUN_MODE = "session_viewer").

Automated tests (no ESP32 needed), from the project folder:
    python -m unittest 01_lidar_visualizer -v

Both data sources (simulation and decoded LD06 packets) produce the same
RAW LD06 conventions described in the course document
"Acquisition des donnees issues du Lidar LD06":
    - angles in degrees, increasing CLOCKWISE seen from above (0 = front),
    - distances in millimetres (integers),
    - intensity between 0 and 255.
Note: the "Angle" column printed by the university LD06 Arduino library is
already converted to COUNTER-CLOCKWISE, so it must not be negated again.
"""

import json
import math
import queue
import re
import tempfile
import textwrap
import threading
import time
import unittest
from datetime import datetime
from pathlib import Path

import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.collections import LineCollection
from matplotlib.patches import FancyArrowPatch, Rectangle
from matplotlib.widgets import Button, CheckButtons, RadioButtons, Slider, TextBox
from serial.tools import list_ports

# ---------------------------------------------------------------------------
# Program mode
# ---------------------------------------------------------------------------
# "simulation"        : interactive visualizer with simulated scans
# "serial_diagnostic" : READ-ONLY inspection of what the ESP32 sends over USB
# "serial_capture"    : capture scans A, B, C by hand (READ-ONLY), align, save
# "session_viewer"    : open SESSION_FILE_TO_OPEN offline (no ESP32 needed)
# "alignment_benchmark": test automatic alignment on simulated rooms with
#                        known answers, then on saved sessions (read-only)
RUN_MODE = "serial_capture"
SERIAL_PORT = "COM5"
CANDIDATE_BAUD_RATES = (115200, 230400, 460800, 921600)
DIAGNOSTIC_SECONDS_PER_BAUD = 3.0

# Serial capture (university firmware ESP32_LD06_Test.ino, Teleplot output).
CAPTURE_BAUD_RATE = 115200
CAPTURE_TIMEOUT_S = 10.0       # per click; the firmware sends ~1 scan per second

# Session files live in the "scans" folder next to this program.
SCANS_DIR = Path(__file__).resolve().parent / "scans"
DEFAULT_SESSION_FILE = "classroom_session.json"
SESSION_FILE_TO_OPEN = DEFAULT_SESSION_FILE  # used by "session_viewer"

# ---------------------------------------------------------------------------
# LD06 protocol (manufacturer "LD06 Development manual" v1.0, section 3)
# ---------------------------------------------------------------------------
LD06_HEADER = 0x54            # first byte of every packet
LD06_VER_LEN = 0x2C           # second byte: 3 reserved bits + 5 bits = 12 points
LD06_POINTS_PER_PACKET = 12
# header(1) + ver_len(1) + speed(2) + start angle(2) + 12 x 3 bytes
# + end angle(2) + timestamp(2) + CRC(1) = 11 + 36 = 47 bytes
LD06_PACKET_SIZE = 11 + 3 * LD06_POINTS_PER_PACKET
LD06_CRC_POLYNOMIAL = 0x4D    # generates the manufacturer's CRC table

# Scan assembly from a stream of packets.
WRAP_DROP_DEG = 180.0          # angle drop larger than this = passed 360 -> 0
MIN_SCAN_COVERAGE_DEG = 340.0  # shorter revolutions (e.g. the first) are dropped

# ---------------------------------------------------------------------------
# Simulation only: the real LD06 never uses these values.
# ---------------------------------------------------------------------------
# Simulated rectangular room, in metres, in the GLOBAL map frame.
# A sensor at pose (0, 0, 0 deg) has its front along +X and its left along +Y.
# The origin is NOT at the centre of the room, so the distances differ in
# every direction (useful to check the angle convention).
ROOM_X_MIN = -1.0  # back wall
ROOM_X_MAX = 4.0   # front wall
ROOM_Y_MIN = -1.5  # right wall
ROOM_Y_MAX = 2.0   # left wall

ANGLE_STEP_DEG = 1.0      # angle between two simulated measurements
SIMULATED_INTENSITY = 200  # LD06 intensity given to every simulated hit


def distance_to_room_wall(sensor_x, sensor_y, theta_global_rad):
    """Return the distance (m) from the sensor to the first room wall.

    (sensor_x, sensor_y) is the sensor position in the global frame (m).
    theta_global_rad is the beam direction in radians, measured
    COUNTER-CLOCKWISE from the global +X axis.
    """
    dx = math.cos(theta_global_rad)  # X component of the unit beam direction
    dy = math.sin(theta_global_rad)  # Y component of the unit beam direction

    candidate_distances = []
    # Travelling t metres along the beam reaches the point
    # (sensor_x + t*dx, sensor_y + t*dy). Solve for t at each wall, e.g.
    # sensor_x + t*dx = ROOM_X_MAX  ->  t = (ROOM_X_MAX - sensor_x) / dx
    if dx > 0:
        candidate_distances.append((ROOM_X_MAX - sensor_x) / dx)
    elif dx < 0:
        candidate_distances.append((ROOM_X_MIN - sensor_x) / dx)
    if dy > 0:
        candidate_distances.append((ROOM_Y_MAX - sensor_y) / dy)
    elif dy < 0:
        candidate_distances.append((ROOM_Y_MIN - sensor_y) / dy)

    # The beam stops at the first wall it meets: the smallest distance.
    return min(candidate_distances)


def simulate_ld06_scan(pose=(0.0, 0.0, 0.0), angle_step_deg=ANGLE_STEP_DEG,
                       noise_mm=0.0, seed=0):
    """Simulate one 360 degree LD06 revolution inside the rectangular room.

    pose = (x_m, y_m, theta_deg): sensor position in the global frame and its
    heading, in degrees COUNTER-CLOCKWISE from the global +X axis.

    Returns three NumPy arrays of equal length, expressed in the SENSOR frame
    exactly as a real LD06 would report them:
        angles_deg   - LD06 angles, clockwise, in [0, 360)
        distances_mm - measured distances in millimetres (integers)
        intensities  - signal intensity (0-255)
    """
    sensor_x, sensor_y, sensor_theta_deg = pose
    if not (ROOM_X_MIN < sensor_x < ROOM_X_MAX
            and ROOM_Y_MIN < sensor_y < ROOM_Y_MAX):
        raise ValueError(f"Sensor position {pose[:2]} is outside the room")

    rng = np.random.default_rng(seed)  # seeded: same "noise" on every run
    angles_deg = np.arange(0.0, 360.0, angle_step_deg)
    distances_mm = np.zeros(len(angles_deg), dtype=int)
    intensities = np.full(len(angles_deg), SIMULATED_INTENSITY, dtype=int)

    for index, ld06_angle_deg in enumerate(angles_deg):
        # The LD06 angle grows clockwise; mathematics uses counter-clockwise.
        # Global beam direction = sensor heading + beam angle in sensor frame.
        theta_global_rad = math.radians(sensor_theta_deg - ld06_angle_deg)
        distance_m = distance_to_room_wall(sensor_x, sensor_y,
                                           theta_global_rad)
        noisy_distance_mm = distance_m * 1000.0 + rng.normal(0.0, noise_mm)
        distances_mm[index] = round(noisy_distance_mm)

    return angles_deg, distances_mm, intensities


def print_scan_summary(angles_deg, distances_mm, intensities):
    """Print the number of points and a few key measurements."""
    print(f"Simulated LD06 scan: {len(angles_deg)} measurements")
    print(f"Distance range: {distances_mm.min()} mm to {distances_mm.max()} mm")
    print()
    print(" LD06 angle (deg) | distance (mm) | intensity | direction")
    key_angles = {0.0: "front", 45.0: "front-right", 90.0: "right",
                  180.0: "back", 270.0: "left"}
    for angle, direction in key_angles.items():
        index = np.where(angles_deg == angle)[0][0]
        print(f" {angle:16.1f} | {distances_mm[index]:13d} |"
              f" {intensities[index]:9d} | {direction}")


# ---------------------------------------------------------------------------
# Simulation of any room made of wall segments (alignment tests/benchmark)
# ---------------------------------------------------------------------------
# The rectangular room above is perfectly symmetric: turned by 180 deg about
# its centre it looks exactly the same, so no algorithm can tell those two
# poses apart. Testing automatic alignment therefore also needs rooms with
# distinctive features (an L shape, desks, a pillar).
def polygon_walls(corners):
    """Return the wall segments of a closed polygon given by its corners."""
    return [(corners[i], corners[(i + 1) % len(corners)])
            for i in range(len(corners))]


def box_walls(x_min, y_min, x_max, y_max):
    """Return the 4 wall segments of an axis-aligned box (e.g. a desk)."""
    return polygon_walls([(x_min, y_min), (x_max, y_min), (x_max, y_max),
                          (x_min, y_max)])


# L-shaped classroom (8 m x 7 m minus a corner) with two desks and a pillar.
CLASSROOM_WALLS = (polygon_walls([(0, 0), (8, 0), (8, 5), (5, 5), (5, 7),
                                  (0, 7)])
                   + box_walls(1.5, 1.5, 2.7, 2.1)
                   + box_walls(4.0, 3.0, 4.6, 4.2)
                   + box_walls(6.5, 1.0, 6.8, 1.3))
SYMMETRIC_ROOM_WALLS = box_walls(0, 0, 6, 4)  # empty rectangle: ambiguous
OCTAGON_ROOM_WALLS = polygon_walls(
    [(3 * math.cos(math.radians(22.5 + 45 * k)),
      3 * math.sin(math.radians(22.5 + 45 * k))) for k in range(8)])


def simulate_scan_of_walls(walls, pose, angle_step_deg=0.8, noise_mm=0.0,
                           max_range_m=12.0, seed=0):
    """Simulate one LiDAR revolution in a world made of wall segments.

    walls - list of ((x1, y1), (x2, y2)) segments in the global frame (m)
    pose  - (x_m, y_m, theta_deg) of the sensor in the global frame
    Returns (x_local_m, y_local_m): the hit points in the SENSOR frame
    (+X front, +Y left), as teleplot_to_sensor_frame() gives real points.
    A ray that hits nothing within max_range_m gives no point (like LD06).

    Ray casting: a ray is origin + t*d (t >= 0), a wall is start + u*e
    (0 <= u <= 1). With the 2D cross product a x b = ax*by - ay*bx,
        t = ((start - origin) x e) / (d x e)
        u = ((start - origin) x d) / (d x e)
    and the measured distance is the smallest valid t over all walls.
    """
    rng = np.random.default_rng(seed)
    sensor_x, sensor_y, sensor_theta_deg = pose
    local_angles = np.radians(np.arange(-180.0, 180.0, angle_step_deg))
    global_angles = local_angles + math.radians(sensor_theta_deg)
    directions = np.column_stack([np.cos(global_angles),
                                  np.sin(global_angles)])       # (rays, 2)
    starts = np.array([wall[0] for wall in walls], dtype=float)
    edges = np.array([wall[1] for wall in walls], dtype=float) - starts
    offsets = starts - np.array([sensor_x, sensor_y])           # (walls, 2)

    def cross(a, b):
        return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]

    denominator = cross(directions[:, None, :], edges[None, :, :])
    with np.errstate(divide="ignore", invalid="ignore"):
        t = cross(offsets[None, :, :], edges[None, :, :]) / denominator
        u = cross(offsets[None, :, :], directions[:, None, :]) / denominator
    hits = (np.abs(denominator) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
    distances = np.where(hits, t, np.inf).min(axis=1)  # first wall per ray
    keep = distances <= max_range_m
    distances = distances[keep] + rng.normal(0.0, noise_mm / 1000.0,
                                             keep.sum())
    return (distances * np.cos(local_angles[keep]),
            distances * np.sin(local_angles[keep]))


def make_alignment_case(walls, reference_pose, source_pose, noise_mm=10.0,
                        outlier_fraction=0.0, source_walls=None, seed=1):
    """Simulate a reference scan and a scan to align, with a KNOWN answer.

    Returns (source_local, reference_scans): the source scan in its own
    sensor frame (what the robot measured, an (N, 2) array), and
    [(reference_local, reference_pose)], the reference scan at its true
    pose, as align_scan() expects it. The correct answer is source_pose.
    outlier_fraction adds random extra points within 2.5 m of the sensor
    (people, chair legs); source_walls lets the source see another room.
    """
    reference_local = np.column_stack(simulate_scan_of_walls(
        walls, reference_pose, noise_mm=noise_mm, seed=seed))
    source_local = np.column_stack(simulate_scan_of_walls(
        source_walls or walls, source_pose, noise_mm=noise_mm,
        seed=seed + 1))
    if outlier_fraction > 0:
        rng = np.random.default_rng(seed + 2)
        count = int(outlier_fraction * len(source_local))
        radius = rng.uniform(0.3, 2.5, count)
        angle = rng.uniform(-math.pi, math.pi, count)
        source_local = np.vstack([source_local, np.column_stack(
            [radius * np.cos(angle), radius * np.sin(angle)])])
    return source_local, [(reference_local, reference_pose)]


# ---------------------------------------------------------------------------
# LD06 raw packet decoding (offline; the bytes may later come from the ESP32)
# ---------------------------------------------------------------------------
def ld06_crc8(data):
    """Return the LD06 CRC-8 of a sequence of bytes (0-255).

    Bit-by-bit version of the manufacturer's table-based CalCRC8():
    polynomial 0x4D, initial value 0, most significant bit first.
    """
    crc = 0
    for byte in data:
        crc ^= byte                       # mix the next byte into the CRC
        for _ in range(8):                # then process its 8 bits
            if crc & 0x80:                # top bit set: shift and apply polynomial
                crc = ((crc << 1) ^ LD06_CRC_POLYNOMIAL) & 0xFF
            else:
                crc = (crc << 1) & 0xFF   # & 0xFF keeps only 8 bits
    return crc


def read_uint16_le(data, offset):
    """Read a 16-bit unsigned integer stored little-endian at data[offset]."""
    low_byte = data[offset]
    high_byte = data[offset + 1]
    return low_byte | (high_byte << 8)    # = low_byte + high_byte * 256


def is_valid_ld06_packet(packet):
    """Return True if `packet` has the right size, header and CRC."""
    return (len(packet) == LD06_PACKET_SIZE
            and packet[0] == LD06_HEADER
            and packet[1] == LD06_VER_LEN
            and ld06_crc8(packet[:-1]) == packet[-1])


def interpolate_packet_angles(start_angle_deg, end_angle_deg):
    """Return the 12 CLOCKWISE angles (deg, in [0, 360)) of one packet.

    Manufacturer rule: step = (end - start) / (12 - 1), angle_i = start + i*step,
    so the first point is exactly at start and the last exactly at end.
    """
    # "% 360" fixes packets that cross 0 deg, e.g. start 355, end 3 -> span 8
    span_deg = (end_angle_deg - start_angle_deg) % 360.0
    step_deg = span_deg / (LD06_POINTS_PER_PACKET - 1)
    indices = np.arange(LD06_POINTS_PER_PACKET)  # 0, 1, ..., 11
    return (start_angle_deg + indices * step_deg) % 360.0


def decode_ld06_packet(packet):
    """Decode one VALID 47-byte packet into a dictionary.

    Byte offsets:  0 header | 1 ver_len | 2-3 speed | 4-5 start angle |
    6-41 twelve (distance LSB, distance MSB, intensity) | 42-43 end angle |
    44-45 timestamp | 46 CRC.  All 16-bit values are little-endian.
    """
    distances_mm = np.zeros(LD06_POINTS_PER_PACKET, dtype=int)
    intensities = np.zeros(LD06_POINTS_PER_PACKET, dtype=int)
    for i in range(LD06_POINTS_PER_PACKET):
        offset = 6 + 3 * i                # each measurement takes 3 bytes
        distances_mm[i] = read_uint16_le(packet, offset)
        intensities[i] = packet[offset + 2]

    start_angle_deg = read_uint16_le(packet, 4) / 100.0  # unit is 0.01 deg
    end_angle_deg = read_uint16_le(packet, 42) / 100.0
    return {
        "speed_deg_per_s": read_uint16_le(packet, 2),
        "start_angle_deg": start_angle_deg,
        "end_angle_deg": end_angle_deg,
        "timestamp_ms": read_uint16_le(packet, 44),  # restarts after 30000
        "crc": packet[46],
        "angles_deg": interpolate_packet_angles(start_angle_deg, end_angle_deg),
        "distances_mm": distances_mm,
        "intensities": intensities,
    }


def extract_ld06_packets(buffer):
    """Remove every complete, valid packet from the front of `buffer`.

    buffer - a bytearray that the caller keeps and appends new serial bytes
             to; this function deletes the bytes it has used or rejected.

    Returns (packets, crc_failures): a list of valid 47-byte packets (bytes)
    and the number of candidate packets rejected by the CRC check.
    An incomplete packet stays in the buffer until more bytes arrive.
    """
    packets = []
    crc_failures = 0
    while True:
        start = buffer.find(bytes([LD06_HEADER, LD06_VER_LEN]))
        if start == -1:
            # No header pair. Keep a final 0x54: its 0x2C may still be coming.
            if buffer.endswith(bytes([LD06_HEADER])):
                del buffer[:-1]
            else:
                buffer.clear()
            break

        del buffer[:start]                     # discard noise before header
        if len(buffer) < LD06_PACKET_SIZE:
            break                              # partial packet: wait for more

        candidate = bytes(buffer[:LD06_PACKET_SIZE])
        if is_valid_ld06_packet(candidate):
            packets.append(candidate)
            del buffer[:LD06_PACKET_SIZE]
        else:
            # False or corrupted header: skip ONE byte only, so a real packet
            # starting inside the rejected bytes can still be found.
            crc_failures += 1
            del buffer[:1]
    return packets, crc_failures


def create_scan_builder():
    """Return an empty state used to assemble packets into 360 deg scans."""
    return {"angles_deg": [], "distances_mm": [], "intensities": [],
            "previous_angle_deg": None}


def finish_scan(builder):
    """Empty the builder; return its points as a scan, or None if incomplete.

    The returned scan has the same format as simulate_ld06_scan():
    (angles_deg, distances_mm, intensities) as NumPy arrays.
    """
    angles_deg = np.array(builder["angles_deg"])
    distances_mm = np.array(builder["distances_mm"], dtype=int)
    intensities = np.array(builder["intensities"], dtype=int)
    builder["angles_deg"].clear()
    builder["distances_mm"].clear()
    builder["intensities"].clear()

    if len(angles_deg) == 0:
        return None
    coverage_deg = angles_deg[-1] - angles_deg[0]
    if coverage_deg < MIN_SCAN_COVERAGE_DEG:
        return None  # partial revolution, e.g. the first one after start-up
    return angles_deg, distances_mm, intensities


def add_packet_to_scan_builder(builder, decoded_packet):
    """Add a decoded packet's 12 points; return the list of completed scans.

    A new revolution starts when the clockwise angle suddenly drops
    (e.g. 359.2 -> 0.4). This can happen in the middle of a packet.
    """
    completed_scans = []
    for angle, distance, intensity in zip(decoded_packet["angles_deg"],
                                          decoded_packet["distances_mm"],
                                          decoded_packet["intensities"]):
        previous = builder["previous_angle_deg"]
        if previous is not None and previous - angle > WRAP_DROP_DEG:
            scan = finish_scan(builder)
            if scan is not None:
                completed_scans.append(scan)
        builder["angles_deg"].append(float(angle))
        builder["distances_mm"].append(int(distance))
        builder["intensities"].append(int(intensity))
        builder["previous_angle_deg"] = float(angle)
    return completed_scans


# ---------------------------------------------------------------------------
# Serial diagnostic: READ-ONLY. Nothing is ever written to the ESP32.
# ---------------------------------------------------------------------------
# USB vendor:product IDs of common ESP32 USB chips, to help identify the port.
KNOWN_USB_CHIPS = {
    "10C4:EA60": "Silicon Labs CP210x USB-UART bridge (common on ESP32 boards)",
    "1A86:7523": "WCH CH340 USB-UART bridge (common on ESP32 boards)",
    "1A86:55D4": "WCH CH9102 USB-UART bridge (common on ESP32 boards)",
    "0403:6001": "FTDI USB-UART bridge",
    "303A:1001": "Espressif native USB (ESP32-S2/S3/C3)",
}


def list_serial_ports():
    """Print every serial port Windows knows and return their names."""
    ports = list_ports.comports()
    if not ports:
        print("No serial ports found. Is the ESP32 plugged in with a DATA "
              "USB cable, and is its USB driver installed?")
    for port in ports:
        usb_id = (f"{port.vid:04X}:{port.pid:04X}"
                  if port.vid is not None else "not USB")
        chip = KNOWN_USB_CHIPS.get(usb_id, "unknown chip")
        print(f"  {port.device}: {port.description} [{usb_id}] -> {chip}")
    return [port.device for port in ports]


def open_serial_readonly(port, baudrate):
    """Open `port` and return the connection. Nothing is ever written to it.

    The port is opened with DTR and RTS inactive. On most ESP32 boards these
    two lines drive the reset/boot circuit, so keeping them inactive avoids
    restarting the robot. Raises serial.SerialException if it cannot open.
    """
    connection = serial.serial_for_url(port, do_not_open=True)
    connection.baudrate = baudrate  # 8 data bits, no parity, 1 stop bit
    connection.timeout = 0.1        # read() waits at most 0.1 s
    connection.dtr = False          # set BEFORE open(): lines never asserted
    connection.rts = False
    connection.open()
    return connection


def read_serial_bytes(port, baudrate, duration_s):
    """Listen to `port` for `duration_s` seconds and return the bytes received.

    Read-only: see open_serial_readonly().
    """
    connection = open_serial_readonly(port, baudrate)
    received = bytearray()
    try:
        end_time = time.monotonic() + duration_s
        while time.monotonic() < end_time:
            received += connection.read(4096)  # whatever arrived, up to 4096
    finally:
        connection.close()                     # always release the port
    return bytes(received)


def count_text_formats(text):
    """Count lines that look like Teleplot, CSV or ESP32 boot messages."""
    csv_line = re.compile(r"^-?\d+(\.\d+)?(,-?\d+(\.\d+)?){3,5}$")
    counts = {"teleplot": 0, "csv": 0, "csv_header": 0, "boot": 0}
    for line in text.splitlines():
        line = line.strip()
        if line.startswith(">"):
            counts["teleplot"] += 1
        elif line.startswith("N,Angle"):
            counts["csv_header"] += 1
        elif csv_line.match(line):
            counts["csv"] += 1
        elif line.startswith(("rst:", "ets ", "boot:")):
            counts["boot"] += 1
    return counts


def describe_serial_data(data, duration_s):
    """Print a report about a sample of serial bytes; return a format guess."""
    print(f"  {len(data)} bytes in {duration_s:.1f} s "
          f"({len(data) / duration_s:.0f} bytes/s)")
    if not data:
        return "nothing received"
    print(f"  first bytes (hex): {data[:32].hex(' ')}")

    printable = sum(1 for byte in data if 32 <= byte <= 126 or byte in b"\r\n\t")
    printable_fraction = printable / len(data)
    packets, crc_failures = extract_ld06_packets(bytearray(data))
    # Non-ASCII bytes are shown as escape codes (e.g. "\xb0"), so the
    # preview never crashes and still shows exactly what was received.
    text = data.decode("ascii", errors="backslashreplace")
    counts = count_text_formats(text)
    print(f"  printable text: {printable_fraction:.0%} | valid LD06 packets: "
          f"{len(packets)} (CRC failures {crc_failures}) | lines: {counts}")
    for line in [line for line in text.splitlines() if line.strip()][:5]:
        print(f"    | {line[:100]}")  # preview, cut to 100 characters

    if len(packets) >= 2:
        return "raw LD06 binary packets"
    if printable_fraction < 0.9:
        return "binary or garbled (wrong baud rate?)"
    if counts["teleplot"]:
        return "Teleplot text"
    if counts["csv"] or counts["csv_header"]:
        return "CSV text"
    if counts["boot"]:
        return "ESP32 boot messages only"
    return "other text"


def run_serial_diagnostic(port, baud_rates, seconds_per_baud):
    """List the ports, then listen to `port` at each baud rate (read-only)."""
    print("=== Serial diagnostic (read-only: nothing is sent) ===")
    print("Available serial ports:")
    list_serial_ports()
    if port is None:
        print("\nSet SERIAL_PORT (e.g. 'COM5') at the top of the file, "
              "then run again.")
        return

    summary = {}
    for baudrate in baud_rates:
        print(f"\n--- {port} at {baudrate} baud ---")
        try:
            data = read_serial_bytes(port, baudrate, seconds_per_baud)
        except serial.SerialException as error:
            print(f"  could not open {port}: {error}")
            print("  (Is the port name right? Close the Arduino Serial "
                  "Monitor or any other program using it.)")
            return
        summary[baudrate] = describe_serial_data(data, seconds_per_baud)

    print("\n=== Summary ===")
    for baudrate, guess in summary.items():
        print(f"  {baudrate:7d} baud -> {guess}")


# ---------------------------------------------------------------------------
# Teleplot XY scans sent by the university firmware (ld06.printScanTeleplot)
# ---------------------------------------------------------------------------
# One text line per complete revolution; the points are ALREADY CARTESIAN,
# signed integers (int16) in millimetres, each followed by ";":
#     >lidar:x0:y0;x1:y1;...;xn:yn;|xy
# The library (ld06.cpp) first converts the clockwise LD06 angle into a
# counter-clockwise angle, then computes
#     x =  distance * cos(angle_ccw)
#     y = -distance * sin(angle_ccw)      <- note the minus sign
# so its +Y axis points to the RIGHT of the LiDAR. This program uses
# +Y = LEFT (see polar_to_cartesian), so Y must change sign. The real data
# confirms it: just after 0 deg, while the LD06 turns clockwise (towards its
# right), the library's y values grow POSITIVE (1320:16; 1336:33; ...).
TELEPLOT_SCAN_NAME = "lidar"
TELEPLOT_Y_POINTS_RIGHT = True  # False = keep the library's Y sign unchanged
TELEPLOT_MAX_LINE_BYTES = 32768  # 1200 points x "-12000:-12000;" fits easily
MIN_POINTS_PER_REAL_SCAN = 50    # a real 360 deg scan has about 450 points
MAX_MALFORMED_FRACTION = 0.05    # more bad entries = corrupted line, rejected


def split_complete_lines(buffer):
    """Remove every complete text line from the front of `buffer`.

    buffer - a bytearray that the caller keeps and appends serial bytes to.

    Returns the complete lines as strings, without line endings. The
    unfinished last line stays in the buffer until its "\\n" arrives, so a
    scan split across several serial reads is joined back together.
    """
    end = buffer.rfind(b"\n")
    if end == -1:
        if len(buffer) > TELEPLOT_MAX_LINE_BYTES:
            buffer.clear()  # far too long without a line ending: garbage
        return []
    complete = bytes(buffer[:end + 1])
    del buffer[:end + 1]
    # A corrupted byte becomes "�", so its entry is rejected later.
    text = complete.decode("ascii", errors="replace")
    return [line.strip() for line in text.splitlines()]  # strip() drops "\r"


def parse_finite_number(text):
    """Return `text` as a float ("-12", "+3", "4.5"), or None if invalid."""
    try:
        value = float(text)
    except ValueError:
        return None
    return value if math.isfinite(value) else None  # rejects "nan", "inf"


def parse_teleplot_xy_line(line, name=TELEPLOT_SCAN_NAME):
    """Parse one Teleplot XY line; return (x_mm, y_mm, malformed) or None.

    None means the line is not a COMPLETE XY line called `name`: another
    Teleplot variable, a boot message, or a line cut at its start (port
    opened mid-line) or at its end (missing "|xy").
    Otherwise x_mm and y_mm are NumPy arrays of signed millimetres, and
    `malformed` counts the skipped entries that were not exactly two
    finite numbers separated by ":".
    """
    prefix = f">{name}:"
    line = line.strip()
    if not line.startswith(prefix):
        return None
    body, separator, flags = line[len(prefix):].rpartition("|")
    if not separator or "xy" not in flags.split(","):
        return None

    x_values, y_values = [], []
    malformed = 0
    for entry in body.split(";"):
        entry = entry.strip()
        if not entry:
            continue  # the ";" after the last point leaves an empty entry
        fields = entry.split(":")
        if len(fields) == 2:
            x = parse_finite_number(fields[0])
            y = parse_finite_number(fields[1])
        else:
            x = y = None
        if x is None or y is None:
            malformed += 1
            continue
        x_values.append(x)
        y_values.append(y)
    return np.array(x_values), np.array(y_values), malformed


def teleplot_to_sensor_frame(x_mm, y_mm, y_points_right=TELEPLOT_Y_POINTS_RIGHT):
    """Convert library Teleplot points (mm) into this program's sensor frame.

    The points are already Cartesian, so there is NO polar-to-Cartesian
    conversion here: only
        - drop (0, 0) points: distance 0 means the LD06 got no echo,
        - millimetres -> metres,
        - change the sign of Y if the library's +Y points right.
    Returns (x_m, y_m): +X = front of the LiDAR, +Y = left of the LiDAR.
    """
    has_echo = (x_mm != 0) | (y_mm != 0)
    x_m = x_mm[has_echo] / 1000.0
    y_m = y_mm[has_echo] / 1000.0
    if y_points_right:
        y_m = -y_m
    return x_m, y_m


# ---------------------------------------------------------------------------
# CSV scans sent by the same library (ld06.printScanCSV)
# ---------------------------------------------------------------------------
# A header line, printed ONCE after the ESP32 boots:
#     N,Angle(°),Distance(mm),Intensity,x(mm),y(mm)
# then one line PER POINT, and an EMPTY line after each complete scan:
#     0,0.38,1320,229,1319,-8
#     1,1.10,1336,231,1335,-25
#     ...
#     <empty line>
# Columns: index, angle (degrees, ALREADY counter-clockwise), distance (mm),
# intensity, then x and y (mm) only when ld06.h defines LD06_COMPUTE_XY.
# x and y are the SAME library values as in the Teleplot line (+Y = RIGHT),
# so both formats go through teleplot_to_sensor_frame(). Without them
# (4 columns) they are computed with the ld06.cpp formulas:
#     x =  distance * cos(angle)        y = -distance * sin(angle)
FORMAT_TELEPLOT = "teleplot_xy"
FORMAT_CSV_XY = "csv_6"
FORMAT_CSV_POLAR = "csv_4"
FORMAT_LABELS = {
    None: "no LiDAR format detected yet",
    FORMAT_TELEPLOT: "Teleplot XY (printScanTeleplot)",
    FORMAT_CSV_XY: "LD06 CSV, 6 columns with x/y (printScanCSV)",
    FORMAT_CSV_POLAR: "LD06 CSV, 4 columns without x/y (printScanCSV)",
}
CSV_HEADER_PREFIX = "N,"
LD06_MAX_DISTANCE_MM = 65535  # uint16_t distance in ld06.h


def parse_ld06_csv_row(line):
    """Parse one printScanCSV point line; return (fmt, index, x_mm, y_mm).

    fmt is FORMAT_CSV_XY or FORMAT_CSV_POLAR; x_mm, y_mm are in the LIBRARY
    frame (+Y = right), like a Teleplot point. Returns None if the line is
    not a valid 4- or 6-column row: wrong column count, non-numbers,
    a negative or fractional index, an angle outside 0..360 degrees or a
    distance outside 0..65535 mm.
    """
    fields = line.strip().split(",")
    if len(fields) not in (4, 6):
        return None
    values = [parse_finite_number(field) for field in fields]
    if any(value is None for value in values):
        return None
    index, angle_deg, distance_mm = values[0], values[1], values[2]
    if (index < 0 or index != int(index) or not 0.0 <= angle_deg <= 360.0
            or not 0.0 <= distance_mm <= LD06_MAX_DISTANCE_MM):
        return None
    if len(fields) == 6:
        return FORMAT_CSV_XY, int(index), values[4], values[5]
    angle_rad = math.radians(angle_deg)
    return (FORMAT_CSV_POLAR, int(index), distance_mm * math.cos(angle_rad),
            -distance_mm * math.sin(angle_rad))


def is_csv_candidate(line):
    """True if `line` looks like a printScanCSV point row (maybe corrupted).

    Teleplot lines (">...") and text without commas (e.g. "New scan
    completed.") are other output, not CSV rows, and are ignored.
    """
    return "," in line and not line.startswith(">")


class CsvScanAssembler:
    """Join printScanCSV point rows into scans; an empty line ends a scan.

    The capture may start in the MIDDLE of a scan, so the rows before the
    first empty line (or header) are an INCOMPLETE scan and are discarded:
    the first scan returned started after the capture began. A scan is also
    discarded when a row is missing (row index != its position: a line was
    lost) or when its rows change column count. A scan still unfinished
    when the capture stops is never returned.
    """

    def __init__(self):
        self.synchronised = False  # True once a scan boundary was seen
        self.start_scan()

    def start_scan(self):
        self.x_mm, self.y_mm = [], []
        self.rows = 0         # all rows of this scan, valid or not
        self.malformed = 0
        self.format = None    # column count of the scan's first valid row
        self.complete = True  # becomes False if a row is missing

    def add_line(self, line):
        """Feed one line; return a finished scan dictionary or None.

        The dictionary has "x_mm", "y_mm" (NumPy arrays, library frame),
        "malformed", "format" and "complete" (False = rows were missing).
        """
        line = line.strip()
        if line.startswith(CSV_HEADER_PREFIX):  # ESP32 (re)booted
            self.synchronised = True
            self.start_scan()
            return None
        if not line:
            finished = self.finish_scan()
            self.synchronised = True
            self.start_scan()
            return finished
        if not is_csv_candidate(line):
            return None
        position = self.rows
        self.rows += 1
        parsed = parse_ld06_csv_row(line)
        if parsed is None or parsed[0] != (self.format or parsed[0]):
            self.malformed += 1  # bad row, or 4 and 6 columns mixed
            return None
        self.format, index, x_mm, y_mm = parsed
        if index != position:
            self.complete = False
        self.x_mm.append(x_mm)
        self.y_mm.append(y_mm)
        return None

    def finish_scan(self):
        if not self.synchronised or self.rows == 0:
            return None  # partial scan from before the capture, or no rows
        return {"x_mm": np.array(self.x_mm, dtype=float),
                "y_mm": np.array(self.y_mm, dtype=float),
                "malformed": self.malformed, "complete": self.complete,
                "format": self.format}


def new_capture_report(stop_reason="timeout"):
    """Return the counters filled in by capture_teleplot_scans()."""
    return {"bytes": 0, "lines": 0, "rejected_scans": 0,
            "malformed_entries": 0, "stop_reason": stop_reason,
            "format": None, "incomplete_scans": 0}


def capture_teleplot_scans(connection, scan_count, timeout_s,
                           name=TELEPLOT_SCAN_NAME, clock=time.monotonic,
                           stop_event=None, on_format=None):
    """Read serial text until `scan_count` complete scans have arrived.

    Both library output formats are accepted, detected line by line:
    Teleplot XY (one ">lidar:...|xy" line per scan) and CSV (one row per
    point, an empty line after each scan; 6 or 4 columns).

    connection - an OPEN serial port, or any object with read(size) -> bytes
                 (the tests use a fake one)
    timeout_s  - give up after this many seconds in total
    clock      - time source in seconds (the tests use a fake clock)
    stop_event - optional threading.Event: when set, stop early ("cancelled")
    on_format  - optional on_format(fmt), called (on THIS thread) whenever
                 the detected format changes; fmt is a FORMAT_LABELS key

    Returns (scans, report). `scans` is a list of (x_mm, y_mm) arrays in the
    library frame (whatever the format) and may be SHORTER than scan_count.
    `report` counts what was received, gives the last detected "format"
    and the "stop_reason": "complete", "timeout", "cancelled" or
    "disconnected: ...".
    """
    buffer = bytearray()  # a NEW buffer: bytes from earlier reads are ignored
    csv_scans = CsvScanAssembler()
    scans = []
    report = new_capture_report()
    deadline = clock() + timeout_s

    def detected(fmt):
        if fmt is not None and fmt != report["format"]:
            report["format"] = fmt
            if on_format is not None:
                on_format(fmt)

    def accept(x_mm, y_mm, malformed):
        """Apply the same quality rules to a scan of either format."""
        report["malformed_entries"] += malformed
        entries = len(x_mm) + malformed
        points_with_echo = np.count_nonzero((x_mm != 0) | (y_mm != 0))
        if (points_with_echo < MIN_POINTS_PER_REAL_SCAN
                or malformed > MAX_MALFORMED_FRACTION * entries):
            report["rejected_scans"] += 1
            return
        scans.append((x_mm, y_mm))

    while len(scans) < scan_count and clock() < deadline:
        if stop_event is not None and stop_event.is_set():
            report["stop_reason"] = "cancelled"
            break
        try:
            chunk = connection.read(4096)  # waits at most the port timeout
        except serial.SerialException as error:  # e.g. USB cable unplugged
            report["stop_reason"] = f"disconnected: {error}"
            break
        report["bytes"] += len(chunk)
        buffer.extend(chunk)

        for line in split_complete_lines(buffer):
            report["lines"] += 1
            parsed = parse_teleplot_xy_line(line, name)
            if parsed is not None:
                detected(FORMAT_TELEPLOT)
                accept(*parsed)
            else:
                # Not a complete Teleplot line: maybe a CSV row or boundary.
                if is_csv_candidate(line):
                    row = parse_ld06_csv_row(line)
                    detected(row[0] if row is not None else None)
                finished = csv_scans.add_line(line)
                if finished is not None:
                    if not finished["complete"]:
                        report["incomplete_scans"] += 1
                        report["rejected_scans"] += 1
                    else:
                        accept(finished["x_mm"], finished["y_mm"],
                               finished["malformed"])
            if len(scans) == scan_count:
                break

    if len(scans) == scan_count:
        report["stop_reason"] = "complete"
    return scans, report


def print_capture_report(scans, report, scan_count):
    """Print what the capture received and, if scans are missing, why."""
    print(f"Received {report['bytes']} bytes, {report['lines']} complete "
          f"lines, {len(scans)}/{scan_count} valid scans "
          f"({report['rejected_scans']} rejected, "
          f"{report['malformed_entries']} malformed entries skipped).")
    print(f"Serial format: {FORMAT_LABELS[report['format']]}")
    print(f"Stop reason: {report['stop_reason']}")
    if len(scans) == scan_count:
        return
    if report["stop_reason"].startswith("disconnected"):
        print("  The port stopped responding: check the USB cable.")
    elif report["bytes"] == 0:
        print("  Nothing received: is the ESP32 powered and running the "
              "firmware? Is another program (Arduino Serial Monitor, "
              "Teleplot) using the port?")
    elif not scans and report["format"] is None:
        print(f"  Text received but no '>{TELEPLOT_SCAN_NAME}:...|xy' line "
              f"and no LD06 CSV row: run diagnose_lidar_serial.py to see "
              f"the format.")
    elif report["incomplete_scans"]:
        print(f"  {report['incomplete_scans']} CSV scan(s) had missing rows "
              f"(serial data lost): check the USB cable, or increase "
              f"CAPTURE_TIMEOUT_S.")
    else:
        print("  Too few scans before the timeout: increase "
              "CAPTURE_TIMEOUT_S, or check that the LiDAR is spinning.")


# ---------------------------------------------------------------------------
# Background serial worker: the GUI never waits for the serial port
# ---------------------------------------------------------------------------
# A Matplotlib window freezes if its own thread blocks, and waiting for a
# scan takes about one second. So a separate THREAD owns the serial port:
#     GUI  --(request, e.g. ("add", None))-->  requests queue  -->  worker
#     GUI  <--(request, scans, report)-------  results queue   <--  worker
# The GUI only puts and gets queue items (never blocking) and checks the
# results queue with a 100 ms timer. Matplotlib objects are touched ONLY by
# the GUI thread; the serial port is touched ONLY by the worker thread.
def start_serial_worker(connection, timeout_s=CAPTURE_TIMEOUT_S):
    """Start the thread that captures one scan per request; return its state.

    connection - an OPEN serial port (or a fake one in the tests)
    Returns a dictionary: "requests" and "results" queues, the "stop" event,
    the "thread" and the "connection".
    """
    worker = {"connection": connection, "requests": queue.Queue(),
              "results": queue.Queue(), "stop": threading.Event()}
    # daemon=True: the thread can never keep Python running after the GUI.
    worker["thread"] = threading.Thread(target=serial_worker_loop,
                                        args=(worker, timeout_s), daemon=True)
    worker["thread"].start()
    return worker


def serial_worker_loop(worker, timeout_s):
    """Body of the worker thread: wait for a request, capture ONE scan.

    The request is not interpreted here: it is sent back unchanged with the
    result, so the GUI knows what the scan was for (add or replace).

    reset_input_buffer() throws away every byte that arrived BEFORE the
    click, and capture_teleplot_scans() starts with an empty buffer. The
    first bytes after that are the end of a line already in progress: they
    do not start with ">lidar:" so they are rejected. The first ACCEPTED
    line therefore started after the click, while the robot was still.
    (CSV: the rows before the first empty line are discarded the same way.)
    """
    connection = worker["connection"]
    while not worker["stop"].is_set():
        try:
            request = worker["requests"].get(timeout=0.1)  # wait for a click
        except queue.Empty:
            continue
        try:
            connection.reset_input_buffer()
            scans, report = capture_teleplot_scans(
                connection, 1, timeout_s, stop_event=worker["stop"])
        except serial.SerialException as error:
            scans, report = [], new_capture_report(f"disconnected: {error}")
        worker["results"].put((request, scans, report))
        if report["stop_reason"].startswith("disconnected"):
            break  # the port is gone: the thread ends, the GUI stays open


def stop_serial_worker(worker):
    """Ask the worker thread to stop, wait for it, then close the port."""
    worker["stop"].set()
    worker["thread"].join(timeout=2.0)  # a capture checks "stop" every 0.1 s
    try:
        worker["connection"].close()
    except serial.SerialException:
        pass  # already unplugged: nothing left to release


# ---------------------------------------------------------------------------
# Coordinates and visualization (shared by simulated and real scans)
# ---------------------------------------------------------------------------
def polar_to_cartesian(angles_deg, distances_mm):
    """Convert LD06 polar measurements into Cartesian points in the sensor frame.

    angles_deg   - LD06 angles in degrees, CLOCKWISE (NumPy array)
    distances_mm - distances in millimetres (NumPy array)

    Returns two NumPy arrays (x_m, y_m) in metres:
    +X = front of the LiDAR, +Y = left of the LiDAR.
    """
    theta_math_rad = np.radians(-angles_deg)  # clockwise -> counter-clockwise
    distances_m = distances_mm / 1000.0       # millimetres -> metres
    x_m = distances_m * np.cos(theta_math_rad)
    y_m = distances_m * np.sin(theta_math_rad)
    return x_m, y_m


def local_to_global(x_local, y_local, pose):
    """Transform points from the sensor frame into the global map frame.

    x_local, y_local - NumPy arrays of points in the sensor frame (m)
    pose             - (x_m, y_m, theta_deg) of the sensor in the global frame

    Returns NEW arrays (x_global, y_global); the inputs are not modified.
    """
    pose_x, pose_y, pose_theta_deg = pose
    cos_theta = math.cos(math.radians(pose_theta_deg))
    sin_theta = math.sin(math.radians(pose_theta_deg))
    # Rotate by the sensor heading, then translate by the sensor position.
    x_global = pose_x + x_local * cos_theta - y_local * sin_theta
    y_global = pose_y + x_local * sin_theta + y_local * cos_theta
    return x_global, y_global


def create_scan(label, color, pose, angles_deg, distances_mm, intensities):
    """Store one scan (simulated or decoded) in a dictionary.

    The dictionary keeps the ORIGINAL measurements (sensor frame, clockwise
    LD06 angles) next to the pose. Global points are always recomputed from
    them, never stored.
    """
    return {
        "label": label,
        "color": color,
        "pose": pose,
        "angles_deg": angles_deg,
        "distances_mm": distances_mm,
        "intensities": intensities,
    }


def create_simulated_scan(label, color, pose):
    """Simulate a scan from the given pose and store it with create_scan()."""
    angles_deg, distances_mm, intensities = simulate_ld06_scan(pose)
    return create_scan(label, color, pose, angles_deg, distances_mm,
                       intensities)


def create_cartesian_scan(label, color, pose, x_local_m, y_local_m):
    """Store one scan whose points are ALREADY Cartesian (e.g. Teleplot).

    x_local_m, y_local_m - points in the sensor frame, in metres
    (+X = front, +Y = left). They are kept unchanged; global points are
    always recomputed from them and the current pose.
    """
    return {
        "label": label,
        "color": color,
        "pose": pose,
        "x_local_m": x_local_m,
        "y_local_m": y_local_m,
    }


def scan_to_local(scan):
    """Return the sensor-frame (x, y) points of a scan, in metres.

    Polar scans (simulated or decoded packets) are converted once with
    polar_to_cartesian(); Cartesian scans are returned as they are, so
    their points are never converted from polar a second time.
    """
    if "x_local_m" in scan:
        return scan["x_local_m"], scan["y_local_m"]
    return polar_to_cartesian(scan["angles_deg"], scan["distances_mm"])


def scan_to_global(scan):
    """Return the global (x, y) points of a scan, using its current pose."""
    x_local, y_local = scan_to_local(scan)
    return local_to_global(x_local, y_local, scan["pose"])


def draw_scans(axes, scans, title):
    """Draw several scans in the global frame on existing axes.

    One colour per scan. Each sensor is drawn as a dot at its position with
    an arrow along its front (the sensor's +X axis).
    """
    arrow_length_m = 0.5

    for scan in scans:
        x_global, y_global = scan_to_global(scan)
        pose_x, pose_y, pose_theta_deg = scan["pose"]
        axes.scatter(x_global, y_global, s=8, color=scan["color"],
                     label=f"{scan['label']}: pose ({pose_x:.1f} m, "
                           f"{pose_y:.1f} m, {pose_theta_deg:.0f} deg)")

        # Sensor position (dot) and forward direction (arrow).
        heading_rad = math.radians(pose_theta_deg)
        arrow_end_x = pose_x + arrow_length_m * math.cos(heading_rad)
        arrow_end_y = pose_y + arrow_length_m * math.sin(heading_rad)
        axes.plot(pose_x, pose_y, marker="o", markersize=9,
                  color=scan["color"], markeredgecolor="black",
                  linestyle="none")
        axes.annotate("", xy=(arrow_end_x, arrow_end_y),
                      xytext=(pose_x, pose_y),
                      arrowprops={"arrowstyle": "->", "color": scan["color"],
                                  "linewidth": 2})

    axes.set_xlabel("X (m)")
    axes.set_ylabel("Y (m)")
    axes.set_title(title)
    axes.set_aspect("equal")  # 1 m on X has the same length as 1 m on Y
    # Empty border around the data; the larger top border holds the legend
    # so it never hides any points.
    axes.margins(x=0.1, y=0.35)
    axes.grid(True)
    axes.legend(loc="upper right", fontsize="small")


def create_pose_editor(scans, title, x_range, y_range):
    """Open a figure where the pose of each scan can be changed with sliders.

    scans   - list of scan dictionaries (their "pose" entries are modified)
    title   - plot title
    x_range - (min, max) of the x slider, in metres
    y_range - (min, max) of the y slider, in metres

    Returns (figure, widgets). The caller must keep `widgets` in a variable:
    Matplotlib widgets stop responding if Python deletes them.
    """
    figure = plt.figure(figsize=(10, 8))
    # Each area is [left, bottom, width, height] as fractions of the window.
    plot_axes = figure.add_axes([0.25, 0.25, 0.72, 0.70])
    selector_axes = figure.add_axes([0.03, 0.60, 0.15, 0.20])
    x_slider_axes = figure.add_axes([0.30, 0.13, 0.55, 0.03])
    y_slider_axes = figure.add_axes([0.30, 0.08, 0.55, 0.03])
    theta_slider_axes = figure.add_axes([0.30, 0.03, 0.55, 0.03])

    labels = [scan["label"] for scan in scans]
    selector = RadioButtons(selector_axes, labels)
    first_x, first_y, first_theta = scans[0]["pose"]
    x_slider = Slider(x_slider_axes, "x (m)", x_range[0], x_range[1],
                      valinit=first_x, valstep=0.01)
    y_slider = Slider(y_slider_axes, "y (m)", y_range[0], y_range[1],
                      valinit=first_y, valstep=0.01)
    theta_slider = Slider(theta_slider_axes, "theta (deg)", -180.0, 180.0,
                          valinit=first_theta, valstep=1.0)

    selected_index = 0  # index in `scans` of the scan being edited

    def redraw():
        """Recompute every scan from its original data and current pose."""
        plot_axes.clear()
        draw_scans(plot_axes, scans, title)
        figure.canvas.draw_idle()  # ask Matplotlib to refresh the window

    def set_pose_component(component_index, value):
        """Change x (0), y (1) or theta (2) of the selected scan's pose."""
        scan = scans[selected_index]
        pose = list(scan["pose"])  # tuples cannot be changed: copy to a list
        pose[component_index] = float(value)
        scan["pose"] = tuple(pose)
        redraw()

    def on_scan_selected(label):
        """Select another scan and move the sliders to its current pose."""
        nonlocal selected_index
        selected_index = labels.index(label)
        x, y, theta = scans[selected_index]["pose"]
        x_slider.set_val(x)
        y_slider.set_val(y)
        theta_slider.set_val(theta)

    selector.on_clicked(on_scan_selected)
    x_slider.on_changed(lambda value: set_pose_component(0, value))
    y_slider.on_changed(lambda value: set_pose_component(1, value))
    theta_slider.on_changed(lambda value: set_pose_component(2, value))

    redraw()
    return figure, (selector, x_slider, y_slider, theta_slider)


# ---------------------------------------------------------------------------
# Automatic scan alignment: correlative search (coarse -> fine) + ICP
# ---------------------------------------------------------------------------
# Goal: find the pose (x, y, theta) that puts a scan's LOCAL points on top
# of the reference map (global points of already aligned scans):
#     p_global = R(theta) p_local + [x, y]            (local_to_global)
#
# Stage 1 - correlative matching ("does the scan fit the map here?")
#     The reference points are drawn into an image whose pixels are 1 on a
#     wall and fade smoothly to 0 away from it (a "likelihood field"). A
#     candidate pose is scored by placing the scan with it and averaging the
#     pixel values under its points: 1 = every point lies on a wall.
#     EVERY heading 0-360 deg (every 2 deg) and every robot position inside
#     the map (every 20 cm) is tried, so the robot may have turned by any
#     angle. The best few clearly different poses are then searched again
#     around themselves on a 5 cm / 0.5 deg grid: "multiresolution".
# Stage 2 - ICP (Iterative Closest Point), from each of those poses:
#     repeat: pair every scan point with its nearest reference point,
#     ignore pairs that are too far apart (the scan sees things the
#     reference does not), compute the rotation + translation that best
#     overlays the pairs (closed form, see best_rigid_transform), apply it.
# Stage 3 - an honest verdict. ICP ALWAYS converges somewhere, so its
#     convergence proves nothing. Each refined pose is checked for
#     FREE-SPACE CONFLICTS: placed correctly, the scan never "sees through"
#     a wall of the reference (if the map has a wall 2 m away in some
#     direction, the scan cannot have measured 5 m in that direction).
#     Candidate poses are ranked by
#         score = matched fraction - CONFLICT_PENALTY x conflict fraction
#     (one contradiction outweighs three agreements). This is what rejects
#     a room wrongly turned by 180 deg: its long walls may overlap even
#     better than the true pose's, but its desks land in open space. Real
#     scans show some conflicts even when correct (chair legs seen once,
#     not twice), so a few are tolerated. The best pose is then judged on
#     its matched fraction, RMS distance, conflicts, and whether a clearly
#     DIFFERENT pose scores almost as well (ambiguity).
ALIGN_VOXEL_M = 0.05           # point spacing for the fine search and checks
ALIGN_ICP_VOXEL_M = 0.10       # coarser source points make ICP faster
ALIGN_COARSE_CELL_M = 0.10     # pixel size of the coarse likelihood image
ALIGN_COARSE_SIGMA_M = 0.15    # how far the coarse image "glows" around walls
ALIGN_COARSE_STEP_M = 0.20     # robot positions tried by the coarse search
ALIGN_COARSE_STEP_DEG = 2.0    # headings tried by the coarse search (0-360)
ALIGN_FINE_CELL_M = 0.05
ALIGN_FINE_SIGMA_M = 0.05
ALIGN_FINE_STEP_M = 0.05
ALIGN_FINE_STEP_DEG = 0.5
ALIGN_CANDIDATES = 10          # best distinct coarse poses refined by ICP
ALIGN_DISTINCT_DEG = 10.0      # two poses are "clearly different" if their
ALIGN_DISTINCT_M = 0.5         # headings OR positions differ more than this
ICP_ITERATIONS = 40
ICP_START_DISTANCE_M = 0.30    # pairs farther apart are ignored (shrinks)
MATCH_DISTANCE_M = 0.08        # a point "matches" within this distance
GOOD_MATCH_FRACTION = 0.50     # verdict thresholds (see judge_alignment)
MIN_MATCH_FRACTION = 0.30
GOOD_RMS_M = 0.030
MAX_RMS_M = 0.045
AMBIGUITY_RATIO = 0.90         # runner-up this close to the best: ambiguous
MIN_ALIGN_POINTS = 30
FREE_SPACE_BIN_DEG = 1.0       # direction bins of the free-space check
FREE_SPACE_MARGIN_M = 0.15     # nearer than the scan by more = conflict
MAX_CONFLICT_FRACTION = 0.20   # more conflicts = impossible pose
GOOD_CONFLICT_FRACTION = 0.12  # "good" needs at most this (real clutter)
CONFLICT_PENALTY = 3.0         # score = matched - 3 x conflicting fraction


def wrap_angle_deg(angle_deg):
    """Return the same direction as an angle in [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def angle_difference_deg(a_deg, b_deg):
    """Smallest difference between two headings, from 0 to 180 deg."""
    return abs(wrap_angle_deg(a_deg - b_deg))


def poses_are_distinct(pose_a, pose_b):
    """True if two poses differ clearly in heading OR in position."""
    return (angle_difference_deg(pose_a[2], pose_b[2]) > ALIGN_DISTINCT_DEG
            or math.hypot(pose_a[0] - pose_b[0], pose_a[1] - pose_b[1])
            > ALIGN_DISTINCT_M)


def transform_points(points, pose):
    """Apply a pose to an (N, 2) array of local points (local_to_global)."""
    x_global, y_global = local_to_global(points[:, 0], points[:, 1], pose)
    return np.column_stack([x_global, y_global])


def voxel_downsample(points, cell_m):
    """Keep one point (the average) per square cell of size cell_m.

    Makes the point density even (a LiDAR measures many more points on
    near walls than on far ones) and makes every later step faster.
    """
    if len(points) == 0:
        return points.reshape(0, 2)
    cells = np.floor(points / cell_m).astype(np.int64)
    _, cell_index, counts = np.unique(cells, axis=0, return_inverse=True,
                                      return_counts=True)
    cell_index = cell_index.ravel()
    return np.column_stack([
        np.bincount(cell_index, weights=points[:, 0]) / counts,
        np.bincount(cell_index, weights=points[:, 1]) / counts])


def build_likelihood_field(points, cell_m, sigma_m, margin_m=1.0):
    """Draw the reference points as an image that "glows" around walls.

    Pixel (row j, column i) covers x0 + i*cell_m and y0 + j*cell_m. Each
    point lights its pixel with 1; a Gaussian blur (done row by row, then
    column by column) makes the value fade as exp(-d^2 / (2 sigma^2)) with
    the distance d to the wall; values are capped at 1.
    """
    x0, y0 = points.min(axis=0) - margin_m
    width, height = (np.ceil((points.max(axis=0) + margin_m
                              - [x0, y0]) / cell_m).astype(int) + 1)
    image = np.zeros((height, width))
    columns = ((points[:, 0] - x0) / cell_m).astype(int)
    rows = ((points[:, 1] - y0) / cell_m).astype(int)
    image[rows, columns] = 1.0
    radius = int(math.ceil(3 * sigma_m / cell_m))
    offsets_m = np.arange(-radius, radius + 1) * cell_m
    kernel = np.exp(-0.5 * (offsets_m / sigma_m) ** 2)  # 1 at the centre
    image = np.apply_along_axis(np.convolve, 1, image, kernel, mode="same")
    image = np.apply_along_axis(np.convolve, 0, image, kernel, mode="same")
    return {"image": np.minimum(image, 1.0), "x0": x0, "y0": y0,
            "cell_m": cell_m}


def field_score(field, points):
    """Average image value under the points (last axis = x, y); 0 to 1."""
    image = field["image"]
    height, width = image.shape
    columns = np.floor((points[..., 0] - field["x0"])
                       / field["cell_m"]).astype(np.int64)
    rows = np.floor((points[..., 1] - field["y0"])
                    / field["cell_m"]).astype(np.int64)
    inside = (columns >= 0) & (columns < width) & (rows >= 0) & (rows < height)
    values = image[rows.clip(0, height - 1), columns.clip(0, width - 1)]
    return np.where(inside, values, 0.0).mean(axis=-1)


def position_grid(x_range, y_range, step_m):
    """Return an (N, 2) array of positions on a regular grid."""
    xs = np.arange(x_range[0], x_range[1] + step_m / 2, step_m)
    ys = np.arange(y_range[0], y_range[1] + step_m / 2, step_m)
    grid_x, grid_y = np.meshgrid(xs, ys)
    return np.column_stack([grid_x.ravel(), grid_y.ravel()])


def correlative_search(source, field, headings_deg, positions):
    """Score every combination of heading and robot position.

    source       - (N, 2) local points of the scan
    headings_deg - 1D array of headings to try
    positions    - (P, 2) robot positions to try
    Returns an array scores[heading index, position index] from 0 to 1.
    The scan is rotated once per heading; all positions are then tried at
    once by NumPy broadcasting: (P, 1, 2) + (1, N, 2) -> (P, N, 2).
    """
    scores = np.empty((len(headings_deg), len(positions)))
    for index, heading_deg in enumerate(headings_deg):
        rotated = transform_points(source, (0.0, 0.0, heading_deg))
        placed = positions[:, None, :] + rotated[None, :, :]
        scores[index] = field_score(field, placed)
    return scores


def pick_distinct_poses(scores, headings_deg, positions, count):
    """Return up to `count` best (pose, score) that are clearly different.

    Neighbours of a good pose score almost as well as it does; skipping
    poses close to an already chosen one ("non-maximum suppression")
    keeps genuinely different hypotheses, e.g. a room turned by 180 deg.
    """
    chosen = []
    for flat_index in np.argsort(scores, axis=None)[::-1][:50000]:
        heading_index, position_index = np.unravel_index(flat_index,
                                                         scores.shape)
        pose = (float(positions[position_index, 0]),
                float(positions[position_index, 1]),
                float(headings_deg[heading_index]))
        if all(poses_are_distinct(pose, other) for other, _ in chosen):
            chosen.append((pose, float(scores[heading_index,
                                              position_index])))
            if len(chosen) == count:
                break
    return chosen


def refine_by_search(source, field, pose):
    """Search finely (5 cm, 0.5 deg) around a coarse pose; return the best."""
    x, y, heading_deg = pose
    headings = heading_deg + np.arange(-ALIGN_COARSE_STEP_DEG,
                                       ALIGN_COARSE_STEP_DEG + 1e-9,
                                       ALIGN_FINE_STEP_DEG)
    positions = position_grid((x - ALIGN_COARSE_STEP_M,
                               x + ALIGN_COARSE_STEP_M),
                              (y - ALIGN_COARSE_STEP_M,
                               y + ALIGN_COARSE_STEP_M), ALIGN_FINE_STEP_M)
    scores = correlative_search(source, field, headings, positions)
    heading_index, position_index = np.unravel_index(np.argmax(scores),
                                                     scores.shape)
    return (float(positions[position_index, 0]),
            float(positions[position_index, 1]),
            float(headings[heading_index]))


def nearest_neighbours(points, reference):
    """For each point: index of, and distance to, the closest reference point.

    Brute force (every pair), in chunks of 256 points to limit memory.
    """
    indices = np.empty(len(points), dtype=np.int64)
    distances = np.empty(len(points))
    for start in range(0, len(points), 256):
        chunk = points[start:start + 256]
        squared = ((chunk[:, None, :] - reference[None, :, :]) ** 2).sum(-1)
        closest = squared.argmin(axis=1)
        indices[start:start + 256] = closest
        distances[start:start + 256] = np.sqrt(
            squared[np.arange(len(chunk)), closest])
    return indices, distances


def best_rigid_transform(points, targets):
    """Rotation + translation that best moves `points` onto `targets`.

    Least squares for paired 2D points (closed form):
    1. remove the averages: a_i = p_i - mean(p), b_i = q_i - mean(q)
    2. the best angle maximises sum(b_i . R a_i), which gives
           angle = atan2( sum(a_x b_y - a_y b_x), sum(a_x b_x + a_y b_y) )
    3. translation = mean(q) - R mean(p)
    Returns (angle_rad, translation) with translation = array([tx, ty]).
    """
    points_mean = points.mean(axis=0)
    targets_mean = targets.mean(axis=0)
    a = points - points_mean
    b = targets - targets_mean
    angle_rad = math.atan2(np.sum(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]),
                           np.sum(a[:, 0] * b[:, 0] + a[:, 1] * b[:, 1]))
    rotation = np.array([[math.cos(angle_rad), -math.sin(angle_rad)],
                         [math.sin(angle_rad), math.cos(angle_rad)]])
    return angle_rad, targets_mean - rotation @ points_mean


def icp(source, reference, pose, iterations=ICP_ITERATIONS):
    """Refine a pose with trimmed point-to-point ICP; return the new pose.

    Pairs farther apart than a limit are ignored. The limit starts at
    ICP_START_DISTANCE_M and shrinks to MATCH_DISTANCE_M, so the first
    iterations can still pull the scan in, and the last ones use only
    true matches (robust to partial overlap and outliers).
    Each step's correction (d_angle, d_t) is COMPOSED with the pose:
        new p_global = R(d) (R(theta) p + t) + d_t
                     = R(theta + d) p + (R(d) t + d_t)
    """
    x, y, theta_deg = pose
    for iteration in range(iterations):
        placed = transform_points(source, (x, y, theta_deg))
        indices, distances = nearest_neighbours(placed, reference)
        limit = max(MATCH_DISTANCE_M,
                    ICP_START_DISTANCE_M * 0.85 ** iteration)
        pairs = distances < limit
        if pairs.sum() < MIN_ALIGN_POINTS // 3:
            break  # nothing left to pair: keep the current pose
        d_angle, d_t = best_rigid_transform(placed[pairs],
                                            reference[indices[pairs]])
        cos_d, sin_d = math.cos(d_angle), math.sin(d_angle)
        x, y = cos_d * x - sin_d * y + d_t[0], sin_d * x + cos_d * y + d_t[1]
        theta_deg = wrap_angle_deg(theta_deg + math.degrees(d_angle))
        if (limit == MATCH_DISTANCE_M and abs(d_angle) < 1e-5
                and math.hypot(*d_t) < 1e-4):
            break  # converged (which alone does NOT mean "correct")
    return (x, y, theta_deg)


def evaluate_alignment(source, reference, pose):
    """Return (match_fraction, rms_m) of a scan placed with `pose`.

    match_fraction - share of scan points within MATCH_DISTANCE_M of a
                     reference point (low when the scans overlap little)
    rms_m          - root mean square distance of those matched points
    """
    _, distances = nearest_neighbours(transform_points(source, pose),
                                      reference)
    matched = distances < MATCH_DISTANCE_M
    if not matched.any():
        return 0.0, math.inf
    return (float(matched.mean()),
            float(np.sqrt(np.mean(distances[matched] ** 2))))


def global_to_local(points, pose):
    """Inverse of transform_points(): global (N, 2) points -> sensor frame.

    p_local = R(-theta) (p_global - [x, y]): undo the translation, then
    undo the rotation.
    """
    pose_x, pose_y, theta_deg = pose
    cos_t, sin_t = math.cos(math.radians(theta_deg)), \
        math.sin(math.radians(theta_deg))
    dx, dy = points[:, 0] - pose_x, points[:, 1] - pose_y
    return np.column_stack([cos_t * dx + sin_t * dy, -sin_t * dx + cos_t * dy])


def see_through_counts(viewer_points, other_points):
    """Compare a scan with other points expressed in THAT scan's frame.

    viewer_points - (N, 2) local points of one scan (its sensor at 0, 0)
    other_points  - (M, 2) points of another scan, in the same frame
    The viewer's points tell, for each direction (1 deg bins), how far its
    LiDAR could see: the nearest point in that direction. Another point in
    the same direction is
        - CONSISTENT  if it is at about that distance (same wall),
        - a CONFLICT  if it is clearly NEARER: the beam passed through it,
        - unknown     if it is farther (hidden behind what was seen).
    A conflict must be nearer than the scan in the bin AND in its two
    neighbours: along a wall seen at a grazing angle the range changes a
    lot within one degree, and that must not count as seeing through it.
    Returns (conflicts, consistent) counts.
    """
    bins = int(round(360.0 / FREE_SPACE_BIN_DEG))

    def direction_bin(points):
        angles = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
        return ((angles + 180.0) // FREE_SPACE_BIN_DEG).astype(int) % bins

    seen_range = np.full(bins, np.inf)  # how far the scan saw, per direction
    np.minimum.at(seen_range, direction_bin(viewer_points),
                  np.hypot(viewer_points[:, 0], viewer_points[:, 1]))
    nearest_around = np.minimum(np.minimum(np.roll(seen_range, 1),
                                           np.roll(seen_range, -1)),
                                seen_range)
    other_range = np.hypot(other_points[:, 0], other_points[:, 1])
    other_bin = direction_bin(other_points)
    scan_range = seen_range[other_bin]
    observed = np.isfinite(scan_range)
    conflict = observed & (other_range < nearest_around[other_bin]
                           - FREE_SPACE_MARGIN_M)
    consistent = observed & (np.abs(other_range - scan_range)
                             <= FREE_SPACE_MARGIN_M)
    return int(conflict.sum()), int(consistent.sum())


def directional_conflicts(source, reference_scans, pose):
    """free_space_conflict() split by WHO sees through whom.

    Returns (pooled, scan_sees_through, reference_sees_through):
        pooled                 - the value of free_space_conflict()
        scan_sees_through      - share of compared reference points the NEW
                                 scan sees through (its beams go past them)
        reference_sees_through - share of the NEW scan's compared points that
                                 a reference scan sees through
    A wrong pose misplaces walls, so BOTH shares are high. A right pose with
    furniture the reference recorded but the new scan saw past (another
    LiDAR height, or the furniture was moved) raises only the first one.
    """
    placed = transform_points(source, pose)
    counts = np.zeros(4, dtype=int)  # out conflicts, out consistent, in, in
    for reference_local, reference_pose in reference_scans:
        reference_global = transform_points(reference_local, reference_pose)
        counts[:2] += see_through_counts(
            source, global_to_local(reference_global, pose))
        counts[2:] += see_through_counts(
            reference_local, global_to_local(placed, reference_pose))

    def share(conflicts, consistent):
        return conflicts / (conflicts + consistent) if conflicts + consistent \
            else 0.0
    return (share(counts[0] + counts[2], counts[1] + counts[3]),
            share(counts[0], counts[1]), share(counts[2], counts[3]))


def free_space_conflict(source, reference_scans, pose):
    """Share of compared points where one scan "sees through" the other.

    source          - (N, 2) local points of the scan being aligned
    reference_scans - list of (local_points, pose) of the reference scans
    pose            - proposed pose of the source scan
    Checked BOTH ways: the new scan must not see through the reference
    walls, and no reference scan may see through the new scan's points.
    Returns conflicts / (conflicts + consistent), 0 if nothing compares.
    """
    placed = transform_points(source, pose)
    conflicts = consistent = 0
    for reference_local, reference_pose in reference_scans:
        reference_global = transform_points(reference_local, reference_pose)
        for viewer, other in (
                (source, global_to_local(reference_global, pose)),
                (reference_local, global_to_local(placed, reference_pose))):
            new_conflicts, new_consistent = see_through_counts(viewer, other)
            conflicts += new_conflicts
            consistent += new_consistent
    checked = conflicts + consistent
    return conflicts / checked if checked else 0.0


def alignment_score(match_fraction, conflict):
    """Ranking score of a pose: agreements minus weighted contradictions."""
    return match_fraction - CONFLICT_PENALTY * conflict


def one_sided_conflict(result):
    """Too many conflicts, but only in the direction clutter can explain.

    Allowed only with the "good" limits on everything else: at least
    GOOD_MATCH_FRACTION matched, RMS within MAX_RMS_M, and the reference
    scans see through at most GOOD_CONFLICT_FRACTION of the scan's points.
    Never "good": the verdict is at best "weak" (check visually). Real
    classroom data: 0 of 169 wrong candidate poses pass this, the falsely
    rejected Scan 6 (sensor above the desks) does.
    """
    reference_sees_through = result.get("reference_sees_through")
    return (result["conflict"] > MAX_CONFLICT_FRACTION
            and reference_sees_through is not None
            and reference_sees_through <= GOOD_CONFLICT_FRACTION
            and result["match_fraction"] >= GOOD_MATCH_FRACTION
            and result["rms_m"] <= MAX_RMS_M)


def possible_pose(result):
    """Not ruled out by free-space conflicts (see one_sided_conflict)."""
    return result["conflict"] <= MAX_CONFLICT_FRACTION \
        or one_sided_conflict(result)


def judge_alignment(best, runner_up):
    """Return (status, reason): "good", "weak" (check visually) or "failed".

    best, runner_up - {"pose", "match_fraction", "rms_m", "conflict",
    "score"}; runner_up is the best CLEARLY DIFFERENT pose (or None).
    """
    fraction, rms = best["match_fraction"], best["rms_m"]
    one_sided = one_sided_conflict(best)
    if best["conflict"] > MAX_CONFLICT_FRACTION and not one_sided:
        return "failed", (
            f"no consistent pose: even the best one makes the scan see "
            f"through {best['conflict']:.0%} of the reference walls (at "
            f"most {MAX_CONFLICT_FRACTION:.0%} allowed). The scans probably "
            f"do not show the same place.")
    if fraction < MIN_MATCH_FRACTION:
        return "failed", (
            f"insufficient overlap: only {fraction:.0%} of the scan's points "
            f"lie within {MATCH_DISTANCE_M * 100:.0f} cm of the reference "
            f"(at least {MIN_MATCH_FRACTION:.0%} needed).")
    if rms > MAX_RMS_M:
        return "failed", (
            f"poor fit: the matched points are {rms * 100:.1f} cm from the "
            f"reference (RMS; at most {MAX_RMS_M * 100:.1f} cm accepted).")
    if (runner_up is not None
            and runner_up["score"] >= AMBIGUITY_RATIO * best["score"]):
        x, y, theta = runner_up["pose"]
        return "failed", (
            f"ambiguous: a clearly different pose (x {x:.2f} m, y {y:.2f} m, "
            f"theta {theta:.0f} deg) fits almost as well (score "
            f"{runner_up['score']:.2f} vs {best['score']:.2f}). Typical of "
            f"symmetric rooms: scan where a door, desk or pillar is visible, "
            f"or align by hand.")
    runner_text = (f"; best different pose scores only "
                   f"{runner_up['score']:.2f} vs {best['score']:.2f}"
                   if runner_up else "")
    summary = (f"{fraction:.0%} of points match, RMS {rms * 100:.1f} cm, "
               f"{best['conflict']:.0%} see-through conflicts{runner_text}")
    if one_sided:
        return "weak", (
            f"plausible but CHECK VISUALLY: {summary}. The conflicts are "
            f"ONE-SIDED: this scan sees past "
            f"{best['scan_sees_through']:.0%} of the reference obstacles, "
            f"but the reference scans see through only "
            f"{best['reference_sees_through']:.0%} of this scan's points. "
            f"Typical of a LiDAR placed higher than the others (it sees over "
            f"desks) or of moved furniture; walls must coincide.")
    if (fraction >= GOOD_MATCH_FRACTION and rms <= GOOD_RMS_M
            and best["conflict"] <= GOOD_CONFLICT_FRACTION):
        return "good", f"good: {summary}."
    return "weak", (
        f"plausible but CHECK VISUALLY: {summary}. Good needs >= "
        f"{GOOD_MATCH_FRACTION:.0%} matched, RMS <= "
        f"{GOOD_RMS_M * 100:.1f} cm, <= {GOOD_CONFLICT_FRACTION:.0%} "
        f"conflicts.")


def failed_alignment(reason, start_time):
    """A result dictionary for an alignment that could not be computed."""
    return {"status": "failed", "reason": reason, "pose": None,
            "match_fraction": 0.0, "rms_m": math.inf, "conflict": 0.0,
            "scan_sees_through": 0.0, "reference_sees_through": 0.0,
            "score": 0.0, "runner_up": None, "coarse_pose": None,
            "runtime_s": time.perf_counter() - start_time}


class AlignmentCancelled(Exception):
    """Raised by align_scan() when its should_stop() callback returns True."""


ALIGN_HEADING_BATCH = 10  # headings scored between progress / cancel checks


def align_scan(source_local, reference_scans, search_window=None,
               progress=None, should_stop=None):
    """Estimate the pose of a scan on the reference map (any rotation).

    source_local    - (N, 2) ORIGINAL local points of the scan to align
    reference_scans - list of (local_points, pose) of the already aligned
                      scans; together they form the reference map
    search_window   - None (default): the whole reference area and every
                      heading, as Auto Align always did. Or (x_m, y_m,
                      theta_deg, half_width_m, half_angle_deg): only poses
                      within that window around a guess (local search).
    progress(fraction), should_stop() - optional callbacks for a GUI;
                      should_stop() returning True raises AlignmentCancelled.
    With the default arguments the result is exactly the original one.
    Neither input is modified. Returns a dictionary:
        "status"    - "good", "weak" or "failed" (see judge_alignment)
        "reason"    - one sentence explaining the status
        "pose"      - best pose found (x_m, y_m, theta_deg), or None
        "match_fraction", "rms_m", "conflict", "score" - quality of it
        "runner_up" - best clearly different pose (dict) or None
        "coarse_pose" - the coarse-search pose the answer was refined from
        "runtime_s" - computation time
    The robot is assumed to stand somewhere inside the area covered by
    the reference points (true when it stays in the same classroom).
    """
    start_time = time.perf_counter()
    source = voxel_downsample(np.asarray(source_local, dtype=float),
                              ALIGN_VOXEL_M)
    reference_scans = [(voxel_downsample(np.asarray(points, dtype=float),
                                         ALIGN_VOXEL_M), tuple(pose))
                       for points, pose in reference_scans]
    reference = voxel_downsample(np.vstack(
        [transform_points(points, pose) for points, pose in reference_scans]
        or [np.empty((0, 2))]), ALIGN_VOXEL_M)
    if len(source) < MIN_ALIGN_POINTS or len(reference) < MIN_ALIGN_POINTS:
        return failed_alignment(
            f"not enough points to align ({len(source)} in the scan, "
            f"{len(reference)} in the reference after thinning to "
            f"{ALIGN_VOXEL_M * 100:.0f} cm; {MIN_ALIGN_POINTS} needed).",
            start_time)

    def check_stop():
        if should_stop is not None and should_stop():
            raise AlignmentCancelled

    # Stage 1a: coarse search, ALL headings, whole reference area (or the
    # local window). Headings are scored in batches: same scores, but a GUI
    # can show progress and cancel between batches.
    coarse_field = build_likelihood_field(reference, ALIGN_COARSE_CELL_M,
                                          ALIGN_COARSE_SIGMA_M)
    if search_window is None:
        low, high = reference.min(axis=0), reference.max(axis=0)
        positions = position_grid((low[0], high[0]), (low[1], high[1]),
                                  ALIGN_COARSE_STEP_M)
        headings = np.arange(0.0, 360.0, ALIGN_COARSE_STEP_DEG)
    else:
        x, y, theta, half_width, half_angle = search_window
        positions = position_grid((x - half_width, x + half_width),
                                  (y - half_width, y + half_width),
                                  ALIGN_COARSE_STEP_M)
        headings = theta + np.arange(-half_angle, half_angle + 1e-9,
                                     ALIGN_COARSE_STEP_DEG)
    coarse_source = voxel_downsample(source, ALIGN_COARSE_STEP_M)
    scores = np.empty((len(headings), len(positions)))
    for first in range(0, len(headings), ALIGN_HEADING_BATCH):
        check_stop()
        batch = slice(first, first + ALIGN_HEADING_BATCH)
        scores[batch] = correlative_search(coarse_source, coarse_field,
                                           headings[batch], positions)
        if progress is not None:
            progress(0.6 * min(1.0, (first + ALIGN_HEADING_BATCH)
                               / len(headings)))
    candidates = pick_distinct_poses(scores, headings, positions,
                                     ALIGN_CANDIDATES)

    # Stage 1b + 2: fine search then ICP from each candidate; measure each.
    fine_field = build_likelihood_field(reference, ALIGN_FINE_CELL_M,
                                        ALIGN_FINE_SIGMA_M)
    icp_source = voxel_downsample(source, ALIGN_ICP_VOXEL_M)
    refined = []
    for number, (coarse_pose, _) in enumerate(candidates):
        check_stop()
        pose = refine_by_search(source, fine_field, coarse_pose)
        pose = icp(icp_source, reference, pose)
        fraction, rms = evaluate_alignment(source, reference, pose)
        conflict, scan_sees, reference_sees = directional_conflicts(
            source, reference_scans, pose)
        result = {"pose": pose, "match_fraction": fraction, "rms_m": rms,
                  "conflict": conflict, "scan_sees_through": scan_sees,
                  "reference_sees_through": reference_sees,
                  "score": alignment_score(fraction, conflict),
                  "coarse_pose": coarse_pose}
        if one_sided_conflict(result):
            # Only the conflicts clutter cannot explain count in its score.
            result["score"] = alignment_score(fraction, reference_sees)
        refined.append(result)
        if progress is not None:
            progress(0.6 + 0.4 * (number + 1) / len(candidates))

    # Stage 3: impossible poses (too many conflicts) are ranked last, poses
    # with one-sided conflicts after every ordinary possible pose, then the
    # highest score wins. Runner-up = best clearly different possible one.
    refined.sort(key=lambda result: (
        not possible_pose(result), one_sided_conflict(result),
        -result["score"], result["rms_m"]))
    best = refined[0]
    runner_up = next((result for result in refined[1:]
                      if possible_pose(result)
                      and poses_are_distinct(result["pose"], best["pose"])),
                     None)
    status, reason = judge_alignment(best, runner_up)
    return {"status": status, "reason": reason, "pose": best["pose"],
            "match_fraction": best["match_fraction"], "rms_m": best["rms_m"],
            "conflict": best["conflict"], "score": best["score"],
            "scan_sees_through": best["scan_sees_through"],
            "reference_sees_through": best["reference_sees_through"],
            "runner_up": runner_up, "coarse_pose": best["coarse_pose"],
            "runtime_s": time.perf_counter() - start_time}


def start_alignment_thread(source_local, reference_scans, results, tag):
    """Run align_scan() in a background thread so the window stays usable.

    The inputs must be COPIES (the GUI may change the session meanwhile).
    When done, (tag, result) is put on the `results` queue.
    """
    def work():
        start_time = time.perf_counter()
        try:
            result = align_scan(source_local, reference_scans)
        except Exception as error:  # report it; never leave the GUI waiting
            result = failed_alignment(f"internal error: {error!r}",
                                      start_time)
        results.put((tag, result))

    thread = threading.Thread(target=work, daemon=True)
    thread.start()
    return thread


# ---------------------------------------------------------------------------
# Mapping session: any number of numbered scans (pure data, no GUI code)
# ---------------------------------------------------------------------------
# A session is a dictionary:
#     "scans"       : a LIST of scans, in the order they were added. Each
#                     scan comes from create_cartesian_scan(): the ORIGINAL
#                     local points (sensor frame, metres) and the current
#                     pose, plus two keys: "id" (its number) and "visible".
#     "next_id"     : the number the next new scan will get
#     "selected_id" : number of the scan the sliders move, or None
# Each scan also has "aligned" (True once an automatic alignment was
# ACCEPTED) and "alignment_quality" (that alignment's measurements, or
# None). The FIRST scan in the list is the fixed reference of the map.
# Scan numbers are NEVER reused. After deleting Scan 2, the next new scan is
# still numbered after the highest number ever given, so "Scan 2" always
# means the same measurements, also in saved files and in your report.
# Global points are never stored: they are recomputed from the local points
# and the pose with local_to_global() each time the map is drawn.
SCAN_COLORS = ("tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple",
               "tab:brown", "tab:pink", "tab:gray", "tab:olive", "tab:cyan")


def color_for_scan_id(scan_id):
    """Colour of a scan: Matplotlib's 10 distinct colours, then repeat."""
    return SCAN_COLORS[(scan_id - 1) % len(SCAN_COLORS)]


def create_session():
    """Return an empty session: no scans, next number 1, nothing selected."""
    return {"scans": [], "next_id": 1, "selected_id": None}


def find_scan_index(session, scan_id):
    """Return the position of a scan in session["scans"] (KeyError if none).

    Compares the numbers only: comparing whole scan dictionaries would
    compare their NumPy arrays, which Python cannot turn into True/False.
    """
    for index, scan in enumerate(session["scans"]):
        if scan["id"] == scan_id:
            return index
    raise KeyError(f"there is no Scan {scan_id}")


def find_scan(session, scan_id):
    """Return the scan with this number (KeyError if there is none)."""
    return session["scans"][find_scan_index(session, scan_id)]


def selected_scan(session):
    """Return the selected scan, or None if nothing is selected."""
    if session["selected_id"] is None:
        return None
    return find_scan(session, session["selected_id"])


def add_scan(session, x_local_m, y_local_m):
    """Append a NEW scan with the next number, select it and return it.

    The points are COPIED, so nothing outside can change the stored
    originals. The new scan is visible and starts at pose (0, 0, 0).
    """
    scan_id = session["next_id"]
    scan = create_cartesian_scan(
        f"Scan {scan_id}", color_for_scan_id(scan_id), (0.0, 0.0, 0.0),
        np.array(x_local_m, dtype=float), np.array(y_local_m, dtype=float))
    scan["id"] = scan_id
    scan["visible"] = True
    scan["aligned"] = False
    scan["alignment_quality"] = None
    session["scans"].append(scan)
    session["next_id"] = scan_id + 1
    session["selected_id"] = scan_id
    return scan


def replace_scan_points(session, scan_id, x_local_m, y_local_m):
    """Give one existing scan NEW measurements; nothing else changes.

    Number, name, colour and POSE are kept: Replace is meant for redoing a
    bad scan from the same place (e.g. someone walked past the LiDAR), so
    its alignment usually still fits. Use Reset pose if it does not.
    The scan is made visible so the new points are seen. It is no longer
    "aligned": an accepted alignment checked the OLD measurements.
    """
    scan = find_scan(session, scan_id)
    scan["x_local_m"] = np.array(x_local_m, dtype=float)
    scan["y_local_m"] = np.array(y_local_m, dtype=float)
    scan["visible"] = True
    scan["aligned"] = False
    scan["alignment_quality"] = None
    return scan


def delete_scan(session, scan_id):
    """Remove one scan; if it was selected, select its neighbour.

    The scan that takes its place in the list is selected (or the new last
    one), or None if the session is now empty. Numbers are not reused.
    """
    index = find_scan_index(session, scan_id)
    del session["scans"][index]
    if session["selected_id"] == scan_id:
        remaining = session["scans"]
        session["selected_id"] = (remaining[min(index, len(remaining) - 1)]["id"]
                                  if remaining else None)


def set_scan_pose(session, scan_id, pose):
    """Set the pose (x_m, y_m, theta_deg) of ONE scan; points are unchanged."""
    find_scan(session, scan_id)["pose"] = tuple(float(value) for value in pose)


def reset_scan_pose(session, scan_id):
    """Put one scan back at pose (0, 0, 0); it is no longer "aligned"."""
    set_scan_pose(session, scan_id, (0.0, 0.0, 0.0))
    scan = find_scan(session, scan_id)
    scan["aligned"] = False
    scan["alignment_quality"] = None


def reference_scan(session):
    """The fixed reference of the map: the first scan (None if empty)."""
    return session["scans"][0] if session["scans"] else None


def alignment_reference_scans(session, scan_id):
    """Scans that form the map when aligning `scan_id`.

    The reference scan plus every scan whose automatic alignment was
    ACCEPTED, leaving out the scan itself and hidden scans (hide a scan to
    keep it out of the reference). Unverified scans are never used.
    """
    first = reference_scan(session)
    return [scan for scan in session["scans"]
            if scan["id"] != scan_id and scan["visible"]
            and (scan is first or scan["aligned"])]


def accept_alignment(session, scan_id, pose, quality):
    """Apply an accepted automatic alignment: ONLY the pose changes.

    quality - the measurements that justified it (stored and saved).
    The local points are not touched.
    """
    set_scan_pose(session, scan_id, pose)
    scan = find_scan(session, scan_id)
    scan["aligned"] = True
    scan["alignment_quality"] = dict(quality)


def set_scan_visible(session, scan_id, visible):
    """Show (True) or hide (False) one scan."""
    find_scan(session, scan_id)["visible"] = bool(visible)


def shown_scans(session):
    """Return the visible scans, in list order."""
    return [scan for scan in session["scans"] if scan["visible"]]


def compute_view_limits(session, empty_half_width_m=5.0):
    """Return ((x_min, x_max), (y_min, y_max)) framing every shown scan.

    Includes the global points and the robot positions, plus a margin.
    With nothing to show: a square of +-empty_half_width_m around (0, 0).
    """
    x_parts, y_parts = [], []
    for scan in shown_scans(session):
        x_global, y_global = scan_to_global(scan)
        x_parts += [x_global, [scan["pose"][0]]]
        y_parts += [y_global, [scan["pose"][1]]]
    if not x_parts:
        return ((-empty_half_width_m, empty_half_width_m),
                (-empty_half_width_m, empty_half_width_m))
    x = np.concatenate(x_parts)
    y = np.concatenate(y_parts)
    margin = 0.1 * max(x.max() - x.min(), y.max() - y.min()) + 0.3
    return ((x.min() - margin, x.max() + margin),
            (y.min() - margin, y.max() + margin))


def replace_session_contents(session, new_session):
    """Copy `new_session` into `session` IN PLACE.

    The editor's functions all refer to the same dictionary, so a loaded
    session must replace the contents, not the dictionary itself.
    """
    session.clear()
    session.update(new_session)


# ---------------------------------------------------------------------------
# Session files (.json in scans/): versioned, readable, validated on load
# ---------------------------------------------------------------------------
# Version 2 (written now): "scans" is a LIST; each scan has its "id".
# Version 1 (previous program): "scans" was {"A": ..., "B": ..., "C": ...}.
# Version 1 files are still READ and converted (see session_from_version_1);
# they are never modified on disk unless you save over them.
SESSION_FORMAT_NAME = "ld06_slam_session"
SESSION_FORMAT_VERSION = 2
LEGACY_SLOT_NAMES = ("A", "B", "C")  # version 1 only
SESSION_CONVENTIONS = {
    "units": "metres and degrees",
    "points_local_m": "ORIGINAL measurements in the sensor frame, never "
                      "transformed: +X = LiDAR front, +Y = LiDAR left",
    "pose": "sensor position (x_m, y_m) in the global map frame and heading "
            "theta_deg, counter-clockwise from the global +X axis",
    "global_point": "x_g = x_m + x*cos(theta) - y*sin(theta); "
                    "y_g = y_m + x*sin(theta) + y*cos(theta)",
    "aligned": "true once an automatic alignment was accepted; "
               "alignment_quality holds the measurements that justified it",
    "reference": "the first scan in the list is the fixed map reference",
    "scan_ids": "unique numbers, never reused after a deletion; "
                "next_scan_id is the number of the next new scan",
    "source": "LD06 Teleplot XY (mm) from ld06.printScanTeleplot(), "
              "converted to metres with the library's Y axis flipped",
}


class SessionFileError(Exception):
    """A session file is missing, is not valid JSON or has a bad structure."""


def session_path_from_name(file_name):
    """Turn the name typed in the File box into a full path.

    A bare name such as "room1" becomes scans/room1.json; an absolute path
    is used as it is. An empty box means DEFAULT_SESSION_FILE.
    """
    path = Path(file_name.strip() or DEFAULT_SESSION_FILE)
    if not path.suffix:
        path = path.with_suffix(".json")
    return path if path.is_absolute() else SCANS_DIR / path


def session_to_json_dict(session):
    """Return the session as plain lists/dicts ready for json.dump().

    Only the ORIGINAL local points are written, rounded to the micrometre;
    global points can always be recomputed from them and the pose.
    """
    scans = []
    for scan in session["scans"]:
        x_m, y_m, theta_deg = scan["pose"]
        points = [[round(float(x), 6), round(float(y), 6)]
                  for x, y in zip(scan["x_local_m"], scan["y_local_m"])]
        scans.append({
            "id": scan["id"],
            "name": scan["label"],
            "color": scan["color"],
            "visible": scan["visible"],
            "aligned": scan["aligned"],
            "alignment_quality": scan["alignment_quality"],
            "pose": {"x_m": float(x_m), "y_m": float(y_m),
                     "theta_deg": float(theta_deg)},
            "point_count": len(points),
            "points_local_m": points,
        })
    return {
        "format": SESSION_FORMAT_NAME,
        "version": SESSION_FORMAT_VERSION,
        "saved_at": datetime.now().isoformat(timespec="seconds"),
        "conventions": SESSION_CONVENTIONS,
        "selected_scan_id": session["selected_id"],
        "next_scan_id": session["next_id"],
        "scans": scans,
    }


def save_session(session, path):
    """Write the session to `path` as readable JSON (one point per line).

    The file is written under a temporary name first and then renamed, so
    an interrupted save can never leave a half-written session file.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)  # creates scans/ if needed
    text = json.dumps(session_to_json_dict(session), indent=2, allow_nan=False)
    # indent=2 puts each number of [x, y] on its own line; join every
    # point back onto one line. Only whitespace changes: still valid JSON.
    text = re.sub(r"\[\s+(\S+),\s+(\S+)\s+\]", r"[\1, \2]", text)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(text + "\n", encoding="utf-8")
    temporary.replace(path)


def require(condition, message):
    """Raise SessionFileError(message) if `condition` is False."""
    if not condition:
        raise SessionFileError(message)


def is_finite_number(value):
    """True for an int or float that is not NaN/infinite (and not a bool)."""
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def is_scan_number(value):
    """True for a whole number >= 1 (a bool such as true is NOT accepted)."""
    return isinstance(value, int) and not isinstance(value, bool) and value >= 1


def scan_from_json_dict(entry, where, scan_id):
    """Check one saved scan's pose, points, name and colour; rebuild it.

    `where` names the entry in error messages (e.g. "scans[3]"). The
    caller adds the "id" and "visible" keys.
    """
    require(isinstance(entry, dict), f"{where} must be an object")

    pose = entry.get("pose")
    require(isinstance(pose, dict)
            and all(is_finite_number(pose.get(key))
                    for key in ("x_m", "y_m", "theta_deg")),
            f"{where}.pose needs the numbers x_m, y_m and theta_deg")

    points = entry.get("points_local_m")
    require(isinstance(points, list) and len(points) > 0,
            f"{where}.points_local_m must be a non-empty list of [x, y]")
    for index, point in enumerate(points):
        require(isinstance(point, list) and len(point) == 2
                and all(is_finite_number(value) for value in point),
                f"{where}.points_local_m[{index}] must be [x, y] numbers")
    count = entry.get("point_count", len(points))
    require(count == len(points),
            f"{where}.point_count is {count} but {len(points)} points are "
            f"stored (file damaged or edited?)")

    name = entry.get("name", f"Scan {scan_id}")
    require(isinstance(name, str), f"{where}.name must be text")
    color = entry.get("color")
    if not (isinstance(color, str) and matplotlib.colors.is_color_like(color)):
        color = color_for_scan_id(scan_id)  # unknown colour: use the default

    # Same heading, written in [-180, 180) to match the theta slider.
    theta_deg = (pose["theta_deg"] + 180.0) % 360.0 - 180.0
    xy = np.array(points, dtype=float)
    return create_cartesian_scan(
        name, color, (float(pose["x_m"]), float(pose["y_m"]), theta_deg),
        xy[:, 0].copy(), xy[:, 1].copy())


def session_from_json_dict(data):
    """Check data read from a session file; return (session, version).

    Reads version 2, and converts version 1 (A/B/C) files. Raises
    SessionFileError with a clear message for anything unexpected, so a bad
    file can never replace the session currently on screen.
    """
    require(isinstance(data, dict), "the file does not contain a JSON object")
    require(data.get("format") == SESSION_FORMAT_NAME,
            f"not an LD06 SLAM session ('format' must be "
            f"'{SESSION_FORMAT_NAME}')")
    version = data.get("version")
    if version == 1:
        return session_from_version_1(data), 1
    require(version == SESSION_FORMAT_VERSION,
            f"unsupported version {version!r} (this program reads versions "
            f"1 and {SESSION_FORMAT_VERSION})")

    scans_data = data.get("scans")
    require(isinstance(scans_data, list), "'scans' must be a list")
    session = create_session()
    used_ids = set()
    for index, entry in enumerate(scans_data):
        where = f"scans[{index}]"
        require(isinstance(entry, dict), f"{where} must be an object")
        scan_id = entry.get("id")
        require(is_scan_number(scan_id),
                f"{where}.id must be a whole number >= 1")
        require(scan_id not in used_ids, f"scan number {scan_id} is used twice")
        used_ids.add(scan_id)
        visible = entry.get("visible", True)
        require(isinstance(visible, bool),
                f"{where}.visible must be true or false")
        # "aligned" and "alignment_quality" are optional: version 2 files
        # saved before automatic alignment existed simply lack them.
        aligned = entry.get("aligned", False)
        require(isinstance(aligned, bool),
                f"{where}.aligned must be true or false")
        quality = entry.get("alignment_quality")
        require(quality is None or (
            isinstance(quality, dict)
            and all(value is None or isinstance(value, str)
                    or is_finite_number(value) for value in quality.values())),
            f"{where}.alignment_quality must be null or an object of "
            f"numbers and texts")
        scan = scan_from_json_dict(entry, where, scan_id)
        scan["id"] = scan_id
        scan["visible"] = visible
        scan["aligned"] = aligned
        scan["alignment_quality"] = quality
        session["scans"].append(scan)

    highest_id = max(used_ids, default=0)
    next_id = data.get("next_scan_id", highest_id + 1)
    require(is_scan_number(next_id), "'next_scan_id' must be a whole number")
    session["next_id"] = max(next_id, highest_id + 1)  # never reuse a number

    selected_id = data.get("selected_scan_id")
    require(selected_id is None or (is_scan_number(selected_id)
                                    and selected_id in used_ids),
            f"'selected_scan_id' {selected_id!r} is not a scan in the file")
    session["selected_id"] = selected_id
    return session, SESSION_FORMAT_VERSION


def session_from_version_1(data):
    """Convert an old A/B/C session (version 1) into the current structure.

    Slot A becomes Scan 1, B Scan 2 and C Scan 3. An empty slot simply has
    no scan, so the letter-number link stays obvious. Points, poses,
    colours and visibility are kept, and so is the old name, e.g.
    "Scan 1 (was Scan A)". The file on disk is not changed.
    """
    scans_data = data.get("scans")
    require(isinstance(scans_data, dict),
            "'scans' must be an object in a version 1 file")
    unknown = sorted(set(scans_data) - set(LEGACY_SLOT_NAMES))
    require(not unknown, f"unknown scan slot(s) {unknown}; expected A, B, C")
    visible = data.get("visible", {})
    require(isinstance(visible, dict), "'visible' must be an object")
    for slot, value in visible.items():
        require(slot in LEGACY_SLOT_NAMES and isinstance(value, bool),
                f"visible.{slot} must be true or false for slot A, B or C")
    selected_slot = data.get("selected_scan", "A")
    require(selected_slot in LEGACY_SLOT_NAMES,
            "'selected_scan' must be A, B or C")

    session = create_session()
    for scan_id, slot in enumerate(LEGACY_SLOT_NAMES, start=1):
        if scans_data.get(slot) is None:
            continue  # this slot was never captured
        scan = scan_from_json_dict(scans_data[slot], f"scans.{slot}", scan_id)
        scan["label"] = f"Scan {scan_id} (was {scan['label']})"
        scan["id"] = scan_id
        scan["visible"] = visible.get(slot, True)
        scan["aligned"] = False  # version 1 had no automatic alignment
        scan["alignment_quality"] = None
        session["scans"].append(scan)
        if slot == selected_slot:
            session["selected_id"] = scan_id
    if session["scans"]:
        session["next_id"] = session["scans"][-1]["id"] + 1
        if session["selected_id"] is None:
            session["selected_id"] = session["scans"][0]["id"]
    return session


def load_session(path):
    """Read and validate a session file; return (session, file_version).

    Every problem (missing file, not text, not JSON, wrong structure) is
    raised as SessionFileError, so the caller handles a single error type.
    """
    path = Path(path)
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as error:
        raise SessionFileError(f"cannot read {path}: "
                               f"{error.strerror or error}") from error
    except UnicodeDecodeError as error:
        raise SessionFileError(f"{path.name} is not a text file") from error
    try:
        data = json.loads(text)
    except json.JSONDecodeError as error:
        raise SessionFileError(f"{path.name} is not valid JSON (line "
                               f"{error.lineno}: {error.msg})") from error
    return session_from_json_dict(data)


# ---------------------------------------------------------------------------
# Mapping window: add/replace/delete scans, align with sliders, save/load
# ---------------------------------------------------------------------------
POSE_SLIDER_LIMIT_M = 8.0   # x and y sliders go from -8 m to +8 m
POSE_ARROW_LENGTH_M = 0.4
LIST_ROWS = 10              # rows shown at once in the scan list (scrollable)
LEGEND_MAX_ENTRIES = 6      # more visible scans: legend shows the selected one
DISPLAY_MODES = ("Points only", "Points + rays", "Rays only")
DEFAULT_DISPLAY_MODE = "Points + rays"
DEFAULT_RAY_OPACITY = 0.08  # faint: hundreds of rays overlap near the sensor
RAY_LINE_WIDTH = 0.5
STATUS_TITLES = {"good": "GOOD", "weak": "CHECK VISUALLY",
                 "failed": "NOT ACCEPTABLE"}
STATUS_COLORS = {"good": "tab:green", "weak": "tab:orange",
                 "failed": "tab:red"}


def widen_slider_range(slider, value):
    """Extend a slider so it can show `value` (e.g. a pose from a file)."""
    if value < slider.valmin or value > slider.valmax:
        slider.valmin = min(slider.valmin, math.floor(value))
        slider.valmax = max(slider.valmax, math.ceil(value))
        slider.ax.set_xlim(slider.valmin, slider.valmax)


def scroll_to_show(top, index, rows, total):
    """Return the first list row to display so that `index` is on screen.

    top   - first row displayed now
    index - position of the row that must be visible (None: no constraint)
    The result is also kept within [0, total - rows].
    """
    if index is not None:
        if index < top:
            top = index
        elif index >= top + rows:
            top = index - rows + 1
    return max(0, min(top, total - rows))


def format_scan_row(scan, is_selected, status=None, alignment=""):
    """One line of the scan list, e.g. '> Scan 12        452 pts shown  ok'.

    alignment - "ref" (fixed reference), "ok" (accepted automatic
                alignment), "..." (being aligned) or "" (not verified)
    """
    marker = ">" if is_selected else " "
    status = status or ("shown" if scan["visible"] else "hidden")
    return (f"{marker} {scan['label'][:14]:<14} "
            f"{len(scan['x_local_m']):>5} pts {status:<6} {alignment}")


def scan_ray_segments(scan):
    """Return the LiDAR rays of a scan as an (N, 2, 2) array of segments.

    Segment i goes from the sensor to measured point i, in the GLOBAL frame:
        start = sensor origin (0, 0) moved by the pose = (x, y)
        end   = R(theta) p_i + (x, y)   (the global point, scan_to_global)
    A rigid transformation maps a segment onto a segment, so transforming
    the two ends is enough: the rays follow every slider move exactly.
    """
    x_global, y_global = scan_to_global(scan)
    segments = np.empty((len(x_global), 2, 2))
    segments[:, 0, 0], segments[:, 0, 1] = scan["pose"][0], scan["pose"][1]
    segments[:, 1, 0], segments[:, 1, 1] = x_global, y_global
    return segments


def create_scan_artists(axes, scan):
    """Create the Matplotlib objects that draw one scan; return them.

    They are created once per scan and then only UPDATED, so a zoom made
    with the toolbar survives slider moves. All the rays of a scan are ONE
    LineCollection: one object drawing N segments, much faster than N
    separate lines.
    """
    color = scan["color"]
    rays = LineCollection([], linewidths=RAY_LINE_WIDTH, colors=color,
                          zorder=1)
    axes.add_collection(rays, autolim=False)
    points = axes.scatter([], [], s=4, color=color, zorder=2)
    marker, = axes.plot([], [], marker="o", markersize=8, color=color,
                        markeredgecolor="black", linestyle="none", zorder=3)
    arrow = FancyArrowPatch((0, 0), (POSE_ARROW_LENGTH_M, 0), arrowstyle="->",
                            mutation_scale=15, linewidth=2, color=color,
                            zorder=3)
    axes.add_patch(arrow)
    number = axes.text(0, 0, str(scan["id"]), fontsize=9, fontweight="bold",
                       color=color, ha="left", va="bottom", zorder=4)
    return {"rays": rays, "points": points, "marker": marker, "arrow": arrow,
            "number": number}


def update_scan_artists(scan_artists, scan, is_selected,
                        display_mode=DEFAULT_DISPLAY_MODE,
                        ray_opacity=DEFAULT_RAY_OPACITY):
    """Recompute one scan's drawing from its local points + pose.

    A hidden scan hides everything, rays included. Otherwise the pose dot,
    arrow and number are always shown (where the scan was taken), and the
    display mode chooses points, rays or both.
    """
    for artist in scan_artists.values():
        artist.set_visible(scan["visible"])
        artist.set_color(scan["color"])
    show_points = scan["visible"] and display_mode != "Rays only"
    show_rays = scan["visible"] and display_mode != "Points only"
    scan_artists["points"].set_visible(show_points)
    scan_artists["rays"].set_visible(show_rays)
    scan_artists["rays"].set_alpha(ray_opacity)
    x_global, y_global = scan_to_global(scan)  # local_to_global() inside
    scan_artists["points"].set_offsets(np.column_stack([x_global, y_global]))
    if show_rays:  # only computed when they will be drawn
        scan_artists["rays"].set_segments(scan_ray_segments(scan))
    pose_x, pose_y, pose_theta_deg = scan["pose"]
    heading_rad = math.radians(pose_theta_deg)
    scan_artists["marker"].set_data([pose_x], [pose_y])
    scan_artists["marker"].set_markersize(12 if is_selected else 8)
    scan_artists["arrow"].set_positions(
        (pose_x, pose_y),
        (pose_x + POSE_ARROW_LENGTH_M * math.cos(heading_rad),
         pose_y + POSE_ARROW_LENGTH_M * math.sin(heading_rad)))
    scan_artists["number"].set_position((pose_x + 0.08, pose_y + 0.08))


def format_alignment_report(scan, proposal):
    """Lines of text describing an alignment proposal (shown on the map)."""
    result = proposal["result"]
    lines = [f"Proposal for {scan['label']}: {STATUS_TITLES[result['status']]}"]
    if result["pose"] is not None:
        x, y, theta = result["pose"]
        cx, cy, ctheta = scan["pose"]
        lines.append(f"proposed pose  x {x:.2f} m  y {y:.2f} m  "
                     f"theta {theta:.1f} deg")
        lines.append(f"current pose   x {cx:.2f} m  y {cy:.2f} m  "
                     f"theta {ctheta:.1f} deg")
        lines.append(f"matched {result['match_fraction']:.0%} (within "
                     f"{MATCH_DISTANCE_M * 100:.0f} cm) | RMS "
                     f"{result['rms_m'] * 100:.1f} cm | see-through "
                     f"{result['conflict']:.0%} (scan past reference "
                     f"{result['scan_sees_through']:.0%}, reference through "
                     f"scan {result['reference_sees_through']:.0%})")
    lines += textwrap.wrap(result["reason"], 70)
    reference_names = ", ".join(str(scan_id)
                                for scan_id in proposal["reference_ids"])
    lines.append(f"reference: Scan(s) {reference_names} | computed in "
                 f"{result['runtime_s']:.1f} s")
    if result["status"] == "failed":
        lines.append("Cannot be accepted. Reject closes it; pose unchanged.")
    else:
        lines.append("Black = proposal. Accept or Reject: pose NOT changed "
                     "yet.")
    return lines


def create_session_editor(session, worker=None, start_message="",
                          file_name=DEFAULT_SESSION_FILE, figure=None):
    """Open the mapping window and return (figure, controls).

    session   - a session dictionary (create_session()); the window edits it
    worker    - from start_serial_worker(), or None to work OFFLINE
    file_name - initial text of the File box (relative to scans/)

    The caller must keep `controls` in a variable: Matplotlib widgets stop
    responding if Python deletes them. controls["actions"] also lets the
    tests press the buttons without a mouse.
    """
    # A host may supply a Figure with its canvas already attached. This keeps
    # widget callbacks and the polling timer on the host's single event loop.
    figure = figure if figure is not None else plt.figure(figsize=(13, 8))
    map_axes = figure.add_axes([0.32, 0.25, 0.66, 0.71])
    # syncing: True while the program itself moves the sliders, so their
    # callbacks do not treat it as a user action.
    state = {"syncing": False, "capturing": None, "confirm_overwrite": None,
             "confirm_delete": None, "list_top": 0, "aligning": None,
             "proposal": None, "display_mode": DEFAULT_DISPLAY_MODE,
             "ray_opacity": DEFAULT_RAY_OPACITY}
    alignment_results = queue.Queue()  # filled by the alignment thread
    artists = {}  # scan number -> its Matplotlib objects on the map
    map_axes.set_xlabel("X (m)")
    map_axes.set_ylabel("Y (m)")
    map_axes.set_aspect("equal", adjustable="box")  # 1 m = 1 m on X and Y
    map_axes.grid(True)

    # Proposal preview: the scan drawn in black at the PROPOSED pose, while
    # its own coloured drawing stays at the current pose until Accept.
    proposal_points = map_axes.scatter([], [], s=5, color="black", zorder=5)
    proposal_marker, = map_axes.plot([], [], marker="D", markersize=9,
                                     color="black", linestyle="none",
                                     zorder=6)
    proposal_arrow = FancyArrowPatch((0, 0), (POSE_ARROW_LENGTH_M, 0),
                                     arrowstyle="->", mutation_scale=15,
                                     linewidth=2, color="black",
                                     linestyle="--", zorder=6)
    map_axes.add_patch(proposal_arrow)
    report_text = map_axes.text(
        0.01, 0.01, "", transform=map_axes.transAxes, va="bottom",
        fontsize=8, family="monospace", zorder=7,
        bbox={"facecolor": "white", "alpha": 0.9, "edgecolor": "gray"})

    # --- Left panel. Each area is [left, bottom, width, height].
    figure.text(0.02, 0.965, "1. Capture (robot still)", fontweight="bold")
    add_button = Button(figure.add_axes([0.02, 0.905, 0.105, 0.05]),
                        "Add scan")
    replace_button = Button(figure.add_axes([0.13, 0.905, 0.105, 0.05]),
                            "Replace selected")

    # The scan list: LIST_ROWS lines of text in one small axes. Clicking a
    # line selects that scan; the arrows or the mouse wheel scroll it.
    list_title = figure.text(0.02, 0.865, "", fontweight="bold")
    list_axes = figure.add_axes([0.02, 0.52, 0.215, 0.33])
    list_axes.set_xlim(0, 1)
    list_axes.set_ylim(0, 1)
    list_axes.set_xticks([])
    list_axes.set_yticks([])
    selection_box = Rectangle((0, 0), 1, 1 / LIST_ROWS, color="0.88",
                              zorder=0)
    list_axes.add_patch(selection_box)
    row_texts = [list_axes.text(0.02, 1 - (row + 0.5) / LIST_ROWS, "",
                                va="center", family="monospace", fontsize=8)
                 for row in range(LIST_ROWS)]
    scroll_up_button = Button(figure.add_axes([0.24, 0.80, 0.025, 0.05]), "▲")
    scroll_down_button = Button(figure.add_axes([0.24, 0.52, 0.025, 0.05]),
                                "▼")

    previous_button = Button(figure.add_axes([0.02, 0.455, 0.105, 0.05]),
                             "< Previous")
    next_button = Button(figure.add_axes([0.13, 0.455, 0.105, 0.05]),
                         "Next >")
    visibility_button = Button(figure.add_axes([0.02, 0.40, 0.105, 0.05]),
                               "Hide selected")
    show_all_button = Button(figure.add_axes([0.13, 0.40, 0.105, 0.05]),
                             "Show all")
    reset_button = Button(figure.add_axes([0.02, 0.345, 0.105, 0.05]),
                          "Reset pose")
    fit_button = Button(figure.add_axes([0.13, 0.345, 0.105, 0.05]),
                        "Fit view")
    delete_button = Button(figure.add_axes([0.02, 0.29, 0.215, 0.05]),
                           "Delete selected", color="mistyrose",
                           hovercolor="salmon")

    figure.text(0.02, 0.25, "3. Session (scans/ folder)", fontweight="bold")
    file_box = TextBox(figure.add_axes([0.06, 0.19, 0.175, 0.045]), "File ",
                       initial=file_name)
    save_button = Button(figure.add_axes([0.02, 0.13, 0.105, 0.05]),
                         "Save session")
    load_button = Button(figure.add_axes([0.13, 0.13, 0.105, 0.05]),
                         "Load session")
    status_text = figure.text(0.02, 0.115, "", va="top", fontsize=9)

    # --- Bottom: pose sliders of the SELECTED scan, then the ray opacity.
    x_slider = Slider(figure.add_axes([0.38, 0.17, 0.32, 0.03]), "x (m)",
                      -POSE_SLIDER_LIMIT_M, POSE_SLIDER_LIMIT_M, valinit=0.0,
                      valstep=0.01)
    y_slider = Slider(figure.add_axes([0.38, 0.13, 0.32, 0.03]), "y (m)",
                      -POSE_SLIDER_LIMIT_M, POSE_SLIDER_LIMIT_M, valinit=0.0,
                      valstep=0.01)
    theta_slider = Slider(figure.add_axes([0.38, 0.09, 0.32, 0.03]),
                          "theta (deg)", -180.0, 180.0, valinit=0.0,
                          valstep=0.5)
    pose_sliders = (x_slider, y_slider, theta_slider)
    opacity_slider = Slider(figure.add_axes([0.38, 0.03, 0.32, 0.03]),
                            "ray opacity", 0.0, 0.5,
                            valinit=DEFAULT_RAY_OPACITY, valstep=0.01,
                            color="0.6")

    # --- Bottom right: automatic alignment, then the display mode.
    figure.text(0.765, 0.205, "4. Auto alignment", fontweight="bold")
    auto_align_button = Button(figure.add_axes([0.765, 0.135, 0.11, 0.05]),
                               "Auto align selected")
    accept_button = Button(figure.add_axes([0.765, 0.075, 0.053, 0.05]),
                           "Accept", color="honeydew", hovercolor="palegreen")
    reject_button = Button(figure.add_axes([0.822, 0.075, 0.053, 0.05]),
                           "Reject")
    display_axes = figure.add_axes([0.885, 0.03, 0.105, 0.15])
    display_axes.set_title("Display", fontsize=9)
    display_selector = RadioButtons(
        display_axes, DISPLAY_MODES,
        active=DISPLAY_MODES.index(DEFAULT_DISPLAY_MODE))

    def show_message(text):
        """Show a message under the panel and in the console."""
        status_text.set_text(textwrap.fill(text, 42))
        print(text)
        figure.canvas.draw_idle()

    def selected_index():
        """Position of the selected scan in the list, or None."""
        if session["selected_id"] is None:
            return None
        return find_scan_index(session, session["selected_id"])

    def sync_artists():
        """Create map objects for new scans, remove those of deleted ones."""
        current_ids = {scan["id"] for scan in session["scans"]}
        for scan_id in list(artists):
            if scan_id not in current_ids:
                for artist in artists.pop(scan_id).values():
                    artist.remove()
        for scan in session["scans"]:
            if scan["id"] not in artists:
                artists[scan["id"]] = create_scan_artists(map_axes, scan)

    def alignment_label(scan):
        """Short alignment state shown in the list."""
        if scan["id"] == state["aligning"]:
            return "..."
        if scan is reference_scan(session):
            return "ref"
        return "ok" if scan["aligned"] else ""

    def update_list():
        """Rewrite the visible rows of the scan list."""
        scans = session["scans"]
        total = len(scans)
        state["list_top"] = scroll_to_show(state["list_top"], None,
                                           LIST_ROWS, total)
        top = state["list_top"]
        capturing_kind, capturing_id = state["capturing"] or (None, None)
        selection_box.set_visible(False)
        for row, text in enumerate(row_texts):
            index = top + row
            if index >= total:
                text.set_text("")
                continue
            scan = scans[index]
            is_selected = scan["id"] == session["selected_id"]
            status = ("replacing..." if capturing_kind == "replace"
                      and capturing_id == scan["id"] else None)
            text.set_text(format_scan_row(scan, is_selected, status,
                                          alignment_label(scan)))
            text.set_color(scan["color"] if scan["visible"] else "0.6")
            text.set_fontweight("bold" if is_selected else "normal")
            if is_selected:
                selection_box.set_y(1 - (row + 1) / LIST_ROWS)
                selection_box.set_visible(True)
        if total == 0:
            row_texts[0].set_text("  (no scans yet: click Add scan)")
            row_texts[0].set_color("0.4")

        title = f"2. Scans: {total} ({len(shown_scans(session))} shown)"
        if total > LIST_ROWS:
            title += f", rows {top + 1}-{min(top + LIST_ROWS, total)}"
        if capturing_kind == "add":
            title += ", capturing..."
        list_title.set_text(title)
        scan = selected_scan(session)
        visibility_button.label.set_text(
            "Show selected" if scan is not None and not scan["visible"]
            else "Hide selected")

    def update_legend_and_title():
        """Legend of the visible scans (only the selected one if many)."""
        visible = shown_scans(session)
        scan = selected_scan(session)
        if len(visible) <= LEGEND_MAX_ENTRIES:
            legend_scans = visible
        else:
            legend_scans = [scan] if scan is not None and scan["visible"] else []
        handles = [artists[item["id"]]["marker"] for item in legend_scans]
        labels = [f"{item['label']}: pose ({item['pose'][0]:.2f} m, "
                  f"{item['pose'][1]:.2f} m, {item['pose'][2]:.1f} deg)"
                  for item in legend_scans]
        if handles:
            map_axes.legend(handles, labels, loc="upper right",
                            fontsize="small", markerscale=0.8)
        elif map_axes.get_legend() is not None:
            map_axes.get_legend().remove()
        if scan is None:
            map_axes.set_title("Map - click Add scan to start")
        else:
            hidden_note = "" if scan["visible"] else " (hidden)"
            map_axes.set_title(f"Map - sliders move {scan['label']}"
                               f"{hidden_note}; numbers mark robot poses")

    def update_proposal():
        """Draw (or hide) the proposed pose and its report on the map."""
        proposal = state["proposal"]
        pose = proposal["result"]["pose"] if proposal else None
        for artist in (proposal_points, proposal_marker, proposal_arrow):
            artist.set_visible(pose is not None)
        report_text.set_visible(proposal is not None)
        if proposal is None:
            return
        scan = find_scan(session, proposal["scan_id"])
        report_text.set_text("\n".join(format_alignment_report(scan,
                                                               proposal)))
        report_text.get_bbox_patch().set_edgecolor(
            STATUS_COLORS[proposal["result"]["status"]])
        report_text.get_bbox_patch().set_linewidth(2)
        if pose is None:
            return
        local = np.column_stack([scan["x_local_m"], scan["y_local_m"]])
        proposal_points.set_offsets(transform_points(local, pose))
        heading_rad = math.radians(pose[2])
        proposal_marker.set_data([pose[0]], [pose[1]])
        proposal_arrow.set_positions(
            (pose[0], pose[1]),
            (pose[0] + POSE_ARROW_LENGTH_M * math.cos(heading_rad),
             pose[1] + POSE_ARROW_LENGTH_M * math.sin(heading_rad)))

    def redraw(scan_ids=None):
        """Update the given scans (None = all), then the list and legend."""
        sync_artists()
        for scan in session["scans"]:
            if scan_ids is None or scan["id"] in scan_ids:
                update_scan_artists(artists[scan["id"]], scan,
                                    scan["id"] == session["selected_id"],
                                    state["display_mode"],
                                    state["ray_opacity"])
        update_list()
        update_legend_and_title()
        update_proposal()
        figure.canvas.draw_idle()

    def fit_view():
        """Frame every shown scan (poses are not changed).

        The limits are widened to the shape of the map area, so that with
        equal X/Y scales the map still fills its whole area.
        """
        (x_min, x_max), (y_min, y_max) = compute_view_limits(session)
        figure_width_in, figure_height_in = figure.get_size_inches()
        area = map_axes.get_position(original=True)
        area_ratio = (area.width * figure_width_in) / (area.height
                                                        * figure_height_in)
        x_center, y_center = (x_min + x_max) / 2, (y_min + y_max) / 2
        half_width = max(x_max - x_min, (y_max - y_min) * area_ratio) / 2
        half_height = half_width / area_ratio
        map_axes.set_xlim(x_center - half_width, x_center + half_width)
        map_axes.set_ylim(y_center - half_height, y_center + half_height)
        figure.canvas.draw_idle()

    def sync_sliders():
        """Move the sliders to the selected scan's own pose."""
        scan = selected_scan(session)
        pose = scan["pose"] if scan is not None else (0.0, 0.0, 0.0)
        state["syncing"] = True
        try:
            for slider, value in zip(pose_sliders, pose):
                widen_slider_range(slider, value)
                slider.set_val(value)
        finally:
            state["syncing"] = False

    def select_scan(scan_id):
        """Select a scan: restore ITS pose on the sliders, show it in list."""
        session["selected_id"] = scan_id
        state["list_top"] = scroll_to_show(state["list_top"], selected_index(),
                                           LIST_ROWS, len(session["scans"]))
        sync_sliders()
        redraw()  # the selected pose marker is drawn larger

    def select_row(row):
        """Select the scan shown on list row `row` (0 = top row)."""
        index = state["list_top"] + row
        if 0 <= row < LIST_ROWS and index < len(session["scans"]):
            select_scan(session["scans"][index]["id"])

    def select_neighbour(step):
        """Select the previous (-1) or next (+1) scan in the list."""
        if not session["scans"]:
            return
        index = selected_index()
        index = 0 if index is None else index + step
        index = max(0, min(index, len(session["scans"]) - 1))
        select_scan(session["scans"][index]["id"])

    def scroll_list(step):
        """Scroll the list by `step` rows without changing the selection."""
        state["list_top"] += step
        update_list()
        figure.canvas.draw_idle()

    def on_list_click(event):
        """Mouse click anywhere: if it is on a list row, select that scan."""
        if event.inaxes is list_axes and event.ydata is not None:
            select_row(int((1 - event.ydata) * LIST_ROWS))

    def on_list_wheel(event):
        """Mouse wheel over the list scrolls it."""
        if event.inaxes is list_axes:
            scroll_list(-1 if event.button == "up" else 1)

    def on_pose_slider(component_index, value):
        """A slider moved: change x (0), y (1) or theta (2) of ONE scan."""
        if state["syncing"]:
            return
        scan = selected_scan(session)
        if scan is None:
            show_message("No scan selected: add a scan or load a session.")
            sync_sliders()  # put the slider back to 0
            return
        pose = list(scan["pose"])
        pose[component_index] = float(value)
        set_scan_pose(session, scan["id"], pose)
        redraw([scan["id"]])  # only this scan's points are recomputed

    def set_display_mode(mode):
        """Points only / Points + rays / Rays only, for every scan."""
        state["display_mode"] = mode
        redraw()

    def set_ray_opacity(opacity):
        """Change the transparency of every ray (0 = invisible, 1 = solid)."""
        state["ray_opacity"] = float(opacity)
        for scan_artists in artists.values():
            scan_artists["rays"].set_alpha(state["ray_opacity"])
        figure.canvas.draw_idle()

    def toggle_selected_visibility():
        scan = selected_scan(session)
        if scan is None:
            show_message("No scan selected.")
            return
        set_scan_visible(session, scan["id"], not scan["visible"])
        redraw([scan["id"]])

    def show_all_scans():
        for scan in session["scans"]:
            set_scan_visible(session, scan["id"], True)
        redraw()

    def reset_selected_pose():
        scan = selected_scan(session)
        if scan is None:
            show_message("No scan selected: nothing to reset.")
            return
        reset_scan_pose(session, scan["id"])
        sync_sliders()
        redraw([scan["id"]])
        show_message(f"{scan['label']} pose reset to (0, 0, 0).")

    def clear_proposal(scan_id=None):
        """Remove the proposal (only if it concerns `scan_id`, when given)."""
        proposal = state["proposal"]
        if proposal is not None and scan_id in (None, proposal["scan_id"]):
            state["proposal"] = None

    def request_capture(kind):
        """Ask the worker thread for one scan ("add" or "replace").

        Returns immediately: the result arrives later in poll_worker().
        """
        scan = selected_scan(session)
        if kind == "replace" and scan is None:
            show_message("Select the scan to replace first.")
        elif kind == "replace" and state["aligning"] is not None:
            show_message("Wait for the automatic alignment to finish first.")
        elif worker is None:
            show_message("Offline: no ESP32 connection. Plug it in and "
                         "restart with RUN_MODE = 'serial_capture'. You can "
                         "still load a saved session.")
        elif not worker["thread"].is_alive():
            show_message("The serial connection was lost. Save your work, "
                         "reconnect the ESP32 and restart the program.")
        elif state["capturing"] is not None:
            show_message("Still capturing the previous scan, please wait.")
        else:
            request = ("replace", scan["id"]) if kind == "replace" \
                else ("add", None)
            state["capturing"] = request
            worker["requests"].put(request)
            update_list()
            target = (f"new measurements for {scan['label']}"
                      if kind == "replace" else "a new scan")
            show_message(f"Capturing {target}: keep the robot still...")

    def poll_worker():
        """Handle finished captures (called by the 100 ms timer)."""
        while worker is not None:
            try:
                request, scans, report = worker["results"].get_nowait()
            except queue.Empty:
                return
            state["capturing"] = None
            print_capture_report(scans, report, 1)
            if not scans:
                redraw(())
                show_message(f"Capture failed ({report['stop_reason']}). "
                             f"Nothing was changed.")
                continue
            x_m, y_m = teleplot_to_sensor_frame(*scans[0])
            kind, scan_id = request
            if kind == "replace":
                # Delete, Load and alignment are refused during a capture,
                # so the scan to replace still exists.
                scan = replace_scan_points(session, scan_id, x_m, y_m)
                clear_proposal(scan_id)  # it was computed for OLD points
                session["selected_id"] = scan_id
                message = (f"{scan['label']} replaced: {len(x_m)} points, "
                           f"pose kept. Other scans unchanged.")
            else:
                scan = add_scan(session, x_m, y_m)  # also selects it
                message = (f"{scan['label']} added: {len(x_m)} points at "
                           f"pose (0, 0, 0). Move the sliders or click Auto "
                           f"align selected.")
            select_scan(scan["id"])
            fit_view()
            show_message(f"{message} [{FORMAT_LABELS[report['format']]}]")

    def auto_align_selected():
        """Start aligning the selected scan in a background thread."""
        scan = selected_scan(session)
        if scan is None:
            show_message("Select the scan to align first.")
            return
        if state["aligning"] is not None:
            show_message(f"Still aligning Scan {state['aligning']}, please "
                         f"wait.")
            return
        if state["capturing"] is not None:
            show_message("Wait for the current capture to finish first.")
            return
        if scan is reference_scan(session):
            show_message(f"{scan['label']} is the fixed reference of the map "
                         f"(first scan): it is never moved automatically. "
                         f"Select another scan.")
            return
        references = alignment_reference_scans(session, scan["id"])
        if not references:
            show_message("No reference to align against: the reference scan "
                         "is hidden and no other scan is aligned yet.")
            return
        # COPIES: the session may change while the thread computes.
        source = np.column_stack([scan["x_local_m"], scan["y_local_m"]])
        reference_data = [(np.column_stack([item["x_local_m"],
                                            item["y_local_m"]]),
                           tuple(item["pose"])) for item in references]
        reference_ids = [item["id"] for item in references]
        state["aligning"] = scan["id"]
        clear_proposal()
        start_alignment_thread(source, reference_data, alignment_results,
                               (scan["id"], reference_ids))
        redraw(())
        show_message(f"Aligning {scan['label']} against Scan(s) "
                     f"{', '.join(map(str, reference_ids))} (all headings "
                     f"0-360 deg; a few seconds). The window stays usable.")

    def poll_alignment():
        """Show a finished alignment as a PROPOSAL (called by the timer)."""
        while True:
            try:
                (scan_id, reference_ids), result = \
                    alignment_results.get_nowait()
            except queue.Empty:
                return
            state["aligning"] = None
            state["proposal"] = {"scan_id": scan_id, "result": result,
                                 "reference_ids": reference_ids}
            redraw(())
            label = find_scan(session, scan_id)["label"]
            if result["status"] == "failed":
                show_message(f"{label}: no reliable alignment. Pose "
                             f"unchanged. See the report on the map.")
            else:
                show_message(f"{label}: proposal shown in black "
                             f"({STATUS_TITLES[result['status']].lower()}). "
                             f"Compare it with the reference, then Accept "
                             f"or Reject.")

    def poll_background():
        """Timer callback (every 100 ms, GUI thread): collect results."""
        poll_worker()
        poll_alignment()

    def accept_proposal():
        proposal = state["proposal"]
        if proposal is None:
            show_message("No proposal: click Auto align selected first.")
            return
        result = proposal["result"]
        scan = find_scan(session, proposal["scan_id"])
        if result["status"] == "failed":
            show_message(f"Cannot accept: {result['reason']} {scan['label']} "
                         f"keeps its pose.")
            return
        runner_up = result["runner_up"]
        accept_alignment(session, scan["id"], result["pose"], {
            "status": result["status"],
            "match_fraction": round(result["match_fraction"], 4),
            "rms_m": round(result["rms_m"], 5),
            "conflict_fraction": round(result["conflict"], 4),
            "scan_sees_through_fraction": round(result["scan_sees_through"], 4),
            "reference_sees_through_fraction": round(
                result["reference_sees_through"], 4),
            "score": round(result["score"], 4),
            "runner_up_score": (round(runner_up["score"], 4)
                                if runner_up else None),
            "reference_scans": ", ".join(map(str,
                                             proposal["reference_ids"]))})
        state["proposal"] = None
        if session["selected_id"] == scan["id"]:
            sync_sliders()
        redraw([scan["id"]])
        show_message(f"{scan['label']} aligned (marked ok): it can now be "
                     f"part of the reference for other scans. The sliders "
                     f"still allow corrections.")

    def reject_proposal():
        if state["proposal"] is None:
            show_message("No proposal to reject.")
            return
        label = find_scan(session, state["proposal"]["scan_id"])["label"]
        state["proposal"] = None
        redraw(())
        show_message(f"Proposal rejected: {label} keeps its pose.")

    def delete_selected():
        scan = selected_scan(session)
        if scan is None:
            show_message("No scan selected: nothing to delete.")
            return
        if state["capturing"] is not None or state["aligning"] is not None:
            show_message("Wait for the current capture or alignment to "
                         "finish first.")
            return
        if state["confirm_delete"] != scan["id"]:
            state["confirm_delete"] = scan["id"]  # ask once before deleting
            show_message(f"Delete {scan['label']} "
                         f"({len(scan['x_local_m'])} points)? Click Delete "
                         f"selected again to confirm.")
            return
        state["confirm_delete"] = None
        delete_scan(session, scan["id"])
        clear_proposal(scan["id"])
        select_scan(session["selected_id"])
        show_message(f"{scan['label']} deleted. Numbers are not reused: the "
                     f"next new scan will be Scan {session['next_id']}.")

    def save_current_session():
        path = session_path_from_name(file_box.text)
        if not session["scans"]:
            show_message("Nothing to save: add or load a scan first.")
            return
        if path.exists() and state["confirm_overwrite"] != path:
            state["confirm_overwrite"] = path  # ask once before overwriting
            show_message(f"{path.name} already exists. Click Save session "
                         f"again to overwrite it, or change the file name.")
            return
        state["confirm_overwrite"] = None
        try:
            save_session(session, path)
        except OSError as error:
            show_message(f"Could not save {path}: {error}")
            return
        show_message(f"Saved {len(session['scans'])} scan(s) to {path}")

    def load_session_file():
        if state["capturing"] is not None or state["aligning"] is not None:
            show_message("Wait for the current capture or alignment to "
                         "finish first.")
            return
        path = session_path_from_name(file_box.text)
        try:
            loaded, version = load_session(path)  # validated BEFORE changes
        except SessionFileError as error:
            show_message(f"Could not load: {error}. The current map was "
                         f"not changed.")
            return
        for scan_artists in artists.values():  # start the drawing afresh
            for artist in scan_artists.values():
                artist.remove()
        artists.clear()
        replace_session_contents(session, loaded)
        state.update(confirm_overwrite=None, confirm_delete=None, list_top=0,
                     proposal=None)
        select_scan(session["selected_id"])
        fit_view()
        message = f"Loaded {path.name}: {len(session['scans'])} scan(s)."
        if version == 1:
            message += (" Old A/B/C file converted: A, B, C are now Scans "
                        "1, 2, 3. Saving writes the new format; use a new "
                        "file name to keep the old file as it is.")
        show_message(message)

    add_button.on_clicked(lambda event: request_capture("add"))
    replace_button.on_clicked(lambda event: request_capture("replace"))
    scroll_up_button.on_clicked(lambda event: scroll_list(-LIST_ROWS // 2))
    scroll_down_button.on_clicked(lambda event: scroll_list(LIST_ROWS // 2))
    previous_button.on_clicked(lambda event: select_neighbour(-1))
    next_button.on_clicked(lambda event: select_neighbour(+1))
    visibility_button.on_clicked(lambda event: toggle_selected_visibility())
    show_all_button.on_clicked(lambda event: show_all_scans())
    reset_button.on_clicked(lambda event: reset_selected_pose())
    fit_button.on_clicked(lambda event: fit_view())
    delete_button.on_clicked(lambda event: delete_selected())
    save_button.on_clicked(lambda event: save_current_session())
    load_button.on_clicked(lambda event: load_session_file())
    auto_align_button.on_clicked(lambda event: auto_align_selected())
    accept_button.on_clicked(lambda event: accept_proposal())
    reject_button.on_clicked(lambda event: reject_proposal())
    display_selector.on_clicked(set_display_mode)
    opacity_slider.on_changed(set_ray_opacity)
    for index, slider in enumerate(pose_sliders):
        slider.on_changed(lambda value, index=index: on_pose_slider(index,
                                                                    value))
    figure.canvas.mpl_connect("button_press_event", on_list_click)
    figure.canvas.mpl_connect("scroll_event", on_list_wheel)

    # The timer runs in the GUI thread: it is the only place where results
    # of the serial and alignment threads touch the session and the drawing.
    timer = figure.canvas.new_timer(interval=100)
    timer.add_callback(poll_background)
    timer.start()
    figure.canvas.mpl_connect("close_event", lambda _event: timer.stop())

    select_scan(session["selected_id"])
    fit_view()
    show_message(start_message or "Click a scan in the list, then move the "
                                  "sliders.")

    controls = {
        "axes": map_axes,
        "sliders": pose_sliders, "opacity_slider": opacity_slider,
        "display_selector": display_selector, "file_box": file_box,
        "timer": timer,
        "buttons": (add_button, replace_button, scroll_up_button,
                    scroll_down_button, previous_button, next_button,
                    visibility_button, show_all_button, reset_button,
                    fit_button, delete_button, save_button, load_button,
                    auto_align_button, accept_button, reject_button),
        "artists": artists, "row_texts": row_texts, "list_title": list_title,
        "status_text": status_text, "visibility_button": visibility_button,
        "report_text": report_text, "proposal_points": proposal_points,
        "is_aligning": lambda: state["aligning"] is not None,
        "proposal": lambda: state["proposal"],
        "actions": {
            "add": lambda: request_capture("add"),
            "replace": lambda: request_capture("replace"),
            "poll": poll_background, "select": select_scan,
            "select_row": select_row, "next": lambda: select_neighbour(+1),
            "previous": lambda: select_neighbour(-1), "scroll": scroll_list,
            "toggle_visibility": toggle_selected_visibility,
            "show_all": show_all_scans, "reset_pose": reset_selected_pose,
            "delete": delete_selected, "fit_view": fit_view,
            "save": save_current_session, "load": load_session_file,
            "auto_align": auto_align_selected, "accept": accept_proposal,
            "reject": reject_proposal, "display_mode": set_display_mode,
            "ray_opacity": set_ray_opacity},
    }
    return figure, controls


def run_serial_capture(port, baudrate):
    """Manual capture: connect (read-only), then open the mapping window.

    If the port cannot be opened, the window still opens OFFLINE so a saved
    session can be loaded and aligned.
    """
    print(f"=== Manual capture on {port} at {baudrate} baud "
          f"(read-only: nothing is sent) ===")
    worker = None
    try:
        connection = open_serial_readonly(port, baudrate)
    except serial.SerialException as error:
        print(f"Could not open {port}: {error}")
        message = (f"{port} is not available (ESP32 unplugged, wrong port, "
                   f"or used by Arduino Serial Monitor / Teleplot). Working "
                   f"OFFLINE: you can still load a saved session.")
    else:
        worker = start_serial_worker(connection)
        message = (f"Connected to {port}. Put the robot at its first "
                   f"position, keep it still, then click Add scan.")
    editor = create_session_editor(create_session(), worker, message)
    try:
        plt.show()  # the program waits here until the window is closed
    finally:
        if worker is not None:
            stop_serial_worker(worker)


def run_session_viewer(file_name):
    """Open a saved session OFFLINE (the ESP32 is not needed at all)."""
    session = create_session()
    path = session_path_from_name(file_name)
    try:
        loaded, version = load_session(path)
        replace_session_contents(session, loaded)
        message = (f"Loaded {path.name} offline: {len(session['scans'])} "
                   f"scan(s).")
        if version == 1:
            message += " Old A/B/C file: A, B, C are now Scans 1, 2, 3."
    except SessionFileError as error:
        message = (f"Could not load: {error}. Type another file name, then "
                   f"click Load session.")
    editor = create_session_editor(session, None, message, file_name)
    plt.show()


# ---------------------------------------------------------------------------
# Alignment benchmark (RUN_MODE = "alignment_benchmark")
# ---------------------------------------------------------------------------
# (name, walls, reference pose, TRUE source pose, options, expected status)
# "ok" = must find the true pose (good or weak); "failed" = must refuse.
REFERENCE_TEST_POSE = (2.0, 2.5, 30.0)
ALIGNMENT_BENCHMARK_CASES = (
    ("rotation 0 deg", CLASSROOM_WALLS, REFERENCE_TEST_POSE,
     (4.5, 2.5, 0.0), {}, "ok"),
    ("rotation 45 deg", CLASSROOM_WALLS, REFERENCE_TEST_POSE,
     (6.0, 3.5, 45.0), {}, "ok"),
    ("rotation 90 deg", CLASSROOM_WALLS, REFERENCE_TEST_POSE,
     (3.0, 4.5, 90.0), {}, "ok"),
    ("rotation 180 deg", CLASSROOM_WALLS, REFERENCE_TEST_POSE,
     (5.5, 1.5, 180.0), {}, "ok"),
    ("rotation -135 deg", CLASSROOM_WALLS, REFERENCE_TEST_POSE,
     (1.0, 5.5, -135.0), {}, "ok"),
    ("rotation 17.3 deg", CLASSROOM_WALLS, REFERENCE_TEST_POSE,
     (7.0, 2.5, 17.3), {}, "ok"),
    ("rotation -73.6 deg", CLASSROOM_WALLS, REFERENCE_TEST_POSE,
     (3.2, 0.8, -73.6), {}, "ok"),
    ("noise 25 mm + 15% outliers", CLASSROOM_WALLS, REFERENCE_TEST_POSE,
     (5.0, 3.8, 123.0), {"noise_mm": 25.0, "outlier_fraction": 0.15}, "ok"),
    ("partial overlap", CLASSROOM_WALLS, (1.2, 1.2, 0.0),
     (2.0, 6.2, -100.0), {}, "ok"),
    ("symmetric empty room", SYMMETRIC_ROOM_WALLS, (1.5, 1.0, 0.0),
     (4.0, 3.0, 120.0), {}, "failed"),
    ("no overlap (other room)", CLASSROOM_WALLS, REFERENCE_TEST_POSE,
     (0.5, 0.3, 10.0), {"source_walls": OCTAGON_ROOM_WALLS}, "failed"),
)


def pose_errors(estimated_pose, true_pose):
    """Return (translation error in m, rotation error in deg)."""
    return (math.hypot(estimated_pose[0] - true_pose[0],
                       estimated_pose[1] - true_pose[1]),
            angle_difference_deg(estimated_pose[2], true_pose[2]))


def run_alignment_benchmark():
    """Print errors, runtimes and verdicts; saved sessions are only READ."""
    print("=== Automatic alignment: simulated rooms with a known answer ===")
    print(f"{'case':28s} {'expected':8s} {'verdict':7s} {'err xy':>8s} "
          f"{'err th':>7s} {'match':>6s} {'conflict':>8s} {'time':>6s}")
    for name, walls, reference_pose, true_pose, options, expected in \
            ALIGNMENT_BENCHMARK_CASES:
        source, references = make_alignment_case(walls, reference_pose,
                                                 true_pose, **options)
        result = align_scan(source, references)
        if result["pose"] is None:
            errors_text = f"{'-':>8s} {'-':>7s}"
        else:
            error_m, error_deg = pose_errors(result["pose"], true_pose)
            errors_text = f"{error_m * 100:6.1f}cm {error_deg:5.2f}deg"
        print(f"{name:28s} {expected:8s} {result['status']:7s} {errors_text} "
              f"{result['match_fraction']:6.0%} {result['conflict']:8.0%} "
              f"{result['runtime_s']:5.1f}s")
        if result["status"] == "failed":
            print(f"    reason: {result['reason']}")

    print("\n=== Saved sessions in scans/ (read only, never modified) ===")
    for path in sorted(SCANS_DIR.glob("*.json")):
        try:
            session, _ = load_session(path)
        except SessionFileError as error:
            print(f"{path.name}: skipped ({error})")
            continue
        print(f"{path.name}: {len(session['scans'])} scan(s), reference "
              f"{reference_scan(session)['label'] if session['scans'] else '-'}")
        for scan in session["scans"][1:]:
            references = alignment_reference_scans(session, scan["id"])
            result = align_scan(
                np.column_stack([scan["x_local_m"], scan["y_local_m"]]),
                [(np.column_stack([item["x_local_m"], item["y_local_m"]]),
                  item["pose"]) for item in references])
            pose_text = ("-" if result["pose"] is None else
                         "({:.2f} m, {:.2f} m, {:.1f} deg)".format(
                             *result["pose"]))
            print(f"  {scan['label']} vs Scan(s) "
                  f"{', '.join(str(item['id']) for item in references)}: "
                  f"{result['status']} {pose_text} "
                  f"in {result['runtime_s']:.1f} s; saved pose "
                  f"{tuple(round(value, 2) for value in scan['pose'])}")
            print(f"    {result['reason']}")


# Example packet printed in the manufacturer's manual (section 3.3).
MANUFACTURER_EXAMPLE_PACKET = bytes.fromhex(
    "54 2C 68 08 AB 7E E0 00 E4 DC 00 E2 D9 00 E5 D5 00 E3 D3 00 E4 D0 00 E9"
    " CD 00 E4 CA 00 E2 C7 00 E9 C5 00 E5 C2 00 E5 C0 00 E5 BE 82 3A 1A 50")


def demonstrate_packet_decoder():
    """Decode the manufacturer's example packet and a messy byte stream."""
    packet = MANUFACTURER_EXAMPLE_PACKET
    print("=== LD06 decoder demo (offline, manufacturer example packet) ===")
    print(f"CRC computed 0x{ld06_crc8(packet[:-1]):02X}, received "
          f"0x{packet[-1]:02X} -> valid: {is_valid_ld06_packet(packet)}")
    decoded = decode_ld06_packet(packet)
    print(f"speed {decoded['speed_deg_per_s']} deg/s, "
          f"start {decoded['start_angle_deg']:.2f} deg, "
          f"end {decoded['end_angle_deg']:.2f} deg, "
          f"timestamp {decoded['timestamp_ms']} ms")
    print("  i | angle CW (deg) | distance (mm) | intensity")
    for i in range(LD06_POINTS_PER_PACKET):
        print(f" {i:2d} | {decoded['angles_deg'][i]:14.2f} |"
              f" {decoded['distances_mm'][i]:13d} |"
              f" {decoded['intensities'][i]:9d}")

    # A messy stream, as a serial port could deliver it: noise (including a
    # lone 0x54), the packet, a corrupted copy, then the packet again.
    corrupted = bytearray(packet)
    corrupted[10] ^= 0xFF  # flip the 8 bits of one distance byte
    stream = bytes([0x00, 0x54, 0x13]) + packet + bytes(corrupted) + packet
    print(f"\nByte stream of {len(stream)} bytes, read in 3 arbitrary chunks:")
    buffer = bytearray()
    for chunk in (stream[:20], stream[20:70], stream[70:]):
        buffer.extend(chunk)  # what a serial read would append
        packets, crc_failures = extract_ld06_packets(buffer)
        print(f"  received {len(chunk):3d} bytes -> {len(packets)} valid "
              f"packet(s), {crc_failures} CRC failure(s), "
              f"{len(buffer)} byte(s) waiting")
    print()


def main():
    if RUN_MODE == "serial_diagnostic":
        run_serial_diagnostic(SERIAL_PORT, CANDIDATE_BAUD_RATES,
                              DIAGNOSTIC_SECONDS_PER_BAUD)
        return
    if RUN_MODE == "serial_capture":
        run_serial_capture(SERIAL_PORT, CAPTURE_BAUD_RATE)
        return
    if RUN_MODE == "session_viewer":
        run_session_viewer(SESSION_FILE_TO_OPEN)
        return
    if RUN_MODE == "alignment_benchmark":
        run_alignment_benchmark()
        return

    demonstrate_packet_decoder()

    # Three scans of the SAME room, simulated from their TRUE poses
    # (x, y, theta). Theta is kept within [-180, 180] to match the slider.
    true_poses = {
        "Scan A": (0.0, 0.0, 0.0),
        "Scan B": (2.5, 1.0, 90.0),
        "Scan C": (1.5, -0.8, -135.0),
    }
    scans = [
        create_simulated_scan("Scan A", "tab:blue", true_poses["Scan A"]),
        create_simulated_scan("Scan B", "tab:orange", true_poses["Scan B"]),
        create_simulated_scan("Scan C", "tab:green", true_poses["Scan C"]),
    ]

    for scan in scans:
        print(f"=== {scan['label']}, true pose {scan['pose']} ===")
        print_scan_summary(scan["angles_deg"], scan["distances_mm"],
                           scan["intensities"])
        print()

    # Like a real robot, start with imperfect pose ESTIMATES for B and C.
    # Use the sliders to align them with Scan A (true poses printed above).
    scans[1]["pose"] = (2.3, 1.2, 80.0)
    scans[2]["pose"] = (1.7, -0.6, -120.0)

    editor = create_pose_editor(  # keep a reference so the widgets work
        scans, "Simulated LD06 scans - select a scan, then move the sliders",
        x_range=(ROOM_X_MIN - 1.0, ROOM_X_MAX + 1.0),
        y_range=(ROOM_Y_MIN - 1.0, ROOM_Y_MAX + 1.0))
    plt.show()  # opens the window; the program waits until it is closed


# ---------------------------------------------------------------------------
# Automated tests (no ESP32 needed): python -m unittest 01_lidar_visualizer -v
# ---------------------------------------------------------------------------
def make_teleplot_line(points, name=TELEPLOT_SCAN_NAME):
    """Build a line exactly like ld06.printScanTeleplot() (with "\\r\\n")."""
    body = "".join(f"{x}:{y};" for x, y in points)
    return f">{name}:{body}|xy\r\n".encode("ascii")


def make_test_points(count, offset_mm=0):
    """Return `count` distinct signed integer (x, y) points, all non-zero."""
    return [(1000 + offset_mm + i, -500 + 7 * i) for i in range(count)]


class FakeSerial:
    """Stands in for a serial port: read() returns the given chunks in turn.

    After the last chunk it returns b"" (a read timeout), or raises
    serial.SerialException if `disconnect_at_end` is True.
    """

    def __init__(self, chunks, disconnect_at_end=False):
        self.chunks = list(chunks)
        self.disconnect_at_end = disconnect_at_end

    def read(self, size):
        if self.chunks:
            return self.chunks.pop(0)
        if self.disconnect_at_end:
            raise serial.SerialException("device disconnected (simulated)")
        return b""

    def reset_input_buffer(self):
        pass  # the chunks stand for bytes that arrive AFTER the click

    def close(self):
        pass


class FakeClock:
    """A clock that moves forward 0.1 s each time it is read."""

    def __init__(self):
        self.now_s = 0.0

    def __call__(self):
        self.now_s += 0.1
        return self.now_s


def chop(data, size):
    """Cut bytes into chunks of `size` bytes, like arbitrary serial reads."""
    return [data[i:i + size] for i in range(0, len(data), size)]


# Two rows exactly as printed by the robot's ESP32 (ld06.printScanCSV).
REAL_CSV_ROWS = ("404,40.08,775,229,592,-499", "405,39.36,775,232,599,-491")
CSV_HEADER = "N,Angle(°),Distance(mm),Intensity,x(mm),y(mm)"


def make_csv_scan(points, with_xy=True, first_index=0):
    """Build the bytes ld06.printScanCSV() sends for one scan.

    points - (x_mm, y_mm) library-frame integers; angle and distance are
    derived with the ld06.cpp convention (y = -distance * sin(angle)).
    """
    rows = []
    for number, (x, y) in enumerate(points, start=first_index):
        distance = round(math.hypot(x, y))
        angle = math.degrees(math.atan2(-y, x)) % 360.0
        row = f"{number},{angle:.2f},{distance},200"
        rows.append(row + (f",{x},{y}" if with_xy else ""))
    return ("\r\n".join(rows) + "\r\n\r\n").encode("utf-8")


def recorded_scan_as_library_points(scan_index=0):
    """A REAL recorded LD06 scan, back in the library frame (int mm)."""
    with open(SCANS_DIR / "classroom_final.json", encoding="utf-8") as handle:
        local_m = np.array(json.load(handle)["scans"][scan_index]["points_local_m"])
    points = [(int(round(x * 1000)), int(round(-y * 1000))) for x, y in local_m]
    return local_m, points


class TeleplotParserTests(unittest.TestCase):
    def test_parses_signed_points(self):
        parsed = parse_teleplot_xy_line(">lidar:1320:16;-250:-3;0:-1200;|xy")
        x_mm, y_mm, malformed = parsed
        np.testing.assert_array_equal(x_mm, [1320, -250, 0])
        np.testing.assert_array_equal(y_mm, [16, -3, -1200])
        self.assertEqual(malformed, 0)

    def test_skips_malformed_entries_and_keeps_good_ones(self):
        line = ">lidar:10:20;abc:5;7;1:2:3;nan:4;12:;30:-40;|xy"
        x_mm, y_mm, malformed = parse_teleplot_xy_line(line)
        np.testing.assert_array_equal(x_mm, [10, 30])
        np.testing.assert_array_equal(y_mm, [20, -40])
        self.assertEqual(malformed, 5)

    def test_rejects_incomplete_or_other_lines(self):
        for line in ("", ">lidar:1:2;3:4;",            # cut before "|xy"
                     "dar:1:2;3:4;|xy",                # cut at the start
                     ">lidar2:1:2;|xy", ">speed:42",   # other variables
                     "rst:0x1 (POWERON_RESET)", ">lidar:1:2;|"):
            self.assertIsNone(parse_teleplot_xy_line(line), line)

    def test_accepts_carriage_return_and_extra_flags(self):
        x_mm, _, _ = parse_teleplot_xy_line(">lidar:5:6;|xy,np\r")
        np.testing.assert_array_equal(x_mm, [5])

    def test_line_split_across_reads_is_joined(self):
        line = make_teleplot_line([(1, 2), (-3, 4)])
        buffer = bytearray()
        found = []
        for byte_index in range(len(line)):  # worst case: 1 byte per read
            buffer.extend(line[byte_index:byte_index + 1])
            found += split_complete_lines(buffer)
        self.assertEqual(found, [">lidar:1:2;-3:4;|xy"])
        self.assertEqual(len(buffer), 0)

    def test_overlong_line_without_ending_is_dropped(self):
        buffer = bytearray(b"9" * (TELEPLOT_MAX_LINE_BYTES + 1))
        self.assertEqual(split_complete_lines(buffer), [])
        self.assertEqual(len(buffer), 0)


class CoordinateTests(unittest.TestCase):
    def test_millimetres_to_metres_and_y_sign(self):
        x_m, y_m = teleplot_to_sensor_frame(np.array([1000.0, 0.0, -250.0]),
                                            np.array([500.0, 0.0, -250.0]))
        np.testing.assert_allclose(x_m, [1.0, -0.25])  # (0, 0) is dropped
        np.testing.assert_allclose(y_m, [-0.5, 0.25])  # library +Y = right

    def test_y_sign_can_be_kept(self):
        _, y_m = teleplot_to_sensor_frame(np.array([1.0]), np.array([500.0]),
                                          y_points_right=False)
        np.testing.assert_allclose(y_m, [0.5])

    def test_library_formula_matches_polar_to_cartesian(self):
        # Rebuild the Teleplot line the library would send for a simulated
        # scan, using the ld06.cpp formulas (C truncates float -> int16).
        angles_cw, distances_mm, _ = simulate_ld06_scan((0.0, 0.0, 0.0))
        angle_ccw_rad = np.radians((360.0 - angles_cw) % 360.0)
        x_lib = np.trunc(distances_mm * np.cos(angle_ccw_rad)).astype(int)
        y_lib = np.trunc(-distances_mm * np.sin(angle_ccw_rad)).astype(int)
        line = make_teleplot_line(zip(x_lib, y_lib)).decode("ascii")

        x_mm, y_mm, malformed = parse_teleplot_xy_line(line)
        x_m, y_m = teleplot_to_sensor_frame(x_mm, y_mm)
        x_expected, y_expected = polar_to_cartesian(angles_cw, distances_mm)
        self.assertEqual(malformed, 0)
        np.testing.assert_allclose(x_m, x_expected, atol=0.0011)  # 1 mm
        np.testing.assert_allclose(y_m, y_expected, atol=0.0011)

    def test_cartesian_scan_is_not_converted_again(self):
        x_local = np.array([1.0, 0.0])
        y_local = np.array([0.0, 2.0])
        scan = create_cartesian_scan("s", "tab:blue", (0.0, 0.0, 0.0),
                                     x_local, y_local)
        x_global, y_global = scan_to_global(scan)
        np.testing.assert_allclose(x_global, x_local)
        np.testing.assert_allclose(y_global, y_local)

        scan["pose"] = (1.0, 2.0, 90.0)  # what the sliders do
        x_global, y_global = scan_to_global(scan)
        np.testing.assert_allclose(x_global, [1.0, -1.0], atol=1e-12)
        np.testing.assert_allclose(y_global, [3.0, 2.0], atol=1e-12)
        np.testing.assert_array_equal(scan["x_local_m"], [1.0, 0.0])


class CaptureTests(unittest.TestCase):
    def test_captures_three_scans_split_across_reads(self):
        stream = (b"123:45;678:-9;|xy\r\n"            # port opened mid-line
                  + b">speed:3600\r\n"                  # another variable
                  + b"".join(make_teleplot_line(make_test_points(60, k))
                             for k in range(4)))
        connection = FakeSerial(chop(stream, 7))  # about 390 tiny reads
        scans, report = capture_teleplot_scans(connection, 3, 1000.0,
                                               clock=FakeClock())
        self.assertEqual(report["stop_reason"], "complete")
        self.assertEqual(len(scans), 3)
        x_mm, y_mm = scans[2]
        self.assertEqual(len(x_mm), 60)
        self.assertEqual((x_mm[0], y_mm[0]), (1002, -500))  # offset_mm = 2

    def test_timeout_returns_the_scans_received(self):
        connection = FakeSerial([make_teleplot_line(make_test_points(60))])
        scans, report = capture_teleplot_scans(connection, 3, 2.0,
                                               clock=FakeClock())
        self.assertEqual(len(scans), 1)
        self.assertEqual(report["stop_reason"], "timeout")

    def test_nothing_received(self):
        scans, report = capture_teleplot_scans(FakeSerial([]), 3, 1.0,
                                               clock=FakeClock())
        self.assertEqual((scans, report["bytes"]), ([], 0))
        self.assertEqual(report["stop_reason"], "timeout")

    def test_disconnection_returns_the_scans_received(self):
        two_scans = (make_teleplot_line(make_test_points(60))
                     + make_teleplot_line(make_test_points(60, 1)))
        connection = FakeSerial(chop(two_scans + b">lidar:1:2;3", 100),
                                disconnect_at_end=True)
        scans, report = capture_teleplot_scans(connection, 3, 10.0,
                                               clock=FakeClock())
        self.assertEqual(len(scans), 2)
        self.assertTrue(report["stop_reason"].startswith("disconnected"))

    def test_rejects_short_corrupted_and_empty_scans(self):
        corrupted = make_teleplot_line(make_test_points(60)).replace(
            b";1", b";x", 10)                             # 10 bad entries
        stream = (make_teleplot_line(make_test_points(5))  # too few points
                  + corrupted
                  + make_teleplot_line([(0, 0)] * 100)     # no echo at all
                  + make_teleplot_line(make_test_points(60)))
        scans, report = capture_teleplot_scans(FakeSerial([stream]), 3, 1.0,
                                               clock=FakeClock())
        self.assertEqual(len(scans), 1)
        self.assertEqual(report["rejected_scans"], 3)


class CsvRowTests(unittest.TestCase):
    def test_real_six_column_rows(self):
        self.assertEqual(parse_ld06_csv_row(REAL_CSV_ROWS[0]),
                         (FORMAT_CSV_XY, 404, 592.0, -499.0))
        self.assertEqual(parse_ld06_csv_row(REAL_CSV_ROWS[1] + "\r"),
                         (FORMAT_CSV_XY, 405, 599.0, -491.0))

    def test_real_rows_are_not_teleplot_lines(self):
        for row in REAL_CSV_ROWS:
            self.assertIsNone(parse_teleplot_xy_line(row))

    def test_four_columns_use_the_library_formula(self):
        # The same real rows without x/y must give the x/y the ESP32 sent
        # (the library stores them as int16, so up to 1 mm truncation).
        for row in REAL_CSV_ROWS:
            _, index6, x6, y6 = parse_ld06_csv_row(row)
            fmt, index4, x4, y4 = parse_ld06_csv_row(row.rsplit(",", 2)[0])
            self.assertEqual((fmt, index4), (FORMAT_CSV_POLAR, index6))
            self.assertAlmostEqual(x4, x6, delta=1.0)
            self.assertAlmostEqual(y4, y6, delta=1.0)

    def test_orientation_in_sensor_frame(self):
        # CSV angles are already counter-clockwise: 0 = front, 90 = left.
        for row, expected in (("0,0.00,1000,200", (1.0, 0.0)),
                              ("1,90.00,1000,200", (0.0, 1.0)),
                              ("2,270.00,1000,200", (0.0, -1.0)),
                              ("3,90.00,1000,200,0,-1000", (0.0, 1.0)),
                              (REAL_CSV_ROWS[0], (0.592, 0.499))):
            _, _, x_mm, y_mm = parse_ld06_csv_row(row)
            x_m, y_m = teleplot_to_sensor_frame(np.array([x_mm]),
                                                np.array([y_mm]))
            np.testing.assert_allclose([x_m[0], y_m[0]], expected,
                                       atol=1e-9, err_msg=row)

    def test_invalid_rows_are_rejected(self):
        for row in ("", CSV_HEADER, "1,2,3", "1,2,3,4,5", "1,2,3,4,5,6,7",
                    "a,1.0,100,5", "1,nan,100,5", "1,10,100,5,inf,2",
                    "-1,10,100,5", "1.5,10,100,5", "1,400,100,5",
                    "1,10,-5,5", "1,10,70000,5", "1,10,,5", "40,4�.08,775,229",
                    "rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)"):
            self.assertIsNone(parse_ld06_csv_row(row), row)


class CsvCaptureTests(unittest.TestCase):
    def capture(self, stream, count=1, chunk=None, **options):
        chunks = chop(stream, chunk) if chunk else [stream]
        return capture_teleplot_scans(FakeSerial(chunks), count, 1000.0,
                                      clock=FakeClock(), **options)

    def test_blank_line_ends_scan_and_partial_first_scan_is_discarded(self):
        partial = make_csv_scan(make_test_points(60, 50), first_index=300)
        stream = (b"5,229,592,-499\r\n"[3:]         # port opened mid-row
                  + partial
                  + make_csv_scan(make_test_points(60))
                  + make_csv_scan(make_test_points(70, 1)))
        scans, report = self.capture(stream, count=2, chunk=7)
        self.assertEqual(report["stop_reason"], "complete")
        self.assertEqual(report["format"], FORMAT_CSV_XY)
        self.assertEqual([len(x) for x, _ in scans], [60, 70])
        self.assertEqual((scans[0][0][0], scans[0][1][0]), (1000, -500))
        self.assertEqual(report["rejected_scans"], 0)

    def test_header_after_boot_starts_a_scan(self):
        stream = (CSV_HEADER + "\r\n").encode() + make_csv_scan(make_test_points(60))
        scans, report = self.capture(stream)
        self.assertEqual((len(scans), report["stop_reason"]), (1, "complete"))

    def test_unfinished_scan_is_never_returned(self):
        stream = b"\r\n" + make_csv_scan(make_test_points(60))[:-2]  # no blank
        scans, report = self.capture(stream)
        self.assertEqual((scans, report["stop_reason"]), ([], "timeout"))
        self.assertEqual(report["format"], FORMAT_CSV_XY)  # still displayed

    def test_scan_with_a_lost_row_is_rejected(self):
        rows = make_csv_scan(make_test_points(60)).split(b"\r\n")
        del rows[20]                                       # row 20 lost
        stream = b"\r\n" + b"\r\n".join(rows) + make_csv_scan(make_test_points(60, 1))
        scans, report = self.capture(stream)
        self.assertEqual(len(scans), 1)
        self.assertEqual(scans[0][0][0], 1001)  # the second, complete scan
        self.assertEqual((report["incomplete_scans"], report["rejected_scans"]), (1, 1))

    def test_malformed_rows_skipped_or_scan_rejected(self):
        rows = make_csv_scan(make_test_points(100)).split(b"\r\n")
        few = list(rows)
        few[10] = b"10,1\xff.5,1000,200,1000,-17"           # corrupted byte
        few[11] = b"11,1.5,1000,200"                       # 4 cols in 6-col scan
        many = list(rows)
        for i in range(0, 60, 6):
            many[i] = b"%d,x,1,2,3,4" % i                   # 10 % bad rows
        stream = b"\r\n" + b"\r\n".join(many) + b"\r\n".join(few)
        scans, report = self.capture(stream)
        self.assertEqual(len(scans), 1)
        self.assertEqual(len(scans[0][0]), 98)
        self.assertEqual(report["rejected_scans"], 1)
        self.assertEqual(report["malformed_entries"], 12)

    def test_too_few_echoes_rejected(self):
        stream = (b"\r\n" + make_csv_scan(make_test_points(10))
                  + b"\r\n".join(b"%d,%.2f,0,0,0,0" % (i, i * 0.8)
                                 for i in range(100)) + b"\r\n\r\n")
        scans, report = self.capture(stream)
        self.assertEqual((scans, report["rejected_scans"]), ([], 2))

    def test_four_column_scans(self):
        points = make_test_points(60)
        stream = b"\r\n" + make_csv_scan(points, with_xy=False)
        changes = []
        scans, report = self.capture(stream, chunk=11, on_format=changes.append)
        self.assertEqual((report["format"], changes), (FORMAT_CSV_POLAR, [FORMAT_CSV_POLAR]))
        # Distance is sent in whole mm and angle to 0.01 deg: <= 0.6 mm here.
        np.testing.assert_allclose(scans[0][0], [p[0] for p in points], atol=0.8)
        np.testing.assert_allclose(scans[0][1], [p[1] for p in points], atol=0.8)

    def test_format_detection_and_switching(self):
        changes = []
        stream = (make_teleplot_line(make_test_points(60))
                  + b"\r\n" + make_csv_scan(make_test_points(60, 1)))
        scans, report = self.capture(stream, count=2, on_format=changes.append)
        self.assertEqual(len(scans), 2)
        self.assertEqual(changes, [FORMAT_TELEPLOT, FORMAT_CSV_XY])
        _, report = self.capture(make_teleplot_line(make_test_points(60)))
        self.assertEqual(report["format"], FORMAT_TELEPLOT)
        _, report = self.capture(b"ets Jul 29 2019\r\n>speed:3\r\nNew scan completed.\r\n")
        self.assertIsNone(report["format"])

    def test_real_recorded_scan_replayed_as_csv(self):
        # A real classroom LD06 scan, re-sent as the ESP32 would send it in
        # both CSV variants, must come back as the recorded sensor points.
        local_m, points = recorded_scan_as_library_points()
        for with_xy, tolerance_m in ((True, 0.0006), (False, 0.0016)):
            stream = (b"412,40.08,775,229,592,-499\r\n\r\n"
                      + make_csv_scan(points, with_xy))
            scans, report = self.capture(stream, chunk=64)
            self.assertEqual(report["stop_reason"], "complete")
            x_m, y_m = teleplot_to_sensor_frame(*scans[0])
            np.testing.assert_allclose(np.column_stack((x_m, y_m)), local_m,
                                       atol=tolerance_m)

    def test_csv_matches_teleplot_for_the_same_scan(self):
        _, points = recorded_scan_as_library_points(1)
        csv_scans, _ = self.capture(b"\r\n" + make_csv_scan(points))
        teleplot_scans, _ = self.capture(make_teleplot_line(points))
        for csv_part, teleplot_part in zip(csv_scans[0], teleplot_scans[0]):
            np.testing.assert_array_equal(csv_part, teleplot_part)

    def test_serial_worker_accepts_csv(self):
        stream = (b"404,40.08,775,229,592,-499\r\n\r\n"
                  + make_csv_scan(make_test_points(60)))
        worker = start_serial_worker(FakeSerial(chop(stream, 100)))
        try:
            worker["requests"].put(("add", None))
            request, scans, report = worker["results"].get(timeout=3.0)
        finally:
            stop_serial_worker(worker)
        self.assertEqual((request, len(scans), report["format"]),
                         (("add", None), 1, FORMAT_CSV_XY))


class ExistingFeatureTests(unittest.TestCase):
    """Simulation and binary decoder must behave exactly as before."""

    def test_simulated_scan_still_uses_polar_conversion(self):
        scan = create_simulated_scan("A", "tab:blue", (0.0, 0.0, 0.0))
        x_global, y_global = scan_to_global(scan)
        front = np.where(scan["angles_deg"] == 0.0)[0][0]
        right = np.where(scan["angles_deg"] == 90.0)[0][0]
        self.assertAlmostEqual(x_global[front], ROOM_X_MAX)
        self.assertAlmostEqual(y_global[right], ROOM_Y_MIN)  # right = -Y

    def test_binary_decoder_still_works(self):
        packet = MANUFACTURER_EXAMPLE_PACKET
        self.assertTrue(is_valid_ld06_packet(packet))
        packets, crc_failures = extract_ld06_packets(
            bytearray(b"\x00\x54" + packet))
        self.assertEqual((len(packets), crc_failures), (1, 0))


def make_test_session(scan_count=2):
    """Session with Scans 1..scan_count; Scan 1 has 2 points, Scan 2 has 3.

    Scans after the second have scan number + 1 points, all different.
    """
    session = create_session()
    add_scan(session, [1.0, 0.0], [0.0, 2.0])
    if scan_count >= 2:
        add_scan(session, [3.0, -1.5, 0.25], [0.5, 0.0, -4.0])
    for scan_id in range(3, scan_count + 1):
        add_scan(session, np.arange(scan_id + 1) + scan_id,
                 np.full(scan_id + 1, -float(scan_id)))
    return session


def scan_ids(session):
    return [scan["id"] for scan in session["scans"]]


class SessionModelTests(unittest.TestCase):
    def test_new_session_is_empty(self):
        session = create_session()
        self.assertEqual(session["scans"], [])
        self.assertIsNone(selected_scan(session))
        self.assertEqual(session["next_id"], 1)

    def test_add_numbers_scans_and_selects_the_new_one(self):
        session = make_test_session(3)
        self.assertEqual(scan_ids(session), [1, 2, 3])
        self.assertEqual([scan["label"] for scan in session["scans"]],
                         ["Scan 1", "Scan 2", "Scan 3"])
        self.assertEqual(session["selected_id"], 3)
        self.assertEqual(session["scans"][1]["pose"], (0.0, 0.0, 0.0))
        self.assertTrue(session["scans"][2]["visible"])

    def test_many_scans_unique_numbers_and_repeating_colours(self):
        session = make_test_session(25)  # no limit on the number of scans
        self.assertEqual(scan_ids(session), list(range(1, 26)))
        self.assertEqual(find_scan(session, 11)["color"],
                         find_scan(session, 1)["color"])  # 10 colours
        self.assertNotEqual(find_scan(session, 2)["color"],
                            find_scan(session, 1)["color"])

    def test_stored_points_are_copies(self):
        x_local = np.array([1.0, 2.0])
        session = create_session()
        add_scan(session, x_local, np.array([0.0, 0.0]))
        x_local[0] = 99.0  # changing the caller's array...
        np.testing.assert_array_equal(session["scans"][0]["x_local_m"],
                                      [1.0, 2.0])  # ...changes nothing here

    def test_replace_changes_only_that_scans_points(self):
        session = make_test_session(3)
        set_scan_pose(session, 2, (2.0, 1.0, 90.0))
        set_scan_visible(session, 2, False)
        scan_1 = find_scan(session, 1)
        replace_scan_points(session, 2, [7.0], [8.0])
        scan_2 = find_scan(session, 2)
        np.testing.assert_array_equal(scan_2["x_local_m"], [7.0])
        self.assertEqual(scan_2["pose"], (2.0, 1.0, 90.0))  # pose kept
        self.assertEqual((scan_2["label"], scan_2["color"]),
                         ("Scan 2", color_for_scan_id(2)))
        self.assertTrue(scan_2["visible"])
        self.assertIs(find_scan(session, 1), scan_1)
        np.testing.assert_array_equal(scan_1["x_local_m"], [1.0, 0.0])
        self.assertEqual(scan_ids(session), [1, 2, 3])

    def test_delete_selects_neighbour_and_never_reuses_numbers(self):
        session = make_test_session(3)
        session["selected_id"] = 2
        delete_scan(session, 2)
        self.assertEqual(scan_ids(session), [1, 3])
        self.assertEqual(session["selected_id"], 3)  # took Scan 2's place
        delete_scan(session, 3)
        self.assertEqual(session["selected_id"], 1)  # was last: previous
        self.assertEqual(add_scan(session, [1.0], [1.0])["id"], 4)
        delete_scan(session, 1)
        delete_scan(session, 4)
        self.assertIsNone(session["selected_id"])
        self.assertEqual(add_scan(session, [1.0], [1.0])["id"], 5)

    def test_unknown_scan_number_is_an_error(self):
        session = make_test_session(2)
        for action in (lambda: delete_scan(session, 9),
                       lambda: set_scan_pose(session, 9, (0, 0, 0)),
                       lambda: replace_scan_points(session, 9, [1], [1])):
            with self.assertRaises(KeyError):
                action()

    def test_poses_are_independent_and_local_points_unchanged(self):
        session = make_test_session(3)
        set_scan_pose(session, 2, (1.0, 2.0, 90.0))
        self.assertEqual(find_scan(session, 1)["pose"], (0.0, 0.0, 0.0))
        self.assertEqual(find_scan(session, 3)["pose"], (0.0, 0.0, 0.0))
        np.testing.assert_array_equal(find_scan(session, 2)["x_local_m"],
                                      [3.0, -1.5, 0.25])
        x_global, y_global = scan_to_global(find_scan(session, 2))
        np.testing.assert_allclose(x_global[0], 0.5)  # 1 - 0.5
        np.testing.assert_allclose(y_global[0], 5.0)  # 2 + 3

    def test_reset_pose(self):
        session = make_test_session(2)
        set_scan_pose(session, 2, (1.0, 2.0, 30.0))
        reset_scan_pose(session, 2)
        self.assertEqual(find_scan(session, 2)["pose"], (0.0, 0.0, 0.0))

    def test_visibility_is_per_scan(self):
        session = make_test_session(3)
        set_scan_visible(session, 2, False)
        self.assertEqual([scan["id"] for scan in shown_scans(session)], [1, 3])

    def test_view_limits_follow_shown_scans(self):
        session = make_test_session(2)
        set_scan_pose(session, 2, (10.0, 0.0, 0.0))
        (x_min, x_max), _ = compute_view_limits(session)
        self.assertGreater(x_max, 13.0)  # Scan 2's point at local x = 3 m
        set_scan_visible(session, 2, False)
        (x_min, x_max), _ = compute_view_limits(session)
        self.assertLess(x_max, 3.0)

    def test_list_scrolling_keeps_selected_row_on_screen(self):
        self.assertEqual(scroll_to_show(0, 14, 10, 25), 5)   # scroll down
        self.assertEqual(scroll_to_show(10, 3, 10, 25), 3)   # scroll up
        self.assertEqual(scroll_to_show(4, 6, 10, 25), 4)    # already shown
        self.assertEqual(scroll_to_show(40, None, 10, 25), 15)  # clamped
        self.assertEqual(scroll_to_show(3, None, 10, 4), 0)  # short list


def make_valid_session_dict(scan_count=2):
    """A valid saved session (as read back by json.loads) for the tests."""
    session = make_test_session(scan_count)
    set_scan_pose(session, 2, (1.25, -0.5, 45.0))
    return json.loads(json.dumps(session_to_json_dict(session)))


def make_version_1_dict():
    """A session exactly as the previous (A/B/C) program saved it.

    A and C were captured (B never was); C is hidden and selected.
    """
    def slot(name, color, pose, points):
        return {"name": name, "color": color,
                "pose": {"x_m": pose[0], "y_m": pose[1], "theta_deg": pose[2]},
                "point_count": len(points), "points_local_m": points}

    return {
        "format": "ld06_slam_session", "version": 1,
        "saved_at": "2026-11-02T16:30:00", "conventions": {},
        "selected_scan": "C",
        "visible": {"A": True, "B": True, "C": False},
        "scans": {
            "A": slot("Scan A", "tab:blue", (0.0, 0.0, 0.0),
                      [[1.32, -0.016], [1.336, -0.033]]),
            "B": None,
            "C": slot("Scan C", "tab:green", (2.5, -1.0, 170.0),
                      [[0.5, 0.25], [-0.75, 1.5], [2.0, -2.0]]),
        },
    }


class SessionFileTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.path = Path(self.folder.name) / "session.json"

    def tearDown(self):
        self.folder.cleanup()

    def assert_same_scans(self, original, restored):
        self.assertEqual(scan_ids(restored), scan_ids(original))
        for before, after in zip(original["scans"], restored["scans"]):
            for key in ("pose", "label", "color", "visible"):
                self.assertEqual(after[key], before[key])
            np.testing.assert_allclose(after["x_local_m"],
                                       before["x_local_m"], atol=1e-6)
            np.testing.assert_allclose(after["y_local_m"],
                                       before["y_local_m"], atol=1e-6)

    def test_round_trip_with_varying_numbers_of_scans(self):
        for scan_count in (0, 1, 3, 12):
            with self.subTest(scan_count=scan_count):
                session = make_test_session(scan_count) if scan_count \
                    else create_session()
                for scan in session["scans"]:  # every scan its own pose
                    set_scan_pose(session, scan["id"],
                                  (0.1 * scan["id"], -0.2 * scan["id"],
                                   15.0 * scan["id"] - 90.0))
                    set_scan_visible(session, scan["id"], scan["id"] % 3 != 0)
                if scan_count:
                    session["selected_id"] = session["scans"][0]["id"]
                    session["scans"][0]["color"] = "#123456"
                save_session(session, self.path)
                loaded, version = load_session(self.path)
                self.assertEqual(version, SESSION_FORMAT_VERSION)
                self.assert_same_scans(session, loaded)
                self.assertEqual(loaded["selected_id"], session["selected_id"])
                self.assertEqual(loaded["next_id"], session["next_id"])

    def test_numbers_survive_deletions_and_reload(self):
        session = make_test_session(3)
        delete_scan(session, 2)
        delete_scan(session, 3)
        save_session(session, self.path)
        loaded, _ = load_session(self.path)
        self.assertEqual(scan_ids(loaded), [1])
        self.assertEqual(add_scan(loaded, [1.0], [1.0])["id"], 4)

    def test_file_is_readable_and_stores_only_local_points(self):
        session = make_test_session(2)
        set_scan_pose(session, 2, (5.0, 5.0, 90.0))  # global != local
        save_session(session, self.path)
        text = self.path.read_text(encoding="utf-8")
        data = json.loads(text)
        self.assertEqual(data["format"], SESSION_FORMAT_NAME)
        self.assertEqual(data["version"], 2)
        self.assertIn("conventions", data)
        self.assertEqual([scan["id"] for scan in data["scans"]], [1, 2])
        self.assertEqual(data["scans"][1]["points_local_m"][0], [3.0, 0.5])
        self.assertIn("[3.0, 0.5]", text)  # one point per line
        self.assertFalse(Path(str(self.path) + ".tmp").exists())

    def test_version_1_file_is_converted_without_losing_data(self):
        self.path.write_text(json.dumps(make_version_1_dict()),
                             encoding="utf-8")
        before = self.path.read_bytes()
        session, version = load_session(self.path)
        self.assertEqual(version, 1)
        self.assertEqual(scan_ids(session), [1, 3])  # A -> 1, C -> 3
        scan_1, scan_3 = session["scans"]
        self.assertEqual(scan_1["label"], "Scan 1 (was Scan A)")
        self.assertEqual(scan_3["label"], "Scan 3 (was Scan C)")
        self.assertEqual(scan_3["pose"], (2.5, -1.0, 170.0))
        self.assertEqual(scan_3["color"], "tab:green")
        self.assertEqual((scan_1["visible"], scan_3["visible"]),
                         (True, False))
        np.testing.assert_array_equal(scan_3["y_local_m"], [0.25, 1.5, -2.0])
        self.assertEqual(session["selected_id"], 3)
        self.assertEqual(session["next_id"], 4)
        self.assertEqual(self.path.read_bytes(), before)  # file untouched

        save_session(session, self.path)  # saving writes version 2
        resaved, version = load_session(self.path)
        self.assertEqual(version, 2)
        self.assert_same_scans(session, resaved)

    def test_malformed_version_1_files_are_rejected(self):
        for description, change in {
                "unknown slot": lambda data: data["scans"].update(D=None),
                "scans list": lambda data: data.update(scans=[]),
                "bad pose": lambda data: data["scans"]["A"]["pose"].update(
                    x_m="1"),
                "bad visible": lambda data: data["visible"].update(A="yes"),
                "bad selected": lambda data: data.update(selected_scan="Z"),
        }.items():
            with self.subTest(description):
                data = make_version_1_dict()
                change(data)
                with self.assertRaises(SessionFileError):
                    session_from_json_dict(data)

    def test_malformed_sessions_are_rejected(self):
        def first_scan(change):
            return lambda data: change(data["scans"][0])

        cases = {
            "wrong format": lambda data: data.update(format="other"),
            "future version": lambda data: data.update(version=3),
            "scans not list": lambda data: data.update(scans={}),
            "scan not object": lambda data: data["scans"].append([1, 2]),
            "id missing": first_scan(lambda scan: scan.pop("id")),
            "id zero": first_scan(lambda scan: scan.update(id=0)),
            "id text": first_scan(lambda scan: scan.update(id="1")),
            "id bool": first_scan(lambda scan: scan.update(id=True)),
            "id twice": lambda data: data["scans"][1].update(id=1),
            "visible text": first_scan(lambda scan: scan.update(
                visible="yes")),
            "pose missing": first_scan(lambda scan: scan.pop("pose")),
            "pose text": first_scan(lambda scan: scan["pose"].update(
                x_m="1")),
            "pose NaN": first_scan(lambda scan: scan["pose"].update(
                y_m=float("nan"))),
            "pose bool": first_scan(lambda scan: scan["pose"].update(
                theta_deg=True)),
            "no points": first_scan(lambda scan: scan.update(
                points_local_m=[])),
            "point not pair": first_scan(lambda scan: scan.update(
                points_local_m=[[1.0, 2.0, 3.0]])),
            "point text": first_scan(lambda scan: scan.update(
                points_local_m=[["1", 2.0]])),
            "count mismatch": first_scan(lambda scan: scan.update(
                point_count=99)),
            "name number": first_scan(lambda scan: scan.update(name=5)),
            "next id text": lambda data: data.update(next_scan_id="3"),
            "selected missing scan": lambda data: data.update(
                selected_scan_id=42),
        }
        for description, change in cases.items():
            with self.subTest(description):
                data = make_valid_session_dict()
                change(data)
                with self.assertRaises(SessionFileError):
                    session_from_json_dict(data)
        with self.assertRaises(SessionFileError):
            session_from_json_dict([1, 2, 3])

    def test_too_small_next_id_is_raised_to_avoid_reuse(self):
        data = make_valid_session_dict(3)
        data["next_scan_id"] = 2
        session, _ = session_from_json_dict(data)
        self.assertEqual(session["next_id"], 4)

    def test_unreadable_files_are_rejected(self):
        with self.assertRaises(SessionFileError):
            load_session(self.path)  # does not exist
        for content in (b"{not json", b"", b"\xff\xfe\x00binary"):
            with self.subTest(content):
                self.path.write_bytes(content)
                with self.assertRaises(SessionFileError):
                    load_session(self.path)

    def test_unknown_colour_falls_back_to_default(self):
        data = make_valid_session_dict()
        data["scans"][0]["color"] = "not-a-colour"
        session, _ = session_from_json_dict(data)
        self.assertEqual(session["scans"][0]["color"], color_for_scan_id(1))

    def test_file_names_go_to_scans_folder(self):
        self.assertEqual(session_path_from_name("room1"),
                         SCANS_DIR / "room1.json")
        self.assertEqual(session_path_from_name("  "),
                         SCANS_DIR / DEFAULT_SESSION_FILE)


def wait_for_result(worker, timeout_s=3.0):
    """Wait (in the test only) until the worker thread has posted a result."""
    deadline = time.monotonic() + timeout_s
    while worker["results"].empty() and time.monotonic() < deadline:
        time.sleep(0.02)


class SerialWorkerTests(unittest.TestCase):
    def test_one_scan_per_request(self):
        before_first = (b"5:6;7:8;|xy\r\n"  # end of a line started earlier
                        + make_teleplot_line(make_test_points(60)))
        after_second = make_teleplot_line(make_test_points(60, 1))
        # Each capture starts with an EMPTY buffer, so the second scan must
        # start after the first capture ended (as after a real second click).
        worker = start_serial_worker(FakeSerial(chop(before_first, 50)
                                                + [after_second]))
        try:
            worker["requests"].put(("add", None))
            request, scans, report = worker["results"].get(timeout=3.0)
            self.assertEqual((request, len(scans)), (("add", None), 1))
            self.assertEqual(report["stop_reason"], "complete")
            self.assertEqual(scans[0][0][0], 1000)

            worker["requests"].put(("replace", 7))  # the NEXT scan
            request, scans, _ = worker["results"].get(timeout=3.0)
            self.assertEqual((request, scans[0][0][0]), (("replace", 7), 1001))
        finally:
            stop_serial_worker(worker)
        self.assertFalse(worker["thread"].is_alive())

    def test_disconnection_is_reported_and_thread_ends(self):
        worker = start_serial_worker(FakeSerial([], disconnect_at_end=True))
        worker["requests"].put(("add", None))
        request, scans, report = worker["results"].get(timeout=3.0)
        self.assertEqual((request, scans), (("add", None), []))
        self.assertTrue(report["stop_reason"].startswith("disconnected"))
        worker["thread"].join(timeout=2.0)
        self.assertFalse(worker["thread"].is_alive())
        stop_serial_worker(worker)


class SessionEditorTests(unittest.TestCase):
    """The real window, drawn off-screen (Agg backend): no mouse needed."""

    def setUp(self):
        plt.switch_backend("Agg")
        self.folder = tempfile.TemporaryDirectory()
        self.path = str(Path(self.folder.name) / "classroom.json")

    def tearDown(self):
        plt.close("all")
        self.folder.cleanup()

    def capture_with_fake_esp32(self, session, action, point_count=60):
        """Press Add scan / Replace selected with a fake ESP32 attached."""
        worker = start_serial_worker(FakeSerial(
            [make_teleplot_line(make_test_points(point_count))]))
        try:
            _, controls = create_session_editor(session, worker)
            controls["actions"][action]()
            wait_for_result(worker)
            controls["actions"]["poll"]()  # what the 100 ms timer does
        finally:
            stop_serial_worker(worker)
        return controls

    def test_add_scan_appends_and_selects_it(self):
        session = make_test_session(3)
        old_scans = list(session["scans"])
        controls = self.capture_with_fake_esp32(session, "add")
        self.assertEqual(scan_ids(session), [1, 2, 3, 4])
        self.assertEqual(session["selected_id"], 4)
        self.assertEqual(len(find_scan(session, 4)["x_local_m"]), 60)
        for old, now in zip(old_scans, session["scans"]):
            self.assertIs(now, old)  # earlier scans untouched
        # Teleplot (1000 mm, -500 mm) -> (1.0 m, +0.5 m): Y flipped once.
        self.assertEqual(find_scan(session, 4)["x_local_m"][0], 1.0)
        self.assertEqual(find_scan(session, 4)["y_local_m"][0], 0.5)
        self.assertIn("Scan 4", controls["row_texts"][3].get_text())
        self.assertIn("60 pts", controls["row_texts"][3].get_text())

    def test_replace_selected_changes_only_that_scan(self):
        session = make_test_session(3)
        set_scan_pose(session, 2, (1.0, 2.0, 30.0))
        session["selected_id"] = 2
        self.capture_with_fake_esp32(session, "replace", point_count=70)
        self.assertEqual(scan_ids(session), [1, 2, 3])
        self.assertEqual(len(find_scan(session, 2)["x_local_m"]), 70)
        self.assertEqual(find_scan(session, 2)["pose"], (1.0, 2.0, 30.0))
        self.assertEqual(len(find_scan(session, 1)["x_local_m"]), 2)
        self.assertEqual(len(find_scan(session, 3)["x_local_m"]), 4)

    def test_offline_add_explains_and_changes_nothing(self):
        session = make_test_session(2)
        _, controls = create_session_editor(session, worker=None)
        controls["actions"]["add"]()
        self.assertEqual(scan_ids(session), [1, 2])
        self.assertIn("Offline", controls["status_text"].get_text())

    def test_selecting_restores_each_scans_own_slider_values(self):
        session = make_test_session(3)
        _, controls = create_session_editor(session)
        x_slider, y_slider, theta_slider = controls["sliders"]

        controls["actions"]["select_row"](1)  # click the Scan 2 row
        x_slider.set_val(1.5)
        theta_slider.set_val(30.0)
        self.assertEqual(find_scan(session, 2)["pose"], (1.5, 0.0, 30.0))
        self.assertEqual(find_scan(session, 1)["pose"], (0.0, 0.0, 0.0))
        self.assertEqual(find_scan(session, 3)["pose"], (0.0, 0.0, 0.0))
        np.testing.assert_array_equal(find_scan(session, 2)["x_local_m"],
                                      [3.0, -1.5, 0.25])

        controls["actions"]["previous"]()  # Scan 1: its own values
        self.assertEqual(session["selected_id"], 1)
        self.assertEqual((x_slider.val, theta_slider.val), (0.0, 0.0))
        y_slider.set_val(-2.0)
        self.assertEqual(find_scan(session, 1)["pose"], (0.0, -2.0, 0.0))

        controls["actions"]["next"]()  # Scan 2's values come back
        self.assertEqual((x_slider.val, y_slider.val, theta_slider.val),
                         (1.5, 0.0, 30.0))
        self.assertTrue(controls["row_texts"][1].get_text().startswith(">"))

    def test_long_list_scrolls_to_the_selected_scan(self):
        session = make_test_session(25)  # last added = selected = Scan 25
        _, controls = create_session_editor(session)
        rows = [text.get_text() for text in controls["row_texts"]]
        self.assertIn("Scan 25", rows[-1])  # scrolled to the bottom
        self.assertIn("rows 16-25", controls["list_title"].get_text())
        controls["actions"]["scroll"](-100)  # back to the top
        self.assertIn("Scan 1 ", controls["row_texts"][0].get_text())
        self.assertEqual(session["selected_id"], 25)  # scrolling != select
        controls["actions"]["select_row"](2)
        self.assertEqual(session["selected_id"], 3)

    def test_hide_show_selected_and_show_all(self):
        session = make_test_session(3)
        _, controls = create_session_editor(session)
        controls["actions"]["select"](2)
        controls["actions"]["toggle_visibility"]()
        self.assertFalse(find_scan(session, 2)["visible"])
        self.assertTrue(find_scan(session, 1)["visible"])
        self.assertFalse(controls["artists"][2]["points"].get_visible())
        self.assertTrue(controls["artists"][1]["points"].get_visible())
        self.assertIn("hidden", controls["row_texts"][1].get_text())
        self.assertEqual(controls["visibility_button"].label.get_text(),
                         "Show selected")
        controls["actions"]["select"](3)
        controls["actions"]["toggle_visibility"]()
        controls["actions"]["show_all"]()
        self.assertEqual(len(shown_scans(session)), 3)

    def test_delete_needs_confirmation_and_removes_only_selected(self):
        session = make_test_session(3)
        _, controls = create_session_editor(session)
        controls["actions"]["select"](2)
        controls["actions"]["delete"]()
        self.assertEqual(scan_ids(session), [1, 2, 3])  # only asked
        controls["actions"]["delete"]()
        self.assertEqual(scan_ids(session), [1, 3])
        self.assertNotIn(2, controls["artists"])  # drawing removed too
        self.assertEqual(session["selected_id"], 3)
        self.assertIn("Scan 4", controls["status_text"].get_text())

    def test_reset_pose_button(self):
        session = make_test_session(2)
        _, controls = create_session_editor(session)
        controls["sliders"][0].set_val(2.0)  # Scan 2 is selected
        controls["actions"]["reset_pose"]()
        self.assertEqual(find_scan(session, 2)["pose"], (0.0, 0.0, 0.0))
        self.assertEqual(controls["sliders"][0].val, 0.0)

    def test_sliders_without_selection_change_nothing(self):
        session = create_session()
        _, controls = create_session_editor(session)
        controls["sliders"][0].set_val(3.0)
        self.assertEqual(controls["sliders"][0].val, 0.0)

    def test_save_then_load_restores_the_aligned_map(self):
        session = make_test_session(5)
        _, controls = create_session_editor(session)
        controls["actions"]["select"](2)
        controls["sliders"][0].set_val(1.2)
        controls["sliders"][2].set_val(-45.0)
        controls["actions"]["select"](4)
        controls["actions"]["toggle_visibility"]()  # hide Scan 4
        controls["actions"]["select"](2)
        controls["file_box"].set_val(self.path)
        controls["actions"]["save"]()
        self.assertTrue(Path(self.path).exists())

        # A NEW, empty window (as after restarting offline) loads the file.
        new_session = create_session()
        _, new_controls = create_session_editor(new_session)
        new_controls["file_box"].set_val(self.path)
        new_controls["actions"]["load"]()
        self.assertEqual(scan_ids(new_session), [1, 2, 3, 4, 5])
        self.assertEqual(find_scan(new_session, 2)["pose"], (1.2, 0.0, -45.0))
        self.assertEqual(new_session["selected_id"], 2)
        self.assertFalse(find_scan(new_session, 4)["visible"])
        self.assertFalse(new_controls["artists"][4]["points"].get_visible())
        self.assertEqual(new_controls["sliders"][0].val, 1.2)
        self.assertEqual(new_controls["sliders"][2].val, -45.0)

    def test_loading_replaces_all_scans_on_screen(self):
        save_session(make_test_session(2), self.path)
        session = make_test_session(6)
        _, controls = create_session_editor(session)
        controls["file_box"].set_val(self.path)
        controls["actions"]["load"]()
        self.assertEqual(scan_ids(session), [1, 2])
        self.assertEqual(sorted(controls["artists"]), [1, 2])

    def test_loading_a_version_1_file_in_the_window(self):
        Path(self.path).write_text(json.dumps(make_version_1_dict()),
                                   encoding="utf-8")
        session = create_session()
        _, controls = create_session_editor(session)
        controls["file_box"].set_val(self.path)
        controls["actions"]["load"]()
        self.assertEqual(scan_ids(session), [1, 3])
        self.assertIn("converted", controls["status_text"].get_text())
        self.assertEqual(controls["sliders"][0].val, 2.5)  # Scan 3 selected

    def test_failed_load_keeps_the_current_map(self):
        Path(self.path).write_text("{broken", encoding="utf-8")
        session = make_test_session(2)
        _, controls = create_session_editor(session)
        controls["file_box"].set_val(self.path)
        controls["actions"]["load"]()
        self.assertEqual(scan_ids(session), [1, 2])
        self.assertIn("Could not load", controls["status_text"].get_text())

    def test_overwrite_needs_a_second_click(self):
        Path(self.path).write_text("old", encoding="utf-8")
        _, controls = create_session_editor(make_test_session(2))
        controls["file_box"].set_val(self.path)
        controls["actions"]["save"]()
        self.assertEqual(Path(self.path).read_text(encoding="utf-8"), "old")
        controls["actions"]["save"]()
        self.assertIn(SESSION_FORMAT_NAME, Path(self.path).read_text())


class AlignmentMathTests(unittest.TestCase):
    def test_angles_wrap_and_compare(self):
        self.assertEqual(wrap_angle_deg(190.0), -170.0)
        self.assertEqual(wrap_angle_deg(-180.0), -180.0)
        self.assertAlmostEqual(angle_difference_deg(179.0, -179.0), 2.0)
        self.assertAlmostEqual(angle_difference_deg(-90.0, 90.0), 180.0)

    def test_global_to_local_undoes_transform_points(self):
        points = np.array([[1.0, 0.0], [0.5, -2.0], [-3.0, 4.0]])
        for pose in ((0.0, 0.0, 0.0), (1.5, -2.0, 90.0), (-4.0, 3.0, -137.5)):
            np.testing.assert_allclose(
                global_to_local(transform_points(points, pose), pose), points,
                atol=1e-12)

    def test_best_rigid_transform_recovers_exact_motion(self):
        rng = np.random.default_rng(3)
        points = rng.uniform(-5, 5, (50, 2))
        moved = transform_points(points, (0.7, -1.2, -63.0))
        angle_rad, translation = best_rigid_transform(points, moved)
        self.assertAlmostEqual(math.degrees(angle_rad), -63.0)
        np.testing.assert_allclose(translation, [0.7, -1.2], atol=1e-9)

    def test_voxel_downsample_keeps_one_average_point_per_cell(self):
        points = np.array([[0.01, 0.01], [0.03, 0.03], [0.51, 0.0]])
        np.testing.assert_allclose(
            sorted(map(tuple, voxel_downsample(points, 0.1))),
            [(0.02, 0.02), (0.51, 0.0)])

    def test_icp_corrects_a_small_error(self):
        source, references = make_alignment_case(
            CLASSROOM_WALLS, REFERENCE_TEST_POSE, (3.0, 4.5, 90.0),
            noise_mm=0.0)
        reference = transform_points(*references[0])
        pose = icp(voxel_downsample(source, ALIGN_ICP_VOXEL_M), reference,
                   (3.15, 4.35, 85.0))
        error_m, error_deg = pose_errors(pose, (3.0, 4.5, 90.0))
        self.assertLess(error_m, 0.02)
        self.assertLess(error_deg, 0.5)

    def test_see_through_counts(self):
        # The viewer saw a wall 5 m ahead (+X) and nothing else.
        viewer = np.array([[5.0, 0.0]])
        other = np.array([[2.0, 0.0],    # nearer: the beam went through it
                          [5.05, 0.0],   # same wall: consistent
                          [7.0, 0.0],    # behind the wall: unknown
                          [0.0, 3.0]])   # direction never measured: unknown
        self.assertEqual(see_through_counts(viewer, other), (1, 1))

    def test_judge_alignment_verdicts(self):
        def result(match, rms, conflict, pose=(0.0, 0.0, 0.0)):
            return {"pose": pose, "match_fraction": match, "rms_m": rms,
                    "conflict": conflict,
                    "score": alignment_score(match, conflict)}

        far_pose = (3.0, 0.0, 180.0)
        cases = [
            (result(0.8, 0.02, 0.0), None, "good"),
            (result(0.8, 0.02, 0.0), result(0.4, 0.02, 0.0, far_pose),
             "good"),
            (result(0.4, 0.02, 0.0), None, "weak"),         # little overlap
            (result(0.8, 0.04, 0.0), None, "weak"),         # rather loose
            (result(0.8, 0.02, 0.15), None, "weak"),        # some conflicts
            (result(0.2, 0.02, 0.0), None, "failed"),       # no overlap
            (result(0.8, 0.06, 0.0), None, "failed"),       # poor fit
            (result(0.8, 0.02, 0.3), None, "failed"),       # sees through
            (result(0.8, 0.02, 0.0), result(0.78, 0.02, 0.0, far_pose),
             "failed"),                                      # ambiguous
        ]
        for best, runner_up, expected in cases:
            with self.subTest(best=best, runner_up=runner_up):
                self.assertEqual(judge_alignment(best, runner_up)[0], expected)

    def test_one_sided_conflicts_need_check_visually_never_good(self):
        # Real Scan 6 (LiDAR above the desks): 49 % pooled conflicts, but
        # the reference scans see through only 2 % of its points.
        def result(match, rms, pooled, reference_sees):
            return {"pose": (0.0, 0.0, 0.0), "match_fraction": match,
                    "rms_m": rms, "conflict": pooled,
                    "scan_sees_through": 0.63,
                    "reference_sees_through": reference_sees,
                    "score": alignment_score(match, reference_sees)}
        cases = [
            (result(0.53, 0.037, 0.49, 0.02), "weak"),
            (result(0.95, 0.010, 0.49, 0.02), "weak"),    # never "good"
            (result(0.53, 0.037, 0.49, 0.15), "failed"),  # two-sided
            (result(0.45, 0.037, 0.49, 0.02), "failed"),  # too little match
            (result(0.53, 0.050, 0.49, 0.02), "failed"),  # loose fit
        ]
        for best, expected in cases:
            with self.subTest(best=best):
                status, reason = judge_alignment(best, None)
                self.assertEqual(status, expected)
                if expected == "weak":
                    self.assertIn("ONE-SIDED", reason)

    def test_directional_conflicts_pool_like_free_space_conflict(self):
        source, reference = make_alignment_case(
            CLASSROOM_WALLS, (2.0, 2.5, 30.0), (4.0, 3.0, 100.0), seed=3)
        for pose in ((4.0, 3.0, 100.0), (4.5, 2.0, 280.0)):
            pooled, scan_sees, reference_sees = directional_conflicts(
                source, reference, pose)
            self.assertEqual(pooled, free_space_conflict(source, reference,
                                                         pose))
            self.assertTrue(0.0 <= scan_sees <= 1.0
                            and 0.0 <= reference_sees <= 1.0)


class AlignmentScenarioTests(unittest.TestCase):
    """The benchmark cases: known answers, any rotation (about 20 s)."""

    def test_benchmark_cases(self):
        for name, walls, reference_pose, true_pose, options, expected in \
                ALIGNMENT_BENCHMARK_CASES:
            with self.subTest(name):
                source, references = make_alignment_case(
                    walls, reference_pose, true_pose, **options)
                result = align_scan(source, references)
                if expected == "failed":
                    self.assertEqual(result["status"], "failed")
                    continue
                self.assertIn(result["status"], ("good", "weak"),
                              result["reason"])
                error_m, error_deg = pose_errors(result["pose"], true_pose)
                self.assertLess(error_m, 0.05)
                self.assertLess(error_deg, 1.5)

    def test_symmetric_room_is_reported_as_ambiguous(self):
        source, references = make_alignment_case(
            SYMMETRIC_ROOM_WALLS, (1.5, 1.0, 0.0), (4.0, 3.0, 120.0))
        result = align_scan(source, references)
        self.assertEqual(result["status"], "failed")
        self.assertIn("ambiguous", result["reason"])

    def test_inputs_are_not_modified(self):
        source, references = make_alignment_case(
            CLASSROOM_WALLS, REFERENCE_TEST_POSE, (3.0, 4.5, 90.0))
        source_copy = source.copy()
        reference_copy = references[0][0].copy()
        align_scan(source, references)
        np.testing.assert_array_equal(source, source_copy)
        np.testing.assert_array_equal(references[0][0], reference_copy)

    def test_too_few_points_fails_cleanly(self):
        result = align_scan(np.array([[1.0, 0.0], [0.0, 1.0]]),
                            [(np.array([[1.0, 0.0]]), (0.0, 0.0, 0.0))])
        self.assertEqual(result["status"], "failed")
        self.assertIsNone(result["pose"])
        self.assertIn("not enough points", result["reason"])


def make_alignment_session(source_pose=(3.0, 4.5, 90.0),
                           walls=CLASSROOM_WALLS):
    """Session: Scan 1 = reference at its true pose, Scan 2 unaligned at 0."""
    source, references = make_alignment_case(walls, REFERENCE_TEST_POSE
                                             if walls is CLASSROOM_WALLS
                                             else (1.5, 1.0, 0.0),
                                             source_pose)
    reference_local, reference_pose = references[0]
    session = create_session()
    add_scan(session, reference_local[:, 0], reference_local[:, 1])
    set_scan_pose(session, 1, reference_pose)
    add_scan(session, source[:, 0], source[:, 1])
    return session


class AlignmentSessionTests(unittest.TestCase):
    def test_reference_scans_are_first_plus_accepted_ones(self):
        session = make_test_session(4)
        self.assertEqual(scan_ids_of(alignment_reference_scans(session, 3)),
                         [1])
        accept_alignment(session, 2, (1.0, 0.0, 10.0), {"status": "good"})
        self.assertEqual(scan_ids_of(alignment_reference_scans(session, 3)),
                         [1, 2])
        set_scan_visible(session, 2, False)  # hidden scans are left out
        self.assertEqual(scan_ids_of(alignment_reference_scans(session, 3)),
                         [1])
        self.assertEqual(scan_ids_of(alignment_reference_scans(session, 1)),
                         [])  # the reference itself has nothing to align to

    def test_accept_changes_only_the_pose(self):
        session = make_test_session(2)
        before = find_scan(session, 2)["x_local_m"].copy()
        accept_alignment(session, 2, (1.0, -2.0, 33.0), {"status": "good"})
        scan = find_scan(session, 2)
        self.assertEqual(scan["pose"], (1.0, -2.0, 33.0))
        self.assertTrue(scan["aligned"])
        np.testing.assert_array_equal(scan["x_local_m"], before)
        self.assertEqual(find_scan(session, 1)["pose"], (0.0, 0.0, 0.0))

    def test_reset_and_replace_remove_the_aligned_mark(self):
        session = make_test_session(3)
        for scan_id in (2, 3):
            accept_alignment(session, scan_id, (1.0, 0.0, 0.0), {})
        reset_scan_pose(session, 2)
        replace_scan_points(session, 3, [1.0], [1.0])
        self.assertFalse(find_scan(session, 2)["aligned"])
        self.assertFalse(find_scan(session, 3)["aligned"])
        self.assertEqual(find_scan(session, 3)["pose"], (1.0, 0.0, 0.0))

    def test_alignment_survives_save_and_load(self):
        session = make_test_session(3)
        quality = {"status": "good", "match_fraction": 0.81, "rms_m": 0.021,
                   "runner_up_score": None, "reference_scans": "1"}
        accept_alignment(session, 2, (1.25, -0.5, 47.5), quality)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "aligned.json"
            save_session(session, path)
            loaded, _ = load_session(path)
        self.assertEqual(find_scan(loaded, 2)["pose"], (1.25, -0.5, 47.5))
        self.assertTrue(find_scan(loaded, 2)["aligned"])
        self.assertEqual(find_scan(loaded, 2)["alignment_quality"], quality)
        self.assertFalse(find_scan(loaded, 3)["aligned"])

    def test_older_version_2_files_without_alignment_fields_load(self):
        data = make_valid_session_dict()
        for entry in data["scans"]:
            del entry["aligned"], entry["alignment_quality"]
        session, _ = session_from_json_dict(data)
        self.assertFalse(any(scan["aligned"] for scan in session["scans"]))

    def test_malformed_alignment_fields_are_rejected(self):
        for change in ({"aligned": "yes"}, {"alignment_quality": [1]},
                       {"alignment_quality": {"rms_m": [0.1]}}):
            with self.subTest(change):
                data = make_valid_session_dict()
                data["scans"][0].update(change)
                with self.assertRaises(SessionFileError):
                    session_from_json_dict(data)


def scan_ids_of(scans):
    return [scan["id"] for scan in scans]


def wait_for_alignment(controls, timeout_s=60.0):
    """Wait (in the tests only) for the alignment thread, polling like the
    window's timer would (timers do not run off-screen)."""
    deadline = time.monotonic() + timeout_s
    while controls["is_aligning"]() and time.monotonic() < deadline:
        controls["actions"]["poll"]()
        time.sleep(0.05)
    controls["actions"]["poll"]()


class AlignmentEditorTests(unittest.TestCase):
    def setUp(self):
        plt.switch_backend("Agg")

    def tearDown(self):
        plt.close("all")

    def test_proposal_then_accept_moves_only_the_pose(self):
        session = make_alignment_session()
        local_before = find_scan(session, 2)["x_local_m"].copy()
        _, controls = create_session_editor(session)  # Scan 2 selected
        controls["actions"]["auto_align"]()
        self.assertTrue(controls["is_aligning"]())  # did not block
        self.assertIn("...", controls["row_texts"][1].get_text())
        wait_for_alignment(controls)

        self.assertEqual(find_scan(session, 2)["pose"], (0.0, 0.0, 0.0))
        self.assertIsNotNone(controls["proposal"]())
        self.assertTrue(controls["proposal_points"].get_visible())
        self.assertIn("Proposal for Scan 2", controls["report_text"].get_text())
        self.assertIn("matched", controls["report_text"].get_text())

        controls["actions"]["accept"]()
        error_m, error_deg = pose_errors(find_scan(session, 2)["pose"],
                                         (3.0, 4.5, 90.0))
        self.assertLess(error_m, 0.05)
        self.assertLess(error_deg, 1.5)
        self.assertTrue(find_scan(session, 2)["aligned"])
        np.testing.assert_array_equal(find_scan(session, 2)["x_local_m"],
                                      local_before)
        self.assertAlmostEqual(controls["sliders"][0].val,
                               find_scan(session, 2)["pose"][0])
        self.assertTrue(controls["row_texts"][1].get_text().endswith("ok"))
        self.assertTrue(controls["row_texts"][0].get_text().endswith("ref"))
        self.assertFalse(controls["proposal_points"].get_visible())

    def test_reject_leaves_the_pose_unchanged(self):
        session = make_alignment_session()
        _, controls = create_session_editor(session)
        controls["actions"]["auto_align"]()
        wait_for_alignment(controls)
        controls["actions"]["reject"]()
        self.assertEqual(find_scan(session, 2)["pose"], (0.0, 0.0, 0.0))
        self.assertFalse(find_scan(session, 2)["aligned"])
        self.assertIsNone(controls["proposal"]())

    def test_failed_alignment_cannot_be_accepted(self):
        session = make_alignment_session((4.0, 3.0, 120.0),
                                         SYMMETRIC_ROOM_WALLS)
        _, controls = create_session_editor(session)
        controls["actions"]["auto_align"]()
        wait_for_alignment(controls)
        self.assertIn("NOT ACCEPTABLE", controls["report_text"].get_text())
        controls["actions"]["accept"]()
        self.assertEqual(find_scan(session, 2)["pose"], (0.0, 0.0, 0.0))
        self.assertIn("Cannot accept", controls["status_text"].get_text())

    def test_reference_scan_is_never_auto_aligned(self):
        session = make_alignment_session()
        _, controls = create_session_editor(session)
        controls["actions"]["select"](1)
        controls["actions"]["auto_align"]()
        self.assertFalse(controls["is_aligning"]())
        self.assertIn("fixed reference", controls["status_text"].get_text())

    def test_delete_and_load_wait_for_the_alignment(self):
        session = make_alignment_session()
        _, controls = create_session_editor(session)
        controls["actions"]["auto_align"]()
        controls["actions"]["delete"]()
        controls["actions"]["delete"]()
        self.assertEqual(scan_ids(session), [1, 2])
        wait_for_alignment(controls)


class RayDisplayTests(unittest.TestCase):
    def setUp(self):
        plt.switch_backend("Agg")

    def tearDown(self):
        plt.close("all")

    def test_ray_segments_follow_the_pose(self):
        scan = create_cartesian_scan("s", "tab:blue", (1.0, 2.0, 90.0),
                                     np.array([1.0, 0.0, 3.0]),
                                     np.array([0.0, 2.0, 4.0]))
        segments = scan_ray_segments(scan)
        self.assertEqual(segments.shape, (3, 2, 2))
        np.testing.assert_allclose(segments[:, 0], [[1.0, 2.0]] * 3)  # origin
        x_global, y_global = scan_to_global(scan)
        np.testing.assert_allclose(segments[:, 1, 0], x_global)       # ends
        np.testing.assert_allclose(segments[:, 1, 1], y_global)
        np.testing.assert_allclose(segments[0, 1], [1.0, 3.0], atol=1e-12)
        lengths = np.hypot(*(segments[:, 1] - segments[:, 0]).T)
        np.testing.assert_allclose(lengths, [1.0, 2.0, 5.0])  # = ranges

    def test_rays_move_with_the_sliders(self):
        session = make_test_session(2)
        _, controls = create_session_editor(session)  # Scan 2 selected
        controls["sliders"][0].set_val(1.5)
        controls["sliders"][2].set_val(90.0)
        segments = controls["artists"][2]["rays"].get_segments()
        np.testing.assert_allclose([segment[0] for segment in segments],
                                   [[1.5, 0.0]] * 3)
        np.testing.assert_allclose(segments[0][1], [1.0, 3.0], atol=1e-12)
        np.testing.assert_allclose(  # Scan 1 untouched
            controls["artists"][1]["rays"].get_segments()[0][0], [0.0, 0.0])

    def test_display_modes_and_hidden_scans(self):
        session = make_test_session(2)
        _, controls = create_session_editor(session)
        scan_artists = controls["artists"][1]

        def shown(name):
            return scan_artists[name].get_visible()

        self.assertEqual(DEFAULT_DISPLAY_MODE, "Points + rays")
        self.assertTrue(shown("points") and shown("rays"))
        controls["actions"]["display_mode"]("Rays only")
        self.assertEqual((shown("points"), shown("rays")), (False, True))
        self.assertTrue(shown("marker") and shown("arrow"))  # origin kept
        controls["actions"]["display_mode"]("Points only")
        self.assertEqual((shown("points"), shown("rays")), (True, False))
        controls["actions"]["display_mode"]("Points + rays")
        controls["actions"]["select"](1)
        controls["actions"]["toggle_visibility"]()  # hide Scan 1
        self.assertFalse(shown("rays") or shown("points") or shown("marker"))
        self.assertTrue(controls["artists"][2]["rays"].get_visible())

    def test_ray_opacity(self):
        session = make_test_session(2)
        _, controls = create_session_editor(session)
        self.assertAlmostEqual(controls["artists"][1]["rays"].get_alpha(),
                               DEFAULT_RAY_OPACITY)
        controls["opacity_slider"].set_val(0.3)
        for scan_id in (1, 2):
            self.assertAlmostEqual(
                controls["artists"][scan_id]["rays"].get_alpha(), 0.3)

    def test_one_line_collection_per_scan(self):
        session = make_test_session(12)
        figure, controls = create_session_editor(session)
        rays = [controls["artists"][scan["id"]]["rays"]
                for scan in session["scans"]]
        self.assertTrue(all(isinstance(item, LineCollection) for item in rays))
        self.assertEqual(len(figure.axes[0].lines), 12 + 1)  # pose dots
        self.assertEqual(sum(len(item.get_segments()) for item in rays),
                         sum(len(scan["x_local_m"])
                             for scan in session["scans"]))


if __name__ == "__main__":
    main()
