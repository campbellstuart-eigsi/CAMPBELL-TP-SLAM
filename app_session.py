"""Small data and acquisition adapters for the desktop host; no GUI ownership."""
import hashlib
import importlib
import json
import threading
from pathlib import Path

import numpy as np
import localisation_common as common
from workspace_inputs import WorkspaceInputs

part1 = common.day1("part1")
part2 = common.day1("part2")
part3 = common.day1("part3")
grid_localisation = importlib.import_module("04_grid_localisation")
vector_localisation = importlib.import_module("05_vector_localisation")
scan_localisation = importlib.import_module("06_scan_localisation")


def session_digest(session):
    """Track edits, including exact coordinates, but not the UI selection."""
    data = part1.session_to_json_dict(session)
    for key in ("saved_at", "selected_scan_id"):
        data.pop(key)
    digest = hashlib.sha256(json.dumps(data, sort_keys=True).encode())
    for scan in session["scans"]:
        digest.update(np.column_stack(part1.scan_to_local(scan)).tobytes())
    return digest.hexdigest()


def mapping_scans(session):
    """Adapt the editor's working data without a file reload or coordinate rounding."""
    scans = []
    for scan in session["scans"]:
        if not scan["visible"]:
            continue
        points = np.column_stack(part1.scan_to_local(scan))
        points.setflags(write=False)
        scans.append(dict(id=scan["id"], name=scan["label"],
                          pose=tuple(scan["pose"]),
                          pose_rad=common.pose_from_day1(scan["pose"]),
                          points_local=points, aligned=scan["aligned"],
                          color=scan["color"], quality=None))
    return scans


class SharedSession:
    """One editable session, a saved marker, and revision-scoped derived maps."""
    def __init__(self, path=None):
        self.path = Path(path).resolve() if path else None
        self.source_path = self.path
        self.session = part1.load_session(self.path)[0] if path else part1.create_session()
        self.saved_digest = session_digest(self.session)
        self.revision = 0
        self.digest = self.saved_digest
        self.scans = mapping_scans(self.session)
        self.cache = {}
        self.workspace_inputs = WorkspaceInputs()
        self.measured = {}
        self.measured_path = common.measured_poses_path(self.path) if path else None
        if self.measured_path and self.measured_path.is_file():
            self.measured = common.load_measured_poses(self.measured_path)
        else:
            self.measured_path = None

    @property
    def name(self):
        return self.path.name if self.path else "Untitled session"

    @property
    def dirty(self):
        return self.digest != self.saved_digest

    def refresh(self):
        current = session_digest(self.session)
        if current == self.digest:
            return False
        self.digest = current
        self.revision += 1
        self.scans = mapping_scans(self.session)
        self.cache.clear()
        return True

    def save_copy(self, path):
        self.refresh()
        path = Path(path).resolve()
        if path in (self.path, self.source_path):
            raise ValueError("Choose a different filename to preserve the source session.")
        # Existing writer is atomic and keeps the original sensor-frame points.
        part1.save_session(self.session, path)
        self.saved_digest = session_digest(self.session)
        self.path = path
        return path


def build_grid(scans):
    if not scans:
        raise ValueError("No visible scans. Capture a scan or show scans in LiDAR Scanner.")
    grid, log_odds = grid_localisation.build_grid_map(scans)
    return grid, log_odds


def build_vectors(scans):
    if not scans:
        raise ValueError("No visible scans. Capture a scan or show scans in LiDAR Scanner.")
    scans = [dict(scan, points_global=part2.local_to_global(
        scan["points_local"], scan["pose"])) for scan in scans]
    part3.preprocess_scans(scans)
    clusters = part3.segment_scans(scans)
    records = part3.fit_segments(clusters)
    merged, items = part3.merge_segments(part3.accepted_segments(records))
    return scans, clusters, records, merged, items


class CaptureAdapter:
    """One bounded capture via Code 01's existing ESP32/LD06 serial reader.

    Accepts both library formats (Teleplot XY and CSV); `state` names the
    format arriving, so the status bar shows it during and after a capture.

    Called on a background thread. That thread alone owns the connection;
    cancel/close only set an Event. No bytes or reset commands are transmitted.
    """
    def __init__(self):
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.active = False
        self.state = "Port closed (not probed)"
        self.last_format = None

    def cancel(self):
        self.stop.set()

    def capture(self, port, baudrate, timeout_s=part1.CAPTURE_TIMEOUT_S):
        if not self.lock.acquire(blocking=False):
            raise RuntimeError("A scan capture is already in progress.")
        connection = None
        self.active = True
        try:
            if self.stop.is_set():
                raise RuntimeError("Capture cancelled.")
            self.state = f"Opening {port}"
            connection = part1.open_serial_readonly(port, baudrate)
            self.state = f"Reading {port}"
            connection.reset_input_buffer()

            def show_format(fmt):
                self.last_format = fmt
                self.state = f"Reading {port} · {part1.FORMAT_LABELS[fmt]}"
            scans, report = part1.capture_teleplot_scans(
                connection, 1, timeout_s, stop_event=self.stop,
                on_format=show_format)
            part1.print_capture_report(scans, report, 1)
            if not scans or self.stop.is_set():
                raise RuntimeError(
                    f"No scan captured ({report['stop_reason']}; serial format: "
                    f"{part1.FORMAT_LABELS[report['format']]}). Check USB, COM port, "
                    "baud rate and that the ESP32 prints Teleplot or CSV scans. "
                    "Close other serial monitors.")
            return np.column_stack(part1.teleplot_to_sensor_frame(*scans[0]))
        except part1.serial.SerialException as error:
            raise RuntimeError(f"Cannot read {port}: {error}. Check USB and port; "
                               "close other serial monitors, or use Load Recorded Scan.") from error
        finally:
            try:
                if connection is not None:
                    connection.close()
            finally:
                self.state = ("Port closed" if self.last_format is None else
                              f"Port closed · last format: {part1.FORMAT_LABELS[self.last_format]}")
                self.active = False
                self.lock.release()
