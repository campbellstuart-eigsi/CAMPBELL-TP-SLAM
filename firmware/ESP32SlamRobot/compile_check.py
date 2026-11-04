"""Reproducible compile check of the ESP32SlamRobot sketch. COMPILES ONLY: never uploads.

Rebuilds the sketch in a temporary folder from pinned, hash-checked inputs:
  - this folder's ESP32SlamRobot.ino and wifi_secrets.example.h;
  - ld06.h / ld06.cpp / ld06crc.h taken from LD06.zip (not in Git: licence unconfirmed);
  - MecanumCar_v2 from MecanumCar_v2.zip (not in Git: no licence);
  - "Async TCP" 3.5.0 and "ESP Async WebServer" 3.12.1 from a libraries folder
    (installed with the Arduino IDE Library Manager);
  - the installed core esp32:esp32 3.3.12, board esp32:esp32:esp32 (ESP32 Dev Module).
Then runs three compiles, each of which must behave as expected:
  1. no wifi_secrets.h          -> fails ("wifi_secrets.h is missing")
  2. the example password       -> fails ("still has the example password")
  3. a throw-away test password -> succeeds
Your real wifi_secrets.h is never read or copied.

Usage (from the worktree, any Python 3.10+):
  python firmware/ESP32SlamRobot/compile_check.py --ld06-zip C:\\Users\\stuar\\Desktop\\SLAM\\LD06.zip
      --mecanum-zip C:\\Users\\stuar\\Downloads\\MecanumCar_v2.zip
      [--libraries C:\\Users\\stuar\\Documents\\Arduino\\libraries] [--keep DIR]
"""
import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
SKETCH = "ESP32SlamRobot"
FQBN = "esp32:esp32:esp32"
CORE = ("esp32:esp32", "3.3.12")
ARDUINO_CLI = (Path(os.environ.get("LOCALAPPDATA", "")) / "Programs" / "Arduino IDE" / "resources"
               / "app" / "lib" / "backend" / "resources" / "arduino-cli.exe")
LD06_ZIP_SHA = "32f928be379d9f8136d94718d61baf0b423206d8e1989998e0e93eb79fc7b529"
LD06_FILES = {  # member of LD06.zip -> SHA-256
    "LD06/src/ld06.h": "c9110d9fb5b40cb0187be431b1375dbdf5139f60affa11a0ba57d131bf571232",
    "LD06/src/ld06.cpp": "d7aa8f062e13875c273b6bf3f0f1a14feaff822c07ec733b473a0d0d97d20561",
    "LD06/src/ld06crc.h": "b7f447cbd2ea17f4b2ccc13b7011cee16006d29ba522610b7ba60812f69f3d15",
}
MECANUM_ZIP_SHA = "4b03d2cedb5a380cc7e26610b45e751ee51d8b4f0aa57c5c4b4f0f65426aaff0"
MECANUM_FILES = ("MecanumCar_v2/MecanumCar_v2.h", "MecanumCar_v2/MecanumCar_v2.cpp")
LIBRARIES = {"Async TCP": "3.5.0", "ESP Async WebServer": "3.12.1"}  # name -> exact version
TEST_SECRETS = ('#pragma once\nconstexpr char ROBOT_SSID[] = "compile-check";\n'
                'constexpr char ROBOT_PASSWORD[] = "compile-check-only";\n')


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def check(condition, message):
    if not condition:
        raise SystemExit(f"FAILED: {message}")


def find_library(folder, name):
    """The library folder under `folder` whose library.properties has name=`name`."""
    for props in Path(folder).glob("*/library.properties"):
        fields = dict(line.split("=", 1) for line in props.read_text(encoding="utf-8",
                      errors="replace").splitlines() if "=" in line)
        if fields.get("name", "").strip() == name:
            return props.parent, fields.get("version", "").strip()
    return None, None


def compile_sketch(cli, sketch_dir, user_dir, build_dir):
    env = dict(os.environ, ARDUINO_DIRECTORIES_USER=str(user_dir))
    result = subprocess.run([str(cli), "compile", "-b", FQBN, "--warnings", "default",
                             "--build-path", str(build_dir), str(sketch_dir)],
                            capture_output=True, text=True, env=env)
    return result.returncode, result.stdout + result.stderr


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ld06-zip", required=True, type=Path)
    parser.add_argument("--mecanum-zip", required=True, type=Path)
    parser.add_argument("--libraries", type=Path,
                        default=Path.home() / "Documents" / "Arduino" / "libraries",
                        help="folder holding the Async TCP and ESP Async WebServer libraries")
    parser.add_argument("--arduino-cli", type=Path, default=ARDUINO_CLI)
    parser.add_argument("--keep", type=Path, help="build here and keep it (default: a temp folder)")
    args = parser.parse_args(argv)

    check(args.arduino_cli.is_file(), f"arduino-cli not found at {args.arduino_cli}")
    cores = json.loads(subprocess.run([str(args.arduino_cli), "core", "list", "--format", "json"],
                                      capture_output=True, text=True).stdout or "{}")
    versions = {p.get("id"): p.get("installed_version") or p.get("installed")
                for p in cores.get("platforms", [])}
    check(versions.get(CORE[0]) == CORE[1],
          f"core {CORE[0]} {CORE[1]} needed, found {versions.get(CORE[0])}")

    ld06 = args.ld06_zip.read_bytes()
    check(sha256(ld06) == LD06_ZIP_SHA, f"{args.ld06_zip} is not the expected LD06.zip")
    mecanum = args.mecanum_zip.read_bytes()
    check(sha256(mecanum) == MECANUM_ZIP_SHA,
          f"{args.mecanum_zip} is not the expected MecanumCar_v2.zip")

    root = args.keep or Path(tempfile.mkdtemp(prefix="esp32slamrobot_"))
    sketch_dir = root / SKETCH
    user_dir = root / "arduino_user"
    shutil.rmtree(sketch_dir, ignore_errors=True)
    shutil.rmtree(user_dir, ignore_errors=True)
    sketch_dir.mkdir(parents=True)
    (user_dir / "libraries").mkdir(parents=True)
    for name in (f"{SKETCH}.ino", "wifi_secrets.example.h"):
        shutil.copy2(HERE / name, sketch_dir / name)
    with zipfile.ZipFile(args.ld06_zip) as archive:          # only the named members
        for member, digest in LD06_FILES.items():
            data = archive.read(member)
            check(sha256(data) == digest, f"{member} in LD06.zip has changed")
            (sketch_dir / Path(member).name).write_bytes(data)
    with zipfile.ZipFile(args.mecanum_zip) as archive:
        target = user_dir / "libraries" / "MecanumCar_v2"
        target.mkdir()
        for member in MECANUM_FILES:
            (target / Path(member).name).write_bytes(archive.read(member))
    for name, version in LIBRARIES.items():
        folder, found = find_library(args.libraries, name)
        check(folder is not None, f'library "{name}" not found in {args.libraries}')
        check(found == version, f'library "{name}" is {found}, {version} is required')
        shutil.copytree(folder, user_dir / "libraries" / folder.name)

    secrets = sketch_dir / "wifi_secrets.h"
    cases = (("no wifi_secrets.h", None, False, "wifi_secrets.h is missing"),
             ("example password", (HERE / "wifi_secrets.example.h").read_text(), False,
              "still has the example password"),
             ("test password", TEST_SECRETS, True, None))
    report = []
    for label, content, should_pass, expected_error in cases:
        if content is None:
            secrets.unlink(missing_ok=True)
        else:
            secrets.write_text(content, encoding="ascii")
        code, output = compile_sketch(args.arduino_cli, sketch_dir, user_dir, root / "build")
        ok = (code == 0) if should_pass else (code != 0 and expected_error in output)
        size = re.search(r"Sketch uses (\d+) bytes \((\d+)%\)", output)
        ram = re.search(r"Global variables use (\d+) bytes \((\d+)%\)", output)
        detail = (f"flash {size.group(1)} B ({size.group(2)} %), RAM {ram.group(1)} B "
                  f"({ram.group(2)} %)" if size and ram else
                  next((line.strip() for line in output.splitlines() if "error" in line), ""))
        report.append((label, "pass" if should_pass else "fail", "OK" if ok else "UNEXPECTED",
                       detail))
        if not ok:
            (root / f"compile_{label.replace(' ', '_')}.log").write_text(output, encoding="utf-8")
    secrets.unlink(missing_ok=True)       # the test password never stays behind
    print(f"Build folder: {root}")
    for label, expected, result, detail in report:
        print(f"  {label:20} expected {expected:4}  {result:10} {detail}")
    if all(result == "OK" for _, _, result, _ in report):
        print("COMPILE CHECK PASSED (compile only; nothing was uploaded; hardware untested)")
        return 0
    print("COMPILE CHECK FAILED: see the compile_*.log files in the build folder")
    return 1


if __name__ == "__main__":
    sys.exit(main())
