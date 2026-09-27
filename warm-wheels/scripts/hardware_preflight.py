"""Enumerate devices without opening serial ports or starting camera streams."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
UNO_USB_IDS = {(0x2341, 0x0043), (0x2341, 0x0001), (0x2A03, 0x0043),
               (0x1A86, 0x7523), (0x1A86, 0x5523), (0x1A86, 0x7522)}
REQUIRED_MODULES = {"fastapi": "fastapi", "uvicorn": "uvicorn", "numpy": "numpy",
                    "opencv-python-headless": "cv2", "pyserial": "serial",
                    "httpx": "httpx", "pyrealsense2": "pyrealsense2"}


def enumerate_serial_ports(port_provider=None):
    try:
        if port_provider is None:
            from serial.tools import list_ports
            port_provider = list_ports.comports
        ports = []
        for item in port_provider():
            label = str(getattr(item, "description", "Unknown serial device"))
            vid, pid = getattr(item, "vid", None), getattr(item, "pid", None)
            bluetooth = "bluetooth" in label.lower() or "BTHENUM" in str(getattr(item, "hwid", "")).upper()
            kind = "bluetooth" if bluetooth else "usb_serial" if vid is not None else "other"
            ports.append({"port": str(item.device), "description": label,
                          "vid": f"{vid:04X}" if isinstance(vid, int) else None,
                          "pid": f"{pid:04X}" if isinstance(pid, int) else None,
                          "kind": kind, "uno_candidate": not bluetooth and (vid, pid) in UNO_USB_IDS})
        return {"status": "ok", "ports": sorted(ports, key=lambda item: item["port"])}
    except ImportError:
        return {"status": "missing_dependency", "ports": []}
    except Exception:
        return {"status": "error", "ports": []}


def enumerate_realsense(rs=None):
    try:
        rs = rs if rs is not None else importlib.import_module("pyrealsense2")
        devices = []
        for device in rs.context().query_devices():
            values = {}
            for field, info in (("name", rs.camera_info.name),
                                ("usb_connection", rs.camera_info.usb_type_descriptor),
                                ("firmware", rs.camera_info.firmware_version)):
                try:
                    values[field] = device.get_info(info) if device.supports(info) else None
                except Exception:
                    values[field] = None
            devices.append(values)
        return {"status": "connected" if devices else "not_connected", "devices": devices}
    except ImportError:
        return {"status": "missing_dependency", "devices": []}
    except Exception:
        return {"status": "error", "devices": []}


def find_arduino_cli(explicit=None):
    if explicit is not None:
        path = Path(explicit)
        return path if path.is_file() else None
    found = shutil.which("arduino-cli")
    if found:
        return Path(found)
    candidates = []
    for directory in (os.environ.get("ProgramFiles"), os.environ.get("ProgramFiles(x86)")):
        if directory:
            candidates.append(Path(directory) / "Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe")
    if os.environ.get("LOCALAPPDATA"):
        candidates.append(Path(os.environ["LOCALAPPDATA"]) / "Programs/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe")
    return next((path for path in candidates if path.is_file()), None)


def arduino_readiness(cli=None, runner=subprocess.run):
    cli = Path(cli) if cli else find_arduino_cli()
    if cli is None:
        return {"status": "missing_cli", "version": None, "uno_core": None}
    try:
        version_run = runner([str(cli), "version"], capture_output=True, text=True, timeout=15, check=False)
        core_run = runner([str(cli), "core", "list", "--format", "json"], capture_output=True, text=True, timeout=30, check=False)
        if version_run.returncode or core_run.returncode:
            raise RuntimeError("CLI query failed")
        cores = json.loads(core_run.stdout)
        platforms = cores.get("platforms", []) if isinstance(cores, dict) else cores
        avr = next((item.get("installed_version") or item.get("installed")
                    for item in platforms if item.get("id") == "arduino:avr"), None)
        version_match = re.search(r"Version:\s*([^\s]+)", version_run.stdout)
        return {"status": "ready" if avr else "missing_uno_core",
                "version": version_match.group(1) if version_match else "installed", "uno_core": avr}
    except Exception:
        return {"status": "query_failed", "version": None, "uno_core": None}


def package_readiness(importer=importlib.import_module, version_getter=importlib.metadata.version):
    result = {}
    for distribution, module in REQUIRED_MODULES.items():
        try:
            importer(module)
            result[distribution] = {"status": "ready", "version": version_getter(distribution)}
        except Exception:
            result[distribution] = {"status": "missing_or_unusable", "version": None}
    return result


def create_report(*, packages=None, serial=None, realsense=None, arduino=None):
    packages = package_readiness() if packages is None else packages
    serial = enumerate_serial_ports() if serial is None else serial
    realsense = enumerate_realsense() if realsense is None else realsense
    arduino = arduino_readiness() if arduino is None else arduino
    runtime_ready = bool(packages) and all(item["status"] == "ready" for item in packages.values())
    uno_candidates = [item for item in serial["ports"] if item["uno_candidate"]]
    devices_present = bool(uno_candidates) and realsense["status"] == "connected"
    return {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
            "os": platform.system(), "architecture": platform.machine(), "python": platform.python_version(),
            "runtime_ready": runtime_ready, "packages": packages, "serial": serial,
            "realsense": realsense, "arduino": arduino,
            "status": "software_missing" if not runtime_ready else "devices_present_need_bench_check" if devices_present else "software_ready_devices_not_connected",
            "actions_performed": ["Package import checks", "USB serial device enumeration", "RealSense SDK device enumeration", "Arduino CLI/core version checks"],
            "not_performed": ["Serial port opening", "Camera streaming", "Firmware upload", "Motor commands"],
            "note": "A USB adapter ID is only a candidate; inspect the actual ELEGOO Uno before selecting its COM port."}


def print_report(report):
    print("W.A.R.M wheels - laptop hardware preflight")
    print("Python/runtime:", "READY" if report["runtime_ready"] else "DEPENDENCIES NEED ATTENTION")
    for name, item in report["packages"].items():
        print(f"  {name}: {item['version'] or item['status']}")
    print("RealSense:", report["realsense"]["status"].upper().replace("_", " "))
    for device in report["realsense"]["devices"]:
        print(f"  {device['name']} | USB {device['usb_connection']} | firmware {device['firmware']}")
    candidates = [item for item in report["serial"]["ports"] if item["uno_candidate"]]
    print("Uno USB candidate:", "FOUND - inspect board before selecting port" if candidates else "NOT CONNECTED")
    for port in report["serial"]["ports"]:
        print(f"  {port['port']}: {port['description']} [{port['kind']}]")
    if not candidates:
        print("  Bluetooth COM ports are not the Uno. Plug the Uno USB data cable in, then rerun.")
    print(f"Arduino toolchain: {report['arduino']['status']} | CLI {report['arduino']['version']} | Uno AVR {report['arduino']['uno_core']}")
    print("No serial ports opened, camera streams started, firmware uploaded, or motor commands sent.")


def main(argv=None):
    parser = argparse.ArgumentParser(description="List laptop readiness and attached robot devices without opening serial ports.")
    parser.add_argument("--output", default="reports/laptop-preflight.json")
    args = parser.parse_args(argv)
    output = Path(args.output)
    if output.is_absolute() or ".." in output.parts or output.suffix.lower() != ".json":
        parser.error("Use a relative JSON report path inside the project.")
    report = create_report()
    target = ROOT / output
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print_report(report)
    print(f"Readiness report: {output.as_posix()}")
    return 0 if report["runtime_ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
