"""Compile Uno firmware; upload only to an explicitly matched, identified port."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

from .hardware_preflight import ROOT, arduino_readiness, enumerate_serial_ports, find_arduino_cli

SKETCH = ROOT / "firmware/rover_bridge"
FQBN = "arduino:avr:uno"


def verification_flags(verified, confirm_pin_check, wheels_raised):
    if verified and not (confirm_pin_check and wheels_raised):
        raise ValueError("Verified firmware requires --confirm-pin-check and --wheels-raised.")
    return 1 if verified else 0


def validate_upload_port(port, confirm_port, confirm_uno, inventory):
    if not port or not re.fullmatch(r"COM[1-9][0-9]*", port, re.IGNORECASE):
        raise ValueError("Specify the actual Windows Uno port as --port COMx.")
    normalized = port.upper()
    if not confirm_port or normalized != confirm_port.upper():
        raise ValueError("--confirm-port must repeat the exact --port COMx selected after device inspection.")
    if not confirm_uno:
        raise ValueError("--confirm-uno is required after checking the actual board.")
    matches = [item for item in inventory.get("ports", []) if item["port"].upper() == normalized]
    if inventory.get("status") != "ok" or len(matches) != 1:
        raise ValueError("Selected COM port is not currently present exactly once. Rerun hardware_preflight.cmd.")
    if not matches[0].get("uno_candidate") or matches[0].get("kind") != "usb_serial":
        raise ValueError("Selected port is not a recognized Uno/CH340 USB candidate; Bluetooth ports are rejected.")
    return matches[0]


def compile_command(cli, build_path, output_path, verified):
    return [str(cli), "compile", "--fqbn", FQBN,
            "--build-property", f"compiler.cpp.extra_flags=-DMOTOR_PROFILE=1 -DHARDWARE_VERIFIED={int(verified)}",
            "--build-path", str(build_path), "--output-dir", str(output_path), str(SKETCH)]


def compile_firmware(cli, verified=False, *, runner=subprocess.run):
    output = ROOT / "reports/firmware" / ("verified-v4" if verified else "unverified-v4")
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="warm-wheels-uno-") as build_path:
        command = compile_command(cli, build_path, output, verified)
        result = runner(command, cwd=ROOT, capture_output=True, text=True, timeout=180, check=False)
    log = (result.stdout or "") + (result.stderr or "")
    (output / "build.log").write_text(log, encoding="utf-8")
    manifest = {"created_utc": datetime.now(timezone.utc).isoformat(), "fqbn": FQBN,
                "motor_profile": "ELEGOO_V4", "hardware_verified": bool(verified),
                "compile_succeeded": result.returncode == 0, "uploaded": False,
                "source_sha256": hashlib.sha256((SKETCH / "rover_bridge.ino").read_bytes()).hexdigest(),
                "binaries": {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in output.glob("*.hex")} if result.returncode == 0 else {}}
    (output / "build.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(log.strip())
    if result.returncode:
        raise RuntimeError("Uno compilation failed; see reports/firmware build.log.")
    print(f"Compile successful. Motor profile: V4 / {'VERIFIED' if verified else 'UNVERIFIED (motor output disabled)' }.")
    print(f"Binaries and log: {output.relative_to(ROOT).as_posix()}")
    return output


def upload_firmware(cli, output, port, expected_device, *, runner=subprocess.run, enumerator=enumerate_serial_ports):
    # Recheck after compilation; a removed/replaced COM device cannot silently
    # inherit a previous selection. USB IDs still require the operator's board check.
    current = validate_upload_port(port, port, True, enumerator())
    if any(current.get(field) != expected_device.get(field) for field in ("vid", "pid", "description")):
        raise ValueError("Selected USB device changed during compilation; rerun after inspecting it.")
    result = runner([str(cli), "upload", "--fqbn", FQBN, "--port", port,
                     "--input-dir", str(output), str(SKETCH)], cwd=ROOT,
                    capture_output=True, text=True, timeout=90, check=False)
    log = (result.stdout or "") + (result.stderr or "")
    (output / "upload.log").write_text(log, encoding="utf-8")
    print(log.strip())
    if result.returncode:
        raise RuntimeError("Upload failed; close other serial programs and inspect upload.log.")
    manifest_path = output / "build.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["uploaded"] = True
    manifest["uploaded_utc"] = datetime.now(timezone.utc).isoformat()
    manifest["uploaded_port"] = port
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print("Upload complete. No drive commands were sent. Follow the separate raised-wheel bench checks.")


def main(argv=None):
    parser = argparse.ArgumentParser(description="W.A.R.M wheels Uno compile/upload helper; default firmware cannot drive motors.")
    parser.add_argument("action", choices=("compile", "upload"), nargs="?", default="compile")
    parser.add_argument("--verified", action="store_true", help="Enable verified V4 pin profile only after physical checks")
    parser.add_argument("--confirm-pin-check", action="store_true")
    parser.add_argument("--wheels-raised", action="store_true")
    parser.add_argument("--port")
    parser.add_argument("--confirm-port")
    parser.add_argument("--confirm-uno", action="store_true")
    parser.add_argument("--arduino-cli", help="Optional explicit path to Arduino CLI")
    args = parser.parse_args(argv)
    try:
        verification_flags(args.verified, args.confirm_pin_check, args.wheels_raised)
        selected = validate_upload_port(args.port, args.confirm_port, args.confirm_uno, enumerate_serial_ports()) if args.action == "upload" else None
        cli = find_arduino_cli(args.arduino_cli)
        if cli is None:
            raise ValueError("Arduino CLI not found. Install Arduino IDE or supply --arduino-cli.")
        readiness = arduino_readiness(cli)
        if readiness["status"] != "ready":
            raise ValueError("Arduino Uno AVR core is unavailable. Install Arduino AVR Boards in Arduino IDE Board Manager.")
        output = compile_firmware(cli, args.verified)
        if args.action == "upload":
            upload_firmware(cli, output, args.port.upper(), selected)
        return 0
    except (ValueError, RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
        print(f"STOPPED: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
