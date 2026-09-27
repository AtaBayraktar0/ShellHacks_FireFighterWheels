"""Measure camera/USB health; optional serial handshake sends STOP only.

Default mode is synthetic and never opens a serial device. Opening a requested
Uno USB serial port can reset its firmware; this is not a passive electrical test.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import platform
import re
import time

import numpy as np

from rover.camera import DemoCamera, RealSenseCamera
from rover.serial_link import SerialMotor
from rover.spatial import MIN_DEPTH_M, MAX_DEPTH_M


LIMITS = {"minimum_capture_fps": 10.0, "maximum_frame_gap_s": 0.45,
          "maximum_delivery_age_s": 0.45, "minimum_mean_valid_depth_fraction": 0.85,
          "minimum_frames": 2, "valid_depth_minimum_m": MIN_DEPTH_M,
          "valid_depth_maximum_m": MAX_DEPTH_M}


def positive_duration(value, maximum=60.0):
    value = float(value)
    if not math.isfinite(value) or not 0 < value <= maximum:
        raise ValueError(f"Duration must be greater than zero and at most {maximum:g} seconds.")
    return value


def safe_error(stage, exc):
    """Do not put device paths, exception messages, hostnames or tokens in reports."""
    message = str(exc).lower()
    if "unverified or disabled" in message:
        code = "firmware_profile_not_verified"
    elif isinstance(exc, KeyboardInterrupt):
        code = "interrupted"
    elif isinstance(exc, (ImportError, ModuleNotFoundError)) or "not installed" in message:
        code = "dependency_unavailable"
    elif "estop" in message or "emergency stop" in message:
        code = "estop_latched"
    elif "timed out" in message or "timeout" in message:
        code = "device_timeout"
    elif "timestamp is invalid or repeated" in message:
        code = "invalid_capture_timestamp"
    elif "frame shape mismatch" in message:
        code = "invalid_frame_shape"
    elif isinstance(exc, OSError):
        code = "device_io_error"
    else:
        code = "operation_failed"
    # The exception type is an allowlisted category, never arbitrary device text.
    return {"stage": stage, "code": code}


def public_usb_info(system=None, sysfs_root=Path("/sys/bus/usb/devices")):
    """Public product IDs/link speeds only; deliberately never read serial files."""
    if (system or platform.system()) != "Linux":
        return {"status": "unavailable", "reason": "Linux sysfs USB probe only", "devices": []}
    devices = []
    try:
        candidates = list(Path(sysfs_root).iterdir())
    except OSError:
        return {"status": "unavailable", "reason": "USB sysfs unavailable", "devices": []}
    for entry in candidates:
        try:
            vendor = (entry / "idVendor").read_text().strip().lower()
            product_id = (entry / "idProduct").read_text().strip().lower()
            product = (entry / "product").read_text().strip().lower()
            if not (re.fullmatch(r"[0-9a-f]{4}", vendor) and re.fullmatch(r"[0-9a-f]{4}", product_id)):
                continue
            camera = "realsense" in product
            serial_adapter = "arduino" in product or ("serial" in product and vendor in {"1a86", "0403", "2341", "2a03"})
            if not camera and not serial_adapter:
                continue
            speed = float((entry / "speed").read_text().strip())
            devices.append({"category": "realsense_camera" if camera else "serial_adapter",
                            "vendor_id": vendor, "product_id": product_id,
                            "link_speed_mbps": speed if math.isfinite(speed) and speed >= 0 else None})
        except (OSError, ValueError):
            continue
    return {"status": "observed" if devices else "unavailable",
            "reason": "Recognized USB product names only; absence is not proof of disconnection",
            "devices": devices}


def public_environment():
    return {"os": platform.system(), "architecture": platform.machine(),
            "python": platform.python_version(), "usb": public_usb_info()}


def default_report_path(prefix):
    return f"captures/{prefix}-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')}.json"


def write_report(report, output):
    """Keep requested report paths relative to the project's captures directory."""
    path = Path(output)
    if path.is_absolute() or not path.parts or path.parts[0] != "captures" or ".." in path.parts or path.suffix.lower() != ".json":
        raise ValueError("Output must be a relative JSON path under captures/.")
    root = (Path.cwd() / "captures").resolve()
    target = path.resolve()
    if not target.is_relative_to(root):
        raise ValueError("Report path escapes captures/.")
    path.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation prevents overwriting a previous acceptance record.
    with path.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return path


def _serial_evidence(motor):
    state = motor.status()
    profile = state.get("profile")
    return {"connected": bool(state.get("connected")),
            "motors_enabled": bool(state.get("motors_enabled")),
            "estop": bool(state.get("estop")),
            "profile": profile if profile in {"V4", "L298N", "SIMULATION"} else None}


def run_check(mode="demo", duration=10.0, serial_port=None, *, camera=None,
              motor=None, clock=time.monotonic, sleeper=time.sleep, environment=None):
    """Return report; injected devices/time keep unit tests entirely off hardware."""
    duration = positive_duration(duration)
    if mode not in {"demo", "hardware"}:
        raise ValueError("Mode must be demo or hardware.")
    if mode == "demo" and (serial_port is not None or motor is not None):
        raise ValueError("Demo mode cannot open a serial device; use hardware mode explicitly.")
    if motor is not None and serial_port is None:
        raise ValueError("An explicit serial port is required for a serial check.")
    report = {"schema_version": 1, "suite": "warm-wheels-hardware-bench",
              "created_utc": datetime.now(timezone.utc).isoformat(), "mode": mode,
              "scope": "camera_and_serial_stop_only" if serial_port else "camera_only",
              "duration_requested_s": duration, "limits": dict(LIMITS),
              "environment": environment if environment is not None else public_environment(),
              "checks": {}, "errors": [], "metrics": {}, "verdict": "fail",
              "limitations": ["Synthetic data cannot validate hardware." if mode == "demo" else "This run does not command motion.",
                              "Capture timestamps measure host delivery, not calibrated sensor exposure time.",
                              "Software checks do not establish physical rotation, braking or safe navigation.",
                              "Operating-system scheduling and device timeouts can exceed requested duration."]}
    checks, errors = report["checks"], report["errors"]
    camera = camera if camera is not None else (DemoCamera() if mode == "demo" else RealSenseCamera())
    motor_obj = None
    timestamps, delivery_ages, valid_ratios = [], [], []
    capture_start = None
    last_sensor_stamp = None
    overall_start = clock()
    try:
        if serial_port:
            try:
                motor_obj = motor if motor is not None else SerialMotor(serial_port)
                motor_obj.connect()
                motor_obj.stop()
                evidence = _serial_evidence(motor_obj)
                if not evidence["connected"] or not evidence["motors_enabled"]:
                    raise RuntimeError("Motor transport did not confirm connection.")
                checks["serial"] = {"status": "blocked" if evidence["estop"] else "pass",
                                    "evidence": evidence, "drive_commands_sent": 0,
                                    "note": "Existing emergency latch was preserved" if evidence["estop"] else "WN1 handshake and STOP acknowledged"}
            except Exception as exc:
                issue = safe_error("serial_handshake", exc)
                blocked = issue["code"] in {"firmware_profile_not_verified", "estop_latched"}
                checks["serial"] = {"status": "blocked" if blocked else "fail",
                                    "code": issue["code"], "drive_commands_sent": 0,
                                    "expected_during_unverified_commissioning": blocked}
                errors.append(issue)
        else:
            checks["serial"] = {"status": "skipped", "reason": "No serial port requested", "drive_commands_sent": 0}
        stage = "camera_start"
        try:
            camera.start()
            capture_start = clock()
            stage = "camera_capture"
            while clock() - capture_start < duration:
                frame = camera.read()
                delivered = clock()
                depth = np.asarray(frame.depth)
                rgb = np.asarray(frame.rgb)
                if depth.ndim != 2 or depth.size == 0 or rgb.ndim != 3 or rgb.shape[:2] != depth.shape or rgb.shape[2] != 3:
                    raise ValueError("RGB/depth frame shape mismatch.")
                stamp = float(frame.captured_at)
                age = delivered - stamp
                if not math.isfinite(stamp) or age < -0.01 or (last_sensor_stamp is not None and stamp <= last_sensor_stamp):
                    raise ValueError("Capture timestamp is invalid or repeated.")
                last_sensor_stamp = stamp
                timestamps.append(delivered)
                delivery_ages.append(max(0.0, age))
                valid_ratios.append(float(np.count_nonzero(np.isfinite(depth) & (depth >= MIN_DEPTH_M) & (depth <= MAX_DEPTH_M)) / depth.size))
                if mode == "demo":
                    sleeper(min(1.0 / 15.0, max(0.0, duration - (clock() - capture_start))))
            checks["camera_stream"] = {"status": "pass", "evidence": "Aligned RGB/depth pairs received"}
        except (Exception, KeyboardInterrupt) as exc:
            issue = safe_error(stage, exc)
            errors.append(issue)
            checks["camera_stream"] = {"status": "fail", "code": issue["code"]}
    except KeyboardInterrupt as exc:
        errors.append(safe_error("bench", exc))
        checks["interruption"] = {"status": "fail", "code": "interrupted"}
    finally:
        try:
            camera.close()
            checks["camera_cleanup"] = {"status": "pass"}
        except Exception as exc:
            errors.append(safe_error("camera_cleanup", exc))
            checks["camera_cleanup"] = {"status": "fail"}
        if motor_obj is not None:
            try:
                if motor_obj.status().get("connected"):
                    motor_obj.stop()
                checks["serial_cleanup"] = {"status": "pass"}
            except Exception as exc:
                errors.append(safe_error("serial_stop", exc))
                checks["serial_cleanup"] = {"status": "fail"}
            finally:
                try:
                    motor_obj.close()
                except Exception as exc:
                    errors.append(safe_error("serial_close", exc))
                    checks["serial_cleanup"] = {"status": "fail"}
    gaps = [b - a for a, b in zip(timestamps, timestamps[1:])]
    fps = (len(timestamps) - 1) / (timestamps[-1] - timestamps[0]) if len(timestamps) > 1 and timestamps[-1] > timestamps[0] else 0.0
    metrics = {"frames": len(timestamps), "capture_fps": round(fps, 3),
               "maximum_frame_gap_s": round(max(gaps), 4) if gaps else None,
               "maximum_delivery_age_s": round(max(delivery_ages), 4) if delivery_ages else None,
               "mean_valid_depth_fraction": round(float(np.mean(valid_ratios)), 4) if valid_ratios else None,
               "minimum_valid_depth_fraction": round(min(valid_ratios), 4) if valid_ratios else None,
               "elapsed_s": round(clock() - overall_start, 4)}
    report["metrics"] = metrics
    criteria = {
        "minimum_frames": (metrics["frames"] >= LIMITS["minimum_frames"], metrics["frames"]),
        "capture_fps": (fps >= LIMITS["minimum_capture_fps"], metrics["capture_fps"]),
        "frame_gap": (bool(gaps) and max(gaps) <= LIMITS["maximum_frame_gap_s"], metrics["maximum_frame_gap_s"]),
        "delivery_age": (bool(delivery_ages) and max(delivery_ages) <= LIMITS["maximum_delivery_age_s"], metrics["maximum_delivery_age_s"]),
        "depth_validity": (bool(valid_ratios) and float(np.mean(valid_ratios)) >= LIMITS["minimum_mean_valid_depth_fraction"], metrics["mean_valid_depth_fraction"]),
    }
    for name, (passed, evidence) in criteria.items():
        checks[name] = {"status": "pass" if passed else "fail", "evidence": evidence}
    speeds = [device.get("link_speed_mbps") for device in report["environment"].get("usb", {}).get("devices", [])
              if device.get("category") == "realsense_camera" and device.get("link_speed_mbps") is not None]
    checks["usb_link"] = {"status": "pass" if speeds and min(speeds) >= 5000 else "warning" if speeds else "unavailable",
                          "evidence_mbps": speeds, "note": "USB 3 link target is at least 5000 Mbps; capability alone does not measure stream quality"}
    statuses = {check["status"] for check in checks.values()}
    report["verdict"] = "fail" if "fail" in statuses else "blocked" if "blocked" in statuses else "pass"
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="W.A.R.M wheels camera bench; no motion commands are sent.")
    parser.add_argument("--mode", choices=("demo", "hardware"), default="demo")
    parser.add_argument("--duration", type=float, default=10.0, help="Capture seconds, greater than zero and at most 60")
    parser.add_argument("--serial", help="Optional Uno port: opening may reset firmware; handshake and STOP only")
    parser.add_argument("--output", default=None, help="New JSON file under captures/; existing reports are not overwritten")
    args = parser.parse_args(argv)
    try:
        positive_duration(args.duration)
        if args.mode == "demo" and args.serial:
            parser.error("--serial requires explicit --mode hardware")
        output = args.output or default_report_path("bench")
        # Validate/create the report before opening devices, so a bad output path
        # never runs an unrecorded hardware check. Replace only our reserved file.
        reserved = write_report({"verdict": "incomplete", "schema_version": 1}, output)
    except (ValueError, OSError) as exc:
        parser.error("Invalid or already existing report path, or invalid duration; use a new captures/*.json path.")
    report = run_check(args.mode, args.duration, args.serial)
    with reserved.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(f"{report['verdict'].upper()}: {reserved.as_posix()}")
    return 0 if report["verdict"] == "pass" else 2 if report["verdict"] == "blocked" else 1


if __name__ == "__main__":
    raise SystemExit(main())
