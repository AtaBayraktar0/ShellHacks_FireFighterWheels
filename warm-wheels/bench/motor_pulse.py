"""One explicitly requested, raised-wheel-only pulse; never refresh or clear estop."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import time

from rover.serial_link import MotorLinkError, SerialMotor
from .check_hardware import default_report_path, positive_duration, safe_error, write_report


def validate_pulse(port, left, right, duration, wheels_raised, confirm_pin_check):
    if not port or not isinstance(port, str):
        raise ValueError("An explicit serial port is required.")
    if wheels_raised is not True or confirm_pin_check is not True:
        raise ValueError("Both --wheels-raised and --confirm-pin-check are required.")
    if any(type(value) is not int or not 0 <= value <= 20 for value in (left, right)):
        raise ValueError("Each motor request must be an integer from 0 through 20.")
    return positive_duration(duration, maximum=0.25)


def run_pulse(port, left, right, duration=0.15, *, wheels_raised=False,
              confirm_pin_check=False, motor=None, clock=time.monotonic, sleeper=time.sleep):
    duration = validate_pulse(port, left, right, duration, wheels_raised, confirm_pin_check)
    report = {"schema_version": 1, "suite": "warm-wheels-raised-wheel-pulse",
              "created_utc": datetime.now(timezone.utc).isoformat(), "verdict": "fail",
              "requested": {"left_pwm_percent": left, "right_pwm_percent": right, "duration_s": duration},
              "operator_flags": {"wheels_raised": True, "pin_check_confirmed": True},
              "evidence": {"drive_calls": 0, "drive_acknowledged": False, "stop_acknowledged": False,
                           "port_closed": False, "elapsed_from_drive_s": None},
              "errors": [],
              "limitations": ["Flags record operator attestations, not sensor-verified wheel position or pin wiring.",
                              "A transport pass does not prove rotation direction, motion, or braking.",
                              "Duration is a requested host deadline; OS scheduling is not a hard timing guarantee.",
                              "The Uno 350 ms watchdog is separate; this script sends no refreshes or estop resets."]}
    motor_obj = motor if motor is not None else SerialMotor(port)
    evidence, errors = report["evidence"], report["errors"]
    pulse_start = None
    blocked = False
    stage = "serial_handshake"
    try:
        motor_obj.connect()
        state = motor_obj.status()
        if not state.get("connected") or not state.get("motors_enabled"):
            raise MotorLinkError("Firmware profile is unverified or disabled.")
        if state.get("estop"):
            raise MotorLinkError("Emergency stop is latched; no pulse was attempted.")
        motor_obj.stop()
        stage = "single_drive"
        pulse_start = clock()  # ACK latency counts toward the pulse deadline.
        evidence["drive_calls"] = 1
        motor_obj.drive(left, right)
        evidence["drive_acknowledged"] = True
        stage = "pulse_wait"
        remaining = duration - (clock() - pulse_start)
        if remaining > 0:
            sleeper(remaining)
    except (Exception, KeyboardInterrupt) as exc:
        issue = safe_error(stage, exc)
        errors.append(issue)
        blocked = evidence["drive_calls"] == 0 and issue["code"] in {"firmware_profile_not_verified", "estop_latched"}
    finally:
        try:
            motor_obj.stop()
            evidence["stop_acknowledged"] = True
        except Exception as exc:
            # A rejected, unverified handshake has already closed SerialMotor.
            # Preserve its expected BLOCKED result while retaining stop evidence.
            if not blocked:
                errors.append(safe_error("final_stop", exc))
        finally:
            try:
                motor_obj.close()
                evidence["port_closed"] = True
            except Exception as exc:
                errors.append(safe_error("serial_close", exc))
                blocked = False
        if pulse_start is not None:
            evidence["elapsed_from_drive_s"] = round(clock() - pulse_start, 4)
    if blocked:
        report["verdict"] = "blocked"
    elif not errors and evidence["drive_acknowledged"] and evidence["stop_acknowledged"] and evidence["port_closed"]:
        report["verdict"] = "pass"
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="One raised-wheel pulse, maximum 20% PWM and 0.25 s; no ground driving.")
    parser.add_argument("--port", required=True)
    parser.add_argument("--wheels-raised", action="store_true", required=True)
    parser.add_argument("--confirm-pin-check", action="store_true", required=True)
    parser.add_argument("--left", type=int, required=True)
    parser.add_argument("--right", type=int, required=True)
    parser.add_argument("--duration", type=float, default=0.15)
    parser.add_argument("--output", default=None, help="New JSON file under captures/")
    args = parser.parse_args(argv)
    try:
        validate_pulse(args.port, args.left, args.right, args.duration, args.wheels_raised, args.confirm_pin_check)
        reserved = write_report({"verdict": "incomplete", "schema_version": 1}, args.output or default_report_path("pulse"))
    except (ValueError, OSError):
        parser.error("Use both confirmation flags, PWM integers 0–20, duration (0, 0.25], and a new captures/*.json report path.")
    report = run_pulse(args.port, args.left, args.right, args.duration,
                       wheels_raised=args.wheels_raised, confirm_pin_check=args.confirm_pin_check)
    with reserved.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(f"{report['verdict'].upper()}: {reserved.as_posix()}")
    return 0 if report["verdict"] == "pass" else 2 if report["verdict"] == "blocked" else 1


if __name__ == "__main__":
    raise SystemExit(main())
