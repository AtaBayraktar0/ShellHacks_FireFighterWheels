"""No-device acceptance tests for camera commissioning and bounded pulse tools."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from bench.check_hardware import public_usb_info, run_check, write_report
from bench.motor_pulse import run_pulse, validate_pulse
from rover.camera import Frame
from rover.serial_link import MotorLinkError


ENVIRONMENT = {"os": "Test", "architecture": "test", "python": "test",
               "usb": {"status": "unavailable", "devices": []}}


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeCamera:
    def __init__(self, clock, gap=1 / 15):
        self.clock = clock
        self.gap = gap
        self.depth = np.ones((4, 4), dtype=np.float32)
        self.closed = False
        self.start_error = self.read_error = self.close_error = None
        self.repeated_timestamp = False
        self.reads = 0

    def start(self):
        if self.start_error:
            raise self.start_error

    def read(self):
        if self.read_error:
            raise self.read_error
        self.clock.sleep(self.gap)
        self.reads += 1
        return Frame(np.zeros((4, 4, 3), dtype=np.uint8), self.depth.copy(),
                     3, 3, 2, 2, 100.0 if self.repeated_timestamp else self.clock() - .001)

    def close(self):
        self.closed = True
        if self.close_error:
            raise self.close_error


class FakeMotor:
    def __init__(self, clock):
        self.clock = clock
        self.calls = []
        self.connected = self.closed = self.estop = False
        self.connect_error = self.drive_error = self.stop_error = self.close_error = None
        self.drive_latency = 0.0

    def connect(self):
        self.calls.append(("connect",))
        if self.connect_error:
            raise self.connect_error
        self.connected = True

    def stop(self):
        self.calls.append(("stop",))
        if self.stop_error:
            raise self.stop_error

    def drive(self, left, right):
        self.calls.append(("drive", left, right))
        self.clock.sleep(self.drive_latency)
        if self.drive_error:
            raise self.drive_error

    def close(self):
        self.calls.append(("close",))
        self.closed = True
        self.connected = False
        if self.close_error:
            raise self.close_error

    def status(self):
        return {"connected": self.connected, "motors_enabled": True,
                "profile": "V4", "estop": self.estop,
                "error": "Do not serialize secret=private-token or /home/private-user"}

    def reset_estop(self):
        raise AssertionError("Bench tools must never clear estop automatically")


class HardwareCheckTests(unittest.TestCase):
    def run_fake(self, camera=None, motor=None, duration=.5):
        clock = camera.clock if camera else FakeClock()
        camera = camera or FakeCamera(clock)
        result = run_check("hardware", duration, "private-port-identifier" if motor else None,
                           camera=camera, motor=motor, clock=clock, sleeper=clock.sleep,
                           environment=ENVIRONMENT)
        return result, camera

    def test_nominal_camera_metrics_and_cleanup(self):
        result, camera = self.run_fake()
        self.assertEqual(result["verdict"], "pass")
        self.assertAlmostEqual(result["metrics"]["capture_fps"], 15, places=2)
        self.assertEqual(result["metrics"]["mean_valid_depth_fraction"], 1)
        self.assertEqual(result["checks"]["serial"]["status"], "skipped")
        self.assertTrue(camera.closed)
        json.dumps(result, allow_nan=False)

    def test_serial_check_only_connects_stops_and_closes(self):
        clock = FakeClock()
        motor, camera = FakeMotor(clock), FakeCamera(clock)
        result, _ = self.run_fake(camera, motor)
        self.assertEqual(result["verdict"], "pass")
        self.assertTrue(motor.closed)
        self.assertTrue(all(call[0] in {"connect", "stop", "close"} for call in motor.calls))
        self.assertEqual(result["checks"]["serial"]["drive_commands_sent"], 0)

    def test_unverified_profile_is_expected_blocked_and_camera_still_measured(self):
        clock = FakeClock()
        motor = FakeMotor(clock)
        motor.connect_error = MotorLinkError("Firmware profile is unverified or disabled.")
        result, camera = self.run_fake(FakeCamera(clock), motor)
        self.assertEqual(result["verdict"], "blocked")
        self.assertGreater(result["metrics"]["frames"], 2)
        self.assertTrue(result["checks"]["serial"]["expected_during_unverified_commissioning"])
        self.assertTrue(camera.closed and motor.closed)

    def test_existing_estop_is_preserved(self):
        clock = FakeClock()
        motor = FakeMotor(clock)
        motor.estop = True
        result, _ = self.run_fake(FakeCamera(clock), motor)
        self.assertEqual(result["verdict"], "blocked")
        self.assertTrue(motor.estop)

    def test_camera_start_and_read_failures_have_json_and_cleanup(self):
        for field in ("start_error", "read_error"):
            with self.subTest(field=field):
                clock = FakeClock()
                camera, motor = FakeCamera(clock), FakeMotor(clock)
                setattr(camera, field, OSError("secret=private-token /home/private-user"))
                result, _ = self.run_fake(camera, motor)
                self.assertEqual(result["verdict"], "fail")
                self.assertTrue(camera.closed and motor.closed)
                self.assertEqual(result["errors"][0]["code"], "device_io_error")
                self.assertNotIn("private-token", json.dumps(result))
                self.assertNotIn("private-user", json.dumps(result))
                self.assertNotIn("private-port", json.dumps(result))

    def test_failed_camera_cleanup_does_not_skip_serial_cleanup(self):
        clock = FakeClock()
        camera, motor = FakeCamera(clock), FakeMotor(clock)
        camera.close_error = OSError("failed to stop camera")
        result, _ = self.run_fake(camera, motor)
        self.assertEqual(result["verdict"], "fail")
        self.assertTrue(motor.closed)

    def test_keyboard_interrupt_still_returns_fail_and_cleans_up(self):
        camera = FakeCamera(FakeClock())
        camera.read_error = KeyboardInterrupt()
        result, _ = self.run_fake(camera)
        self.assertEqual(result["verdict"], "fail")
        self.assertTrue(camera.closed)
        self.assertEqual(result["errors"][0]["code"], "interrupted")

    def test_invalid_depth_and_frame_gap_fail_evidence_limits(self):
        camera = FakeCamera(FakeClock(), gap=.5)
        camera.depth[:] = np.nan
        result, _ = self.run_fake(camera, duration=1.5)
        self.assertEqual(result["verdict"], "fail")
        for name in ("capture_fps", "depth_validity", "frame_gap"):
            self.assertEqual(result["checks"][name]["status"], "fail")
        json.dumps(result, allow_nan=False)

    def test_repeated_timestamp_fails_instead_of_claiming_fresh_frames(self):
        camera = FakeCamera(FakeClock())
        camera.repeated_timestamp = True
        result, _ = self.run_fake(camera)
        self.assertEqual(result["verdict"], "fail")
        self.assertEqual(result["checks"]["camera_stream"]["status"], "fail")

    def test_depth_outside_application_range_does_not_count_as_valid(self):
        for depth in (.05, 6.1, float("inf")):
            with self.subTest(depth=depth):
                camera = FakeCamera(FakeClock())
                camera.depth[:] = depth
                result, _ = self.run_fake(camera)
                self.assertEqual(result["metrics"]["mean_valid_depth_fraction"], 0)
                self.assertEqual(result["checks"]["depth_validity"]["status"], "fail")

    def test_duration_and_demo_serial_rejected_before_opening(self):
        motor = FakeMotor(FakeClock())
        for duration in (0, -1, 61, float("nan"), float("inf")):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                run_check("hardware", duration, "port", motor=motor)
        with self.assertRaises(ValueError):
            run_check("demo", .5, "port", motor=motor)
        self.assertEqual(motor.calls, [])

    def test_demo_mode_never_constructs_real_devices(self):
        clock = FakeClock()
        camera = FakeCamera(clock, gap=0)
        # DemoCamera stamps each read after the runner's intentional 15 Hz pace.
        with patch("bench.check_hardware.SerialMotor") as serial_constructor, \
             patch("bench.check_hardware.RealSenseCamera") as camera_constructor:
            report = run_check("demo", .5, camera=camera, clock=clock,
                               sleeper=clock.sleep, environment=ENVIRONMENT)
        self.assertEqual(report["verdict"], "pass")
        serial_constructor.assert_not_called()
        camera_constructor.assert_not_called()

    def test_usb_report_has_public_ids_and_speed_but_no_serial_or_product_text(self):
        with tempfile.TemporaryDirectory() as temporary:
            device = Path(temporary) / "1-1"
            device.mkdir()
            for name, value in {"idVendor": "8086", "idProduct": "0b3a",
                                "speed": "5000", "product": "Intel RealSense D435i private-label",
                                "serial": "private-serial-number"}.items():
                (device / name).write_text(value)
            result = public_usb_info("Linux", Path(temporary))
        self.assertEqual(result["devices"][0]["link_speed_mbps"], 5000)
        self.assertNotIn("private", json.dumps(result))

    def test_report_path_cannot_escape_captures(self):
        with tempfile.TemporaryDirectory() as temporary, patch("pathlib.Path.cwd", return_value=Path(temporary)):
            # Rejection occurs before filesystem creation; no working-directory change.
            for output in ("../report.json", "report.json", "captures/../report.json", "captures/report.txt"):
                with self.subTest(output=output), self.assertRaises(ValueError):
                    write_report({}, output)


class MotorPulseTests(unittest.TestCase):
    def pulse(self, motor, **kwargs):
        options = {"duration": .15, "wheels_raised": True, "confirm_pin_check": True,
                   "motor": motor, "clock": motor.clock, "sleeper": motor.clock.sleep}
        options.update(kwargs)
        return run_pulse("private-port", 15, 0, **options)

    def test_requires_both_attestations_before_motor_use(self):
        motor = FakeMotor(FakeClock())
        for options in ({"wheels_raised": False}, {"confirm_pin_check": False}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.pulse(motor, **options)
        self.assertEqual(motor.calls, [])

    def test_rejects_unsafe_bounds_and_booleans(self):
        for left, right, duration in ((21, 0, .1), (-1, 0, .1), (True, 0, .1),
                                     (1.0, 0, .1), (0, 0, .251), (0, 0, 0),
                                     (0, 0, float("nan")), (0, 0, float("inf"))):
            with self.subTest(values=(left, right, duration)), self.assertRaises(ValueError):
                validate_pulse("port", left, right, duration, True, True)

    def test_single_pulse_stops_and_closes_without_refresh(self):
        motor = FakeMotor(FakeClock())
        report = self.pulse(motor)
        self.assertEqual(report["verdict"], "pass")
        self.assertEqual(motor.calls, [("connect",), ("stop",), ("drive", 15, 0), ("stop",), ("close",)])
        self.assertEqual(report["evidence"]["drive_calls"], 1)
        self.assertAlmostEqual(report["evidence"]["elapsed_from_drive_s"], .15)

    def test_ack_latency_counts_against_requested_pulse_duration(self):
        motor = FakeMotor(FakeClock())
        motor.drive_latency = .10
        report = self.pulse(motor, duration=.15)
        self.assertAlmostEqual(report["evidence"]["elapsed_from_drive_s"], .15)

    def test_ack_later_than_deadline_triggers_stop_without_additional_sleep(self):
        motor = FakeMotor(FakeClock())
        motor.drive_latency = .2
        waits = []
        report = self.pulse(motor, duration=.1, sleeper=waits.append)
        self.assertEqual(waits, [])
        self.assertTrue(report["evidence"]["stop_acknowledged"])
        self.assertEqual(len([call for call in motor.calls if call[0] == "drive"]), 1)

    def test_drive_failure_still_attempts_stop_and_close(self):
        motor = FakeMotor(FakeClock())
        motor.drive_error = MotorLinkError("Motor acknowledgement timed out.")
        report = self.pulse(motor)
        self.assertEqual(report["verdict"], "fail")
        self.assertEqual(motor.calls[-2:], [("stop",), ("close",)])

    def test_interruption_during_wait_still_stops(self):
        motor = FakeMotor(FakeClock())

        def interrupt(_):
            raise KeyboardInterrupt()

        report = self.pulse(motor, sleeper=interrupt)
        self.assertEqual(report["verdict"], "fail")
        self.assertEqual(motor.calls[-2:], [("stop",), ("close",)])

    def test_stop_failure_cannot_skip_close_or_report_pass(self):
        motor = FakeMotor(FakeClock())
        motor.stop_error = OSError("secret=private-token")
        report = self.pulse(motor)
        self.assertEqual(report["verdict"], "fail")
        self.assertTrue(motor.closed)
        self.assertNotIn("private-token", json.dumps(report))

    def test_latched_or_unverified_firmware_never_receives_drive(self):
        for profile_blocked in (False, True):
            with self.subTest(profile_blocked=profile_blocked):
                motor = FakeMotor(FakeClock())
                motor.estop = not profile_blocked
                if profile_blocked:
                    motor.connect_error = MotorLinkError("Firmware profile is unverified or disabled.")
                report = self.pulse(motor)
                self.assertEqual(report["verdict"], "blocked")
                self.assertEqual(report["evidence"]["drive_calls"], 0)
                self.assertFalse(any(call[0] == "drive" for call in motor.calls))


if __name__ == "__main__":
    unittest.main()
