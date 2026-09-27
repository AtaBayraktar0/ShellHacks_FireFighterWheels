"""Regression checks for independent runtime motor/camera fault handling."""
import time
import unittest
from unittest.mock import patch

from rover.camera import DemoCamera
from rover.runtime import RoverRuntime
from rover.serial_link import MotorLinkError


class MotorProbe:
    def __init__(self, estop=False):
        self.latched = estop
        self.connected = True
        self.error = None
        self.commands = []
        self.stop_error = None

    def connect(self):
        self.connected = True

    def stop(self):
        self.commands.append((0, 0))
        if self.stop_error:
            raise MotorLinkError(self.stop_error)

    def drive(self, left, right):
        self.commands.append((left, right))

    def estop(self):
        self.stop()
        self.latched = True

    def reset_estop(self):
        self.stop()
        self.latched = False

    def close(self):
        self.connected = False

    def status(self):
        return {"connected": self.connected, "motors_enabled": True,
                "estop": self.latched, "error": self.error,
                "left": 0, "right": 0, "simulated": False}


class CameraProbe:
    def __init__(self):
        self.closed = False
        self.read_callback = None

    def start(self):
        pass

    def read(self):
        return self.read_callback()

    def close(self):
        self.closed = True


class RuntimeSafetyReviewTests(unittest.TestCase):
    def make_runtime(self, latched=False):
        motor, camera = MotorProbe(latched), CameraProbe()
        runtime = RoverRuntime(mode="hardware", enable_motors=True,
                               camera=camera, motor=motor)
        runtime.camera_ok = True
        runtime.frame_at = time.monotonic()
        runtime.spatial = {"clearance_m": 2.0, "depth_valid_fraction": 1.0,
                           "grid": None, "points": []}
        return runtime, motor, camera

    def test_existing_firmware_estop_is_visible_and_blocks_arm(self):
        runtime, _, _ = self.make_runtime(latched=True)
        self.assertTrue(runtime.snapshot()["estop"])
        with self.assertRaisesRegex(ValueError, "[Ss]top|latched"):
            runtime.command("arm")
        self.assertFalse(runtime.gate.armed)
        runtime.command("reset")
        runtime.command("arm")
        self.assertTrue(runtime.gate.armed)

    def test_camera_fault_disarms_and_requests_immediate_motor_stop(self):
        runtime, motor, camera = self.make_runtime()
        runtime.gate.armed = True
        runtime.requested = (20, 20)
        motor.drive(20, 20)

        def fail_read():
            runtime.quit.set()  # Run exactly one capture attempt.
            raise RuntimeError("Camera USB lost")

        camera.read_callback = fail_read
        runtime._capture_loop()
        self.assertFalse(runtime.gate.armed)
        self.assertEqual(runtime.requested, (0, 0))
        self.assertEqual(motor.commands[-1], (0, 0))
        self.assertFalse(runtime.camera_ok)

    def test_fresh_camera_frame_cannot_erase_persistent_motor_fault(self):
        runtime, motor, camera = self.make_runtime()
        motor.connected = False
        motor.error = "USB link lost"
        runtime.error = "Motor fault: USB link lost"
        demo = DemoCamera(width=64, height=48).start()

        def read_one():
            runtime.quit.set()
            return demo.read()

        camera.read_callback = read_one
        with patch("rover.runtime.analyze", return_value=runtime.spatial), \
             patch("rover.runtime.flame_candidates", return_value=[]):
            runtime._capture_loop()
        self.assertTrue(runtime.camera_ok)
        self.assertIn("USB link lost", runtime.snapshot()["error"] or "")

    def test_camera_cleanup_runs_when_motor_stop_fails(self):
        runtime, motor, camera = self.make_runtime()
        motor.stop_error = "Motor link unavailable"
        try:
            runtime.close()
        except MotorLinkError:
            pass  # Reporting the motor failure is compatible with cleanup.
        self.assertTrue(camera.closed)
        self.assertFalse(motor.connected)


if __name__ == "__main__":
    unittest.main()
