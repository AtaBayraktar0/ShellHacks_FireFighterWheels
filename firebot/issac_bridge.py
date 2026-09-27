"""Connect Isaac Sim 6.1 data to the robot's shared brain."""
import math
import time
import numpy as np

from .vision import Frame, Pose


def depth_to_xyz(depth, fx, fy, cx, cy):
    """Turn Isaac camera depth into D435-style 3D points."""
    depth = np.asarray(depth, dtype=np.float32)
    rows, cols = np.indices(depth.shape, dtype=np.float32)
    x = (cols - cx) * depth / fx
    y = (rows - cy) * depth / fy
    return np.stack((x, y, depth), axis=-1)


class IsaacCamera:
    """Read color and depth supplied by an Isaac Sim camera."""

    def __init__(self, sensor_reader, intrinsics):
        self.sensor_reader = sensor_reader
        self.fx, self.fy, self.cx, self.cy = intrinsics

    def read(self):
        """Return the same frame shape used by the real D435."""
        rgb, depth = self.sensor_reader()
        rgb = np.asarray(rgb)
        depth = np.asarray(depth)
        if rgb.shape[:2] != depth.shape or rgb.shape[-1] < 3:
            raise RuntimeError("Isaac color and depth sizes do not match")
        if not np.isfinite(depth).any():
            raise RuntimeError("Isaac camera returned no usable depth")
        bgr = rgb[..., :3][..., ::-1].copy()
        xyz = depth_to_xyz(depth, self.fx, self.fy, self.cx, self.cy)
        return Frame(xyz, bgr, time.monotonic())


class IsaacPoseReader:
    """Read the simulated car position and direction."""

    def __init__(self, pose_reader):
        self.pose_reader = pose_reader

    def __call__(self):
        """Convert an Isaac position and quaternion into x, y, yaw."""
        position, quat = self.pose_reader()
        w, x, y, z = (float(value) for value in quat)
        yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
        return Pose(float(position[0]), float(position[1]), yaw)


class IsaacMotors:
    """Send short movement commands to simulated wheels."""

    def __init__(self, velocity_writer, stepper, linear_speed=.20, turn_speed=1.2):
        self.velocity_writer = velocity_writer
        self.stepper = stepper
        self.linear_speed = linear_speed
        self.turn_speed = turn_speed

    def stop(self):
        """Stop both simulated wheels."""
        self.velocity_writer(0.0, 0.0)

    def pulse(self, direction, pwm=70, duration_ms=100):
        """Move in Isaac for a short time, then stop."""
        del pwm  # Isaac uses measured speed instead of Arduino PWM.
        commands = {
            "FWD": (self.linear_speed, 0.0),
            "BACK": (-self.linear_speed, 0.0),
            "LEFT": (0.0, self.turn_speed),
            "RIGHT": (0.0, -self.turn_speed),
        }
        if direction not in commands:
            raise ValueError("Unknown direction")
        try:
            self.velocity_writer(*commands[direction])
            self.stepper(duration_ms / 1000.0)
        finally:
            self.stop()
