"""D435 frames, flame-prop colors, and a calibrated floor transform."""
from dataclasses import dataclass
import math
import time
import numpy as np


@dataclass(frozen=True)
class Pose:
    """The robot's location and direction."""
    x: float
    y: float
    yaw: float


@dataclass
class Frame:
    """One depth picture and one color picture."""
    xyz: np.ndarray
    bgr: np.ndarray
    captured: float


class Camera:
    """Open and read the RealSense D435 camera."""

    def __enter__(self):
        import pyrealsense2 as rs
        self.rs = rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        # Depth and color use the same picture size.
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.cloud = rs.pointcloud()
        return self

    def read(self):
        """Take a fresh picture and turn depth pixels into 3D points."""
        # Ignore queued frames from before the latest stop.
        for _ in range(3):
            frames = self.pipeline.wait_for_frames(2000)
        # Match every color pixel with the correct depth pixel.
        frames = self.align.process(frames)
        depth, color = frames.get_depth_frame(), frames.get_color_frame()
        if not depth or not color:
            raise RuntimeError("Missing depth or color frame")
        points = self.cloud.calculate(depth)
        xyz = np.asanyarray(points.get_vertices()).view(np.float32)
        xyz = xyz.reshape(depth.get_height(), depth.get_width(), 3).copy()
        return Frame(xyz, np.asanyarray(color.get_data()).copy(), time.monotonic())

    def __exit__(self, *args):
        self.pipeline.stop()


def flame_mask(bgr, min_area=80):
    """Mark large red, orange, and yellow areas."""
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # Red, orange, and yellow are candidates, not proof of real fire.
    mask = cv2.inRange(hsv, (0, 130, 100), (35, 255, 255))
    mask |= cv2.inRange(hsv, (170, 130, 100), (179, 255, 255))
    # Ignore tiny colored dots caused by camera noise.
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    keep = np.zeros(count, dtype=bool)
    keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= min_area
    return keep[labels]


def world_points(xyz, pose, height_m, pitch_deg, forward_m=0.0):
    """Move camera points into the room's map coordinates."""
    right, down, forward = np.moveaxis(xyz, -1, 0)
    pitch = math.radians(pitch_deg)
    body_x = forward * math.cos(pitch) - down * math.sin(pitch) + forward_m
    body_y = -right
    z = height_m - forward * math.sin(pitch) - down * math.cos(pitch)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return np.stack((pose.x + c*body_x - s*body_y,
                     pose.y + s*body_x + c*body_y, z), axis=-1)


def update_map(grid, frame, pose, config, mask=None):
    """Use one camera frame to mark floor and obstacles."""
    xyz = frame.xyz
    valid = np.isfinite(xyz).all(axis=-1) & (xyz[..., 2] >= config["min_depth_m"])
    valid &= xyz[..., 2] <= config["max_depth_m"]
    if valid.sum() < 100:
        raise RuntimeError("Too little valid depth; keep motors stopped")
    points = world_points(xyz, pose, config["camera_height_m"],
                          config["camera_pitch_deg"], config["camera_forward_m"])
    # Low points are floor. Higher points are obstacles.
    floor = valid & (np.abs(points[..., 2]) <= config["floor_tolerance_m"])
    obstacle = valid & (points[..., 2] > config["floor_tolerance_m"])
    if mask is not None:
        if np.any(mask & ~valid):
            raise RuntimeError("Flame-color candidate lacks depth; cannot locate hazard")
        obstacle |= valid & mask
        floor &= ~mask
    def cells(selection):
        return {grid.cell(float(p[0]), float(p[1])) for p in points[selection]}
    # Only observed floor becomes free; holes and drop-offs stay unknown.
    grid.observe(cells(floor), cells(obstacle), frame.captured)
    return points


def save_cloud(path, xyz, bgr):
    """Save the 3D camera points for later viewing."""
    valid = np.isfinite(xyz).all(axis=-1) & (xyz[..., 2] > 0)
    values = np.column_stack((xyz[valid], bgr[valid][:, ::-1]))
    with open(path, "w", encoding="ascii") as out:
        out.write("ply\nformat ascii 1.0\nelement vertex %d\n" % len(values))
        out.write("property float x\nproperty float y\nproperty float z\n")
        out.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        np.savetxt(out, values, fmt="%.5f %.5f %.5f %d %d %d")
