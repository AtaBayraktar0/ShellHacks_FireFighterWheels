"""D435 depth + color -> Observation (floor / obstacle / flame points, robot frame).

The math helpers are plain numpy so they can be tested without a camera.
pyrealsense2 and OpenCV are only imported when a D435Camera is created.
"""

import math

import numpy as np

from .grid import Observation


def camera_to_robot(pts, cfg, yaw):
    """RealSense optical-frame points (x right, y down, z forward), shape (N,3),
    -> robot-frame points (x forward, y left, z up from the floor).

    Applies the mount: pitched down by cam_pitch_deg, panned by `yaw` radians
    (positive = left), lens cam_height_m above the floor, pan axis
    cam_forward_m ahead of the car's center.
    """
    # Optical axes -> forward / left / up, before applying the mount.
    f, l, u = pts[:, 2], -pts[:, 0], -pts[:, 1]
    p = math.radians(cfg.cam_pitch_deg)
    # Undo the downward pitch: rotate about the left axis.
    x = f * math.cos(p) + u * math.sin(p)
    z = -f * math.sin(p) + u * math.cos(p) + cfg.cam_height_m
    # Apply the pan yaw about the vertical axis, then shift to the car's center.
    c, s = math.cos(yaw), math.sin(yaw)
    xr = c * x - s * l + cfg.cam_forward_m
    yr = s * x + c * l
    return np.column_stack([xr, yr, z])


def classify(pts, cfg):
    """Split robot-frame (N,3) points into floor and obstacle (N,2) xy arrays."""
    rng = np.hypot(pts[:, 0], pts[:, 1])
    keep = rng > cfg.robot_radius_m  # ignore the car's own body
    z = pts[:, 2]
    floor = keep & (np.abs(z) < cfg.floor_tol_m)
    obst = keep & (z >= cfg.floor_tol_m) & (z <= cfg.obstacle_max_z_m)
    return pts[floor, :2], pts[obst, :2]


def fit_floor(pts):
    """Fit a plane to optical-frame floor points. Returns (height_m, pitch_deg)
    of the camera above that plane, for filling in cam_height_m / cam_pitch_deg."""
    # The plane normal is the direction of least spread: the last right-singular vector.
    centroid = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - centroid)
    n = vt[2]
    if n[1] > 0:  # make the normal point up (optical -y)
        n = -n
    # Distance from the lens (the origin) to the plane, measured along the normal.
    height = abs(float(n @ centroid))
    # How far the normal tips toward the lens axis gives the downward pitch.
    pitch = math.degrees(math.asin(-n[2]))
    return height, pitch


def flame_mask(bgr, cfg):
    """Boolean mask of pixels matching the flame color, small blobs removed."""
    import cv2

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    # Red wraps around hue 0/180, so the flame color is a union of HSV ranges.
    for lo, hi in cfg.flame_hsv:
        mask |= cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    # Opening removes pixel-sized speckle before blob filtering.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    # Keep only blobs big enough to be the flame prop.
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    keep = np.zeros_like(mask)
    for c in contours:
        if cv2.contourArea(c) >= cfg.flame_min_area_px:
            cv2.drawContours(keep, [c], -1, 255, thickness=cv2.FILLED)
    return keep > 0


class D435Camera:
    def __init__(self, cfg, width=640, height=480, fps=30):
        import pyrealsense2 as rs

        self.cfg = cfg
        self.pipeline = rs.pipeline()
        rc = rs.config()
        rc.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        rc.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        profile = self.pipeline.start(rc)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)  # depth pixels line up with color pixels

        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        # Precompute each subsampled pixel's ray direction; times depth gives a 3D point.
        s = cfg.depth_stride
        v, u = np.mgrid[0:height:s, 0:width:s]
        self._sel = (slice(0, height, s), slice(0, width, s))
        self._rx = (u - intr.ppx) / intr.fx  # ray x per unit depth
        self._ry = (v - intr.ppy) / intr.fy

    def frames(self):
        """Latest aligned (depth_m, bgr) pair as numpy arrays."""
        # Throw away buffered frames so the image shows where the camera points now.
        for _ in range(self.cfg.discard_frames):
            self.pipeline.wait_for_frames()
        fs = self.align.process(self.pipeline.wait_for_frames())
        depth = np.asanyarray(fs.get_depth_frame().get_data()) * self.depth_scale
        color = np.asanyarray(fs.get_color_frame().get_data())
        return depth, color

    def optical_points(self, depth, mask=None):
        """Subsampled optical-frame (N,3) points with valid depth (and mask, if given)."""
        z = depth[self._sel]
        # Drop pixels with no depth (0) or outside the reliable range.
        ok = (z > self.cfg.min_range_m) & (z < self.cfg.max_range_m)
        if mask is not None:
            ok &= mask[self._sel]
        return np.column_stack([self._rx[ok] * z[ok], self._ry[ok] * z[ok], z[ok]])

    def observe(self, yaw=0.0):
        """Take one look with the camera panned `yaw` radians; return robot-frame points."""
        depth, color = self.frames()
        pts = camera_to_robot(self.optical_points(depth), self.cfg, yaw)
        floor, obstacles = classify(pts, self.cfg)
        flame = np.empty((0, 2))
        # Flame pixels become 3D points too; drop any on the floor (reflections).
        fm = flame_mask(color, self.cfg)
        if fm.any():
            fp = camera_to_robot(self.optical_points(depth, fm), self.cfg, yaw)
            flame = fp[fp[:, 2] >= self.cfg.floor_tol_m, :2]
        return Observation(floor=floor, obstacles=obstacles, flame=flame)

    def close(self):
        self.pipeline.stop()
