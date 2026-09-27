"""Experimental RGB-D odometry and retained surfaces, with NO motor interface.

Pose is relative to the first camera view. No loop closure, global relocalization,
coverage certification, or autonomous navigation is claimed. Tracking failures
latch until a new scan; previously measured surfaces remain available to export.
"""
from collections import deque
import math
import threading
import time

import cv2
import numpy as np
from fastapi import HTTPException
from pydantic import BaseModel
from typing import Literal

from .mesh import build_mesh


class TrackingLost(ValueError):
    pass


def intrinsics(frame):
    return np.array([[frame.fx, 0, frame.cx], [0, frame.fy, frame.cy], [0, 0, 1]], dtype=float)


def estimate_transform(points, pixels, depths, matrix, image_shape):
    """Return previous-camera -> current-camera transform, checked against depth."""
    xyz, uv, z = map(lambda a: np.asarray(a, dtype=float), (points, pixels, depths))
    if len(xyz) < 30 or not all(np.isfinite(a).all() for a in (xyz, uv, z, matrix)):
        raise TrackingLost("Too few reliable RGB/depth matches")
    ok, rvec, tvec, accepted = cv2.solvePnPRansac(
        xyz, uv, matrix, None, iterationsCount=150, reprojectionError=2.0,
        confidence=.999, flags=cv2.SOLVEPNP_EPNP)
    if not ok or accepted is None or len(accepted) < max(25, len(xyz) * .55):
        raise TrackingLost("Image matches do not agree on camera movement")
    ids = accepted.reshape(-1)
    h, w = image_shape
    cells = np.floor(uv[ids] / [w, h] * 3).astype(int).clip(0, 2)
    if len(np.unique(cells, axis=0)) < 4:
        raise TrackingLost("Tracking features are concentrated in too little of the view")
    rvec, tvec = cv2.solvePnPRefineLM(xyz[ids], uv[ids], matrix, None, rvec, tvec)
    rotation = cv2.Rodrigues(rvec)[0]
    predicted = xyz[ids] @ rotation.T + tvec.reshape(3)
    projected = cv2.projectPoints(xyz[ids], rvec, tvec, matrix, None)[0].reshape(-1, 2)
    depth_error = np.abs(predicted[:, 2] - z[ids])
    if np.mean(depth_error < np.maximum(.05, z[ids] * .035)) < .85:
        raise TrackingLost("Depth disagrees with the image-based movement estimate")
    reprojection = float(np.median(np.linalg.norm(projected - uv[ids], axis=1)))
    if reprojection > 1.5 or np.any(predicted[:, 2] <= 0):
        raise TrackingLost("Camera pose verification failed")
    angle = float(np.linalg.norm(rvec))
    if np.linalg.norm(tvec) > .35 or angle > math.radians(30):
        raise TrackingLost("Movement between frames is too large; move more slowly")
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, tvec.reshape(3)
    return transform, {"inliers": len(ids), "matches": len(xyz),
                       "reprojection_px": round(reprojection, 3)}


class RoomMapper:
    MAX_KEYFRAMES = 32

    def __init__(self, camera_height=.22):
        self.camera_height = camera_height
        self.orb = cv2.ORB_create(nfeatures=1200, fastThreshold=15)
        self.status = "idle"
        self.reason = "Start a new scan, then move the camera slowly by hand with motors off"
        self.pose = np.eye(4)
        self.reference = None
        self.patches = []
        self.last_patch_pose = None
        self.frame_id = None
        self.captured_at = None
        self.quality = {}
        self.revision = 0
        self.trajectory = deque(maxlen=1200)

    def fail(self, reason):
        self.status, self.reason = "lost", reason

    def features(self, frame):
        mask = (np.isfinite(frame.depth) & (frame.depth > .25) & (frame.depth < 5)).astype(np.uint8) * 255
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8))
        keys, descriptors = self.orb.detectAndCompute(cv2.cvtColor(frame.rgb, cv2.COLOR_BGR2GRAY), mask)
        if descriptors is None or len(keys) < 40:
            raise TrackingLost("Too few textured surfaces with valid depth; use a well-lit indoor view")
        points = np.array([key.pt for key in keys], dtype=float)
        col, row = np.rint(points).astype(int).T
        depth = frame.depth[row, col]
        xyz = np.c_[(points[:, 0] - frame.cx) * depth / frame.fx,
                    (points[:, 1] - frame.cy) * depth / frame.fy, depth]
        return points, xyz, descriptors

    def process(self, frame, frame_id):
        if self.status not in {"starting", "tracking"}:
            return
        if frame_id == self.frame_id:
            return
        if not math.isfinite(frame.captured_at):
            self.fail("Invalid capture timestamp")
            return
        if self.captured_at is not None and not 0 < frame.captured_at - self.captured_at <= 1.5:
            self.fail("Frame sequence interrupted; start a new scan")
            return
        try:
            matrix = intrinsics(frame)
            if not np.isfinite(matrix).all() or min(frame.fx, frame.fy) <= 0:
                raise TrackingLost("Invalid camera calibration")
            current = self.features(frame)
            if self.reference is not None:
                if frame.depth.shape != self.shape or not np.allclose(matrix, self.matrix):
                    raise TrackingLost("Camera calibration changed during the scan")
                previous_uv, previous_xyz, previous_des = self.reference
                pairs = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(previous_des, current[2])
                pairs = [p for p in pairs if p.distance < 55]
                xyz = [previous_xyz[p.queryIdx] for p in pairs]
                pixels = [current[0][p.trainIdx] for p in pairs]
                depths = [current[1][p.trainIdx, 2] for p in pairs]
                delta, self.quality = estimate_transform(xyz, pixels, depths, matrix, frame.depth.shape)
                self.pose = self.pose @ np.linalg.inv(delta)
            self.reference, self.matrix, self.shape = current, matrix, frame.depth.shape
            self.frame_id, self.captured_at = frame_id, frame.captured_at
            self.status, self.reason = "tracking", "Experimental visual odometry; drift and moving objects can corrupt alignment"
            self.trajectory.append(self.viewer_pose()[:3, 3].tolist())
            relative = np.linalg.inv(self.last_patch_pose) @ self.pose if self.last_patch_pose is not None else None
            if relative is None or np.linalg.norm(relative[:3, 3]) >= .1 or np.linalg.norm(cv2.Rodrigues(relative[:3, :3])[0]) >= .1:
                self.add_patch(frame)
        except (TrackingLost, cv2.error, np.linalg.LinAlgError) as exc:
            self.fail(str(exc))

    def viewer_pose(self):
        # OpenCV right/down/forward -> viewer right/up/backward, with an assumed
        # level first camera pose at the user-configured mounting height.
        flip = np.diag([1., -1., -1., 1.])
        transform = flip @ self.pose @ flip
        transform[1, 3] += self.camera_height
        return transform

    def add_patch(self, frame):
        if len(self.patches) >= self.MAX_KEYFRAMES:
            self.status, self.reason = "capacity", "Scan storage limit reached; export before starting a new scan"
            return
        patch = build_mesh(frame, step=16, camera_height=self.camera_height)
        positions = np.asarray(patch["vertices"]).reshape(-1, 3)
        positions[:, 1] -= self.camera_height
        transform = self.viewer_pose()
        positions = positions @ transform[:3, :3].T + transform[:3, 3]
        patch["vertices"] = np.round(positions, 4).reshape(-1).tolist()
        self.patches.append(patch)
        self.last_patch_pose = self.pose.copy()
        self.revision += 1

    def snapshot(self):
        age = max(0., time.monotonic() - self.captured_at) if self.captured_at is not None else None
        vertices, colors, indices = [], [], []
        for patch in self.patches:
            offset = len(vertices) // 3
            vertices.extend(patch["vertices"])
            colors.extend(patch["colors"])
            indices.extend(i + offset for i in patch["indices"])
        points = np.asarray(vertices).reshape(-1, 3)
        return {"vertices": vertices, "colors": colors, "indices": indices,
                "vertex_count": len(points), "triangle_count": len(indices) // 3,
                "bounds": {"min": points.min(axis=0).tolist(), "max": points.max(axis=0).tolist()} if len(points) else None,
                "frame": "scan-origin", "units": "metres", "frame_id": self.frame_id,
                "revision": self.revision, "age_s": age, "tracking": self.status,
                "reason": self.reason, "quality": self.quality, "keyframes": len(self.patches),
                "capacity": self.MAX_KEYFRAMES, "camera_position": self.viewer_pose()[:3, 3].tolist(),
                "camera_rotation": self.viewer_pose()[:3, :3].tolist(), "trajectory": list(self.trajectory),
                "observations": [], "autonomous_ready": False,
                "note": "Retained RGB-D keyframes using experimental visual odometry. No loop closure or complete-room guarantee. Motor navigation is not implemented."}


class MappingService:
    def __init__(self, runtime):
        self.runtime = runtime
        self.lock = threading.RLock()
        self.mapper = RoomMapper(runtime.geometry["camera_height_m"])
        self.session = 0
        self.thread = None
        self.stop_event = threading.Event()

    def command(self, action):
        if action == "start":
            with self.runtime.lock:
                if self.runtime.mode != "hardware":
                    raise ValueError("Retained room scanning requires the real camera")
                if self.runtime.gate.armed or self.runtime.gate.motion_enabled:
                    raise ValueError("Use camera-only mode for experimental mapping")
                if not self.runtime.camera_ok or time.monotonic() - self.runtime.frame_at > .45:
                    raise ValueError("A fresh camera stream is required")
            with self.lock:
                self.session += 1
                self.mapper = RoomMapper(self.runtime.geometry["camera_height_m"])
                self.mapper.status = "starting"
                if self.thread is None:
                    self.thread = threading.Thread(target=self.run, daemon=True, name="room-mapping")
                    self.thread.start()
        else:
            with self.lock:
                self.mapper.status, self.mapper.reason = "stopped", "Scan stopped; retained surfaces are available to export"
        return {"ok": True}

    def run(self):
        while not self.stop_event.is_set() and not self.runtime.quit.is_set():
            with self.runtime.lock:
                frame_id = self.runtime.frame_id
                frame = self.runtime.frames.get(frame_id)
                fresh = self.runtime.camera_ok and frame is not None and time.monotonic() - frame.captured_at <= .45
            with self.lock:
                if self.mapper.status in {"starting", "tracking"}:
                    if fresh:
                        try:
                            self.mapper.process(frame, frame_id)
                        except Exception as exc:
                            self.mapper.fail(f"Mapping failed ({type(exc).__name__}); retained scan frozen")
                    else:
                        self.mapper.fail("Live camera stream interrupted; retained scan frozen")
            self.stop_event.wait(.2)

    def snapshot(self):
        with self.lock:
            result = self.mapper.snapshot()
            result.update(session=self.session, mode=self.runtime.mode, synthetic=self.runtime.mode == "demo")
            return result

    def close(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2)


class MappingCommand(BaseModel):
    action: Literal["start", "stop"]


def add_mapping_routes(app, runtime, auth):
    service = MappingService(runtime)
    app.state.mapping_service = service

    @app.get("/api/room-map", dependencies=auth)
    def room_map():
        return service.snapshot()

    @app.post("/api/room-map-control", dependencies=auth)
    def room_map_control(body: MappingCommand):
        try:
            return service.command(body.action)
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc
