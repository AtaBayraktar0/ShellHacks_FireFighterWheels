"""Aligned RGB/depth capture. Demo data never silently substitutes for hardware."""
from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np


@dataclass
class Frame:
    rgb: np.ndarray  # OpenCV BGR, uint8 HxWx3
    depth: np.ndarray  # Metres, float32 HxW, aligned to RGB; zero means unknown
    fx: float
    fy: float
    cx: float
    cy: float
    captured_at: float  # time.monotonic() at acquisition


class RealSenseCamera:
    """D435i stream with original depth holes preserved for conservative stopping."""

    def __init__(self, width: int = 640, height: int = 480, fps: int = 15,
                 timeout_ms: int = 1500):
        self.width, self.height, self.fps = width, height, fps
        self.timeout_ms = max(1, min(int(timeout_ms), 5000))
        self._pipeline = None
        self._align = None
        self._scale = None

    def start(self):
        if self._pipeline is not None:
            return self
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError(
                "pyrealsense2 is not installed for this Pi/Python combination. "
                "Install a compatible librealsense Python binding; no demo fallback is used."
            ) from exc
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        profile = pipeline.start(config)
        try:
            self._scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
            self._align = rs.align(rs.stream.color)
        except Exception:
            pipeline.stop()
            raise
        self._pipeline = pipeline
        return self

    def read(self) -> Frame:
        if self._pipeline is None:
            raise RuntimeError("Call RealSenseCamera.start() before read().")
        frames = self._pipeline.wait_for_frames(self.timeout_ms)
        captured_at = time.monotonic()
        frames = self._align.process(frames)
        depth_frame, color_frame = frames.get_depth_frame(), frames.get_color_frame()
        if not depth_frame or not color_frame:
            raise RuntimeError("RealSense returned an incomplete aligned RGB/depth pair.")
        intr = color_frame.profile.as_video_stream_profile().get_intrinsics()
        return Frame(
            rgb=np.asanyarray(color_frame.get_data()).copy(),
            depth=np.asanyarray(depth_frame.get_data()).astype(np.float32) * self._scale,
            fx=float(intr.fx), fy=float(intr.fy), cx=float(intr.ppx), cy=float(intr.ppy),
            captured_at=captured_at,
        )

    def close(self):
        pipeline, self._pipeline = self._pipeline, None
        if pipeline is not None:
            pipeline.stop()


class DemoCamera:
    """Static, explicitly synthetic room. Shapes are not neural-model predictions.

    This simulates a level camera 0.22 m above a flat floor, a back wall, a
    stylized person, and a flame-shaped orange object. Robot commands do not
    alter this scene or produce an odometry estimate.
    """

    def __init__(self, width: int = 640, height: int = 480, fps: int = 15):
        self.width, self.height, self.fps = width, height, fps
        self._started = False

    def start(self):
        import cv2

        w, h = self.width, self.height
        self._fx = self._fy = w * 0.90
        self._cx, self._cy = w / 2, h / 2
        self._rgb = np.full((h, w, 3), (61, 52, 40), dtype=np.uint8)
        self._depth = np.full((h, w), 4.0, dtype=np.float32)
        rows = np.arange(h, dtype=np.float32)
        floor_z = np.divide(0.22 * self._fy, rows - self._cy,
                            out=np.full(h, 99.0, dtype=np.float32), where=rows > self._cy)
        floor_mask = np.broadcast_to((floor_z < 4.0)[:, None], (h, w))
        self._depth[floor_mask] = np.broadcast_to(floor_z[:, None], (h, w))[floor_mask]
        self._rgb[floor_mask] = (88, 83, 72)
        # Scaled drawing coordinates keep the synthetic scene usable at low resolution.
        def p(x, y):
            return (round(x * w / 640), round(y * h / 480))
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(mask, p(165, 140), max(2, round(w * 21 / 640)), 255, -1)
        cv2.rectangle(mask, p(139, 164), p(190, 252), 255, -1)
        cv2.line(mask, p(147, 249), p(142, 310), 255, max(2, round(w * 17 / 640)))
        cv2.line(mask, p(181, 249), p(189, 310), 255, max(2, round(w * 17 / 640)))
        self._rgb[mask > 0] = (210, 156, 66)
        self._depth[mask > 0] = 1.8
        fire_poly = np.array([p(449, 304), p(438, 270), p(460, 249), p(457, 219),
                              p(480, 241), p(497, 201), p(503, 255), p(523, 280),
                              p(510, 306)], dtype=np.int32)
        mask[:] = 0
        cv2.fillPoly(mask, [fire_poly], 255)
        self._rgb[mask > 0] = (20, 132, 252)
        self._depth[mask > 0] = 2.4
        cv2.putText(self._rgb, "SIMULATED ROOM - NO HARDWARE", p(18, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, max(0.3, w / 1100), (245, 245, 245), 1)
        self._started = True
        return self

    def read(self) -> Frame:
        if not self._started:
            raise RuntimeError("Call DemoCamera.start() before read().")
        return Frame(self._rgb.copy(), self._depth.copy(), self._fx, self._fy,
                     self._cx, self._cy, time.monotonic())

    def close(self):
        self._started = False
