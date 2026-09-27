"""A bounded, discontinuity-aware RGB-D surface mesh; never fabricated SLAM."""
from pathlib import Path
import copy
import math
import threading
import time

import cv2
import numpy as np
from fastapi import HTTPException
from fastapi.responses import FileResponse

from .spatial import MIN_DEPTH_M, MAX_DEPTH_M


def build_mesh(frame, *, step=8, camera_height=.22):
    if type(step) is not int or not 4 <= step <= 32:
        raise ValueError("Mesh stride must be an integer from 4 to 32.")
    if not all(math.isfinite(v) and v > 0 for v in (frame.fx, frame.fy, camera_height)):
        raise ValueError("Invalid camera intrinsics/height.")
    if not all(math.isfinite(v) for v in (frame.cx, frame.cy)):
        raise ValueError("Invalid camera optical centre.")
    depth = np.asarray(frame.depth, dtype=np.float32)
    if depth.ndim != 2 or frame.rgb.shape != (*depth.shape, 3):
        raise ValueError("Aligned color and depth dimensions are required.")
    h, w = depth.shape
    if h < 2 or w < 2 or h > 2160 or w > 4096:
        raise ValueError("Unsupported depth dimensions.")
    valid = np.isfinite(depth) & (depth >= MIN_DEPTH_M) & (depth <= MAX_DEPTH_M)
    rows, cols = np.arange(0, h, step), np.arange(0, w, step)
    yy, xx = np.meshgrid(rows, cols, indexing="ij")
    sampled = depth[yy, xx]
    sampled_valid = valid[yy, xx]
    safe_depth = np.where(sampled_valid, sampled, 0)
    positions = np.stack(((xx - frame.cx) * safe_depth / frame.fx,
                          camera_height - (yy - frame.cy) * safe_depth / frame.fy,
                          -safe_depth), axis=-1)
    colors = frame.rgb[yy, xx, ::-1].astype(np.float32) / 255.
    # Check EVERY source pixel in each coarse quad, including interior holes.
    # A face cannot bridge an invalid return or a large front/back depth jump.
    kernel = np.ones((step + 1, step + 1), dtype=np.uint8)
    block_valid = cv2.erode(valid.astype(np.uint8), kernel, anchor=(0, 0), borderType=cv2.BORDER_CONSTANT, borderValue=0)
    maximum = cv2.dilate(np.where(valid, depth, 0), kernel, anchor=(0, 0))
    minimum = cv2.erode(np.where(valid, depth, MAX_DEPTH_M + 1), kernel, anchor=(0, 0))
    starts = np.ix_(rows[:-1], cols[:-1])
    safe_quads = block_valid[starts].astype(bool)
    safe_quads &= maximum[starts] - minimum[starts] <= np.maximum(.06, .035 * minimum[starts])
    vertex_ids = np.arange(sampled.size, dtype=np.int32).reshape(sampled.shape)
    a, b, c, d = vertex_ids[:-1, :-1], vertex_ids[:-1, 1:], vertex_ids[1:, :-1], vertex_ids[1:, 1:]
    triangles = np.concatenate((np.stack((a, c, b), axis=-1)[safe_quads],
                                np.stack((b, c, d), axis=-1)[safe_quads]), axis=0)
    flat_positions = positions.reshape(-1, 3)
    if len(triangles):
        pts = flat_positions[triangles]
        edge_lengths = np.stack((np.linalg.norm(pts[:, 0] - pts[:, 1], axis=1),
                                 np.linalg.norm(pts[:, 1] - pts[:, 2], axis=1),
                                 np.linalg.norm(pts[:, 2] - pts[:, 0], axis=1)), axis=1)
        triangles = triangles[np.max(edge_lengths, axis=1) <= .15 + .05 * -np.mean(pts[:, :, 2], axis=1)]
    kept = np.flatnonzero(sampled_valid.reshape(-1))
    remap = np.full(sampled.size, -1, dtype=np.int32)
    remap[kept] = np.arange(len(kept))
    compact = flat_positions[kept]
    return {"vertices": np.round(compact, 4).reshape(-1).tolist(),
            "colors": np.round(colors.reshape(-1, 3)[kept].astype(np.float64), 4).reshape(-1).tolist(),
            "indices": remap[triangles].reshape(-1).tolist(),
            "vertex_count": len(kept), "triangle_count": len(triangles),
            "bounds": {"min": compact.min(axis=0).tolist(), "max": compact.max(axis=0).tolist()} if len(compact) else None,
            "camera_position": [0, camera_height, 0], "frame": "camera-local",
            "units": "metres", "stride_px": step, "source_size": [w, h],
            "note": "Single aligned depth frame; no global pose or accumulated house mesh. Invalid depth and discontinuities remain gaps."}


def add_scan_routes(app, runtime, auth):
    cache_lock = threading.Lock()
    cache = {"frame_id": None, "mesh": None, "computed_at": 0.0}

    @app.get("/scan")
    def scan_page():
        return FileResponse(Path(__file__).resolve().parents[1] / "dashboard" / "scan.html")

    @app.get("/api/mesh", dependencies=auth)
    def depth_mesh():
        # Only copy immutable frame references under the motor/runtime lock;
        # meshing runs outside it and is bounded to one update every 0.2 s.
        with runtime.lock:
            if not runtime.camera_ok or not runtime.frames or time.monotonic() - runtime.frame_at > .45:
                raise HTTPException(503, "Fresh depth unavailable; existing mesh must be marked stale.")
            frame_id = runtime.frame_id
            matched = runtime.frames.get(runtime.remote_frame_id)
            if matched is not None and time.monotonic() - matched.captured_at < .3:
                frame_id = runtime.remote_frame_id
            frame = runtime.frames.get(frame_id)
            height = runtime.geometry["camera_height_m"]
            mode = runtime.mode
            observations = []
            for detection in runtime.remote_detections + runtime.local_detections:
                position = detection.get("position_m")
                if detection.get("source_frame_id") != frame_id or position is None:
                    continue
                observations.append({"id": detection["id"], "kind": detection["kind"],
                                     "position": [position[0], position[1] + height, -position[2]],
                                     "confidence": detection["confidence"], "source": detection["source"],
                                     "source_frame_id": frame_id,
                                     "note": "Approximate median depth; may include background. Not a tracked person or verified flame location."})
        if frame is None:
            raise HTTPException(503, "Matched depth frame unavailable")
        with cache_lock:
            now = time.monotonic()
            if cache["mesh"] is None or (frame_id != cache["frame_id"] and now - cache["computed_at"] >= .2):
                mesh = build_mesh(frame, camera_height=height)
                mesh.update({"frame_id": frame_id, "mode": mode, "synthetic": mode == "demo", "captured_at": frame.captured_at,
                             "observations": copy.deepcopy(observations)})
                cache.update({"frame_id": frame_id, "mesh": mesh, "computed_at": now})
            elif frame_id == cache["frame_id"]:
                # Inference may arrive after this frame's geometry was built.
                # Refresh only matching-frame observations; during the geometry
                # throttle, a newer frame's positions must not label an older mesh.
                cache["mesh"]["observations"] = copy.deepcopy(observations)
            result = dict(cache["mesh"])
            result["age_s"] = round(max(0.0, time.monotonic() - result.pop("captured_at")), 3)
            if result["age_s"] > .45:
                raise HTTPException(503, "Depth mesh is stale; wait for a fresh frame")
            return result
