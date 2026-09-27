"""Weak visual hints only: no medical state, age, or verified fire claims."""
from __future__ import annotations

import math

import cv2
import numpy as np

from .camera import Frame


def attach_depth(detections: list[dict], frame: Frame) -> list[dict]:
    """Return sanitized copies with median central-box depth in metres.

    Box depth is an approximation: background can contaminate it. Missing or
    sparse depth produces None. Aligned depth is essential for this operation.
    """
    height, width = frame.depth.shape
    results = []
    for detection in detections:
        if not isinstance(detection, dict):
            continue
        box = detection.get("box")
        try:
            if len(box) != 4 or not all(math.isfinite(float(x)) for x in box):
                continue
            x1, y1, x2, y2 = [int(round(float(x))) for x in box]
            x1, x2 = max(0, min(width, x1)), max(0, min(width, x2))
            y1, y2 = max(0, min(height, y1)), max(0, min(height, y2))
            confidence = float(detection.get("confidence", 0.0))
        except (TypeError, ValueError, OverflowError):
            continue
        if x2 <= x1 or y2 <= y1 or not math.isfinite(confidence):
            continue
        dx, dy = int((x2 - x1) * 0.25), int((y2 - y1) * 0.25)
        values = frame.depth[y1 + dy:y2 - dy, x1 + dx:x2 - dx]
        valid = values[np.isfinite(values) & (values >= 0.1) & (values <= 6.0)]
        distance = None
        if valid.size >= 5 and valid.size / max(1, values.size) >= 0.5:
            distance = round(float(np.median(valid)), 3)
        result = {
            "kind": str(detection.get("kind", "unknown"))[:80],
            "confidence": round(max(0.0, min(1.0, confidence)), 3),
            "box": [x1, y1, x2, y2],
            "source": str(detection.get("source", "unspecified"))[:160],
            "distance_m": distance,
        }
        result["position_m"] = None if distance is None else [
            round((((x1+x2)/2)-frame.cx)*distance/frame.fx, 3),
            round((frame.cy-((y1+y2)/2))*distance/frame.fy, 3), distance]
        if detection.get("posture_hint"):
            result["posture_hint"] = str(detection["posture_hint"])[:120]
        # Pose is an observation, never an age, consciousness, or ability assessment.
        if detection.get("posture") in ("upright", "low_or_horizontal", "unknown"):
            result["posture"] = detection["posture"]
        results.append(result)
    return results


def flame_candidates(frame: Frame) -> list[dict]:
    """Find orange/red saturated blobs, which may simply be ordinary objects.

    Confidence is heuristic visual strength, not a calibrated probability of
    fire. A trained model plus independent sensing would be needed to improve
    reliability. Never use these candidates to declare a safe fire exit.
    """
    hsv = cv2.cvtColor(frame.rgb, cv2.COLOR_BGR2HSV)
    # HSV hue is 0..179 in OpenCV. Include wraparound deep red.
    warm = ((hsv[..., 0] <= 35) | (hsv[..., 0] >= 174))
    mask = (warm & (hsv[..., 1] >= 125) & (hsv[..., 2] >= 170)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = max(30, frame.depth.size * 0.0004)
    candidates = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:12]:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        candidates.append({
            "kind": "fire_candidate", "confidence": 0.35,
            "box": [x, y, x + w, y + h],
            "source": "color heuristic — unverified",
        })
    return attach_depth(candidates, frame)
