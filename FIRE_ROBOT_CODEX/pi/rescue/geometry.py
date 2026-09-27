"""Angle helpers and the 2D robot pose shared by mapping, planning and the sim."""

import math
from dataclasses import dataclass

import numpy as np


def wrap(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def dist(a, b):
    """Straight-line distance between two (x, y) points."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # radians, CCW from +x

    @property
    def xy(self):
        """Position as an (x, y) tuple."""
        return (self.x, self.y)

    def advance(self, d):
        """Move d meters along the current heading (heading unchanged)."""
        self.x += d * math.cos(self.theta)
        self.y += d * math.sin(self.theta)

    def to_world(self, pts):
        """Robot-frame (N,2) points -> world-frame (N,2) points."""
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        # Rotate by theta, then translate by the robot's position.
        c, s = math.cos(self.theta), math.sin(self.theta)
        x = c * pts[:, 0] - s * pts[:, 1] + self.x
        y = s * pts[:, 0] + c * pts[:, 1] + self.y
        return np.column_stack([x, y])

    def to_robot(self, pts):
        """World-frame (N,2) points -> robot-frame (N,2) points."""
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        c, s = math.cos(self.theta), math.sin(self.theta)
        # Undo the translation, then rotate by -theta (the inverse rotation).
        dx, dy = pts[:, 0] - self.x, pts[:, 1] - self.y
        return np.column_stack([c * dx + s * dy, -s * dx + c * dy])
