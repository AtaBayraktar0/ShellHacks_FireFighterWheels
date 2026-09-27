# ======================================================================
# MODIFIED BY CLAUDE (Anthropic, Claude Code), 2026-09-26. NOT the Codex version.
# Search for "[CLAUDE EDIT]" to find each change. Full list: CLAUDE_CHANGES.md
# ======================================================================
# Change here: border re-applied after clearing the car's footprint.

"""The behavior loop from the build plan:

1. scan the area and build the map
2. A* from start to goal
3. drive the path, recording where the car actually went
4. retrace the recorded waypoints back to start,
   re-checking each leg against a fresh camera look before driving it

Works with either the real car (UnoBase + D435Camera) or the sim
(SimBase + SimCamera); both expose the same methods.
"""

import math

from .astar import astar, smooth, split_long
from .geometry import Pose, dist, wrap
from .grid import OCC, OccupancyGrid


class MissionFailed(Exception):
    pass


class Mission:
    def __init__(self, cfg, base, camera, telemetry=None, log=print):
        self.cfg = cfg
        self.base = base
        self.camera = camera
        self.tel = telemetry
        self.log = log
        self.grid = OccupancyGrid(cfg)
        self.pose = Pose()  # estimated pose (dead reckoning)
        self.start = (0.0, 0.0)
        self.goal = tuple(cfg.goal_xy)
        # Routes for telemetry and the plot: planned, and where the car went out and back.
        self.planned = []
        self.driven_out = []
        self.driven_back = []
        self.replans = 0
        self.phase = "idle"
        # Margins the current path was planned with (None = config defaults).
        self._margins = None
        h = base.heading()
        self._heading_offset = -h if h is not None else None  # start heading = 0

    # --- top level ---

    def run(self):
        self.scan_and_plan()
        self.outbound()
        self.return_trip()
        self.phase = "done"
        self._send_state()
        self.log(f"Done. Back at start ({self.replans} replans).")

    def scan_and_plan(self):
        self.phase = "scan"
        self.log("Scanning...")
        self.scan()
        self.planned = self.plan(self.goal)
        self.log(f"Planned {len(self.planned) - 1} legs to goal {self.goal}")
        self._send_state()

    def outbound(self):
        self.phase = "outbound"
        self.driven_out = self.follow(self.planned, self.goal, self.cfg.reverify_outbound)
        self.log(f"Reached goal, estimated pose ({self.pose.x:.2f}, {self.pose.y:.2f})")

    def return_trip(self):
        self.phase = "return"
        # Retrace the route actually driven: that ground is known to be passable.
        back = list(reversed(self.driven_out))
        self.driven_back = self.follow(back, self.start, reverify=True)

    # --- sensing ---

    def scan(self):
        yaws = [math.radians(d) for d in self.cfg.scan_yaws_deg]
        # With the pan servo, sweep the camera; otherwise turn the whole car to each angle.
        if self.cfg.pan_enabled:
            for yaw in yaws:
                self.base.pan(yaw)
                self.grid.update(self.camera.observe(yaw), self.pose)
            self.base.pan(0.0)
        else:
            heading = self.pose.theta
            for yaw in yaws:
                self.turn_to_heading(heading + yaw)
                self.grid.update(self.camera.observe(0.0), self.pose)
            self.turn_to_heading(heading)
        self._send_grid()

    def look_ahead(self):
        """Refresh the map with one frame straight ahead."""
        self.grid.update(self.camera.observe(0.0), self.pose)
        self._send_grid()

    # --- planning ---

    def blocked(self, margins=None):
        """Blocked mask for planning, with the car's own footprint cleared."""
        mask = self.grid.blocked_mask(*(margins or (None, None)))
        self.grid.clear_disk(mask, self.pose.xy, self.cfg.robot_radius_m)
        # [CLAUDE EDIT] Bug fix: clear_disk() unblocks cells under the car. Near an edge that
        #   erased part of the border, letting the planner route along/off the edge.
        #   Re-applying block_edges() afterwards puts the border back.
        # Clearing our current footprint must never reopen an arena edge.
        self.grid.block_edges(mask, margins[0] if margins is not None else None)
        return mask

    def plan(self, goal):
        s, g = self.grid.cell(self.pose.xy), self.grid.cell(goal)
        if s is None or g is None:
            raise MissionFailed("start or goal is outside the map bounds")
        # Try the normal margins first, then the progressively tighter fallbacks.
        for margins in [None, *self.cfg.fallback_margins]:
            mask = self.blocked(margins)
            gc = self._goal_cell(mask, goal)
            cells = astar(mask, s, gc) if gc else None
            if cells is not None:
                break
        # for/else: runs only if no set of margins produced a path.
        else:
            x, y = self.pose.xy
            raise MissionFailed(f"no safe path from ({x:.2f}, {y:.2f}) to {goal}")
        if margins:
            self.log(f"  tight spot: planning with clearance {margins[0]} m, "
                     f"fire margin {margins[1]} m")
        self._margins = margins  # legs of this path are checked with the same margins
        # Cell path -> world points at cell centers.
        pts = [self.grid.center(c) for c in cells]
        # Use the exact start and goal instead of their cell centers.
        pts[0] = self.pose.xy
        if gc == g:
            pts[-1] = goal
        # Collapse straight runs, then split long legs so each gets re-checked.
        pts = smooth(pts, lambda a, b: self.grid.segment_clear(mask, a, b))
        return split_long(pts, self.cfg.max_segment_m)

    def _goal_cell(self, mask, goal):
        """The goal's cell, or if margins cover it, the nearest open cell within
        goal_tolerance_m. None if the goal is really inside an obstacle."""
        g = self.grid.cell(goal)
        if not mask[g[1], g[0]]:
            return g
        r = int(self.cfg.goal_tolerance_m / self.grid.res)
        best = None
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                c = (g[0] + dx, g[1] + dy)
                if not (0 <= c[0] < self.grid.nx and 0 <= c[1] < self.grid.ny) or mask[c[1], c[0]]:
                    continue
                d = dist(self.grid.center(c), goal)
                if d <= self.cfg.goal_tolerance_m and (best is None or d < best[0]):
                    best = (d, c)
        return best[1] if best else None

    def replan(self, goal, why):
        self.replans += 1
        if self.replans > self.cfg.max_replans:
            raise MissionFailed(f"gave up after {self.cfg.max_replans} replans")
        self.log(f"Replanning ({why})")
        path = self.plan(goal)
        self._send_state(path)
        # Drop the first point: it's where the car already is.
        return path[1:]

    # --- motion ---

    def imu_heading(self):
        """Heading from the gyro in the world frame, or None without an IMU."""
        if self._heading_offset is None:
            return None
        return wrap(self.base.heading() + self._heading_offset)

    def turn_to_heading(self, heading):
        """Turn in place to face `heading` (world radians), then update the pose."""
        diff = wrap(heading - self.pose.theta)
        # Skip tiny corrections; they add more error than they remove.
        if abs(diff) > math.radians(1):
            turned = self.base.turn(diff)
            h = self.imu_heading()
            # Trust the gyro when there is one, otherwise the commanded turn.
            self.pose.theta = h if h is not None else wrap(self.pose.theta + turned)

    def drive(self, d):
        """Drive forward d meters and update the pose. Returns stopped_by_sonar."""
        th0 = self.pose.theta
        moved, stopped = self.base.move(d)
        h = self.imu_heading()
        if h is None:
            self.pose.advance(moved)
        else:
            # use the average heading over the move, then take the gyro's word for it
            self.pose.theta = wrap(th0 + wrap(h - th0) / 2)
            self.pose.advance(moved)
            self.pose.theta = h
        return stopped

    def follow(self, path, goal, reverify):
        """Drive through `path` to `goal`, returning the waypoints actually driven."""
        driven = [self.pose.xy]
        # Skip the first waypoint if it's just the current position.
        wps = list(path[1:] if path and dist(path[0], self.pose.xy) < 0.02 else path)
        while True:
            # Arrived: near the goal, and any remaining waypoints end there too.
            if dist(self.pose.xy, goal) <= self.cfg.goal_tolerance_m and \
                    (not wps or dist(self.pose.xy, wps[-1]) <= self.cfg.goal_tolerance_m):
                return driven
            # Out of waypoints but not at the goal (drift): plan the rest of the way.
            if not wps:
                wps = self.replan(goal, "short of goal")
                continue
            target = wps[0]
            d = dist(self.pose.xy, target)
            if d < 0.05:  # not worth a turn
                wps.pop(0)
                continue

            # Face the next waypoint first, so the camera looks along the leg.
            self.turn_to_heading(math.atan2(target[1] - self.pose.y, target[0] - self.pose.x))
            # Check the leg against a fresh look before committing to it.
            if reverify:
                self.look_ahead()
                if not self.grid.segment_clear(self.blocked(self._margins), self.pose.xy, target):
                    wps = self.replan(goal, "route no longer clear")
                    continue

            stopped = self.drive(d)
            driven.append(self.pose.xy)
            self._send_state()

            if stopped:
                # Ultrasonic stop: something is ~sonar_stop_m past the bumper.
                ahead = self.cfg.robot_front_m + self.cfg.sonar_stop_m
                self.grid.mark((self.pose.x + ahead * math.cos(self.pose.theta),
                                self.pose.y + ahead * math.sin(self.pose.theta)), OCC)
                self.look_ahead()
                wps = self.replan(goal, "ultrasonic stop")
                continue
            # Leg completed normally.
            wps.pop(0)

    # --- telemetry ---

    def _send_state(self, path=None):
        if self.tel:
            self.tel.state(self.phase, self.pose, path or self.planned,
                           self.driven_out, self.driven_back)

    def _send_grid(self):
        if self.tel:
            self.tel.grid(self.grid)
