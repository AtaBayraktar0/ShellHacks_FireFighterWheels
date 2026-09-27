"""Small occupancy grid and deterministic A*."""
import heapq
import math
import numpy as np


class Grid:
    """Store what the robot knows about the room."""

    def __init__(self, width=60, height=60, cell_m=0.05):
        # False means the cell has no known obstacle yet.
        self.cell_m = cell_m
        self.blocked = np.zeros((height, width), dtype=bool)
        self.seen = np.full((height, width), -np.inf)

    def cell(self, x, y):
        """Turn a location in meters into a grid square."""
        return math.floor(x / self.cell_m), math.floor(y / self.cell_m)

    def center(self, cell):
        """Find the middle of a grid square in meters."""
        return tuple((v + 0.5) * self.cell_m for v in cell)

    def inside(self, cell):
        """Check that a square is inside the map."""
        x, y = cell
        return 0 <= y < self.blocked.shape[0] and 0 <= x < self.blocked.shape[1]

    def observe(self, free, occupied, now):
        """Add new camera observations to the map."""
        for cell in free:
            if self.inside(cell):
                self.seen[cell[1], cell[0]] = now
        # Keep hazards until an operator starts a new map.
        for cell in occupied:
            if self.inside(cell):
                self.blocked[cell[1], cell[0]] = True

    def safe(self, now, max_age=2.0, radius_m=0.20):
        """Return squares that the whole robot can safely use."""
        # Old and unseen squares are unsafe.
        unavailable = self.blocked | ((now - self.seen) > max_age)
        radius = math.ceil(radius_m / self.cell_m)
        padded = np.pad(unavailable, radius, constant_values=True)
        result = np.ones_like(unavailable)
        h, w = result.shape
        # Expand hazards and unknown space by the robot's turning radius.
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if math.hypot(dx, dy) <= radius:
                    result &= ~padded[radius+dy:radius+dy+h, radius+dx:radius+dx+w]
        return result


def astar(safe, start, goal):
    """Find the shortest safe route from start to goal."""
    def valid(p):
        x, y = p
        return 0 <= y < safe.shape[0] and 0 <= x < safe.shape[1] and safe[y, x]

    if not valid(start) or not valid(goal):
        return []
    distance = lambda p: abs(p[0] - goal[0]) + abs(p[1] - goal[1])
    # The queue holds the best squares to check next.
    queue, cost, parent = [(distance(start), 0, start)], {start: 0}, {}
    while queue:
        _, g, p = heapq.heappop(queue)
        if g != cost[p]:
            continue
        if p == goal:
            # Walk backward through the saved parents to build the route.
            route = [p]
            while p in parent:
                p = parent[p]
                route.append(p)
            return route[::-1]
        for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            # The robot moves one square up, down, left, or right.
            q = p[0] + dx, p[1] + dy
            if valid(q) and g + 1 < cost.get(q, math.inf):
                cost[q], parent[q] = g + 1, p
                heapq.heappush(queue, (g + 1 + distance(q), g + 1, q))
    return []
