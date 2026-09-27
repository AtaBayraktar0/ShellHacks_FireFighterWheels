"""A* on the blocked mask, plus path cleanup for a car that turns in place."""

import heapq
import math

SQRT2 = math.sqrt(2)
# 8-connected moves as (dx, dy, cost): straight steps cost 1, diagonals sqrt(2).
NEIGHBORS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


def _octile(a, b):
    """Octile distance: exact cost on an empty 8-connected grid, so it never overestimates."""
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return dx + dy + (SQRT2 - 2) * min(dx, dy)


def astar(blocked, start, goal):
    """Shortest 8-connected path from start to goal cell, as a list of (ix, iy).

    blocked is a bool array indexed [iy, ix]. Diagonal moves may not cut the
    corner of a blocked cell. Returns None if there's no path.
    """
    h, w = blocked.shape
    if blocked[goal[1], goal[0]]:
        return None
    # g: best known cost to each cell; parent: back-pointers to rebuild the path.
    g = {start: 0.0}
    parent = {start: None}
    # Heap entries are (f = g + h, g, cell).
    heap = [(_octile(start, goal), 0.0, start)]
    closed = set()

    while heap:
        _, cost, cur = heapq.heappop(heap)
        # Stale heap entry: this cell was already expanded at a lower cost.
        if cur in closed:
            continue
        if cur == goal:
            # Walk the back-pointers from the goal, then reverse.
            path = []
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]
        closed.add(cur)
        cx, cy = cur
        for dx, dy, step in NEIGHBORS:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < w and 0 <= ny < h) or blocked[ny, nx]:
                continue
            # Diagonal move: both side cells must be free so the car can't clip a corner.
            if dx and dy and (blocked[cy, nx] or blocked[ny, cx]):
                continue
            ng = cost + step
            # Found a cheaper route to this neighbor; record it and queue it.
            if ng < g.get((nx, ny), math.inf):
                g[(nx, ny)] = ng
                parent[(nx, ny)] = cur
                heapq.heappush(heap, (ng + _octile((nx, ny), goal), ng, (nx, ny)))
    return None


def smooth(points, clear):
    """Drop waypoints the car can skip in a straight line.

    Fewer turns means less dead-reckoning error. `clear(a, b)` says whether the
    straight segment a->b is safe.
    """
    if len(points) < 3:
        return list(points)
    # Greedy: from each kept point, jump to the farthest later point with a clear line.
    out = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1 and not clear(points[i], points[j]):
            j -= 1
        out.append(points[j])
        i = j
    return out


def split_long(points, max_len):
    """Insert waypoints so no leg is longer than max_len."""
    out = [points[0]]
    for a, b in zip(points, points[1:]):
        # Number of equal pieces needed so each is at most max_len.
        n = max(1, math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / max_len))
        for k in range(1, n + 1):
            out.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
    return out
