"""Seeded, solvable procedural arenas for navigation regression tests."""
from collections import deque
import math
import random

CELL = .05
BOUNDS = (-.5, 2.5, -1.25, 1.25)
START = (0., 0.)
GOAL = (2., 0.)
WALL = .05


def configure(cfg):
    cfg.x_min, cfg.x_max, cfg.y_min, cfg.y_max = BOUNDS
    cfg.goal_xy = GOAL
    cfg.robot_radius_m = .145
    cfg.clearance_m = .015
    cfg.assumed_depth_m = 0.
    cfg.edge_is_wall = True


def _clear_of(rect, point, gap):
    x0, y0, x1, y1 = rect
    x, y = point
    return not (x0-gap <= x <= x1+gap and y0-gap <= y <= y1+gap)


def _solvable(boxes, flame, radius=.16, fire_margin=.10):
    x0, x1, y0, y1 = BOUNDS
    nx, ny = round((x1-x0)/CELL), round((y1-y0)/CELL)
    blocked = [[False]*nx for _ in range(ny)]
    for iy in range(ny):
        y = y0+(iy+.5)*CELL
        for ix in range(nx):
            x = x0+(ix+.5)*CELL
            edge = x < x0+radius or x > x1-radius or y < y0+radius or y > y1-radius
            obstacle = any(a-radius <= x <= c+radius and b-radius <= y <= d+radius
                           for a,b,c,d in boxes)
            fire = math.hypot(x-flame[0], y-flame[1]) <= radius+fire_margin+.05
            blocked[iy][ix] = edge or obstacle or fire
    def cell(p):
        return int((p[0]-x0)/CELL), int((p[1]-y0)/CELL)
    start, goal = cell(START), cell(GOAL)
    q, seen = deque([start]), {start}
    while q:
        c = q.popleft()
        if c == goal:
            return True
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            n = c[0]+dx, c[1]+dy
            if (0 <= n[0] < nx and 0 <= n[1] < ny and n not in seen
                    and not blocked[n[1]][n[0]]):
                seen.add(n)
                q.append(n)
    return False


def generate(seed):
    """Return (boxes, flame); retry deterministically until a route exists."""
    rng = random.Random(seed)
    for _ in range(1000):
        boxes = []
        for _ in range(rng.randint(4, 7)):
            w = rng.randrange(3, 8)*CELL
            h = rng.randrange(3, 9)*CELL
            cx = rng.uniform(0., 2.15)
            cy = rng.uniform(-.95, .95)
            rect = (cx-w/2, cy-h/2, cx+w/2, cy+h/2)
            if (_clear_of(rect, START, .32) and _clear_of(rect, GOAL, .32)
                    and all(rect[2]+.08 < r[0] or r[2]+.08 < rect[0]
                            or rect[3]+.08 < r[1] or r[3]+.08 < rect[1] for r in boxes)):
                boxes.append(rect)
        flame = (rng.uniform(.45, 1.65), rng.uniform(-.75, .75))
        if (len(boxes) >= 4 and math.dist(flame, START) > .4
                and math.dist(flame, GOAL) > .4
                and all(_clear_of(r, flame, .12) for r in boxes)
                and _solvable(boxes, flame)):
            return boxes, flame
    raise RuntimeError(f'could not generate a solvable random map for seed {seed}')


def shapes(seed):
    boxes, flame = generate(seed)
    result = [(f'Box_{i}', r, .26, (.25,.28,.33)) for i,r in enumerate(boxes)]
    fx, fy = flame
    result.append(('Flame', (fx-.05,fy-.05,fx+.05,fy+.05), .24, (1.,.15,0.)))
    x0,x1,y0,y1 = BOUNDS
    result.extend([
        ('South',(x0-WALL,y0-WALL,x1+WALL,y0),.35,(.48,.42,.32)),
        ('North',(x0-WALL,y1,x1+WALL,y1+WALL),.35,(.48,.42,.32)),
        ('West',(x0-WALL,y0,x0,y1),.35,(.48,.42,.32)),
        ('East',(x1,y0,x1+WALL,y1),.35,(.48,.42,.32)),
    ])
    return result
