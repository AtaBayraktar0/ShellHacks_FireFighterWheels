"""Our original 5 cm room layout, measured from the car's start."""
CELL = 0.05
START = (0.525, 0.525)
OBJECTS = [
    ('Table', 20, 18, 15, 8, .26, (.30, .24, .18)),
    ('Shelf', 40, 9, 6, 19, .40, (.39, .45, .55)),
    ('BoxA', 18, 36, 7, 6, .26, (.40, .34, .25)),
    ('BoxB', 42, 36, 6, 5, .26, (.40, .34, .25)),
    ('Flame', 30, 45, 5, 6, .30, (1., .15, 0.)),
]


def rectangle(x, y, width, height):
    """Convert drawing cells into metres relative to home."""
    return (x*CELL-START[0], y*CELL-START[1],
            (x+width)*CELL-START[0], (y+height)*CELL-START[1])


def configure(cfg):
    """The inner wall faces bound the usable floor."""
    cfg.x_min = cfg.y_min = .1-START[0]
    cfg.x_max = cfg.y_max = 2.9-START[0]
    cfg.goal_xy = (2.425-START[0], 2.425-START[1])
    cfg.robot_radius_m = .20
    cfg.edge_is_wall = True


def shapes():
    """Return physical scene shapes, never a path for A*."""
    result = [(name, rectangle(x,y,w,h), height, color)
              for name,x,y,w,h,height,color in OBJECTS]
    for name,x,y,w,h in [('South',0,0,60,2), ('North',0,58,60,2),
                          ('West',0,2,2,56), ('East',58,2,2,56)]:
        result.append((name, rectangle(x,y,w,h), .35, (.48,.42,.32)))
    return result
