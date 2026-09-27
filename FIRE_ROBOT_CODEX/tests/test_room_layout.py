"""Check the room drawing's scale and physical clearance."""
from types import SimpleNamespace
from isaac.room_layout import configure, shapes, rectangle
from isaac.math_utils import body_gaps


def test_room_matches_drawing():
    cfg = SimpleNamespace()
    configure(cfg)
    assert cfg.goal_xy == (1.9, 1.9)
    objects = shapes()
    assert len(objects) == 9
    assert objects[0][1] == rectangle(20,18,15,8)
    boxes = [bounds for _,bounds,_,_ in objects]
    for x,y in [(0,0),cfg.goal_xy]:
        gaps = body_gaps(x,y,cfg.robot_radius_m,cfg,boxes,None)
        assert gaps['edge'] > 0
        assert gaps['box'] > 0
