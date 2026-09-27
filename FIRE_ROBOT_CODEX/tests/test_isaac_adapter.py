"""Created by OpenAI Codex: CPU checks; these do not run Isaac/PhysX/RTX."""
import math
import numpy as np
import pytest
from isaac.math_utils import wheel_speeds,yaw_from_quaternion,intrinsics,optical_points,body_gaps
from isaac.control import IsaacBase
from rescue.config import Config
from rescue.mission import MissionFailed


def test_wheel_commands_forward_and_left():
    assert np.allclose(wheel_speeds(.2,0,.04,.2),[5,5,5,5])
    assert np.allclose(wheel_speeds(0,1,.04,.2),[-2.5,-2.5,2.5,2.5])


def test_quaternion_yaw_convention():
    assert yaw_from_quaternion([math.sqrt(.5),0,0,math.sqrt(.5)])==pytest.approx(math.pi/2)


def test_depth_unprojection_and_invalid_pixels():
    cfg=Config(depth_stride=1)
    K=intrinsics(4,2,90)
    depth=np.array([[np.nan,np.inf,0,.1],[1,1,1,4.]])
    pts=optical_points(depth,K,cfg)
    assert pts.shape==(3,3)
    assert pts[2]==pytest.approx([0,0,1])
    assert pts[0,0]<0  # pixel on image left has negative optical x
    mask=np.zeros_like(depth,dtype=bool); mask[1,2]=True
    assert optical_points(depth,K,cfg,mask).shape==(1,3)


def test_surface_gaps_include_body_and_fire_radius():
    cfg=Config()
    gaps=body_gaps(0,0,.15,cfg,[(.3,-.1,.4,.1)],(1.,0.))
    assert gaps['box']==pytest.approx(.15)
    assert gaps['fire']==pytest.approx(.8)
    assert body_gaps(0,cfg.y_max-.1,.15,cfg,[],None)['edge']<0


class FakeRuntime:
    def __init__(self, frozen=False, sonar=math.inf, fail=False):
        self.t=0.; self.h=0.; self.v=0.; self.omega=0.
        self.frozen=frozen; self.sonar=sonar; self.fail=fail
    def pose(self): return (0,0,self.h)
    def time(self): return self.t
    def command(self,v,w): self.v,self.omega=v,w
    def step(self):
        if self.fail: raise RuntimeError('physics failed')
        self.t+=.02
        if not self.frozen: self.h+=self.omega*.02
    def wait(self,s): self.t+=s
    def sonar_distance(self): return self.sonar


def test_turn_reaches_positive_heading_and_stops():
    rt=FakeRuntime(); base=IsaacBase(rt,Config())
    assert base.turn(math.pi/2)==pytest.approx(math.pi/2,abs=.02)
    assert (rt.v,rt.omega)==(0,0)


def test_frozen_heading_times_out_and_stops():
    rt=FakeRuntime(frozen=True)
    with pytest.raises(MissionFailed,match='timed out'):
        IsaacBase(rt,Config()).turn(.5)
    assert (rt.v,rt.omega)==(0,0)


def test_sonar_stop_reports_no_distance():
    rt=FakeRuntime(sonar=.05)
    assert IsaacBase(rt,Config()).move(.4)==(0.,True)
    assert (rt.v,rt.omega)==(0,0)


def test_move_stops_on_physics_error():
    rt=FakeRuntime(fail=True)
    with pytest.raises(RuntimeError,match='physics failed'):
        IsaacBase(rt,Config()).move(.4)
    assert (rt.v,rt.omega)==(0,0)
