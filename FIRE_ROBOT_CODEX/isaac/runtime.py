"""Created by OpenAI Codex: Isaac Sim 6.1 lifecycle, physics and validation.

Imported only after SimulationApp starts. Truth XY is used for evaluation and
idealized debug sensors, never to correct mission odometry.
"""
import math
import time
import numpy as np
from .math_utils import body_gaps, wheel_speeds, yaw_from_quaternion
from .scene import create_scene, CHASSIS, JOINTS, WHEEL_RADIUS, TRACK, BODY_RADIUS


class Runtime:
    def __init__(self, app, cfg, layout, max_seconds, seed=0):
        import omni.usd
        import omni.physx
        import isaacsim.core.experimental.utils.app as app_utils
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.utils.backend import use_backend
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.core.simulation_manager import SimulationManager
        self.app,self.cfg,self.app_utils,self.sm = app,cfg,app_utils,SimulationManager
        self.use_backend=use_backend
        self.max_seconds=max_seconds
        self.samples=[]
        self.min_gaps=dict(edge=math.inf,box=math.inf,fire=math.inf)
        self.guard_enabled=False
        self.violation=None
        if cfg.robot_radius_m < BODY_RADIUS:
            raise ValueError(f'robot_radius_m must be >= {BODY_RADIUS} for this model')
        stage_utils.create_new_stage()
        self.stage=omni.usd.get_context().get_stage()
        self.boxes,self.flame=create_scene(self.stage,cfg,layout,seed)
        self.robot=Articulation(CHASSIS)
        SimulationManager.setup_simulation(dt=1./120.,device='cpu')
        scene=SimulationManager.get_physics_scenes()[0]
        scene.set_enabled_gpu_dynamics(False)
        self.query=omni.physx.get_physx_scene_query_interface()
        app_utils.play()
        app_utils.update_app(steps=10)
        self.indices=self.robot.get_dof_indices(JOINTS)
        self.command(0.,0.)
        self.wait(.5)
        self.start_time=self.time()
        self.guard_enabled=True

    def time(self):
        return float(self.sm.get_simulation_time())

    def pose(self):
        with self.use_backend('tensor',raise_on_unsupported=True):
            positions,orientations=self.robot.get_world_poses()
        p=positions.numpy()[0]
        q=orientations.numpy()[0]
        if not np.isfinite(p).all() or not np.isfinite(q).all():
            raise RuntimeError('Isaac returned a non-finite robot pose')
        return float(p[0]),float(p[1]),yaw_from_quaternion(q)

    def command(self,linear,angular):
        values=wheel_speeds(linear,angular,WHEEL_RADIUS,TRACK)
        with self.use_backend('tensor',raise_on_unsupported=True):
            self.robot.set_dof_velocity_targets(values.reshape(1,4),dof_indices=self.indices)

    def raycast(self,origin,direction,distance):
        return self.query.raycast_closest(tuple(float(v) for v in origin),
                                         tuple(float(v) for v in direction),float(distance))

    def sonar_distance(self):
        x,y,h=self.pose()
        # Forward origin is beyond the entire physical footprint.
        offset=max(self.cfg.robot_front_m,BODY_RADIUS+.005)
        hit=self.raycast((x+offset*math.cos(h),y+offset*math.sin(h),.10),
                         (math.cos(h),math.sin(h),0.),self.cfg.sonar_stop_m)
        return float(hit['distance']) if hit.get('hit') else math.inf

    def step(self):
        if not self.app.is_running() or not self.app_utils.is_playing():
            raise RuntimeError('Isaac simulation was closed, stopped or paused')
        before=self.time()
        self.app_utils.update_app()
        if self.time() <= before:
            raise RuntimeError('Isaac physics clock did not advance')
        if not self.guard_enabled:
            return
        x,y,h=self.pose()
        self.samples.append([self.time(),x,y,h])
        gaps=body_gaps(x,y,BODY_RADIUS,self.cfg,self.boxes,self.flame)
        for key,value in gaps.items():
            self.min_gaps[key]=min(self.min_gaps[key],value)
        if min(gaps.values()) < 0:
            self.violation=dict(time=self.time(),gaps=gaps)
            self.command(0.,0.)
            raise RuntimeError('Conservative robot footprint overlaps an obstacle, flame or arena edge')
        if self.time()-self.start_time > self.max_seconds:
            self.command(0.,0.)
            raise RuntimeError('Mission exceeded simulated-time limit')

    def wait(self,seconds):
        end=self.time()+seconds
        wall_deadline=time.monotonic()+max(60.,seconds*60.)
        while self.time()<end:
            if time.monotonic()>wall_deadline:
                raise RuntimeError('Isaac wait stalled')
            self.step()

    def shutdown(self):
        try:
            if self.app_utils.is_playing():
                self.command(0.,0.)
        finally:
            self.app_utils.stop()
