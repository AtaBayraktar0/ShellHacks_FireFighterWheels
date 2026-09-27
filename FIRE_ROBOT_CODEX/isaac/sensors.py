"""Created by OpenAI Codex: Isaac Sim 6.1 RTX RGB-D and debug ray sensors.

RGB-D uses rendered color and image-plane depth, with existing perception code.
Raycast mode is an idealized diagnostic using scene labels, not a D435 model.
"""
import math
import numpy as np
from rescue.grid import Observation
from rescue.perception import camera_to_robot, classify, flame_mask
from .math_utils import intrinsics, optical_points


class RgbdCamera:
    def __init__(self, runtime, cfg):
        from pxr import Gf, UsdGeom
        from isaacsim.sensors.experimental.rtx import RtxCamera, CameraSensor
        from .scene import CHASSIS, CHASSIS_Z
        self.rt, self.cfg = runtime, cfg
        self.Gf, self.chassis_z = Gf, CHASSIS_Z
        self.width, self.height = 640,480
        self.K = intrinsics(self.width,self.height,cfg.hfov_deg)
        self.path = CHASSIS+'/DepthCamera'
        camera = UsdGeom.Camera.Define(runtime.stage,self.path)
        camera.CreateFocalLengthAttr(20.)
        aperture = 40.*math.tan(math.radians(cfg.hfov_deg)/2)
        camera.CreateHorizontalApertureAttr(aperture)
        camera.CreateVerticalApertureAttr(aperture*self.height/self.width)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.01,10.))
        self.transform = camera.AddTransformOp()
        self.pan(0.)
        self.author = RtxCamera(self.path,tick_rate=30.,reset_xform_op_properties=False)
        # 6.1 takes (height,width), unlike the old Camera API's (width,height).
        self.sensor = CameraSensor(self.author,resolution=(self.height,self.width),
                                   annotators=['rgb','distance_to_image_plane'])
        # CameraSensor may rebuild the camera prim while it attaches RTX
        # annotators. Reacquire the prim and author a fresh transform op so
        # later pan calls never use a schema object for the replaced prim.
        xform = UsdGeom.Xformable(runtime.stage.GetPrimAtPath(self.path))
        xform.ClearXformOpOrder()
        self.transform = xform.AddTransformOp(opSuffix='pan')
        self.pan(0.)
        self.last_rgb = None
        self.last_depth = None

    def pan(self, yaw):
        Gf,cfg = self.Gf,self.cfg
        p = math.radians(cfg.cam_pitch_deg)
        eye = Gf.Vec3d(cfg.cam_forward_m,0,cfg.cam_height_m-self.chassis_z)
        forward = Gf.Vec3d(math.cos(yaw)*math.cos(p),math.sin(yaw)*math.cos(p),-math.sin(p))
        up = Gf.Vec3d(math.cos(yaw)*math.sin(p),math.sin(yaw)*math.sin(p),math.cos(p))
        # USD camera looks along -Z with +Y up; look-at supplies the conversion.
        self.transform.Set(Gf.Matrix4d().SetLookAt(eye,eye+forward,up).GetInverse())

    def observe(self, yaw=0.):
        self.pan(yaw)
        self.rt.wait(max(.12,self.cfg.pan_settle_s))
        for _ in range(120):
            self.rt.step()
            rgb,_ = self.sensor.get_data('rgb')
            depth,_ = self.sensor.get_data('distance_to_image_plane')
            if rgb is None or depth is None:
                continue
            rgb = rgb.numpy() if hasattr(rgb,'numpy') else np.asarray(rgb)
            depth = depth.numpy() if hasattr(depth,'numpy') else np.asarray(depth)
            depth = np.squeeze(depth)
            if depth.shape != (self.height,self.width) or rgb.shape[:2] != depth.shape:
                continue
            if np.count_nonzero(np.isfinite(depth) & (depth>0) & (depth<self.cfg.max_range_m)) < 100:
                continue
            self.last_rgb,self.last_depth = rgb[...,:3].copy(),depth.copy()
            pts = camera_to_robot(optical_points(depth,self.K,self.cfg),self.cfg,yaw)
            floor,obstacles = classify(pts,self.cfg)
            fm = flame_mask(np.ascontiguousarray(rgb[...,:3][...,::-1]),self.cfg)
            fp = camera_to_robot(optical_points(depth,self.K,self.cfg,fm),self.cfg,yaw)
            fire = fp[(fp[:,2]>=self.cfg.floor_tol_m) & (fp[:,2]<=self.cfg.obstacle_max_z_m),:2]
            return Observation(floor,obstacles,fire)
        raise RuntimeError('RTX camera produced no usable RGB-D frame. Check renderer/GPU; try --sensor raycast to isolate physics.')


class RaycastCamera:
    def __init__(self,runtime,cfg):
        self.rt,self.cfg = runtime,cfg

    def pan(self,yaw):
        pass

    def observe(self,yaw=0.):
        cfg=self.cfg
        x,y,heading=self.rt.pose()
        cx,cy=x+cfg.cam_forward_m*math.cos(heading),y+cfg.cam_forward_m*math.sin(heading)
        floor,obstacles,fire=[],[],[]
        def local(px,py):
            dx,dy=px-x,py-y
            return (math.cos(heading)*dx+math.sin(heading)*dy,
                    -math.sin(heading)*dx+math.cos(heading)*dy)
        for a in np.linspace(-math.radians(cfg.hfov_deg)/2,math.radians(cfg.hfov_deg)/2,120):
            angle=heading+yaw+a
            c,s=math.cos(angle),math.sin(angle)
            hit=self.rt.raycast((cx,cy,cfg.cam_height_m),(c,s,0.),cfg.max_range_m)
            d=float(hit['distance']) if hit.get('hit') else cfg.max_range_m
            for r in np.arange(cfg.min_range_m,d-.02,cfg.cell_m):
                floor.append(local(cx+r*c,cy+r*s))
            if hit.get('hit') and d>=cfg.min_range_m:
                path=str(hit.get('collision',hit.get('rigidBody','')))
                for k in range(cfg.min_obstacle_hits+1):
                    point=local(cx+(d+.005*k)*c,cy+(d+.005*k)*s)
                    obstacles.append(point)
                    if 'FlameProp' in path or 'Room_Flame' in path:
                        fire.append(point)
        return Observation(*(np.asarray(p,dtype=float).reshape(-1,2) for p in (floor,obstacles,fire)))
