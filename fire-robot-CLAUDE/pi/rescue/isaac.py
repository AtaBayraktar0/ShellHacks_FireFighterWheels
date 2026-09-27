# ======================================================================
# CREATED BY CLAUDE (Anthropic, Claude Code), 2026-09-26. NOT the Codex version.
# [CLAUDE EDIT] New file: Isaac Sim 6.1 backend. Full list: CLAUDE_CHANGES.md
# ======================================================================

"""NVIDIA Isaac Sim 6.1 stand-in for the car and the D435.

What Isaac adds over sim.py: a rendered RGB + depth camera. Its images go through
the SAME code as the real D435 (camera_to_robot, classify, flame_mask), so the
perception pipeline and the flame HSV thresholds get tested, not just planning.

What stays from sim.py: the arena layout (SimWorld.demo), the car's motion error
model (SimBase: timed moves, veer, gyro drift), the sonar stop and the collision
count. The car is moved kinematically; there is no wheel physics yet.

The pure-numpy helpers at the top need no Isaac install (see test_isaac_geometry.py).
Everything that needs Isaac is imported inside IsaacScene, after run_isaac.py has
started the SimulationApp, which Isaac requires to happen first.
"""

import math
from pathlib import Path

import numpy as np

from .perception import D435Camera
from .sim import SimBase

BOX_HEIGHT_M = 0.25                 # tall enough for the pitched-down camera to see
FLAME_HEIGHT_M = 0.20
BODY_SIZE_M = (0.26, 0.20, 0.08)    # visual only; collisions use SimBase.BODY_RADIUS
WIDTH, HEIGHT = 848, 480            # a D435 depth mode: 87 x 58 degree field of view


def pose_matrix(x, y, yaw, z=0.0):
    """4x4 USD transform (row-vector convention): rotate yaw about +z, then translate."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, s, 0, 0],
                     [-s, c, 0, 0],
                     [0, 0, 1, 0],
                     [x, y, z, 1]], dtype=float)


def camera_mount_matrix(cfg):
    """USD camera transform inside the pan frame.

    A USD camera looks down its -z axis with +y up. Point it along the car's +x,
    pitched down cam_pitch_deg, with the lens cam_height_m above the floor: the same
    mount camera_to_robot() assumes for the real D435.
    """
    p = math.radians(cfg.cam_pitch_deg)
    right = (0.0, -1.0, 0.0)
    up = (math.sin(p), 0.0, math.cos(p))
    forward = (math.cos(p), 0.0, -math.sin(p))
    return np.array([[*right, 0],
                     [*up, 0],
                     [-forward[0], -forward[1], -forward[2], 0],
                     [0, 0, cfg.cam_height_m, 1]], dtype=float)


def pan_matrix(cfg, yaw):
    """Pan frame inside the car frame: pan axis cam_forward_m ahead, turned `yaw` left."""
    return pose_matrix(cfg.cam_forward_m, 0.0, yaw)


def focal_px(cfg, width=WIDTH):
    """Pixel focal length for the configured horizontal field of view (square pixels)."""
    return (width / 2) / math.tan(math.radians(cfg.hfov_deg) / 2)


class IsaacScene:
    """Builds the arena on the USD stage and renders the car's camera."""

    def __init__(self, app, world, render_frames=4, animate_frames=0):
        import omni.replicator.core as rep
        import omni.usd
        from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

        self.app, self.world, self.cfg = app, world, world.cfg
        self.render_frames, self.animate_frames = render_frames, animate_frames
        self._Gf, self._Sdf, self._UsdGeom, self._UsdShade = Gf, Sdf, UsdGeom, UsdShade
        cfg = self.cfg

        stage = self.stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, "/World")

        # Lights: soft ambient plus a sun, so box sides are shaded but not black.
        UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(800.0)
        sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
        sun.CreateIntensityAttr(2500.0)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-40, 20, 0))

        # Floor: a large gray slab with its top at z = 0 (camera_to_robot's floor).
        cx, cy = (cfg.x_min + cfg.x_max) / 2, (cfg.y_min + cfg.y_max) / 2
        sx, sy = cfg.x_max - cfg.x_min, cfg.y_max - cfg.y_min
        self._box("/World/Floor", (cx, cy, -0.01), (sx + 4, sy + 4, 0.02), (0.45, 0.45, 0.45))
        # Arena outline painted on the floor (visual only, 2 mm tall, reads as floor).
        for i, (px, py, lx, ly) in enumerate([(cx, cfg.y_min, sx, 0.02), (cx, cfg.y_max, sx, 0.02),
                                              (cfg.x_min, cy, 0.02, sy), (cfg.x_max, cy, 0.02, sy)]):
            self._box(f"/World/Edge{i}", (px, py, 0.001), (lx, ly, 0.002), (0.1, 0.1, 0.1))

        # Obstacles and flame from the same layout the 2D sim uses.
        for i, b in enumerate(world.boxes):
            self._box(f"/World/Box{i}", ((b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2, BOX_HEIGHT_M / 2),
                      (b.x1 - b.x0, b.y1 - b.y0, BOX_HEIGHT_M), (0.25, 0.35, 0.6))
        if world.flame:
            flame = UsdGeom.Cylinder.Define(stage, "/World/Flame")
            flame.CreateRadiusAttr(world.flame_r)
            flame.CreateHeightAttr(FLAME_HEIGHT_M)
            flame.CreateAxisAttr("Z")
            UsdGeom.Xformable(flame).AddTranslateOp().Set(
                Gf.Vec3d(world.flame[0], world.flame[1], FLAME_HEIGHT_M / 2))
            # Saturated orange that glows, so it stays inside the default flame_hsv range.
            self._paint(flame, "Flame", (1.0, 0.3, 0.0), emissive=True)

        # Car: /World/Car (pose) > Pan (servo) > D435 (fixed mount).
        car = UsdGeom.Xform.Define(stage, "/World/Car")
        self._car_op = car.AddTransformOp()
        self._box("/World/Car/Body", (0, 0, 0.02 + BODY_SIZE_M[2] / 2), BODY_SIZE_M, (0.8, 0.8, 0.2))
        pan = UsdGeom.Xform.Define(stage, "/World/Car/Pan")
        self._pan_op = pan.AddTransformOp()
        cam = UsdGeom.Camera.Define(stage, "/World/Car/Pan/D435")
        cam.AddTransformOp().Set(Gf.Matrix4d(camera_mount_matrix(cfg).tolist()))
        # Only the aperture/focal ratio matters for the field of view.
        f = 1.93
        h_ap = 2 * f * math.tan(math.radians(cfg.hfov_deg) / 2)
        cam.CreateFocalLengthAttr(f)
        cam.CreateHorizontalApertureAttr(h_ap)
        cam.CreateVerticalApertureAttr(h_ap * HEIGHT / WIDTH)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 20.0))

        # Replicator annotators give numpy RGB and z-depth (like the D435's z16 depth).
        rp = rep.create.render_product("/World/Car/Pan/D435", (WIDTH, HEIGHT))
        self._rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        self._depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        self._rgb.attach([rp])
        self._depth.attach([rp])

        self._shown = (world.pose.x, world.pose.y, world.pose.theta)
        self.set_car(*self._shown)
        self.set_pan(0.0)
        self.render(10)  # let the renderer warm up

    # --- stage helpers ---

    def _box(self, path, center, size, rgb):
        cube = self._UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        xf = self._UsdGeom.Xformable(cube)
        xf.AddTranslateOp().Set(self._Gf.Vec3d(*center))
        xf.AddScaleOp().Set(self._Gf.Vec3f(*size))
        self._paint(cube, path.rsplit("/", 1)[-1], rgb)

    def _paint(self, gprim, name, rgb, emissive=False):
        Gf, Sdf, UsdShade = self._Gf, self._Sdf, self._UsdShade
        gprim.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
        path = f"/World/Looks/{name}"
        mat = UsdShade.Material.Define(self.stage, path)
        sh = UsdShade.Shader.Define(self.stage, path + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
        if emissive:
            sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(gprim.GetPrim()).Bind(mat)

    # --- runtime ---

    def set_car(self, x, y, theta):
        self._car_op.Set(self._Gf.Matrix4d(pose_matrix(x, y, theta).tolist()))

    def set_pan(self, yaw):
        self._pan_op.Set(self._Gf.Matrix4d(pan_matrix(self.cfg, yaw).tolist()))

    def render(self, n):
        for _ in range(n):
            self.app.update()

    def show_true_pose(self):
        """Move the USD car to the sim's true pose, animated in the GUI if asked."""
        p = self.world.pose
        start, end = self._shown, (p.x, p.y, p.theta)
        dth = math.atan2(math.sin(end[2] - start[2]), math.cos(end[2] - start[2]))
        for k in range(1, self.animate_frames + 1):
            t = k / self.animate_frames
            self.set_car(start[0] + t * (end[0] - start[0]),
                         start[1] + t * (end[1] - start[1]), start[2] + t * dth)
            self.app.update()
        self.set_car(*end)
        self._shown = end

    def images(self):
        """Latest (depth_m, bgr) pair, in the same form D435Camera.frames() returns."""
        rgba, depth = self._rgb.get_data(), self._depth.get_data()
        if isinstance(rgba, dict):
            rgba = rgba["data"]
        if isinstance(depth, dict):
            depth = depth["data"]
        rgba = np.asarray(rgba).reshape(HEIGHT, WIDTH, -1)
        bgr = np.ascontiguousarray(rgba[:, :, 2::-1]).astype(np.uint8)
        depth = np.asarray(depth, dtype=float).reshape(HEIGHT, WIDTH)
        depth[~np.isfinite(depth)] = 0.0  # no hit reads as "no depth", like the D435's 0
        return depth, bgr


class IsaacBase(SimBase):
    """SimBase's error model, with the Isaac car following the true pose."""

    def __init__(self, world, scene, **kw):
        super().__init__(world, **kw)
        self.scene = scene

    def turn(self, angle):
        out = super().turn(angle)
        self.scene.show_true_pose()
        return out

    def move(self, dist):
        out = super().move(dist)
        self.scene.show_true_pose()
        return out


class IsaacCamera(D435Camera):
    """D435Camera's perception on Isaac-rendered images (no pyrealsense2 needed)."""

    def __init__(self, scene, save_dir=None):
        self.scene, self.cfg = scene, scene.cfg
        self.save_dir = Path(save_dir) if save_dir else None
        if self.save_dir:
            self.save_dir.mkdir(parents=True, exist_ok=True)
        self._n = 0
        # Same per-pixel rays D435Camera builds from the RealSense intrinsics.
        s, f = self.cfg.depth_stride, focal_px(self.cfg)
        v, u = np.mgrid[0:HEIGHT:s, 0:WIDTH:s]
        self._sel = (slice(0, HEIGHT, s), slice(0, WIDTH, s))
        self._rx = (u - WIDTH / 2) / f
        self._ry = (v - HEIGHT / 2) / f

    def observe(self, yaw=0.0):
        self.scene.set_pan(yaw)
        return super().observe(yaw)

    def frames(self):
        self.scene.render(max(self.scene.render_frames, self.cfg.discard_frames))
        depth, bgr = self.scene.images()
        if self.save_dir:
            # Color frame + flame mask, for tuning flame_hsv (see d435_check.py).
            import cv2
            from .perception import flame_mask
            cv2.imwrite(str(self.save_dir / f"{self._n:04d}_rgb.png"), bgr)
            cv2.imwrite(str(self.save_dir / f"{self._n:04d}_flame.png"),
                        flame_mask(bgr, self.cfg).astype(np.uint8) * 255)
            self._n += 1
        return depth, bgr

    def close(self):
        pass
