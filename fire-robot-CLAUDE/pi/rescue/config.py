# ======================================================================
# MODIFIED BY CLAUDE (Anthropic, Claude Code), 2026-09-26. NOT the Codex version.
# Search for "[CLAUDE EDIT]" to find each change. Full list: CLAUDE_CHANGES.md
# ======================================================================
# Change here: arena y bounds widened and edge_is_wall added.

"""All tunable numbers in one place. Override any of them with a JSON file:

    cfg = Config.load("config.json")   # missing keys keep their defaults

World frame: origin at the start point, +x = the car's heading at the start,
+y = to the car's left. Meters and degrees throughout.
"""

import json
from dataclasses import asdict, dataclass, fields
from pathlib import Path


@dataclass
class Config:
    # --- Map (occupancy grid) ---
    cell_m: float = 0.05
    x_min: float = -0.5
    x_max: float = 2.5
    # [CLAUDE EDIT] Map y bounds widened to +/-1.25 m (were +/-1.0 m) as part of the map-edge fix.
    #   Set these to the MEASURED real arena; bigger numbers don't make a small table safe.
    y_min: float = -1.25
    y_max: float = 1.25
    # [CLAUDE EDIT] New: treat the map border as a wall so the whole car body stays inside.
    edge_is_wall: bool = True      # keep the whole body inside the arena bounds
    unknown_is_free: bool = True     # cells the camera hasn't seen yet
    min_obstacle_hits: int = 3       # depth points needed to mark a cell (noise filter)
    assumed_depth_m: float = 0.15    # unseen cells this close behind an obstacle count as solid

    # --- Robot footprint ---
    robot_radius_m: float = 0.15     # the car spins in place, so plan with a circle
    robot_front_m: float = 0.13      # center to front bumper (where the HC-SR04 is)
    clearance_m: float = 0.03        # extra gap kept from obstacles
    fire_margin_m: float = 0.10      # extra gap kept from the flame on top of clearance

    # --- Mission ---
    goal_xy: tuple = (2.0, 0.0)
    goal_tolerance_m: float = 0.10
    max_segment_m: float = 0.40      # long legs are split so the return trip re-checks often
    max_replans: int = 15
    # If no path exists, retry with these (clearance, fire margin) pairs, in order.
    # The robot radius itself is never reduced.
    fallback_margins: tuple = ((0.0, 0.10), (0.0, 0.05))
    scan_yaws_deg: tuple = (-60, -30, 0, 30, 60)
    reverify_outbound: bool = True   # look before every leg; the flame may be hidden at the start
    sonar_stop_m: float = 0.15       # must match STOP_CM in the Uno firmware

    # --- D435 mount and filtering ---
    cam_height_m: float = 0.15       # lens height above the floor
    cam_pitch_deg: float = 15.0      # tilt down from horizontal
    cam_forward_m: float = 0.10      # pan axis ahead of the car's center
    hfov_deg: float = 87.0
    min_range_m: float = 0.2
    max_range_m: float = 3.0
    floor_tol_m: float = 0.03        # |z| below this counts as floor
    obstacle_max_z_m: float = 0.5    # ignore anything taller (people, table legs above)
    depth_stride: int = 4            # use every Nth pixel (speed on the Pi)
    discard_frames: int = 3          # let exposure settle after the camera moves

    # --- Flame color threshold (OpenCV HSV: H 0-180, S/V 0-255) ---
    # Defaults are for an orange/red print. Tune with d435_check.py --flame.
    flame_hsv: tuple = (((0, 120, 120), (20, 255, 255)),
                        ((165, 120, 120), (180, 255, 255)))
    flame_min_area_px: int = 150

    # --- Motion (fill in with calibrate.py) ---
    port: str = "/dev/ttyACM0"
    drive_pwm: int = 110
    turn_pwm: int = 110
    speed_mps: float = 0.25          # ground speed at drive_pwm
    turn_rate_dps: float = 180.0     # spin rate at turn_pwm
    settle_s: float = 0.3            # pause after each move so the car stops rocking

    # --- Gyro (MPU6050 on the shield) ---
    use_imu: bool = True             # closed-loop turns + straight-line steering
    imu_sign: int = 1                # flip to -1 if left turns read negative
    turn_lead_deg: float = 5.0       # stop the spin this early to absorb overshoot
    steer_gain: float = 3.0          # PWM per degree of heading error while driving

    # --- Pan servo (Servo 1, pin 11) ---
    pan_enabled: bool = True         # False = scan by turning the whole car
    pan_sign: int = 1                # flip to -1 if a higher angle pans right
    pan_settle_s: float = 0.4

    @classmethod
    def load(cls, path=None):
        # Start from the defaults, then overlay whatever the JSON file sets.
        cfg = cls()
        if path and Path(path).exists():
            data = json.loads(Path(path).read_text())
            # Reject typos: a misspelled key would otherwise be silently ignored.
            known = {f.name for f in fields(cls)}
            unknown = set(data) - known
            if unknown:
                raise ValueError(f"unknown config keys: {sorted(unknown)}")
            # JSON has no tuples, so convert lists back to match the tuple defaults.
            for k, v in data.items():
                setattr(cfg, k, tuple(v) if isinstance(v, list) else v)
        return cfg

    def save(self, path):
        # Write every field, not just overrides, so the file documents the full setup.
        Path(path).write_text(json.dumps(asdict(self), indent=2) + "\n")
