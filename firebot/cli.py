"""Independent checks in the project's integration order."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from .planning import Grid
from .vision import Camera, Pose, flame_mask, update_map, save_cloud
from .motors import Motors
from .mission import run_live
from .simulation import simulate


def load_config(path):
    """Load settings and reject unsafe or broken values."""
    cfg = json.loads(Path(path).read_text(encoding="utf-8"))
    positive = ("width_cells", "height_cells", "cell_m", "robot_radius_m", "map_max_age_s",
                "camera_height_m", "floor_tolerance_m", "min_depth_m", "max_depth_m",
                "pose_max_age_s", "pwm", "pulse_ms", "max_pulse_travel_m")
    for name in positive:
        if not isinstance(cfg[name], (int, float)) or not math.isfinite(cfg[name]) or cfg[name] <= 0:
            raise ValueError(f"{name} must be positive and finite")
    for name in ("width_cells", "height_cells", "pwm", "pulse_ms"):
        if type(cfg[name]) is not int:
            raise ValueError(f"{name} must be an integer")
    if cfg["pwm"] > 120 or cfg["pulse_ms"] > 200:
        raise ValueError("PWM or pulse duration exceeds firmware limits")
    if cfg["max_pulse_travel_m"] > cfg["cell_m"] / 2:
        raise ValueError("Pulse travel must be at most half a cell")
    if cfg["min_depth_m"] >= cfg["max_depth_m"]:
        raise ValueError("Depth range is reversed")
    for name in ("camera_pitch_deg", "camera_forward_m"):
        if not math.isfinite(cfg[name]):
            raise ValueError(f"{name} must be finite")
    if not 0 <= cfg["camera_pitch_deg"] < 90 or type(cfg["calibrated"]) is not bool:
        raise ValueError("Invalid camera pitch or calibrated flag")
    for key in ("start_cell", "goal_cell"):
        cell = cfg[key]
        if len(cell) != 2 or any(type(v) is not int for v in cell):
            raise ValueError(f"Invalid {key}")
        if not (0 <= cell[0] < cfg["width_cells"] and 0 <= cell[1] < cfg["height_cells"]):
            raise ValueError(f"{key} is outside map")
    return cfg


def main():
    """Read the command and start the requested robot tool."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["simulate", "camera-test", "serial-test", "map-scan", "flame-test", "run-live"])
    parser.add_argument("--config", default="config.json")
    parser.add_argument("--output", default="runs/latest")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--pose-file")
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--block-return", action="store_true")
    parser.add_argument("--yaw-deg", type=float, default=0)
    args = parser.parse_args()
    cfg = load_config(args.config)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    grid = Grid(cfg["width_cells"], cfg["height_cells"], cfg["cell_m"])
    if args.command == "simulate":
        # This proves the algorithm only; it does not open any hardware.
        print(simulate(grid, cfg, output, args.block_return))
        return
    if args.command == "serial-test":
        # Motion only happens when the user adds --enable-motion.
        with Motors(args.port) as motors:
            if args.enable_motion:
                motors.pulse("FWD", cfg["pwm"], cfg["pulse_ms"])
        print("Serial test passed" + ("; forward pulse sent" if args.enable_motion else "; STOP only"))
        return
    if args.command == "run-live":
        # Real driving needs all three safety requirements.
        if not args.enable_motion or not args.pose_file or not cfg["calibrated"]:
            parser.error("run-live requires --enable-motion, --pose-file, and calibrated config")
        with Motors(args.port) as motors, Camera() as camera:
            print(run_live(camera, motors, grid, cfg, args.pose_file, output))
        return
    with Camera() as camera:
        # Camera tools save their results so they can be inspected later.
        frame = camera.read()
        np.savez_compressed(output / "frame.npz", xyz=frame.xyz, bgr=frame.bgr)
        save_cloud(output / "cloud.ply", frame.xyz, frame.bgr)
        valid = np.isfinite(frame.xyz).all(axis=-1) & (frame.xyz[..., 2] > 0)
        if valid.sum() < 100:
            raise RuntimeError("Camera returned too little valid depth")
        print(f"Valid depth: {valid.mean():.1%}")
        if args.command in {"flame-test", "map-scan"}:
            import cv2
            mask = flame_mask(frame.bgr)
            preview = frame.bgr.copy()
            preview[mask] = (255, 0, 255)
            if not cv2.imwrite(str(output / "flame-overlay.png"), preview):
                raise RuntimeError("Could not save overlay")
            print(f"Flame-color pixels: {mask.sum()}")
            if args.command == "map-scan":
                x, y = grid.center(tuple(cfg["start_cell"]))
                update_map(grid, frame, Pose(x, y, math.radians(args.yaw_deg)), cfg, mask)
                np.savez_compressed(output / "map.npz", blocked=grid.blocked, seen=grid.seen,
                                    safe=grid.safe(frame.captured, cfg["map_max_age_s"], cfg["robot_radius_m"]))
        print(f"Saved results to {output.resolve()}")


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError, OSError, ImportError) as exc:
        raise SystemExit(f"Stopped: {exc}") from exc
