# ======================================================================
# CREATED BY CLAUDE (Anthropic, Claude Code), 2026-09-26. NOT the Codex version.
# [CLAUDE EDIT] New file: run the mission in Isaac Sim 6.1. Full list: CLAUDE_CHANGES.md
# ======================================================================

"""Run the full mission in NVIDIA Isaac Sim 6.1, using Isaac's own Python:

    ~/isaacsim/python.sh pi/run_isaac.py                         # with the GUI
    ~/isaacsim/python.sh pi/run_isaac.py --headless --plot isaac.png
    ~/isaacsim/python.sh pi/run_isaac.py --save-frames frames    # dump camera + flame mask

Same Mission code as run_mission.py; only the car and camera are swapped out.
"""

import argparse
import sys
from pathlib import Path

# Make `rescue` importable however this script is launched.
sys.path.insert(0, str(Path(__file__).resolve().parent))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default="config.json", help="overrides (created by calibrate.py)")
    p.add_argument("--seed", type=int, default=0, help="motion noise seed")
    p.add_argument("--goal", type=float, nargs=2, metavar=("X", "Y"), help="goal in meters")
    p.add_argument("--no-imu", action="store_true", help="dead-reckon heading from turn timing")
    p.add_argument("--telemetry", metavar="HOST[:PORT]", help="laptop running laptop/viewer.py")
    p.add_argument("--plot", metavar="PNG", help="save a picture of the result")
    p.add_argument("--headless", action="store_true", help="no window (servers, CI)")
    p.add_argument("--save-frames", metavar="DIR", help="save every camera frame and flame mask")
    p.add_argument("--render-frames", type=int, default=4,
                   help="renderer updates before each camera read (raise if images lag)")
    p.add_argument("--keep-open", action="store_true", help="leave the window open at the end")
    args = p.parse_args()

    # Isaac requires the app to exist before any other omni / isaacsim import.
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": args.headless})

    from rescue.config import Config
    from rescue.isaac import IsaacBase, IsaacCamera, IsaacScene
    from rescue.mission import Mission, MissionFailed
    from rescue.sim import SimWorld

    cfg = Config.load(args.config)
    if args.goal:
        cfg.goal_xy = tuple(args.goal)
    if args.no_imu:
        cfg.use_imu = False

    tel = None
    if args.telemetry:
        from rescue.telemetry import Telemetry
        host, _, port = args.telemetry.partition(":")
        tel = Telemetry(host, int(port or 5005))

    # SimWorld keeps the true pose, the layout and the collision count; Isaac renders it.
    world = SimWorld.demo(cfg, seed=args.seed)
    scene = IsaacScene(app, world, render_frames=args.render_frames,
                       animate_frames=0 if args.headless else 20)
    if not args.headless:
        try:  # point the GUI viewport at the arena; cosmetic only
            from isaacsim.core.utils.viewports import set_camera_view
            set_camera_view(eye=[cfg.x_min - 1.0, cfg.y_min - 1.5, 2.5],
                            target=[(cfg.x_min + cfg.x_max) / 2, 0.0, 0.0])
        except Exception as e:
            print(f"(viewport not moved: {e})")
    base, camera = IsaacBase(world, scene), IsaacCamera(scene, args.save_frames)

    mission = Mission(cfg, base, camera, telemetry=tel)
    ok = False
    try:
        mission.run()
        ok = True
    except MissionFailed as e:
        print(f"Mission failed: {e}")
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        base.stop()
        if args.plot:
            from rescue.viz import plot_mission
            plot_mission(mission, args.plot, world)
            print(f"Saved {args.plot}")
        print(f"Isaac: {world.collisions} collisions, true end position "
              f"({world.pose.x:.2f}, {world.pose.y:.2f})")
        if args.keep_open and not args.headless:
            while app.is_running():
                app.update()
        app.close()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
