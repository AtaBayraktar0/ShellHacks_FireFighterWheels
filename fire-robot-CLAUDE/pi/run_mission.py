"""Run the full mission: scan, plan, drive to goal, retrace to start.

    python3 run_mission.py --sim --plot sim.png          # laptop, no hardware
    python3 run_mission.py --config config.json --telemetry 192.168.1.20
"""

import argparse
import sys

from rescue.config import Config
from rescue.mission import Mission, MissionFailed


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default="config.json", help="overrides (created by calibrate.py)")
    p.add_argument("--sim", action="store_true", help="simulated car and camera")
    p.add_argument("--seed", type=int, default=0, help="sim noise seed")
    p.add_argument("--goal", type=float, nargs=2, metavar=("X", "Y"), help="goal in meters")
    p.add_argument("--no-imu", action="store_true", help="dead-reckon heading from turn timing")
    p.add_argument("--telemetry", metavar="HOST[:PORT]", help="laptop running laptop/viewer.py")
    p.add_argument("--plot", metavar="PNG", help="save a picture of the result")
    args = p.parse_args()

    cfg = Config.load(args.config)
    # Command-line flags override the config file.
    if args.goal:
        cfg.goal_xy = tuple(args.goal)
    if args.no_imu:
        cfg.use_imu = False

    tel = None
    if args.telemetry:
        from rescue.telemetry import Telemetry
        # Accept HOST or HOST:PORT.
        host, _, port = args.telemetry.partition(":")
        tel = Telemetry(host, int(port or 5005))

    world = uno = camera = None
    # Pick the sim or the real hardware; the Mission code is the same either way.
    if args.sim:
        from rescue.sim import SimBase, SimCamera, SimWorld
        world = SimWorld.demo(cfg, seed=args.seed)
        base, camera = SimBase(world), SimCamera(world)
    else:
        from rescue.base import UnoBase
        from rescue.perception import D435Camera
        from rescue.uno_link import UnoLink
        uno = UnoLink(cfg.port, on_event=lambda e: print(f"  [uno] {e}")).open()
        base = UnoBase(uno, cfg)
        camera = D435Camera(cfg)

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
        # Always stop the motors and release hardware, even after a failure.
        base.stop()
        if uno:
            uno.close()
        if camera and hasattr(camera, "close"):
            camera.close()
        if args.plot:
            from rescue.viz import plot_mission
            plot_mission(mission, args.plot, world)
            print(f"Saved {args.plot}")
        if world:
            print(f"Sim: {world.collisions} collisions, true end position "
                  f"({world.pose.x:.2f}, {world.pose.y:.2f})")
    # Nonzero exit code if the mission failed.
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
