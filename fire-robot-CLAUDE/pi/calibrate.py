"""Measure how far and how fast the car actually moves, and save it to config.json.

The car has no wheel encoders, so every move is timed. Run this on the demo
surface (carpet vs. table changes the numbers) with the battery charged.

    python3 calibrate.py              # drive test, then spin test
"""

import argparse
import time

from rescue.config import Config
from rescue.uno_link import UnoLink


def ask(prompt):
    """Prompt until the user types a number."""
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("Enter a number.")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default="config.json")
    p.add_argument("--drive-s", type=float, default=2.0)
    p.add_argument("--spin-s", type=float, default=2.0)
    args = p.parse_args()
    cfg = Config.load(args.config)

    with UnoLink(cfg.port, on_event=lambda e: print(f"  [uno] {e}")) as uno:
        # Drive test: speed = measured distance / drive time.
        input(f"DRIVE: mark the car's position, clear {cfg.drive_pwm}-PWM path ahead. Enter...")
        uno.forward(cfg.drive_pwm)
        time.sleep(args.drive_s)
        uno.stop()
        cm = ask("Distance driven (cm): ")
        cfg.speed_mps = cm / 100 / args.drive_s

        # Spin test: turn rate = measured angle / spin time.
        input("SPIN: mark the car's heading. Enter...")
        uno.spin(cfg.turn_pwm)
        time.sleep(args.spin_s)
        uno.stop()
        deg = ask("Degrees turned, counting full turns (e.g. 400): ")
        cfg.turn_rate_dps = deg / args.spin_s

    cfg.save(args.config)
    print(f"speed_mps={cfg.speed_mps:.3f}, turn_rate_dps={cfg.turn_rate_dps:.0f} -> {args.config}")


if __name__ == "__main__":
    main()
