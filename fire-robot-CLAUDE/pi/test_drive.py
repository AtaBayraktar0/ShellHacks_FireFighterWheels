"""Throwaway serial test: forward, stop, spin, stop.

Run with the wheels off the ground first:
    python3 test_drive.py [--port /dev/ttyACM0] [--speed 120]
"""

import argparse
import time

from rescue.uno_link import UnoLink


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--speed", type=int, default=120)
    args = p.parse_args()

    # Print Uno events (e.g. obstacle stops) as they arrive.
    with UnoLink(args.port, on_event=lambda e: print(f"  [uno] {e}")) as uno:
        print(f"Connected on {args.port}. Ultrasonic: {uno.distance()} cm")
        input("Battery on, wheels up or path clear? Press Enter to start...")

        print("Forward 1 s (all four wheels forward)")
        uno.forward(args.speed)
        time.sleep(1.0)
        uno.stop()
        time.sleep(0.5)

        print("Spin right 0.5 s (left wheels forward, right wheels back)")
        uno.spin(args.speed)
        time.sleep(0.5)
        uno.stop()

        print("Done.")


if __name__ == "__main__":
    main()
