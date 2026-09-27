"""Phase 2 sanity check: does pyrealsense2 open the D435 and return depth?

    python3 d435_check.py            # frame rate + center distance
    python3 d435_check.py --floor    # also estimate cam_height_m / cam_pitch_deg
    python3 d435_check.py --flame    # also save flame mask images for HSV tuning

Point the camera at open floor for --floor, and at the flame prop for --flame.
"""

import argparse
import time

import numpy as np

from rescue.config import Config
from rescue.perception import D435Camera, fit_floor, flame_mask


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default="config.json")
    p.add_argument("--floor", action="store_true")
    p.add_argument("--flame", action="store_true")
    args = p.parse_args()
    cfg = Config.load(args.config)

    cam = D435Camera(cfg)
    try:
        # Time 30 reads; each read also discards cfg.discard_frames frames.
        t0 = time.monotonic()
        for _ in range(30):
            depth, color = cam.frames()
        fps = 30 * (cfg.discard_frames + 1) / (time.monotonic() - t0)
        h, w = depth.shape
        # Median of a small center patch, ignoring zero (invalid) depth.
        center = depth[h // 2 - 5:h // 2 + 5, w // 2 - 5:w // 2 + 5]
        valid = center[center > 0]
        print(f"OK: depth {w}x{h}, ~{fps:.0f} fps")
        print(f"Center distance: {np.median(valid):.3f} m" if valid.size else
              "Center distance: no valid depth (too close, or nothing in range)")
        print(f"Valid depth pixels: {100 * np.mean(depth > 0):.0f}%")

        if args.floor:
            # bottom-middle band of the image is usually floor
            band = np.zeros_like(depth, dtype=bool)
            band[int(h * 0.75):, int(w * 0.3):int(w * 0.7)] = True
            pts = cam.optical_points(depth, band)
            if len(pts) < 50:
                print("Floor: not enough valid points in the lower image")
            else:
                height, pitch = fit_floor(pts)
                print(f"Floor fit from {len(pts)} points:")
                print(f'  "cam_height_m": {height:.3f}, "cam_pitch_deg": {pitch:.1f}')

        if args.flame:
            import cv2
            mask = flame_mask(color, cfg)
            # Save the raw image and mask so the HSV range can be tuned by eye.
            cv2.imwrite("flame_color.png", color)
            cv2.imwrite("flame_mask.png", (mask * 255).astype(np.uint8))
            hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
            print(f"Flame pixels: {int(mask.sum())} (min blob {cfg.flame_min_area_px})")
            if mask.any():
                # Report the typical color and distance of the detected pixels.
                d = depth[mask & (depth > 0)]
                med = np.median(hsv[mask], axis=0).astype(int)
                print(f"  median HSV {tuple(med)}, distance {np.median(d):.2f} m" if d.size
                      else f"  median HSV {tuple(med)}, no depth on it")
            print("Saved flame_color.png and flame_mask.png")
    finally:
        cam.close()


if __name__ == "__main__":
    main()
