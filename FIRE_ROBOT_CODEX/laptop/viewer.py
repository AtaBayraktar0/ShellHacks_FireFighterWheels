"""Live demo screen: shows the robot's map, path and position as it drives.

On the laptop (same WiFi as the Pi):
    pip install matplotlib numpy
    python3 viewer.py                 # listens on UDP 5005
Then on the Pi:
    python3 run_mission.py --telemetry <laptop-ip>
"""

import argparse
import json
import socket

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=5005)
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    # Non-blocking so the plot window stays responsive between packets.
    sock.setblocking(False)
    print(f"Listening on UDP {args.port}...")

    # Latest map and robot state received.
    grid, state = None, None
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 7))
    cmap = ListedColormap(["#d9d9d9", "#ffffff", "#333333", "#e8590c"])  # unknown free occ fire

    while plt.fignum_exists(fig.number):
        dirty = False
        # Drain every waiting packet; only the newest of each type matters.
        while True:
            try:
                data, _ = sock.recvfrom(65535)
            except BlockingIOError:
                break
            msg = json.loads(data)
            if msg["type"] == "grid":
                grid = msg
            elif msg["type"] == "state":
                state = msg
            dirty = True

        # Redraw only when something new arrived.
        if dirty:
            ax.clear()
            if grid:
                # Rebuild the grid image from the index lists (unlisted cells stay unknown = 0).
                img = np.zeros(grid["nx"] * grid["ny"])
                img[grid["free"]] = 1
                img[grid["occ"]] = 2
                img[grid["fire"]] = 3
                ext = (grid["x0"], grid["x0"] + grid["nx"] * grid["res"],
                       grid["y0"], grid["y0"] + grid["ny"] * grid["res"])
                ax.imshow(img.reshape(grid["ny"], grid["nx"]), origin="lower", extent=ext,
                          cmap=cmap, vmin=0, vmax=3, interpolation="nearest")
            if state:
                for key, style in (("planned", dict(color="#2f9e44", ls=":", lw=2)),
                                   ("driven_out", dict(color="#2f9e44", lw=2, marker=".")),
                                   ("driven_back", dict(color="#9c36b5", lw=2, marker="."))):
                    pts = np.array(state[key])
                    if len(pts) > 1:
                        ax.plot(pts[:, 0], pts[:, 1], label=key, **style)
                # Arrow at the robot's position, pointing along its heading.
                x, y, th = state["pose"]
                ax.arrow(x, y, 0.12 * np.cos(th), 0.12 * np.sin(th), width=0.02, color="#1971c2")
                ax.set_title(f"phase: {state['phase']}")
                ax.legend(loc="upper left", fontsize=8)
            ax.set_aspect("equal")
            fig.canvas.draw_idle()
        plt.pause(0.1)


if __name__ == "__main__":
    main()
