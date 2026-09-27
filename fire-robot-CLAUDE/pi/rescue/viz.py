"""Save a picture of a finished (or failed) mission. Needs matplotlib."""

import numpy as np

from .grid import FIRE, FREE, OCC


def plot_mission(mission, path, world=None):
    import matplotlib
    # Headless backend: works on the Pi and in tests without a display.
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Circle, Rectangle

    g = mission.grid
    # Layer the map from least to most important so later writes win.
    img = np.full(g.state.shape, 0.0)
    img[g.state == FREE] = 1
    img[mission.grid.blocked_mask()] = 2
    img[g.state == OCC] = 3
    img[g.state == FIRE] = 4
    # unknown, free, inflated margin, obstacle, fire
    cmap = ListedColormap(["#d9d9d9", "#ffffff", "#f3d9b1", "#333333", "#e8590c"])
    # Place the image in world meters so the paths plot on top in the same units.
    extent = (g.x0, g.x0 + g.nx * g.res, g.y0, g.y0 + g.ny * g.res)

    fig, ax = plt.subplots(figsize=(10, 7))
    ax.imshow(img, origin="lower", extent=extent, cmap=cmap, vmin=0, vmax=4,
              interpolation="nearest")

    # Sim only: draw the true obstacles, flame and trail for comparison.
    if world is not None:
        for b in world.boxes:
            ax.add_patch(Rectangle((b.x0, b.y0), b.x1 - b.x0, b.y1 - b.y0,
                                   fill=False, ec="#1971c2", lw=1.5, ls="--"))
        if world.flame:
            ax.add_patch(Circle(world.flame, world.flame_r, fill=False, ec="#c92a2a", lw=1.5, ls="--"))
        t = np.array(world.trail)
        ax.plot(t[:, 0], t[:, 1], color="#1971c2", lw=1, alpha=0.6, label="true trail (sim)")

    def line(pts, **kw):
        """Plot a polyline if it has at least two points."""
        if len(pts) > 1:
            p = np.array(pts)
            ax.plot(p[:, 0], p[:, 1], **kw)

    line(mission.planned, color="#2f9e44", lw=2, ls=":", label="planned")
    line(mission.driven_out, color="#2f9e44", lw=2, marker=".", label="driven out (estimated)")
    line(mission.driven_back, color="#9c36b5", lw=2, marker=".", label="driven back (estimated)")
    ax.plot(*mission.start, "ko", ms=8)
    ax.plot(*mission.goal, "k*", ms=14)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Mission: phase={mission.phase}, replans={mission.replans}")
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
