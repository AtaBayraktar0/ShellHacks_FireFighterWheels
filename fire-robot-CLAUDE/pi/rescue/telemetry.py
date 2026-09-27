"""Small JSON messages over UDP to the laptop running laptop/viewer.py.

UDP is fire-and-forget: if the laptop isn't listening, nothing breaks.
"""

import json
import socket

import numpy as np

from .grid import FIRE, FREE, OCC


class Telemetry:
    def __init__(self, host, port=5005):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, kind, **data):
        # Compact separators keep each message small enough for one datagram.
        msg = json.dumps({"type": kind, **data}, separators=(",", ":")).encode()
        try:
            self.sock.sendto(msg, self.addr)
        except OSError:
            pass  # laptop off the network; keep driving

    def grid(self, grid):
        # Send only indices of known cells; the viewer treats the rest as unknown.
        flat = grid.state.reshape(-1)
        self.send("grid", nx=grid.nx, ny=grid.ny, res=grid.res, x0=grid.x0, y0=grid.y0,
                  free=np.flatnonzero(flat == FREE).tolist(),
                  occ=np.flatnonzero(flat == OCC).tolist(),
                  fire=np.flatnonzero(flat == FIRE).tolist())

    def state(self, phase, pose, planned, driven_out, driven_back):
        # Round to millimeters to keep the JSON short.
        r = lambda pts: [[round(x, 3), round(y, 3)] for x, y in pts]
        self.send("state", phase=phase,
                  pose=[round(pose.x, 3), round(pose.y, 3), round(pose.theta, 3)],
                  planned=r(planned), driven_out=r(driven_out), driven_back=r(driven_back))
