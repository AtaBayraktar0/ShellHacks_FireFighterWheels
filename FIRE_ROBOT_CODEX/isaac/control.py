"""Created by OpenAI Codex: mission adapter controlling physical wheel joints.

Distance remains time-based, as on the real robot. Heading uses an ideal
simulated gyro (physics yaw). Ground-truth XY is never fed to the mission.
"""
import math
from rescue.geometry import wrap
from rescue.mission import MissionFailed


class IsaacBase:
    def __init__(self, runtime, cfg):
        self.rt, self.cfg = runtime, cfg
        self.camera = None

    def heading(self):
        return self.rt.pose()[2]

    def pan(self, yaw):
        if self.camera is not None:
            self.camera.pan(yaw)
        self.rt.wait(self.cfg.pan_settle_s)

    def stop(self):
        self.rt.command(0.,0.)

    def turn(self, angle):
        initial = previous = self.heading()
        turned = 0.
        deadline = self.rt.time() + 4. + abs(angle)/.25
        try:
            while abs(angle-turned) > math.radians(1.):
                if self.rt.time() > deadline:
                    raise MissionFailed('Isaac turn timed out: check wheel joints/friction/gyro')
                error = angle-turned
                omega = math.copysign(min(.9,max(.12,2.*abs(error))),error)
                self.rt.command(0.,omega)
                self.rt.step()
                current = self.heading()
                turned += wrap(current-previous)
                previous = current
        finally:
            self.stop()
        self.rt.wait(self.cfg.settle_s)
        return turned + wrap(self.heading()-previous)

    def move(self, distance):
        hold = self.heading()
        start = self.rt.time()
        stopped = False
        elapsed = 0.
        try:
            while elapsed < distance/self.cfg.speed_mps:
                if self.rt.sonar_distance() < self.cfg.sonar_stop_m:
                    stopped = True
                    break
                error = wrap(hold-self.heading())
                self.rt.command(self.cfg.speed_mps,max(-.6,min(.6,3.*error)))
                self.rt.step()
                elapsed = self.rt.time()-start
        finally:
            self.stop()
        moved = min(distance,elapsed*self.cfg.speed_mps)
        self.rt.wait(self.cfg.settle_s)
        return moved,stopped
