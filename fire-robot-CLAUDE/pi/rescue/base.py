"""Motion primitives on the real car: turn in place, drive straight, pan the camera.

There are no wheel encoders, so distance is timed from a calibrated speed
(see calibrate.py). With use_imu, turns stop on the gyro heading instead of a
timer, and straight moves steer to hold the heading they started with.
The sim has a matching SimBase with the same methods.
"""

import math
import queue
import time

from .uno_link import UnoError


def _wrap_deg(a):
    """Wrap degrees to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


class UnoBase:
    def __init__(self, uno, cfg):
        self.uno = uno
        self.cfg = cfg

    def heading(self):
        """Gyro heading in radians (CCW positive), or None without an IMU."""
        if not self.cfg.use_imu:
            return None
        return math.radians(self._deg())

    def _deg(self):
        """Raw gyro heading in degrees, sign-corrected so left turns are positive."""
        return self.cfg.imu_sign * self.uno.heading()

    def pan(self, yaw):
        """Point the camera `yaw` radians left of straight ahead."""
        if not self.cfg.pan_enabled:
            return
        # Servo 90 = straight ahead; clamp to the servo's 0-180 range.
        angle = 90 + self.cfg.pan_sign * math.degrees(yaw)
        self.uno.pan(int(round(max(0, min(180, angle)))))
        time.sleep(self.cfg.pan_settle_s)

    def turn(self, angle):
        """Spin in place by `angle` radians (positive = left). Returns the angle turned."""
        deg = math.degrees(angle)
        # Expected spin time from calibration: the timer without IMU, a timeout with it.
        timed = abs(deg) / self.cfg.turn_rate_dps
        # UnoLink.spin: positive = clockwise (right)
        spin = -self.cfg.turn_pwm if angle > 0 else self.cfg.turn_pwm

        # No gyro: open loop, spin for the calibrated time and assume it worked.
        if not self.cfg.use_imu:
            self.uno.spin(spin)
            time.sleep(timed)
            self.uno.stop()
            time.sleep(self.cfg.settle_s)
            return angle

        # Gyro: spin until the heading has changed enough, stopping turn_lead_deg early
        # because the car coasts after the motors stop.
        h0 = self._deg()
        target = max(0.0, abs(deg) - self.cfg.turn_lead_deg)
        self.uno.spin(spin)
        # Safety timeout in case the gyro stalls or the car is stuck.
        deadline = time.monotonic() + 2 * timed + 1.0
        while time.monotonic() < deadline and abs(self._deg() - h0) < target:
            time.sleep(0.01)
        self.uno.stop()
        time.sleep(self.cfg.settle_s)
        # Report what the gyro says actually happened, including overshoot.
        return math.radians(self._deg() - h0)

    def move(self, dist):
        """Drive forward `dist` meters. Returns (meters driven, stopped_by_sonar)."""
        cfg = self.cfg
        while not self.uno.events.empty():  # ignore stale events
            self.uno.events.get_nowait()
        # Heading to hold while driving straight.
        hold = self._deg() if cfg.use_imu else None
        try:
            self.uno.forward(cfg.drive_pwm)
        except UnoError as e:
            # The Uno refuses to drive forward when the sonar already sees something close.
            if "BLOCKED" in str(e):
                return 0.0, True
            raise

        # No encoders: drive for as long as the calibrated speed needs to cover dist.
        duration = dist / cfg.speed_mps
        t0 = time.monotonic()
        stopped = False
        try:
            while time.monotonic() - t0 < duration:
                try:
                    # Waiting on the event queue doubles as the loop's ~30 ms tick.
                    ev = self.uno.events.get(timeout=0.03)
                except queue.Empty:
                    ev = None
                if ev and ev.startswith("E STOP_OBSTACLE"):
                    stopped = True
                    break
                if ev and ev.startswith(("E WATCHDOG", "E KEEPALIVE", "E SERIAL")):
                    raise UnoError(f"move interrupted: {ev}")
                if hold is not None:
                    # positive error = drifted right of the heading -> speed up the right side
                    err = _wrap_deg(hold - self._deg())
                    c = max(-40.0, min(40.0, cfg.steer_gain * err))
                    self.uno.drive(cfg.drive_pwm - c, cfg.drive_pwm + c)
        # A BLOCKED reply to a steering command means the sonar tripped mid-move.
        except UnoError as e:
            if "BLOCKED" not in str(e):
                self.uno.stop()
                raise
            stopped = True
        elapsed = time.monotonic() - t0
        self.uno.stop()
        time.sleep(cfg.settle_s)
        # Estimate distance from elapsed time (shorter if the sonar cut the move).
        return min(dist, elapsed * cfg.speed_mps), stopped

    def stop(self):
        self.uno.stop()
