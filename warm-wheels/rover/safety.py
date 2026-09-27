"""Deterministic motor gates. These reduce demo risks, not certify free space."""
from dataclasses import dataclass
import math


@dataclass
class SafetyGate:
    motion_enabled: bool = False
    armed: bool = False
    estop: bool = False
    max_frame_age_s: float = 0.45
    command_lease_s: float = 0.30
    stop_distance_m: float = 0.65
    min_valid_fraction: float = 0.85
    max_pwm: int = 25
    disabled_hint: str = ""

    def environment_reason(self, now, frame_at, clearance, valid_fraction, connected):
        if self.estop:
            return "Emergency stop latched; reset then arm explicitly"
        if not self.motion_enabled:
            return "Motor output disabled at launch" + (f". {self.disabled_hint}" if self.disabled_hint else "")
        if not connected:
            return "Motor link unavailable"
        age = now - frame_at
        if not math.isfinite(age) or age < 0 or age > self.max_frame_age_s:
            return "Camera frame missing or stale"
        if not math.isfinite(valid_fraction) or valid_fraction < self.min_valid_fraction:
            return "Insufficient valid depth in forward view"
        if clearance is None or not math.isfinite(clearance):
            return "Forward clearance unknown"
        if clearance < self.stop_distance_m:
            return "Obstacle within stopping buffer"
        return None

    def output(self, now, frame_at, clearance, valid_fraction, connected, command_at, requested):
        reason = self.environment_reason(now, frame_at, clearance, valid_fraction, connected)
        if reason:
            self.armed = False
            return (0, 0), reason
        if not self.armed:
            return (0, 0), "Disarmed"
        if now - command_at > self.command_lease_s or now < command_at:
            return (0, 0), "Hold a drive control to move"
        left, right = requested
        if any(type(v) is not int or not 0 <= v <= self.max_pwm for v in (left, right)):
            return (0, 0), "Only bounded forward commands are supported"
        # Tight turns sweep unobserved sides; allow only gentle forward arcs.
        if (left or right) and min(left, right) < max(left, right) * 0.45:
            return (0, 0), "Tight turns and reverse require additional sensors"
        return (left, right), "Stopped" if not (left or right) else "Operator hold-to-drive"
