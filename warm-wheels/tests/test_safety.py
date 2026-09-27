import math
import pytest
from rover.safety import SafetyGate


def run(gate, **changes):
    fields = dict(now=10., frame_at=9.9, clearance=2., valid_fraction=.98,
                  connected=True, command_at=9.95, requested=(25, 25))
    fields.update(changes)
    return gate.output(**fields)


@pytest.mark.parametrize("changes", [
    {"frame_at": 9.0}, {"frame_at": 11.}, {"frame_at": float("nan")},
    {"clearance": None}, {"clearance": .5}, {"clearance": math.nan},
    {"valid_fraction": .2}, {"valid_fraction": math.nan}, {"connected": False},
])
def test_environment_fault_stops_and_disarms(changes):
    gate = SafetyGate(motion_enabled=True, armed=True)
    assert run(gate, **changes)[0] == (0, 0)
    assert not gate.armed


def test_network_loss_stops_without_new_command():
    gate = SafetyGate(motion_enabled=True, armed=True)
    assert run(gate)[0] == (25, 25)
    assert run(gate, command_at=9.0)[0] == (0, 0)


@pytest.mark.parametrize("command", [(-10, -10), (0, 25), (26, 25), (True, 25), (10.2, 20)])
def test_bounded_forward_only(command):
    assert run(SafetyGate(motion_enabled=True, armed=True), requested=command)[0] == (0, 0)


def test_estop_and_disabled_output_override_arm():
    assert run(SafetyGate(motion_enabled=True, armed=True, estop=True))[0] == (0, 0)
    assert run(SafetyGate(motion_enabled=False, armed=True))[0] == (0, 0)


def test_gentle_forward_arc():
    assert run(SafetyGate(motion_enabled=True, armed=True), requested=(12,25))[0] == (12,25)
