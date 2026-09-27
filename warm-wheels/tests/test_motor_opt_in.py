"""Physical motor output is an explicit launch opt-in; the safety gates stay in force."""
import time

import pytest

from rover import app as rover_app
from rover.runtime import RoverRuntime
from rover.safety import SafetyGate
from rover.serial_link import SimulationMotor
from scripts import launch


def test_hardware_defaults_to_camera_only():
    args = rover_app.parse_args(["--mode", "hardware"], environ={})
    assert args.enable_motors is False
    assert args.port == "/dev/ttyACM0"
    assert "OFF (camera-only)" in rover_app.motor_banner(args)


def test_cli_flag_enables_motors_in_hardware_mode():
    args = rover_app.parse_args(["--mode", "hardware", "--enable-motors", "--port", "/dev/ttyUSB0"], environ={})
    assert args.enable_motors is True and args.port == "/dev/ttyUSB0"
    banner = rover_app.motor_banner(args)
    assert "ENABLED on /dev/ttyUSB0" in banner and "Arm" in banner


@pytest.mark.parametrize("value", ["1", "true", "YES", " on "])
def test_environment_opt_in(value):
    env = {rover_app.ENABLE_MOTORS_ENV: value, rover_app.SERIAL_PORT_ENV: "/dev/serial/by-id/uno"}
    args = rover_app.parse_args(["--mode", "hardware"], environ=env)
    assert args.enable_motors is True and args.port == "/dev/serial/by-id/uno"


@pytest.mark.parametrize("value", ["", "0", "false", "off", "no"])
def test_environment_falsey_values_keep_motors_off(value):
    args = rover_app.parse_args(["--mode", "hardware"], environ={rover_app.ENABLE_MOTORS_ENV: value})
    assert args.enable_motors is False


def test_ambiguous_environment_value_is_rejected():
    with pytest.raises(SystemExit):
        rover_app.parse_args(["--mode", "hardware"], environ={rover_app.ENABLE_MOTORS_ENV: "maybe"})


def test_cli_port_overrides_environment_port():
    env = {rover_app.SERIAL_PORT_ENV: "/dev/ttyACM1"}
    args = rover_app.parse_args(["--mode", "hardware", "--port", "/dev/ttyACM2"], environ=env)
    assert args.port == "/dev/ttyACM2"


@pytest.mark.parametrize("argv,env", [
    (["--enable-motors"], {}),
    (["--mode", "demo"], {rover_app.ENABLE_MOTORS_ENV: "1"}),
])
def test_motor_opt_in_rejected_outside_hardware_mode(argv, env):
    with pytest.raises(SystemExit):
        rover_app.parse_args(argv, environ=env)


def test_camera_only_block_reason_explains_how_to_enable():
    runtime = RoverRuntime("hardware", enable_motors=False, camera=object(), motor=SimulationMotor())
    runtime.motor.connect()
    runtime.frame_at = time.monotonic()
    runtime.camera_ok = True
    runtime.spatial.update(clearance_m=2.0, depth_valid_fraction=1.0)
    state = runtime.snapshot()
    assert state["motion_enabled"] is False and state["can_arm"] is False
    assert state["block_reason"].startswith("Motor output disabled at launch")
    assert "--enable-motors" in state["block_reason"] and rover_app.ENABLE_MOTORS_ENV in state["block_reason"]
    with pytest.raises(ValueError, match="--enable-motors"):
        runtime.command("arm")
    assert runtime.gate.armed is False


def test_enabled_motors_still_require_arm_and_fresh_clear_depth():
    gate = SafetyGate(motion_enabled=True)
    now = 100.0
    assert gate.output(now, now, 2.0, 1.0, True, now, (20, 20)) == ((0, 0), "Disarmed")
    gate.armed = True
    assert gate.output(now, now, 0.3, 1.0, True, now, (20, 20))[1] == "Obstacle within stopping buffer"
    assert gate.armed is False
    gate.armed = True
    assert gate.output(now, now - 1.0, 2.0, 1.0, True, now, (20, 20))[1] == "Camera frame missing or stale"
    gate.armed = True
    assert gate.output(now, now, 2.0, 1.0, True, now - 1.0, (20, 20))[1] == "Hold a drive control to move"
    assert gate.output(now, now, 2.0, 1.0, True, now, (60, 60))[1] == "Only bounded forward commands are supported"
    gate.estop = True
    assert gate.output(now, now, 2.0, 1.0, True, now, (20, 20))[0] == (0, 0)


def test_launcher_opens_drive_console_only_when_motors_enabled():
    assert launch.landing_path({"motors": True}, "hardware") == "/"
    assert launch.landing_path({"motors": False}, "hardware") == "/scan"
    assert launch.landing_path({}, "hardware") == "/scan"
    assert launch.landing_path({"motors": True}, "demo") == "/presentation"


def test_launcher_environment_opt_in_still_requires_explicit_port(monkeypatch, capsys):
    monkeypatch.setenv(launch.ENABLE_MOTORS_ENV, "1")
    monkeypatch.delenv(launch.SERIAL_PORT_ENV, raising=False)
    monkeypatch.setattr(launch.os, "chdir", lambda path: None)
    with pytest.raises(SystemExit):
        launch.main(["--mode", "hardware", "--no-browser"])
    assert "explicit --port" in capsys.readouterr().err
