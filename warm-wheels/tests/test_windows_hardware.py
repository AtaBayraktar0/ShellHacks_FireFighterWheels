import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from scripts.firmware import compile_command, validate_upload_port, verification_flags
from scripts.hardware_preflight import arduino_readiness, create_report, enumerate_realsense, enumerate_serial_ports


def port(name="COM7", vid=0x1A86, pid=0x7523, description="USB-SERIAL CH340"):
    return SimpleNamespace(device=name, vid=vid, pid=pid, description=description, hwid="USB")


class HardwarePreflightTests(unittest.TestCase):
    def test_enumeration_does_not_open_serial_ports_and_rejects_bluetooth_candidate(self):
        bluetooth = port("COM5", None, None, "Standard Serial over Bluetooth link")
        with patch("serial.Serial", side_effect=AssertionError("Must not open serial")):
            report = enumerate_serial_ports(lambda: [port(), bluetooth])
        self.assertEqual(report["status"], "ok")
        self.assertEqual(report["ports"][0]["kind"], "bluetooth")
        self.assertFalse(report["ports"][0]["uno_candidate"])
        self.assertTrue(report["ports"][1]["uno_candidate"])

    def test_unknown_usb_id_is_not_automatically_an_uno(self):
        report = enumerate_serial_ports(lambda: [port(vid=0x1234, pid=0x5678)])
        self.assertFalse(report["ports"][0]["uno_candidate"])

    def test_realsense_enumeration_omits_serial_and_does_not_start_pipeline(self):
        seen = []

        class Device:
            def supports(self, key):
                return True
            def get_info(self, key):
                seen.append(key)
                return {"name": "Intel RealSense D435I", "usb": "3.2", "firmware": "5.test"}[key]

        rs = SimpleNamespace(camera_info=SimpleNamespace(name="name", usb_type_descriptor="usb", firmware_version="firmware"),
                             context=lambda: SimpleNamespace(query_devices=lambda: [Device()]))
        report = enumerate_realsense(rs)
        self.assertEqual(report["status"], "connected")
        self.assertNotIn("serial", seen)
        self.assertEqual(set(report["devices"][0]), {"name", "usb_connection", "firmware"})

    def test_missing_devices_are_not_reported_as_completed_hardware_validation(self):
        report = create_report(packages={"test": {"status": "ready", "version": "1"}},
            serial={"status": "ok", "ports": []}, realsense={"status": "not_connected", "devices": []},
            arduino={"status": "ready", "version": "1.5", "uno_core": "1.8"})
        self.assertTrue(report["runtime_ready"])
        self.assertEqual(report["status"], "software_ready_devices_not_connected")
        self.assertIn("Motor commands", report["not_performed"])

    def test_cli_core_query_reads_only_installed_version(self):
        calls = []
        def runner(command, **kwargs):
            calls.append(command)
            text = "arduino-cli Version: 1.5.0" if "version" in command else json.dumps({"platforms": [{"id": "arduino:avr", "installed_version": "1.8.8"}]})
            return SimpleNamespace(returncode=0, stdout=text, stderr="")
        report = arduino_readiness("arduino-cli.exe", runner)
        self.assertEqual(report, {"status": "ready", "version": "1.5.0", "uno_core": "1.8.8"})
        self.assertTrue(all("upload" not in command for command in calls))


class FirmwareWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.inventory = enumerate_serial_ports(lambda: [port()])

    def test_unverified_compile_is_explicitly_motor_disabled(self):
        command = compile_command("cli", "build", "output", False)
        self.assertIn("compiler.cpp.extra_flags=-DMOTOR_PROFILE=1 -DHARDWARE_VERIFIED=0", command)
        self.assertNotIn("upload", command)

    def test_verified_compile_requires_both_operator_flags(self):
        for pin, raised in ((False, False), (True, False), (False, True)):
            with self.subTest(pin=pin, raised=raised), self.assertRaises(ValueError):
                verification_flags(True, pin, raised)
        self.assertEqual(verification_flags(True, True, True), 1)
        self.assertEqual(verification_flags(False, False, False), 0)

    def test_upload_requires_explicit_same_present_com_and_board_confirmation(self):
        self.assertEqual(validate_upload_port("com7", "COM7", True, self.inventory)["port"], "COM7")
        for selected, confirmed, board in ((None, None, True), ("COM7", "COM8", True),
                                           ("COM8", "COM8", True), ("COM7", "COM7", False),
                                           ("COM7 & run", "COM7 & run", True)):
            with self.subTest(selected=selected, confirmed=confirmed, board=board), self.assertRaises(ValueError):
                validate_upload_port(selected, confirmed, board, self.inventory)

    def test_bluetooth_is_never_an_upload_target(self):
        inventory = enumerate_serial_ports(lambda: [port("COM5", None, None, "Bluetooth Serial")])
        with self.assertRaises(ValueError):
            validate_upload_port("COM5", "COM5", True, inventory)

    def test_launchers_anchor_working_directory_and_project_python(self):
        root = Path(__file__).resolve().parents[1]
        for filename in ("hardware_preflight.cmd", "firmware_compile.cmd", "firmware_upload.cmd"):
            with self.subTest(filename=filename):
                content = (root / filename).read_text()
                self.assertIn('cd /d "%~dp0"', content)
                self.assertIn('".venv\\Scripts\\python.exe" -m scripts.', content)
                self.assertNotIn("ROVER_TOKEN", content)


if __name__ == "__main__":
    unittest.main()
