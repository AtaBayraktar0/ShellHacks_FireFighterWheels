"""Safety behavior with a byte-oriented fake serial device (no motors)."""
import unittest
from unittest.mock import patch

from rover.serial_link import MotorLinkError, SerialMotor, SimulationMotor


class FakeSerial:
    def __init__(self, *args, profile="PROFILE V4 VERIFIED", **kwargs):
        self.rx = bytearray(f"READY WN1\n{profile}\n".encode())
        self.writes = []
        self.closed = False
        self.estop = False
        self.response_override = None
        self.partial_write = False
        self.read_error = False

    def read(self, count):
        if self.read_error:
            raise OSError("USB disconnected")
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data

    def write(self, data):
        self.writes.append(data)
        if self.partial_write:
            return max(0, len(data) - 1)
        if self.response_override is not None:
            self.rx.extend(self.response_override)
            return len(data)
        if data == b"S\n":
            self.rx.extend(f"STOPPED {int(self.estop)}\n".encode())
        elif data == b"E\n":
            self.estop = True
            self.rx.extend(b"ESTOP 1\n")
        elif data == b"R\n":
            self.estop = False
            self.rx.extend(b"ESTOP 0\n")
        elif data.startswith(b"M "):
            self.rx.extend(b"ERR ESTOP\n" if self.estop else b"ACK " + data[2:])
        return len(data)

    def close(self):
        self.closed = True


class SerialMotorTests(unittest.TestCase):
    def create(self, profile="PROFILE V4 VERIFIED"):
        fake = FakeSerial(profile=profile)
        motor = SerialMotor("test", serial_factory=lambda *a, **k: fake,
                            handshake_timeout=0.01, ack_timeout=0.003)
        return motor, fake

    def assert_failed_safe(self, motor, fake):
        self.assertFalse(motor.status()["connected"])
        self.assertFalse(motor.status()["motors_enabled"])
        self.assertTrue(motor.status()["estop"])
        self.assertTrue(motor.status()["error"])
        self.assertEqual(fake.writes[-1], b"\nE\nS\n")
        self.assertTrue(fake.closed)

    def test_handshake_requires_verified_profile_before_any_motion(self):
        for profile in ("PROFILE DISABLED UNVERIFIED", "PROFILE V4 UNVERIFIED"):
            with self.subTest(profile=profile):
                motor, fake = self.create(profile)
                with self.assertRaisesRegex(MotorLinkError, "unverified(.|\n)*HARDWARE_VERIFIED=1"):
                    motor.connect()
                self.assert_failed_safe(motor, fake)
                self.assertFalse(any(data.startswith(b"M ") for data in fake.writes))

    def test_wrong_protocol_never_connects(self):
        motor, fake = self.create()
        fake.rx = bytearray(b"READY WN0\nPROFILE V4 VERIFIED\n")
        with self.assertRaises(MotorLinkError):
            motor.connect()
        self.assert_failed_safe(motor, fake)

    def test_drive_requires_matching_ack_and_echo(self):
        motor, fake = self.create()
        motor.connect()
        motor.drive(23, -17)
        self.assertEqual(fake.writes, [b"S\n", b"M 1 23 -17\n"])
        self.assertEqual((motor.status()["left"], motor.status()["right"]), (23, -17))
        motor.close()
        self.assertEqual(fake.writes[-1], b"S\n")

    def test_l298_verified_handshake(self):
        motor, _ = self.create("PROFILE L298N VERIFIED")
        motor.connect()
        self.assertEqual(motor.status()["profile"], "L298N")

    def test_close_failure_still_clears_connection_state(self):
        motor, fake = self.create()
        motor.connect()
        with patch.object(fake, "close", side_effect=OSError("close failed")):
            with self.assertRaisesRegex(MotorLinkError, "close failed"):
                motor.close()
        self.assertFalse(motor.status()["connected"])
        self.assertFalse(motor.status()["motors_enabled"])

    def test_stale_sequence_mismatched_echo_and_reset_are_failures(self):
        for response in (b"ACK 0 20 20\n", b"ACK 1 20 0\n", b"READY WN1\n",
                         b"ERR WATCHDOG\n", b"ERR ESTOP\n"):
            with self.subTest(response=response):
                motor, fake = self.create()
                motor.connect()
                fake.response_override = response
                with self.assertRaises(MotorLinkError):
                    motor.drive(20, 20)
                self.assert_failed_safe(motor, fake)

    def test_missing_and_partial_ack_do_not_count_as_success(self):
        for response in (b"", b"ACK 1 20 20"):
            with self.subTest(response=response):
                motor, fake = self.create()
                motor.connect()
                fake.response_override = response
                with self.assertRaisesRegex(MotorLinkError, "timed out"):
                    motor.drive(20, 20)
                self.assert_failed_safe(motor, fake)

    def test_partial_serial_write_is_failure(self):
        motor, fake = self.create()
        motor.connect()
        fake.partial_write = True
        with self.assertRaisesRegex(MotorLinkError, "Partial serial write"):
            motor.drive(20, 20)
        self.assert_failed_safe(motor, fake)

    def test_usb_disconnection_is_failure(self):
        motor, fake = self.create()
        motor.connect()
        fake.read_error = True
        with self.assertRaisesRegex(MotorLinkError, "USB disconnected"):
            motor.drive(20, 20)
        self.assert_failed_safe(motor, fake)

    def test_bad_speed_stops_an_already_moving_rover(self):
        for bad in (101, -101, True, 1.5, "20"):
            with self.subTest(bad=bad):
                motor, fake = self.create()
                motor.connect()
                motor.drive(20, 20)
                with self.assertRaises(MotorLinkError):
                    motor.drive(bad, 20)
                self.assert_failed_safe(motor, fake)

    def test_stop_does_not_clear_estop_and_reset_never_starts_motion(self):
        motor, fake = self.create()
        motor.connect()
        motor.drive(20, 20)
        motor.estop()
        motor.stop()
        self.assertTrue(motor.status()["estop"])
        motor.reset_estop()
        self.assertFalse(motor.status()["estop"])
        self.assertEqual((motor.status()["left"], motor.status()["right"]), (0, 0))
        self.assertEqual(fake.writes[-1], b"R\n")

    def test_existing_firmware_estop_survives_connect(self):
        motor, fake = self.create()
        fake.estop = True
        motor.connect()
        self.assertTrue(motor.status()["estop"])
        self.assertNotIn(b"R\n", fake.writes)

    def test_oversized_and_non_ascii_responses_fail(self):
        for response in (b"x" * 97, b"\xff\n"):
            with self.subTest(response=response):
                motor, fake = self.create()
                motor.connect()
                fake.response_override = response
                with self.assertRaises(MotorLinkError):
                    motor.drive(20, 20)
                self.assert_failed_safe(motor, fake)


class SimulationMotorTests(unittest.TestCase):
    def test_simulation_never_imports_pyserial(self):
        with patch("rover.serial_link.importlib.import_module") as imported:
            motor = SimulationMotor()
            motor.connect()
            motor.drive(20, -20)
            motor.estop()
            motor.reset_estop()
            motor.close()
            imported.assert_not_called()

    def test_simulated_watchdog_latches_and_zeroes_commands(self):
        motor = SimulationMotor()
        motor.connect()
        with patch("rover.serial_link.time.monotonic", return_value=1.0):
            motor.drive(20, 20)
        with patch("rover.serial_link.time.monotonic", return_value=1.36):
            state = motor.status()
        self.assertTrue(state["estop"])
        self.assertEqual((state["left"], state["right"]), (0, 0))
        with self.assertRaises(MotorLinkError):
            motor.drive(20, 20)
        motor.reset_estop()
        motor.drive(20, 20)


if __name__ == "__main__":
    unittest.main()
