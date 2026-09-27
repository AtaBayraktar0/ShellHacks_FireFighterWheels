"""Acknowledged WN1 motor transport. Importing this module does not need pyserial.

Each drive is one synchronous exchange; call from the rover control thread at
~10 Hz. Do not auto-replay commands after failures. Firmware owns the watchdog.
"""
from __future__ import annotations

import importlib
import threading
import time
from typing import Callable


class MotorLinkError(RuntimeError):
    """Motor state could not be confirmed; the caller must disarm motion."""


def _validate_speed(left: int, right: int) -> None:
    if any(type(value) is not int or not -100 <= value <= 100 for value in (left, right)):
        raise ValueError("Motor commands must be integers in [-100, 100].")


class SerialMotor:
    def __init__(self, port: str, baud: int = 115200, *,
                 serial_factory: Callable | None = None,
                 handshake_timeout: float = 4.0, ack_timeout: float = 0.15):
        if not 0 < ack_timeout <= 0.2 or handshake_timeout <= 0:
            raise ValueError("ACK timeout must be (0, 0.2] seconds; handshake timeout must be positive.")
        self.port, self.baud = port, baud
        self._factory = serial_factory
        self._handshake_timeout, self._ack_timeout = handshake_timeout, ack_timeout
        self._serial = None
        self._lock = threading.RLock()
        self._connected = self._enabled = self._estop = False
        self._left = self._right = self._sequence = 0
        self._error: str | None = None
        self._profile: str | None = None
        self._last_ack: float | None = None
        self._receive_buffer = bytearray()

    def connect(self) -> None:
        with self._lock:
            if self._connected:
                return
            self._error = None
            self._receive_buffer.clear()
            try:
                factory = self._factory or importlib.import_module("serial").Serial
                self._serial = factory(self.port, self.baud, timeout=0.02, write_timeout=0.1)
                deadline = time.monotonic() + self._handshake_timeout
                ready = False
                while time.monotonic() < deadline:
                    line = self._read_line(deadline)
                    if not line:
                        continue
                    if line == "READY WN1":
                        ready = True
                    elif ready and line in ("PROFILE V4 VERIFIED", "PROFILE L298N VERIFIED"):
                        self._profile = line.split()[1]
                        self._connected = self._enabled = True
                        self._exchange(b"S\n", {"STOPPED 0", "STOPPED 1"})
                        return
                    elif line.startswith("PROFILE"):
                        raise MotorLinkError(
                            f"Firmware profile is unverified or disabled (Uno reported '{line}'). "
                            "Its motor outputs stay off until you upload a build with HARDWARE_VERIFIED=1 "
                            "after the raised-wheel checks in docs/hardware.md.")
                    # Bootloader noise is tolerated only before READY WN1.
                    elif ready:
                        raise MotorLinkError(f"Unexpected handshake response: {line}")
                raise MotorLinkError("WN1 startup handshake timed out; reset Uno and verify firmware/port.")
            except Exception as exc:
                self._fail(exc)

    def _read_line(self, deadline: float) -> str | None:
        while time.monotonic() < deadline:
            # read() avoids accepting an incomplete readline() after a timeout.
            data = self._serial.read(1)
            if not data:
                continue
            if data == b"\n":
                raw = bytes(self._receive_buffer).rstrip(b"\r")
                self._receive_buffer.clear()
                try:
                    return raw.decode("ascii")
                except UnicodeDecodeError as exc:
                    raise MotorLinkError("Non-ASCII motor response.") from exc
            self._receive_buffer.extend(data)
            if len(self._receive_buffer) > 96:
                raise MotorLinkError("Oversized motor response.")
        return None

    def _write(self, command: bytes) -> None:
        if self._serial.write(command) != len(command):
            raise MotorLinkError("Partial serial write.")

    def _exchange(self, command: bytes, expected: set[str]) -> str:
        self._write(command)
        deadline = time.monotonic() + self._ack_timeout
        line = self._read_line(deadline)
        if line is None:
            raise MotorLinkError("Motor acknowledgement timed out.")
        if line not in expected:
            raise MotorLinkError(f"Unexpected motor response: {line}")
        self._last_ack = time.monotonic()
        if line in ("STOPPED 0", "STOPPED 1", "ESTOP 0", "ESTOP 1"):
            self._left = self._right = 0
            self._estop = line.endswith("1")
        return line

    def _fail(self, exc: Exception) -> None:
        self._error = str(exc)
        self._connected = self._enabled = False
        self._left = self._right = 0
        # A failed round trip leaves physical state unknown. Latch both ends
        # where possible and close; never return a false successful drive.
        self._estop = True
        if self._serial is not None:
            try:
                # Leading newline terminates a possible truncated prior write.
                self._serial.write(b"\nE\nS\n")
            except Exception:
                pass
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        raise MotorLinkError(self._error) from exc

    def _require_connection(self) -> None:
        if not self._connected or self._serial is None:
            raise MotorLinkError(self._error or "Motor link is not connected.")

    def drive(self, left: int, right: int) -> None:
        with self._lock:
            self._require_connection()
            try:
                _validate_speed(left, right)
                if self._estop:
                    raise MotorLinkError("Emergency stop is latched; explicit reset is required.")
                self._sequence = self._sequence % 2147483647 + 1
                payload = f"M {self._sequence} {left} {right}"
                self._exchange((payload + "\n").encode("ascii"),
                               {f"ACK {self._sequence} {left} {right}"})
                self._left, self._right = left, right
            except Exception as exc:
                self._fail(exc)

    def _control(self, command: bytes, expected: set[str]) -> None:
        with self._lock:
            self._require_connection()
            try:
                self._exchange(command, expected)
            except Exception as exc:
                self._fail(exc)

    def stop(self) -> None:
        self._control(b"S\n", {"STOPPED 0", "STOPPED 1"})

    def estop(self) -> None:
        self._control(b"E\n", {"ESTOP 1"})

    def reset_estop(self) -> None:
        self._control(b"R\n", {"ESTOP 0"})

    def close(self) -> None:
        with self._lock:
            failure = None
            try:
                if self._connected:
                    self.stop()
            except MotorLinkError as exc:
                failure = exc
            finally:
                if self._serial is not None:
                    try:
                        self._serial.close()
                    except Exception as exc:
                        self._error = str(exc)
                        failure = MotorLinkError(self._error)
                    finally:
                        self._serial = None
                self._connected = self._enabled = False
                self._left = self._right = 0
            if failure is not None:
                raise failure

    def status(self) -> dict:
        with self._lock:
            return {"connected": self._connected, "motors_enabled": self._enabled,
                    "error": self._error, "estop": self._estop, "profile": self._profile,
                    "left": self._left, "right": self._right,
                    "last_ack_age_s": None if self._last_ack is None else round(time.monotonic() - self._last_ack, 3),
                    "simulated": False}


class SimulationMotor:
    """Same public interface without importing/opening any serial device."""
    def __init__(self):
        self._connected = self._estop = False
        self._left = self._right = 0
        self._error: str | None = None
        self._last_drive = 0.0
        self._lock = threading.RLock()

    def connect(self) -> None:
        with self._lock:
            self._connected = True
            self._error = None

    def _watchdog(self) -> None:
        if (self._left or self._right) and time.monotonic() - self._last_drive >= 0.35:
            self._left = self._right = 0
            self._estop = True
            self._error = "Simulated firmware watchdog expired."

    def drive(self, left: int, right: int) -> None:
        with self._lock:
            self._watchdog()
            try:
                _validate_speed(left, right)
                if not self._connected or self._estop:
                    raise MotorLinkError("Simulation motor disconnected or emergency stop latched.")
            except Exception:
                self._left = self._right = 0
                self._estop = True
                raise
            self._left, self._right = left, right
            self._last_drive = time.monotonic()

    def stop(self) -> None:
        with self._lock:
            self._left = self._right = 0

    def estop(self) -> None:
        with self._lock:
            self.stop()
            self._estop = True

    def reset_estop(self) -> None:
        with self._lock:
            self.stop()
            self._estop = False
            self._error = None

    def close(self) -> None:
        with self._lock:
            self.stop()
            self._connected = False

    def status(self) -> dict:
        with self._lock:
            self._watchdog()
            return {"connected": self._connected, "motors_enabled": self._connected,
                    "error": self._error, "estop": self._estop, "profile": "SIMULATION",
                    "left": self._left, "right": self._right, "simulated": True}
