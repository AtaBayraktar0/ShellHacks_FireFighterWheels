"""Short, acknowledged commands; the Uno owns the stop timer."""
import time


class Motors:
    """Send simple movement words to the Arduino Uno."""

    def __init__(self, port):
        self.port = port

    def __enter__(self):
        import serial
        self.link = serial.Serial(self.port, 115200, timeout=0.25, write_timeout=0.25)
        try:
            # Uno resets when the USB serial link opens.
            time.sleep(2.0)
            self.link.reset_input_buffer()
            self.stop()
        except BaseException:
            self.link.close()
            raise
        return self

    def send(self, command):
        """Send one command and wait for Uno to say OK."""
        self.link.write((command + "\n").encode("ascii"))
        answer = self.link.readline().decode("ascii", errors="replace").strip()
        if answer != "OK":
            raise RuntimeError(f"Uno did not acknowledge: {answer!r}")

    def stop(self):
        """Stop both motors."""
        self.send("STOP")

    def pulse(self, direction, pwm=70, duration_ms=100):
        """Move briefly, then stop again."""
        if direction not in {"FWD", "BACK", "LEFT", "RIGHT"}:
            raise ValueError("Unknown direction")
        if not 1 <= pwm <= 120 or not 1 <= duration_ms <= 200:
            raise ValueError("Use PWM 1..120 and duration 1..200 ms")
        try:
            self.send(f"{direction} {pwm} {duration_ms}")
            time.sleep(duration_ms / 1000)
        finally:
            # Always try to stop, even when communication fails.
            self.stop()

    def __exit__(self, *args):
        try:
            self.stop()
        finally:
            self.link.close()
