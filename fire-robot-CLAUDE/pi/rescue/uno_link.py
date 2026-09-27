"""Serial link from the Raspberry Pi to the Uno motor driver.

Protocol is documented in ../README.md. Typical use:

    with UnoLink("/dev/ttyACM0") as uno:
        uno.forward(120)
        time.sleep(1)
        uno.stop()

A background thread re-sends the last drive command so the Uno's 500 ms
watchdog doesn't stop the car mid-move. Leaving the `with` block always stops.
"""

import queue
import threading
import time

import serial


class UnoError(RuntimeError):
    """The Uno answered ERR, or didn't answer at all."""


class UnoLink:
    def __init__(self, port="/dev/ttyACM0", baud=115200, keepalive_s=0.15,
                 reply_timeout_s=0.5, on_event=None):
        self.port = port
        self.baud = baud
        self.keepalive_s = keepalive_s
        self.reply_timeout_s = reply_timeout_s
        self.on_event = on_event          # called with "E ..." lines from the Uno
        self.events = queue.Queue()       # the same lines, for polling

        self._ser = None
        self._replies = queue.Queue()
        self._lock = threading.Lock()     # one command in flight at a time
        self._running = False
        self._last_drive = None           # command the keepalive re-sends
        self._last_tx = 0.0
        self._threads = []

    # --- lifecycle ---

    def open(self, ready_timeout_s=3.0):
        # Opening the port resets the Uno (DTR), so wait for its READY banner.
        self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        deadline = time.monotonic() + ready_timeout_s
        ready = False
        while time.monotonic() < deadline:
            line = self._ser.readline().decode(errors="replace").strip()
            if line == "READY":
                ready = True
                break
        self._ser.reset_input_buffer()

        self._running = True
        # Reader thread routes replies and events; keepalive thread holds the drive command.
        for target in (self._reader, self._keepalive):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)

        if not ready:
            # Board may not have reset (no DTR); check it's alive anyway.
            try:
                self.ping()
            except UnoError:
                self.close()
                raise UnoError(f"no response from Uno on {self.port}")
        return self

    def close(self):
        if self._ser is None:
            return
        try:
            # Always leave the motors off.
            self.stop()
        except (UnoError, serial.SerialException):
            pass
        self._running = False
        # Threads exit on their next loop now that _running is False.
        for t in self._threads:
            t.join(timeout=1.0)
        self._threads.clear()
        self._ser.close()
        self._ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # --- commands ---

    def drive(self, left, right):
        """Tank drive, signed PWM per side (-255..255, |v| < 30 is treated as 0)."""
        cmd = f"M {int(left)} {int(right)}"
        # Remember nonzero drive commands so the keepalive can repeat them.
        self._last_drive = cmd if (left or right) else None
        try:
            return self._command(cmd)
        except UnoError:
            # The command wasn't accepted; don't keep re-sending it.
            self._last_drive = None
            raise

    def forward(self, speed):
        return self.drive(speed, speed)

    def back(self, speed):
        return self.drive(-speed, -speed)

    def spin(self, speed):
        """Spin in place. Positive = clockwise (right), negative = left."""
        return self.drive(speed, -speed)

    def stop(self):
        self._last_drive = None
        return self._command("S")

    def ping(self):
        return self._command("P")

    def heading(self):
        """Gyro heading in degrees since the Uno booted (raises UnoError if no IMU)."""
        # Reply is 'H <centidegrees>'.
        return int(self._command("H").split()[1]) / 100.0

    def pan(self, angle):
        """Camera pan servo angle, 0-180 (90 = straight ahead)."""
        return self._command(f"V {int(angle)}")

    def distance(self):
        """Latest ultrasonic reading in cm, or None if nothing is in range."""
        reply = self._command("D")
        # Reply is 'D <cm>'; a negative value means no echo.
        cm = int(reply.split()[1])
        return None if cm < 0 else cm

    # --- internals ---

    def _command(self, cmd):
        with self._lock:
            while not self._replies.empty():  # drop late replies to earlier commands
                self._replies.get_nowait()
            self._ser.write((cmd + "\n").encode())
            self._last_tx = time.monotonic()
            try:
                # The reader thread puts the reply on _replies.
                reply = self._replies.get(timeout=self.reply_timeout_s)
            except queue.Empty:
                raise UnoError(f"timeout waiting for reply to {cmd!r}")
        if reply.startswith("ERR"):
            raise UnoError(f"{cmd!r} -> {reply}")
        return reply

    def _reader(self):
        """Background thread: sort incoming lines into events ("E ...") and replies."""
        while self._running:
            try:
                raw = self._ser.readline()
            except serial.SerialException as e:
                self._emit(f"E SERIAL {e}")
                break
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("E "):
                if line.startswith(("E STOP_OBSTACLE", "E WATCHDOG")):
                    self._last_drive = None  # Uno stopped; don't restart it
                self._emit(line)
            else:
                self._replies.put(line)

    def _keepalive(self):
        """Background thread: re-send the last drive command before the Uno's watchdog fires."""
        while self._running:
            time.sleep(self.keepalive_s / 3)
            cmd = self._last_drive
            # Skip if stopped, or if any command went out recently (that also feeds the watchdog).
            if cmd is None or time.monotonic() - self._last_tx < self.keepalive_s:
                continue
            try:
                self._command(cmd)
            except UnoError as e:
                self._last_drive = None
                self._emit(f"E KEEPALIVE {e}")
            except serial.SerialException:
                # Port closed underneath us; close() is shutting down.
                break

    def _emit(self, line):
        """Deliver an Uno event to both the polling queue and the callback."""
        self.events.put(line)
        if self.on_event:
            self.on_event(line)
