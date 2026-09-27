"""UnoLink against a fake Uno on a pseudo-terminal (no hardware needed)."""

import os
import pty
import threading
import time

import pytest

from rescue.uno_link import UnoError, UnoLink


def fake_uno(fd, sent):
    """Pretend to be the Uno firmware on the pty master: answer commands and
    simulate an obstacle stop on the second forward command."""
    f = os.fdopen(fd, "r+b", buffering=0)
    time.sleep(0.2)
    f.write(b"READY\n")
    buf = b""
    blocked = False
    while True:
        try:
            buf += f.read(1)
        except OSError:
            return
        if not buf.endswith(b"\n"):
            continue
        line, buf = buf.decode().strip(), b""
        sent.append(line)
        if line == "D":
            f.write(b"D 42\n")
        elif line == "H":
            f.write(b"H 9050\n")
        # After the obstacle stop, refuse forward motion like the firmware does.
        elif line.startswith("M") and blocked and sum(map(int, line.split()[1:])) > 0:
            f.write(b"ERR BLOCKED 10\n")
        else:
            f.write(b"OK\n")
        # The second 'M 100 100' (a keepalive re-send) trips the simulated sonar.
        if line == "M 100 100" and sent.count(line) == 2:
            blocked = True
            f.write(b"E STOP_OBSTACLE 10\n")


@pytest.fixture
def link():
    master, slave = pty.openpty()
    sent = []
    threading.Thread(target=fake_uno, args=(master, sent), daemon=True).start()
    events = []
    # UnoLink talks to the pty slave end as if it were a serial port.
    with UnoLink(os.ttyname(slave), on_event=events.append) as uno:
        yield uno, sent, events


def test_commands_and_replies(link):
    uno, sent, _ = link
    assert uno.distance() == 42
    assert uno.heading() == pytest.approx(90.5)
    uno.pan(120)
    assert "V 120" in sent


def test_keepalive_and_safety_stop(link):
    uno, sent, events = link
    uno.forward(100)
    time.sleep(0.5)  # keepalive re-sends until the fake Uno reports an obstacle
    assert any(e.startswith("E STOP_OBSTACLE") for e in events)
    with pytest.raises(UnoError, match="BLOCKED"):
        uno.forward(100)
    uno.spin(80)  # turning away is still allowed
    assert sent[-1] == "M 80 -80"
