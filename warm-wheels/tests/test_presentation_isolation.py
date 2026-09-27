"""Expensive simulated scans cannot block live camera/motor state access."""
import threading
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from rover.runtime import RoverRuntime


@pytest.mark.parametrize("operation", ["control", "tick"])
def test_slow_presentation_does_not_hold_control_lock(operation):
    entered, release = threading.Event(), threading.Event()

    def expensive_scan(*args, **kwargs):
        entered.set()
        assert release.wait(3)
        return {}

    fake = SimpleNamespace(control=expensive_scan, tick=expensive_scan)
    with patch("rover.runtime.PresentationSimulator", return_value=fake):
        runtime = RoverRuntime()
    runtime.mission = None
    target = (lambda: runtime.presentation_command("play")) if operation == "control" else runtime._mission_loop
    worker = threading.Thread(target=target)
    worker.start()
    try:
        assert entered.wait(3)
        acquired = runtime.lock.acquire(timeout=.2)
        assert acquired, "A reconstruction operation blocked the live control state lock"
        runtime.lock.release()
    finally:
        runtime.quit.set()
        release.set()
        worker.join(3)
    assert not worker.is_alive()
