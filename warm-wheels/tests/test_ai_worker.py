"""AI transport tests use tiny fake predictions, never external weights or devices."""
import json
from types import SimpleNamespace

import cv2
import httpx
import numpy as np
import pytest

from pc.detector import normalize_base_url, resolve_device, run_worker
from scripts.start_ai import select_source, worker_command


class Clock:
    def __init__(self):
        self.now = 0.

    def __call__(self):
        return self.now

    def sleep(self, amount):
        self.now += amount


class Tensor:
    def __init__(self, value):
        self.value = np.array(value)

    def cpu(self):
        return self

    def numpy(self):
        return self.value


class Model:
    def __init__(self, label, clock=None, delay=0.):
        self.label, self.clock, self.delay, self.calls = label, clock, delay, []

    def predict(self, image, **kwargs):
        self.calls.append(kwargs)
        if self.clock:
            self.clock.sleep(self.delay)
        result = SimpleNamespace(
            boxes=SimpleNamespace(xyxy=Tensor([[1, 1, 10, 10]]), cls=Tensor([0]), conf=Tensor([.91])),
            keypoints=None, names={0: self.label})
        return [result]


class Server:
    def __init__(self, *, mode="hardware", frames=None, age="0.01", post_status=200):
        self.mode, self.age, self.post_status = mode, age, post_status
        self.frames, self.frame_index = frames or [1], 0
        self.paths, self.submissions = [], []
        self.submission_headers = []
        self.jpeg = cv2.imencode(".jpg", np.zeros((24, 32, 3), np.uint8))[1].tobytes()

    def __call__(self, request):
        self.paths.append(request.url.path)
        if request.url.path.endswith("/api/state"):
            return httpx.Response(200, json={"mode": self.mode})
        if request.url.path.endswith("/api/inference-frame.jpg"):
            frame = self.frames[min(self.frame_index, len(self.frames) - 1)]
            self.frame_index += 1
            return httpx.Response(200, content=self.jpeg, headers={"X-Frame-Id": str(frame), "X-Frame-Age": self.age,
                                                                  "X-Pi-Generation": "7"})
        if request.url.path.endswith("/api/detections"):
            self.submissions.append(json.loads(request.content))
            self.submission_headers.append(request.headers)
            return httpx.Response(self.post_status, json={"ok": True})
        raise AssertionError(f"Unexpected request {request.url.path}")


def execute(server, *, clock=None, person=None, fire=None, **kwargs):
    clock = clock or Clock()
    person, fire = person or Model("person"), fire or Model("fire")
    with httpx.Client(base_url=normalize_base_url("http://laptop/api/pi/proxy"),
                      transport=httpx.MockTransport(server)) as client:
        return run_worker(client, person, fire, clock=clock, sleeper=clock.sleep,
                          max_frames=kwargs.pop("max_frames", 1), max_seconds=kwargs.pop("max_seconds", 3), **kwargs)


def test_proxy_prefix_matching_frame_and_separate_confidence_are_preserved():
    server, person, fire = Server(frames=[71]), Model("person"), Model("fire")
    stats = execute(server, person=person, fire=fire)
    assert stats["submitted"] == 1
    assert all(path.startswith("/api/pi/proxy/api/") for path in server.paths)
    assert server.submissions[0]["frame_id"] == 71
    assert server.submission_headers[0]["X-Pi-Generation"] == "7"
    assert [item["kind"] for item in server.submissions[0]["detections"]] == ["person", "fire_candidate"]
    assert person.calls[0]["conf"] == .45
    assert fire.calls[0]["conf"] == .35


def test_synthetic_source_is_refused_without_inference():
    person, fire = Model("person"), Model("fire")
    with pytest.raises(RuntimeError, match="Synthetic source refused"):
        execute(Server(mode="demo"), person=person, fire=fire)
    assert not person.calls and not fire.calls


def test_explicit_demo_transport_labels_each_prediction():
    server = Server(mode="demo")
    stats = execute(server, allow_demo=True)
    assert stats["mode"] == "demo"
    assert all(item["source"].startswith("AI on synthetic demo frame;") for item in server.submissions[0]["detections"])


def test_stale_input_is_discarded_before_model_runs():
    server, person = Server(age="0.95"), Model("person")
    stats = execute(server, person=person)
    assert stats["discarded"] == 1
    assert not person.calls and not server.submissions


def test_slow_inference_is_discarded_before_upload():
    clock, server = Clock(), Server()
    stats = execute(server, clock=clock, person=Model("person", clock, 1.11))
    assert stats["discarded"] == 1 and stats["submitted"] == 0
    assert not server.submissions


def test_transport_elapsed_time_is_added_to_server_frame_age():
    clock, server = Clock(), Server(age="0.3")
    def slow_server(request):
        if request.url.path.endswith("inference-frame.jpg"):
            clock.sleep(.7)
        return server(request)
    person = Model("person")
    stats = execute(slow_server, clock=clock, person=person)
    assert stats["discarded"] == 1 and not person.calls


def test_repeated_frame_does_not_repeat_inference_and_timeout_bounds_it():
    server, person = Server(frames=[19]), Model("person")
    stats = execute(server, person=person, max_frames=2, max_seconds=.2)
    assert stats["frames_seen"] == stats["submitted"] == len(person.calls) == 1
    assert stats["elapsed_s"] >= .2


def test_restarted_frame_sequence_stops_worker():
    server = Server(frames=[19, 2])
    with pytest.raises(RuntimeError, match="sequence restarted"):
        execute(server, max_frames=2)
    assert len(server.submissions) == 1


def test_server_rejection_is_not_counted_as_submission():
    stats = execute(Server(post_status=409))
    assert stats["discarded"] == 1 and stats["submitted"] == 0


def test_token_revocation_stops_worker():
    with pytest.raises(RuntimeError, match="token was rejected"):
        execute(Server(post_status=401))


@pytest.mark.parametrize("age", ["nan", "-1", "inf", "invalid"])
def test_invalid_timestamp_never_reaches_model(age):
    server, person = Server(age=age), Model("person")
    stats = execute(server, person=person, max_seconds=.6)
    assert not server.submissions and not person.calls
    assert stats["errors"] > 0


def test_source_mode_change_stops_before_new_prediction():
    server = Server()
    calls = 0
    def changing(request):
        nonlocal calls
        if request.url.path.endswith("/api/state"):
            calls += 1
            if calls > 1:
                server.mode = "demo"
        return server(request)
    with pytest.raises(RuntimeError, match="Source mode changed"):
        execute(changing, allow_demo=True)
    assert not server.submissions


def test_gateway_reconnection_stops_before_predicting_on_new_source():
    server, person = Server(frames=[1, 2]), Model("person")
    def changing(request):
        response = server(request)
        if request.url.path.endswith("inference-frame.jpg") and server.frame_index > 1:
            response.headers["X-Pi-Generation"] = "8"
        return response
    with pytest.raises(RuntimeError, match="Joined Pi connection changed"):
        execute(changing, person=person, max_frames=2)
    assert len(person.calls) == len(server.submissions) == 1


def test_gateway_missing_generation_refuses_predictions():
    server, person = Server(), Model("person")
    def missing(request):
        response = server(request)
        response.headers.pop("X-Pi-Generation", None)
        return response
    with pytest.raises(RuntimeError, match="generation is unavailable"):
        execute(missing, person=person)
    assert not person.calls and not server.submissions


def test_direct_pi_does_not_require_gateway_generation():
    server, clock = Server(), Clock()
    def direct(request):
        response = server(request)
        response.headers.pop("X-Pi-Generation", None)
        return response
    with httpx.Client(base_url="http://pi:8000/", transport=httpx.MockTransport(direct)) as client:
        stats = run_worker(client, Model("person"), Model("fire"), max_frames=1, clock=clock, sleeper=clock.sleep)
    assert stats["submitted"] == 1
    assert "X-Pi-Generation" not in server.submission_headers[0]


@pytest.mark.parametrize("value", ["ftp://pi", "http://token@pi", "http://pi?token=x", "http://pi#token=x", "http://pi:abc"])
def test_url_rejects_credentials_and_unsupported_parts(value):
    with pytest.raises(ValueError):
        normalize_base_url(value)


def test_auto_device_supports_gpu_and_cpu_without_importing_real_torch():
    assert resolve_device("auto", SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: True))) == "0"
    assert resolve_device("auto", SimpleNamespace(cuda=SimpleNamespace(is_available=lambda: False))) == "cpu"
    assert resolve_device("cpu") == "cpu"
    with pytest.raises(ValueError):
        resolve_device("cuda:unknown")


def test_joined_and_local_sources_choose_correct_session_and_do_not_put_token_in_argv():
    modes = []
    def load(mode):
        modes.append(mode)
        return {"port": 8010, "token": "private-test-session-token"}
    joined = select_source("joined", session_loader=load)
    local = select_source("local", session_loader=load)
    assert modes == ["demo", "hardware"]
    assert joined["base_url"] == "http://127.0.0.1:8010/api/pi/proxy/"
    assert local["base_url"] == "http://127.0.0.1:8010/"
    args = SimpleNamespace(device="auto", person_confidence=.45, fire_confidence=.35,
                           allow_demo=True, max_frames=3, max_seconds=10.)
    command = worker_command(joined, args, "python.exe")
    assert joined["token"] not in " ".join(command)
    assert command[command.index("--rover") + 1] == joined["base_url"]
    assert "--allow-demo" in command and "--max-frames" in command


def test_direct_source_reads_token_from_hidden_prompt_only():
    messages = []
    def hidden(message):
        messages.append(message)
        return "secret-pi-token-for-test"
    source = select_source("direct", "192.168.1.42", secret_prompt=hidden)
    assert source["base_url"] == "http://192.168.1.42:8000/"
    assert len(messages) == 1 and "hidden" in messages[0]


def test_missing_session_and_implicit_demo_return_actionable_errors():
    with pytest.raises(ValueError, match="START_PRESENTATION"):
        select_source("joined", session_loader=lambda mode: None)
    with pytest.raises(ValueError, match="--allow-demo"):
        select_source("demo", session_loader=lambda mode: None)
