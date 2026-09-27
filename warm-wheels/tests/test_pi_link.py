import httpx
import pytest
from fastapi import FastAPI, Depends, HTTPException, Header
from fastapi.testclient import TestClient
from rover.pi_link import PiBridge, add_pi_routes, local_target


@pytest.mark.parametrize("value", ["https://192.168.1.2", "http://user:pass@192.168.1.2", "http://8.8.8.8", "http://169.254.169.254", "http://192.168.1.2/api/state", "http://192.168.1.2?x=1", "example.com", "http://[::1]"])
def test_target_restrictions(value):
    with pytest.raises(ValueError):
        local_target(value)


def test_target_pinned_to_resolved_private_address():
    resolver = lambda *a: [(2, 1, 6, "", ("192.168.5.9", 8000))]
    assert local_target("rover.local:8000", resolver) == "http://192.168.5.9:8000"
    assert local_target("192.168.2.5") == "http://192.168.2.5:8000"
    assert local_target("127.0.0.1:8001") == "http://127.0.0.1:8001"


def make_bridge(handler):
    return PiBridge(lambda **kw: httpx.Client(transport=httpx.MockTransport(handler), **kw))


def test_explicit_link_forwarding_and_disconnect_disarms_without_drive():
    requests = []
    def handler(request):
        requests.append((request.method, request.url.path, request.headers.get("authorization")))
        if request.url.path == "/api/state":
            return httpx.Response(200, json={"mode": "hardware", "motor": {}})
        return httpx.Response(200, json={"armed": False})
    bridge = make_bridge(handler)
    status = bridge.connect("192.168.1.25", "private-pi-token-for-test")
    assert status["connected"] and not status["physical_boundary_enforced"]
    assert "private-pi-token" not in str(status)
    bridge.forward("POST", "disarm", body={})
    bridge.disconnect()
    assert requests == [("GET", "/api/state", "Bearer private-pi-token-for-test"),
                        ("POST", "/api/disarm", "Bearer private-pi-token-for-test"),
                        ("POST", "/api/disarm", "Bearer private-pi-token-for-test")]
    with pytest.raises(ValueError, match="Connect"):
        bridge.forward("GET", "state")


def test_wrong_token_never_connects_and_redirect_is_not_followed():
    for response in (httpx.Response(401), httpx.Response(302, headers={"Location": "http://8.8.8.8"})):
        bridge = make_bridge(lambda request: response)
        with pytest.raises(ValueError):
            bridge.connect("192.168.1.25", "private-pi-token-for-test")
        assert not bridge.status()["connected"]


def test_fault_never_falls_back_to_demo_or_retries_drive():
    requests = []
    def handler(request):
        requests.append(request.url.path)
        if len(requests) == 1:
            return httpx.Response(200, json={"mode": "hardware", "motor": {}})
        raise httpx.ReadTimeout("private connection detail")
    bridge = make_bridge(handler)
    bridge.connect("192.168.1.25", "private-pi-token-for-test")
    with pytest.raises(ValueError, match="connection lost"):
        bridge.forward("POST", "drive", body={"left": 15, "right": 15})
    assert requests == ["/api/state", "/api/drive"]
    assert bridge.status()["mode"] == "hardware"
    assert "private connection detail" not in bridge.status()["error"]


def test_proxy_requires_local_auth_fixed_paths_and_preserves_frame_headers():
    app = FastAPI()
    def auth(authorization: str = Header(default="")):
        if authorization != "Bearer local-test-token":
            raise HTTPException(401)
    add_pi_routes(app, [Depends(auth)])
    bridge = app.state.pi_bridge
    seen = []
    def handler(request):
        seen.append(request.url.path)
        if request.url.path == "/api/state":
            return httpx.Response(200, json={"mode": "hardware", "motor": {}})
        return httpx.Response(200, content=b"fake-jpeg", headers={"X-Frame-Id": "42", "X-Frame-Age": "0.01"})
    bridge._factory = lambda **kw: httpx.Client(transport=httpx.MockTransport(handler), **kw)
    with TestClient(app) as client:
        assert client.get("/api/pi/link").status_code == 401
        headers = {"Authorization": "Bearer local-test-token"}
        assert client.post("/api/pi/connect", headers=headers, json={"address": "192.168.1.25", "token": "private-pi-token-for-test"}).status_code == 200
        response = client.get("/api/pi/proxy/api/frame.jpg?view=depth", headers=headers)
        assert response.content == b"fake-jpeg" and response.headers["X-Frame-Id"] == "42"
        assert client.post("/api/pi/proxy/api/mission/start", headers=headers).status_code == 404
        assert client.post("/api/pi/proxy/api/anything", headers=headers).status_code == 409
        assert seen == ["/api/state", "/api/frame.jpg"]


def test_pending_connect_is_canceled_by_disconnect():
    import threading
    started, release = threading.Event(), threading.Event()
    def handler(request):
        started.set()
        assert release.wait(3)
        return httpx.Response(200, json={"mode": "hardware", "motor": {}})
    bridge = make_bridge(handler)
    errors = []
    def connect():
        try:
            bridge.connect("192.168.1.25", "private-pi-token-for-test")
        except ValueError as exc:
            errors.append(str(exc))
    thread = threading.Thread(target=connect)
    thread.start()
    assert started.wait(3)
    bridge.disconnect()
    release.set()
    thread.join(3)
    assert not thread.is_alive()
    assert errors and "canceled" in errors[0]
    assert not bridge.status()["connected"]


def test_detection_cannot_cross_pi_connections_with_matching_frame_ids():
    uploads = []
    def handler(request):
        if request.url.path == "/api/state":
            return httpx.Response(200, json={"mode": "hardware", "motor": {}})
        if request.url.path == "/api/detections":
            uploads.append(request.url.host)
        return httpx.Response(200, json={}, headers={"X-Frame-Id": "42"})
    bridge = make_bridge(handler)
    bridge.connect("192.168.1.25", "private-pi-token-for-test")
    generation = bridge.forward("GET", "inference-frame.jpg").headers["X-Pi-Generation"]
    bridge.forward("POST", "detections", body={"frame_id": 42}, expected_generation=generation)
    bridge.connect("192.168.1.26", "private-pi-token-for-test")
    with pytest.raises(ValueError, match="source changed"):
        bridge.forward("POST", "detections", body={"frame_id": 42}, expected_generation=generation)
    with pytest.raises(ValueError, match="generation is missing"):
        bridge.forward("POST", "detections", body={"frame_id": 42})
    assert uploads == ["192.168.1.25"]
