import time
import pytest
from fastapi.testclient import TestClient
from rover.app import create_app
from rover.runtime import RoverRuntime
from rover.camera import DemoCamera
from rover.spatial import analyze

TOKEN = "test-session-token-never-deploy"
AUTH = {"Authorization": f"Bearer {TOKEN}"}


@pytest.fixture
def runtime():
    runtime = RoverRuntime()
    runtime.motor.connect()
    camera = DemoCamera().start()
    frame = camera.read()
    runtime.spatial = analyze(frame)
    runtime.camera_ok = True
    runtime.frame_at = frame.captured_at
    runtime.frame_id = 1
    runtime.frames[1] = frame
    runtime.jpeg = {key: runtime._encode(frame.rgb) for key in ("raw", "color", "depth")}
    yield runtime
    runtime.motor.close()
    camera.close()


@pytest.fixture
def client(runtime):
    with TestClient(create_app(runtime, TOKEN, manage_runtime=False)) as client:
        yield client


def test_read_and_control_auth(client, runtime):
    assert client.get("/").status_code == 200
    for path in ("/api/state", "/api/frame.jpg", "/api/inference-frame.jpg"):
        assert client.get(path).status_code == 401
        # This fixture does not run capture. Keep the auth check independent of
        # scheduler delays; test_camera_staleness_blocks_images_and_arming covers expiry.
        runtime.frame_at = time.monotonic()
        assert client.get(path, headers=AUTH).status_code == 200
    for path in ("/api/arm", "/api/estop", "/api/reset"):
        assert client.post(path).status_code == 401


def test_estop_reset_never_rearms(client, runtime):
    assert client.post("/api/arm", headers=AUTH).status_code == 200
    assert runtime.gate.armed
    assert client.post("/api/estop", headers=AUTH).status_code == 200
    assert runtime.gate.estop and not runtime.gate.armed
    assert client.post("/api/arm", headers=AUTH).status_code == 409
    assert client.post("/api/reset", headers=AUTH).status_code == 200
    assert not runtime.gate.armed and not runtime.gate.estop
    assert client.post("/api/drive", headers=AUTH, json={"left":25,"right":25}).status_code == 409


@pytest.mark.parametrize("body", [{"left":-1,"right":10},{"left":100,"right":100},
    {"left":True,"right":20},{"left":20.5,"right":20},{"left":20,"right":20,"extra":1}])
def test_invalid_motor_payload_rejected(client, body):
    assert client.post("/api/drive", headers=AUTH, json=body).status_code == 422


def test_inference_must_match_original_frame_and_expire(client, runtime):
    body = {"frame_id":1,"detections":[{"kind":"person","confidence":.8,
        "box":[130,116,201,320],"source":"test model"}]}
    response = client.post("/api/detections", headers=AUTH, json=body)
    assert response.status_code == 200
    assert runtime.remote_detections[0]["distance_m"] is not None
    assert client.post("/api/detections", headers=AUTH, json=body).status_code == 409
    runtime.remote_frame_id = None
    runtime.frames[1].captured_at -= 2
    assert client.post("/api/detections", headers=AUTH, json=body).status_code == 409


def test_assessment_is_snapshot_record_not_medical_prediction(client, runtime):
    runtime.observations["observation"] = (time.monotonic(), {"id":"observation","kind":"person","distance_m":2.0})
    result = client.post("/api/assessment", headers=AUTH, json={"id":"observation","assessment":"needs_assistance"})
    assert result.status_code == 200
    assert "Operator annotation" in result.json()["event"]["note"]
    assert client.post("/api/assessment", headers=AUTH, json={"id":"observation","assessment":"unconscious"}).status_code == 422


def test_route_preview_never_sends_motion(client, runtime):
    previous = runtime.motor.status().copy()
    response = client.post("/api/route", headers=AUTH, json={"col":30,"row":40})
    assert response.status_code == 200
    assert response.json()["preview_only"]
    assert runtime.motor.status() == previous


def test_camera_staleness_blocks_images_and_arming(client, runtime):
    runtime.frame_at -= 2
    assert client.get("/api/frame.jpg", headers=AUTH).status_code == 503
    assert client.post("/api/arm", headers=AUTH).status_code == 409


def test_empty_mission_returns_without_motor_commands(client, runtime):
    before = runtime.motor.status()
    assert client.post("/api/mission/start", headers=AUTH).status_code == 200
    assert client.post("/api/mission/start", headers=AUTH).status_code == 409
    assert client.post("/api/arm", headers=AUTH).status_code == 409
    for _ in range(400):
        status = runtime.mission.tick()
        if status["phase"] in ("complete", "blocked"):
            break
    assert status["phase"] == "complete"
    assert status["position"] == status["home"]
    assert status["coverage_percent"] == 100
    assert runtime.motor.status() == before


def test_physical_autonomy_never_silently_simulates(client, runtime):
    runtime.mission = None
    assert client.post("/api/mission/start", headers=AUTH).status_code == 409


def test_can_drive_is_separate_from_status_text(client, runtime):
    assert client.post("/api/arm", headers=AUTH).status_code == 200
    runtime.block_reason = "Hold a drive control to move"
    assert client.get("/api/state", headers=AUTH).json()["can_drive"]
