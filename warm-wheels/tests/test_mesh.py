import json
import time
import threading
from types import SimpleNamespace
import numpy as np
from fastapi import FastAPI
from fastapi.testclient import TestClient
from rover.camera import Frame, DemoCamera
from rover.app import create_app
from rover.runtime import RoverRuntime
from rover.mesh import build_mesh
from rover import mesh as mesh_module


def frame(depth):
    return Frame(np.full((*depth.shape, 3), [10, 20, 30], dtype=np.uint8), depth,
                 100, 100, 16, 16, 1.0)


def test_plane_has_finite_metric_vertices_rgb_colors_and_valid_faces():
    result = build_mesh(frame(np.full((32, 32), 2., np.float32)), step=8)
    assert result["vertex_count"] == 16 and result["triangle_count"] == 18
    points = np.array(result["vertices"]).reshape(-1, 3)
    assert np.all(points[:, 2] == -2)
    assert result["colors"][:3] == [round(30/255, 4), round(20/255, 4), round(10/255, 4)]
    assert max(result["indices"]) < result["vertex_count"]
    json.dumps(result, allow_nan=False)


def test_interior_depth_hole_removes_faces_even_between_sampled_vertices():
    depth = np.full((32, 32), 2., np.float32)
    original = build_mesh(frame(depth), step=8)
    depth[4, 4] = 0
    with_hole = build_mesh(frame(depth), step=8)
    assert with_hole["vertex_count"] == original["vertex_count"]
    assert with_hole["triangle_count"] == original["triangle_count"] - 2


def test_depth_jump_and_invalid_returns_are_not_connected():
    depth = np.full((32, 32), 2., np.float32)
    depth[:, 12:] = 5.
    depth[0, 0] = np.nan
    depth[8, 0] = np.inf
    result = build_mesh(frame(depth), step=8)
    p = np.array(result["vertices"]).reshape(-1, 3)
    for indices in np.array(result["indices"]).reshape(-1, 3):
        assert np.ptp(p[indices, 2]) < .1
    json.dumps(result, allow_nan=False)


def test_all_unknown_returns_empty_surface():
    result = build_mesh(frame(np.zeros((32,32), np.float32)))
    assert result["vertices"] == result["indices"] == []
    assert result["bounds"] is None


def test_mesh_api_requires_auth_and_refuses_stale_depth():
    runtime = RoverRuntime()
    camera = DemoCamera().start()
    try:
        captured = camera.read()
        runtime.frames[1] = captured
        runtime.frame_id = 1
        runtime.frame_at = captured.captured_at
        runtime.camera_ok = True
        with TestClient(create_app(runtime, token="test-mesh-token-long-enough", manage_runtime=False)) as client:
            assert client.get("/api/mesh").status_code == 401
            headers = {"Authorization": "Bearer test-mesh-token-long-enough"}
            response = client.get("/api/mesh", headers=headers)
            assert response.status_code == 200
            data = response.json()
            assert data["synthetic"] and data["frame_id"] == 1 and data["triangle_count"] > 0
            runtime.frame_at = time.monotonic() - 1
            assert client.get("/api/mesh", headers=headers).status_code == 503
    finally:
        camera.close()


def mesh_cache_fixture(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(mesh_module, "time", SimpleNamespace(monotonic=lambda: now[0]))
    captured = frame(np.full((32, 32), 2., np.float32))
    captured.captured_at = now[0]
    runtime = SimpleNamespace(lock=threading.RLock(), camera_ok=True, frames={1: captured},
        frame_at=now[0], frame_id=1, remote_frame_id=None, geometry={"camera_height_m": .22},
        mode="demo", remote_detections=[], local_detections=[])
    count = [0]
    original = mesh_module.build_mesh
    def counted(*args, **kwargs):
        count[0] += 1
        return original(*args, **kwargs)
    monkeypatch.setattr(mesh_module, "build_mesh", counted)
    app = FastAPI()
    mesh_module.add_scan_routes(app, runtime, [])
    return runtime, TestClient(app), now, count


def observation(frame_id, x=0.0):
    return {"id": f"remote-{frame_id}-0", "kind": "person", "position_m": [x, .1, 2.],
            "confidence": .9, "source": "test detector", "source_frame_id": frame_id}


def test_late_matching_observations_refresh_cached_geometry_without_remeshing(monkeypatch):
    runtime, client, now, count = mesh_cache_fixture(monkeypatch)
    with client:
        first = client.get("/api/mesh").json()
        assert first["frame_id"] == 1 and first["observations"] == []
        runtime.remote_frame_id = 1
        runtime.remote_detections = [observation(1)]
        now[0] += .05
        updated = client.get("/api/mesh").json()
        assert updated["observations"][0]["source_frame_id"] == updated["frame_id"] == 1
        assert updated["observations"][0]["position"] == [0., .32, -2.]
        assert updated["vertices"] == first["vertices"]
        assert count[0] == 1
        runtime.remote_detections = []
        assert client.get("/api/mesh").json()["observations"] == []
        assert count[0] == 1


def test_new_frame_observations_never_label_older_throttled_geometry(monkeypatch):
    runtime, client, now, count = mesh_cache_fixture(monkeypatch)
    with client:
        runtime.remote_frame_id = 1
        runtime.remote_detections = [observation(1)]
        assert client.get("/api/mesh").json()["observations"][0]["source_frame_id"] == 1
        now[0] = 100.05
        newer = frame(np.full((32, 32), 3., np.float32))
        newer.captured_at = now[0]
        runtime.frames[2] = newer
        runtime.frame_id = runtime.remote_frame_id = 2
        runtime.frame_at = now[0]
        runtime.remote_detections = [observation(2, x=.5)]
        throttled = client.get("/api/mesh").json()
        assert throttled["frame_id"] == 1
        assert [item["source_frame_id"] for item in throttled["observations"]] == [1]
        assert count[0] == 1
        now[0] = 100.25
        current = client.get("/api/mesh").json()
        assert current["frame_id"] == 2
        assert [item["source_frame_id"] for item in current["observations"]] == [2]
        assert count[0] == 2
