import time

import cv2
import numpy as np
import pytest
from fastapi.testclient import TestClient

from rover.app import create_app
from rover.camera import Frame
from rover.room_mapping import RoomMapper, TrackingLost, estimate_transform
from rover.runtime import RoverRuntime


def correspondences():
    rng = np.random.default_rng(42)
    xyz = rng.uniform([-1.4, -1., 2.5], [1.4, 1., 4.5], (180, 3))
    matrix = np.array([[450., 0, 320], [0, 450, 240], [0, 0, 1]])
    rotation = np.array([.01, .04, -.015])
    translation = np.array([-.08, .01, .025])
    uv = cv2.projectPoints(xyz, rotation, translation, matrix, None)[0].reshape(-1, 2)
    z = (xyz @ cv2.Rodrigues(rotation)[0].T + translation)[:, 2]
    expected = np.eye(4)
    expected[:3, :3] = cv2.Rodrigues(rotation)[0]
    expected[:3, 3] = translation
    return xyz, uv, z, matrix, expected


def test_metric_pose_recovers_known_transform_despite_outliers():
    xyz, uv, z, matrix, expected = correspondences()
    uv[:20] += [75, -60]
    result, quality = estimate_transform(xyz, uv, z, matrix, (480, 640))
    np.testing.assert_allclose(result, expected, atol=.002)
    assert quality['inliers'] >= 150


def test_depth_disagreement_and_sparse_matches_are_rejected():
    xyz, uv, z, matrix, _ = correspondences()
    with pytest.raises(TrackingLost, match="Depth disagrees"):
        estimate_transform(xyz, uv, z + .7, matrix, (480, 640))
    with pytest.raises(TrackingLost):
        estimate_transform(xyz[:10], uv[:10], z[:10], matrix, (480, 640))


def textured_frame(timestamp=1., shift=0):
    rng = np.random.default_rng(9)
    mono = rng.integers(0, 256, (240, 320), dtype=np.uint8)
    mono = cv2.GaussianBlur(mono, (3, 3), .5)
    mono = cv2.warpAffine(mono, np.float32([[1, 0, shift], [0, 1, 0]]), (320, 240))
    return Frame(cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR), np.full((240, 320), 2., dtype=np.float32),
                 250, 250, 160, 120, timestamp)


def test_real_features_track_translation_and_retain_previous_surface():
    mapper = RoomMapper()
    mapper.status = 'starting'
    mapper.process(textured_frame(), 1)
    original = mapper.snapshot()['vertices'][:]
    assert mapper.status == 'tracking'
    # Each 8-pixel image shift corresponds to -0.064 m camera movement.
    mapper.process(textured_frame(1.2, 8), 2)
    assert mapper.status == 'tracking', mapper.reason
    mapper.process(textured_frame(1.4, 16), 3)
    assert mapper.status == 'tracking', mapper.reason
    assert mapper.pose[0, 3] == pytest.approx(-.128, abs=.025)
    assert len(mapper.patches) == 2
    mesh = mapper.snapshot()
    assert mesh['vertices'][:len(original)] == original
    assert max(mesh['indices']) < mesh['vertex_count']
    assert mesh['autonomous_ready'] is False


def test_tracking_failure_freezes_and_does_not_silently_relocalize():
    mapper = RoomMapper()
    mapper.status = 'starting'
    mapper.process(textured_frame(), 1)
    original = mapper.snapshot()['vertices'][:]
    frame = textured_frame(1.2)
    frame.rgb[:] = 0
    mapper.process(frame, 2)
    assert mapper.status == 'lost'
    mapper.process(textured_frame(1.4, 20), 3)
    assert mapper.status == 'lost'
    assert mapper.snapshot()['vertices'] == original
    assert mapper.frame_id == 1


def test_timestamp_gap_and_duplicate_frame_cannot_add_geometry():
    mapper = RoomMapper()
    mapper.status = 'starting'
    mapper.process(textured_frame(), 1)
    mapper.process(textured_frame(1.1, 12), 1)
    assert len(mapper.patches) == 1
    mapper.process(textured_frame(4., 12), 2)
    assert mapper.status == 'lost'
    assert len(mapper.patches) == 1


def test_viewer_coordinates_follow_camera_translation():
    mapper = RoomMapper(camera_height=.22)
    mapper.pose[:3, 3] = [1., .1, 2.]
    np.testing.assert_allclose(mapper.viewer_pose()[:3, 3], [1., .12, -2.])


def test_capacity_retains_old_geometry_and_stops_acquisition():
    mapper = RoomMapper()
    mapper.MAX_KEYFRAMES = 1
    mapper.status = 'starting'
    mapper.process(textured_frame(), 1)
    original = mapper.snapshot()['vertices'][:]
    mapper.process(textured_frame(1.2, 16), 2)
    assert mapper.status == 'capacity', mapper.reason
    assert mapper.snapshot()['vertices'] == original


def test_api_requires_token_and_rejects_motor_enabled_or_stale_source():
    runtime = RoverRuntime(mode='hardware')
    headers = {'Authorization': 'Bearer mapping-test-token-long'}
    with TestClient(create_app(runtime, token='mapping-test-token-long', manage_runtime=False)) as client:
        assert client.get('/api/room-map').status_code == 401
        assert client.post('/api/room-map-control', json={'action': 'start'}).status_code == 401
        assert client.post('/api/room-map-control', headers=headers, json={'action': 'start'}).status_code == 409
        runtime.camera_ok, runtime.frame_at = True, time.monotonic()
        runtime.gate.motion_enabled = True
        response = client.post('/api/room-map-control', headers=headers, json={'action': 'start'})
        assert response.status_code == 409 and 'camera-only' in response.text
        assert client.post('/api/room-map-control', headers=headers, json={'action': 'drive'}).status_code == 422


def test_service_processes_frames_without_motor_commands():
    runtime = RoverRuntime(mode='hardware')
    runtime.camera_ok = True
    runtime.frame_at = time.monotonic()
    runtime.frame_id = 1
    runtime.frames[1] = textured_frame(runtime.frame_at)
    headers = {'Authorization': 'Bearer mapping-test-token-long'}
    with TestClient(create_app(runtime, token='mapping-test-token-long', manage_runtime=False)) as client:
        assert client.post('/api/room-map-control', headers=headers, json={'action': 'start'}).status_code == 200
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            result = client.get('/api/room-map', headers=headers).json()
            if result['keyframes']:
                break
            time.sleep(.01)
        assert result['keyframes'] == 1
        assert runtime.requested == runtime.motion == (0, 0)
        assert runtime.gate.armed is False
        assert client.post('/api/room-map-control', headers=headers, json={'action': 'stop'}).status_code == 200
        assert client.get('/api/room-map', headers=headers).json()['tracking'] == 'stopped'
