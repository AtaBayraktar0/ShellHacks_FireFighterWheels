import numpy as np
from pc.detector import posture_hint
from rover.camera import DemoCamera
from rover.perception import attach_depth


def test_pose_hint_never_infers_consciousness_or_mobility():
    keypoints = np.zeros((17, 3))
    assert posture_hint([0,0,100,40], keypoints) == "Pose occluded; assess visually"
    keypoints[5] = (10,10,.9)
    keypoints[6] = (10,20,.9)
    keypoints[11] = (80,10,.9)
    keypoints[12] = (80,20,.9)
    assert posture_hint([0,0,100,40], keypoints) == "Horizontal pose candidate; assess visually"


def test_pose_hint_and_camera_position_survive_depth_attachment():
    camera = DemoCamera().start()
    frame = camera.read()
    box = [130,116,201,320]
    result = attach_depth([{"kind":"person","confidence":.8,"box":box,
        "source":"PC person model","posture_hint":"Person visible; mobility unknown"}], frame)[0]
    assert result["posture_hint"] == "Person visible; mobility unknown"
    assert result["distance_m"] == 1.8
    assert result["position_m"][0] < 0
    assert result["position_m"][2] == 1.8
    camera.close()
