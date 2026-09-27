from sim.gazebo.generate_worlds import SCENARIOS, build_world


def test_fixtures_have_self_contained_robot_and_rgbd():
    for scenario in SCENARIOS:
        root = build_world(scenario)
        assert not list(root.iter("uri"))  # no remote assets required
        robot = root.find("world/model[@name='warm_wheels']")
        assert robot is not None
        links = {node.attrib["name"] for node in robot.findall("link")}
        joints = {node.attrib["name"] for node in robot.findall("joint")}
        assert len(joints) == 4
        for joint in robot.findall("joint"):
            assert joint.findtext("parent") in links
            assert joint.findtext("child") in links
        drive = robot.find("plugin[@name='gz::sim::systems::DiffDrive']")
        assert len(drive.findall("left_joint")) == len(drive.findall("right_joint")) == 2
        assert {node.text for node in drive.findall("left_joint")+drive.findall("right_joint")} == joints
        assert robot.find("link/sensor[@type='rgbd_camera']") is not None
        assert robot.findtext("link/sensor/camera/image/width") == "640"
        assert root.find("world/model[@name='floor']") is not None


def test_distinct_scenarios_preserve_empty_negative_control():
    empty = {m.attrib["name"] for m in build_world("empty").findall("world/model")}
    assert not any("carton" in name or "fire" in name for name in empty)
    assert build_world("blocked-door").find("world/model[@name='closed_door']") is not None
    assert build_world("visual-distractor").find("world/model[@name='orange_nonfire_object']") is not None
