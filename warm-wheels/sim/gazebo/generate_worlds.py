"""Generate self-contained Gazebo rooms and a generic four-wheel rover.

Geometry, mass, friction and RGB-D settings are TEST FIXTURE assumptions, not a
calibrated digital twin of ELEGOO or D435i. Run Gazebo on a workstation, not Pi.
No autonomous rover adapter is supplied by this fixture.
"""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

SCENARIOS = ("empty", "blocked-door", "obstacle-course", "visual-distractor")


def element(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if text is not None:
        node.text = str(text)
    return node


def plugin(parent, library, classname):
    return element(parent, "plugin", filename=f"gz-sim-{library}-system", name=f"gz::sim::systems::{classname}")


def box_geometry(parent, size):
    return element(element(element(parent, "geometry"), "box"), "size", " ".join(map(str, size)))


def box(world, name, pose, size, color="0.65 0.70 0.74 1"):
    model = element(world, "model", name=name)
    element(model, "static", "true")
    element(model, "pose", " ".join(map(str, pose)) + " 0 0 0")
    link = element(model, "link", name="body")
    box_geometry(element(link, "collision", name="collision"), size)
    visual = element(link, "visual", name="visual")
    box_geometry(visual, size)
    material = element(visual, "material")
    element(material, "ambient", color)
    element(material, "diffuse", color)
    return model


def inertial(link, mass, diagonal):
    node = element(link, "inertial")
    element(node, "mass", mass)
    tensor = element(node, "inertia")
    for key, value in zip(("ixx", "iyy", "izz"), diagonal):
        element(tensor, key, value)
    for key in ("ixy", "ixz", "iyz"):
        element(tensor, key, 0)


def rover(world):
    model = element(world, "model", name="warm_wheels")
    element(model, "pose", "-2.25 -2.25 0.01 0 0 0")
    body = element(model, "link", name="chassis")
    element(body, "pose", "0 0 0.065 0 0 0")
    inertial(body, 1.0, (.0020, .0065, .0070))
    box_geometry(element(body, "collision", name="body_collision"), (.28, .16, .055))
    visual = element(body, "visual", name="body_visual")
    box_geometry(visual, (.28, .16, .055))
    material = element(visual, "material")
    element(material, "ambient", "1 0.35 0.08 1")
    element(material, "diffuse", "1 0.35 0.08 1")

    # Sensor is fixed relative to chassis: absolute camera height ~0.22m.
    sensor = element(body, "sensor", name="rgbd", type="rgbd_camera")
    element(sensor, "pose", "0.11 0 0.145 0 0 0")
    element(sensor, "topic", "/warm_wheels/rgbd")
    element(sensor, "always_on", "true")
    element(sensor, "update_rate", 15)
    camera = element(sensor, "camera")
    element(camera, "horizontal_fov", 1.20)
    image = element(camera, "image")
    element(image, "width", 640)
    element(image, "height", 480)
    element(image, "format", "R8G8B8")
    clip = element(camera, "clip")
    element(clip, "near", .1)
    element(clip, "far", 6.0)

    joints = {"left": [], "right": []}
    for side, y in (("left", .10), ("right", -.10)):
        for end, x in (("front", .085), ("rear", -.085)):
            name = f"{side}_{end}_wheel"
            link = element(model, "link", name=name)
            element(link, "pose", f"{x} {y} 0.033 1.57079632679 0 0")
            inertial(link, .06, (.000018, .000018, .000033))
            for tag in ("visual", "collision"):
                shape = element(link, tag, name=f"{name}_{tag}")
                cylinder = element(element(shape, "geometry"), "cylinder")
                element(cylinder, "radius", .033)
                element(cylinder, "length", .025)
                if tag == "visual":
                    material = element(shape, "material")
                    element(material, "ambient", ".08 .08 .08 1")
                    element(material, "diffuse", ".08 .08 .08 1")
                else:
                    ode = element(element(element(shape, "surface"), "friction"), "ode")
                    element(ode, "mu", .8)
                    element(ode, "mu2", .8)
            joint_name = f"{name}_joint"
            joint = element(model, "joint", name=joint_name, type="revolute")
            element(joint, "parent", "chassis")
            element(joint, "child", name)
            axis = element(joint, "axis")
            # Model-frame +y is wheel axle; explicit frame avoids link rotations.
            element(axis, "xyz", "0 1 0", expressed_in="__model__")
            limit = element(axis, "limit")
            element(limit, "lower", -1e16)
            element(limit, "upper", 1e16)
            element(limit, "effort", .3)
            element(limit, "velocity", 10)
            joints[side].append(joint_name)
    drive = plugin(model, "diff-drive", "DiffDrive")
    for side, names in joints.items():
        for name in names:
            element(drive, f"{side}_joint", name)
    for key, value in {
        "wheel_separation": .20, "wheel_radius": .033,
        "topic": "/warm_wheels/cmd_vel", "odom_topic": "/warm_wheels/odometry",
        "odom_publish_frequency": 15, "frame_id": "odom", "child_frame_id": "chassis",
        "max_linear_velocity": .15, "min_linear_velocity": -.15,
        "max_angular_velocity": .5, "min_angular_velocity": -.5,
        "max_linear_acceleration": .15, "min_linear_acceleration": -.15,
    }.items():
        element(drive, key, value)
    return model


def build_world(scenario):
    if scenario not in SCENARIOS:
        raise ValueError("Unknown fixture scenario")
    root = ET.Element("sdf", version="1.9")
    world = element(root, "world", name="warm_lab")
    physics = element(world, "physics", name="physics", type="ignored")
    element(physics, "max_step_size", .001)
    element(physics, "real_time_factor", 1)
    plugin(world, "physics", "Physics")
    plugin(world, "user-commands", "UserCommands")
    plugin(world, "scene-broadcaster", "SceneBroadcaster")
    element(plugin(world, "sensors", "Sensors"), "render_engine", "ogre2")
    light = element(world, "light", name="sun", type="directional")
    element(light, "pose", "0 0 8 0 0 0")
    element(light, "diffuse", ".9 .9 .9 1")
    element(light, "specular", ".1 .1 .1 1")
    element(light, "direction", "-.2 -.3 -1")
    element(light, "cast_shadows", "true")
    box(world, "floor", (0, 0, -.05), (6.2, 6.2, .1), ".25 .3 .32 1")
    for name, pose, size in (
        ("west", (-3, 0, .7), (.1, 6.1, 1.4)),
        ("east", (3, 0, .7), (.1, 6.1, 1.4)),
        ("north", (0, 3, .7), (6.1, .1, 1.4)),
        ("south", (0, -3, .7), (6.1, .1, 1.4)),
    ):
        box(world, name, pose, size)
    # 0.9 m openings in both room partitions.
    for index, (center, length) in enumerate(((-2.5, 1), (0, 2.2), (2.5, 1))):
        box(world, f"vertical_{index}", (0, center, .7), (.1, length, 1.4))
        box(world, f"horizontal_{index}", (center, 0, .7), (length, .1, 1.4))
    if scenario == "blocked-door":
        box(world, "closed_door", (0, -1.55, .7), (.12, .9, 1.4), ".4 .25 .12 1")
    elif scenario == "obstacle-course":
        for i, pose in enumerate(((-1.2, -2.2, .2), (-2.0, 1.4, .2), (1.1, 2.0, .2), (2, -1, .2))):
            box(world, f"carton_{i}", pose, (.45, .55, .4), ".4 .55 .6 1")
    elif scenario == "visual-distractor":
        box(world, "orange_nonfire_object", (-1, -2.2, .25), (.30, .35, .5), "1 .3 .02 1")
    rover(world)
    ET.indent(root, space="  ")
    return root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path(__file__).parent/"worlds")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for scenario in SCENARIOS:
        target = args.output/f"{scenario}.sdf"
        ET.ElementTree(build_world(scenario)).write(target, encoding="utf-8", xml_declaration=True)
        print(target)
    print("Generated fixtures. XML generation is not Gazebo execution or physical validation.")


if __name__ == "__main__":
    main()
