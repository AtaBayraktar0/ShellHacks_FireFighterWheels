# Architecture and path to physical autonomy

```text
D435i --USB 3--> Pi: capture + aligned depth + local geometry
                    |          |
                    |          +--> authenticated web dashboard (phone/PC)
                    +--JPEG--> PC: person/pose/custom fire model
                    <--boxes + frame ID--+
                    |
                    +--lease/clearance gates--> USB serial --> Uno --> V4 motors
                                                       watchdog + estop

Separate demo only:
hidden synthetic house --> observed occupancy + forward depth patches --> frontier exploration
                       --> path replanning --> return-to-home state machine
```

Camera processing, model inference and motor keepalive are separate. The PC pulls only the latest image. The Pi retains a bounded frame cache so detections can be joined to their original depth image; expired and out-of-order results are rejected. Live observations expire instead of staying on the map indefinitely. Manual judgments are snapshot records and are never silently attached to a different person.

The motor loop runs approximately every 75 ms plus serial acknowledgement time. It requires a browser drive command younger than 300 ms and camera data younger than 450 ms. The Uno independently stops after 350 ms without a valid motor command. These are software/firmware timing limits, not measured braking distances. Front-only depth does not validate the rear, turning footprint, stairs, smoke, or every obstacle height.

## Coordinate frames

Image boxes use pixels `[left, top, right, bottom]` in aligned RGB coordinates. Camera points use meters: x right, y up, z forward. Reported object depth is the median central bounding-box z, which can contain background. The local occupancy grid uses 10 cm cells and `-1=unknown, 0=observed-free, 1=occupied`; rows decrease as z increases. Its origin moves with the camera and resets every frame. It is not a stitched house map. Approximate detection markers are relative to the observation camera frame and are not world coordinates.

The local A* planner inflates obstacles, unknown cells and the grid border by rover radius. A complete footprint cannot fit at a front-camera map's bottom-edge origin, so this strict local preview often refuses every route. This is an intentional limitation of the available observation, not a prompt to reduce the robot to a point. The separate synthetic mission supplies a full exploration testbed.

## Before enabling real search and return

Build these integrations against real measurements; the hardware `/api/mission/start` endpoint deliberately refuses to pretend they exist:

1. **Persistent mapping and localization.** Add an RGB-D SLAM pipeline with timestamped color, depth and intrinsics, calibrated camera-to-base transform, valid pose estimates and failure detection. A possible integration is [RTAB-Map ROS](https://github.com/introlab/rtabmap_ros); it is not installed or configured by this repository. Wheel encoders/odometry would materially help; raw PWM is not odometry and IMU integration alone is not reliable position.
2. **Observed free-space coverage.** Fuse scans into a world-frame occupancy map. Track what has been observed and which frontiers are reachable. Empty detections are never a termination condition. Closed doors, occlusion, darkness, range limits and inaccessible rooms remain explicit gaps.
3. **Measured navigation controller.** Measure footprint, wheel directions, velocity versus command, braking, terrain and turn clearance. Add suitable side/rear/near-field/cliff sensing before reverse or rotating exploration. Use localization age/confidence and sensor health to stop motion.
4. **Mission adapter.** Replace the simulator sensor/motion adapter with real map/pose/path-following data. Only advance a waypoint after measured pose reaches it. Record the launch exit, reserve time/energy for returning, replan on obstacle changes, and report blocked if home becomes unreachable. Stop on pose loss; do not dead-reckon a fictional return.
5. **Assisted escort.** Require an operator-verified exit and explicit human consent/readiness. Add an appropriate instruction interface and verify the person is following; camera detection alone does not establish mobility. An unresponsive person cannot be escorted by this platform.

For the hackathon, demonstrate empty-room exploration in the simulator and live real-world depth/people reporting as separate tested capabilities. The next hardware milestone is a single measured room, a known start marker/exit, working localization, a stable occupancy map, and repeatable return trials—before any whole-building claim.
