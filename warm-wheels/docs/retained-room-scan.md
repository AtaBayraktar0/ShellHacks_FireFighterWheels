# Retained real-camera room scans

This update adds experimental RGB-D visual odometry and retained measured mesh
keyframes to the existing `/scan` page. It does not enable physical autonomous
driving. The existing camera-only service and motor safety gates remain in place.

## Use on the Raspberry Pi

1. Launch `bash scripts/start_pi_camera.sh` and open `/scan` on that Pi.
2. Enter the session token, keep motor power off, and select **New retained scan**.
3. Begin with a level stationary camera at the configured camera height. Move it
   slowly by hand, keeping substantial overlap with textured, stationary surfaces.
4. Watch the tracking state and pose inlier count. `LOST` freezes the map. Stop
   moving, export that scan, then start a new scan; unrelated views are not merged.
5. **Stop scan** freezes acquisition. **Export snapshot** saves the accumulated
   metric vertices, colors, triangles, pose trajectory and tracking status as JSON.
   **Single frame** returns to the existing camera-local view.

The reference is the first camera pose; there is no measured ground plane or
IMU integration. ORB feature matches with source depth feed PnP/RANSAC, then
current depth checks the estimated motion. Accepted poses transform measured
surface patches into the reference frame. Source depth holes and discontinuities
remain gaps. Patches are captured after 10 cm translation or about 6 degrees of
rotation, with a 32-keyframe storage limit. On reaching the limit acquisition
stops, retaining all accepted patches. A new scan replaces the current in-memory
map, so export before starting again. Restarting the service clears that map.

This is odometry, not a full SLAM solution: no loop closure, global relocalization,
dynamic-object exclusion or independently verified position error. Repeated
patterns, moving subjects, glare, blank walls, smoke, fast motion and invalid
depth may cause failure or undetected drift. It cannot certify whole-room
coverage or serve as the sole localization source for unattended driving.

## What still blocks physical autonomous search

- The Arduino firmware identity, pin profile and stopping behavior must be
  established on the connected unit. A simulated motor object is not an Arduino
  connection; the dashboard now says **Camera only · Arduino not opened**.
- The prototype visual odometry needs measured motion and loss/recovery trials.
- A global traversability map, footprint-aware navigation controller and verified
  return-to-home behavior still need implementation and physical validation.
- Existing drive gates permit gentle forward arcs only; no pivot or reverse
  commands were enabled by this update.

Run `.venv-pi/bin/python -m scripts.pi_inventory` for a read-only USB/serial
inventory. This lists candidate ports but neither opens them nor identifies the
firmware. Keep existing firmware and motor configurations until that device
identity and the raised-wheel commissioning steps are completed.

## Validation

Unit tests recover a known metric camera transform with injected bad matches,
reject depth disagreement, track translation through actual ORB matching,
check pose coordinate conventions and retained geometry, enforce the storage
limit, latch tracking loss, reject timestamp gaps, enforce API authentication,
and verify mapping sends no drive commands. These checks are synthetic software
tests; they do not establish Pi throughput or real-room localization accuracy.

The pipeline follows OpenCV's documented object-to-camera PnP transform:
https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
