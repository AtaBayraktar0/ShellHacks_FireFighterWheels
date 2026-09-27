"""Separate capture and motor loops; inference never blocks motor watchdogs."""
from collections import OrderedDict, deque
import copy
import math
import threading
import time

import cv2
import numpy as np

from .camera import DemoCamera, RealSenseCamera
from .perception import attach_depth, flame_candidates
from .serial_link import SerialMotor, SimulationMotor
from .spatial import analyze, plan_path
from .safety import SafetyGate
from .mission import MissionSimulator
from .presentation import PresentationSimulator

CAMERA_ONLY_HINT = ("Camera-only launch: restart with --enable-motors --port <Uno serial port> "
                    "(or WARM_WHEELS_ENABLE_MOTORS=1 and WARM_WHEELS_SERIAL_PORT) after the bench checks")


class RoverRuntime:
    def __init__(self, mode="demo", port=None, enable_motors=False, camera=None, motor=None,
                 camera_height=.22, robot_half_width=.20, obstacle_height=.45):
        if not all(math.isfinite(v) and v > 0 for v in (camera_height, robot_half_width, obstacle_height)):
            raise ValueError("Measured camera/robot dimensions must be positive and finite")
        self.mode = mode
        self.geometry = dict(camera_height_m=camera_height, robot_half_width_m=robot_half_width,
                             obstacle_top_m=obstacle_height)
        self.camera = camera or (DemoCamera() if mode == "demo" else RealSenseCamera())
        self.motor = motor or (SimulationMotor() if mode == "demo" or not enable_motors else SerialMotor(port))
        self.gate = SafetyGate(motion_enabled=mode == "demo" or enable_motors, disabled_hint=CAMERA_ONLY_HINT)
        self.lock = threading.RLock()
        self.presentation_lock = threading.RLock()
        self.motor_lock = threading.Lock()
        self.quit = threading.Event()
        self.started_at = time.monotonic()
        self.frame_at = 0.0
        self.frame_id = 0
        self.frames = OrderedDict()
        self.jpeg = {"color": None, "depth": None, "raw": None}
        self.spatial = {"clearance_m": None, "depth_valid_fraction": 0.0, "grid": None, "points": []}
        self.local_detections = []
        self.remote_detections = []
        self.remote_at = 0.0
        self.remote_frame_id = None
        self.events = deque(maxlen=100)
        self.observations = OrderedDict()
        self.goal = None
        self.route = []
        self.requested = (0, 0)
        self.command_at = 0.0
        self.motion = (0, 0)
        self.camera_ok = False
        self.error = None
        self.camera_error = None
        self.block_reason = "Starting camera"
        self.threads = []
        self.mission = MissionSimulator() if mode == "demo" else None
        self.presentation = PresentationSimulator() if mode == "demo" else None

    def start(self):
        try:
            self.motor.connect()
            self.motor.stop()
            self.camera.start()
        except Exception as exc:
            self.error = f"Startup failed: {exc}"
            for closer in (self.motor.close, self.camera.close):
                try:
                    closer()
                except Exception:
                    pass
            raise RuntimeError(self.error) from exc
        for name, target in (("motor-watchdog", self._motor_loop), ("depth-capture", self._capture_loop),
                             ("mission-simulator", self._mission_loop)):
            thread = threading.Thread(name=name, target=target, daemon=True)
            self.threads.append(thread)
            thread.start()

    def close(self):
        self.quit.set()
        failure = None
        try:
            with self.motor_lock:
                with self.lock:
                    self.gate.armed = False
                    self.requested = self.motion = (0, 0)
                self.motor.stop()
        except Exception as exc:
            failure = exc
        finally:
            for thread in self.threads:
                thread.join(timeout=2.0)
            for closer in (self.motor.close, self.camera.close):
                try:
                    closer()
                except Exception as exc:
                    failure = failure or exc
        if failure:
            raise failure

    def _mission_loop(self):
        previous = time.monotonic()
        while not self.quit.is_set():
            now = time.monotonic()
            elapsed = min(.25, max(0.0, now - previous))
            previous = now
            with self.lock:
                if self.mission is not None and not self.gate.estop:
                    self.mission.tick()
            # Synthetic reconstruction can be substantially more expensive
            # than a sensor snapshot. It must never hold the camera/motor lock.
            with self.presentation_lock:
                if self.presentation is not None:
                    self.presentation.tick(elapsed)
            self.quit.wait(.1)

    def presentation_snapshot(self):
        with self.presentation_lock:
            if self.mode != "demo" or self.presentation is None:
                raise ValueError("Presentation scenarios are laptop simulation only; unavailable in hardware mode")
            return self.presentation.snapshot()

    def presentation_command(self, action, **kwargs):
        # Deliberately no motor lock, motor call, arming, or mission command:
        # this separate simulator cannot turn an action into physical motion.
        with self.presentation_lock:
            if self.mode != "demo" or self.presentation is None:
                raise ValueError("Presentation scenarios are laptop simulation only; unavailable in hardware mode")
            return self.presentation.control(action, **kwargs)

    def presentation_report(self):
        with self.presentation_lock:
            if self.mode != "demo" or self.presentation is None:
                raise ValueError("Presentation scenarios are laptop simulation only; unavailable in hardware mode")
            return self.presentation.report()

    def mission_command(self, action):
        with self.motor_lock:
            with self.lock:
                if self.mission is None:
                    raise ValueError("Physical missions need validated SLAM/localization and navigation; only the simulator is available")
                if self.gate.estop:
                    raise ValueError("Reset the emergency latch before starting/resuming the simulator")
                self.gate.armed = False
                self.requested = self.motion = (0, 0)
                self.motor.stop()
                if action == "start":
                    self.mission.start()
                elif action == "return":
                    self.mission.abort()
                else:
                    raise ValueError("Unknown mission action")
                return self.mission.snapshot()

    def _capture_loop(self):
        while not self.quit.is_set():
            try:
                frame = self.camera.read()
                spatial = analyze(frame, **self.geometry)
                candidates = flame_candidates(frame)
                if self.mode == "demo":
                    # Deliberately synthetic annotation: this is not model inference.
                    h, w = frame.rgb.shape[:2]
                    candidates.append({"kind": "person", "confidence": 1.0,
                        "box": [int(w*130/640), int(h*116/480), int(w*201/640), int(h*320/480)],
                        "source": "synthetic demo annotation", "posture_hint": "Unassessed"})
                candidates = attach_depth(candidates, frame)
                raw = self._encode(frame.rgb)
                depth8 = np.uint8(np.clip(np.nan_to_num(frame.depth, nan=0, posinf=0)*255/5, 0, 255))
                depth_color = cv2.applyColorMap(255-depth8, cv2.COLORMAP_TURBO)
                depth_color[(~np.isfinite(frame.depth)) | (frame.depth <= 0)] = 0
                with self.lock:
                    self.frame_id += 1
                    for index, detection in enumerate(candidates):
                        detection.update(id=f"local-{self.frame_id}-{index}", assessment="unassessed", source_frame_id=self.frame_id)
                        self.observations[detection["id"]] = (frame.captured_at, copy.deepcopy(detection))
                    self._prune_observations()
                    self.frames[self.frame_id] = frame
                    # Matching original RGB/depth is mandatory for remote detections.
                    while len(self.frames) > 24:
                        self.frames.popitem(last=False)
                    self.local_detections = candidates
                    self.spatial = spatial
                    self.frame_at = frame.captured_at
                    self.camera_ok = True
                    self.camera_error = None
                    self.jpeg = {"color": raw, "raw": raw, "depth": self._encode(depth_color)}
                    grid = spatial.get("grid")
                    self.route = plan_path(grid, self.goal) if grid and self.goal else []
                self.quit.wait(0.06)
            except Exception as exc:
                # Serialize with drive and API stop: no selected drive may follow
                # this fault transition. Firmware watchdog remains independent.
                with self.motor_lock:
                    with self.lock:
                        self.camera_ok = False
                        self.camera_error = f"Camera processing stopped: {exc}"
                        self.gate.armed = False
                        self.requested = self.motion = (0, 0)
                        self.route = []
                    try:
                        self.motor.stop()
                    except Exception as stop_exc:
                        with self.lock:
                            self.error = f"Motor fault: {stop_exc}"
                # No automatic reconnection/re-arm after a hardware fault.
                self.quit.wait(0.2)

    @staticmethod
    def _encode(rgb):
        ok, data = cv2.imencode(".jpg", rgb, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if not ok:
            raise RuntimeError("Could not encode camera image")
        return data.tobytes()

    def _motor_loop(self):
        last = (None, None)
        while not self.quit.is_set():
            with self.motor_lock:
                if self.quit.is_set():
                    break
                with self.lock:
                    now = time.monotonic()
                    motor_state = self._sync_motor_state()
                    command, self.block_reason = self.gate.output(now,
                        self.frame_at if self.camera_ok else 0,
                        self.spatial["clearance_m"], self.spatial["depth_valid_fraction"],
                        motor_state["connected"], self.command_at, self.requested)
                try:
                    if command != (0, 0):
                        self.motor.drive(*command)  # refresh firmware watchdog, even unchanged
                    elif last != (0, 0):
                        self.motor.stop()
                    last = command
                    with self.lock:
                        self.motion = command
                except Exception as exc:
                    with self.lock:
                        self.gate.armed = False
                        self.motion = (0, 0)
                        self.requested = (0, 0)
                        self.error = f"Motor fault: {exc}"
                        self.block_reason = "Motor fault; restart after inspection"
                    last = (None, None)
            self.quit.wait(0.075)

    def _environment_reason(self):
        motor_state = self._sync_motor_state()
        return self.gate.environment_reason(time.monotonic(), self.frame_at if self.camera_ok else 0,
            self.spatial["clearance_m"], self.spatial["depth_valid_fraction"], motor_state["connected"])

    def _sync_motor_state(self):
        motor_state = self.motor.status()
        if motor_state.get("estop"):
            self.gate.estop = True
            self.gate.armed = False
        if motor_state.get("error"):
            self.error = f"Motor fault: {motor_state['error']}"
        return motor_state

    def command(self, action, left=0, right=0):
        # Same lock order as motor thread prevents a drive after a processed stop.
        with self.motor_lock:
            with self.lock:
                if action == "arm":
                    if self.mission and self.mission.snapshot()["phase"] in ("exploring", "returning"):
                        raise ValueError("Wait for the simulated mission to return before manual drive")
                    reason = self._environment_reason()
                    if reason:
                        raise ValueError(reason)
                    self.requested = (0, 0)
                    self.command_at = 0
                    self.gate.armed = True
                elif action == "drive":
                    if not self.gate.armed:
                        raise ValueError("Arm first")
                    reason = self._environment_reason()
                    if reason:
                        self.gate.armed = False
                        self.requested = (0, 0)
                        self.motor.stop()
                        raise ValueError(reason)
                    if (left or right) and min(left, right) < max(left, right)*.45:
                        raise ValueError("Only gentle forward arcs are supported")
                    self.requested = (left, right)
                    self.command_at = time.monotonic()
                elif action in ("disarm", "estop", "reset"):
                    self.gate.armed = False
                    self.requested = (0, 0)
                    self.command_at = 0
                    self.motion = (0, 0)
                    if action == "estop":
                        self.gate.estop = True
                        self.motor.estop()
                    elif action == "reset":
                        self.motor.reset_estop()
                        self.gate.estop = False
                        if not self.motor.status().get("error"):
                            self.error = None
                    else:
                        self.motor.stop()
                else:
                    raise ValueError("Unknown action")
        return {"ok": True}

    def set_route(self, goal):
        with self.lock:
            grid = self.spatial.get("grid")
            if not grid or not self.camera_ok or time.monotonic()-self.frame_at > .45:
                raise ValueError("Current depth scan unavailable")
            if not (0 <= goal[0] < grid["width"] and 0 <= goal[1] < grid["height"]):
                raise ValueError("Goal outside scan")
            self.goal = list(goal)
            self.route = plan_path(grid, self.goal)
            return {"route": self.route, "preview_only": True,
                "message": "Local preview; no motor commands" if self.route else "No path through observed free space with footprint clearance"}

    def submit_detections(self, frame_id, detections):
        with self.lock:
            frame = self.frames.get(frame_id)
            now = time.monotonic()
            if frame is None or now-frame.captured_at > 1.2:
                raise ValueError("Inference frame expired; discard this result")
            if self.remote_frame_id is not None and frame_id <= self.remote_frame_id:
                raise ValueError("Out-of-order inference result")
            h, w = frame.rgb.shape[:2]
            for detection in detections:
                x1, y1, x2, y2 = detection["box"]
                if not (0 <= x1 < x2 <= w and 0 <= y1 < y2 <= h):
                    raise ValueError("Box outside matching source frame")
            result = attach_depth(detections, frame)
            for index, detection in enumerate(result):
                detection.update(id=f"remote-{frame_id}-{index}", assessment="unassessed", source_frame_id=frame_id)
                self.observations[detection["id"]] = (frame.captured_at, copy.deepcopy(detection))
            self._prune_observations()
            self.remote_detections = result
            self.remote_at = frame.captured_at
            self.remote_frame_id = frame_id
            return {"ok": True, "count": len(result)}

    def assess(self, detection_id, assessment):
        with self.lock:
            observation = self.observations.get(detection_id)
            if observation is None or time.monotonic()-observation[0] > 15:
                raise ValueError("Observation expired; assess a current observation")
            detection = observation[1]
            if detection["kind"] != "person":
                raise ValueError("Assistance annotations apply to person observations")
            # Snapshot only: do not transfer a judgment to a different person by position.
            event = {"id": detection_id, "assessment": assessment, "kind": "person",
                "time": time.time(), "distance_m": detection.get("distance_m"),
                "note": "Operator annotation of this observation; identity not tracked"}
            self.events.appendleft(event)
            return {"ok": True, "event": event}

    def _prune_observations(self):
        while self.observations and (len(self.observations) > 1500 or time.monotonic()-next(iter(self.observations.values()))[0] > 15):
            self.observations.popitem(last=False)

    def _detections(self, now):
        detections = copy.deepcopy(self.local_detections) if self.camera_ok and now-self.frame_at < .6 else []
        if now-self.remote_at < 1.5:
            detections.extend(copy.deepcopy(self.remote_detections))
        for detection in detections:
            detection["detection_age_s"] = round(now-(self.remote_at if detection["id"].startswith("remote") else self.frame_at), 2)
        return detections

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            environment_reason = self._environment_reason()
            mission = self.mission.snapshot() if self.mission else None
            can_arm = environment_reason is None and not (mission and mission["phase"] in ("exploring", "returning"))
            state = {"mode": self.mode, "uptime_s": round(now-self.started_at, 1),
                "frame_id": self.frame_id, "frame_age_s": round(now-self.frame_at, 3) if self.frame_at else None,
                "camera_ok": self.camera_ok and now-self.frame_at <= .45,
                "armed": self.gate.armed, "estop": self.gate.estop, "motion_enabled": self.gate.motion_enabled,
                "can_arm": can_arm, "can_drive": can_arm and self.gate.armed,
                "motion": dict(zip(("left", "right"), self.motion)), "motor": self.motor.status(),
                "detections": self._detections(now), "route": self.route,
                "inference_status": "Synthetic demonstration" if self.mode == "demo" else
                    ("PC detector connected" if now-self.remote_at < 1.5 else "PC detector offline; person detection unavailable"),
                "block_reason": environment_reason or self.block_reason,
                "error": self.error or self.camera_error, "events": list(self.events), "mission": mission,
                "mapping_status": "Camera-local scan only; no SLAM or global position"}
            state.update(self.spatial)
            return copy.deepcopy(state)

    def image(self, view):
        with self.lock:
            if not self.camera_ok or time.monotonic()-self.frame_at > .6:
                raise ValueError("Camera image unavailable or stale")
            return self.jpeg[view], self.frame_id, time.monotonic()-self.frame_at
