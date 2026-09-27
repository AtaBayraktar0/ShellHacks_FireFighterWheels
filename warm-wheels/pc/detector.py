"""Pull matching RGB frames and return real person/fire model observations."""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import time
from urllib.parse import urlsplit, urlunsplit

import cv2
import httpx
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "work/ultralytics"
CONFIG_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("YOLO_CONFIG_DIR", str(CONFIG_DIR))
PERSON_MODEL = ROOT / "models/yolo11n-pose.pt"
FIRE_MODEL = ROOT / "models/fire.pt"
MAX_INPUT_AGE_S = .9
MAX_SUBMISSION_AGE_S = 1.1  # Pi rejects observations older than 1.2 seconds.


def normalize_base_url(value):
    parsed = urlsplit(value.strip())
    if parsed.scheme not in {"http", "https"} or not parsed.hostname or parsed.username or parsed.password or parsed.query or parsed.fragment:
        raise ValueError("Use an HTTP(S) rover base URL without credentials, query parameters, or fragments.")
    try:
        parsed.port
    except ValueError as exc:
        raise ValueError("Rover URL has an invalid port.") from exc
    return urlunsplit((parsed.scheme, parsed.netloc, parsed.path.rstrip("/") + "/", "", ""))


def resolve_device(requested="auto", torch_module=None):
    requested = str(requested).strip().lower()
    if requested == "auto":
        if torch_module is None:
            import torch as torch_module
        return "0" if torch_module.cuda.is_available() else "cpu"
    if requested != "cpu" and not requested.isdigit():
        raise ValueError("Device must be auto, cpu, or a GPU index such as 0.")
    return requested


def require_source_mode(state, allow_demo=False):
    mode = state.get("mode")
    if mode == "hardware":
        return mode
    if mode == "demo" and allow_demo:
        return mode
    if mode == "demo":
        raise RuntimeError("Synthetic source refused. Connect the real camera/Pi, or explicitly use --allow-demo for transport testing.")
    raise RuntimeError("Rover did not report a recognized hardware/demo source mode.")


def source_state(client, allow_demo=False):
    response = client.get("api/state")  # Relative paths preserve a gateway prefix.
    response.raise_for_status()
    state = response.json()
    if not isinstance(state, dict):
        raise ValueError("Invalid rover state response.")
    return require_source_mode(state, allow_demo)


def posture_hint(box, keypoints=None):
    if keypoints is None or len(keypoints) < 13:
        return "Pose unavailable; assess visually"
    indices = [5, 6, 11, 12]
    if any(len(keypoints[i]) < 3 or keypoints[i][2] < .5 for i in indices):
        return "Pose occluded; assess visually"
    shoulders = (np.array(keypoints[5][:2]) + np.array(keypoints[6][:2])) / 2
    hips = (np.array(keypoints[11][:2]) + np.array(keypoints[12][:2])) / 2
    torso = hips - shoulders
    if abs(torso[0]) > 1.4 * abs(torso[1]) and abs(torso[0]) > 12:
        return "Horizontal pose candidate; assess visually"
    return "Person visible; mobility unknown"


def extract(model, image, kind, device, imgsz, confidence, fire_names):
    result = model.predict(image, imgsz=imgsz, conf=confidence, device=device, verbose=False)[0]
    if result.boxes is None:
        return []
    boxes = result.boxes.xyxy.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy()
    scores = result.boxes.conf.cpu().numpy()
    pose = result.keypoints.data.cpu().numpy() if result.keypoints is not None else None
    h, w = image.shape[:2]
    detections = []
    for i, (box, cls, score) in enumerate(zip(boxes, classes, scores)):
        label = str(result.names[int(cls)]).lower()
        if (kind == "person" and label != "person") or (kind == "fire" and label not in fire_names):
            continue
        if not np.all(np.isfinite(box)) or not math.isfinite(float(score)):
            continue
        box = np.clip(box, [0, 0, 0, 0], [w, h, w, h]).tolist()
        if box[2] <= box[0] or box[3] <= box[1]:
            continue
        actual_kind = "person" if kind == "person" else "smoke_candidate" if "smoke" in label else "fire_candidate"
        detections.append({"kind": actual_kind, "confidence": float(score), "box": box,
            "source": "PC person/pose model" if kind == "person" else f"PC trained fire/smoke model: {label}; candidate",
            "posture_hint": posture_hint(box, pose[i] if pose is not None and i < len(pose) else None) if kind == "person" else "Unassessed"})
    return detections


def run_worker(client, person_model, fire_model, *, device="cpu", imgsz=640,
               person_confidence=.45, fire_confidence=.35, fire_names=None,
               allow_demo=False, max_frames=None, max_seconds=None,
               clock=time.monotonic, sleeper=time.sleep):
    """One matching frame/response at a time; no queues of obsolete images."""
    fire_names = fire_names or {"fire", "flame", "smoke"}
    stats = {"frames_seen": 0, "submitted": 0, "discarded": 0, "errors": 0,
             "mode": source_state(client, allow_demo), "device": device}
    started = clock()
    last_id = None
    source_generation = None
    is_gateway = client.base_url.path.rstrip("/").endswith("/api/pi/proxy")
    last_error = None
    last_error_at = 0.0
    while (max_frames is None or stats["frames_seen"] < max_frames) and (max_seconds is None or clock() - started < max_seconds):
        try:
            mode = source_state(client, allow_demo)
            if mode != stats["mode"]:
                raise RuntimeError("Source mode changed; restart the AI worker for the selected source.")
            requested_at = clock()
            response = client.get("api/inference-frame.jpg")
            response.raise_for_status()
            frame_id = int(response.headers["X-Frame-Id"])
            frame_age = float(response.headers["X-Frame-Age"])
            if frame_id < 1 or not math.isfinite(frame_age) or frame_age < 0:
                raise ValueError("Invalid frame ID/age metadata.")
            generation = response.headers.get("X-Pi-Generation")
            if is_gateway and (generation is None or not generation.isdigit()):
                raise RuntimeError("Gateway source generation is unavailable. Restart/update the laptop dashboard.")
            if source_generation is not None and generation != source_generation:
                raise RuntimeError("Joined Pi connection changed; restart the AI worker for the new source.")
            source_generation = generation
            if last_id is not None and frame_id < last_id:
                raise RuntimeError("Frame sequence restarted; restart the AI worker to attach to the new source session.")
            if frame_id == last_id:
                sleeper(.05)
                continue
            last_id = frame_id
            stats["frames_seen"] += 1
            if frame_age + (clock() - requested_at) > MAX_INPUT_AGE_S:
                stats["discarded"] += 1
                sleeper(.05)
                continue
            image = cv2.imdecode(np.frombuffer(response.content, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("Invalid camera JPEG.")
            detections = extract(person_model, image, "person", device, imgsz, person_confidence, fire_names)
            detections += extract(fire_model, image, "fire", device, imgsz, fire_confidence, fire_names)
            if mode == "demo":
                for detection in detections:
                    detection["source"] = "AI on synthetic demo frame; " + detection["source"]
            if frame_age + (clock() - requested_at) > MAX_SUBMISSION_AGE_S:
                stats["discarded"] += 1
                print("Slow inference: obsolete observation discarded before upload.", flush=True)
                continue
            headers = {"X-Pi-Generation": generation} if generation is not None else {}
            response = client.post("api/detections", headers=headers,
                                   json={"frame_id": frame_id, "detections": detections[:50]})
            if response.status_code == 409:
                stats["discarded"] += 1
                print("Rover rejected an expired/out-of-order frame; observation discarded.", flush=True)
            else:
                response.raise_for_status()
                stats["submitted"] += 1
            last_error = None
            sleeper(.05)
        except KeyboardInterrupt:
            break
        except httpx.HTTPStatusError as exc:
            if exc.response.status_code in {401, 403}:
                raise RuntimeError("Access token was rejected; reconnect and restart the AI worker.") from exc
            stats["errors"] += 1
            message = f"Source temporarily unavailable (HTTP {exc.response.status_code})."
            if message != last_error or clock() - last_error_at > 5:
                print(message, flush=True)
                last_error, last_error_at = message, clock()
            sleeper(.5)
        except (httpx.HTTPError, ValueError, KeyError):
            stats["errors"] += 1
            message = "Network/frame unavailable; waiting for fresh camera data."
            if message != last_error or clock() - last_error_at > 5:
                print(message, flush=True)
                last_error, last_error_at = message, clock()
            sleeper(.5)
    stats["elapsed_s"] = round(clock() - started, 3)
    return stats


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rover", required=True, help="Pi URL, local hardware URL, or laptop Pi proxy base URL")
    parser.add_argument("--person-model", default=str(PERSON_MODEL))
    parser.add_argument("--fire-model", default=str(FIRE_MODEL))
    parser.add_argument("--fire-labels", default="fire,flame,smoke")
    parser.add_argument("--device", default="auto", help="auto selects CUDA GPU 0 when available, otherwise CPU")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--person-confidence", type=float, default=.45)
    parser.add_argument("--fire-confidence", type=float, default=.35)
    parser.add_argument("--allow-demo", action="store_true", help="Explicitly allow synthetic-source transport testing")
    parser.add_argument("--max-frames", type=int, help="Stop after this many new frames, including discarded frames")
    parser.add_argument("--max-seconds", type=float, help="Bound a transport smoke test even if the camera stalls")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        base = normalize_base_url(args.rover)
        if not all(math.isfinite(v) and 0 < v <= 1 for v in (args.person_confidence, args.fire_confidence)):
            raise ValueError("Confidence thresholds must be finite values in (0, 1].")
        if not 160 <= args.imgsz <= 1280 or args.imgsz % 32:
            raise ValueError("Image size must be a multiple of 32 between 160 and 1280.")
        if args.max_frames is not None and args.max_frames < 1:
            raise ValueError("--max-frames must be positive.")
        if args.max_seconds is not None and (not math.isfinite(args.max_seconds) or args.max_seconds <= 0):
            raise ValueError("--max-seconds must be positive and finite.")
        if not all(Path(path).is_file() for path in (args.person_model, args.fire_model)):
            raise ValueError("Local model files are missing. Run .venv-ai\\Scripts\\python.exe -m pc.setup_models.")
        token = os.environ.get("ROVER_TOKEN", "")
        if len(token) < 16:
            raise ValueError("Use START_AI.cmd or provide the rover session token in ROVER_TOKEN.")
        device = resolve_device(args.device)
        with httpx.Client(base_url=base, headers={"Authorization": f"Bearer {token}"}, timeout=2.0,
                          trust_env=False, follow_redirects=False) as client:
            mode = source_state(client, args.allow_demo)
            print(f"Source: {base} | mode: {mode} | device: {device}", flush=True)
            print(f"Models: {Path(args.person_model).name} @ {args.person_confidence}; {Path(args.fire_model).name} @ {args.fire_confidence}", flush=True)
            from ultralytics import YOLO
            person_model, fire_model = YOLO(args.person_model), YOLO(args.fire_model)
            if "person" not in {str(v).lower() for v in person_model.names.values()}:
                raise ValueError("Person checkpoint has no person class.")
            fire_names = {s.strip().lower() for s in args.fire_labels.split(",") if s.strip()}
            if not {str(v).lower() for v in fire_model.names.values()} & fire_names:
                raise ValueError("Fire checkpoint labels do not match --fire-labels.")
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            for model in (person_model, fire_model):
                model.predict(blank, imgsz=args.imgsz, device=device, verbose=False)
            print("AI ready. Ctrl+C stops this worker. Pose cues need operator assessment; candle sensitivity is unverified.", flush=True)
            stats = run_worker(client, person_model, fire_model, device=device, imgsz=args.imgsz,
                person_confidence=args.person_confidence, fire_confidence=args.fire_confidence,
                fire_names=fire_names, allow_demo=args.allow_demo,
                max_frames=args.max_frames, max_seconds=args.max_seconds)
            print(f"AI stopped: {stats}", flush=True)
            return 0 if stats["submitted"] > 0 or args.max_frames is None and args.max_seconds is None else 1
    except KeyboardInterrupt:
        print("AI worker stopped.", flush=True)
        return 0
    except (ValueError, RuntimeError, httpx.HTTPError) as exc:
        print(f"AI not started/stopped: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
