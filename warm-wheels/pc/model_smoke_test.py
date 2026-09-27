"""Real model inference on public samples; this is not an accuracy benchmark."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.metadata
import json
import os
from pathlib import Path
import shutil
import statistics
import time
import urllib.request

from .setup_models import ROOT, MODELS, SOURCES, fetch_checkpoint, sha256, verify_checkpoint

os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / "work/ultralytics"))
FIRE_SAMPLE_URL = "https://raw.githubusercontent.com/gaia-solutions-on-demand/DFireDataset/master/figures/dfire_examples.png"


def sample_images():
    import ultralytics
    samples = MODELS / "samples"
    samples.mkdir(parents=True, exist_ok=True)
    person = samples / "ultralytics-bus.jpg"
    if not person.exists():
        shutil.copy2(Path(ultralytics.__file__).parent / "assets/bus.jpg", person)
    fire = samples / "dfire-examples.png"
    if not fire.exists():
        with urllib.request.urlopen(FIRE_SAMPLE_URL, timeout=30) as response:
            content = response.read(15 * 1024 * 1024)
        fire.write_bytes(content)
    return {"person": person, "fire": fire}


def run_test(iterations=20):
    if not 1 <= iterations <= 100:
        raise ValueError("Iterations must be 1 through 100.")
    import cv2
    import torch
    from ultralytics import YOLO
    device = 0 if torch.cuda.is_available() else "cpu"
    report = {"created_utc": datetime.now(timezone.utc).isoformat(),
              "test_type": "installation_and_inference_smoke_test_not_accuracy_benchmark",
              "torch": torch.__version__, "cuda_runtime": torch.version.cuda,
              "cuda_available": torch.cuda.is_available(),
              "device": torch.cuda.get_device_name(0) if device == 0 else "CPU",
              "ultralytics": importlib.metadata.version("ultralytics"),
              "iterations_after_three_warmups": iterations, "image_size": 640,
              "models": {}, "candle_validation": "Not performed; candle flame sensitivity is unknown",
              "hardware_camera_tested": False,
              "limitations": ["Public example images can overlap training data; these results are not held-out accuracy.",
                              "No live candle, D435i feed, Wi-Fi latency or real-time robot pipeline was tested.",
                              "Pose cues do not establish age, consciousness or ability to evacuate."]}
    samples = sample_images()
    report_dir = ROOT / "reports/ai-smoke"
    report_dir.mkdir(parents=True, exist_ok=True)
    for kind, source in SOURCES.items():
        path = fetch_checkpoint(source, MODELS / source["file"])
        verification = verify_checkpoint(path, source["sha256"])
        image = cv2.imread(str(samples[kind]))
        if image is None:
            raise RuntimeError("Sample image could not be read.")
        model = YOLO(str(path))
        for _ in range(3):
            model.predict(image, device=device, imgsz=640, conf=.25, verbose=False)
        elapsed, inference = [], []
        for _ in range(iterations):
            if device == 0:
                torch.cuda.synchronize()
            start = time.perf_counter()
            result = model.predict(image, device=device, imgsz=640, conf=.25, verbose=False)[0]
            if device == 0:
                torch.cuda.synchronize()
            elapsed.append((time.perf_counter() - start) * 1000)
            inference.append(result.speed["inference"])
        detections = [{"class": model.names[int(box.cls.item())], "confidence": round(float(box.conf.item()), 4)} for box in result.boxes]
        cv2.imwrite(str(report_dir / f"{kind}-prediction.jpg"), result.plot())
        sorted_times = sorted(elapsed)
        p95 = sorted_times[min(len(sorted_times) - 1, int(.95 * len(sorted_times)))]
        report["models"][kind] = {"file": str(path.relative_to(ROOT)), "labels": verification["labels"],
            "sha256": source["sha256"], "sample": str(samples[kind].relative_to(ROOT)),
            "sample_sha256": sha256(samples[kind]), "detections": detections,
            "median_end_to_end_ms": round(statistics.median(elapsed), 3),
            "p95_end_to_end_ms": round(p95, 3),
            "mean_model_inference_ms": round(statistics.mean(inference), 3),
            "installation_inference_pass": any(item["class"] == ("person" if kind == "person" else "fire") for item in detections)}
    report["verdict"] = "pass" if all(item["installation_inference_pass"] for item in report["models"].values()) else "review_sample_detections"
    (ROOT / "reports/ai-smoke-test.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="Verify real person/fire models and GPU timing on public example images.")
    parser.add_argument("--iterations", type=int, default=20)
    args = parser.parse_args(argv)
    report = run_test(args.iterations)
    return 0 if report["verdict"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
