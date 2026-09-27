"""Download two pinned standard Ultralytics checkpoints and verify their hashes."""
from __future__ import annotations

import hashlib
import importlib
import json
import os
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT / "models"
(ROOT / "work/ultralytics").mkdir(parents=True, exist_ok=True)
os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / "work/ultralytics"))

SOURCES = {
    "person": {
        "file": "yolo11n-pose.pt",
        "url": "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n-pose.pt",
        "sha256": "869e83fcdffdc7371fa4e34cd8e51c838cc729571d1635e5141e3075e9319dc0",
        "publisher": "Ultralytics", "architecture": "YOLO11n pose",
        "license": "AGPL-3.0", "model_card": "https://huggingface.co/Ultralytics/YOLO11",
        "labels": {"0": "person"}, "dataset": "COCO pose",
    },
    "fire": {
        "file": "fire.pt",
        "url": "https://huggingface.co/rabahdev/fire-smoke-yolov8n/resolve/13017fe8af477c25f5298d168e2dfede4b000753/best.pt",
        "sha256": "b91633799ceb052c814b4f8b77a37efc9a40f002d528df97d74463585fa4f28f",
        "publisher": "rabahdev", "architecture": "YOLOv8n detection",
        "license": "AGPL-3.0 (publisher model card)",
        "model_card": "https://huggingface.co/rabahdev/fire-smoke-yolov8n/blob/13017fe8af477c25f5298d168e2dfede4b000753/README.md",
        "revision": "13017fe8af477c25f5298d168e2dfede4b000753",
        "labels": {"0": "smoke", "1": "fire"}, "dataset": "D-Fire, according to publisher",
    },
}

# Only classes/functions already shipped by PyTorch/Ultralytics are accepted.
# No remote Python source, import path supplied by a model card, or custom layer
# is executed. Hashes pin the exact files inspected for this project.
ALLOWED_GLOBALS = {
    "torch.nn.modules.activation.SiLU", "torch.nn.modules.batchnorm.BatchNorm2d",
    "torch.nn.modules.container.ModuleList", "torch.nn.modules.container.Sequential",
    "torch.nn.modules.conv.Conv2d", "torch.nn.modules.linear.Identity",
    "torch.nn.modules.pooling.MaxPool2d", "torch.nn.modules.upsampling.Upsample",
    "ultralytics.nn.tasks.DetectionModel", "ultralytics.nn.tasks.PoseModel",
    "ultralytics.nn.modules.conv.Concat", "ultralytics.nn.modules.conv.Conv",
    "ultralytics.nn.modules.conv.DWConv", "ultralytics.nn.modules.head.Detect",
    "ultralytics.nn.modules.head.Pose", "ultralytics.nn.modules.block.DFL",
    "ultralytics.nn.modules.block.Bottleneck", "ultralytics.nn.modules.block.SPPF",
    "ultralytics.nn.modules.block.C2f", "ultralytics.nn.modules.block.C2PSA",
    "ultralytics.nn.modules.block.Attention", "ultralytics.nn.modules.block.C3k2",
    "ultralytics.nn.modules.block.PSABlock", "ultralytics.nn.modules.block.C3k",
    "builtins.getattr",
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fetch_checkpoint(source, destination):
    destination = Path(destination)
    if destination.exists():
        if sha256(destination) != source["sha256"]:
            raise RuntimeError(f"Existing {destination.name} does not match the pinned model; refusing to load or overwrite it.")
        return destination
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".download")
    try:
        with urllib.request.urlopen(source["url"], timeout=60) as response, temporary.open("wb") as output:
            total = 0
            while chunk := response.read(1024 * 1024):
                total += len(chunk)
                if total > 25 * 1024 * 1024:
                    raise RuntimeError("Checkpoint exceeds the expected lightweight model size.")
                output.write(chunk)
        if sha256(temporary) != source["sha256"]:
            raise RuntimeError("Downloaded checkpoint checksum does not match the pinned source.")
        temporary.replace(destination)
        return destination
    finally:
        if temporary.exists():
            temporary.unlink()


def verify_checkpoint(path, expected_sha):
    if sha256(path) != expected_sha:
        raise RuntimeError("Checkpoint hash differs from pinned source.")
    import torch
    globals_found = set(torch.serialization.get_unsafe_globals_in_checkpoint(path))
    unexpected = globals_found - ALLOWED_GLOBALS
    if unexpected:
        raise RuntimeError("Checkpoint requests unapproved Python globals.")
    safe_objects = []
    for name in sorted(globals_found):
        module, attribute = name.rsplit(".", 1)
        safe_objects.append(getattr(importlib.import_module(module), attribute))
    # This performs a restricted deserialization, rather than trusting a scan alone.
    with torch.serialization.safe_globals(safe_objects):
        checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    model = checkpoint.get("ema") or checkpoint.get("model")
    if model is None:
        raise RuntimeError("Checkpoint has no standard Ultralytics model.")
    labels = {str(key): value for key, value in model.names.items()}
    return {"sha256_verified": True, "restricted_load_verified": True,
            "checkpoint_globals": sorted(globals_found), "labels": labels}


def main():
    MODELS.mkdir(parents=True, exist_ok=True)
    results = {}
    for key, source in SOURCES.items():
        path = fetch_checkpoint(source, MODELS / source["file"])
        verification = verify_checkpoint(path, source["sha256"])
        if verification["labels"] != source["labels"]:
            raise RuntimeError(f"Unexpected class labels in {key} checkpoint.")
        results[key] = {**source, "bytes": path.stat().st_size, "verification": verification}
        print(f"Verified {path.relative_to(ROOT)}: {verification['labels']}")
    (MODELS / "provenance.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
