# Real person and fire models

Both downloaded models are installed locally and ran successfully on this
laptop's NVIDIA GeForce RTX 4060 Laptop GPU. They are separate trained models:

| Local file | Model and source | Labels | Published license |
| --- | --- | --- | --- |
| `models/yolo11n-pose.pt` | [Official Ultralytics YOLO11](https://huggingface.co/Ultralytics/YOLO11), nano pose model | `0: person`, plus body keypoints | AGPL-3.0 |
| `models/fire.pt` | [rabahdev YOLOv8n D-Fire fine-tune](https://huggingface.co/rabahdev/fire-smoke-yolov8n/blob/13017fe8af477c25f5298d168e2dfede4b000753/README.md) | `0: smoke`, `1: fire` | AGPL-3.0, as stated by its publisher |

The fire model is a community fine-tune, not an official Ultralytics fire model.
Its publisher describes training on D-Fire. D-Fire's authors publish the image
collection under [CC0 in their repository](https://github.com/gaia-solutions-on-demand/DFireDataset/blob/master/LICENSE).
This project did not train either model or independently reproduce the
publisher's held-out accuracy numbers.

Exact URLs, revision, SHA-256 hashes, labels, file sizes, and checkpoint checks
are stored in `models/provenance.json`. The fire checkpoint is pinned to revision
`13017fe8af477c25f5298d168e2dfede4b000753`. Before inference, setup verified each
hash, rejected non-allowlisted checkpoint globals, and successfully deserialized
with PyTorch `weights_only=True` and known PyTorch/Ultralytics classes. No custom
model repository code or training scripts were executed.

## Installed and measured

The separate `.venv-ai` environment has PyTorch **2.6.0+cu124**, torchvision
**0.21.0+cu124**, and Ultralytics **8.4.163**. CUDA 12.4 was available through the
existing GPU driver; no driver update or reboot was needed. The main `.venv`
remains separate, avoiding OpenCV package conflicts.

`reports/ai-smoke-test.json` records 20 measured inferences after three warmups
for each model at `imgsz=640`, batch one, confidence 0.25:

| Model | Median end-to-end inference | p95 | Observed public-sample result |
| --- | --- | --- | --- |
| Person/pose | 10.397 ms | 16.147 ms | Four person detections in the bundled Ultralytics bus image |
| Fire/smoke | 8.894 ms | 13.386 ms | Five fire and six smoke detections in the official D-Fire example collage |

These times include local preprocessing, inference, and postprocessing with GPU
synchronization. They exclude camera capture, HTTP transfer, Pi processing, and
dashboard rendering. They are not an end-to-end rover frame-rate guarantee.
Annotated outputs are in `reports/ai-smoke/`.

The [person sample is bundled by Ultralytics](https://github.com/ultralytics/ultralytics/blob/main/ultralytics/assets/bus.jpg).
The [fire example collage comes from the D-Fire authors](https://github.com/gaia-solutions-on-demand/DFireDataset/blob/master/figures/dfire_examples.png).
Examples can overlap model training data, so this test establishes installation
and inference operation, **not held-out accuracy**.

## Candles and live input

The trained fire detector is installed; **actual candle sensitivity has not been
tested**. A small flame occupies far fewer pixels than a large fire in a dataset.
Once the D435i and Pi stream are connected, test a stationary setup at planned
demo distances and lighting, including no-flame images and orange/light sources
that might cause false positives. Record detections, misses, confidence, distance,
and frame age before choosing the demo threshold. Lowering confidence can
increase false positives; it does not establish reliability.

The pose model detects people and body keypoints. It does not determine a
person's age, consciousness, health, or ability to evacuate. Any assistance
assessment remains an operator annotation.

## Reproduce the local checks

From a Windows terminal in the project directory:

```bat
.venv-ai\Scripts\python.exe -m pc.setup_models
.venv-ai\Scripts\python.exe -m pc.model_smoke_test --iterations 20
```

Setup reuses matching local files and refuses a mismatched checkpoint rather
than overwriting it. The smoke test uses only the two public example images;
it does not open a webcam, RealSense stream, or serial port. When models and sample
images already exist, they are reused locally.

Recreate the AI environment when moving the project:

```bat
py -3.12 -m venv .venv-ai
.venv-ai\Scripts\python.exe -m pip install torch==2.6.0 torchvision==0.21.0 --index-url https://download.pytorch.org/whl/cu124
.venv-ai\Scripts\python.exe -m pip install ultralytics==8.4.163 httpx==0.28.1
.venv-ai\Scripts\python.exe -m pc.setup_models
```

The application launcher/detector uses these local model paths. Keep them on the
laptop for GPU inference; the Raspberry Pi provides the camera and depth data.

## Start AI for a connected camera

Double-click **START_AI.cmd** and select **Joined Raspberry Pi**, **Local USB**, or
**Direct Raspberry Pi address**. Start the corresponding dashboard first. Joined
Pi reuses the laptop session privately; a direct Pi connection asks for its token
in a hidden console prompt. Tokens are not placed in worker command lines.

The default device is automatic: CUDA GPU 0 when available, otherwise CPU. Person
confidence defaults to 0.45 and fire/smoke confidence to 0.35. The worker runs both
local checkpoints, returns observations for the exact source frame, and discards
stale images/results. A changed Pi gateway connection stops the worker; restart
it after choosing the new source. These guards do not validate model accuracy.

For an explicit, bounded synthetic transport check only:

```bat
START_AI.cmd --source demo --allow-demo --max-frames 3 --max-seconds 10
START_AI.cmd --source joined --allow-demo --max-frames 3 --max-seconds 10
```

The second command requires a deliberately joined synthetic test server. Normal
AI launch refuses synthetic sources; the opt-in test labels its observations as
synthetic. Transport results in `reports/ai-transport-smoke.json` are separate
from the public-image model timing report and do not prove live camera operation.
