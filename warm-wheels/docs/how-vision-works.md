# How real vision works

The live system uses images from the D435i connected to the Raspberry Pi. The laptop runs the trained person and fire models on those images. The animated presentation is a separate simulator; its people, fire and response decisions are generated test conditions.

**The software can be checked on this laptop before installation. The complete Pi-and-rover system cannot be declared working until its camera, USB, power, network and motor setup are tested on the actual equipment.** Current results and their limits are in [validation.md](validation.md).

## From the camera to a detection

1. **The Pi captures color and depth together.** The service requests 640 × 480 images at 15 frames per second. It aligns depth to the color image so a color pixel can be matched to a distance measurement. Missing depth remains unknown. A missing or failed camera does not silently become a simulated feed.
2. **The Pi makes the latest camera image available over the local network.** Each JPEG has a frame number and age. The Pi retains up to 24 original color/depth pairs for matching later results. These are real camera frames, not a generated video.
3. **The laptop chooses the latest new frame when it is ready.** It processes one frame at a time and skips repeated frame numbers. It does not queue every camera frame. The requested 15 FPS camera setting is therefore not a promise of 15 AI updates per second; achieved speed depends on capture, networking and inference.
4. **Two trained models inspect the same color image.** The person/pose model and the fire/smoke model run on the laptop, using its CUDA GPU when available and otherwise its CPU. The Pi does not need to run those neural networks. The models currently inspect the image, not the depth map or a thermal image.
5. **Results return with their original frame number.** The Pi checks that the source frame is still available and recent. It then uses that frame's aligned depth, rather than whichever camera image happens to be newest, to estimate each detection's distance.
6. **The dashboard displays observations.** A bounding box, model score, source label and available depth estimate describe what was observed. Real detections do not initiate autonomous physical search, medical assessment or escort in this version.

The AI worker rejects input older than 0.9 seconds and discards results whose estimated age exceeds 1.1 seconds before upload. The Pi independently rejects results older than 1.2 seconds and out-of-order submissions. Connection-generation checks prevent an in-flight result being transferred to a different joined Pi. These checks protect matching and freshness; they do not make a prediction correct.

## What the models can tell us

| Component | Actual output | What it does not establish |
| --- | --- | --- |
| YOLO11n pose on the laptop | Person boxes and body keypoints; an optional horizontal-pose hint when enough keypoints are visible | Age, consciousness, injury, whether someone fainted, or ability to evacuate |
| YOLOv8n fire/smoke model on the laptop | Image regions resembling its learned fire or smoke examples | Temperature, a confirmed fire, its physical severity, or a safe route |
| Color heuristic on the Pi | Bright orange/red regions labeled an unverified fire candidate | Whether an orange object, lamp or reflection is actually burning |
| Aligned depth on the Pi | Approximate distance and camera-relative position for a detection | A person's identity or a fixed location in a whole-house map |

The models learned visual patterns during prior training; this project uses their saved weights. It did not train them on this rover's camera. The default person threshold is 0.45 and fire/smoke threshold is 0.35. A score is a model confidence value, not a verified probability that the scene is safe or dangerous. Lowering a threshold can produce more false positives.

Distance uses the median valid depth in the central part of the detection box. Background surfaces can contaminate that estimate. Too few valid measurements produce an unknown distance, not zero. The live 3D view is a camera-local depth surface; it does not accumulate the rover's movement into a registered house scan.

In the presentation, several visible samples of a **simulated actor response** drive assistance or escort decisions. In the real-camera path, assistance labels are operator annotations. The simulation's behavior is not evidence that the real model can diagnose a person or autonomously escort them.

## What has already been checked

The laptop software tests cover authentication, frame matching, stale-result rejection, source changes, missing depth, motor gates, and separation of simulation from hardware. Procedural campaigns exercise generated layouts, response decisions, fire changes, clearance and truthful return/blocked outcomes. See [validation results](validation.md) and [simulation behavior](presentation-behaviors.md).

Both real model files were hash-verified and successfully ran on public example images using this laptop's RTX 4060 GPU. That checks model installation and inference, not accuracy on unseen emergency scenes. The saved report does **not** test a live D435i stream, candle sensitivity, Wi-Fi latency, or physical driving. Sources, weights and measured inference timings are recorded in [model provenance](model-provenance.md).

## Before and during the first Pi installation

- **Before copying:** keep the model weights and Windows AI environment on the laptop. Use the Pi source package and [Pi quickstart](pi-quickstart.md). Preserve any existing teammate code. Identify the actual kit revision and wiring from [hardware.md](hardware.md).
- **On the Pi:** verify 64-bit OS/Python, USB 3 camera connection, separate suitable Pi power and local network access. Follow [Raspberry Pi setup](raspberry-pi.md); then run `bash scripts/prepare_pi.sh`. A compatible ARM RealSense binding and camera profile must be verified there. The helper stops if no suitable wheel is available.
- **Test capture before starting the server:** with motor power disconnected and other camera apps closed, run the camera-only bench below. It measures actual capture behavior and writes a new report. A failed result needs investigation; a pass is still only a capture check.

```sh
.venv-pi/bin/python -m bench.check_hardware --mode hardware --duration 10
```

- **Join the real stream:** run `bash scripts/start_pi_camera.sh` on the Pi. On the laptop, open `START_PRESENTATION.cmd`, choose **Connect Raspberry Pi**, and enter the Pi address and its printed session token. The camera-only starter disables motor output. Follow the [quickstart connection steps](pi-quickstart.md).
- **Start real AI:** run `START_AI.cmd` on the laptop and choose **Joined Raspberry Pi**. Keep the worker open. Normal AI launch rejects synthetic sources; the `--allow-demo` option is only for explicitly labeled transport testing.
- **Check actual vision:** with the rover stationary, verify people at the planned distances and lighting, including partial occlusion and negative scenes. Record detections, misses, false alarms, frame age and available depth. Check the intended fire demonstration separately; candle sensitivity is currently unverified. Follow [model validation guidance](model-provenance.md#candles-and-live-input).
- **Commission motors separately:** inspect the pin map, upload the appropriate verified firmware only after the required checks, and perform the documented raised-wheel handshake, stop and bounded pulse tests. Follow [hardware preparation](windows-hardware.md) and [hardware test stages](hardware-test-environment.md). Only then start supervised motor controls with the documented Pi rover launcher.

Passing laptop tests cannot substitute for those hardware stages. The current real system supports live sensing, laptop inference and supervised controls. Physical whole-place exploration, localization, automatic return and escort remain unimplemented; the simulator demonstrates their software behavior under explicit synthetic assumptions.
