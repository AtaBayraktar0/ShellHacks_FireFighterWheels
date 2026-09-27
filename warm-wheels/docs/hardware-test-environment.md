# Hardware test environment

Use a clear indoor bench, a stable stand holding every drive wheel off the
surface, a spotter, and a reachable motor-power disconnect. The camera/serial
check sends no drive commands. A separate pulse tool is only for raised wheels.

## Prepare the equipment

1. Confirm the ELEGOO V4 board and wiring against [hardware.md](hardware.md).
   The supplied firmware selects V4 but leaves `HARDWARE_VERIFIED 0`; that is the
   expected first commissioning state.
2. Install the Pi runtime requirements and the D435i Python binding described
   in [raspberry-pi.md](raspberry-pi.md). Use the Pi's USB 3 port and a USB 3
   camera data cable. Power the Pi/camera separately from the motor supply as
   described in the wiring guide.
3. Place a matte, well-lit target approximately 1–3 m in front of the camera.
   Avoid evaluating initial depth validity against a blank window, mirror, or
   a scene dominated by objects closer than the sensor's operating range.
4. Stop the dashboard process, Arduino Serial Monitor, PC viewer, and any other
   process using the camera or Uno serial port. Only this test should own them.
5. Run commands from the `warm-wheels` project directory. Reports are newly
   created JSON files in `captures/`; existing report files are never overwritten.

## Stage 1: prove the software test path without devices

```sh
python -m bench.check_hardware --mode demo --duration 10
```

This uses synthetic camera frames, opens no serial device, and labels the report
`mode: demo`. A pass validates this test path only. Demo mode rejects `--serial`.

## Stage 2: camera and USB capture check with motor power disconnected

```sh
python -m bench.check_hardware --mode hardware --duration 10 --output captures/camera-first.json
```

The runner starts the D435i at 640 × 480 and 15 FPS and measures host capture rate,
maximum frame gap, maximum delivery age, and whole-image valid depth fractions.
It always attempts camera cleanup, including after capture errors or Ctrl+C.
The requested duration must be greater than zero and at most 60 seconds; a device
read/cleanup timeout can make wall-clock duration longer.

On Linux the report includes only recognized camera/serial USB product IDs and
negotiated link speeds. It does not read USB serial-number fields, hostname,
network addresses, full environment variables, camera images, or access tokens.
Other operating systems report that USB evidence is unavailable. The absence
of a recognized USB entry does not establish that a device is disconnected.

The initial, documented acceptance limits are:

| Measurement | Pass criterion | Interpretation |
| --- | --- | --- |
| Frames | At least 2 | Enough samples to measure an interval |
| Host capture rate | At least 10 FPS | Headroom below the requested 15 FPS stream |
| Maximum frame gap | At most 0.45 s | Matches the application's frame-age limit |
| Maximum delivery age | At most 0.45 s | Wrapper host timestamp to processed delivery |
| Mean valid depth | At least 85% | Finite depth from 0.1 through 6.0 m over the whole image |
| USB camera link | Target at least 5000 Mbps | Advisory link evidence; lower speed warns |

The depth ratio depends on the test scene. It does not certify obstacle detection
or replace the application's forward-view safety check. Sensor exposure timing
is not independently calibrated here. No recognized USB link is marked
`unavailable`; it is not silently recorded as a pass. Overall `pass` refers to
the measured capture/serial checks, and can coexist with a USB advisory warning.

## Stage 3: optional Uno handshake and stop, still no drive

```sh
python -m bench.check_hardware --mode hardware --duration 10 --serial /dev/serial/by-id/YOUR_UNO_DEVICE --output captures/camera-and-uno.json
```

Replace the port with the actual stable device path from `ls -l /dev/serial/by-id/`.
Windows also accepts an explicit port such as `COM4`.

This option actively opens USB serial, which commonly resets an Uno, performs
the WN1 handshake, and sends acknowledged stop commands. It never sends a drive
command or clears an emergency latch. **It is not a passive electrical read.**

With the shipped `HARDWARE_VERIFIED 0` build, expect `verdict: blocked` and
`firmware_profile_not_verified`. That is an expected commissioning result,
not permission to bypass the check. Camera measurements still run and can fail
independently. After inspecting the V4 pin map, uploading the verified build,
and keeping wheels raised, repeat this stage to confirm handshake and stop ACKs.

An existing estop produces `blocked` and remains latched. Follow the explicit
operator reset procedure in the main hardware guide; neither bench tool resets
it. An Uno power/reset event itself clears a RAM latch and boots stopped, so
do not interpret reconnecting as proof that the previous session stayed latched.

## Stage 4: one low-power raised-wheel pulse

Only after matching the V4 wiring and uploading `HARDWARE_VERIFIED 1`, place
the chassis on its stand and have the spotter ready at the power disconnect.
Run the left and right sides separately:

```sh
python -m bench.motor_pulse --port /dev/serial/by-id/YOUR_UNO_DEVICE --wheels-raised --confirm-pin-check --left 15 --right 0 --duration 0.15 --output captures/left-wheel.json
python -m bench.motor_pulse --port /dev/serial/by-id/YOUR_UNO_DEVICE --wheels-raised --confirm-pin-check --left 0 --right 15 --duration 0.15 --output captures/right-wheel.json
```

Both flags are required operator attestations. They do not detect the stand,
wiring, or a person's presence. Each PWM request must be an integer from 0 to 20.
Duration must be greater than zero and at most 0.25 seconds; default is 0.15.
The tool sends exactly one drive request, never refreshes it, counts ACK wait
time against the requested duration, then attempts stop and close in `finally`.
An existing estop or unverified firmware prevents the pulse. Do not use this
script for ground driving.

Host scheduling is not a hard timing guarantee. If the process or USB link stalls,
the separately implemented Uno 350 ms watchdog is the fallback. A process killed
before Python cleanup runs may never send the final stop. PWM removal also does
not stop wheel inertia instantly.

Record the **observed** side, direction, stop behavior, and any abnormal sound or
motion in the test log. A successful ACK proves command acceptance only. A wheel
that does not turn at 15–20% PWM may be below its startup threshold; do not remove
the script's bound to force a result. Inspect the hardware and record that
observation. Correct wiring or `INVERT_LEFT`/`INVERT_RIGHT` only after stopping and
disconnecting motor power, then rebuild and repeat.

## Evidence and failure handling

Both tools write `schema_version: 1`, a suite identifier, UTC creation time,
`verdict`, sanitized error codes, and measured evidence. Camera reports also
include `checks`, `limits`, `metrics`, mode, scope, and limited public environment
information. Pulse reports contain requested PWM/duration, operator flags,
drive-call count, drive/stop ACKs, port-close outcome, and elapsed time.

Exit codes are **0 pass**, **1 fail**, and **2 blocked**. Invalid CLI arguments
also use exit code 2 without opening devices. A reserved report left as
`verdict: incomplete` means execution never produced a completed report; retain
it as incomplete evidence. The pulse's `pass` is a transport result, not proof of
physical motion or braking. Stop progression after any unexpected failure,
inspect wiring/power/log evidence, and rerun the same stage using a new report
filename. See [test-plan.md](test-plan.md) for the wider staged validation process.
