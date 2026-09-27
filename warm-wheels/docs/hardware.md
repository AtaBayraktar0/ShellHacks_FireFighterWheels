# Wiring and motor bring-up

Use controlled indoor testing with an immediate physical motor-power disconnect.
A software stop removes drive; it is not a mechanical brake. Keep wheels raised
until the checks below pass.

## Confirm the V4 wiring before enabling motors

The user confirmed the [ELEGOO Smart Robot Car Kit V4.0 with Camera](https://us.elegoo.com/products/elegoo-smart-robot-car-kit-v-4-0). The sketch selects
`MOTOR_PROFILE PROFILE_ELEGOO_V4` but ships with `HARDWARE_VERIFIED 0`. It compiles
with all motor IO disabled until that flag is explicitly changed. Inspect the
motor board label, connector labels, and wiring against the table below. Its
unverified handshake makes the Python client refuse motor control. **Keep motor
power physically disconnected while checking wiring**; firmware cannot ensure
the electrical state of unidentified or altered hardware.

For the matching ELEGOO Smart Robot Car V4 board, these pins are verified against
the [official ELEGOO driver header](https://github.com/elegooofficial/ELEGOO-Smart-Robot-Car-Kit-V4.0-New/blob/main/DeviceDriverSet.h)
and [driver implementation](https://github.com/elegooofficial/ELEGOO-Smart-Robot-Car-Kit-V4.0-New/blob/main/DeviceDriverSet.cpp):

| Signal | Uno pin |
| --- | --- |
| Right motor PWM | D5 |
| Left motor PWM | D6 |
| Right direction | D7 |
| Left direction | D8 |
| Driver standby | D3 |

Leave the selected V4 profile in place. Set `HARDWARE_VERIFIED 1` only after
matching that wiring, then perform the raised-wheel checks below. The implementation uses HIGH for forward on each direction
pin, following the official [motion control source](https://github.com/elegooofficial/ELEGOO-Smart-Robot-Car-Kit-V4.0-New/blob/main/MotionControl.cpp).
Actual mounting/wiring must still pass the raised-wheel direction test.

For a different kit with an L298N, select `PROFILE_L298N_CUSTOM` and explicitly
set all six `L298_*` pin definitions from the inspected wiring. There is no
guessed L298N pin map. PWM pins must be PWM-capable Uno pins; pins 0/1 are reserved
for serial. Remove ENA/ENB always-on jumpers if that board's schematic requires
PWM on those enables. Do not connect an Uno output against a strapped supply.
The compiler rejects omitted, invalid, or duplicate pins. Confirm logic voltage,
motor supply, enable wiring, and common ground for the actual motor module.

## Connections and power

- Pi 4 USB-A to Uno USB-B using a data cable. This carries serial and USB ground.
  Do not wire Uno 5 V TX directly to the Pi's 3.3 V GPIO UART.
- D435i USB-C to a Pi USB 3 port with a suitable USB 3 data cable. Plug the camera
  into the Pi, not the Uno. Check `lsusb -t` for the negotiated link speed.
- Power the Pi and camera from a suitable regulated supply; power motors from
  their kit battery/driver supply. Do not feed motors from the Pi or power the Pi
  from the Uno 5 V pin. Follow the actual shield's power/backfeed instructions.
- Remove or disconnect any kit camera/Bluetooth/ESP module that also drives
  Uno RX/TX, following its manual. The USB serial port must have a single owner.
- Secure the camera rigidly. Keep the emergency motor-power disconnect reachable.

Upload `firmware/rover_bridge/rover_bridge.ino` in Arduino IDE with **Arduino Uno**
selected. No additional Arduino libraries are needed. Close Serial Monitor before
starting the Pi application; only one process should own the USB serial port.

On Linux prefer `/dev/serial/by-id/...` to an unstable `/dev/ttyACM0` name:

```sh
ls -l /dev/serial/by-id/
```

Uno USB opening commonly resets the board. The client waits up to four seconds
for its boot handshake. If your adapter/board does not auto-reset, reset it while
connecting. Never bypass the handshake. A reset/power cycle clears the Uno's RAM
estop latch; it still boots with zero drive. The application must stay disarmed
after reconnecting or rebooting.

## Protocol and application contract

115200 baud, 8N1, newline-delimited ASCII. A verified boot emits two lines:

```text
READY WN1
PROFILE V4 VERIFIED
```

`PROFILE L298N VERIFIED` is the other accepted enabled profile. The supplied sketch's default build
emits `PROFILE V4 UNVERIFIED` because verification is disabled. "VERIFIED" records a build-time operator
selection; it is not automatic electrical detection.

| Command | Reply | Effect |
| --- | --- | --- |
| `M seq left right` | `ACK seq left right` | Signed integer PWM percentages, -100 to 100 |
| `S` | `STOPPED 0` or `STOPPED 1` | Zero drive; suffix reports estop latch |
| `E` | `ESTOP 1` | Zero drive and latch emergency stop |
| `R` | `ESTOP 0` | Clear latch while remaining stopped |

Sequence numbers are integers from 0 through 2147483647. They correlate replies;
the firmware does not treat them as timestamps or replay protection. Only one
command may be outstanding. Commands have no encoder velocity guarantee: 20%
PWM is not a speed in meters per second.

The firmware latches estop on malformed/oversized commands, partial lines left
incomplete for 100 ms, and the 350 ms motion watchdog. It replies `ERR reason`.
`S` always removes drive without clearing a latch. A valid `R` requires an explicit
operator decision; it never replays a previous command. Do not automatically
reset after a fault. Unplugged USB, a stalled Pi, or slow inference stops motion
through the Uno watchdog. Wheel inertia means physical travel can continue.

The Python API in `rover/serial_link.py` is:

```python
motor = SerialMotor(port, baud=115200)  # needs pyserial only in real mode
motor.connect()                       # blocking handshake + acknowledged stop
motor.drive(left, right)              # synchronous exact sequence/value ACK
motor.stop()
motor.estop()
motor.reset_estop()                   # explicit operator action only
state = motor.status()                # cached transport state, no IO
motor.close()                        # acknowledged stop, then close
```

`SimulationMotor()` implements the same public methods without touching serial.
The app may also use it in camera-only mode: a simulated `connected: true` only
means that this software object is connected. The application must keep its own
`motion_enabled` gate false in camera-only mode; simulated status does not establish
a real Arduino connection.
Catch `MotorLinkError` around hardware operations, disarm, and surface the error.
The default ACK deadline is 150 ms. Missing/incorrect ACK, firmware reset, serial
failure, or invalid drive input triggers a best-effort estop/stop, closes the port,
and raises; it never reports success. The physical result is unknown if serial
failed, so firmware watchdog and motor-power disconnect remain necessary.

`status()` includes `connected`, `motors_enabled`, `error`, `estop`, `profile`,
`left`, `right`, and `simulated`. Real mode also reports `last_ack_age_s`.
`left/right` are the last acknowledged requests, not encoder measurements;
`connected` is cached transport health, not a live poll. The next exchange detects
a disconnected cable or pending watchdog fault. `motors_enabled` means an accepted
hardware profile/connection, not permission to override the estop.

Refresh an allowed drive around 10 Hz from the control loop, with a fresh camera
and a recent operator command required by that loop. Do not put an independent
thread in charge of perpetually refreshing the last motor request: that would
defeat the application's stale-input stop behavior. Do not perform model inference
in a way that blocks the control loop. If processing cannot keep up, stop.

## Bench acceptance checks before floor operation

1. Disconnect motor power, upload the default disabled sketch, open Serial
   Monitor at 115200, and verify the disabled handshake. `M 1 20 20` must yield
   `ERR MOTORS_DISABLED`. Verify the Python client refuses that profile.
2. Inspect the board, select its profile/pins, set `HARDWARE_VERIFIED 1`, and upload.
   Keep wheels raised, reconnect motor power, and verify no movement at boot.
3. With Serial Monitor newline enabled, send `M 1 15 0` followed promptly by `S`.
   Confirm only the left side moves forward. Repeat `M 2 0 15` for right. A motor
   may need more PWM to overcome its deadband; increase cautiously while raised.
   If needed correct wiring or the corresponding `INVERT_*` constant.
4. Send a low-power `M` once, then send nothing. Confirm drive ceases by about
   350 ms and `ERR WATCHDOG` appears. It must refuse more `M` until `R`.
5. Verify `E` stops immediately; `S` does not clear it; `R` leaves wheels stopped.
6. Test malformed input, an overlong line, and an incomplete line. Each must stop
   and latch. Test unplugging USB and terminating the Pi process while raised.
7. Close Serial Monitor, start the real client, and verify an ACK fault disarms
   the application. Confirm reconnect/reboot never resumes an old command.
8. Start floor tests at low PWM in a clear, level area with a spotter holding the
   motor-power disconnect. Measure stopping distance and turn swept area; use
   those measurements to choose obstacle margins.

Host tests exercise serial failures using a fake device. The firmware was also
compiled for `arduino:avr:uno` with the installed AVR 1.8.8 core in three
configurations: shipped V4/unverified, V4/verified, and a custom L298N profile with
test-only pin assignments. No firmware was flashed. Compilation does not validate
wiring, direction, stopping distance, USB power, or watchdog timing on real
hardware. Those require the checks above.
