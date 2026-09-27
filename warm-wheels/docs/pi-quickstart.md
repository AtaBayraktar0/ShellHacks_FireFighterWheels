# Join the Raspberry Pi from this laptop

The Pi is not connected yet. These steps prepare the first camera-only session; no autonomous movement is enabled by them.

## 1. Connect the devices

Use 64-bit Raspberry Pi OS. Connect D435i to a blue USB 3 port with a data cable. Connect the Uno to another Pi USB port after checking the kit wiring. Power the Pi separately through USB-C; the micro-HDMI ports are optional display outputs. Keep the motors disarmed for the camera check.

Put the Pi and laptop on the same private Wi-Fi or hotspot. Enable SSH on the Pi using Raspberry Pi configuration, with your own username. On the Pi run `hostname -I` to find its address. Do not send passwords in chat.

## 2. Copy the small Pi source package

The delivered `WARM-wheels-Pi-source.zip` excludes Windows environments, model weights and test recordings. The trained models stay on the laptop. Open PowerShell in the outputs folder containing that ZIP. Substitute your actual username and Pi IP:

```powershell
scp .\WARM-wheels-Pi-source.zip rover@192.168.1.42:~/
ssh rover@192.168.1.42
```

Verify the SSH host fingerprint against the Pi before accepting a first connection. In that Pi terminal:

```sh
mkdir -p ~/warm-wheels-install
python3 -m zipfile -e ~/WARM-wheels-Pi-source.zip ~/warm-wheels-install
cd ~/warm-wheels-install/warm-wheels
sudo apt update
sudo apt install python3-venv python3-dev build-essential cmake git libusb-1.0-0-dev pkg-config
bash scripts/prepare_pi.sh
```

If a compatible RealSense Python wheel is unavailable, the helper stops. Follow the source-build and USB permission instructions in [raspberry-pi.md](raspberry-pi.md), using the `.venv-pi` environment made by the helper. ARM dependencies and camera profiles must be checked on the actual Pi.

## 3. Start and join the stream

On the Pi, in the project directory:

```sh
bash scripts/start_pi_camera.sh
```

Keep its terminal open. It prints a token and local addresses. On the laptop:

1. Open the **WARM wheels Presentation** desktop shortcut or `START_PRESENTATION.cmd`.
2. Choose **Connect Raspberry Pi**.
3. Enter `http://PI_IP:8000` and the token printed by the Pi.
4. Choose **Join live stream**, then **Open 3D firefighter view**.
5. Run `START_AI.cmd`, choose **Joined Pi**, and keep that worker open for trained person/fire inference.

The laptop launcher's token signs into the laptop. The Pi token goes in the connection form. They need not match. Re-running the laptop launcher opens its current session without manual token entry. The Pi token is held in gateway memory only, so reconnect after a laptop-server restart.

## 4. Commission motors separately

Run the camera-only bench first:

```sh
.venv-pi/bin/python -m bench.check_hardware --mode hardware --duration 10 --output captures/pi-camera-bench.json
```

Follow [Windows hardware preparation](windows-hardware.md) and [hardware test environment](hardware-test-environment.md) for firmware upload and raised-wheel checks. Only after these pass, use `bash scripts/start_pi_rover.sh --port /dev/serial/by-id/YOUR_UNO --enable-motors` with the actual device path. This enables supervised controls; it does not implement physical autonomous search or escort.

The 1–50 m sliders control the simulation rectangle. A real metre boundary and return-home require validated localization and a persistent map. The current live mesh is camera-relative and does not accumulate into a house scan.

If joining fails, check that guest Wi-Fi client isolation is off, the Pi address is current, and the Pi service says it is listening on `0.0.0.0`. Keep the prototype on the trusted local network; do not port-forward it.
