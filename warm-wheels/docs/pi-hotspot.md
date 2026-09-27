# Rover Wi-Fi: connect the laptop directly to the Raspberry Pi

The Pi can broadcast **WARM-Wheels** as its own protected Wi-Fi network. The laptop joins it like any other Wi-Fi network, and the Pi always uses **10.42.0.1**. A router, venue Wi-Fi, and internet connection are unnecessary during the presentation.

This is prepared for **Raspberry Pi OS Bookworm or later using NetworkManager**. The installer is opt-in and has not been run on your unconnected Pi. It changes networking only. Camera commissioning, firmware upload, and motor checks remain separate.

| Connection | Address or credential |
| --- | --- |
| Wi-Fi network | `WARM-Wheels` by default; you can choose another name |
| Wi-Fi password | You choose it on the Pi; the script asks privately |
| Pi SSH | `YOUR_PI_USER@10.42.0.1` |
| Pi camera server | `http://10.42.0.1:8000` |
| Rover access token | Printed by `start_pi_camera.sh`; separate from the Wi-Fi password |
| Laptop dashboard | Open the existing **WARM wheels Presentation** shortcut |

Windows may show **Connected, no internet**. That is expected: the direct local connection still carries the camera stream, mesh, detections, and controls. This setup does not turn an unconfigured Pi into an internet-connected computer. The built-in Wi-Fi interface is used as an access point; a separate Ethernet or USB-network uplink would be needed to supply internet while it remains in that mode.

## First installation when the Pi has never joined Wi-Fi

Use the microSD card and a temporary monitor/keyboard for the first setup. The Pi does not need to be on a network for these steps.

1. On the laptop, use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to prepare a microSD card with **64-bit Raspberry Pi OS**. Select the correct card: writing an OS replaces its existing contents, so preserve any teammate code first. A desktop image is convenient for the first setup; Lite also works with a terminal.
2. In Imager's customisation, choose your hostname, your own username/password, locale, and correct wireless country. Enable SSH. You can leave Wi-Fi network credentials unconfigured. A blank OS image does not automatically broadcast this project's hotspot.
3. After Imager finishes, reinsert the microSD card into Windows. Copy **WARM-wheels-Pi-source.zip** onto the small, Windows-readable `bootfs` partition. Alternatively copy that ZIP to a USB flash drive. The ZIP contains project source, including the hotspot installer; it is not a bootable OS image.
4. Put the card in the Pi. Attach a keyboard and a monitor through **micro-HDMI → HDMI**, then power the Pi through USB-C. The laptop's HDMI port is normally an output, so it is not a substitute for a monitor input.
5. Log in as the username you configured. Open a terminal and extract the ZIP. On Bookworm, the boot partition is normally mounted at `/boot/firmware`:

```sh
mkdir -p ~/warm-wheels-install
python3 -m zipfile -e /boot/firmware/WARM-wheels-Pi-source.zip ~/warm-wheels-install
cd ~/warm-wheels-install/warm-wheels
```

If you used a USB drive, replace the ZIP path with its actual mounted path shown in the file manager. Do not unzip over an existing project with changes you want to preserve.

Raspberry Pi documents both local first boot and Imager SSH/account customisation in its [getting-started guide](https://www.raspberrypi.com/documentation/computers/getting-started.html). Bookworm no longer uses a `wpa_supplicant.conf` dropped onto `bootfs` to configure initial networking.

## Configure the direct Wi-Fi link

The hotspot helper uses only Python's standard library and the OS's existing NetworkManager. It does **not** need the rover Python environment or trained models.

First inspect the plan and available network interface:

```sh
python3 -m scripts.pi_hotspot plan
nmcli device status
```

The Pi 4's built-in wireless adapter is normally `wlan0`. If wireless is unavailable, run `sudo raspi-config` and set the WLAN country to the country where you will operate it. NetworkManager is the default from Bookworm onward; the [official Raspberry Pi networking guide](https://www.raspberrypi.com/documentation/computers/configuration.html#host-a-wireless-network-on-your-raspberry-pi) explains hotspot mode and wireless-country setup.

Apply the configuration **on the Pi**:

```sh
sudo bash scripts/setup_pi_hotspot.sh --apply --ssid WARM-Wheels --interface wlan0
```

Choose and repeat a **12–63 character password** when prompted. It is not put in the shell command, reports, or project files. The derived WPA2 key is stored in a root-readable NetworkManager profile so that the hotspot can return after reboot. Treat that stored key as a password too.

This creates one access-point profile with WPA2/AES, 2.4 GHz channel 6, DHCP, and the Pi address `10.42.0.1/24`. Other saved Wi-Fi profiles remain present. Activating the hotspot replaces the current connection on `wlan0`, so do the initial activation from the Pi's local console. If you deliberately run it through an existing Wi-Fi SSH session, expect that session to disconnect and reconnect through the new network afterward.

By default it broadcasts again after reboot. Add `--no-autoconnect` to the setup command if you only want to activate it for the current session. NetworkManager's [shared IPv4 mode](https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nm-settings-nmcli.html) supplies local addressing/DHCP; the profile file follows its [keyfile format](https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nm-settings-keyfile.html).

## Join from Windows and start the camera

1. Open Windows Wi-Fi settings, select **WARM-Wheels**, and enter the Wi-Fi password you just chose. Keep the connection if Windows notes that there is no internet.
2. In PowerShell, connect using your actual Pi username:

```powershell
ssh YOUR_PI_USER@10.42.0.1
```

On the first connection, verify the SSH fingerprint against the Pi console. You can display the Pi's Ed25519 host-key fingerprint locally with `ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub`. Use your account password or the SSH key configured in Imager; this is independent of the Wi-Fi password.

3. In the SSH terminal, start the installed camera service:

```sh
cd ~/warm-wheels-install/warm-wheels
bash scripts/start_pi_camera.sh
```

Keep that terminal open. The script binds the server to the Pi's network interfaces and prints a session access token. This is a **camera-only** launch with motor output disabled. If it says `.venv-pi` is missing, complete the dependency step below first.

4. Open **WARM wheels Presentation** on the laptop, then **Connect Raspberry Pi**. Enter **`http://10.42.0.1:8000`** and the **rover access token** from the SSH terminal. Choose **Join live stream → Open 3D firefighter view**.
5. Start `START_AI.cmd` on the laptop and select **Joined Pi**. The installed laptop models handle person/fire inference, so the 4 GB Pi does not need those large model files.

The laptop launcher's automatic login token, Pi rover token, Wi-Fi password, and SSH login are separate credentials. Reconnect the dashboard after restarting the laptop server, and use the new rover token after restarting the Pi camera script. Hotspot autoconnect starts networking, **not** the camera server; start the camera over SSH for each session.

## Internet-free presentation versus internet-free package installation

The hotspot and project source can be set up without putting the Pi on Wi-Fi. The camera application additionally needs ARM-compatible Python packages and RealSense bindings. The source ZIP is deliberately **not** a prevalidated Pi image or a complete offline dependency bundle.

The straightforward initial installation is to temporarily give the Pi internet over **Ethernet**, install the dependencies below, and then remove Ethernet for the demonstration. No Pi Wi-Fi connection to a router is required. If no wired internet is available, a separately prepared OS image or offline package cache must include packages matching the Pi's OS, ARM64 architecture, and Python version; the laptop's Windows virtual environment cannot be copied across as a substitute.

With a working temporary internet uplink, from the Pi project directory:

```sh
sudo apt update
sudo apt install python3-venv python3-dev build-essential cmake git libusb-1.0-0-dev pkg-config
bash scripts/prepare_pi.sh
```

If no compatible RealSense wheel is available, that helper stops. Follow [Raspberry Pi bring-up](raspberry-pi.md) for the source-build path. Complete a real camera check before disconnecting the temporary uplink:

```sh
.venv-pi/bin/python -m bench.check_hardware --mode hardware --duration 10 --output reports/pi-camera-bench.json
```

After installation, the laptop dashboard, locally stored model weights, and Pi camera service operate across the direct hotspot without internet. This does not add physical autonomous search, a reliable real-world boundary, or whole-house SLAM; see [the Pi quickstart](pi-quickstart.md) for current hardware limits and separate motor commissioning.

## Check, stop, and undo the hotspot

Read the managed state and adapter status on the Pi:

```sh
sudo python3 -m scripts.pi_hotspot status
ip -4 address show wlan0
```

To remove this project's hotspot and try to restore the previously active Wi-Fi profile:

```sh
sudo bash scripts/rollback_pi_hotspot.sh --apply
```

Rollback removes only the recorded profile UUID and its state file. Other saved networks are preserved. If the Pi had no previous Wi-Fi connection, use `nmtui` to choose one later. Run rollback from the local console when possible: disconnecting the hotspot will also disconnect an SSH session using it. Re-run setup to choose a new hotspot password.

If the network does not appear, check `nmcli device status`, `rfkill list`, and `journalctl -u NetworkManager -n 60 --no-pager` on the Pi. Confirm the wireless country and that the interface supports AP mode. If NetworkManager reports a missing DHCP helper, install the required `dnsmasq-base` package using a temporary uplink or an OS-matched offline package. The installer retains rollback state after an activation failure.

If Wi-Fi joins but the dashboard does not, confirm the Pi has `10.42.0.1`, the camera terminal is still running, and the new Pi token is entered. A VPN using `10.42.0.0/24` can conflict with this local route; disconnect that VPN during the demonstration. The helper does not change firewall rules, enable SSH, alter Windows adapters, or install a camera daemon.

## Verification status

The generated profile, secret handling, Linux/Pi guard, preservation of existing profiles, failure recovery state, and UUID-scoped rollback are covered by local automated tests using temporary files and mocked commands. **No real NetworkManager adapter, Pi hotspot, DHCP lease, or wireless camera stream has been exercised yet.** The first hardware check is to join `WARM-Wheels` and verify SSH before starting the camera.
