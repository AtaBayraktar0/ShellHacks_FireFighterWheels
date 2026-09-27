"""Opt-in Raspberry Pi NetworkManager hotspot; stdlib only, no remote changes.

The default command prints a plan. `apply` is Linux/root only and prompts for a
local passphrase. No secret is accepted as a command-line argument or logged.
"""
from __future__ import annotations

import argparse
import configparser
import getpass
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import uuid


PROFILE_NAME = "warm-wheels-hotspot"
PROFILE_PATH = Path("/etc/NetworkManager/system-connections/warm-wheels-hotspot.nmconnection")
STATE_PATH = Path("/var/lib/warm-wheels-hotspot/state.json")
ADDRESS = "10.42.0.1/24"
OWNER = "warm-wheels-hotspot-v1"


def validate_settings(ssid, interface):
    if not isinstance(ssid, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9 ._-]{0,31}", ssid) or ssid.endswith(" "):
        raise ValueError("SSID must be 1–32 ASCII letters, numbers, spaces, dots, underscores or hyphens, starting with a letter/number and without trailing spaces")
    if not isinstance(interface, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,14}", interface):
        raise ValueError("Interface must be a Linux interface name, such as wlan0")


def derive_psk(ssid, passphrase):
    if not isinstance(passphrase, str) or not 12 <= len(passphrase) <= 63 or any(not 32 <= ord(c) <= 126 for c in passphrase):
        raise ValueError("Choose a 12–63 character printable ASCII passphrase")
    # WPA2's standard SSID-bound PSK derivation. This key is still a secret;
    # storing the derived value avoids retaining the typed passphrase itself.
    return hashlib.pbkdf2_hmac("sha1", passphrase.encode("ascii"), ssid.encode("ascii"), 4096, 32).hex()


def render_profile(ssid, interface, connection_uuid, psk, autoconnect=True):
    validate_settings(ssid, interface)
    if str(uuid.UUID(connection_uuid)) != connection_uuid:
        raise ValueError("Invalid canonical connection UUID")
    if not re.fullmatch(r"[0-9a-f]{64}", psk):
        raise ValueError("Expected derived 256-bit WPA2 key")
    return f"""[connection]
id={PROFILE_NAME}
uuid={connection_uuid}
type=wifi
interface-name={interface}
autoconnect={'true' if autoconnect else 'false'}
autoconnect-priority=999

[wifi]
mode=ap
ssid={ssid}
band=bg
channel=6

[wifi-security]
key-mgmt=wpa-psk
proto=rsn;
pairwise=ccmp;
group=ccmp;
psk={psk}
psk-flags=0

[ipv4]
method=shared
address1={ADDRESS}
never-default=true

[ipv6]
method=disabled
"""


def _run(args, check=True):
    result = subprocess.run(args, capture_output=True, text=True, timeout=60, env={**os.environ, "LC_ALL": "C"})
    if check and result.returncode:
        # nmcli arguments here never contain the passphrase or derived key.
        raise RuntimeError((result.stderr or result.stdout or "Command failed").strip())
    return result


def _require_pi_linux_root():
    if platform.system() != "Linux":
        raise RuntimeError("Apply/rollback runs on the Raspberry Pi, not on this Windows laptop. Use plan here.")
    if os.geteuid() != 0:
        raise RuntimeError("Run this operation with sudo on the Raspberry Pi")
    model_path = Path("/proc/device-tree/model")
    if not model_path.exists() or "raspberry pi" not in model_path.read_text(errors="replace").lower():
        raise RuntimeError("This installer is scoped to a Raspberry Pi; no network changes were made")
    if not shutil.which("nmcli"):
        raise RuntimeError("NetworkManager/nmcli is required; use Raspberry Pi OS Bookworm or later with NetworkManager")
    _run(["systemctl", "is-active", "--quiet", "NetworkManager"])


def _private_write(path, contents):
    if path.is_symlink() or path.exists():
        raise RuntimeError(f"Refusing to overwrite existing path: {path}")
    if path.parent.is_symlink():
        raise RuntimeError(f"Refusing symlink directory: {path.parent}")
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(contents)


def plan(ssid="WARM-Wheels", interface="wlan0", autoconnect=True):
    validate_settings(ssid, interface)
    return {"operation": "plan only; no files or adapters changed", "ssid": ssid,
            "interface": interface, "pi_address": ADDRESS, "dashboard_address": "http://10.42.0.1:8000",
            "security": "WPA2/AES, user-chosen passphrase prompted locally", "band": "2.4 GHz, channel 6",
            "autoconnect_at_boot": autoconnect, "dhcp": "NetworkManager shared-mode DHCP",
            "requires_internet_to_operate": False, "provides_internet": "only if Pi has an additional working uplink",
            "profile_file": PROFILE_PATH.as_posix(), "rollback_state": STATE_PATH.as_posix(),
            "effect": "Activating AP mode replaces the current Wi-Fi connection on this interface; existing profiles are retained",
            "camera": "Start separately with bash scripts/start_pi_camera.sh; motors remain disabled"}


def apply(ssid, interface, autoconnect=True):
    validate_settings(ssid, interface)
    _require_pi_linux_root()
    if PROFILE_PATH.exists() or PROFILE_PATH.is_symlink() or STATE_PATH.exists() or STATE_PATH.is_symlink():
        raise RuntimeError("A hotspot profile/state already exists. Use status and rollback before creating another; nothing was overwritten.")
    existing = _run(["nmcli", "-g", "connection.uuid", "connection", "show", PROFILE_NAME], check=False)
    if existing.returncode == 0:
        raise RuntimeError(f"An existing NetworkManager profile is named {PROFILE_NAME}; refusing to replace it")
    device_type = _run(["nmcli", "-g", "GENERAL.TYPE", "device", "show", interface]).stdout.strip()
    if device_type != "wifi":
        raise RuntimeError("Selected interface is not a NetworkManager Wi-Fi device")
    ap_support = _run(["nmcli", "-g", "WIFI-PROPERTIES.AP", "device", "show", interface]).stdout.strip()
    if ap_support != "yes":
        raise RuntimeError("Wi-Fi device does not report access-point support; inspect nmcli and the WLAN country setting")
    previous = _run(["nmcli", "-g", "GENERAL.CON-UUID", "device", "show", interface]).stdout.strip()
    if previous in ("", "--"):
        previous = None
    elif str(uuid.UUID(previous)) != previous:
        raise RuntimeError("Could not identify the current connection UUID for rollback")
    print(f"Creating {ssid!r} on {interface}; Pi address will be 10.42.0.1.")
    print("Activation disconnects this interface's existing Wi-Fi/SSH link. Other saved profiles remain intact.")
    passphrase = getpass.getpass("Choose hotspot passphrase (12–63 characters): ")
    confirmation = getpass.getpass("Repeat hotspot passphrase: ")
    if passphrase != confirmation:
        raise ValueError("Passphrases do not match; nothing changed")
    psk = derive_psk(ssid, passphrase)
    del passphrase, confirmation
    profile_uuid = str(uuid.uuid4())
    profile = render_profile(ssid, interface, profile_uuid, psk, autoconnect)
    state = {"owner": OWNER, "profile_uuid": profile_uuid, "profile_path": str(PROFILE_PATH),
             "previous_uuid": previous, "interface": interface, "ssid": ssid,
             "autoconnect": autoconnect, "pi_address": ADDRESS}
    # Persist rollback information before activating anything, including if
    # changing the Wi-Fi interface terminates the calling SSH session.
    _private_write(STATE_PATH, json.dumps(state, indent=2) + "\n")
    try:
        _private_write(PROFILE_PATH, profile)
        _run(["nmcli", "connection", "load", str(PROFILE_PATH)])
        print("Profile saved. Join the new Wi-Fi from the laptop, then SSH to YOUR_USER@10.42.0.1.", flush=True)
        _run(["nmcli", "--wait", "45", "connection", "up", "uuid", profile_uuid])
    except Exception:
        print("Setup did not confirm activation. Rollback information is saved; run the rollback helper from the Pi console.", file=sys.stderr)
        raise
    print("Hotspot active. Windows may say 'No internet'; the local rover link still works.")
    print("Camera service: bash scripts/start_pi_camera.sh (run as your normal Pi user).")
    return state


def _read_state():
    if STATE_PATH.is_symlink():
        raise RuntimeError("Refusing symlink rollback state")
    if not STATE_PATH.exists():
        raise RuntimeError("No managed hotspot state exists")
    state = json.loads(STATE_PATH.read_text(encoding="utf-8"))
    if state.get("owner") != OWNER or state.get("profile_path") != str(PROFILE_PATH):
        raise RuntimeError("State does not describe this helper's managed profile")
    for key in ("profile_uuid", "previous_uuid"):
        if state.get(key) is not None and str(uuid.UUID(state[key])) != state[key]:
            raise RuntimeError("Invalid UUID in saved state")
    validate_settings(state["ssid"], state["interface"])
    return state


def rollback():
    _require_pi_linux_root()
    state = _read_state()
    if PROFILE_PATH.is_symlink():
        raise RuntimeError("Refusing symlink profile")
    if PROFILE_PATH.exists():
        parser = configparser.ConfigParser(interpolation=None)
        parser.read(PROFILE_PATH, encoding="utf-8")
        if parser.get("connection", "uuid", fallback=None) != state["profile_uuid"]:
            raise RuntimeError("Profile UUID changed; refusing to remove a different profile")
    # UUID, rather than connection name, scopes deletion to our created AP.
    _run(["nmcli", "connection", "down", "uuid", state["profile_uuid"]], check=False)
    present = _run(["nmcli", "-g", "connection.uuid", "connection", "show", "uuid", state["profile_uuid"]], check=False)
    if present.returncode == 0:
        _run(["nmcli", "connection", "delete", "uuid", state["profile_uuid"]])
    if PROFILE_PATH.exists():
        PROFILE_PATH.unlink()
        _run(["nmcli", "connection", "reload"])
    STATE_PATH.unlink()
    print("Only the managed WARM wheels hotspot was removed; other saved connections are preserved.")
    if state["previous_uuid"]:
        result = _run(["nmcli", "--wait", "30", "connection", "up", "uuid", state["previous_uuid"]], check=False)
        if result.returncode:
            print("Previous Wi-Fi profile could not reconnect. Select an available saved network with nmcli/nmtui.")
            return 1
        print("Previous Wi-Fi profile restored.")
    else:
        print("No prior active Wi-Fi profile was recorded. Use nmtui to choose a network when needed.")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", nargs="?", choices=("plan", "apply", "status", "rollback"), default="plan")
    parser.add_argument("--ssid", default="WARM-Wheels")
    parser.add_argument("--interface", default="wlan0")
    parser.add_argument("--no-autoconnect", action="store_true", help="Do not broadcast automatically after reboot")
    args = parser.parse_args(argv)
    try:
        if args.action == "plan":
            print(json.dumps(plan(args.ssid, args.interface, not args.no_autoconnect), indent=2))
        elif args.action == "apply":
            apply(args.ssid, args.interface, not args.no_autoconnect)
        elif args.action == "rollback":
            return rollback()
        else:
            _require_pi_linux_root()
            print(json.dumps(_read_state(), indent=2))
            result = _run(["nmcli", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device", "status"])
            print(result.stdout)
    except (ValueError, RuntimeError, OSError, subprocess.TimeoutExpired) as exc:
        print(f"Hotspot: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
