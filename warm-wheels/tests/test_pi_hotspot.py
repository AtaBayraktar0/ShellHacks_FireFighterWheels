"""Hotspot configuration tests; no real adapters, nmcli or root paths used."""
import configparser
import json
from pathlib import Path
import subprocess
import uuid

import pytest

from scripts import pi_hotspot as hotspot


def test_default_plan_does_not_access_network_or_prompt(monkeypatch, capsys):
    def forbidden(*args, **kwargs):
        pytest.fail("Read-only plan attempted a system command or secret prompt")
    monkeypatch.setattr(hotspot, "_run", forbidden)
    monkeypatch.setattr(hotspot.getpass, "getpass", forbidden)
    assert hotspot.main([]) == 0
    plan = json.loads(capsys.readouterr().out)
    assert plan["pi_address"] == "10.42.0.1/24"
    assert not plan["requires_internet_to_operate"]


def test_keyfile_is_protected_wpa2_ap_with_stable_dhcp_address():
    secret = "local test phrase only"
    key = hotspot.derive_psk("WARM-Wheels", secret)
    profile = hotspot.render_profile("WARM-Wheels", "wlan0", str(uuid.uuid4()), key)
    assert secret not in profile
    parsed = configparser.ConfigParser(interpolation=None)
    parsed.read_string(profile)
    assert parsed["wifi"]["mode"] == "ap"
    assert parsed["wifi-security"]["key-mgmt"] == "wpa-psk"
    assert parsed["wifi-security"]["proto"] == "rsn;"
    assert parsed["wifi-security"]["psk"] == key and len(key) == 64
    assert parsed["ipv4"]["method"] == "shared"
    assert parsed["ipv4"]["address1"] == "10.42.0.1/24"
    assert parsed["connection"]["autoconnect"] == "true"
    assert key != hotspot.derive_psk("Different-SSID", secret)


@pytest.mark.parametrize("ssid,interface", [("bad\n[ipv4]", "wlan0"), ("-option", "wlan0"), ("x"*33, "wlan0"), ("Rover", "wlan0;cmd"), ("Rover", "../wlan0")])
def test_untrusted_settings_cannot_inject_keyfile_or_command(ssid, interface):
    with pytest.raises(ValueError):
        hotspot.validate_settings(ssid, interface)


@pytest.mark.parametrize("secret", ["short", "x"*64, "valid-length\nline", "nonASCII-\u00e9-phrase"])
def test_bad_passphrases_rejected(secret):
    with pytest.raises(ValueError):
        hotspot.derive_psk("WARM-Wheels", secret)


@pytest.fixture
def fake_pi(tmp_path, monkeypatch):
    monkeypatch.setattr(hotspot, "PROFILE_PATH", tmp_path / "connections" / "warm-wheels.nmconnection")
    monkeypatch.setattr(hotspot, "STATE_PATH", tmp_path / "state" / "state.json")
    monkeypatch.setattr(hotspot, "_require_pi_linux_root", lambda: None)
    monkeypatch.setattr(hotspot.getpass, "getpass", lambda _prompt: "test-only-local-passphrase")
    prior = str(uuid.uuid4())
    calls = []
    def run(args, check=True):
        calls.append(list(args))
        if args[-1] == hotspot.PROFILE_NAME and "show" in args:
            return subprocess.CompletedProcess(args, 10, "", "not found")
        if "GENERAL.TYPE" in args:
            return subprocess.CompletedProcess(args, 0, "wifi\n", "")
        if "WIFI-PROPERTIES.AP" in args:
            return subprocess.CompletedProcess(args, 0, "yes\n", "")
        if "GENERAL.CON-UUID" in args:
            return subprocess.CompletedProcess(args, 0, prior + "\n", "")
        return subprocess.CompletedProcess(args, 0, "", "")
    monkeypatch.setattr(hotspot, "_run", run)
    return prior, calls


def test_apply_preserves_prior_and_never_puts_secret_on_command_line(fake_pi, capsys):
    prior, calls = fake_pi
    state = hotspot.apply("My-Rover", "wlan0")
    assert state["previous_uuid"] == prior
    saved = json.loads(hotspot.STATE_PATH.read_text())
    assert saved["owner"] == hotspot.OWNER
    assert "psk" not in saved and "password" not in saved
    assert hotspot.PROFILE_PATH.exists()
    outputs = capsys.readouterr().out
    key = hotspot.derive_psk("My-Rover", "test-only-local-passphrase")
    assert key not in outputs and "test-only-local-passphrase" not in outputs
    assert all(key not in " ".join(call) and "passphrase" not in " ".join(call) for call in calls)
    assert not any("delete" in call for call in calls)
    assert calls[-1] == ["nmcli", "--wait", "45", "connection", "up", "uuid", state["profile_uuid"]]


def test_rollback_only_deletes_managed_uuid_and_restores_previous(fake_pi):
    prior, calls = fake_pi
    state = hotspot.apply("My-Rover", "wlan0")
    calls.clear()
    assert hotspot.rollback() == 0
    assert not hotspot.PROFILE_PATH.exists() and not hotspot.STATE_PATH.exists()
    deletions = [call for call in calls if "delete" in call]
    assert deletions == [["nmcli", "connection", "delete", "uuid", state["profile_uuid"]]]
    assert calls[-1] == ["nmcli", "--wait", "30", "connection", "up", "uuid", prior]


def test_existing_or_replaced_profiles_are_not_overwritten_or_removed(fake_pi):
    hotspot.apply("My-Rover", "wlan0")
    with pytest.raises(RuntimeError, match="already exists"):
        hotspot.apply("Another", "wlan0")
    profile = hotspot.PROFILE_PATH.read_text()
    saved = json.loads(hotspot.STATE_PATH.read_text())
    hotspot.PROFILE_PATH.write_text(profile.replace(saved["profile_uuid"], str(uuid.uuid4())))
    with pytest.raises(RuntimeError, match="UUID changed"):
        hotspot.rollback()


def test_failed_activation_retains_rollback_state(fake_pi, monkeypatch):
    original = hotspot._run
    def fail_up(args, check=True):
        if "up" in args:
            raise RuntimeError("simulated AP failure")
        return original(args, check)
    monkeypatch.setattr(hotspot, "_run", fail_up)
    with pytest.raises(RuntimeError, match="simulated AP failure"):
        hotspot.apply("My-Rover", "wlan0")
    assert hotspot.STATE_PATH.exists() and hotspot.PROFILE_PATH.exists()


def test_windows_apply_stops_before_any_network_command(monkeypatch):
    monkeypatch.setattr(hotspot.platform, "system", lambda: "Windows")
    monkeypatch.setattr(hotspot, "_run", lambda *_args, **_kwargs: pytest.fail("Network command attempted"))
    with pytest.raises(RuntimeError, match="not on this Windows laptop"):
        hotspot.apply("My-Rover", "wlan0")
