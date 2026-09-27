"""Beginner-friendly AI source selection; no token in command lines or reports."""
from __future__ import annotations

import argparse
import getpass
import os
import subprocess

from .launch import ROOT, existing_session
from pc.detector import normalize_base_url


def select_source(source, rover=None, *, allow_demo=False, session_loader=existing_session,
                  prompt=input, secret_prompt=getpass.getpass):
    if source is None:
        print("W.A.R.M wheels AI - choose the camera source:")
        print("  1. Joined Raspberry Pi through the laptop dashboard (Wi-Fi)")
        print("  2. RealSense connected directly to this laptop (Local USB)")
        print("  3. Direct Raspberry Pi address")
        source = {"1": "joined", "2": "local", "3": "direct"}.get(prompt("Choose 1, 2, or 3 [1]: ").strip() or "1")
        if source is None:
            raise ValueError("Choose one of the three listed sources.")
    if source in {"local", "joined", "demo"}:
        if source == "demo" and not allow_demo:
            raise ValueError("Demo transport testing requires --allow-demo.")
        session = session_loader("hardware" if source == "local" else "demo")
        if not session:
            instruction = "START_HARDWARE.cmd" if source == "local" else "START_PRESENTATION.cmd"
            raise ValueError(f"Start {instruction} first" + (", then join the Raspberry Pi in Connect Raspberry Pi." if source == "joined" else "."))
        base = f"http://127.0.0.1:{int(session['port'])}"
        if source == "joined":
            base += "/api/pi/proxy"
        return {"source": source, "base_url": normalize_base_url(base), "token": session["token"]}
    if source == "direct":
        value = rover or prompt("Pi address, e.g. http://192.168.1.50:8000: ").strip()
        if "://" not in value:
            value = "http://" + value + ("" if ":" in value else ":8000")
        base = normalize_base_url(value)
        token = secret_prompt("Pi session token (hidden; not saved): ").strip()
        if len(token) < 16:
            raise ValueError("Pi token must contain at least 16 characters.")
        return {"source": source, "base_url": base, "token": token}
    raise ValueError("Unknown camera source.")


def worker_command(selection, args, python_path):
    command = [str(python_path), "-m", "pc.detector", "--rover", selection["base_url"],
               "--device", args.device, "--person-confidence", str(args.person_confidence),
               "--fire-confidence", str(args.fire_confidence)]
    if args.allow_demo:
        command.append("--allow-demo")
    if args.max_frames is not None:
        command.extend(["--max-frames", str(args.max_frames)])
    if args.max_seconds is not None:
        command.extend(["--max-seconds", str(args.max_seconds)])
    return command


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("joined", "local", "direct", "demo"))
    parser.add_argument("--rover", help="Direct Pi base URL; token is requested hidden in the console")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--person-confidence", type=float, default=.45)
    parser.add_argument("--fire-confidence", type=float, default=.35)
    parser.add_argument("--allow-demo", action="store_true")
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--max-seconds", type=float)
    args = parser.parse_args(argv)
    try:
        os.chdir(ROOT)
        python_path = ROOT / ".venv-ai/Scripts/python.exe"
        if not python_path.is_file():
            raise ValueError("AI environment is missing; follow docs/model-provenance.md to recreate .venv-ai.")
        selection = select_source(args.source, args.rover, allow_demo=args.allow_demo)
        env = os.environ.copy()
        env["ROVER_TOKEN"] = selection["token"]
        config = ROOT / "work/ultralytics"
        config.mkdir(parents=True, exist_ok=True)
        env["YOLO_CONFIG_DIR"] = str(config)
        env["PYTHONUTF8"] = "1"
        command = worker_command(selection, args, python_path)
        print(f"Selected source: {selection['source']} | {selection['base_url']}", flush=True)
        print("Worker command (token is supplied privately through the environment):", flush=True)
        print(subprocess.list2cmdline(command), flush=True)
        print("Keep this window open. Ctrl+C stops AI without stopping the dashboard.", flush=True)
        return subprocess.run(command, cwd=ROOT, env=env, check=False).returncode
    except KeyboardInterrupt:
        return 0
    except (ValueError, OSError) as exc:
        print(f"AI launcher: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
