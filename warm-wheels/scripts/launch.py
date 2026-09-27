"""Folder-independent Windows launch with session reuse and browser sign-in."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import secrets
import socket
import subprocess
import sys
import time
from urllib.parse import quote
import urllib.request
import webbrowser

ROOT = Path(__file__).resolve().parents[1]
STATE_DIR = ROOT / ".runtime"
ENABLE_MOTORS_ENV = "WARM_WHEELS_ENABLE_MOTORS"
SERIAL_PORT_ENV = "WARM_WHEELS_SERIAL_PORT"


class _RejectRedirects(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, request, response, code, message, headers, new_url):
        # Health/state/disarm are fixed local endpoints. Never forward the
        # session Authorization header to another URL or a replacement service.
        return None


def request_json(base, path, token=None, body=None):
    headers = {"Authorization": f"Bearer {token}"} if token else {}
    if body is not None:
        headers["Content-Type"] = "application/json"
    request = urllib.request.Request(base + path, data=None if body is None else json.dumps(body).encode(), headers=headers)
    # Local service only; do not send session tokens to an environment proxy.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _RejectRedirects())
    with opener.open(request, timeout=1) as response:
        return json.load(response)


def available_port(start):
    for port in range(start, min(start + 30, 65536)):
        with socket.socket() as probe:
            try:
                probe.bind(("127.0.0.1", port))
                return port
            except OSError:
                continue
    raise RuntimeError("No free local dashboard port found.")


def existing_session(mode):
    try:
        record = json.loads((STATE_DIR / f"{mode}.json").read_text(encoding="utf-8"))
        base = f"http://127.0.0.1:{int(record['port'])}"
        health = request_json(base, "/api/health")
        if health.get("product") != "warm-wheels" or health.get("mode") != mode or health.get("version") != "procedural-v2":
            return None
        state = request_json(base, "/api/state", record["token"])
        if state.get("mode") != mode:
            return None
        return record
    except (OSError, ValueError, KeyError, TypeError):
        return None


def landing_path(record, mode):
    if mode == "demo":
        return "/presentation"
    # The /scan viewer has no arm/drive controls; the operator console does.
    return "/" if record.get("motors") else "/scan"


def launch_url(record, mode):
    return f"http://127.0.0.1:{record['port']}" + landing_path(record, mode) + "#token=" + quote(record["token"], safe="")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("demo", "hardware"), default="demo")
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--http-port", type=int, default=None)
    parser.add_argument("--enable-motors", action="store_true")
    parser.add_argument("--port", help="Explicit Uno COM port; only with --enable-motors")
    args = parser.parse_args(argv)
    os.chdir(ROOT)
    if os.environ.get(ENABLE_MOTORS_ENV, "").strip().lower() in ("1", "true", "yes", "on"):
        args.enable_motors = True
    args.port = args.port or os.environ.get(SERIAL_PORT_ENV) or None
    if args.enable_motors and (args.mode != "hardware" or not args.port):
        parser.error("Physical motors require --mode hardware and an explicit --port COMx after bench verification.")
    existing = existing_session(args.mode)
    if existing:
        if args.enable_motors:
            parser.error("A hardware session is already running. Stop it before changing motor permissions.")
        print(f"W.A.R.M wheels is already running at http://127.0.0.1:{existing['port']}", flush=True)
        if not args.no_browser:
            webbrowser.open(launch_url(existing, args.mode))
        return 0
    if args.mode == "hardware":
        from scripts.hardware_preflight import enumerate_realsense
        camera = enumerate_realsense()
        if camera["status"] != "connected":
            print("RealSense not ready. Plug the D435i into this laptop's USB 3 port and run hardware_preflight.cmd. For a camera connected to the Pi, use START_PRESENTATION.cmd > Connect Raspberry Pi instead.", flush=True)
            return 2
    port = available_port(args.http_port or (8010 if args.mode == "demo" else 8020))
    token = secrets.token_urlsafe(24)
    env = os.environ.copy()
    env["ROVER_TOKEN"] = token
    # Motor permission reaches the service only through the flags validated above.
    env.pop(ENABLE_MOTORS_ENV, None)
    env.pop(SERIAL_PORT_ENV, None)
    command = [sys.executable, "-m", "rover.app", "--mode", args.mode, "--http-port", str(port)]
    if args.enable_motors:
        command += ["--enable-motors", "--port", args.port]
    child = subprocess.Popen(command, cwd=ROOT, env=env)
    base = f"http://127.0.0.1:{port}"
    record = {"port": port, "token": token, "pid": child.pid, "mode": args.mode, "motors": bool(args.enable_motors)}
    try:
        for _ in range(100):
            if child.poll() is not None:
                print("Service could not start. Read the error above, then run hardware_preflight.cmd for hardware checks.", flush=True)
                return child.returncode or 1
            try:
                state = request_json(base, "/api/state", token)
                if state.get("mode") == args.mode:
                    break
            except (OSError, ValueError):
                time.sleep(.15)
        else:
            raise RuntimeError("Dashboard did not become ready in time.")
        STATE_DIR.mkdir(exist_ok=True)
        (STATE_DIR / f"{args.mode}.json").write_text(json.dumps(record), encoding="utf-8")
        print(f"\nW.A.R.M wheels READY: {base}" + landing_path(record, args.mode), flush=True)
        if args.mode == "hardware":
            print(f"Motors: ENABLED on {args.port}; arm in the dashboard, then hold a drive control." if args.enable_motors
                  else "Motors: OFF (camera-only). Relaunch with --enable-motors --port COMx for physical motion.", flush=True)
        print("The launcher signs the browser in automatically. Keep this window open; Ctrl+C stops the service.", flush=True)
        print(f"Manual sign-in token: {token}", flush=True)
        if not args.no_browser:
            webbrowser.open(launch_url(record, args.mode))
        return child.wait()
    except KeyboardInterrupt:
        print("\nStopping W.A.R.M wheels...", flush=True)
        return 0
    finally:
        for endpoint in ("/api/disarm", "/api/pi/disconnect"):
            try:
                request_json(base, endpoint, token, {})
            except (OSError, ValueError):
                pass
        if child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()


if __name__ == "__main__":
    raise SystemExit(main())
