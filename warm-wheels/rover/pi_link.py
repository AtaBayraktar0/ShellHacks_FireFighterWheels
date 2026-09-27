"""Authenticated laptop gateway to one explicitly selected Pi on a local network.

The Pi retains all drive gates. This gateway never refreshes drive commands on its
own. Credentials are held only in memory and requests use fixed API paths.
"""
from __future__ import annotations

import ipaddress
from contextlib import nullcontext
from pathlib import Path
import socket
import threading
from urllib.parse import urlsplit

import httpx
from fastapi import HTTPException, Request
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel, ConfigDict, Field

ROOT = Path(__file__).resolve().parents[1]
READ_PATHS = {"state", "frame.jpg", "inference-frame.jpg", "mesh", "room-map"}
WRITE_PATHS = {"arm", "disarm", "estop", "reset", "drive", "route", "detections", "assessment", "room-map-control"}
PRIVATE_NETS = [ipaddress.ip_network(n) for n in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16", "127.0.0.0/8")]


def local_target(value: str, resolver=socket.getaddrinfo):
    value = value.strip()
    if "://" not in value:
        value = "http://" + value
    parsed = urlsplit(value)
    if parsed.scheme != "http" or parsed.username or parsed.password or parsed.query or parsed.fragment or parsed.path not in ("", "/"):
        raise ValueError("Use a local HTTP address such as http://192.168.1.42:8000, without a path or password.")
    host = parsed.hostname or ""
    port = parsed.port or 8000
    if not 1 <= port <= 65535:
        raise ValueError("Port must be between 1 and 65535.")
    try:
        address = ipaddress.IPv4Address(host)
    except ValueError:
        if host != "localhost" and not host.lower().endswith(".local"):
            raise ValueError("Enter the Pi's IPv4 address or its .local hostname.") from None
        try:
            addresses = resolver(host, port, socket.AF_INET, socket.SOCK_STREAM)
            address = ipaddress.IPv4Address(addresses[0][4][0])
        except (OSError, IndexError, ValueError):
            raise ValueError("Could not resolve that hostname. Use the Pi's address from hostname -I.") from None
    if not any(address in network for network in PRIVATE_NETS):
        raise ValueError("Only private LAN or localhost addresses are accepted. Do not expose the rover to the Internet.")
    return f"http://{address}:{port}"


class PiBridge:
    def __init__(self, client_factory=httpx.Client):
        self._factory = client_factory
        self._lock = threading.RLock()
        self._command_lock = threading.RLock()
        self._client = None
        self._address = None
        self._mode = None
        self._error = None
        self._generation = 0
        self._connect_attempt = 0

    def status(self):
        with self._lock:
            return {"connected": self._client is not None, "address": self._address,
                    "mode": self._mode, "error": self._error, "generation": str(self._generation),
                    "physical_boundary_enforced": False,
                    "boundary_note": "Real-world metre limits need validated localization; this connection provides live viewing and supervised drive only."}

    def connect(self, address, token):
        target = local_target(address)
        if not 16 <= len(token) <= 512:
            raise ValueError("Use the token printed by the Pi (at least 16 characters).")
        with self._lock:
            self._connect_attempt += 1
            attempt = self._connect_attempt
        client = self._factory(base_url=target, headers={"Authorization": f"Bearer {token}"},
                               timeout=httpx.Timeout(2.0, connect=2.0), follow_redirects=False, trust_env=False)
        try:
            response = client.get("/api/state")
            if response.status_code == 401:
                raise ValueError("The Pi rejected this token. Copy ROVER_TOKEN from the Pi terminal.")
            if response.status_code != 200 or len(response.content) > 2_000_000:
                raise ValueError("The address did not return a W.A.R.M wheels state. Start the Pi service and check its port.")
            state = response.json()
            if not isinstance(state, dict) or state.get("mode") not in {"demo", "hardware"} or "motor" not in state:
                raise ValueError("This is not a compatible W.A.R.M wheels stream.")
        except (httpx.HTTPError, ValueError) as exc:
            client.close()
            if isinstance(exc, httpx.HTTPError):
                raise ValueError("Pi unreachable. Join the same Wi-Fi, check its IP, and start it with --host 0.0.0.0.") from None
            raise
        with self._command_lock, self._lock:
            if attempt != self._connect_attempt:
                client.close()
                raise ValueError("Connection attempt canceled by a newer connection or disconnect.")
            self._disconnect_locked()
            self._client, self._address, self._mode = client, target, state["mode"]
            self._error = None
            self._generation += 1
        return self.status()

    def _disconnect_locked(self):
        previous, self._client = self._client, None
        self._address = self._mode = None
        self._generation += 1
        if previous is not None:
            try:
                previous.post("/api/disarm", json={})
            except httpx.HTTPError:
                pass
            finally:
                previous.close()

    def disconnect(self):
        with self._command_lock, self._lock:
            self._connect_attempt += 1
            self._disconnect_locked()
            self._error = None
        return self.status()

    def forward(self, method, endpoint, *, query=None, body=None, expected_generation=None):
        allowed = READ_PATHS if method == "GET" else WRITE_PATHS if method == "POST" else set()
        if endpoint not in allowed:
            raise ValueError("That API path is not part of the remote control interface.")
        with self._lock:
            client, generation = self._client, self._generation
        if client is None:
            raise ValueError("Connect the Raspberry Pi first.")
        try:
            with self._command_lock if method == "POST" else nullcontext():
                with self._lock:
                    if generation != self._generation:
                        raise ValueError("Pi connection changed before the command was sent.")
                    if endpoint == "detections" and method == "POST" and expected_generation != str(generation):
                        raise ValueError("Pi source changed or inference generation is missing; discard these detections.")
                response = client.request(method, "/api/" + endpoint, params=query, json=body if method == "POST" else None)
        except (httpx.HTTPError, RuntimeError):
            with self._lock:
                if generation == self._generation:
                    self._error = "Pi stream unavailable; stop sending drive commands and check Wi-Fi."
            raise ValueError("Pi connection lost. Drive refreshes are stopped; the Pi/Uno timeouts remain responsible for physical stopping.") from None
        with self._lock:
            if generation != self._generation:
                raise ValueError("Pi connection changed; discard this response.")
            self._error = None if response.status_code < 400 else "Pi rejected the request."
            response.headers["X-Pi-Generation"] = str(generation)
        if len(response.content) > 5_000_000:
            raise ValueError("Pi response exceeded the size limit.")
        return response


class PiConnection(BaseModel):
    model_config = ConfigDict(extra="forbid")
    address: str = Field(min_length=1, max_length=200)
    token: str = Field(min_length=16, max_length=512)


def add_pi_routes(app, auth):
    bridge = PiBridge()
    app.state.pi_bridge = bridge

    @app.get("/connect")
    def connect_page():
        return FileResponse(ROOT / "dashboard" / "connect.html")

    @app.get("/pi")
    def pi_dashboard():
        return FileResponse(ROOT / "dashboard" / "index.html")

    @app.get("/api/pi/link", dependencies=auth)
    def link_status():
        return bridge.status()

    @app.post("/api/pi/connect", dependencies=auth)
    def connect_pi(body: PiConnection):
        try:
            return bridge.connect(body.address, body.token)
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.post("/api/pi/disconnect", dependencies=auth)
    def disconnect_pi():
        return bridge.disconnect()

    @app.api_route("/api/pi/proxy/api/{endpoint}", methods=["GET", "POST"], dependencies=auth)
    async def forward_pi(endpoint: str, request: Request):
        import anyio
        raw = await request.body()
        if len(raw) > 64_000:
            raise HTTPException(413, "Request too large")
        try:
            body = await request.json() if raw else None
            query = dict(request.query_params)
            if set(query) - {"view"} or ("view" in query and query["view"] not in {"color", "depth"}):
                raise ValueError("Unsupported image query.")
            response = await anyio.to_thread.run_sync(lambda: bridge.forward(request.method, endpoint, query=query, body=body,
                expected_generation=request.headers.get("X-Pi-Generation")))
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc
        if 300 <= response.status_code < 400:
            raise HTTPException(502, "Remote redirects are not followed")
        headers = {name: response.headers[name] for name in ("X-Frame-Id", "X-Frame-Age", "X-Pi-Generation") if name in response.headers}
        media = "image/jpeg" if endpoint.endswith(".jpg") and response.status_code == 200 else "application/json"
        return Response(response.content, status_code=response.status_code, media_type=media, headers=headers)
