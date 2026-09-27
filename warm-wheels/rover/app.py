"""Authenticated local dashboard and robot API; bind to loopback by default."""
import argparse
from contextlib import asynccontextmanager
import hmac
import json
import os
from pathlib import Path
import secrets
from typing import Literal

from fastapi import Depends, FastAPI, Header, HTTPException, Query, Request
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field, StrictInt
import uvicorn

from .runtime import RoverRuntime
from .pi_link import add_pi_routes
from .mesh import add_scan_routes
from .room_mapping import add_mapping_routes

ROOT = Path(__file__).resolve().parents[1]


class DriveBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    left: StrictInt = Field(ge=0, le=25)
    right: StrictInt = Field(ge=0, le=25)


class RouteBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    col: StrictInt = Field(ge=0, lt=60)
    row: StrictInt = Field(ge=0, lt=60)


class Detection(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    kind: Literal["person", "fire_candidate", "smoke_candidate"]
    confidence: float = Field(ge=0, le=1)
    box: list[float] = Field(min_length=4, max_length=4)
    source: str = Field(max_length=120)
    posture_hint: str = Field(default="Unassessed", max_length=120)


class DetectionsBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    frame_id: StrictInt = Field(ge=1)
    detections: list[Detection] = Field(max_length=50)


class AssessmentBody(BaseModel):
    model_config = ConfigDict(extra="forbid")
    id: str = Field(max_length=100)
    assessment: Literal["unassessed", "needs_assistance", "can_self_evacuate", "child_reported"]


class PresentationControlBody(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    action: Literal["play", "pause", "reset", "restart", "speed", "boundary", "randomize"]
    speed: float | None = Field(default=None, ge=.25, le=4, strict=True)
    width_m: float | None = Field(default=None, ge=1, le=50, strict=True)
    height_m: float | None = Field(default=None, ge=1, le=50, strict=True)
    seed: StrictInt | None = Field(default=None, ge=0, le=2147483647)


def create_app(runtime=None, token=None, manage_runtime=True):
    runtime = runtime or RoverRuntime()
    token = token or os.environ.get("ROVER_TOKEN")
    if not token or len(token) < 16:
        raise ValueError("Set ROVER_TOKEN to at least 16 characters")

    @asynccontextmanager
    async def lifespan(app):
        if manage_runtime:
            runtime.start()
        try:
            yield
        finally:
            try:
                if hasattr(app.state, "mapping_service"):
                    app.state.mapping_service.close()
                if hasattr(app.state, "pi_bridge"):
                    app.state.pi_bridge.disconnect()
            finally:
                if manage_runtime:
                    runtime.close()

    app = FastAPI(title="W.A.R.M wheels", lifespan=lifespan, docs_url=None, redoc_url=None)
    app.state.runtime = runtime

    @app.middleware("http")
    async def headers(request: Request, call_next):
        # No CORS enabled; UI is same-origin. Token required for camera and controls.
        response = await call_next(request)
        response.headers["Cache-Control"] = "no-store"
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["Referrer-Policy"] = "no-referrer"
        response.headers["Content-Security-Policy"] = "default-src 'self'; img-src 'self' blob:; style-src 'self' 'unsafe-inline'; script-src 'self'; frame-ancestors 'none'"
        return response

    def authorize(authorization: str = Header(default="")):
        if not hmac.compare_digest(authorization, f"Bearer {token}"):
            raise HTTPException(401, "A valid access token is required")

    auth = [Depends(authorize)]

    @app.get("/")
    def dashboard():
        return FileResponse(ROOT/"dashboard"/"index.html")

    @app.get("/presentation")
    def presentation_page():
        return FileResponse(ROOT/"dashboard"/"presentation.html")

    @app.get("/api/health")
    def health():
        return {"product": "warm-wheels", "mode": runtime.mode, "version": "procedural-v2"}

    app.mount("/static", StaticFiles(directory=ROOT/"dashboard"), name="static")
    add_pi_routes(app, auth)
    add_scan_routes(app, runtime, auth)
    add_mapping_routes(app, runtime, auth)

    @app.get("/api/presentation", dependencies=auth)
    def presentation_state():
        try:
            return runtime.presentation_snapshot()
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.post("/api/presentation/control", dependencies=auth)
    def presentation_control(body: PresentationControlBody):
        try:
            return runtime.presentation_command(**body.model_dump(exclude_none=True))
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.get("/api/presentation/report", dependencies=auth)
    def presentation_report():
        try:
            report = runtime.presentation_report()
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc
        name = report["scenario"]["id"]
        return Response(json.dumps(report, indent=2, allow_nan=False), media_type="application/json",
                        headers={"Content-Disposition": f'attachment; filename="warm-wheels-{name}-report.json"'})

    @app.get("/api/state", dependencies=auth)
    def state():
        return runtime.snapshot()

    @app.get("/api/frame.jpg", dependencies=auth)
    def frame(view: Literal["color", "depth"] = Query(default="color")):
        return frame_response(view)

    def frame_response(view):
        try:
            data, frame_id, age = runtime.image(view)
        except ValueError as exc:
            raise HTTPException(503, str(exc)) from exc
        return Response(data, media_type="image/jpeg", headers={"X-Frame-Id": str(frame_id), "X-Frame-Age": str(age)})

    @app.get("/api/inference-frame.jpg", dependencies=auth)
    def inference_frame():
        return frame_response("raw")

    def do_command(action, **kwargs):
        try:
            return runtime.command(action, **kwargs)
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc
        except Exception as exc:
            raise HTTPException(503, f"Motor action failed: {exc}") from exc

    @app.post("/api/arm", dependencies=auth)
    def arm():
        return do_command("arm")

    @app.post("/api/disarm", dependencies=auth)
    def disarm():
        return do_command("disarm")

    @app.post("/api/estop", dependencies=auth)
    def estop():
        return do_command("estop")

    @app.post("/api/reset", dependencies=auth)
    def reset():
        return do_command("reset")

    @app.post("/api/drive", dependencies=auth)
    def drive(body: DriveBody):
        return do_command("drive", left=body.left, right=body.right)

    @app.post("/api/route", dependencies=auth)
    def route(body: RouteBody):
        try:
            return runtime.set_route((body.col, body.row))
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.post("/api/detections", dependencies=auth)
    def detections(body: DetectionsBody):
        try:
            return runtime.submit_detections(body.frame_id, [d.model_dump() for d in body.detections])
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.post("/api/assessment", dependencies=auth)
    def assessment(body: AssessmentBody):
        try:
            return runtime.assess(body.id, body.assessment)
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc

    def do_mission(action):
        try:
            return runtime.mission_command(action)
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc

    @app.post("/api/mission/start", dependencies=auth)
    def start_mission():
        return do_mission("start")

    @app.post("/api/mission/return", dependencies=auth)
    def return_mission():
        return do_mission("return")

    return app


ENABLE_MOTORS_ENV = "WARM_WHEELS_ENABLE_MOTORS"
SERIAL_PORT_ENV = "WARM_WHEELS_SERIAL_PORT"


def parse_args(argv=None, environ=None):
    environ = os.environ if environ is None else environ
    env_enable = environ.get(ENABLE_MOTORS_ENV, "").strip().lower()
    parser = argparse.ArgumentParser(description="W.A.R.M wheels: controlled indoor robot prototype")
    if env_enable not in ("", "0", "false", "no", "off", "1", "true", "yes", "on"):
        parser.error(f"{ENABLE_MOTORS_ENV} must be 1/true/yes/on or 0/false/no/off")
    parser.add_argument("--mode", choices=("demo", "hardware"), default="demo")
    parser.add_argument("--port", default=environ.get(SERIAL_PORT_ENV) or "/dev/ttyACM0",
                        help=f"Arduino USB serial port (default: ${SERIAL_PORT_ENV} or /dev/ttyACM0)")
    parser.add_argument("--enable-motors", action="store_true", default=env_enable in ("1", "true", "yes", "on"),
                        help=f"Opt in to physical motor output in hardware mode after bench calibration "
                             f"(or set {ENABLE_MOTORS_ENV}=1). Arming in the dashboard is still required.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8000)
    parser.add_argument("--camera-height", type=float, default=.22, help="Measured level camera height in metres")
    parser.add_argument("--robot-half-width", type=float, default=.20, help="Measured half-width incl. payload in metres")
    parser.add_argument("--obstacle-height", type=float, default=.45, help="Measured rover/payload height in metres")
    args = parser.parse_args(argv)
    if args.enable_motors and args.mode != "hardware":
        parser.error(f"--enable-motors / {ENABLE_MOTORS_ENV} only applies with --mode hardware")
    return args


def motor_banner(args):
    if args.mode == "demo":
        return "Motors: simulated (demo mode never opens a serial port)."
    if not args.enable_motors:
        return ("Motors: OFF (camera-only). Physical motion needs --enable-motors --port <Uno serial port> "
                f"or {ENABLE_MOTORS_ENV}=1 {SERIAL_PORT_ENV}=<port>.")
    return (f"Motors: ENABLED on {args.port}. Verified firmware, dashboard Arm and a fresh clear depth view "
            "are still required; hold a drive control to move. Emergency stop stays active.")


def main(argv=None):
    args = parse_args(argv)
    token = os.environ.get("ROVER_TOKEN") or secrets.token_urlsafe(24)
    if "ROVER_TOKEN" not in os.environ:
        print(f"Session dashboard token: {token}", flush=True)
    print(f"Mode: {args.mode}. Dashboard: http://{args.host}:{args.http_port}", flush=True)
    print(motor_banner(args), flush=True)
    print("Controlled indoor demonstration only. No autonomous evacuation or whole-house SLAM.", flush=True)
    runtime = RoverRuntime(args.mode, args.port, args.enable_motors,
        camera_height=args.camera_height, robot_half_width=args.robot_half_width,
        obstacle_height=args.obstacle_height)
    uvicorn.run(create_app(runtime, token), host=args.host, port=args.http_port, log_level="warning")


if __name__ == "__main__":
    main()
