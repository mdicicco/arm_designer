"""HTTP server for the arm analyzer GUI.

Stateless: the browser holds the robot URDF text and the trajectory text and
sends both with each analysis request, so an uploaded file never has to be
written to disk. The bundled examples are served read-only from
``examples/robots`` and ``examples/trajectories``.

Local development tool; CORS is restricted to localhost.
"""

from __future__ import annotations

import functools
import re
from pathlib import Path

import numpy as np
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.base import BaseHTTPMiddleware

from arm_analyzer.analysis import analyze
from arm_analyzer.dynamics import GRAVITY, Payload
from arm_analyzer.robot import parse_arm
from arm_analyzer.trajectory import load_trajectory

PROJECT_ROOT = Path(__file__).resolve().parents[2]
WEB_DIR = PROJECT_ROOT / "web"
EXAMPLES = PROJECT_ROOT / "examples"
ROBOTS_DIR = EXAMPLES / "robots"
TRAJ_DIR = EXAMPLES / "trajectories"
PORT = 8766

SAFE_FILE = re.compile(r"^[A-Za-z0-9_\-]+\.[A-Za-z0-9]+$")
TRAJ_SUFFIXES = {".json", ".csv"}

app = FastAPI(title="Arm Analyzer")
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        f"http://{h}:{p}" for h in ("127.0.0.1", "localhost", "[::1]") for p in (PORT, 5173)
    ],
    allow_credentials=False,
    allow_methods=["GET", "POST"],
    allow_headers=["Content-Type"],
)


class _NoStaticCache(BaseHTTPMiddleware):
    """Serve the GUI with no-store so JS/CSS edits show up on reload."""

    async def dispatch(self, request: Request, call_next):
        response = await call_next(request)
        if request.url.path == "/" or request.url.path.startswith("/static/"):
            response.headers["Cache-Control"] = "no-store, max-age=0"
        return response


app.add_middleware(_NoStaticCache)


def translate_exceptions(fn):
    """Map ``ValueError``/``KeyError``/``TypeError`` to 400, others to 500."""

    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except HTTPException:
            raise
        except (ValueError, KeyError, TypeError) as e:
            raise HTTPException(status_code=400, detail=str(e)) from e
        except Exception as e:  # pragma: no cover - defensive
            raise HTTPException(status_code=500, detail=str(e)) from e

    return wrapper


def _example_file(directory: Path, name: str, suffixes: set[str]) -> Path:
    if not SAFE_FILE.match(name or ""):
        raise HTTPException(status_code=400, detail="invalid file name")
    p = (directory / name).resolve()
    if p.parent != directory.resolve() or p.suffix.lower() not in suffixes:
        raise HTTPException(status_code=400, detail="invalid file name")
    if not p.is_file():
        raise HTTPException(status_code=404, detail=f"{name} not found")
    return p


def _listing(directory: Path, suffixes: set[str]) -> list[dict[str, str]]:
    if not directory.is_dir():
        return []
    return [
        {"file": p.name, "label": p.stem.replace("_", " ")}
        for p in sorted(directory.iterdir())
        if p.is_file() and p.suffix.lower() in suffixes
    ]


def _text(body: dict, key: str) -> str:
    v = body.get(key)
    if not isinstance(v, str) or not v.strip():
        raise HTTPException(status_code=400, detail=f"missing {key}")
    return v


# ----------------------------------------------------------------------------
# Routes
# ----------------------------------------------------------------------------


@app.get("/api/robots")
def api_robots() -> list[dict[str, str]]:
    return _listing(ROBOTS_DIR, {".urdf"})


@app.get("/api/robots/{name}")
def api_robot_file(name: str) -> dict[str, str]:
    p = _example_file(ROBOTS_DIR, name, {".urdf"})
    return {"file": p.name, "text": p.read_text(encoding="utf-8")}


@app.get("/api/trajectories")
def api_trajectories() -> list[dict[str, str]]:
    return _listing(TRAJ_DIR, TRAJ_SUFFIXES)


@app.get("/api/trajectories/{name}")
def api_trajectory_file(name: str) -> dict[str, str]:
    p = _example_file(TRAJ_DIR, name, TRAJ_SUFFIXES)
    return {"file": p.name, "text": p.read_text(encoding="utf-8")}


@app.post("/api/robot/model")
@translate_exceptions
def api_robot_model(body: dict) -> dict:
    """Parse URDF text into the pose-independent model the viewport draws."""
    return parse_arm(_text(body, "urdf")).to_dict()


@app.post("/api/trajectory/cartesian/preview")
@translate_exceptions
def api_cartesian_preview(body: dict) -> dict:
    """Path and timing for the trajectory editor (robot-independent).

    Body: ``trajectory`` (cartesian spec, object or text) and, optionally,
    ``urdf`` (text) to also IK-check each waypoint against that robot.
    Unreachable waypoints are reported, not raised; malformed specs are 400s.
    """
    import json

    from arm_analyzer.cartesian import preview

    spec = body.get("trajectory")
    if isinstance(spec, str):
        spec = json.loads(spec)
    if not isinstance(spec, dict) or spec.get("format") != "cartesian":
        raise ValueError("trajectory must be a cartesian spec")
    urdf = body.get("urdf")
    return preview(parse_arm(urdf) if isinstance(urdf, str) and urdf.strip() else None, spec)


@app.post("/api/analyze")
@translate_exceptions
def api_analyze(body: dict) -> dict:
    """Time the trajectory, run inverse dynamics, and summarise every drive.

    Body: ``urdf`` (text), ``trajectory`` (text), optional ``trajectory_kind``
    ("json"/"csv"), ``units`` ("rad"/"deg"), ``smoothing`` ("none"/"auto"),
    ``rate_hz``, ``gravity`` (3-vector), ``payload``:
    ``{"mass", "tool_tip"?: name, "offset"?: [x, y, z]}`` -- the payload's
    centre of mass relative to that tool tip (default: the first one) -- and
    ``efficiency``: ``{joint: {"gearbox"?: eta, "transmission"?: eta}}``
    overriding the file's efficiency placeholders for this analysis only.
    """
    arm = parse_arm(_text(body, "urdf"))
    if not arm.actuated:
        raise ValueError("robot has no actuated joints")
    overrides = body.get("efficiency") or {}
    if not isinstance(overrides, dict):
        raise ValueError("efficiency must be an object keyed by joint name")
    for joint, values in overrides.items():
        if joint not in arm.drives:
            raise ValueError(f"efficiency override for {joint!r}, which has no drive")
        if not isinstance(values, dict) or set(values) - {"gearbox", "transmission"}:
            raise ValueError(f"efficiency[{joint!r}] may only set 'gearbox' and 'transmission'")
        arm.drives[joint].set_efficiency(
            gearbox=None if values.get("gearbox") is None else float(values["gearbox"]),
            transmission=None
            if values.get("transmission") is None
            else float(values["transmission"]),
        )
    plan = load_trajectory(
        arm,
        _text(body, "trajectory"),
        kind=body.get("trajectory_kind"),
        units=body.get("units") or "rad",
        smoothing=body.get("smoothing"),
    )

    g = np.asarray(body.get("gravity", GRAVITY), dtype=float)
    if g.shape != (3,) or not np.all(np.isfinite(g)):
        raise ValueError("gravity must be three finite numbers")

    payload = None
    pb = body.get("payload") or {}
    mass = float(pb.get("mass") or 0.0)
    if mass < 0:
        raise ValueError("payload mass must not be negative")
    if mass > 0:
        tips = [t for l in arm.links.values() for t in l.tool_tips]
        if pb.get("tool_tip"):
            tips = [t for t in tips if t["name"] == pb["tool_tip"]]
        if tips:
            tip = tips[0]
            T_tip = np.asarray(tip["T"], dtype=float)
            link = tip["link"]
        else:
            # No tool tip declared: carry the payload at the last link's origin.
            link = arm.order[-1] if arm.order else arm.root
            T_tip = np.eye(4)
        offset = np.asarray(pb.get("offset") or [0.0, 0.0, 0.0], dtype=float)
        com = T_tip[:3, :3] @ offset + T_tip[:3, 3]
        payload = Payload(link=link, mass=mass, com=com)

    result = analyze(
        arm, plan, rate_hz=float(body.get("rate_hz") or 200.0), gravity=g, payload=payload
    )
    result["warnings"] = arm.warnings
    return result


@app.get("/")
def index() -> FileResponse:
    return FileResponse(WEB_DIR / "index.html")


if WEB_DIR.is_dir():
    app.mount("/static", StaticFiles(directory=str(WEB_DIR)), name="static")


def main() -> None:
    import uvicorn

    uvicorn.run("arm_analyzer.server:app", host="127.0.0.1", port=PORT, reload=False)


if __name__ == "__main__":
    main()
