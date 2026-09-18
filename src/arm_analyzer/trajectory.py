"""Trajectory import: waypoint files and timed-sample files.

Both formats end up as the same ``profile.TrajectoryPlan`` -- piecewise cubic
polynomials per joint -- which is what the browser evaluates for playback and
what the dynamics pass samples. Cubics give continuous acceleration, so the
torque trace is continuous too.

Waypoints (JSON), timed by the jerk-limited planner in ``profile.py``::

    {
      "format": "waypoints",
      "units": "deg",                       # optional, default "rad"
      "joint_order": ["j1", "j2", ...],     # needed only for list-form points
      "limits": {"accel_ratio": 4, "jerk_ratio": 20, "time_scale": 1.0,
                 "per_joint": {"j1": {"max_velocity": 1.0}}},
      "waypoints": [
        {"joints": {"j1": 0, "j2": -30}, "stop": true},
        {"joints": [10, -20, 45, 0, 30, 0], "stop": false}
      ]
    }

Cartesian tool paths (``"format": "cartesian"``) are robot-independent and
turned into joint motion by IK; see ``cartesian.py``.

Timed samples, fitted with a cubic spline through every sample:

* JSON: ``{"format": "samples", "units": "rad", "joint_names": [...],
  "t": [...], "q": [[...], ...]}`` (one row of ``q`` per time).
* CSV: a header row ``t,<joint>,<joint>,...`` then one row per sample.
  Lines starting with ``#`` are ignored. Units come from the request.

``smoothing="auto"`` replaces the interpolating spline with a smoothing spline
whose strength is chosen by generalized cross-validation -- use it for
measured data, where differentiating noise twice ruins the torque trace.
Joints the trajectory does not mention are held at zero. Angles are converted
from degrees only for revolute joints; prismatic positions are always metres.
"""

from __future__ import annotations

import csv
import io
import json
import math
from typing import Any, Optional, Sequence

import numpy as np

from arm_analyzer.profile import (
    PolySegment,
    TrajectoryPlan,
    Waypoint,
    limits_from_velocities,
    plan_trajectory,
)
from arm_analyzer.robot import ArmDescription

MAX_SAMPLES = 20_000


def _unit_scale(units: str) -> float:
    u = (units or "rad").lower()
    if u in ("rad", "radian", "radians"):
        return 1.0
    if u in ("deg", "degree", "degrees"):
        return math.pi / 180.0
    raise ValueError(f"units must be 'rad' or 'deg', got {units!r}")


def _converter(arm: ArmDescription, units: str):
    scale = _unit_scale(units)

    def convert(name: str, value: float) -> float:
        if name not in arm.joints or arm.joints[name].joint_type not in ("revolute", "prismatic"):
            raise ValueError(f"trajectory names {name!r}, which is not an actuated joint")
        v = float(value)
        if not math.isfinite(v):
            raise ValueError(f"trajectory value for {name!r} is not finite")
        return v * scale if arm.joints[name].joint_type == "revolute" else v

    return convert


# ----------------------------------------------------------------------------
# Waypoints
# ----------------------------------------------------------------------------


def plan_from_waypoints(arm: ArmDescription, spec: dict[str, Any]) -> TrajectoryPlan:
    convert = _converter(arm, spec.get("units", "rad"))
    order = spec.get("joint_order") or arm.actuated
    raw = spec.get("waypoints")
    if not isinstance(raw, list) or not raw:
        raise ValueError("waypoints must be a non-empty list")

    waypoints: list[Waypoint] = []
    for i, w in enumerate(raw):
        if not isinstance(w, dict):
            raise ValueError(f"waypoints[{i}] must be an object")
        joints = w.get("joints")
        if isinstance(joints, list):
            if len(joints) != len(order):
                raise ValueError(
                    f"waypoints[{i}] has {len(joints)} values for {len(order)} joints"
                )
            joints = dict(zip(order, joints))
        if not isinstance(joints, dict):
            raise ValueError(f"waypoints[{i}].joints must be an object or a list")
        waypoints.append(
            Waypoint(
                joints={str(k): convert(str(k), v) for k, v in joints.items()},
                stop=bool(w.get("stop", True)),
                name=str(w.get("name") or w.get("id") or ""),
            )
        )

    lim = spec.get("limits") or {}
    accel_ratio = float(lim.get("accel_ratio", 4.0))
    jerk_ratio = float(lim.get("jerk_ratio", 20.0))
    time_scale = float(lim.get("time_scale", 1.0))
    if min(accel_ratio, jerk_ratio, time_scale) <= 0:
        raise ValueError("limit ratios and time_scale must be positive")
    velocities = {n: arm.velocity_limit(n) for n in arm.actuated}
    limits = limits_from_velocities(velocities, accel_ratio, jerk_ratio, lim.get("per_joint"))
    return plan_trajectory(waypoints, limits, time_scale=time_scale)


# ----------------------------------------------------------------------------
# Timed samples
# ----------------------------------------------------------------------------


def parse_csv_samples(text: str) -> tuple[list[str], np.ndarray, np.ndarray]:
    """``(joint_names, t, q)`` from ``t,<joint>,...`` CSV text."""
    rows = [
        r
        for r in csv.reader(io.StringIO(text))
        if r and any(c.strip() for c in r) and not r[0].lstrip().startswith("#")
    ]
    if len(rows) < 3:
        raise ValueError("CSV needs a header row and at least two samples")
    header = [h.strip() for h in rows[0]]
    if header[0].lower() not in ("t", "time", "time_s", "t_s"):
        raise ValueError("first CSV column must be the time column, named 't'")
    names = header[1:]
    if not names:
        raise ValueError("CSV has no joint columns")
    try:
        data = np.array([[float(c) for c in r] for r in rows[1:]], dtype=float)
    except ValueError as e:
        raise ValueError(f"CSV contains a non-numeric value: {e}") from e
    if data.shape[1] != len(header):
        raise ValueError("every CSV row must have one value per header column")
    return names, data[:, 0], data[:, 1:]


def plan_from_samples(
    names: Sequence[str],
    t: np.ndarray,
    q: np.ndarray,
    all_joints: Sequence[str],
    smoothing: Optional[str] = None,
) -> TrajectoryPlan:
    """Fit a C2 cubic through timed samples and emit it as ``PolySegment``s."""
    from scipy.interpolate import CubicSpline, make_smoothing_spline

    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float)
    if t.ndim != 1 or len(t) < 2:
        raise ValueError("need at least two samples")
    if len(t) > MAX_SAMPLES:
        raise ValueError(f"at most {MAX_SAMPLES} samples are supported, got {len(t)}")
    if q.shape != (len(t), len(names)):
        raise ValueError("sample matrix must have one row per time and one column per joint")
    if not np.all(np.isfinite(t)) or not np.all(np.isfinite(q)):
        raise ValueError("samples must be finite")
    if np.any(np.diff(t) <= 0):
        raise ValueError("sample times must be strictly increasing")
    if smoothing not in (None, "", "none", "auto"):
        raise ValueError("smoothing must be 'none' or 'auto'")
    if smoothing == "auto" and len(t) < 5:
        raise ValueError("smoothing needs at least five samples")

    t0 = float(t[0])
    tt = t - t0
    fits = {}
    for k, n in enumerate(names):
        if smoothing == "auto":
            fits[n] = make_smoothing_spline(tt, q[:, k])
        elif len(tt) < 4:
            # not-a-knot needs four points; with fewer, a natural spline is exact.
            fits[n] = CubicSpline(tt, q[:, k], bc_type="natural")
        else:
            fits[n] = CubicSpline(tt, q[:, k])

    # Evaluate each fit's Taylor coefficients at the left end of every sample
    # interval. Both spline kinds are cubic between samples and C2 across
    # them, so this is exact, and c3 is read at the midpoint to stay inside
    # the interval's own polynomial.
    left, right = tt[:-1], tt[1:]
    mid = 0.5 * (left + right)
    coeffs: dict[str, np.ndarray] = {}
    for n, f in fits.items():
        coeffs[n] = np.stack(
            [f(left), f(left, 1), f(left, 2) / 2.0, f(mid, 3) / 6.0], axis=1
        )
    for n in all_joints:
        if n not in coeffs:
            coeffs[n] = np.zeros((len(left), 4))

    segments = [
        PolySegment(
            float(left[i]),
            float(right[i]),
            {n: tuple(float(x) for x in c[i]) for n, c in coeffs.items()},
        )
        for i in range(len(left))
    ]
    return TrajectoryPlan(
        segments=segments,
        joints=list(all_joints),
        waypoint_times=[],
    )


def load_trajectory(
    arm: ArmDescription,
    content: Any,
    *,
    kind: Optional[str] = None,
    units: str = "rad",
    smoothing: Optional[str] = None,
) -> TrajectoryPlan:
    """Dispatch on ``kind`` ("json" / "csv") or sniff it from ``content``.

    ``content`` is the file text, or an already-decoded JSON object.
    """
    if isinstance(content, str) and (kind == "json" or (kind is None and content.lstrip()[:1] in "{[")):
        try:
            content = json.loads(content)
        except json.JSONDecodeError as e:
            raise ValueError(f"invalid JSON: {e}") from e

    if isinstance(content, dict):
        fmt = content.get("format") or ("waypoints" if "waypoints" in content else "samples")
        if fmt == "waypoints":
            return plan_from_waypoints(arm, content)
        if fmt == "cartesian":
            from arm_analyzer.cartesian import plan_from_cartesian

            return plan_from_cartesian(arm, content)
        if fmt != "samples":
            raise ValueError(f"unknown trajectory format {fmt!r}")
        names = content.get("joint_names") or arm.actuated
        convert = _converter(arm, content.get("units", units))
        q = np.asarray(content.get("q"), dtype=float)
        if q.ndim != 2 or q.shape[1] != len(names):
            raise ValueError("samples 'q' must be a list of rows, one value per joint")
        q = np.array([[convert(n, v) for n, v in zip(names, row)] for row in q])
        return plan_from_samples(
            names, content.get("t"), q, arm.actuated, content.get("smoothing", smoothing)
        )

    if isinstance(content, str):
        names, t, q = parse_csv_samples(content)
        convert = _converter(arm, units)
        q = np.array([[convert(n, v) for n, v in zip(names, row)] for row in q])
        return plan_from_samples(names, t, q, arm.actuated, smoothing)

    raise ValueError("trajectory must be JSON (object) or CSV text")
