"""Cartesian trajectories: tool-pose paths, turned into joint motion by IK.

A Cartesian trajectory is robot-independent: it lists tool poses in the world
(robot base) frame and how to move between them. It becomes a joint-space
plan only when combined with a specific arm:

1. **Path** (ndcurves). Each waypoint after the first says how the tool gets
   there: ``"linear"`` -- a straight line (degree-1 Bézier), e.g. approach and
   retract -- or ``"smooth"`` -- a cubic Bézier that leaves along the previous
   linear segment's direction and arrives along the next one's, so transfer
   moves curve into approaches and out of retracts without a corner.
   Orientation follows ``ndcurves.SO3Linear`` (slerp).
2. **Timing** (ndcurves). Every segment runs rest-to-rest on a minimum-jerk
   time law, ``ndcurves.polynomial.MinimumJerk`` from 0 to 1. Its duration is
   given, or the shortest one whose peak tool speed along the curve (and peak
   rotation rate) meets the file's limits. ``dwell`` holds a pose.
3. **IK** (pink on Pinocchio). The path is sampled (default 100 Hz) and each
   pose is solved with ``pink.solve_ik`` under the joint position limits,
   warm-started from the previous sample so the arm stays on one branch. The
   first pose is tried from several seeds around the zero pose. Unreachable
   poses and sudden branch changes are errors naming the time and waypoint.
4. The joint samples go through the same C2 spline fit as any timed-sample
   trajectory (``trajectory.plan_from_samples``).

File format::

    {
      "format": "cartesian",
      "label": "Default pick and place",
      "tool": "tcp",                        # tool_tip name; default: the first one
      "units": "deg",                       # for rpy; default "rad"
      "limits": {"linear_speed": 0.15, "smooth_speed": 0.6, "angular_speed": 180},
      "waypoints": [
        {"name": "home", "xyz": [0.4, 0, 0.55], "rpy": [0, 180, 0]},   # tool pointing down
        {"name": "above_pick", "xyz": [...], "rpy": [...], "path": "smooth"},
        {"name": "pick", "xyz": [...], "rpy": [...], "path": "linear", "dwell": 0.3},
        ...
      ]
    }

Per waypoint, ``duration`` (s) or ``speed`` (m/s) override the limits.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Optional

import ndcurves
import numpy as np
import pink
import pinocchio as pin
from pink.limits import ConfigurationLimit
from pink.tasks import FrameTask, PostureTask

from arm_analyzer.pin_model import build_model, joint_indices
from arm_analyzer.robot import ArmDescription

MIN_JERK_PEAK = 1.875  # peak / average speed of a minimum-jerk move
MIN_SEGMENT_S = 0.1
DEFAULT_LIMITS = {"linear_speed": 0.15, "smooth_speed": 0.5, "angular_speed": 180.0}  # m/s, m/s, deg/s
DEFAULT_IK_RATE = 100.0
POS_TOL = 1e-6  # m
ROT_TOL = 1e-5  # rad
MAX_ITERATIONS = 60
REPORT_POS_TOL = 1e-4  # a pose missed by more than this is "unreachable"
REPORT_ROT_TOL = 1e-3
MAX_JOINT_STEP = 0.35  # rad (or m) between consecutive samples: a branch flip
# Pull toward the previous sample's joints while tracking a path. Near a wrist
# singularity the tool pose barely constrains j4 - j6, and without this the
# solver swings them back and forth to chase micron-level errors, which shows
# up as huge joint accelerations (and torques). The price is a few microns of
# tool error at such poses. The start-pose search keeps the weak value.
TRACKING_POSTURE_COST = 1e-3
SEED_POSTURE_COST = 1e-6
TOOL_FRAME = "__tool__"


# ----------------------------------------------------------------------------
# Path
# ----------------------------------------------------------------------------


@dataclass
class Waypoint:
    name: str
    p: np.ndarray
    R: np.ndarray
    path: str  # how to get here: "linear" | "smooth" (ignored for the first)
    duration: Optional[float] = None
    speed: Optional[float] = None
    dwell: float = 0.0


@dataclass
class Segment:
    start: Waypoint
    end: Waypoint
    t0: float
    duration: float
    position: Any  # ndcurves.bezier over u in [0, 1]
    rotation: Any  # ndcurves.SO3Linear over u in [0, 1]
    timing: Any  # ndcurves.polynomial: s(t) from 0 to 1 over [t0, t0 + duration]

    @property
    def t1(self) -> float:
        return self.t0 + self.duration

    def pose(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        s = float(np.clip(self.timing(min(max(t, self.t0), self.t1))[0], 0.0, 1.0))
        return np.asarray(self.position(s)).ravel(), np.asarray(self.rotation(s))


def _parse_waypoints(spec: dict) -> list[Waypoint]:
    units = (spec.get("units") or "rad").lower()
    if units not in ("rad", "deg"):
        raise ValueError("units must be 'rad' or 'deg'")
    scale = math.pi / 180.0 if units == "deg" else 1.0
    raw = spec.get("waypoints")
    if not isinstance(raw, list) or len(raw) < 2:
        raise ValueError("a cartesian trajectory needs at least two waypoints")
    out = []
    for i, w in enumerate(raw):
        if not isinstance(w, dict):
            raise ValueError(f"waypoints[{i}] must be an object")
        name = str(w.get("name") or f"wp{i}")
        xyz = np.asarray(w.get("xyz"), dtype=float)
        rpy = np.asarray(w.get("rpy", [0.0, 0.0, 0.0]), dtype=float) * scale
        if xyz.shape != (3,) or rpy.shape != (3,) or not (np.all(np.isfinite(xyz)) and np.all(np.isfinite(rpy))):
            raise ValueError(f"waypoint {name!r} needs finite xyz and rpy triples")
        path = w.get("path", "smooth")
        if path not in ("linear", "smooth"):
            raise ValueError(f"waypoint {name!r}: path must be 'linear' or 'smooth'")
        duration = w.get("duration")
        speed = w.get("speed")
        dwell = float(w.get("dwell", 0.0))
        for label, v in (("duration", duration), ("speed", speed)):
            if v is not None and not float(v) > 0:
                raise ValueError(f"waypoint {name!r}: {label} must be positive")
        if dwell < 0:
            raise ValueError(f"waypoint {name!r}: dwell must not be negative")
        out.append(
            Waypoint(
                name=name,
                p=xyz,
                R=pin.rpy.rpyToMatrix(*rpy),
                path=path,
                duration=None if duration is None else float(duration),
                speed=None if speed is None else float(speed),
                dwell=dwell,
            )
        )
    return out


def _unit(v: np.ndarray) -> Optional[np.ndarray]:
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else None


def _position_curve(wps: list[Waypoint], i: int) -> Any:
    """Bézier from waypoint i-1 to i, over u in [0, 1]."""
    a, b = wps[i - 1], wps[i]
    chord = b.p - a.p
    if b.path == "linear" or np.linalg.norm(chord) < 1e-9:
        return ndcurves.bezier(np.column_stack([a.p, b.p]), 0.0, 1.0)
    # Tangents: continue the neighbouring linear segments, else follow the chord.
    t_start = _unit(a.p - wps[i - 2].p) if i >= 2 and a.path == "linear" else None
    t_end = _unit(wps[i + 1].p - b.p) if i + 1 < len(wps) and wps[i + 1].path == "linear" else None
    d = _unit(chord)
    k = 0.4 * float(np.linalg.norm(chord))
    c1 = a.p + k * (t_start if t_start is not None else d)
    c2 = b.p - k * (t_end if t_end is not None else d)
    return ndcurves.bezier(np.column_stack([a.p, c1, c2, b.p]), 0.0, 1.0)


_TAU = np.linspace(0.0, 1.0, 201)
_SIGMA = 10 * _TAU**3 - 15 * _TAU**4 + 6 * _TAU**5  # minimum-jerk progress
_SIGMA_RATE = 30 * _TAU**2 - 60 * _TAU**3 + 30 * _TAU**4  # d(sigma)/d(tau)


def _duration_for_speed(curve: Any, v_max: float) -> float:
    """Shortest duration whose peak tool speed along ``curve`` is ``v_max``.

    Tool speed is ``|dP/du| * dsigma/dtau / T``; on a curve ``|dP/du|`` varies
    along ``u``, so the peak is found over the whole move, not from the length.
    """
    peak = max(
        float(np.linalg.norm(np.asarray(curve.derivate(float(u), 1)).ravel())) * rate
        for u, rate in zip(_SIGMA, _SIGMA_RATE)
    )
    return peak / v_max


def _min_jerk(t0: float, t1: float) -> Any:
    return ndcurves.polynomial.MinimumJerk(np.array([[0.0]]), np.array([[1.0]]), t0, t1)


def build_path(spec: dict) -> tuple[list[Waypoint], list[Segment]]:
    """Waypoints and timed segments; dwells become zero-motion segments."""
    wps = _parse_waypoints(spec)
    limits = {**DEFAULT_LIMITS, **(spec.get("limits") or {})}
    for k in DEFAULT_LIMITS:
        if not float(limits[k]) > 0:
            raise ValueError(f"limits.{k} must be positive")
    omega = math.radians(float(limits["angular_speed"]))

    segments: list[Segment] = []
    t = 0.0

    def hold(w: Waypoint, duration: float) -> None:
        nonlocal t
        seg = Segment(
            w, w, t, duration,
            ndcurves.bezier(np.column_stack([w.p, w.p]), 0.0, 1.0),
            ndcurves.SO3Linear(w.R, w.R, 0.0, 1.0),
            _min_jerk(t, t + duration),
        )
        segments.append(seg)
        t += duration

    if wps[0].dwell > 0:
        hold(wps[0], wps[0].dwell)
    for i in range(1, len(wps)):
        a, b = wps[i - 1], wps[i]
        position = _position_curve(wps, i)
        angle = float(np.linalg.norm(pin.log3(a.R.T @ b.R)))
        if b.duration is not None:
            duration = b.duration
        else:
            v = b.speed or float(limits["linear_speed" if b.path == "linear" else "smooth_speed"])
            duration = max(
                _duration_for_speed(position, v),
                MIN_JERK_PEAK * angle / omega,  # slerp: constant rate along u
                MIN_SEGMENT_S,
            )
        segments.append(
            Segment(a, b, t, duration, position, ndcurves.SO3Linear(a.R, b.R, 0.0, 1.0), _min_jerk(t, t + duration))
        )
        t += duration
        if b.dwell > 0:
            hold(b, b.dwell)
    return wps, segments


def sample_path(segments: list[Segment], rate: float) -> tuple[np.ndarray, list[pin.SE3], list[str]]:
    """Times, tool poses and the waypoint each sample is heading to."""
    total = segments[-1].t1
    n = max(2, int(round(total * rate)) + 1)
    times = np.linspace(0.0, total, n)
    poses, labels = [], []
    k = 0
    for t in times:
        while k + 1 < len(segments) and t > segments[k].t1 + 1e-12:
            k += 1
        seg = segments[k]
        p, R = seg.pose(t)
        poses.append(pin.SE3(R, p))
        labels.append(seg.start.name if t <= seg.t0 + 1e-12 else f"{seg.start.name} → {seg.end.name}")
    return times, poses, labels


# ----------------------------------------------------------------------------
# IK
# ----------------------------------------------------------------------------


def _tool_tip(arm: ArmDescription, name: Optional[str]) -> dict:
    tips = [t for link in [arm.root] + arm.order for t in arm.links[link].tool_tips]
    if name:
        tips = [t for t in tips if t["name"] == name]
        if not tips:
            raise ValueError(f"robot has no tool_tip named {name!r}")
    if not tips:
        raise ValueError("a cartesian trajectory needs a <tool_tip> on the robot")
    return tips[0]


def ik_model(arm: ArmDescription, tool: Optional[str]) -> tuple[pin.Model, str]:
    """Kinematic model with an operational frame at the tool tip."""
    model = build_model(arm, with_drives=False)
    tip = _tool_tip(arm, tool)
    body = model.getFrameId(tip["link"], pin.FrameType.BODY)
    parent = model.frames[body]
    placement = parent.placement * pin.SE3(np.asarray(tip["T"], dtype=float))
    model.addFrame(pin.Frame(TOOL_FRAME, parent.parentJoint, body, placement, pin.FrameType.OP_FRAME))
    return model, tip["name"]


class _Tracker:
    """Sequential pink IK: one converged solve per target, warm-started."""

    def __init__(self, model: pin.Model, q0: np.ndarray, posture_cost: float = SEED_POSTURE_COST):
        self.model = model
        self.config = pink.Configuration(model, model.createData(), q0)
        self.frame = model.getFrameId(TOOL_FRAME)
        self.task = FrameTask(TOOL_FRAME, position_cost=1.0, orientation_cost=1.0, lm_damping=1e-8)
        self.lower = model.lowerPositionLimit.copy()
        self.upper = model.upperPositionLimit.copy()
        # Pull toward the previous solution (see TRACKING_POSTURE_COST).
        self.posture = PostureTask(cost=posture_cost)
        self.limits = [ConfigurationLimit(model)]

    def error(self, target: pin.SE3) -> tuple[float, float]:
        current = self.config.data.oMf[self.frame]
        dp = float(np.linalg.norm(target.translation - current.translation))
        dr = float(np.linalg.norm(pin.log3(current.rotation.T @ target.rotation)))
        return dp, dr

    def solve(self, target: pin.SE3) -> tuple[np.ndarray, float, float]:
        self.task.set_target(target)
        self.posture.set_target(self.config.q.copy())
        dp, dr = self.error(target)
        for _ in range(MAX_ITERATIONS):
            if dp < POS_TOL and dr < ROT_TOL:
                break
            last = (dp, dr)
            v = pink.solve_ik(
                self.config, [self.task, self.posture], 1.0, solver="proxqp", limits=self.limits,
                safety_break=False,
            )
            # Clamp the step's round-off back inside the limits (pink logs a
            # warning for every micro-radian over otherwise).
            q = pin.integrate(self.model, self.config.q, v)
            self.config.update(np.clip(q, self.lower, self.upper))
            dp, dr = self.error(target)
            # With a posture pull the error settles slightly above tolerance;
            # stop once it no longer improves.
            if dp > 0.999 * last[0] and dr > 0.999 * last[1]:
                break
        return self.config.q.copy(), dp, dr


def _seeds(model: pin.Model, arm: ArmDescription, idx_q: np.ndarray) -> list[np.ndarray]:
    """Zero pose (clamped into limits) plus a few elbow / wrist variations."""
    base = np.zeros(model.nq)
    lo, hi = model.lowerPositionLimit, model.upperPositionLimit
    base = np.clip(base, lo, hi)
    seeds = [base]
    n = len(idx_q)
    for d2, d3, d5 in ((0.5, 0.5, 0.8), (0.5, -0.5, 0.8), (-0.5, 0.5, -0.8), (0.3, 1.0, 1.2), (0.8, -1.0, -1.2)):
        q = base.copy()
        for pos, delta in ((1, d2), (2, d3), (4, d5)):
            if pos < n:
                q[idx_q[pos]] += delta
        seeds.append(np.clip(q, lo, hi))
    return seeds


def _reached(dp: float, dr: float) -> bool:
    return dp < REPORT_POS_TOL and dr < REPORT_ROT_TOL


def _start_solution(
    model: pin.Model, arm: ArmDescription, idx_q: np.ndarray, target: pin.SE3
) -> tuple[Optional[np.ndarray], float, float]:
    """Best of several seeds: converged and closest to the zero pose.

    Returns ``(q, dp, dr)``; ``q`` is None when no seed converges, and the
    errors are then the smallest miss found.
    """
    best, miss = None, (math.inf, math.inf)
    for seed in _seeds(model, arm, idx_q):
        q, dp, dr = _Tracker(model, seed).solve(target)
        if _reached(dp, dr):
            cost = float(np.sum(q[idx_q] ** 2))
            if best is None or cost < best[0]:
                best = (cost, q, dp, dr)
        elif dp + dr < sum(miss):
            miss = (dp, dr)
    if best is None:
        return None, *miss
    return best[1], best[2], best[3]


def solve_waypoints(arm: ArmDescription, wps: list[Waypoint], tool: Optional[str]) -> tuple[str, list[dict]]:
    """Quick reachability check of the waypoints alone (for the editor).

    The first waypoint uses the same start rule as ``solve_path``; each later
    one is warm-started from the previous reachable solution, so the reported
    configurations follow the branch the full path would take.
    """
    model, tool_name = ik_model(arm, tool)
    names = arm.actuated
    idx_q, _ = joint_indices(model, names)
    out = []
    prev = None
    for w in wps:
        target = pin.SE3(w.R, w.p)
        q = None
        if prev is not None:
            q, dp, dr = _Tracker(model, prev, TRACKING_POSTURE_COST).solve(target)
            if not _reached(dp, dr):
                q = None
        if q is None:
            q, dp, dr = _start_solution(model, arm, idx_q, target)
        entry = {
            "name": w.name,
            "reachable": q is not None,
            "position_error": dp,
            "orientation_error": dr,
            "q": None if q is None else {n: float(q[i]) for n, i in zip(names, idx_q)},
        }
        out.append(entry)
        if q is not None:
            prev = q
    return tool_name, out


def solve_path(
    arm: ArmDescription, poses: list[pin.SE3], tool: Optional[str], times: np.ndarray, labels: list[str]
) -> tuple[np.ndarray, dict]:
    model, tool_name = ik_model(arm, tool)
    names = arm.actuated
    idx_q, _ = joint_indices(model, names)

    start, dp, _ = _start_solution(model, arm, idx_q, poses[0])
    if start is None:
        raise ValueError(
            f"start pose {labels[0]!r} is not reachable by {arm.name} (tool {tool_name!r}) "
            f"within its joint limits (closest miss {dp * 1000:.1f} mm)"
        )

    tracker = _Tracker(model, start, TRACKING_POSTURE_COST)
    Q = np.empty((len(poses), len(names)))
    max_dp = max_dr = 0.0
    prev = None
    for i, target in enumerate(poses):
        q, dp, dr = tracker.solve(target)
        if dp > REPORT_POS_TOL or dr > REPORT_ROT_TOL:
            raise ValueError(
                f"pose at {labels[i]} (t={times[i]:.2f} s) is not reachable by {arm.name}: "
                f"off by {dp * 1000:.1f} mm / {math.degrees(dr):.2f}°"
            )
        row = q[idx_q]
        if prev is not None:
            jump = np.abs(row - prev)
            k = int(np.argmax(jump))
            if jump[k] > MAX_JOINT_STEP:
                raise ValueError(
                    f"IK jumped {math.degrees(jump[k]):.0f}° on {names[k]} at {labels[i]} "
                    f"(t={times[i]:.2f} s): the path crosses a singularity or joint limit"
                )
        Q[i] = prev = row
        max_dp, max_dr = max(max_dp, dp), max(max_dr, dr)
    info = {
        "tool": tool_name,
        "samples": len(poses),
        "max_position_error": max_dp,
        "max_orientation_error": max_dr,
    }
    return Q, info


def preview(arm: Optional[ArmDescription], spec: dict, rate: float = 50.0) -> dict:
    """Path geometry and timing; with ``arm``, also a per-waypoint IK check.

    The path itself is robot-independent. ``waypoints`` entries only carry
    reachability (and ``tool`` / ``tools`` are only filled in) when an arm is
    given; no full-path solve happens either way.
    """
    wps, segments = build_path(spec)
    times, poses, _ = sample_path(segments, rate)
    if arm is None:
        tool, tools = spec.get("tool"), []
        waypoints = [{"name": w.name, "reachable": None} for w in wps]
    else:
        tool, waypoints = solve_waypoints(arm, wps, spec.get("tool"))
        tools = [t["name"] for link in [arm.root] + arm.order for t in arm.links[link].tool_tips]
    return {
        "tool": tool,
        "tools": tools,
        "duration": segments[-1].t1,
        "path": [p.translation.tolist() for p in poses],
        "segments": [
            {"to": s.end.name, "t0": s.t0, "t1": s.t1, "path": "dwell" if s.start is s.end else s.end.path}
            for s in segments
        ],
        "waypoints": waypoints,
    }


def plan_from_cartesian(arm: ArmDescription, spec: dict, rate: Optional[float] = None):
    """Joint-space ``TrajectoryPlan`` for ``arm`` following a cartesian spec."""
    from arm_analyzer.trajectory import plan_from_samples

    rate = float(rate or spec.get("ik_rate") or DEFAULT_IK_RATE)
    if not (5.0 <= rate <= 1000.0):
        raise ValueError("ik_rate must be between 5 and 1000 Hz")
    wps, segments = build_path(spec)
    times, poses, labels = sample_path(segments, rate)
    Q, info = solve_path(arm, poses, spec.get("tool"), times, labels)
    plan = plan_from_samples(arm.actuated, times, Q, arm.actuated)
    plan.waypoint_times = sorted({round(s.t1, 9) for s in segments} | {0.0})
    plan.meta = {
        "kind": "cartesian",
        "ik": {**info, "rate": rate},
        "waypoints": [
            {"name": w.name, "xyz": w.p.tolist(), "path": w.path, "dwell": w.dwell} for w in wps
        ],
        "segments": [
            {"to": s.end.name, "t0": s.t0, "t1": s.t1, "path": "dwell" if s.start is s.end else s.end.path}
            for s in segments
        ],
    }
    return plan
