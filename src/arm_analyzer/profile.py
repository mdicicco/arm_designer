"""Trajectory profile generation under velocity, acceleration and jerk limits.

Why this replaces the old trapezoidal profile
---------------------------------------------
The previous playback profile was piecewise-*linear* in velocity: ramp up,
hold, ramp down. That makes acceleration a step function with four
discontinuities per segment and jerk a train of impulses. Since joint torque
is linear in ``qdd``, a torque plot computed from that profile is dominated by
four vertical jumps per segment — artifacts of the profile, not properties of
the robot. It also only ever exposed *velocity* limits, despite living in a
tab about acceleration and deceleration.

(Copied from modular_robot.profile; ``limits_from_velocities`` and the
bisect segment lookup differ.)

What this module produces
-------------------------
Both modes emit the same thing: a **piecewise cubic polynomial per joint**.
Position is cubic, velocity quadratic, acceleration linear, jerk piecewise
constant and bounded. Acceleration is continuous everywhere, so the torque
trace is continuous too.

* ``stop`` waypoints get a proper 7-phase double-S (jerk-limited trapezoidal
  acceleration) segment that starts and ends at exactly zero velocity.
* runs of ``pass-through`` waypoints are fitted with a clamped cubic spline
  and then uniformly time-scaled until every limit is satisfied. Time scaling
  is exact for polynomials: stretching time by ``k`` divides velocity by
  ``k``, acceleration by ``k**2`` and jerk by ``k**3``.

Joint synchronization
---------------------
All joints in a segment share one scalar progress ``s(t): [0,1]``, so they
start and arrive together. The per-joint limits collapse onto that scalar:

    S_v = min_j (v_max_j / |dq_j|)     S_a = min_j (a_max_j / |dq_j|)
    S_j = min_j (j_max_j / |dq_j|)

which turns an N-joint synchronization problem into one 1-D double-S solve,
and tells you exactly which joint and which limit is binding.

The polynomial form is also what crosses the wire: the browser evaluates the
same coefficients for playback, so planning has a single implementation.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import Any, Iterable, Iterator, Optional, Sequence

import numpy as np

# Defaults used when a URDF declares no acceleration/jerk information. Both are
# expressed as multiples of the joint's velocity limit, which keeps them
# dimensionally sane for revolute (rad) and prismatic (m) joints alike:
# reaching top speed in 1/DEFAULT_ACCEL_RATIO seconds, and reaching top
# acceleration in 1/DEFAULT_JERK_RATIO seconds.
DEFAULT_ACCEL_RATIO = 4.0
DEFAULT_JERK_RATIO = 20.0

_EPS = 1e-12


@dataclass(frozen=True)
class JointLimits:
    """Per-joint kinematic limits used by the planner."""

    name: str
    max_velocity: float
    max_acceleration: float
    max_jerk: float

    @staticmethod
    def from_velocity(
        name: str,
        max_velocity: float,
        accel_ratio: float = DEFAULT_ACCEL_RATIO,
        jerk_ratio: float = DEFAULT_JERK_RATIO,
    ) -> "JointLimits":
        v = max(_EPS, float(max_velocity))
        return JointLimits(name, v, v * accel_ratio, v * accel_ratio * jerk_ratio)


@dataclass
class Waypoint:
    """One keyframe. ``stop=True`` forces zero velocity at this point."""

    joints: dict[str, float]
    stop: bool = True
    name: str = ""


@dataclass
class PolySegment:
    """``q(t) = c0 + c1*dt + c2*dt^2 + c3*dt^3`` with ``dt = t - t0``."""

    t0: float
    t1: float
    coeffs: dict[str, tuple[float, float, float, float]]

    @property
    def duration(self) -> float:
        return self.t1 - self.t0

    def evaluate(self, t: float) -> tuple[dict[str, float], dict[str, float], dict[str, float]]:
        dt = min(max(float(t) - self.t0, 0.0), max(self.duration, 0.0))
        q: dict[str, float] = {}
        qd: dict[str, float] = {}
        qdd: dict[str, float] = {}
        for name, (c0, c1, c2, c3) in self.coeffs.items():
            q[name] = c0 + dt * (c1 + dt * (c2 + dt * c3))
            qd[name] = c1 + dt * (2.0 * c2 + 3.0 * c3 * dt)
            qdd[name] = 2.0 * c2 + 6.0 * c3 * dt
        return q, qd, qdd

    def to_dict(self) -> dict[str, Any]:
        return {
            "t0": self.t0,
            "t1": self.t1,
            "coeffs": {k: list(v) for k, v in self.coeffs.items()},
        }


@dataclass
class SegmentInfo:
    """Diagnostics for one waypoint-to-waypoint move."""

    index: int
    duration: float
    mode: str  # "scurve" | "spline"
    binding_joint: Optional[str]
    binding_limit: Optional[str]  # "velocity" | "acceleration" | "jerk" | None
    peak_velocity: float
    peak_acceleration: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "duration": self.duration,
            "mode": self.mode,
            "binding_joint": self.binding_joint,
            "binding_limit": self.binding_limit,
            "peak_velocity": self.peak_velocity,
            "peak_acceleration": self.peak_acceleration,
        }


@dataclass
class TrajectoryPlan:
    """A fully-timed trajectory as piecewise cubics, plus diagnostics."""

    segments: list[PolySegment] = field(default_factory=list)
    joints: list[str] = field(default_factory=list)
    info: list[SegmentInfo] = field(default_factory=list)
    waypoint_times: list[float] = field(default_factory=list)
    # Extra description from whoever built the plan (e.g. cartesian IK stats).
    meta: dict[str, Any] = field(default_factory=dict)
    _ends: list[float] = field(default_factory=list, repr=False, compare=False)
    _ends_len: int = field(default=-1, repr=False, compare=False)

    @property
    def duration(self) -> float:
        return self.segments[-1].t1 if self.segments else 0.0

    def _segment_at(self, t: float) -> PolySegment:
        t = float(t)
        if not self.segments:
            raise ValueError("empty plan")
        if t <= self.segments[0].t0:
            return self.segments[0]
        # Spline-fitted sample trajectories have one segment per sample, so
        # search the (monotonic) end times rather than scanning.
        ends = [seg.t1 + _EPS for seg in self.segments] if self._ends_len != len(self.segments) else self._ends
        self._ends, self._ends_len = ends, len(self.segments)
        return self.segments[min(bisect.bisect_left(ends, t), len(self.segments) - 1)]

    def sample(self, t: float) -> tuple[dict[str, float], dict[str, float], dict[str, float]]:
        return self._segment_at(t).evaluate(t)

    def sample_uniform(
        self, rate_hz: float = 200.0, max_samples: int = 20_000
    ) -> Iterator[tuple[float, dict[str, float], dict[str, float], dict[str, float]]]:
        """Yield ``(t, q, qd, qdd)`` at a fixed rate, endpoints included."""
        total = self.duration
        if total <= 0.0:
            q, qd, qdd = self.sample(0.0)
            yield 0.0, q, qd, qdd
            return
        n = int(min(max_samples, max(2, round(total * float(rate_hz)) + 1)))
        for i in range(n):
            t = total * i / (n - 1)
            q, qd, qdd = self.sample(t)
            yield t, q, qd, qdd

    def peak_stats(self, rate_hz: float = 400.0) -> dict[str, dict[str, float]]:
        """Max |qd| and |qdd| per joint, for limit verification and display."""
        out = {n: {"velocity": 0.0, "acceleration": 0.0} for n in self.joints}
        for _t, _q, qd, qdd in self.sample_uniform(rate_hz):
            for n in self.joints:
                out[n]["velocity"] = max(out[n]["velocity"], abs(qd.get(n, 0.0)))
                out[n]["acceleration"] = max(out[n]["acceleration"], abs(qdd.get(n, 0.0)))
        return out

    def to_dict(self) -> dict[str, Any]:
        return {
            "duration": self.duration,
            "joints": self.joints,
            "segments": [s.to_dict() for s in self.segments],
            "waypoint_times": self.waypoint_times,
            "segment_info": [i.to_dict() for i in self.info],
            "meta": self.meta,
        }


# ----------------------------------------------------------------------------
# Scalar double-S (7-phase, jerk-limited) profile over s: 0 -> 1
# ----------------------------------------------------------------------------


@dataclass
class _ScurvePhases:
    """Phase durations for a rest-to-rest double-S profile."""

    t_jerk: float  # duration of each constant-jerk ramp
    t_const_accel: float  # duration of the constant-acceleration plateau
    t_cruise: float  # duration of the constant-velocity plateau
    peak_velocity: float
    peak_acceleration: float
    binding: str  # "velocity" | "acceleration" | "jerk"

    @property
    def t_accel(self) -> float:
        return 2.0 * self.t_jerk + self.t_const_accel

    @property
    def total(self) -> float:
        return 2.0 * self.t_accel + self.t_cruise


def solve_scurve(distance: float, v_max: float, a_max: float, j_max: float) -> _ScurvePhases:
    """Rest-to-rest double-S profile covering ``distance``.

    Uses the fact that a symmetric acceleration phase has mean velocity exactly
    ``v_peak / 2``, so the displacement bookkeeping is closed-form rather than
    an integration.
    """
    D = float(distance)
    if D <= _EPS:
        return _ScurvePhases(0.0, 0.0, 0.0, 0.0, 0.0, "velocity")
    V = max(_EPS, float(v_max))
    A = max(_EPS, float(a_max))
    J = max(_EPS, float(j_max))

    # --- Case 1: cruise at V. Does acceleration reach A on the way up? ------
    if V * J >= A * A:
        t_j = A / J
        t_ca = V / A - t_j
        peak_a = A
        binding_accel = "acceleration"
    else:
        # Jerk-limited: the acceleration ramp turns around before reaching A.
        t_j = math.sqrt(V / J)
        t_ca = 0.0
        peak_a = J * t_j
        binding_accel = "jerk"
    t_a = 2.0 * t_j + t_ca
    t_cruise = D / V - t_a
    if t_cruise >= -_EPS:
        return _ScurvePhases(t_j, t_ca, max(0.0, t_cruise), V, peak_a, "velocity")

    # --- Case 2: too short to reach V. Solve for the peak velocity ----------
    # Assume A is still reached: D = v*(v/A + A/J)  ->  v^2/A + v*A/J - D = 0.
    disc = (A / J) ** 2 + 4.0 * D / A
    v_peak = 0.5 * A * (math.sqrt(disc) - A / J)
    t_ca = v_peak / A - A / J
    if t_ca >= -_EPS:
        t_j = A / J
        return _ScurvePhases(t_j, max(0.0, t_ca), 0.0, v_peak, A, "acceleration")

    # --- Case 3: neither V nor A is reached; pure jerk-limited --------------
    # D = v * 2*sqrt(v/J)  ->  v = (D^2 * J / 4)^(1/3)
    v_peak = (D * D * J / 4.0) ** (1.0 / 3.0)
    t_j = math.sqrt(v_peak / J)
    return _ScurvePhases(t_j, 0.0, 0.0, v_peak, J * t_j, "jerk")


def _scurve_phase_table(ph: _ScurvePhases) -> list[tuple[float, float]]:
    """``(duration, jerk_sign)`` for the seven phases, zero-length ones kept out."""
    J_up = 1.0
    phases = [
        (ph.t_jerk, +J_up),
        (ph.t_const_accel, 0.0),
        (ph.t_jerk, -J_up),
        (ph.t_cruise, 0.0),
        (ph.t_jerk, -J_up),
        (ph.t_const_accel, 0.0),
        (ph.t_jerk, +J_up),
    ]
    return [(d, s) for (d, s) in phases if d > _EPS]


def scurve_polynomials(
    ph: _ScurvePhases, jerk: float, t_start: float
) -> tuple[list[tuple[float, float, tuple[float, float, float, float]]], float]:
    """Piecewise cubics for ``s(t)`` over the double-S profile.

    Returns ``[(t0, t1, (c0, c1, c2, c3)), ...]`` and the final ``s`` value
    (which should be 1 up to rounding; the caller renormalizes).
    """
    out: list[tuple[float, float, tuple[float, float, float, float]]] = []
    t = t_start
    s, v, a = 0.0, 0.0, 0.0
    for duration, sign in _scurve_phase_table(ph):
        j = sign * jerk
        out.append((t, t + duration, (s, v, a / 2.0, j / 6.0)))
        dt = duration
        s = s + v * dt + a * dt * dt / 2.0 + j * dt**3 / 6.0
        v = v + a * dt + j * dt * dt / 2.0
        a = a + j * dt
        t += duration
    return out, s


def _plan_stop_segment(
    index: int,
    a: dict[str, float],
    b: dict[str, float],
    limits: dict[str, JointLimits],
    t_start: float,
) -> tuple[list[PolySegment], SegmentInfo]:
    """One rest-to-rest synchronized move between two waypoints."""
    names = sorted(limits)
    delta = {n: float(b.get(n, a.get(n, 0.0))) - float(a.get(n, 0.0)) for n in names}
    max_delta = max((abs(d) for d in delta.values()), default=0.0)

    if max_delta <= _EPS:
        # Identical waypoints: emit a zero-length hold so playback still has a
        # breakpoint to land on, rather than dividing by zero.
        coeffs = {n: (float(a.get(n, 0.0)), 0.0, 0.0, 0.0) for n in names}
        seg = PolySegment(t_start, t_start, coeffs)
        return [seg], SegmentInfo(index, 0.0, "scurve", None, None, 0.0, 0.0)

    # Collapse per-joint limits onto the scalar progress variable and remember
    # which joint set each one, so the UI can say *why* a segment is slow.
    def _tightest(attr: str) -> tuple[float, str]:
        best = math.inf
        who = names[0]
        for n in names:
            d = abs(delta[n])
            if d <= _EPS:
                continue
            cap = getattr(limits[n], attr) / d
            if cap < best:
                best, who = cap, n
        return best, who

    s_v, who_v = _tightest("max_velocity")
    s_a, who_a = _tightest("max_acceleration")
    s_j, who_j = _tightest("max_jerk")

    ph = solve_scurve(1.0, s_v, s_a, s_j)
    polys, s_end = scurve_polynomials(ph, s_j, t_start)
    scale = 1.0 / s_end if abs(s_end) > _EPS else 1.0

    segments: list[PolySegment] = []
    for t0, t1, (c0, c1, c2, c3) in polys:
        coeffs: dict[str, tuple[float, float, float, float]] = {}
        for n in names:
            q0 = float(a.get(n, 0.0))
            d = delta[n] * scale
            coeffs[n] = (q0 + d * c0, d * c1, d * c2, d * c3)
        segments.append(PolySegment(t0, t1, coeffs))

    binding = {"velocity": who_v, "acceleration": who_a, "jerk": who_j}[ph.binding]
    info = SegmentInfo(
        index=index,
        duration=ph.total,
        mode="scurve",
        binding_joint=binding,
        binding_limit=ph.binding,
        peak_velocity=ph.peak_velocity * max_delta,
        peak_acceleration=ph.peak_acceleration * max_delta,
    )
    return segments, info


# ----------------------------------------------------------------------------
# Clamped cubic spline through a run of pass-through waypoints
# ----------------------------------------------------------------------------


def _solve_tridiagonal(
    lower: Sequence[float], diag: Sequence[float], upper: Sequence[float], rhs: Sequence[float]
) -> list[float]:
    """Thomas algorithm. Kept local so we don't pull in scipy for 20 lines."""
    n = len(diag)
    if n == 0:
        return []
    c = list(upper)
    d = list(rhs)
    b = list(diag)
    for i in range(1, n):
        w = lower[i] / b[i - 1]
        b[i] -= w * c[i - 1]
        d[i] -= w * d[i - 1]
    x = [0.0] * n
    x[-1] = d[-1] / b[-1]
    for i in range(n - 2, -1, -1):
        x[i] = (d[i] - c[i] * x[i + 1]) / b[i]
    return x


def clamped_cubic_slopes(times: Sequence[float], values: Sequence[float]) -> list[float]:
    """Slopes of the clamped cubic spline (zero velocity at both ends).

    Solving for slopes rather than second derivatives keeps the Hermite form
    directly usable and makes the C2 condition the tridiagonal system below.
    """
    n = len(times) - 1
    if n < 1:
        return [0.0] * len(times)
    h = [times[i + 1] - times[i] for i in range(n)]
    d = [(values[i + 1] - values[i]) / h[i] if h[i] > _EPS else 0.0 for i in range(n)]
    m = [0.0] * (n + 1)
    if n == 1:
        return m  # both ends clamped to zero; a single interval is fully determined
    size = n - 1
    lower = [0.0] * size
    diag = [0.0] * size
    upper = [0.0] * size
    rhs = [0.0] * size
    for k in range(size):
        i = k + 1
        lower[k] = h[i]
        diag[k] = 2.0 * (h[i - 1] + h[i])
        upper[k] = h[i - 1]
        rhs[k] = 3.0 * (h[i] * d[i - 1] + h[i - 1] * d[i])
    # Fold the known clamped end slopes (both zero) into the right-hand side.
    rhs[0] -= h[1] * m[0]
    rhs[-1] -= h[n - 2] * m[n]
    interior = _solve_tridiagonal(lower, diag, upper, rhs)
    for k, v in enumerate(interior):
        m[k + 1] = v
    return m


def _hermite_coeffs(
    y0: float, y1: float, m0: float, m1: float, h: float
) -> tuple[float, float, float, float]:
    """Cubic in local ``dt`` matching values and slopes at both ends."""
    if h <= _EPS:
        return (y0, 0.0, 0.0, 0.0)
    c0 = y0
    c1 = m0
    c2 = (3.0 * (y1 - y0) / h - 2.0 * m0 - m1) / h
    c3 = (2.0 * (y0 - y1) / h + m0 + m1) / (h * h)
    return (c0, c1, c2, c3)


def _plan_spline_run(
    first_index: int,
    points: Sequence[dict[str, float]],
    limits: dict[str, JointLimits],
    t_start: float,
) -> tuple[list[PolySegment], list[SegmentInfo], list[float]]:
    """Fit a clamped cubic spline through ``points`` and time-scale to limits."""
    names = sorted(limits)
    n_int = len(points) - 1

    # Nominal knot spacing: proportional to the velocity-limited time of each
    # leg, so a long move gets proportionally more time before scaling.
    nominal: list[float] = []
    for i in range(n_int):
        worst = 0.0
        for n in names:
            d = abs(float(points[i + 1].get(n, 0.0)) - float(points[i].get(n, 0.0)))
            worst = max(worst, d / max(_EPS, limits[n].max_velocity))
        nominal.append(max(worst, 1e-3))
    knots = [0.0]
    for h in nominal:
        knots.append(knots[-1] + h)

    slopes = {n: clamped_cubic_slopes(knots, [float(p.get(n, 0.0)) for p in points]) for n in names}

    # Peak derivatives over the unscaled spline, sampled densely enough that a
    # cubic's extremum is not missed. Tracked per segment (``seg_peak_*``), not
    # just accumulated per joint across the whole run: every segment used to
    # report the same run-wide peaks/binding joint in its SegmentInfo even
    # though its own coefficients (and thus its own peaks) differ from its
    # neighbours'. The scale factor ``k`` below is legitimately a whole-run
    # quantity -- every segment in one spline run shares one uniform scale --
    # but each segment's *diagnostics* should describe that segment.
    peak_v = {n: 0.0 for n in names}
    peak_a = {n: 0.0 for n in names}
    peak_j = {n: 0.0 for n in names}
    seg_peak_v: list[dict[str, float]] = []
    seg_peak_a: list[dict[str, float]] = []
    seg_peak_j: list[dict[str, float]] = []
    raw: list[tuple[float, float, dict[str, tuple[float, float, float, float]]]] = []
    for i in range(n_int):
        h = knots[i + 1] - knots[i]
        coeffs = {}
        pv_i: dict[str, float] = {}
        pa_i: dict[str, float] = {}
        pj_i: dict[str, float] = {}
        for n in names:
            c = _hermite_coeffs(
                float(points[i].get(n, 0.0)),
                float(points[i + 1].get(n, 0.0)),
                slopes[n][i],
                slopes[n][i + 1],
                h,
            )
            coeffs[n] = c
            # v is quadratic, a linear, j constant: extrema are at the ends or
            # at the single stationary point of v.
            cand_t = [0.0, h]
            if abs(c[3]) > _EPS:
                t_star = -c[2] / (3.0 * c[3])
                if 0.0 < t_star < h:
                    cand_t.append(t_star)
            pv = 0.0
            for tt in cand_t:
                pv = max(pv, abs(c[1] + 2 * c[2] * tt + 3 * c[3] * tt * tt))
            pa = max(abs(2 * c[2]), abs(2 * c[2] + 6 * c[3] * h))
            pj = abs(6 * c[3])
            pv_i[n], pa_i[n], pj_i[n] = pv, pa, pj
            peak_v[n] = max(peak_v[n], pv)
            peak_a[n] = max(peak_a[n], pa)
            peak_j[n] = max(peak_j[n], pj)
        seg_peak_v.append(pv_i)
        seg_peak_a.append(pa_i)
        seg_peak_j.append(pj_i)
        raw.append((knots[i], knots[i + 1], coeffs))

    # Uniform time scaling. Exact for polynomials: t -> k*t divides velocity by
    # k, acceleration by k^2 and jerk by k^3, so one pass suffices.
    def _binding(
        pv: dict[str, float], pa: dict[str, float], pj: dict[str, float]
    ) -> tuple[float, Optional[str], Optional[str]]:
        need_best = 0.0
        joint_best: Optional[str] = None
        limit_best: Optional[str] = None
        for n in names:
            for value, cap, power, label in (
                (pv[n], limits[n].max_velocity, 1.0, "velocity"),
                (pa[n], limits[n].max_acceleration, 2.0, "acceleration"),
                (pj[n], limits[n].max_jerk, 3.0, "jerk"),
            ):
                if value <= _EPS or cap <= _EPS:
                    continue
                need = (value / cap) ** (1.0 / power)
                if need > need_best:
                    need_best, joint_best, limit_best = need, n, label
        return need_best, joint_best, limit_best

    k, binding_joint, binding_limit = _binding(peak_v, peak_a, peak_j)
    k = max(k, 1.0)

    segments: list[PolySegment] = []
    infos: list[SegmentInfo] = []
    times = [t_start + kt * k for kt in knots]
    for i, (a0, a1, coeffs) in enumerate(raw):
        scaled = {
            n: (c[0], c[1] / k, c[2] / (k * k), c[3] / (k**3)) for n, c in coeffs.items()
        }
        segments.append(PolySegment(times[i], times[i + 1], scaled))

        # This segment's own tightest joint/limit -- same formula as the
        # run-wide scale factor above, scoped to this segment's own peaks.
        # Falls back to the run-wide binding joint if this segment has no
        # binding one of its own (e.g. a zero-movement segment).
        _seg_need, seg_joint, seg_limit = _binding(seg_peak_v[i], seg_peak_a[i], seg_peak_j[i])
        infos.append(
            SegmentInfo(
                index=first_index + i,
                duration=times[i + 1] - times[i],
                mode="spline",
                binding_joint=seg_joint if seg_joint is not None else binding_joint,
                binding_limit=seg_limit if seg_limit is not None else binding_limit,
                peak_velocity=max(seg_peak_v[i][n] / k for n in names),
                peak_acceleration=max(seg_peak_a[i][n] / (k * k) for n in names),
            )
        )
    return segments, infos, times


# ----------------------------------------------------------------------------
# Top-level planner
# ----------------------------------------------------------------------------


def plan_trajectory(
    waypoints: Sequence[Waypoint],
    limits: dict[str, JointLimits],
    *,
    time_scale: float = 1.0,
) -> TrajectoryPlan:
    """Time a waypoint list under per-joint velocity/acceleration/jerk limits.

    Consecutive pass-through waypoints are splined together; every ``stop``
    waypoint breaks the trajectory into an independent rest-to-rest double-S
    move. ``time_scale`` > 1 slows the whole result down proportionally (still
    exact, still limit-respecting).
    """
    names = sorted(limits)
    plan = TrajectoryPlan(joints=names)
    if len(waypoints) == 0:
        return plan
    if len(waypoints) == 1:
        q = {n: float(waypoints[0].joints.get(n, 0.0)) for n in names}
        plan.segments.append(PolySegment(0.0, 0.0, {n: (q[n], 0.0, 0.0, 0.0) for n in names}))
        plan.waypoint_times = [0.0]
        return plan

    # Endpoints always stop; interior points stop unless marked pass-through.
    stops = [True] + [bool(w.stop) for w in waypoints[1:-1]] + [True]

    t = 0.0
    plan.waypoint_times = [0.0]
    i = 0
    while i < len(waypoints) - 1:
        # Collect the longest run of pass-through waypoints starting at i.
        j = i + 1
        while j < len(waypoints) - 1 and not stops[j]:
            j += 1
        if j == i + 1:
            segs, info = _plan_stop_segment(
                i, waypoints[i].joints, waypoints[i + 1].joints, limits, t
            )
            plan.segments.extend(segs)
            plan.info.append(info)
            t = segs[-1].t1
            plan.waypoint_times.append(t)
        else:
            pts = [w.joints for w in waypoints[i : j + 1]]
            segs, infos, times = _plan_spline_run(i, pts, limits, t)
            plan.segments.extend(segs)
            plan.info.extend(infos)
            t = segs[-1].t1
            plan.waypoint_times.extend(times[1:])
        i = j

    if time_scale != 1.0 and time_scale > 0:
        plan = _rescale(plan, float(time_scale))
    return plan


def _rescale(plan: TrajectoryPlan, k: float) -> TrajectoryPlan:
    out = TrajectoryPlan(joints=plan.joints, waypoint_times=[t * k for t in plan.waypoint_times])
    for seg in plan.segments:
        out.segments.append(
            PolySegment(
                seg.t0 * k,
                seg.t1 * k,
                {
                    n: (c[0], c[1] / k, c[2] / (k * k), c[3] / (k**3))
                    for n, c in seg.coeffs.items()
                },
            )
        )
    for info in plan.info:
        out.info.append(
            SegmentInfo(
                info.index,
                info.duration * k,
                info.mode,
                info.binding_joint,
                info.binding_limit,
                info.peak_velocity / k,
                info.peak_acceleration / (k * k),
            )
        )
    return out


def limits_from_velocities(
    velocities: dict[str, float],
    accel_ratio: float = DEFAULT_ACCEL_RATIO,
    jerk_ratio: float = DEFAULT_JERK_RATIO,
    overrides: Optional[dict[str, dict[str, float]]] = None,
) -> dict[str, JointLimits]:
    """Build planner limits from per-joint velocity limits plus ratio defaults.

    URDF has no standard acceleration or jerk field, so those are derived from
    the declared velocity limit unless the caller overrides them per joint.
    """
    out: dict[str, JointLimits] = {}
    for name, v_max in velocities.items():
        jl = JointLimits.from_velocity(name, v_max, accel_ratio, jerk_ratio)
        ov = (overrides or {}).get(name) or {}
        out[name] = JointLimits(
            name=name,
            max_velocity=max(_EPS, float(ov.get("max_velocity", jl.max_velocity))),
            max_acceleration=max(_EPS, float(ov.get("max_acceleration", jl.max_acceleration))),
            max_jerk=max(_EPS, float(ov.get("max_jerk", jl.max_jerk))),
        )
    return out
