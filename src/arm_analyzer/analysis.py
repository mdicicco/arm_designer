"""Speed/torque analysis of an arm along a trajectory.

For every sample of the trajectory this runs inverse dynamics, then carries
the joint torque back through each drive to find what the gearbox and motor
actually see:

* **joint side**   ``tau_joint = tau_link + tau_rotor`` (the torque the motion
  requires, without losses) against the joint speed ``qd``. Its envelope is
  the motor envelope referred through the drivetrain
  (``tau_m(w*N) * N * eta``), capped by the gearbox rating.
* **gearbox output**  ``tau_link / N_tr`` through the transmission efficiency,
  against its peak and rated output torque.
* **motor side**   ``w_m = N * qd`` and
  ``tau_m = tau_link / (N * eta) + (I_rotor + I_gb_in) * N * qdd`` while the
  drive powers the load, ``tau_link * eta / N + ...`` while the load
  back-drives it. That is what gets compared with the motor's peak /
  continuous torque-speed curves.

Drives coupled by a differential are carried back together: the group's joint
torques are mapped through ``A^-T`` (``robot.Coupling``), so each motor is
sized on its real share of both joints -- half the sum of them in one motor,
half the difference in the other -- rather than on one joint alone.

``eta`` = gearbox efficiency x transmission efficiency. Both are placeholders
that default to 1.0, where the drivetrain is lossless and ``tau_m * N`` equals
the joint torque exactly; lowering them stands in for gear, joint and
belt/cable friction.

Utilization is reported as ``max(|tau| / envelope(|w|))`` over the run -- so
a point above the curve at high speed counts even when its torque is below
the stall rating -- and RMS torque against the continuous rating. Anything
above 1.0 is out of spec; above ``WARN_FRACTION`` is flagged as marginal.
"""

from __future__ import annotations

import math
from typing import Any, Optional

import numpy as np

from arm_analyzer.dynamics import GRAVITY, DynamicsModel, Payload, torque_series
from arm_analyzer.profile import TrajectoryPlan
from arm_analyzer.robot import ArmDescription, Drive

WARN_FRACTION = 0.8
ENVELOPE_POINTS = 64
# Utilizations are capped here so a zero rating (infinite utilization) still
# serializes as JSON and still reads as "over".
UTIL_CAP = 999.0
# The planner drives joints to exactly their velocity limit; don't let
# floating-point rounding turn "at the limit" into "over".
UTIL_TOL = 1e-6


def _rms(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(x * x))) if x.size else 0.0


def _ratio(num: float, den: Optional[float]) -> Optional[float]:
    if den is None or not math.isfinite(den):
        return None
    if den <= 0:
        return UTIL_CAP if num > 0 else 0.0
    return min(num / den, UTIL_CAP)


def _util_series(tau: np.ndarray, limit: np.ndarray) -> Optional[np.ndarray]:
    """``|tau| / limit`` per sample (NaN where no limit is declared)."""
    finite = np.isfinite(limit)
    if not finite.any():
        return None
    a = np.abs(tau)
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.where(limit > 0, a / limit, np.where(a > 1e-9, UTIL_CAP, 0.0))
    return np.where(finite, np.minimum(u, UTIL_CAP), np.nan)


def _envelope_util(u: Optional[np.ndarray]) -> Optional[float]:
    if u is None or not np.isfinite(u).any():
        return None
    return float(np.nanmax(u))


SERIES_DIGITS = 6  # significant digits sent for plotted series


def _json_list(x: np.ndarray) -> list[Optional[float]]:
    """Series for the browser: 6 significant digits (plots need no more), and
    None for non-finite values. Cuts the response to roughly a third."""
    return [float(f"{v:.{SERIES_DIGITS}g}") if math.isfinite(v) else None for v in x]


def _status(*utils: Optional[float]) -> str:
    vals = [u for u in utils if u is not None]
    if not vals:
        return "unknown"
    worst = max(vals)
    if worst > 1.0 + UTIL_TOL:
        return "over"
    if worst > WARN_FRACTION + UTIL_TOL:
        return "marginal"
    return "ok"


def _curve(speed_max: float, fn) -> dict[str, list[float]]:
    """Sample an envelope function on ``[0, speed_max]`` for plotting."""
    w = np.linspace(0.0, speed_max, ENVELOPE_POINTS)
    tau = fn(w)
    return {
        "speed": [float(x) for x in w],
        "torque": [float(x) if math.isfinite(x) else None for x in tau],
    }


def _group_side(
    drives: list[Drive],
    A: np.ndarray,
    QD: np.ndarray,
    QDD: np.ndarray,
    TAU_LOAD: np.ndarray,
) -> list[dict[str, np.ndarray]]:
    """Gearbox-output and motor-side series for one group of drives.

    ``A`` maps joint rates to motor rates (``w = A q'``), so motor torque is
    ``A^-T tau``; a plain drive is the 1x1 case ``A = [[N]]`` and comes out
    exactly as ``tau / N``. Columns of ``QD``/``QDD``/``TAU_LOAD`` are this
    group's joints, in ``drives`` order. Efficiency is applied joint by joint
    before the map -- exact for a plain drive, and for a coupled one as long as
    its members share an efficiency (they are placeholders at 1.0 today).
    """
    eta = np.array([d.total_efficiency for d in drives])
    motoring = TAU_LOAD * QD >= 0.0
    tau_eff = np.where(motoring, TAU_LOAD / eta, TAU_LOAD * eta)
    tau_m_load = tau_eff @ np.linalg.inv(A)
    W = QD @ A.T
    spin = np.array(
        [d.motor_spec.rotor_inertia + d.gearbox_spec.input_inertia for d in drives]
    )
    tau_m = tau_m_load + spin * (QDD @ A.T)

    out = []
    for i, drive in enumerate(drives):
        gs = drive.gearbox_spec
        # Gearbox output, from its own input torque and its own direction.
        driving = tau_m_load[:, i] * W[:, i] >= 0.0
        tau_gb = tau_m_load[:, i] * np.where(
            driving, gs.ratio * gs.efficiency, gs.ratio / gs.efficiency
        )
        s = {
            "motor_speed": W[:, i],
            "motor_torque": tau_m[:, i],
            "gearbox_torque": tau_gb,
            "motor_power": tau_m[:, i] * W[:, i],
        }
        kt, r = drive.motor_spec.torque_constant, drive.motor_spec.resistance
        if kt:
            current = s["motor_torque"] / kt
            s["motor_current"] = current
            if r:
                s["copper_loss"] = current * current * r
        out.append(s)
    return out


def _drive_groups(arm: ArmDescription, names: list[str]) -> list[tuple[list[str], Any]]:
    """Driven joints as ``(joints, coupling)``; uncoupled drives stand alone."""
    groups: list[tuple[list[str], Any]] = []
    seen: set[str] = set()
    for n in names:
        if n in seen or n not in arm.drives:
            continue
        c = arm.coupling(n)
        if c is None:
            groups.append(([n], None))
            seen.add(n)
        else:
            groups.append((list(c.joints), c))
            seen.update(c.joints)
    return groups


def _joint_summary(
    arm: ArmDescription,
    name: str,
    s: dict[str, np.ndarray],
    duration: float,
) -> dict[str, Any]:
    j = arm.joints[name]
    drive = arm.drives.get(name)
    tau, qd = s["tau"], s["qd"]
    out: dict[str, Any] = {
        "name": name,
        "type": j.joint_type,
        "joint": {
            "peak_torque": float(np.max(np.abs(tau))),
            "rms_torque": _rms(tau),
            "peak_speed": float(np.max(np.abs(qd))),
            "rms_speed": _rms(qd),
            "peak_power": float(np.max(np.abs(tau * qd))),
            "effort_limit": arm.effort_limit(name),
            "velocity_limit": arm.velocity_limit(name),
            "range_used": [float(np.min(s["q"])), float(np.max(s["q"]))],
            "range_limit": [j.lower, j.upper],
        },
        "drive": None,
    }
    jl = out["joint"]
    jl["torque_util"] = _ratio(jl["peak_torque"], jl["effort_limit"])
    jl["speed_util"] = _ratio(jl["peak_speed"], jl["velocity_limit"])
    lo, hi = j.lower, j.upper
    jl["range_ok"] = bool(jl["range_used"][0] >= lo - 1e-6 and jl["range_used"][1] <= hi + 1e-6)

    if drive is None:
        out["status"] = _status(jl["torque_util"], jl["speed_util"])
        out["envelopes"] = None
        return out

    ms, gs = drive.motor_spec, drive.gearbox_spec
    w_m, tau_m, tau_gb = s["motor_speed"], s["motor_torque"], s["gearbox_torque"]
    motor = {
        "peak_torque": float(np.max(np.abs(tau_m))),
        "rms_torque": _rms(tau_m),
        "peak_speed": float(np.max(np.abs(w_m))),
        "rms_speed": _rms(w_m),
        "peak_power": float(np.max(np.abs(s["motor_power"]))),
        "mean_power": float(np.mean(s["motor_power"])),
        "peak_util": _envelope_util(s.get("util_motor")),
        "rms_util": _ratio(_rms(tau_m), ms.continuous_torque),
        "speed_util": _ratio(float(np.max(np.abs(w_m))), ms.no_load_speed),
    }
    if "motor_current" in s:
        motor["peak_current"] = float(np.max(np.abs(s["motor_current"])))
        motor["rms_current"] = _rms(s["motor_current"])
    if "copper_loss" in s:
        motor["mean_copper_loss"] = float(np.mean(s["copper_loss"]))
        motor["energy_copper"] = float(np.mean(s["copper_loss"]) * duration)

    gearbox = {
        "peak_torque": float(np.max(np.abs(tau_gb))),
        "rms_torque": _rms(tau_gb),
        "peak_input_speed": float(np.max(np.abs(w_m))),
        "peak_util": _ratio(float(np.max(np.abs(tau_gb))), gs.peak_torque),
        "rms_util": _ratio(_rms(tau_gb), gs.rated_torque),
        "speed_util": _ratio(float(np.max(np.abs(w_m))), gs.max_input_speed),
    }
    jl["envelope_util"] = _envelope_util(s.get("util_joint"))

    coupling = arm.coupling(name)
    out["drive"] = {
        "motor": motor,
        "gearbox": gearbox,
        "efficiency": {
            "gearbox": gs.efficiency,
            "transmission": drive.transmission_efficiency,
            "total": drive.total_efficiency,
        },
        "coupling": None
        if coupling is None
        else {
            "name": coupling.name,
            "type": coupling.kind,
            "joints": list(coupling.joints),
            "partners": coupling.partners(name),
        },
    }
    out["status"] = _status(
        jl["torque_util"],
        jl["speed_util"],
        motor["peak_util"],
        motor["rms_util"],
        motor["speed_util"],
        gearbox["peak_util"],
        gearbox["rms_util"],
        gearbox["speed_util"],
    )

    # Envelope curves for the speed-torque plots. The speed axis extends to the
    # no-load / max-input speed, or past the run's own peak if nothing is declared.
    # ``n`` is this joint's own motor turns per joint turn (its partners held
    # still), which for a plain drive is just the total ratio.
    if coupling is None:
        n = drive.total_ratio
    else:
        k = coupling.joints.index(name)
        n = abs(coupling.A[k, k])
    w_m_top = min(
        [v for v in (ms.no_load_speed, gs.max_input_speed) if v] or [math.inf]
    )
    if not math.isfinite(w_m_top):
        w_m_top = max(float(np.max(np.abs(w_m))) * 1.25, 1e-3)
    out["envelopes"] = {
        "motor_peak": _curve(w_m_top, lambda w: ms.torque_limit(w)),
        "motor_continuous": _curve(w_m_top, lambda w: ms.torque_limit(w, continuous=True)),
        "joint_peak": _curve(w_m_top / n, lambda w: arm.joint_torque_limit(name, w)),
        "joint_continuous": _curve(
            w_m_top / n, lambda w: arm.joint_torque_limit(name, w, continuous=True)
        ),
    }
    return out


def analyze(
    arm: ArmDescription,
    plan: TrajectoryPlan,
    *,
    rate_hz: float = 200.0,
    gravity: np.ndarray = GRAVITY,
    payload: Optional[Payload] = None,
) -> dict[str, Any]:
    """Sample ``plan`` at ``rate_hz`` and return series + per-joint summaries."""
    if not plan.segments:
        raise ValueError("trajectory is empty")
    if not (1.0 <= rate_hz <= 5000.0):
        raise ValueError("rate_hz must be between 1 and 5000")
    model = DynamicsModel.build(arm, payload)
    names = arm.actuated
    samples = list(plan.sample_uniform(rate_hz))

    t = np.array([s[0] for s in samples])
    Q, QD, QDD = (
        np.array([[float(s[k].get(n, 0.0)) for n in names] for s in samples]) for k in (1, 2, 3)
    )
    terms = torque_series(model, Q, QD, QDD, gravity)

    series: dict[str, dict[str, np.ndarray]] = {
        n: {
            "q": Q[:, j],
            "qd": QD[:, j],
            "qdd": QDD[:, j],
            "tau": terms["total"][:, j],
            "tau_link": terms["link"][:, j],
            "tau_rotor": terms["rotor"][:, j],
        }
        for j, n in enumerate(names)
    }

    # Motor and gearbox series, one drive group at a time: coupled drives have
    # to be carried back through the mechanism together.
    column = {n: j for j, n in enumerate(names)}
    for group, coupling in _drive_groups(arm, names):
        drives = [arm.drives[n] for n in group]
        cols = [column[n] for n in group]
        A = coupling.A if coupling is not None else np.array([[drives[0].total_ratio]])
        sides = _group_side(drives, A, QD[:, cols], QDD[:, cols], terms["link"][:, cols])
        for n, drive, side in zip(group, drives, sides):
            s = series[n]
            s.update(side)
            for key, u in (
                ("util_joint", _util_series(s["tau"], arm.joint_torque_limit(n, s["qd"]))),
                (
                    "util_motor",
                    _util_series(
                        s["motor_torque"], drive.motor_spec.torque_limit(s["motor_speed"])
                    ),
                ),
            ):
                if u is not None:
                    s[key] = u

    duration = float(plan.duration)
    summaries = [_joint_summary(arm, n, series[n], duration) for n in names]
    return {
        "joints": names,
        "t": t.tolist(),
        "duration": duration,
        "plan": plan.to_dict(),
        "series": {n: {k: _json_list(v) for k, v in s.items()} for n, s in series.items()},
        "summary": summaries,
        "mass_budget": arm.mass_budget(),
        "moving_mass": model.moving_mass,
        "payload_mass": float(payload.mass) if payload is not None else 0.0,
        "gravity": [float(x) for x in gravity],
        "rate_hz": rate_hz,
    }
