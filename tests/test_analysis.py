import json

import numpy as np
import pytest

from arm_analyzer.analysis import UTIL_CAP, analyze
from arm_analyzer.dynamics import Payload
from arm_analyzer.robot import parse_arm
from arm_analyzer.trajectory import load_trajectory

from conftest import EXAMPLES

SWING = {
    "format": "waypoints",
    "waypoints": [{"joints": {"shoulder": -1.0}}, {"joints": {"shoulder": 1.0}}],
}


def _run(arm, traj=SWING, **kw):
    return analyze(arm, load_trajectory(arm, traj), **kw)


def test_result_is_strict_json(simple_arm):
    text = (EXAMPLES / "trajectories" / "sine_sweep.csv").read_text()
    result = analyze(simple_arm, load_trajectory(simple_arm, text, kind="csv"), rate_hz=100)
    json.dumps(result, allow_nan=False)
    assert result["joints"] == simple_arm.actuated
    assert len(result["t"]) == len(result["series"]["j1"]["tau"])


def test_motor_side_is_the_lossless_joint_torque_divided_by_ratio(one_joint):
    arm = parse_arm(one_joint(ratio=40, rotor_inertia=1e-5))
    r = _run(arm)
    s = {k: np.array(v, dtype=float) for k, v in r["series"]["shoulder"].items()}
    n = 40.0
    assert np.allclose(s["motor_speed"], n * s["qd"])
    assert np.allclose(s["motor_torque"], s["tau_link"] / n + 1e-5 * n * s["qdd"])
    # Default efficiencies are 1: motor torque through the ratio is the joint torque.
    assert np.allclose(s["motor_torque"] * n, s["tau"])
    assert "tau_friction" not in s


def test_efficiency_placeholders_load_the_motor(one_joint):
    arm = parse_arm(
        one_joint(ratio=40, rotor_inertia=1e-5, transmission='<transmission ratio="1" efficiency="0.9"/>')
    )
    arm.drives["shoulder"].set_efficiency(gearbox=0.8)
    r = _run(arm)
    s = {k: np.array(v, dtype=float) for k, v in r["series"]["shoulder"].items()}
    n, eta = 40.0, 0.8 * 0.9
    load = s["tau_link"]
    driving = load * s["qd"] >= 0
    expected = np.where(driving, load / (n * eta), load * eta / n) + 1e-5 * n * s["qdd"]
    assert np.allclose(s["motor_torque"], expected)
    # Both regimes occur on a swing through the bottom of a gravity arc.
    assert driving.any() and (~driving).any()
    # Losses sit in the drive, not in the joint torque the motion requires.
    lossless = _run(parse_arm(one_joint(ratio=40, rotor_inertia=1e-5)))["series"]["shoulder"]
    assert np.allclose(s["tau"], lossless["tau"])
    assert r["summary"][0]["drive"]["efficiency"] == pytest.approx(
        {"gearbox": 0.8, "transmission": 0.9, "total": eta}
    )


def test_gearbox_sees_output_torque_through_transmission(one_joint):
    arm = parse_arm(
        one_joint(ratio=20, transmission='<transmission ratio="2.5"/>')
    )
    s = _run(arm)["series"]["shoulder"]
    assert np.allclose(s["gearbox_torque"], np.array(s["tau_link"]) / 2.5)
    assert np.allclose(s["motor_speed"], 50 * np.array(s["qd"]))


def test_heavy_payload_goes_over(one_joint):
    arm = parse_arm(one_joint(ratio=50, motor_link="base"))
    light = _run(arm)["summary"][0]
    heavy = _run(arm, payload=Payload("arm", 20.0, np.array([1.0, 0, 0])))["summary"][0]
    assert light["status"] in ("ok", "marginal")
    assert heavy["status"] == "over"
    assert heavy["drive"]["motor"]["peak_util"] > 1
    assert heavy["joint"]["envelope_util"] > 1


def test_utilization_matches_envelope(one_joint):
    arm = parse_arm(one_joint(ratio=50))
    r = _run(arm)
    summary = r["summary"][0]
    s = r["series"]["shoulder"]
    ms = arm.drives["shoulder"].motor_spec
    limit = ms.torque_limit(np.array(s["motor_speed"]))
    expected = np.max(np.abs(s["motor_torque"]) / limit)
    assert summary["drive"]["motor"]["peak_util"] == pytest.approx(expected)
    assert summary["drive"]["motor"]["rms_util"] == pytest.approx(
        np.sqrt(np.mean(np.square(s["motor_torque"]))) / 0.4
    )
    assert max(s["util_motor"]) == pytest.approx(expected)


def test_undeclared_ratings_are_reported_as_unknown(one_joint):
    arm = parse_arm(one_joint(extra_motor_attrs=""))
    summary = _run(arm)["summary"][0]
    assert summary["drive"]["motor"]["peak_util"] is None
    assert summary["drive"]["motor"]["rms_util"] is None
    assert "util_motor" not in _run(arm)["series"]["shoulder"]


def test_zero_speed_limit_caps_utilization(one_joint):
    arm = parse_arm(one_joint())
    plan = load_trajectory(arm, SWING)
    # Shrink the limit after planning, so the plan itself still moves.
    arm.joints["shoulder"].velocity = 1e-12
    summary = analyze(arm, plan)["summary"][0]
    assert summary["joint"]["speed_util"] == UTIL_CAP
    assert summary["status"] == "over"


def test_rate_is_validated(one_joint):
    with pytest.raises(ValueError, match="rate_hz"):
        _run(parse_arm(one_joint()), rate_hz=0.1)
