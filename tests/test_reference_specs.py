"""Check the example arms against the real robots they are modelled on.

These examples are the baseline every new design gets compared with, so it
matters how far they start from the machines they imitate. The published
numbers and their sources live in ``reference_specs.py``.

The tolerances here are **where the models actually are**, not where they
ought to be: the point is to pin the current agreement down so that a change
which drifts further away fails loudly. Where a model is known to disagree,
the test says so and says why rather than being silently loosened.

As of 2026-09-21, with gearbox masses derived from their ratings and motor
masses from their rating **and construction** (the per-form BLDC fit), the KR
lands within **1%** of the real robot and the WAM within **2%**. The UR5e comes
out 50% heavy because of how its own gearbox ratings are written -- see
``OVERWEIGHT_BY_RATING``.
"""

from __future__ import annotations

import math

import numpy as np
import pinocchio as pin
import pytest

from arm_analyzer.pin_model import build_model
from arm_analyzer.robot import load_arm

from conftest import EXAMPLES
from reference_specs import REFERENCE

# With motor masses per construction and gearbox masses per rating, the KR
# lands within 1% and the WAM within 2%. Held tight so any drift shows.
MASS_TOLERANCE = 0.05
REACH_SAMPLES = 20000

# KNOWN GAP, ur_style_6dof: its gearboxes declare rated_torque = 150 N*m on the
# inner joints, which is the UR5e's *peak* joint torque, not a reducer's
# continuous rating -- peak/rated comes out at 1.40 where the KR and WAM
# examples sit at 2.3, the usual figure for a harmonic drive. The gearbox mass
# law reads rated_torque as a continuous frame rating, so those three joints
# are sized as 3.5 kg reducers and the arm lands 38% heavy. Fix the ratings in
# scripts/make_ur_style.py; this xfail is strict, so it will flag the moment
# they are.
OVERWEIGHT_BY_RATING = {"ur_style_6dof.urdf"}


@pytest.fixture(scope="module", params=sorted(REFERENCE))
def case(request):
    filename = request.param
    return filename, load_arm(EXAMPLES / "robots" / filename), REFERENCE[filename]


def _max_horizontal_reach(arm, frame: str, samples: int = REACH_SAMPLES) -> float:
    """Furthest the named frame gets from the A1 axis, in metres.

    Manufacturers quote reach as a horizontal radius about the base axis, so
    that is what this measures -- to the flange or to the tool tip, whichever
    the published figure refers to.
    """
    model = build_model(arm, with_drives=False)
    data = model.createData()
    targets = []
    if frame == "flange":
        last = arm.order[-1]
        targets.append((model.getFrameId(last, pin.FrameType.BODY), np.eye(4)))
    else:
        for link in arm.links.values():
            for tip in link.tool_tips:
                targets.append(
                    (model.getFrameId(link.name, pin.FrameType.BODY), np.asarray(tip["T"]))
                )
    assert targets, f"no {frame} frame to measure"

    pin.seed(12345)
    best = 0.0
    for _ in range(samples):
        q = pin.randomConfiguration(model)
        pin.framesForwardKinematics(model, data, q)
        for fid, offset in targets:
            p = (data.oMf[fid].homogeneous @ offset)[:3, 3]
            best = max(best, float(math.hypot(p[0], p[1])))
    return best


def test_degrees_of_freedom_match(case):
    filename, arm, spec = case
    assert len(arm.actuated) == spec["dof"], filename


def test_mass_is_within_tolerance(case, request):
    """Total mass against the real robot.

    With gearbox masses from their ratings and motor masses from rating and
    construction, the KR lands within 1% and the WAM within 2%. The UR5e does
    not, for a reason in its own ratings rather than in either mass law -- see
    ``OVERWEIGHT_BY_RATING``.
    """
    filename, arm, spec = case
    if filename in OVERWEIGHT_BY_RATING:
        request.node.add_marker(
            pytest.mark.xfail(strict=True, reason="see OVERWEIGHT_BY_RATING")
        )
    got = arm.mass_budget()["total"]
    want = spec["mass_kg"]
    assert got == pytest.approx(want, rel=MASS_TOLERANCE), (
        f"{filename}: {got:.2f} kg vs {spec['real']} at {want} kg "
        f"({100 * (got / want - 1):+.0f}%)"
    )


def test_drive_mass_is_a_plausible_share_of_the_arm(case):
    """Motors plus reducers should be a substantial minority of an arm, not a
    rounding error and not most of it. This is the sanity check that catches a
    mass law being fed the wrong quantity -- it is what the UR5e's peak-torque
    gearbox ratings trip."""
    filename, arm, spec = case
    if filename in OVERWEIGHT_BY_RATING:
        pytest.xfail("see OVERWEIGHT_BY_RATING")
    budget = arm.mass_budget()
    drive_share = (budget["motors"] + budget["gearboxes"]) / budget["total"]
    assert 0.05 < drive_share < 0.45, (
        f"{filename}: drives are {100 * drive_share:.0f}% of the arm "
        f"({budget['motors']:.2f} kg motors + {budget['gearboxes']:.2f} kg gearboxes "
        f"of {budget['total']:.2f} kg)"
    )


def test_gearbox_peak_to_rated_ratio_is_realistic(case):
    """A harmonic drive's peak output torque is roughly 2-3x its continuous
    rating. A ratio near 1 means ``rated_torque`` is carrying a peak figure,
    which the gearbox mass law will read as a much larger frame."""
    filename, arm, spec = case
    if filename in OVERWEIGHT_BY_RATING:
        pytest.xfail("see OVERWEIGHT_BY_RATING")
    for name, drive in arm.drives.items():
        gs = drive.gearbox_spec
        if not (gs.peak_torque and gs.rated_torque):
            continue
        assert 1.8 <= gs.peak_torque / gs.rated_torque <= 4.0, (
            f"{filename}:{name} peak/rated = {gs.peak_torque / gs.rated_torque:.2f}"
        )


def test_joint_range_widths_match(case):
    """Widths, not endpoints: a model may use a different zero pose than the
    manufacturer (the KR example does), which shifts every limit but must not
    change how far an axis can travel."""
    filename, arm, spec = case
    published = spec["joint_range_deg"]
    assert len(published) == len(arm.actuated), filename
    for name, (lo, hi) in zip(arm.actuated, published):
        joint = arm.joints[name]
        got = math.degrees(joint.upper - joint.lower)
        want = hi - lo
        if filename == "ur_style_6dof.urdf" and name == "j3":
            # The UR5e's elbow is rated +/-360 deg; this example keeps it at
            # +/-160, which is the practical range and the conservative choice.
            assert got < want
            continue
        if filename == "wam_style_7dof.urdf" and name == "j5":
            # Barrett publishes -273/+71 deg; the example rounds to -275/+75.
            assert got == pytest.approx(want, abs=8.0), f"{filename}:{name}"
            continue
        assert got == pytest.approx(want, abs=1.0), f"{filename}:{name}"


def test_joint_speeds_match_where_published(case):
    filename, arm, spec = case
    published = spec["joint_speed_deg_s"]
    if published is None:
        pytest.skip(f"{spec['real']} does not publish per-axis speeds")
    for name, want in zip(arm.actuated, published):
        got = math.degrees(arm.velocity_limit(name))
        if filename == "ur_style_6dof.urdf" and name in ("j4", "j5", "j6"):
            # KNOWN GAP: the example runs the wrist at 360 deg/s, but the UR5e
            # datasheet rates every axis at 180 deg/s. Left as-is so the
            # disagreement is recorded rather than quietly corrected here; fix
            # it in scripts/make_ur_style.py.
            assert got == pytest.approx(2 * want, rel=0.05), f"{filename}:{name}"
            continue
        assert got == pytest.approx(want, rel=0.05), f"{filename}:{name}"


def test_reach_is_within_tolerance(case):
    filename, arm, spec = case
    got_mm = 1000.0 * _max_horizontal_reach(arm, spec["reach_to"])
    want = spec["reach_mm"]
    if filename == "wam_style_7dof.urdf":
        # Barrett quotes reach to the mounting plate, which this model has a
        # tool tip for, so the two are directly comparable and agree closely.
        assert got_mm == pytest.approx(want, rel=0.03), filename
        return
    # UR and KUKA quote a rated working envelope rather than the geometric
    # maximum this measures, so only a loose bound is meaningful. The UR5e's
    # link lengths are checked exactly below instead.
    assert want <= got_mm <= 1.15 * want, (
        f"{filename}: geometric reach {got_mm:.0f} mm vs published {want:.0f} mm"
    )


def test_ur5e_link_lengths_match_the_datasheet():
    """The UR5e's DH parameters are published, so this is an exact check --
    the strongest validation available for any of the three."""
    spec = REFERENCE["ur_style_6dof.urdf"]
    arm = load_arm(EXAMPLES / "robots" / "ur_style_6dof.urdf")
    want = spec["link_lengths_mm"]

    origin = lambda n: np.asarray(arm.joints[n].T_origin)[:3, 3] * 1000.0
    # Each DH length appears as one component of a joint origin: the base
    # height at j1, the upper arm and forearm along +Z at j3 and j4, and the
    # three wrist offsets at j5, j6 and the tool tip.
    assert origin("j1")[2] == pytest.approx(want["d1"], abs=0.5)
    assert origin("j3")[2] == pytest.approx(want["a2"], abs=0.5)
    assert origin("j4")[2] == pytest.approx(want["a3"], abs=0.5)
    assert origin("j5")[1] == pytest.approx(want["d4"], abs=0.5)
    assert origin("j6")[2] == pytest.approx(want["d5"], abs=0.5)

    tips = [t for l in arm.links.values() for t in l.tool_tips]
    assert len(tips) == 1
    assert np.max(np.abs(np.asarray(tips[0]["T"])[:3, 3] * 1000.0)) == pytest.approx(
        want["d6"], abs=0.5
    )

    # The shoulder offset is carried at j2 and taken back at j3, so it does not
    # lengthen the arm; the two must cancel.
    assert origin("j2")[1] == pytest.approx(-origin("j3")[1], abs=1e-6)


def test_wam_joint_torque_limits_are_the_right_size():
    """Barrett publishes the 7-DOF arm's joint torque limits. The example is a
    different drive design in detail, so this checks the order of magnitude and
    the shape of the profile, not the individual numbers."""
    spec = REFERENCE["wam_style_7dof.urdf"]
    arm = load_arm(EXAMPLES / "robots" / "wam_style_7dof.urdf")
    published = spec["joint_torque_nm"]
    got = [arm.effort_limit(n) for n in arm.actuated]
    assert all(v is not None for v in got)
    for name, g, w in zip(arm.actuated, got, published):
        assert 0.2 * w <= g <= 5.0 * w, f"{name}: {g:.1f} vs published {w} N·m"
    # Inner joints must be rated well above the wrist, as on the real arm.
    assert min(got[:3]) > 3 * max(got[4:])
