"""The reducer mass model and its use on an arm."""

import math

import pytest

from arm_analyzer import gearbox_mass
from arm_analyzer.analysis import analyze
from arm_analyzer.robot import load_arm, parse_arm

from conftest import EXAMPLES, one_joint_urdf

EXAMPLE_ROBOTS = [p.name for p in sorted((EXAMPLES / "robots").glob("*.urdf"))]


def test_matches_the_reference_fit():
    """Values reproduced from robot_arm_data/analyze_gearbox_mass.py."""
    assert gearbox_mass.estimate_mass(40, 50, "harmonic") == pytest.approx(1.434, abs=5e-4)
    assert gearbox_mass.estimate_mass(40, 50, "planetary") == pytest.approx(1.245, abs=5e-4)
    assert gearbox_mass.estimate_mass(40, 50, "cycloidal") == pytest.approx(1.824, abs=5e-4)
    assert gearbox_mass.estimate_mass(40, 50, "worm") == pytest.approx(2.652, abs=5e-4)
    assert gearbox_mass.estimate_mass(40, 50, "spur") == pytest.approx(2.675, abs=5e-4)


def test_ratio_barely_changes_the_mass():
    """The dataset's headline result: within a frame, a reducer weighs the
    same at 30:1 as at 100:1 -- the frame sets the mass, not the gearing."""
    at_30 = gearbox_mass.estimate_mass(4.0, 30, "harmonic")
    at_100 = gearbox_mass.estimate_mass(4.0, 100, "harmonic")
    assert at_100 == pytest.approx(at_30, rel=0.02)
    assert abs(gearbox_mass.C_RATIO) < 0.02


def test_mass_grows_slower_than_rated_torque():
    one = gearbox_mass.estimate_mass(20, 50, "harmonic")
    two = gearbox_mass.estimate_mass(40, 50, "harmonic")
    assert two / one == pytest.approx(2.0 ** gearbox_mass.B_TORQUE, rel=1e-9)
    assert 1.5 < two / one < 1.8


def test_type_changes_the_mass_substantially():
    """A worm box is about twice a planetary at the same rating."""
    worm = gearbox_mass.estimate_mass(40, 50, "worm")
    planetary = gearbox_mass.estimate_mass(40, 50, "planetary")
    assert worm / planetary > 1.8


def test_an_undeclared_type_is_harmonic():
    assert gearbox_mass.normalize_type(None) == "harmonic"
    assert gearbox_mass.estimate_mass(40, 50) == pytest.approx(
        gearbox_mass.estimate_mass(40, 50, "harmonic")
    )


@pytest.mark.parametrize("bad", ["cyclo", "belt", "HARMONICA"])
def test_an_unknown_type_is_refused(bad):
    with pytest.raises(ValueError, match="unknown gearbox type"):
        gearbox_mass.normalize_type(bad)


def test_type_is_case_insensitive():
    assert gearbox_mass.normalize_type("Harmonic") == "harmonic"
    assert gearbox_mass.normalize_type(" WORM ") == "worm"


@pytest.mark.parametrize("torque, ratio", [(0.0, 50), (-1.0, 50)])
def test_a_gearbox_needs_a_rating(torque, ratio):
    with pytest.raises(ValueError, match="rated_torque must be positive"):
        gearbox_mass.estimate_mass(torque, ratio, "harmonic")


def test_a_gearbox_needs_a_ratio():
    with pytest.raises(ValueError, match="ratio must be positive"):
        gearbox_mass.estimate_mass(40, 0, "harmonic")


@pytest.mark.parametrize(
    "torque, ratio, expect",
    [
        (40, 50, None),
        (0.1, 50, "below the fitted range"),
        (2000, 50, "above the fitted range"),
        (40, 200, "outside the fitted range"),
    ],
)
def test_extrapolation_is_flagged(torque, ratio, expect):
    note = gearbox_mass.out_of_range(torque, ratio, "harmonic")
    assert (expect is None and note is None) or (expect and expect in note)


def test_every_type_has_a_density():
    assert set(gearbox_mass.BULK_DENSITY) == set(gearbox_mass.A)
    for kind in gearbox_mass.TYPES:
        assert 1500 < gearbox_mass.bulk_density(kind) < 9000


# ---------------------------------------------------------------------------
# On an arm
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def kr():
    return load_arm(EXAMPLES / "robots" / "kr_style_6dof.urdf")


def test_urdf_declares_the_gearbox_type():
    arm = parse_arm(one_joint_urdf().replace('ratio="50.0"', 'ratio="50.0" type="worm"'))
    assert arm.drives["shoulder"].gearbox_spec.kind == "worm"


def test_an_untyped_gearbox_is_harmonic(kr):
    assert all(d.gearbox_spec.kind == "harmonic" for d in kr.drives.values())


def test_a_bad_type_in_the_urdf_is_refused():
    with pytest.raises(ValueError, match="unknown gearbox type"):
        parse_arm(one_joint_urdf().replace('ratio="50.0"', 'ratio="50.0" type="belt"'))


def test_estimates_report_declared_and_predicted_side_by_side(kr):
    rows = kr.gearbox_mass_estimates()
    assert set(rows) == set(kr.drives)
    for name, r in rows.items():
        gs = kr.drives[name].gearbox_spec
        assert r["declared"] == kr.drives[name].gearbox.mass
        assert r["rated_torque"] == gs.rated_torque
        assert r["ratio"] == gs.ratio
        assert r["estimated"] == pytest.approx(
            gearbox_mass.estimate_mass(gs.rated_torque, gs.ratio, gs.kind)
        )


def test_applying_the_model_rewrites_gearbox_masses_only(kr):
    before = kr.mass_budget()
    applied = kr.with_estimated_gearbox_masses()
    after = applied.mass_budget()
    assert after["structure"] == before["structure"]
    assert after["motors"] == before["motors"]
    assert after["gearboxes"] == pytest.approx(
        sum(r["estimated"] for r in kr.gearbox_mass_estimates().values())
    )
    # Input inertia is a spec, not a property of the housing: untouched.
    for name, drive in kr.drives.items():
        assert (
            applied.drives[name].gearbox_spec.input_inertia
            == drive.gearbox_spec.input_inertia
        )
    assert kr.mass_budget() == before  # original untouched


def test_a_gearbox_without_a_rating_keeps_its_declared_mass():
    urdf = one_joint_urdf().replace('peak_torque="60" rated_torque="25"', "")
    arm = parse_arm(urdf)
    row = arm.gearbox_mass_estimates()["shoulder"]
    assert row["estimated"] is None and "no rated_torque" in row["note"]
    applied = arm.with_estimated_gearbox_masses()
    assert applied.drives["shoulder"].gearbox.mass == arm.drives["shoulder"].gearbox.mass


# ---------------------------------------------------------------------------
# Generated examples carry no contradiction
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("filename", EXAMPLE_ROBOTS)
def test_examples_declare_no_gearbox_mass_that_contradicts_its_rating(filename):
    """As with the motors, the rating is the defined parameter and the mass
    follows. Regenerate with ``pixi run examples`` if this fails."""
    arm = load_arm(EXAMPLES / "robots" / filename)
    for joint, row in arm.gearbox_mass_estimates().items():
        assert row["estimated"] is not None, f"{filename}:{joint} declares no rated_torque"
        assert row["mass_ratio"] == pytest.approx(1.0, rel=1e-5), f"{filename}:{joint}"


@pytest.mark.parametrize("filename", EXAMPLE_ROBOTS)
def test_example_gearboxes_have_the_density_of_a_real_one(filename):
    arm = load_arm(EXAMPLES / "robots" / filename)
    for joint, drive in arm.drives.items():
        shape = drive.gearbox.shape
        assert shape.type == "cylinder", f"{filename}:{joint}"
        radius, length = shape.params[0], shape.params[1]
        density = drive.gearbox.mass / (math.pi * radius * radius * length)
        assert density == pytest.approx(
            gearbox_mass.bulk_density(drive.gearbox_spec.kind), rel=1e-4
        ), f"{filename}:{joint}"


# ---------------------------------------------------------------------------
# Through the analysis
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def kr_plan(kr):
    from arm_analyzer.trajectory import load_trajectory

    text = (EXAMPLES / "trajectories" / "kr_pick_and_place.json").read_text(encoding="utf-8")
    return load_trajectory(kr, text)


def test_declared_mode_is_the_default(kr, kr_plan):
    r = analyze(kr, kr_plan, rate_hz=50)
    gm = r["gearbox_mass"]
    assert gm["mode"] == "declared"
    assert gm["applied_total"] == pytest.approx(gm["declared_total"])
    # Generated examples agree either way.
    assert gm["estimated_total"] == pytest.approx(gm["declared_total"], rel=1e-5)


GEARBOX_ON_THE_ARM = """<?xml version="1.0"?>
<robot name="one_joint">
  <link name="base">
    <inertial><origin xyz="0 0 0"/><mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>
  </link>
  <link name="arm">
    <inertial><origin xyz="0.5 0 0"/><mass value="2"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <tool_tip name="tip" xyz="1 0 0"/>
  </link>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="arm"/>
    <origin xyz="0 0 1"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" velocity="2" effort="100"/>
    <drive>
      <motor link="base" peak_torque="1.0" stall_torque="2.0" no_load_speed="500">
        <origin xyz="0 0 0"/><mass value="0.5"/>
      </motor>
      <!-- Out at the tip, so its mass shows up in the shoulder torque, and
           declared far lighter than its rating implies. -->
      <gearbox link="arm" ratio="50" peak_torque="60" rated_torque="25">
        <origin xyz="0.8 0 0"/><mass value="0.3"/>
      </gearbox>
    </drive>
  </joint>
</robot>
"""


def test_model_mode_changes_the_mass_and_the_torques():
    """On an arm whose declared gearbox mass disagrees with its rating."""
    from arm_analyzer.profile import PolySegment, TrajectoryPlan

    arm = parse_arm(GEARBOX_ON_THE_ARM)
    declared_mass = arm.drives["shoulder"].gearbox.mass
    modelled = arm.gearbox_mass_estimates()["shoulder"]["estimated"]
    # A 25 N*m harmonic drive is a ~1 kg part; this one is declared at 0.3 kg.
    assert modelled > 2 * declared_mass, "fixture no longer exercises a difference"

    plan = TrajectoryPlan(joints=list(arm.actuated))
    plan.segments.append(PolySegment(0.0, 0.2, {n: (0.0, 0.3, 0.5, 0.0) for n in arm.actuated}))
    plan.waypoint_times = [0.0, 0.2]

    a = analyze(arm, plan, rate_hz=50)
    b = analyze(arm, plan, rate_hz=50, gearbox_mass_mode="model")
    assert b["mass_budget"]["gearboxes"] == pytest.approx(modelled)
    # A heavier reducer out at the tip costs the shoulder more torque.
    assert b["moving_mass"] > a["moving_mass"]
    peak = lambda r: r["summary"][0]["joint"]["peak_torque"]
    assert peak(b) > peak(a)


def test_an_unknown_mode_is_refused(kr, kr_plan):
    with pytest.raises(ValueError, match="gearbox_mass_mode must be one of"):
        analyze(kr, kr_plan, rate_hz=50, gearbox_mass_mode="guess")
