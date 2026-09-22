"""The BLDC motor-mass model and its use on an arm."""

import math

import pytest

from arm_analyzer import motor_mass
from arm_analyzer.analysis import analyze
from arm_analyzer.robot import load_arm, parse_arm

from conftest import EXAMPLES, PROJECT_ROOT, one_joint_urdf

EXAMPLE_ROBOTS = [p.name for p in sorted((EXAMPLES / "robots").glob("*.urdf"))]


def test_matches_the_reference_fit():
    """Values reproduced from the per-form fit over bldc_motor_data.csv."""
    assert motor_mass.estimate_mass(1.0, "outrunner") == pytest.approx(0.172253, abs=5e-6)
    assert motor_mass.estimate_mass(1.0, "frameless") == pytest.approx(0.196990, abs=5e-6)
    assert motor_mass.estimate_mass(1.0, "industrial") == pytest.approx(0.582293, abs=5e-6)
    assert motor_mass.estimate_mass(1.0, "integrated") == pytest.approx(0.651013, abs=5e-6)


def test_it_predicts_a_real_small_motor():
    """The Faulhaber 1226S024B: 0.0102 N*m, 13.0 g. The small end of the set is
    what the enlarged dataset bought, so check the model actually lands there."""
    got = motor_mass.estimate_mass(0.0102, "inrunner")
    assert got == pytest.approx(0.013, rel=0.15)


def test_mass_grows_slower_than_torque():
    """b < 1, so doubling the rating costs 2^0.70 ~ 1.62x the mass, not 2x."""
    one = motor_mass.estimate_mass(4.0, "frameless")
    two = motor_mass.estimate_mass(8.0, "frameless")
    assert two / one == pytest.approx(2.0 ** motor_mass.B_TORQUE, rel=1e-9)
    assert 1.5 < two / one < 1.8


def test_form_moves_the_mass_more_than_the_rating():
    """The dataset's main result: construction dominates. An integrated servo
    is several times an outrunner at the same torque -- a bigger spread than
    doubling the rating produces."""
    at_same_torque = motor_mass.estimate_mass(2.0, "integrated") / motor_mass.estimate_mass(
        2.0, "outrunner"
    )
    doubling = 2.0 ** motor_mass.B_TORQUE
    assert at_same_torque > 3.5
    assert at_same_torque > doubling


def test_an_undeclared_form_is_frameless():
    assert motor_mass.normalize_form(None) == "frameless"
    assert motor_mass.estimate_mass(2.0) == pytest.approx(
        motor_mass.estimate_mass(2.0, "frameless")
    )


@pytest.mark.parametrize("bad", ["brushless", "pancake", "OUTRUNNERS"])
def test_an_unknown_form_is_refused(bad):
    with pytest.raises(ValueError, match="unknown motor form"):
        motor_mass.normalize_form(bad)


def test_form_is_case_insensitive():
    assert motor_mass.normalize_form("Frameless") == "frameless"
    assert motor_mass.normalize_form(" INDUSTRIAL ") == "industrial"


def test_a_motor_needs_a_torque_rating():
    with pytest.raises(ValueError, match="peak_torque must be positive"):
        motor_mass.estimate_mass(0.0, "frameless")


@pytest.mark.parametrize(
    "torque, form, expect",
    [
        (5.0, "frameless", None),
        (0.5, "inrunner", None),
        # Each form covers its own slice, so the same torque can be inside one
        # form's range and outside another's.
        (0.01, "frameless", "below the frameless range"),
        (0.01, "inrunner", None),
        (500.0, "frameless", "above the frameless range"),
        (5.0, "hub", "below the hub range"),
    ],
)
def test_extrapolation_is_flagged_per_form(torque, form, expect):
    note = motor_mass.out_of_range(torque, form)
    assert (expect is None and note is None) or (expect and expect in note)


def test_every_form_has_a_range_and_a_density():
    assert set(motor_mass.TORQUE_RANGE) == set(motor_mass.A)
    for form in motor_mass.FORMS:
        lo, hi = motor_mass.TORQUE_RANGE[form]
        assert 0 < lo < hi
        assert 1500 < motor_mass.bulk_density(form) < 9000


def test_forms_without_dimension_data_fall_back_to_the_overall_density():
    """Only four forms have enough rows listing OD and length."""
    assert motor_mass.bulk_density("industrial") == motor_mass.OVERALL_DENSITY
    assert motor_mass.bulk_density("outrunner") != motor_mass.OVERALL_DENSITY


# ---------------------------------------------------------------------------
# On an arm
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def kr():
    return load_arm(EXAMPLES / "robots" / "kr_style_6dof.urdf")


def test_urdf_declares_the_motor_form(kr):
    """The KR example is built from housed industrial servos."""
    assert all(d.motor_spec.form == "industrial" for d in kr.drives.values())


def test_an_untyped_motor_is_frameless():
    arm = parse_arm(one_joint_urdf())
    assert arm.drives["shoulder"].motor_spec.form == "frameless"


def test_a_bad_form_in_the_urdf_is_refused():
    urdf = one_joint_urdf().replace('<motor link="arm"', '<motor form="pancake" link="arm"')
    with pytest.raises(ValueError, match="unknown motor form"):
        parse_arm(urdf)


def test_estimates_report_declared_and_predicted_side_by_side(kr):
    rows = kr.motor_mass_estimates()
    assert set(rows) == set(kr.drives)
    for name, r in rows.items():
        ms = kr.drives[name].motor_spec
        assert r["declared"] == kr.drives[name].motor.mass
        assert r["peak_torque"] == ms.peak_torque
        assert r["form"] == ms.form
        assert r["estimated"] == pytest.approx(
            motor_mass.estimate_mass(ms.peak_torque, ms.form)
        )
        assert r["ratio"] == pytest.approx(r["estimated"] / r["declared"])


def test_a_motor_without_a_rating_is_reported_not_guessed():
    arm = parse_arm(one_joint_urdf(extra_motor_attrs=""))
    row = arm.motor_mass_estimates()["shoulder"]
    assert row["estimated"] is None
    assert "no peak_torque" in row["note"]
    applied = arm.with_estimated_motor_masses()
    assert applied.drives["shoulder"].motor.mass == arm.drives["shoulder"].motor.mass


def test_applying_the_model_rewrites_motor_masses_only(kr):
    before = kr.mass_budget()
    applied = kr.with_estimated_motor_masses()
    after = applied.mass_budget()
    assert after["structure"] == before["structure"]
    assert after["gearboxes"] == before["gearboxes"]
    assert after["motors"] == pytest.approx(
        sum(r["estimated"] for r in kr.motor_mass_estimates().values())
    )
    assert kr.mass_budget() == before


def test_applying_the_model_scales_lump_inertia_with_mass(kr):
    applied = kr.with_estimated_motor_masses()
    for name, drive in kr.drives.items():
        new = applied.drives[name].motor
        scale = new.mass / drive.motor.mass
        assert new.inertia == pytest.approx(drive.motor.inertia * scale)
        # Rotor inertia is a spec, not a property of the housing: untouched.
        assert applied.drives[name].motor_spec.rotor_inertia == drive.motor_spec.rotor_inertia


def test_applied_arm_keeps_its_couplings_pointing_at_the_same_drives():
    wam = load_arm(EXAMPLES / "robots" / "wam_style_7dof.urdf")
    applied = wam.with_estimated_motor_masses()
    assert applied.couplings
    for c in applied.couplings:
        for joint, drive in zip(c.joints, c.drives):
            assert drive is applied.drives[joint]


# ---------------------------------------------------------------------------
# Generated examples carry no contradiction
# ---------------------------------------------------------------------------


def _bulk_density_const():
    import sys

    scripts = str(PROJECT_ROOT / "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    from urdf_parts import motor_body_for  # noqa: F401  (import check only)

    return None


@pytest.mark.parametrize("filename", EXAMPLE_ROBOTS)
def test_examples_declare_no_mass_that_contradicts_their_torque(filename):
    """Torque and form are the defined parameters in the example arms and mass
    follows, so ``<mass>`` cannot drift from ``peak_torque``. Regenerate with
    ``pixi run examples`` if this fails."""
    arm = load_arm(EXAMPLES / "robots" / filename)
    assert arm.drives, f"{filename} has no drives"
    for joint, row in arm.motor_mass_estimates().items():
        assert row["estimated"] is not None, f"{filename}:{joint} declares no peak_torque"
        assert row["ratio"] == pytest.approx(1.0, rel=1e-5), f"{filename}:{joint}"


@pytest.mark.parametrize("filename", EXAMPLE_ROBOTS)
def test_example_motors_have_the_density_of_their_form(filename):
    """Mass follows the rating and the envelope is then sized to hold it, at
    the bulk density measured for that construction.

    Not only cosmetic: a lump with no ``<inertia>`` takes its tensor from its
    geometry scaled to its mass, so an oversized envelope would give the motor
    an inertia it does not have."""
    arm = load_arm(EXAMPLES / "robots" / filename)
    for joint, drive in arm.drives.items():
        shape = drive.motor.shape
        assert shape.type == "cylinder", f"{filename}:{joint}"
        radius, length = shape.params[0], shape.params[1]
        density = drive.motor.mass / (math.pi * radius * radius * length)
        assert density == pytest.approx(
            motor_mass.bulk_density(drive.motor_spec.form), rel=1e-4
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
    mm = r["motor_mass"]
    assert mm["mode"] == "declared"
    assert mm["applied_total"] == pytest.approx(mm["declared_total"])
    assert mm["estimated_total"] == pytest.approx(mm["declared_total"], rel=1e-5)
    assert set(mm["exponents"]) == {"torque"}, "the speed term is gone"


def test_model_mode_changes_the_mass_and_the_torques():
    """On an arm whose declared mass disagrees with its rating."""
    from arm_analyzer.profile import PolySegment, TrajectoryPlan

    arm = parse_arm(one_joint_urdf(motor_xyz="0.8 0 0"))
    declared = arm.drives["shoulder"].motor.mass
    modelled = arm.motor_mass_estimates()["shoulder"]["estimated"]
    assert modelled < 0.7 * declared, "fixture no longer exercises a difference"

    plan = TrajectoryPlan(joints=list(arm.actuated))
    plan.segments.append(PolySegment(0.0, 0.2, {n: (0.0, 0.3, 0.5, 0.0) for n in arm.actuated}))
    plan.waypoint_times = [0.0, 0.2]

    a = analyze(arm, plan, rate_hz=50)
    b = analyze(arm, plan, rate_hz=50, motor_mass_mode="model")
    assert b["mass_budget"]["motors"] == pytest.approx(modelled)
    assert b["moving_mass"] < a["moving_mass"]
    peak = lambda r: r["summary"][0]["joint"]["peak_torque"]
    assert peak(b) < peak(a)


def test_an_unknown_mode_is_refused(kr, kr_plan):
    with pytest.raises(ValueError, match="motor_mass_mode must be one of"):
        analyze(kr, kr_plan, rate_hz=50, motor_mass_mode="guess")
