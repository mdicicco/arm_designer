"""Link mass derived from geometry: the tube, the collars, and the wiring."""

import math

import numpy as np
import pytest

from arm_analyzer.analysis import analyze
from arm_analyzer.link_mass import LinkMassParams, derive, link_span
from arm_analyzer.pin_model import build_model
from arm_analyzer.robot import load_arm

from conftest import EXAMPLES

EXAMPLE_ROBOTS = [p.name for p in sorted((EXAMPLES / "robots").glob("*.urdf"))]


@pytest.fixture(scope="module")
def kr():
    return load_arm(EXAMPLES / "robots" / "kr_style_6dof.urdf")


@pytest.fixture(scope="module")
def kr_plan(kr):
    from arm_analyzer.trajectory import load_trajectory

    text = (EXAMPLES / "trajectories" / "kr_pick_and_place.json").read_text(encoding="utf-8")
    return load_trajectory(kr, text)


# ---------------------------------------------------------------------------
# Span: the length comes out of the URDF, not out of a parameter
# ---------------------------------------------------------------------------


def test_span_is_the_offset_to_the_next_joint(kr):
    """A link's frame sits at its inbound joint, so its length is the origin
    of the joint that carries the next link."""
    for joint in kr.joints.values():
        length, axis = link_span(kr, joint.parent)
        offset = np.asarray(joint.T_origin)[:3, 3]
        if np.linalg.norm(offset) > 0:
            assert length >= np.linalg.norm(offset) - 1e-9
        assert np.linalg.norm(axis) == pytest.approx(1.0)


def test_a_tip_link_spans_to_its_tool_tip(kr):
    tip = kr.order[-1]
    assert not any(j.parent == tip for j in kr.joints.values())
    tips = kr.links[tip].tool_tips
    assert tips, "the KR's last link should carry a tool tip"
    length, _ = link_span(kr, tip)
    assert length == pytest.approx(
        max(float(np.linalg.norm(np.asarray(t["T"])[:3, 3])) for t in tips)
    )


@pytest.mark.parametrize("filename", EXAMPLE_ROBOTS)
def test_every_example_link_gets_a_positive_span(filename):
    arm = load_arm(EXAMPLES / "robots" / filename)
    rows = derive(arm, LinkMassParams())
    assert set(rows) == set(arm.links)
    for name, row in rows.items():
        assert row["length"] > 0, name
        assert row["mass"] > 0, name


# ---------------------------------------------------------------------------
# The tube
# ---------------------------------------------------------------------------


def test_diameter_tapers_from_base_to_tip(kr):
    p = LinkMassParams(baseline_diameter=0.12, taper=0.4)
    rows = derive(kr, p)
    chain = [kr.root] + list(kr.order)
    diameters = [rows[n]["outer_diameter"] for n in chain]
    assert diameters[0] == pytest.approx(p.baseline_diameter)
    assert diameters[-1] == pytest.approx(p.baseline_diameter * p.taper)
    assert all(a > b for a, b in zip(diameters, diameters[1:])), "should shrink monotonically"


def test_taper_of_one_is_a_constant_diameter_arm(kr):
    rows = derive(kr, LinkMassParams(taper=1.0, baseline_diameter=0.1))
    assert {round(r["outer_diameter"], 9) for r in rows.values()} == {0.1}


def test_tube_mass_is_the_annulus_times_density(kr):
    p = LinkMassParams(baseline_diameter=0.1, taper=0.6, wall=0.005, density=2700)
    for name, row in derive(kr, p).items():
        outer = row["outer_diameter"] / 2
        inner = row["inner_diameter"] / 2
        volume = math.pi * (outer**2 - inner**2) * row["length"]
        assert row["tube_mass"] == pytest.approx(volume * p.density), name


def test_a_thick_wall_makes_the_link_solid(kr):
    rows = derive(kr, LinkMassParams(baseline_diameter=0.08, wall=0.05))
    for name, row in rows.items():
        assert row["solid"], name
        assert row["inner_diameter"] == 0.0


def test_mass_scales_with_density(kr):
    a = derive(kr, LinkMassParams(density=2700, actuator_mass=0, actuator_fraction=0))
    b = derive(kr, LinkMassParams(density=5400, actuator_mass=0, actuator_fraction=0))
    for name in a:
        assert b[name]["mass"] == pytest.approx(2.0 * a[name]["mass"]), name


# ---------------------------------------------------------------------------
# The collars
# ---------------------------------------------------------------------------


def test_a_collar_is_added_for_every_mounted_actuator(kr):
    p = LinkMassParams(actuator_mass=0.2, actuator_fraction=0.5)
    rows = derive(kr, p)
    mounted: dict[str, list] = {n: [] for n in kr.links}
    for drive in kr.drives.values():
        for lump in (drive.motor, drive.gearbox):
            mounted[lump.link].append(lump)
    for name, row in rows.items():
        expected = sum(p.actuator_mass + p.actuator_fraction * l.mass for l in mounted[name])
        assert row["collar_mass"] == pytest.approx(expected), name
        assert len(row["collars"]) == len(mounted[name]), name
        assert row["mass"] == pytest.approx(row["tube_mass"] + row["collar_mass"]), name


def test_collars_follow_the_actuator_not_the_joint(kr):
    """The KR's wrist motors sit on link3, behind the elbow, so link3 carries
    their collars even though they drive joints further out."""
    rows = derive(kr, LinkMassParams(actuator_mass=0.5, actuator_fraction=0.0))
    names = [c["lump"] for c in rows["link3"]["collars"]]
    for joint in ("j4", "j5", "j6"):
        assert f"{joint}_motor" in names


def test_zero_actuator_terms_leave_only_the_tube(kr):
    rows = derive(kr, LinkMassParams(actuator_mass=0.0, actuator_fraction=0.0))
    for name, row in rows.items():
        assert row["collar_mass"] == 0.0
        assert row["mass"] == pytest.approx(row["tube_mass"]), name


# ---------------------------------------------------------------------------
# Reaching the dynamics
# ---------------------------------------------------------------------------


def test_overriding_with_the_declared_inertias_changes_nothing(kr):
    """The override path has to be exact, or every derived number would carry
    a silent transform error."""
    plain = build_model(kr)
    same = build_model(kr, link_inertias={n: l.inertial for n, l in kr.links.items()})
    for i in range(plain.njoints):
        assert np.allclose(
            plain.inertias[i].toDynamicParameters(),
            same.inertias[i].toDynamicParameters(),
            atol=1e-12,
        )


def test_derived_links_reach_the_torques_not_just_the_budget(kr, kr_plan):
    """A mass that moved the budget but not the model would be worse than
    useless -- the plots would describe a different robot."""
    light = LinkMassParams(
        baseline_diameter=0.06, wall=0.002, actuator_mass=0.0, actuator_fraction=0.0
    )
    heavy = LinkMassParams(
        baseline_diameter=0.2, wall=0.03, actuator_mass=1.0, actuator_fraction=1.0
    )
    a = analyze(kr, kr_plan, rate_hz=50, link_mass_mode="derived", link_mass_params=light)
    b = analyze(kr, kr_plan, rate_hz=50, link_mass_mode="derived", link_mass_params=heavy)

    assert b["mass_budget"]["structure"] > 3 * a["mass_budget"]["structure"]
    assert b["moving_mass"] > a["moving_mass"]
    peak = lambda r, n: next(s for s in r["summary"] if s["name"] == n)["joint"]["peak_torque"]
    assert peak(b, "j2") > peak(a, "j2")


def test_declared_mode_reports_the_derived_number_without_using_it(kr, kr_plan):
    r = analyze(kr, kr_plan, rate_hz=50)
    lm = r["link_mass"]
    assert lm["mode"] == "declared"
    assert r["mass_budget"]["structure"] == pytest.approx(lm["declared_total"])
    assert lm["derived_total"] != pytest.approx(lm["declared_total"])
    assert lm["ratio"] == pytest.approx(lm["derived_total"] / lm["declared_total"])
    # The sliders' starting positions come back so the browser need not guess.
    assert set(lm["params"]) == set(LinkMassParams.LIMITS)
    assert "properties" not in next(iter(lm["links"].values())), "not JSON-serialisable"


def test_derived_mode_uses_it(kr, kr_plan):
    r = analyze(kr, kr_plan, rate_hz=50, link_mass_mode="derived")
    lm = r["link_mass"]
    assert r["mass_budget"]["structure"] == pytest.approx(lm["derived_total"])
    assert r["mass_budget"]["structure"] != pytest.approx(lm["declared_total"])


def test_link_mass_params_are_range_checked():
    for field, (lo, hi) in LinkMassParams.LIMITS.items():
        for bad in (lo - abs(lo) - 1.0, hi * 10 + 1.0):
            p = LinkMassParams(**{field: bad})
            with pytest.raises(ValueError, match=f"link mass {field}"):
                p.validated()


def test_an_unknown_link_mass_mode_is_refused(kr, kr_plan):
    with pytest.raises(ValueError, match="link_mass_mode must be one of"):
        analyze(kr, kr_plan, rate_hz=50, link_mass_mode="guess")
