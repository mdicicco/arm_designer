import math

import numpy as np
import pytest

from arm_analyzer.robot import COLOCATED_TOLERANCE_M, RPM_TO_RAD_S, parse_arm


def test_example_arm_structure(simple_arm):
    assert simple_arm.root == "base_link"
    assert simple_arm.actuated == ["j1", "j2", "j3", "j4", "j5", "j6"]
    assert simple_arm.warnings == []
    # Six link bodies plus a motor and a gearbox for every joint.
    assert len(simple_arm.links) == 7
    assert len(simple_arm.drives) == 6
    assert len(simple_arm.lumps()) == 12
    assert all(d.colocated for d in simple_arm.drives.values())


def test_example_drives_are_on_the_parent_link(simple_arm):
    for name, d in simple_arm.drives.items():
        parent = simple_arm.joints[name].parent
        assert d.motor.link == parent
        assert d.gearbox.link == parent
        assert max(d.motor_offset, d.gearbox_offset) <= COLOCATED_TOLERANCE_M


def test_mass_budget_adds_up(simple_arm):
    b = simple_arm.mass_budget()
    assert b["total"] == pytest.approx(b["structure"] + b["motors"] + b["gearboxes"])
    lump_mass = sum(l.mass for l in simple_arm.lumps())
    assert lump_mass == pytest.approx(b["motors"] + b["gearboxes"])


def test_lump_inertia_from_geometry(simple_arm):
    m = simple_arm.drives["j1"].motor
    r, h = m.shape.params
    expected = np.diag([(3 * r * r + h * h) / 12, (3 * r * r + h * h) / 12, r * r / 2]) * m.mass
    assert np.allclose(m.inertia, expected)


def test_drive_derived_limits(simple_arm):
    d = simple_arm.drives["j2"]
    assert d.total_ratio == 120
    # No <limit effort> in the example: effort comes from the drive at stall,
    # min(motor peak * N * efficiency, gearbox peak); the example's efficiencies are 1.
    assert simple_arm.effort_limit("j2") == pytest.approx(min(1.3 * 120, 180))
    # j3's 60 N*m gearbox is tighter than its motor (0.64 * 100).
    assert simple_arm.effort_limit("j3") == pytest.approx(60)
    assert d.reflected_inertia == pytest.approx((4.0e-5 + 5e-6) * 120**2)
    # Declared velocity (2.5) is tighter than the drive (628 / 120).
    assert simple_arm.velocity_limit("j2") == pytest.approx(2.5)


def test_motor_envelope_shape(simple_arm):
    ms = simple_arm.drives["j1"].motor_spec
    w = np.array([0.0, 100.0, 400.0, 628.0, 700.0])
    peak = ms.torque_limit(w)
    assert peak[0] == pytest.approx(1.3)  # current-limited plateau
    assert peak[2] == pytest.approx(3.2 * (1 - 400 / 628))  # voltage line
    assert peak[3] == pytest.approx(0.0)
    assert peak[4] == 0.0
    cont = ms.torque_limit(w, continuous=True)
    assert np.all(cont <= peak + 1e-12)


def test_remote_drive_detected(one_joint):
    arm = parse_arm(one_joint(motor_link="base", motor_xyz="0 0 0.1"))
    d = arm.drives["shoulder"]
    # Base is the joint's parent, 0.9 m below the joint origin.
    assert d.motor_offset == pytest.approx(0.9)
    assert not d.colocated


def test_transmission_multiplies_ratio(one_joint):
    arm = parse_arm(one_joint(ratio=50, transmission='<transmission ratio="3"/>'))
    d = arm.drives["shoulder"]
    assert d.total_ratio == 150
    # Gearbox output rating (60) is carried through the transmission.
    assert float(d.joint_torque_limit(0.0)) == pytest.approx(min(1.0 * 150, 60 * 3))


def test_efficiencies_default_to_lossless(one_joint):
    d = parse_arm(one_joint()).drives["shoulder"]
    assert d.gearbox_spec.efficiency == 1.0
    assert d.transmission_efficiency == 1.0
    assert d.to_dict()["total_efficiency"] == 1.0


def test_efficiencies_scale_the_available_torque(one_joint):
    xml = one_joint(transmission='<transmission ratio="2" efficiency="0.9"/>').replace(
        'ratio="50.0"', 'ratio="50.0" efficiency="0.8"'
    )
    d = parse_arm(xml).drives["shoulder"]
    assert d.total_efficiency == pytest.approx(0.72)
    # min(motor 1.0 * 100 * 0.72, gearbox 60 * 2 * 0.9)
    assert float(d.joint_torque_limit(0.0)) == pytest.approx(min(72.0, 108.0))
    d.set_efficiency(gearbox=1.0, transmission=0.5)
    assert d.total_efficiency == pytest.approx(0.5)
    with pytest.raises(ValueError, match="efficiency"):
        d.set_efficiency(gearbox=1.1)


def test_joint_friction_is_ignored_with_a_warning(one_joint):
    arm = parse_arm(one_joint().replace("<drive>", '<dynamics damping="0.5" friction="1"/>\n    <drive>', 1))
    assert any("<dynamics> friction is not modelled" in w for w in arm.warnings)
    assert not hasattr(arm.joints["shoulder"], "friction")


def test_rpm_speeds(one_joint):
    arm = parse_arm(
        one_joint(extra_motor_attrs='peak_torque="1" no_load_speed_rpm="6000"')
    )
    assert arm.drives["shoulder"].motor_spec.no_load_speed == pytest.approx(6000 * RPM_TO_RAD_S)


def test_missing_drive_is_a_warning(one_joint):
    arm = parse_arm(one_joint(drive=False))
    assert arm.drives == {}
    assert any("no <drive>" in w for w in arm.warnings)


def test_missing_inertial_is_a_warning(one_joint):
    xml = one_joint().replace(
        '<inertial><origin xyz="0 0 0"/><mass value="5"/>\n      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>',
        "",
    )
    arm = parse_arm(xml)
    assert arm.links["base"].inertial.mass == 0
    assert any("base" in w and "<inertial>" in w for w in arm.warnings)


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda x: x.replace('motor link="arm"', 'motor link="nowhere"'), "unknown link"),
        (lambda x: x.replace('ratio="50.0"', 'ratio="50.0" efficiency="1.2"'), "efficiency"),
        (lambda x: x.replace('ratio="50.0"', 'ratio="50.0" efficiency="0"'), "efficiency"),
        (lambda x: x.replace('<mass value="0.5"/>', ""), "mass"),
        (lambda x: x.replace('ratio="50.0"', 'ratio="0"'), "ratio"),
        (lambda x: x.replace('type="revolute"', 'type="fixed"'), "fixed"),
        (lambda x: x.replace("<gearbox", "<gearbax").replace("</gearbox>", "</gearbax>"), "gearbox"),
        (lambda x: x.replace('<child link="arm"/>', '<child link="base"/>'), "not connected"),
        (lambda x: x.replace("<robot", "<robott").replace("</robot>", "</robott>"), "robot"),
        (lambda x: x[:-20], "invalid XML"),
    ],
)
def test_invalid_descriptions(one_joint, mutate, message):
    with pytest.raises(ValueError, match=message):
        parse_arm(mutate(one_joint()))


def test_model_payload_is_json_ready(simple_arm):
    import json

    payload = simple_arm.to_dict()
    json.dumps(payload, allow_nan=False)
    assert [d["joint"] for d in payload["drives"]] == simple_arm.actuated
    assert payload["links"][0]["name"] == "base_link"
    assert payload["links"][0]["visuals"][0]["color"] == pytest.approx([0.30, 0.33, 0.38, 1])
    tips = [t for l in payload["links"] for t in l["tool_tips"]]
    assert tips and tips[0]["name"] == "tcp"
    assert math.isfinite(payload["mass_budget"]["total"])


def test_remote_elbow_example_unloads_only_the_joints_behind_the_move(simple_arm):
    from arm_analyzer.analysis import analyze
    from arm_analyzer.robot import load_arm
    from arm_analyzer.trajectory import load_trajectory

    from conftest import EXAMPLES

    remote = load_arm(EXAMPLES / "robots" / "simple_6dof_remote_elbow.urdf")
    d = remote.drives["j3"]
    assert not d.colocated
    assert {d.motor.link, d.gearbox.link} == {"link1"}
    assert d.transmission_ratio == 1.0
    assert remote.effort_limit("j3") == simple_arm.effort_limit("j3")  # same drive
    assert remote.mass_budget()["total"] == pytest.approx(simple_arm.mass_budget()["total"])

    traj = (EXAMPLES / "trajectories" / "pick_and_place.json").read_text()

    def rms(arm, joint):
        r = analyze(arm, load_trajectory(arm, traj), rate_hz=100)
        return next(s for s in r["summary"] if s["name"] == joint)

    for joint in ("j1", "j2"):  # carried the moved mass before
        assert rms(remote, joint)["joint"]["rms_torque"] < rms(simple_arm, joint)["joint"]["rms_torque"]
    for joint in ("j3", "j4", "j5", "j6"):  # never carried it
        assert rms(remote, joint)["joint"]["rms_torque"] == pytest.approx(
            rms(simple_arm, joint)["joint"]["rms_torque"], rel=1e-12
        )


def test_normalized_urdf_is_accepted_by_pinocchio(one_joint):
    import pinocchio as pin

    from arm_analyzer.pin_model import normalized_urdf

    # No effort (derived from the drive), and a continuous joint.
    xml = one_joint().replace(' effort="100"', "").replace('type="revolute"', 'type="continuous"')
    arm = parse_arm(xml)
    text = normalized_urdf(arm.urdf)
    assert "<drive" not in text and "<tool_tip" not in text
    model = pin.buildModelFromXML(text)
    assert model.nq == 1  # a continuous joint stays a single angle
    assert arm.joints["shoulder"].lower == pytest.approx(-math.pi)


def test_kr_style_example_puts_wrist_motors_behind_the_elbow():
    from arm_analyzer.dynamics import DynamicsModel, gravity_torques
    from arm_analyzer.robot import load_arm
    from arm_analyzer.transforms import make_T

    from conftest import EXAMPLES

    path = EXAMPLES / "robots" / "kr_style_6dof.urdf"
    kr = load_arm(path)
    assert kr.warnings == []
    assert kr.mass_budget()["total"] == pytest.approx(51.2)
    for j in ("j1", "j2", "j3"):
        assert kr.drives[j].colocated
    for j in ("j4", "j5", "j6"):
        d = kr.drives[j]
        assert not d.colocated
        assert d.motor.link == "link3"
        assert d.motor.T[0, 3] < 0  # behind the elbow (A3 is the link3 origin)
    assert [kr.drives[j].gearbox.link for j in ("j4", "j5", "j6")] == ["link3", "link4", "link5"]

    # Counterbalance: the same motors moved forward to their joints load A3 more.
    fwd = load_arm(path)
    for j, host, x in (("j4", "link3", 0.03), ("j5", "link4", 0.20), ("j6", "link5", -0.01)):
        m = fwd.drives[j].motor
        m.link, m.T = host, make_T(m.T[:3, :3], np.array([x, 0, 0]))
    home = {}
    t_kr = gravity_torques(DynamicsModel.build(kr), home)["j3"]
    t_fwd = gravity_torques(DynamicsModel.build(fwd), home)["j3"]
    assert abs(t_kr) < abs(t_fwd) - 5.0


def test_kr_pick_and_place_keeps_the_tool_down(simple_arm):
    import json

    from arm_analyzer.pin_model import build_model, frame_placements
    from arm_analyzer.robot import load_arm

    from conftest import EXAMPLES

    kr = load_arm(EXAMPLES / "robots" / "kr_style_6dof.urdf")
    spec = json.loads((EXAMPLES / "trajectories" / "kr_pick_and_place.json").read_text())
    model = build_model(kr, with_drives=False)
    for w in spec["waypoints"]:
        q = np.radians(w["joints"])
        for n, v in zip(kr.actuated, q):
            assert kr.joints[n].lower <= v <= kr.joints[n].upper, (w["name"], n)
        _, frames = frame_placements(model, q)
        assert np.allclose(frames["link6"][:3, 0], [0, 0, -1], atol=1e-9), w["name"]
        assert frames["link6"][2, 3] > 0.2, w["name"]


# ---------------------------------------------------------------------------
# The remote-drive examples: WAM (base motors + differentials), palletizer
# (base motors + parallelogram linkages), UR (everything co-located).
# ---------------------------------------------------------------------------


def _example(name):
    from arm_analyzer.robot import load_arm

    from conftest import EXAMPLES

    return load_arm(EXAMPLES / "robots" / f"{name}.urdf")


def test_wam_style_example_carries_its_inner_motors_on_the_base():
    from arm_analyzer.dynamics import DynamicsModel

    wam = _example("wam_style_7dof")
    assert wam.warnings == []
    assert wam.actuated == ["j1", "j2", "j3", "j4", "j5", "j6", "j7"]
    # M1-M3 and their capstans sit in the base shell, below J1.
    for j in ("j1", "j2", "j3"):
        d = wam.drives[j]
        assert d.motor.link == "base_link" and d.gearbox.link == "base_link"
    # The wrist motors ride at the inner end of the forearm, not at the wrist.
    assert [wam.drives[j].motor.link for j in ("j5", "j6", "j7")] == ["link4"] * 3
    assert not any(wam.drives[j].colocated for j in ("j2", "j3", "j5", "j6", "j7"))
    # What the joints actually have to carry excludes everything on the base.
    model = DynamicsModel.build(wam)
    total = wam.mass_budget()["total"]
    on_base = sum(
        d.motor.mass + d.gearbox.mass
        for j, d in wam.drives.items()
        if d.motor.link == "base_link"
    )
    assert model.moving_mass == pytest.approx(total - on_base - wam.links["base_link"].inertial.mass)
    assert model.moving_mass < 0.65 * total


def test_wam_style_differentials_share_each_joint_between_two_motors():
    wam = _example("wam_style_7dof")
    assert [(c.name, c.joints) for c in wam.couplings] == [
        ("shoulder_differential", ["j2", "j3"]),
        ("wrist_differential", ["j5", "j6"]),
    ]
    # Both motors push on shoulder pitch, so it can be driven twice as hard as
    # one motor alone -- while its partner is unloaded.
    one_motor = float(wam.drives["j2"].joint_torque_limit(0.0))
    assert float(wam.joint_torque_limit("j2", 0.0)) == pytest.approx(2 * one_motor)
    # And each of the two joints carries both rotors.
    c = wam.couplings[0]
    spin = wam.drives["j2"].motor_spec.rotor_inertia + wam.drives["j2"].gearbox_spec.input_inertia
    ratio = wam.drives["j2"].total_ratio
    assert np.allclose(c.rotor_matrix(), 2 * spin * ratio**2 * np.eye(2))


def test_wam_style_differential_motors_see_the_same_duty():
    """Shoulder pitch does the work; both differential motors carry half of it."""
    import json

    from arm_analyzer.analysis import analyze
    from arm_analyzer.cartesian import plan_from_cartesian

    from conftest import EXAMPLES

    wam = _example("wam_style_7dof")
    spec = json.loads((EXAMPLES / "trajectories" / "default_pick_place.json").read_text())
    result = analyze(wam, plan_from_cartesian(wam, spec), rate_hz=100)
    by_joint = {s["name"]: s for s in result["summary"]}
    pitch, roll = by_joint["j2"], by_joint["j3"]
    # The joints are loaded very differently...
    assert pitch["joint"]["rms_torque"] > 10 * roll["joint"]["rms_torque"]
    # ... but their motors are not: each carries half of (pitch +/- roll).
    m_pitch = pitch["drive"]["motor"]["rms_torque"]
    m_roll = roll["drive"]["motor"]["rms_torque"]
    assert m_roll == pytest.approx(m_pitch, rel=0.1)
    assert m_pitch == pytest.approx(
        pitch["joint"]["rms_torque"] / (2 * wam.drives["j2"].total_ratio), rel=0.15
    )
    assert pitch["drive"]["coupling"]["partners"] == ["j3"]


def test_palletizer_example_is_four_axes_with_two_passive_linkages():
    pal = _example("palletizer_4dof")
    assert pal.warnings == []
    assert pal.actuated == ["j1", "j2", "j3", "j4"]
    assert pal.passive == ["level_elbow", "level_shoulder"]
    assert pal.couplings == []
    # The three heavy drives are on the pedestal, only the tool motor rides along.
    for j in ("j1", "j2", "j3"):
        assert pal.drives[j].motor.link == "base_link"
    assert pal.drives["j4"].motor.link == "link3"
    assert not any(pal.drives[j].colocated for j in ("j2", "j3", "j4"))


def test_palletizer_parallelogram_keeps_the_tool_plate_level():
    import pinocchio as pin

    from arm_analyzer.pin_model import build_model, joint_indices

    pal = _example("palletizer_4dof")
    model = build_model(pal, with_drives=False)
    assert model.nq == 4  # the two linkage joints cost no degree of freedom
    data = model.createData()
    idx_q, _ = joint_indices(model, pal.actuated)
    plate = model.getFrameId("link5")
    for q4 in ([0, 0, 0, 0], [0.5, 0.6, -0.3, 1.0], [-1.2, -0.2, 1.1, -2.0]):
        q = pin.neutral(model)
        q[idx_q] = q4
        pin.framesForwardKinematics(model, data, q)
        # Level whatever the arm does: the plate never pitches or rolls.
        assert np.allclose(data.oMf[plate].rotation[:, 2], [0, 0, 1], atol=1e-9), q4


def test_ur_style_example_carries_every_drive_on_the_arm():
    from arm_analyzer.dynamics import DynamicsModel

    ur = _example("ur_style_6dof")
    assert ur.warnings == []
    assert ur.couplings == [] and ur.passive == []
    assert all(d.colocated for d in ur.drives.values())
    for name, d in ur.drives.items():
        j = ur.joints[name]
        assert {d.motor.link, d.gearbox.link} <= {j.parent, j.child}
    # Nothing but the base shell and J1's motor stays behind.
    model = DynamicsModel.build(ur)
    assert model.moving_mass > 0.85 * ur.mass_budget()["total"]
