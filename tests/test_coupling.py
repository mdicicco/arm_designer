"""Coupled drives (differentials) and passive linkages (<mimic>)."""

import numpy as np
import pytest

from arm_analyzer.analysis import analyze
from arm_analyzer.dynamics import DynamicsModel, mass_matrix
from arm_analyzer.pin_model import build_model
from arm_analyzer.profile import PolySegment, TrajectoryPlan
from arm_analyzer.robot import parse_arm

RATIO = 20.0
ROTOR = 1.0e-4

MOTOR = (
    'rotor_inertia="%s" peak_torque="1.0" continuous_torque="0.4" '
    'stall_torque="2.0" no_load_speed="500"' % ROTOR
)


def two_joint_urdf(coupling: str = "") -> str:
    """Base -> link1 (j1, pitch) -> link2 (j2, pitch), both motors on the base."""
    drive = lambda j: f"""
      <drive>
        <motor name="{j}_motor" link="base" {MOTOR}>
          <origin xyz="0 0 0.05"/><mass value="0.5"/>
        </motor>
        <gearbox name="{j}_gearbox" link="base" ratio="{RATIO}"
                 peak_torque="60" rated_torque="25">
          <origin xyz="0 0 0.05"/><mass value="0.3"/>
        </gearbox>
      </drive>"""
    return f"""<?xml version="1.0"?>
<robot name="two_joint">
  <link name="base">
    <inertial><mass value="5"/><origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>
  </link>
  <link name="link1">
    <inertial><mass value="2"/><origin xyz="0.25 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
  <link name="link2">
    <inertial><mass value="1"/><origin xyz="0.2 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
    <tool_tip name="tcp" xyz="0.4 0 0"/>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" velocity="4" effort="200"/>{drive('j1')}
  </joint>
  <joint name="j2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0.5 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" velocity="4" effort="200"/>{drive('j2')}
  </joint>
  {coupling}
</robot>
"""


DIFFERENTIAL = '<drive_coupling name="shoulder" type="differential" joints="j1 j2"/>'


@pytest.fixture(scope="module")
def plain():
    return parse_arm(two_joint_urdf())


@pytest.fixture(scope="module")
def diff_arm():
    return parse_arm(two_joint_urdf(DIFFERENTIAL))


def _plan(arm, q, qd, qdd):
    """A short constant-acceleration plan, so every sample has a known state."""
    plan = TrajectoryPlan(joints=list(arm.actuated))
    plan.segments.append(
        PolySegment(
            0.0,
            0.05,
            {n: (q[i], qd[i], 0.5 * qdd[i], 0.0) for i, n in enumerate(arm.actuated)},
        )
    )
    plan.waypoint_times = [0.0, 0.05]
    return plan


def test_differential_maps_both_joints_onto_both_motors(diff_arm):
    c = diff_arm.coupling("j1")
    assert c is not None and c.kind == "differential" and c.joints == ["j1", "j2"]
    assert np.allclose(c.A, RATIO * np.array([[1.0, 1.0], [1.0, -1.0]]))
    # tau_m1 = (tau_1 + tau_2) / 2N, tau_m2 = (tau_1 - tau_2) / 2N.
    tau = np.array([[60.0, 20.0]])
    assert np.allclose(c.motor_torques(tau), [[80.0 / (2 * RATIO), 40.0 / (2 * RATIO)]])
    # w_m1 = N (q1' + q2'), w_m2 = N (q1' - q2').
    assert np.allclose(c.motor_speeds(np.array([[1.0, 0.25]])), [[RATIO * 1.25, RATIO * 0.75]])
    assert diff_arm.coupling("j2") is c


def test_each_differential_joint_carries_both_rotors(plain, diff_arm):
    """Reflected inertia is a matrix: equal rotors double it on both joints."""
    c = diff_arm.coupling("j1")
    assert np.allclose(c.rotor_matrix(), 2 * ROTOR * RATIO**2 * np.eye(2))
    q = {"j1": 0.3, "j2": -0.4}
    M_plain, _ = mass_matrix(DynamicsModel.build(plain), q)
    M_diff, _ = mass_matrix(DynamicsModel.build(diff_arm), q)
    extra = ROTOR * RATIO**2
    assert np.allclose(np.diag(M_diff - M_plain), [extra, extra])
    assert np.allclose(M_diff - M_plain, np.diag([extra, extra]))  # equal rotors: no cross term


def test_unequal_rotors_couple_the_two_joints():
    urdf = two_joint_urdf(DIFFERENTIAL).replace(
        f'<motor name="j2_motor" link="base" rotor_inertia="{ROTOR}"',
        f'<motor name="j2_motor" link="base" rotor_inertia="{ROTOR / 2}"',
    )
    c = parse_arm(urdf).coupling("j1")
    off = ROTOR * RATIO**2 * (1 - 0.5)
    assert c.rotor_matrix()[0, 1] == pytest.approx(off)
    assert c.rotor_matrix()[1, 0] == pytest.approx(off)


def test_differential_doubles_a_single_joint_envelope(plain, diff_arm):
    """Both motors push on one joint when its partner is unloaded."""
    alone = float(plain.joint_torque_limit("j1", 0.0))
    together = float(diff_arm.joint_torque_limit("j1", 0.0))
    assert together == pytest.approx(2 * alone)
    # ... but each motor still runs out of speed at the same place.
    assert diff_arm.drive_speed_limit("j1") == pytest.approx(plain.drive_speed_limit("j1"))
    assert float(diff_arm.joint_torque_limit("j1", 30.0)) == 0.0  # past no-load speed


def test_analysis_splits_the_load_between_the_two_motors(plain, diff_arm):
    """Same motion, same joint torques -- but each motor sees half the sum."""
    q, qd, qdd = [0.4, -0.6], [0.5, -0.2], [1.0, 2.0]
    r_plain = analyze(plain, _plan(plain, q, qd, qdd), rate_hz=200)
    r_diff = analyze(diff_arm, _plan(diff_arm, q, qd, qdd), rate_hz=200)

    i = len(r_diff["t"]) // 2
    tau1, tau2 = (r_diff["series"][n]["tau_link"][i] for n in ("j1", "j2"))
    m1, m2 = (r_diff["series"][n]["motor_torque"][i] for n in ("j1", "j2"))
    spin = ROTOR
    a1 = RATIO * (qdd[0] + qdd[1])
    a2 = RATIO * (qdd[0] - qdd[1])
    assert m1 == pytest.approx((tau1 + tau2) / (2 * RATIO) + spin * a1, rel=1e-5)
    assert m2 == pytest.approx((tau1 - tau2) / (2 * RATIO) + spin * a2, rel=1e-5)
    # Motor speeds follow the same map.
    assert r_diff["series"]["j1"]["motor_speed"][i] == pytest.approx(
        RATIO * (r_diff["series"]["j1"]["qd"][i] + r_diff["series"]["j2"]["qd"][i]), rel=1e-5
    )
    # The uncoupled arm, with the same geometry, puts each joint on one motor.
    p1 = r_plain["series"]["j1"]["motor_torque"][i]
    assert p1 == pytest.approx(
        r_plain["series"]["j1"]["tau_link"][i] / RATIO + spin * RATIO * qdd[0], rel=1e-5
    )
    # Coupling info reaches the GUI.
    assert r_diff["summary"][0]["drive"]["coupling"]["partners"] == ["j2"]
    assert r_plain["summary"][0]["drive"]["coupling"] is None


def test_gearbox_output_is_its_own_motor_through_its_own_ratio(diff_arm):
    """Each gearbox still sees only the motor bolted to it, times its ratio."""
    qdd = [0.5, -0.5]
    r = analyze(diff_arm, _plan(diff_arm, [0.2, 0.3], [0.1, -0.1], qdd), rate_hz=200)
    i = len(r["t"]) // 2
    motor_accel = {"j1": RATIO * (qdd[0] + qdd[1]), "j2": RATIO * (qdd[0] - qdd[1])}
    for n in ("j1", "j2"):
        s = r["series"][n]
        load = s["motor_torque"][i] - ROTOR * motor_accel[n]  # what enters the gearbox
        assert s["gearbox_torque"][i] == pytest.approx(load * RATIO, rel=1e-5)


# ---------------------------------------------------------------------------
# Bad couplings
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "xml, message",
    [
        ('<drive_coupling name="c" type="differential" joints="j1"/>', "exactly two"),
        ('<drive_coupling name="c" type="differential" joints="j1 nope"/>', "unknown joint"),
        ('<drive_coupling name="c" type="differential" joints="j1 j1"/>', "only once"),
        ('<drive_coupling name="c" type="screw" joints="j1 j2"/>', "unsupported type"),
        ('<drive_coupling name="c" type="matrix" joints="j1 j2">'
         '<motor joint="j1" gains="1 1"/><motor joint="j2" gains="2 2"/></drive_coupling>',
         "singular"),
        ('<drive_coupling name="c" type="matrix" joints="j1 j2">'
         '<motor joint="j1" gains="1 1"/></drive_coupling>', "one <motor"),
        ('<drive_coupling name="a" type="differential" joints="j1 j2"/>'
         '<drive_coupling name="b" type="differential" joints="j1 j2"/>', "already coupled"),
    ],
)
def test_bad_couplings(xml, message):
    with pytest.raises(ValueError, match=message):
        parse_arm(two_joint_urdf(xml))


def test_coupling_needs_a_drive():
    urdf = two_joint_urdf(DIFFERENTIAL)
    start = urdf.index("<drive>\n", urdf.index('name="j2"'))
    end = urdf.index("</drive>", start) + len("</drive>")
    with pytest.raises(ValueError, match="no <drive>"):
        parse_arm(urdf[:start] + urdf[end:])


# ---------------------------------------------------------------------------
# Passive linkages
# ---------------------------------------------------------------------------


def mimic_urdf(mimic: str = '<mimic joint="j3" multiplier="-1"/>', drive: str = "") -> str:
    return f"""<?xml version="1.0"?>
<robot name="parallelogram">
  <link name="base"><inertial><mass value="4"/><origin xyz="0 0 0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial></link>
  <link name="link1"><inertial><mass value="2"/><origin xyz="0.25 0 0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="link2"><inertial><mass value="1"/><origin xyz="0.2 0 0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial></link>
  <link name="plate"><inertial><mass value="0.5"/><origin xyz="0 0 0"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
    <tool_tip name="tcp" xyz="0 0 -0.05"/></link>
  <joint name="j2" type="revolute">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0.3"/><axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" velocity="3" effort="200"/>
    <drive>
      <motor link="base" rotor_inertia="1e-4" peak_torque="1.0" continuous_torque="0.4"
             stall_torque="2.0" no_load_speed="500">
        <origin xyz="0 0 0.05"/><mass value="0.5"/></motor>
      <gearbox link="base" ratio="20" peak_torque="60" rated_torque="25">
        <origin xyz="0 0 0.05"/><mass value="0.3"/></gearbox>
    </drive>
  </joint>
  <joint name="j3" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0.5 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" velocity="3" effort="200"/>
    <drive>
      <motor link="base" rotor_inertia="1e-4" peak_torque="1.0" continuous_torque="0.4"
             stall_torque="2.0" no_load_speed="500">
        <origin xyz="0 0 0.05"/><mass value="0.5"/></motor>
      <gearbox link="base" ratio="20" peak_torque="60" rated_torque="25">
        <origin xyz="0 0 0.05"/><mass value="0.3"/></gearbox>
    </drive>
  </joint>
  <joint name="level" type="revolute">
    <parent link="link2"/><child link="plate"/>
    <origin xyz="0.4 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-4" upper="4" velocity="3" effort="200"/>
    {mimic}{drive}
  </joint>
</robot>
"""


def test_mimic_joint_is_passive():
    arm = parse_arm(mimic_urdf())
    assert arm.actuated == ["j2", "j3"]
    assert arm.passive == ["level"]
    assert arm.joints["level"].mimic.multiplier == -1.0
    assert not any("level" in w for w in arm.warnings)  # not a missing drive
    model = build_model(arm)
    assert model.nq == 2  # the linkage costs no degree of freedom


def test_parallelogram_keeps_the_plate_level():
    import pinocchio as pin

    from arm_analyzer.pin_model import joint_indices

    arm = parse_arm(mimic_urdf())
    model = build_model(arm)
    data = model.createData()
    idx_q, _ = joint_indices(model, arm.actuated)
    q = pin.neutral(model)
    frame = model.getFrameId("plate")
    for j3 in (-0.7, 0.0, 0.9):
        q[idx_q] = [0.3, j3]
        pin.framesForwardKinematics(model, data, q)
        R = data.oMf[frame].rotation
        # The rod cancels j3, so the plate keeps the shoulder's pitch, 0.3.
        assert np.allclose(R, pin.exp3(np.array([0.0, 0.3, 0.0])), atol=1e-9)


def test_mimic_joint_may_not_have_a_drive():
    drive = """
    <drive>
      <motor link="base" peak_torque="1"><mass value="0.2"/></motor>
      <gearbox link="base" ratio="10"><mass value="0.2"/></gearbox>
    </drive>"""
    with pytest.raises(ValueError, match="cannot have a drive"):
        parse_arm(mimic_urdf(drive=drive))


@pytest.mark.parametrize(
    "mimic, message",
    [
        ('<mimic joint="nope"/>', "unknown joint"),
        ("<mimic/>", "needs joint="),
    ],
)
def test_bad_mimic(mimic, message):
    with pytest.raises(ValueError, match=message):
        parse_arm(mimic_urdf(mimic=mimic))


def test_analysis_runs_with_a_passive_linkage():
    arm = parse_arm(mimic_urdf())
    plan = _plan(arm, [0.2, -0.3], [0.4, 0.1], [1.0, -1.0])
    result = analyze(arm, plan, rate_hz=100)
    assert result["joints"] == ["j2", "j3"]
    assert all(s["drive"] is not None for s in result["summary"])
    # The plate's mass is carried by both joints even though it has no drive.
    assert result["moving_mass"] > 3.0
