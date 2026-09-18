import numpy as np
import pytest

from arm_analyzer.dynamics import (
    DynamicsModel,
    Payload,
    gravity_torques,
    joint_torque_terms,
    mass_matrix,
    potential_energy,
    torque_series,
)
from arm_analyzer.robot import parse_arm
from arm_analyzer.trajectory import load_trajectory

G = 9.80665


def test_gravity_torque_matches_hand_calculation(one_joint):
    # 2 kg at 0.5 m plus a 0.5 kg motor at 0.2 m, arm horizontal. A positive
    # rotation about +Y tips +X toward -Z, so gravity pulls toward +q and the
    # actuator must hold with a negative torque.
    arm = parse_arm(one_joint(link_mass=2.0, link_com_x=0.5, motor_xyz="0.2 0 0"))
    tau = gravity_torques(DynamicsModel.build(arm), {"shoulder": 0.0})["shoulder"]
    assert tau == pytest.approx(-G * (2.0 * 0.5 + 0.5 * 0.2))


def test_moving_the_motor_to_the_base_removes_its_load(one_joint):
    on_arm = parse_arm(one_joint(motor_link="arm", motor_xyz="0.4 0 0"))
    on_base = parse_arm(one_joint(motor_link="base", motor_xyz="0 0 0.2"))
    t_arm = gravity_torques(DynamicsModel.build(on_arm), {"shoulder": 0.0})["shoulder"]
    t_base = gravity_torques(DynamicsModel.build(on_base), {"shoulder": 0.0})["shoulder"]
    assert t_arm - t_base == pytest.approx(-G * 0.5 * 0.4)
    # Total mass is unchanged; only where it rides changed.
    assert on_arm.mass_budget()["total"] == pytest.approx(on_base.mass_budget()["total"])


def test_gravity_torque_follows_pose(one_joint):
    arm = parse_arm(one_joint(link_mass=2.0, link_com_x=0.5, motor_link="base"))
    model = DynamicsModel.build(arm)
    for q in (0.0, 0.4, -1.1, np.pi / 2):
        tau = gravity_torques(model, {"shoulder": q})["shoulder"]
        # The lever arm of the 1 kg*m first moment shrinks as cos(q).
        assert tau == pytest.approx(-G * 1.0 * np.cos(q), abs=1e-12)


def test_reflected_rotor_inertia(one_joint):
    arm = parse_arm(one_joint(rotor_inertia=2e-5, ratio=50))
    model = DynamicsModel.build(arm)
    terms = joint_torque_terms(model, {"shoulder": 0.3}, {}, {"shoulder": 4.0}, np.zeros(3))
    assert terms["shoulder"]["rotor"] == pytest.approx(2e-5 * 50**2 * 4.0)
    # Point mass 2 kg at 0.5 m plus 0.5 kg motor at 0.2 m.
    assert terms["shoulder"]["link"] == pytest.approx((2.0 * 0.25 + 0.5 * 0.04) * 4.0)


def test_payload_adds_load(one_joint):
    arm = parse_arm(one_joint(motor_link="base"))
    base = gravity_torques(DynamicsModel.build(arm), {})["shoulder"]
    loaded = gravity_torques(
        DynamicsModel.build(arm, Payload("arm", 1.5, np.array([1.0, 0, 0]))), {}
    )["shoulder"]
    assert loaded - base == pytest.approx(-G * 1.5)


def test_mass_matrix_is_symmetric_positive_definite(simple_arm):
    q = {"j1": 0.3, "j2": -0.7, "j3": 1.1, "j4": 0.4, "j5": -0.9, "j6": 0.2}
    M, names = mass_matrix(DynamicsModel.build(simple_arm), q)
    assert names == simple_arm.actuated
    assert np.allclose(M, M.T, atol=1e-10)
    assert np.all(np.linalg.eigvalsh(M) > 0)
    # Reflected drive inertia sits on the diagonal and dominates for j2.
    assert M[1, 1] > simple_arm.drives["j2"].reflected_inertia


def test_work_equals_potential_energy_change(simple_urdf):
    """Rest-to-rest move: joint work == change in potential energy.

    Checks the whole adapter -- joint mapping, gravity sign, attached drive
    lumps, armature -- through an energy identity rather than a second solver.
    Reflected rotor inertia stores no net energy over a rest-to-rest move.
    """
    simple_arm = parse_arm(simple_urdf)
    traj = {
        "format": "waypoints",
        "units": "deg",
        "waypoints": [
            {"joints": [0, 0, 0, 0, 0, 0]},
            {"joints": [60, 40, 70, 30, 60, 20], "stop": False},
            {"joints": [-30, 70, 20, -40, 30, 90]},
        ],
    }
    plan = load_trajectory(simple_arm, traj)
    model = DynamicsModel.build(simple_arm)
    samples = list(plan.sample_uniform(2000))
    power = []
    for _t, q, qd, qdd in samples:
        terms = joint_torque_terms(model, q, qd, qdd)
        power.append(sum(terms[n]["total"] * qd.get(n, 0.0) for n in simple_arm.actuated))
    t = np.array([s[0] for s in samples])
    work = float(np.sum(0.5 * (np.array(power[1:]) + np.array(power[:-1])) * np.diff(t)))
    q0, q1 = samples[0][1], samples[-1][1]
    d_pe = potential_energy(model, q1) - potential_energy(model, q0)
    assert abs(d_pe) > 1.0  # the move genuinely changes height
    assert work == pytest.approx(d_pe, rel=2e-3, abs=2e-3)


def test_drive_lumps_and_armature_reach_pinocchio(simple_arm):
    model = DynamicsModel.build(simple_arm)
    lumps_on_base = sum(l.mass for l in simple_arm.lumps() if l.link == simple_arm.root)
    budget = simple_arm.mass_budget()
    expected = budget["total"] - simple_arm.links[simple_arm.root].inertial.mass - lumps_on_base
    assert model.moving_mass == pytest.approx(expected)
    assert np.allclose(model.armature, [simple_arm.drives[n].reflected_inertia for n in model.names])
    for lump in simple_arm.lumps():
        assert model.model.existBodyName(lump.name)


def test_batch_matches_single_sample(simple_arm):
    model = DynamicsModel.build(simple_arm)
    rng = np.random.default_rng(7)
    Q, QD, QDD = (rng.uniform(-1, 1, (5, 6)) for _ in range(3))
    batch = torque_series(model, Q, QD, QDD)
    for i in range(5):
        row = lambda M: dict(zip(model.names, M[i]))  # noqa: E731
        single = joint_torque_terms(model, row(Q), row(QD), row(QDD))
        for j, n in enumerate(model.names):
            for k in ("link", "rotor", "total"):
                assert batch[k][i, j] == pytest.approx(single[n][k], abs=1e-12)


def test_rnea_is_consistent_with_mass_matrix_and_gravity(simple_arm):
    """tau(q, 0, a) - g(q) == M(q) a, with armature, for a Pinocchio sanity check."""
    model = DynamicsModel.build(simple_arm)
    q = dict(zip(model.names, (0.4, -0.3, 1.0, 0.2, 0.7, -0.5)))
    a = dict(zip(model.names, (1.0, -2.0, 0.5, 3.0, -1.0, 2.0)))
    terms = joint_torque_terms(model, q, {}, a)
    g = gravity_torques(model, q)
    M, names = mass_matrix(model, q)
    Ma = M @ np.array([a[n] for n in names])
    for i, n in enumerate(names):
        assert terms[n]["link"] + terms[n]["rotor"] - g[n] == pytest.approx(Ma[i], abs=1e-9)
