"""Inverse dynamics via Pinocchio, with the arm's lumped drives attached.

The rigid-body solve is Pinocchio's C++ RNEA (``pin.rnea``). Each link's body
is its URDF structure plus every motor and gearbox mounted on it (see
``pin_model.build_model``), wherever in the arm those lumps sit, so moving a
motor from the elbow to the base changes the load on every joint in between.

``pin.rnea`` also adds ``model.armature * qdd``, which we set to each drive's
reflected spinning inertia ``(I_rotor + I_gb_in) * N^2``. Drives coupled
through a differential reflect an inertia *matrix* instead (``A^T I A``, see
``robot.Coupling``), which no diagonal armature can express, so those joints
are left out of ``model.armature`` and their block is added here. The result is
split back into terms because the motor-side analysis needs them separately:

* ``link``   rigid-body torque (structure + lumps + payload, gravity, Coriolis)
* ``rotor``  ``armature * qdd``
* ``total``  their sum, the torque the drive must deliver at the joint

There is no friction term: losses are lumped into the drive efficiencies,
which the analysis applies on the motor side.

The spinning rotor is treated as a reflected inertia on its own joint axis;
its gyroscopic coupling with the host link's rotation is neglected (standard
for geared drives, where ``N^2`` dominates).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import pinocchio as pin

from arm_analyzer.pin_model import attach_body, build_model, joint_indices
from arm_analyzer.robot import ArmDescription

GRAVITY = np.array([0.0, 0.0, -9.80665])


@dataclass
class Payload:
    """A rigid mass carried by ``link``; ``com`` is in that link's frame."""

    link: str
    mass: float
    com: np.ndarray = field(default_factory=lambda: np.zeros(3))
    inertia: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))


@dataclass
class DynamicsModel:
    """A Pinocchio model of the arm plus the name <-> vector index mapping."""

    arm: ArmDescription
    model: pin.Model
    data: pin.Data
    names: list[str]  # actuated joints, base to tip
    idx_q: np.ndarray
    idx_v: np.ndarray
    # Coupled drives: (positions in ``names``, reflected inertia matrix). These
    # are the entries Pinocchio's diagonal armature cannot express.
    coupled: list[tuple[np.ndarray, np.ndarray]] = field(default_factory=list)

    @staticmethod
    def build(
        arm: ArmDescription,
        payload: Optional[Payload] = None,
        *,
        link_inertias: Optional[dict] = None,
    ) -> "DynamicsModel":
        """``link_inertias`` replaces the file's structural inertia link by
        link, for a derived link mass (see ``pin_model.set_link_inertias``)."""
        model = build_model(arm, link_inertias=link_inertias)
        if payload is not None and payload.mass > 0:
            if payload.link not in arm.links:
                raise ValueError(f"payload link {payload.link!r} is not in the robot")
            T = np.eye(4)
            T[:3, 3] = np.asarray(payload.com, dtype=float)
            attach_body(model, payload.link, "payload", payload.mass, T, payload.inertia)
        names = arm.actuated
        idx_q, idx_v = joint_indices(model, names)
        coupled = [
            (np.array([names.index(n) for n in c.joints]), c.rotor_matrix())
            for c in arm.couplings
        ]
        return DynamicsModel(
            arm=arm,
            model=model,
            data=model.createData(),
            names=names,
            idx_q=idx_q,
            idx_v=idx_v,
            coupled=coupled,
        )

    @property
    def moving_mass(self) -> float:
        """Mass carried by the joints (everything except what sits on the base)."""
        return float(pin.computeTotalMass(self.model))

    @property
    def armature(self) -> np.ndarray:
        return np.asarray(self.model.armature, dtype=float)[self.idx_v]

    def _vectors(self, q, qd=None, qdd=None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Name-keyed dicts (missing = 0) to Pinocchio q / v / a vectors."""
        out = []
        for values, size, idx in (
            (q, self.model.nq, self.idx_q),
            (qd, self.model.nv, self.idx_v),
            (qdd, self.model.nv, self.idx_v),
        ):
            vec = np.zeros(size)
            if values:
                vec[idx] = [float(values.get(n, 0.0)) for n in self.names]
            out.append(vec)
        return out[0], out[1], out[2]

    def _set_gravity(self, gravity) -> None:
        self.model.gravity.linear = np.asarray(gravity, dtype=float)


def torque_series(
    model: DynamicsModel,
    Q: np.ndarray,
    QD: np.ndarray,
    QDD: np.ndarray,
    gravity: np.ndarray = GRAVITY,
) -> dict[str, np.ndarray]:
    """Torque terms for ``N`` samples at once.

    ``Q``, ``QD``, ``QDD`` are ``(N, n_joints)`` in ``model.names`` order.
    Returns ``(N, n_joints)`` arrays ``link``, ``rotor`` and ``total``.
    """
    Q, QD, QDD = (np.atleast_2d(np.asarray(x, dtype=float)) for x in (Q, QD, QDD))
    n = len(Q)
    m, d = model.model, model.data
    model._set_gravity(gravity)
    q = pin.neutral(m)
    v = np.zeros(m.nv)
    a = np.zeros(m.nv)
    rnea = np.empty((n, len(model.names)))
    for i in range(n):
        q[model.idx_q] = Q[i]
        v[model.idx_v] = QD[i]
        a[model.idx_v] = QDD[i]
        rnea[i] = pin.rnea(m, d, q, v, a)[model.idx_v]
    # The diagonal part is already inside the RNEA; the coupled blocks are not.
    diagonal = model.armature * QDD
    rotor = diagonal.copy()
    for idx, M_r in model.coupled:
        rotor[:, idx] = QDD[:, idx] @ M_r
    link = rnea - diagonal
    return {"link": link, "rotor": rotor, "total": link + rotor}


def joint_torque_terms(
    model: DynamicsModel,
    q: dict[str, float],
    qd: dict[str, float],
    qdd: dict[str, float],
    gravity: np.ndarray = GRAVITY,
) -> dict[str, dict[str, float]]:
    """Single-sample, name-keyed version of ``torque_series``."""
    row = lambda values: [float((values or {}).get(n, 0.0)) for n in model.names]  # noqa: E731
    terms = torque_series(model, row(q), row(qd), row(qdd), gravity)
    return {
        n: {k: float(v[0, j]) for k, v in terms.items()} for j, n in enumerate(model.names)
    }


def gravity_torques(
    model: DynamicsModel, q: dict[str, float], gravity: np.ndarray = GRAVITY
) -> dict[str, float]:
    """Static holding torque at pose ``q`` (``pin.computeGeneralizedGravity``)."""
    model._set_gravity(gravity)
    qv, _, _ = model._vectors(q)
    g = pin.computeGeneralizedGravity(model.model, model.data, qv)
    return {n: float(g[i]) for n, i in zip(model.names, model.idx_v)}


def mass_matrix(model: DynamicsModel, q: dict[str, float]) -> tuple[np.ndarray, list[str]]:
    """Joint-space inertia ``M(q)`` including armature (``pin.crba``)."""
    qv, _, _ = model._vectors(q)
    M = pin.crba(model.model, model.data, qv)
    M = np.triu(M) + np.triu(M, 1).T  # crba fills the upper triangle
    idx = model.idx_v
    out = M[np.ix_(idx, idx)]
    for group, M_r in model.coupled:
        out[np.ix_(group, group)] += M_r
    return out, list(model.names)


def potential_energy(model: DynamicsModel, q: dict[str, float], gravity: np.ndarray = GRAVITY) -> float:
    model._set_gravity(gravity)
    qv, _, _ = model._vectors(q)
    return float(pin.computePotentialEnergy(model.model, model.data, qv))
