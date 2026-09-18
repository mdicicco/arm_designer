"""Rigid-body mass properties as declared in the robot description.

Parsing-side helpers only; combining bodies for dynamics is Pinocchio's job
(see ``pin_model.py``). Motors and gearboxes are not solid metal, so an
undeclared lump inertia is the shape's inertia scaled to the declared mass
(uniform-density assumption), not density times volume.

Conventions: inertia tensors are about the centre of mass, expressed in the
frame the properties are given in.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class MassProperties:
    """Mass, centre of mass, and the inertia tensor about that centre."""

    mass: float
    com: np.ndarray  # (3,)
    inertia: np.ndarray  # (3, 3) about com

    @staticmethod
    def zero() -> "MassProperties":
        return MassProperties(0.0, np.zeros(3), np.zeros((3, 3)))

    def transformed(self, T: np.ndarray) -> "MassProperties":
        """Express these properties in the parent frame of ``T`` (4x4)."""
        T = np.asarray(T, dtype=float)
        R, t = T[:3, :3], T[:3, 3]
        return MassProperties(self.mass, t + R @ self.com, R @ self.inertia @ R.T)


def unit_mass_inertia(shape: str, params: tuple[float, ...]) -> np.ndarray:
    """Inertia tensor of a 1 kg solid primitive about its own centroid.

    URDF primitives are centred on their origin; cylinders run along local +Z.
    """
    if shape == "box":
        sx, sy, sz = params
        return np.diag(
            [(sy * sy + sz * sz) / 12.0, (sx * sx + sz * sz) / 12.0, (sx * sx + sy * sy) / 12.0]
        )
    if shape == "cylinder":
        r, h = params
        radial = (3.0 * r * r + h * h) / 12.0
        return np.diag([radial, radial, r * r / 2.0])
    if shape == "sphere":
        (r,) = params
        i = 2.0 / 5.0 * r * r
        return np.diag([i, i, i])
    raise ValueError(f"unsupported primitive {shape!r}")


def sphere_radius_for_mass(mass: float, density: float = 3000.0) -> float:
    """Radius of a solid sphere of ``mass`` at ``density`` (display fallback)."""
    return (3.0 * max(mass, 1e-6) / (4.0 * math.pi * density)) ** (1.0 / 3.0)
