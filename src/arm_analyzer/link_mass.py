"""Derive each link's mass from its geometry and what is bolted to it.

A link's structure is modelled as **one tube plus a collar per actuator**:

* **The tube** spans the link -- from the joint that attaches it to its parent
  (its own frame origin) to the joint that carries the next link. That span is
  the length; nothing here invents it, it comes out of the URDF kinematics.
  Its outside diameter is not free either: it tapers down the chain, because a
  shoulder carries the whole arm and a wrist carries a gripper. With
  ``baseline_diameter`` at the base and ``taper`` the tip-to-base ratio,

      d(link i of n) = baseline_diameter * taper ** (i / (n - 1))

  so ``taper = 1`` is a constant-diameter arm and ``taper = 0.4`` a wrist tube
  40% the diameter of the base. The wall is ``wall`` thick, and a wall at least
  as thick as the radius makes it solid.

* **A collar per actuator**, for the flanges, bosses and bearing seats that
  carry a motor or gearbox. Each lump mounted on the link adds
  ``actuator_mass + actuator_fraction * lump.mass`` at that lump's own
  position, taking the lump's shape -- literally a cylinder around the mount
  point. ``actuator_fraction`` is there because a bigger actuator needs a
  bigger boss; ``actuator_mass`` is the part that does not scale.

Mass follows from geometry, and inertia with it: the tube's tensor is a hollow
cylinder about its own centre, each collar's is its shape scaled to its mass,
and the two are combined through the parallel-axis theorem. Nothing here reads
the ``<inertial>`` block it replaces.

This is a *sizing* model, not a stress model -- there is no load path, no
buckling check and no joint housing. It is meant for asking what an arm weighs
when its links are sized like each other, and for seeing how that mass feeds
back into the torque the motion needs.

**The defaults are a starting point, not a calibration.** ``baseline_diameter``
is an absolute size, so one set of numbers cannot fit a 7 kg desktop arm and an
80 kg palletizer of the same reach at once; on the bundled examples the default
lands anywhere from 0.4x to 1.6x their declared structure. Tune it per robot --
that is what the sliders are for -- and read the ratio the analysis reports
against the declared mass as the gauge.

The drawn ``<visual>`` geometry is left alone: the tube is the structural
idealisation of a link, not a claim about how it looks.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np

from arm_analyzer.mass_properties import MassProperties, unit_mass_inertia

# Aluminium, the usual default for a robot structure.
DEFAULT_DENSITY = 2700.0

# A link with no child joint and no tool tip still needs a length; this is a
# last resort, not a modelling choice.
FALLBACK_LENGTH_M = 0.05


@dataclass
class LinkMassParams:
    """The knobs behind the link-mass sliders. Lengths in metres."""

    baseline_diameter: float = 0.12   # tube OD at the base link
    taper: float = 0.50               # tip OD / base OD; 1.0 = no taper
    wall: float = 0.008               # tube wall; >= radius means solid
    density: float = DEFAULT_DENSITY  # structural material
    actuator_mass: float = 0.30       # kg of collar per mounted actuator
    actuator_fraction: float = 0.50   # + this much of the actuator's own mass

    LIMITS = {
        "baseline_diameter": (0.02, 0.40),
        "taper": (0.10, 1.50),
        "wall": (0.0005, 0.05),
        "density": (500.0, 9000.0),
        "actuator_mass": (0.0, 5.0),
        "actuator_fraction": (0.0, 2.0),
    }

    def validated(self) -> "LinkMassParams":
        for name, (lo, hi) in self.LIMITS.items():
            v = float(getattr(self, name))
            if not math.isfinite(v) or not (lo <= v <= hi):
                raise ValueError(f"link mass {name} must be between {lo:g} and {hi:g}, got {v:g}")
        return self

    @staticmethod
    def from_dict(d: Optional[dict[str, Any]]) -> "LinkMassParams":
        p = LinkMassParams()
        for name in LinkMassParams.LIMITS:
            if d and d.get(name) is not None:
                setattr(p, name, float(d[name]))
        return p.validated()

    def to_dict(self) -> dict[str, float]:
        return {name: float(getattr(self, name)) for name in self.LIMITS}


def _combine(parts: list[MassProperties]) -> MassProperties:
    """Merge bodies given in one frame, about their common centre of mass."""
    total = sum(p.mass for p in parts)
    if total <= 0:
        return MassProperties.zero()
    com = sum((p.mass * p.com for p in parts), np.zeros(3)) / total
    inertia = np.zeros((3, 3))
    for p in parts:
        d = p.com - com
        # Parallel axis: I + m (|d|^2 E - d d^T)
        inertia = inertia + p.inertia + p.mass * (float(d @ d) * np.eye(3) - np.outer(d, d))
    return MassProperties(total, com, inertia)


def _tube(length: float, outer_r: float, wall: float, density: float, axis: np.ndarray):
    """A hollow cylinder along ``axis``, one end at the link origin.

    Returns its :class:`MassProperties` in the link frame, plus the inner
    radius actually used (clamped to solid).
    """
    inner_r = max(outer_r - max(wall, 0.0), 0.0)
    volume = math.pi * (outer_r**2 - inner_r**2) * length
    mass = volume * density
    if mass <= 0:
        return MassProperties.zero(), inner_r
    # Hollow cylinder about its own centre, local +Z along the tube.
    radial = mass * (3.0 * (outer_r**2 + inner_r**2) + length**2) / 12.0
    axial = mass * (outer_r**2 + inner_r**2) / 2.0
    local = np.diag([radial, radial, axial])
    R = _rotation_to(axis)
    return MassProperties(mass, axis * (length / 2.0), R @ local @ R.T), inner_r


def _rotation_to(axis: np.ndarray) -> np.ndarray:
    """A rotation taking +Z onto the unit vector ``axis``."""
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, axis)
    c = float(np.dot(z, axis))
    s = float(np.linalg.norm(v))
    if s < 1e-12:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def link_span(arm, link: str) -> tuple[float, np.ndarray]:
    """``(length, unit axis)`` of a link, from the URDF's own kinematics.

    The link frame sits at the joint that attaches it to its parent, so the
    span is the offset to the joint that carries the next link -- the longest
    one where a link branches (a palletizer's parallelogram). A tip link has no
    child joint and uses its tool tip instead.
    """
    best = np.zeros(3)
    for j in arm.joints.values():
        if j.parent != link:
            continue
        v = np.asarray(j.T_origin, dtype=float)[:3, 3]
        if np.linalg.norm(v) > np.linalg.norm(best):
            best = v
    if np.linalg.norm(best) < 1e-9:
        for tip in arm.links[link].tool_tips:
            v = np.asarray(tip["T"], dtype=float)[:3, 3]
            if np.linalg.norm(v) > np.linalg.norm(best):
                best = v
    length = float(np.linalg.norm(best))
    if length < 1e-9:
        return FALLBACK_LENGTH_M, np.array([0.0, 0.0, 1.0])
    return length, best / length


def derive(arm, params: LinkMassParams) -> dict[str, dict[str, Any]]:
    """Per-link tube, collars and the resulting mass properties.

    Keyed by link name, base to tip; the value carries the numbers the GUI
    shows alongside ``"properties"``, the :class:`MassProperties` that would
    replace the link's ``<inertial>``.
    """
    params.validated()
    chain = [arm.root] + list(arm.order)
    last = max(len(chain) - 1, 1)
    base_r = params.baseline_diameter / 2.0

    # Which lumps ride on which link.
    mounted: dict[str, list] = {n: [] for n in arm.links}
    for drive in arm.drives.values():
        for lump in (drive.motor, drive.gearbox):
            if lump.link in mounted:
                mounted[lump.link].append(lump)

    out: dict[str, dict[str, Any]] = {}
    for i, name in enumerate(chain):
        length, axis = link_span(arm, name)
        outer_r = base_r * (params.taper ** (i / last))
        tube, inner_r = _tube(length, outer_r, params.wall, params.density, axis)

        parts = [tube]
        collars = []
        for lump in mounted.get(name, []):
            mass = params.actuator_mass + params.actuator_fraction * lump.mass
            if mass <= 0:
                continue
            shape = lump.shape
            local = unit_mass_inertia(shape.type, tuple(shape.params)) * mass
            # The collar takes the lump's own pose, so it sits around the mount.
            T = np.asarray(lump.T, dtype=float)
            R = T[:3, :3]
            parts.append(MassProperties(mass, T[:3, 3], R @ local @ R.T))
            collars.append({"lump": lump.name, "kind": lump.kind, "mass": mass})

        props = _combine(parts)
        out[name] = {
            "index": i,
            "length": length,
            "outer_diameter": 2.0 * outer_r,
            "inner_diameter": 2.0 * inner_r,
            "solid": inner_r <= 0.0,
            "tube_mass": tube.mass,
            "collar_mass": sum(c["mass"] for c in collars),
            "collars": collars,
            "mass": props.mass,
            "declared": arm.links[name].inertial.mass,
            "properties": props,
        }
    return out
