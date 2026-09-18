"""Arm description: URDF plus a per-joint ``<drive>`` extension.

A single serial arm is described in one URDF file. Links carry their structure
the standard way (``<visual>``/``<collision>`` geometry and an ``<inertial>``
block). Every actuated joint additionally declares the motor and gearbox that
drive it::

    <joint name="j2" type="revolute">
      ...
      <limit lower="-2.2" upper="2.2" velocity="2.5" effort="90"/>
      <drive>
        <motor name="j2_motor" link="link1" rotor_inertia="4.0e-5"
               peak_torque="1.3" continuous_torque="0.45" stall_torque="3.0"
               no_load_speed="628" torque_constant="0.07" resistance="0.6">
          <origin xyz="0 0.13 0.12" rpy="-1.5708 0 0"/>
          <mass value="0.45"/>
          <geometry><cylinder radius="0.03" length="0.07"/></geometry>
        </motor>
        <gearbox name="j2_gearbox" link="link1" ratio="120" efficiency="1.0"
                 input_inertia="5e-6" peak_torque="150" rated_torque="60"
                 max_input_speed="650">
          <origin xyz="0 0.07 0.12" rpy="-1.5708 0 0"/>
          <mass value="0.55"/>
          <geometry><cylinder radius="0.045" length="0.05"/></geometry>
        </gearbox>
        <transmission ratio="1.0" efficiency="1.0"/>
      </drive>
    </joint>

Each ``<motor>``/``<gearbox>`` is a rigid **lump of mass** bolted to the link
named by ``link=`` at ``<origin>`` (its centre of mass; local +Z is the shaft).
That link does not have to be the joint's parent: putting the motor on the
base and running a cable or linkage to the elbow is just ``link="base_link"``
plus a ``<transmission>`` stage for the cable/linkage reduction. Inertia is
taken from ``<inertia>`` if given, otherwise from ``<geometry>`` scaled to the
declared mass, otherwise the lump is a point mass.

Two mechanisms break the one-motor-one-joint rule, and both are declared
outside the drive:

* **Coupled drives.** A differential gears two motors to two joints at once::

      <drive_coupling name="shoulder" type="differential" joints="j2 j3"/>

  Each motor then carries a share of both joints (and each joint carries both
  rotors' inertia); ``type="matrix"`` with one ``<motor joint= gains=>`` row
  per joint states an arbitrary map. See ``Coupling``.
* **Passive linkages.** A standard URDF ``<mimic>`` makes a joint follow
  another one, the way a palletizer's parallelogram rod keeps the tool plate
  level. A mimic joint is not a degree of freedom, carries no drive and never
  appears in a trajectory; Pinocchio enforces the linkage.

Losses are placeholders for now. There is no joint friction model (a URDF
``<dynamics>`` element is ignored with a warning); instead all losses are to be
lumped into two efficiencies per drive, both defaulting to 1.0 (lossless):

* gearbox ``efficiency``: friction inside the gears;
* transmission ``efficiency``: friction in the joint itself and in any belts,
  cables or linkages between gearbox and joint.

The GUI can override both per joint without touching the file.

Units: SI throughout (kg, m, N*m, rad/s, kg*m^2). Speeds may alternatively be
given in rpm via ``no_load_speed_rpm`` / ``max_input_speed_rpm``. Gearbox
torque ratings are on its **output** side; ``input_inertia`` is on its input
(motor) side, the way datasheets quote them.

Problems that make the file unusable raise ``ValueError``; problems that only
make the analysis less complete (a link with no ``<inertial>``, an actuated
joint with no ``<drive>``) are collected in ``ArmDescription.warnings`` so the
GUI can show them next to the chain.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import numpy as np

from arm_analyzer.mass_properties import (
    MassProperties,
    sphere_radius_for_mass,
    unit_mass_inertia,
)
from arm_analyzer.pin_model import ACTUATED_TYPES, build_model, frame_placements
from arm_analyzer.transforms import make_T, rpy_to_matrix

RPM_TO_RAD_S = 2.0 * math.pi / 60.0

# A drive is reported as remotely located when either lump is mounted on a link
# other than the joint's parent/child, or sits further than this from the
# joint origin at the zero pose.
COLOCATED_TOLERANCE_M = 0.2

DEFAULT_RANGE = {"revolute": (-math.pi, math.pi), "prismatic": (-0.1, 0.1)}


# ----------------------------------------------------------------------------
# XML helpers
# ----------------------------------------------------------------------------


def _floats(raw: Optional[str], count: int, what: str) -> list[float]:
    parts = (raw or "").replace(",", " ").split()
    if not parts:
        return [0.0] * count
    if len(parts) != count:
        raise ValueError(f"{what}: expected {count} numbers, got {raw!r}")
    try:
        return [float(p) for p in parts]
    except ValueError as e:
        raise ValueError(f"{what}: {raw!r} is not numeric") from e


def _origin_T(el: Optional[ET.Element], what: str) -> np.ndarray:
    origin = el.find("origin") if el is not None else None
    if origin is None:
        return np.eye(4)
    xyz = _floats(origin.get("xyz"), 3, f"{what} origin xyz")
    rpy = _floats(origin.get("rpy"), 3, f"{what} origin rpy")
    return make_T(rpy_to_matrix(*rpy), np.array(xyz))


def _attr_float(
    el: Optional[ET.Element],
    name: str,
    what: str,
    default: Optional[float] = None,
    *,
    positive: bool = False,
) -> Optional[float]:
    raw = el.get(name) if el is not None else None
    if raw is None or raw.strip() == "":
        return default
    try:
        v = float(raw)
    except ValueError as e:
        raise ValueError(f"{what}: {name}={raw!r} is not numeric") from e
    if not math.isfinite(v):
        raise ValueError(f"{what}: {name} must be finite")
    if positive and v <= 0:
        raise ValueError(f"{what}: {name} must be positive, got {v}")
    return v


def _speed_attr(el: ET.Element, name: str, what: str) -> Optional[float]:
    """Read ``name`` (rad/s) or ``name_rpm`` (rpm); declaring both is an error."""
    rad = _attr_float(el, name, what, positive=True)
    rpm = _attr_float(el, f"{name}_rpm", what, positive=True)
    if rad is not None and rpm is not None:
        raise ValueError(f"{what}: give {name} or {name}_rpm, not both")
    return rad if rad is not None else (rpm * RPM_TO_RAD_S if rpm is not None else None)


def _inertia_tensor(el: Optional[ET.Element]) -> Optional[np.ndarray]:
    if el is None:
        return None
    g = lambda k: float(el.get(k, "0") or 0.0)  # noqa: E731
    ixx, iyy, izz = g("ixx"), g("iyy"), g("izz")
    ixy, ixz, iyz = g("ixy"), g("ixz"), g("iyz")
    return np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float)


@dataclass
class Shape:
    """One visual primitive, posed in its host link's frame."""

    type: str  # "box" | "cylinder" | "sphere"
    T: np.ndarray
    params: tuple[float, ...]
    color: Optional[list[float]] = None  # rgba from <material><color>

    def to_dict(self) -> dict[str, Any]:
        return {
            "type": self.type,
            "T": self.T.tolist(),
            "params": list(self.params),
            "color": self.color,
        }


def _parse_geometry(geom: Optional[ET.Element], what: str) -> Optional[tuple[str, tuple]]:
    if geom is None:
        return None
    box = geom.find("box")
    if box is not None:
        size = _floats(box.get("size"), 3, f"{what} box size")
        return "box", tuple(size)
    cyl = geom.find("cylinder")
    if cyl is not None:
        return "cylinder", (
            _attr_float(cyl, "radius", what, positive=True),
            _attr_float(cyl, "length", what, positive=True),
        )
    sph = geom.find("sphere")
    if sph is not None:
        return "sphere", (_attr_float(sph, "radius", what, positive=True),)
    # <mesh> is legal URDF but has no closed form here; it is simply not drawn.
    return None


# ----------------------------------------------------------------------------
# Data model
# ----------------------------------------------------------------------------


@dataclass
class Link:
    name: str
    inertial: MassProperties  # structure only, in link frame
    has_inertial: bool
    visuals: list[Shape] = field(default_factory=list)
    tool_tips: list[dict[str, Any]] = field(default_factory=list)


@dataclass
class Mimic:
    """A passive joint driven by another one: ``q = multiplier * q_src + offset``.

    Standard URDF ``<mimic>``. It is how a parallelogram linkage is described
    here -- the rod that keeps a palletizer's tool plate level is not a degree
    of freedom and has no drive; it follows the joint it is linked to.
    """

    joint: str
    multiplier: float = 1.0
    offset: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {"joint": self.joint, "multiplier": self.multiplier, "offset": self.offset}


@dataclass
class Joint:
    name: str
    joint_type: str  # "revolute" | "prismatic" | "fixed"
    parent: str
    child: str
    T_origin: np.ndarray
    axis: np.ndarray
    lower: float = 0.0
    upper: float = 0.0
    velocity: Optional[float] = None  # declared <limit velocity>
    effort: Optional[float] = None  # declared <limit effort>
    mimic: Optional[Mimic] = None  # passive: follows another joint


@dataclass
class Lump:
    """A motor or gearbox body rigidly attached to ``link`` at ``T``."""

    name: str
    kind: str  # "motor" | "gearbox"
    link: str
    T: np.ndarray  # lump frame (COM, +Z = shaft) in host link frame
    mass: float
    inertia: np.ndarray  # about COM, lump frame
    shape: Shape  # in host link frame (display)


@dataclass
class MotorSpec:
    rotor_inertia: float = 0.0
    peak_torque: Optional[float] = None
    continuous_torque: Optional[float] = None
    stall_torque: Optional[float] = None
    no_load_speed: Optional[float] = None
    torque_constant: Optional[float] = None
    resistance: Optional[float] = None

    def torque_limit(self, speed: np.ndarray | float, continuous: bool = False) -> np.ndarray:
        """Available motor torque at ``|speed|`` (rad/s).

        ``min(rating, stall * (1 - w / w0))``: the current limit caps the flat
        part, the DC-motor voltage line caps the rest. With no ``stall_torque``
        the envelope is the rating up to ``no_load_speed``, then zero. Returns
        ``inf`` where nothing is declared.
        """
        w = np.abs(np.asarray(speed, dtype=float))
        rating = self.continuous_torque if continuous else self.peak_torque
        out = np.full(w.shape, math.inf if rating is None else rating, dtype=float)
        if self.no_load_speed:
            w0 = self.no_load_speed
            if self.stall_torque:
                out = np.minimum(out, self.stall_torque * np.clip(1.0 - w / w0, 0.0, None))
            else:
                out = np.where(w <= w0, out, 0.0)
        return out


@dataclass
class GearboxSpec:
    ratio: float = 1.0
    efficiency: float = 1.0  # placeholder for gear friction
    input_inertia: float = 0.0
    peak_torque: Optional[float] = None  # output side
    rated_torque: Optional[float] = None  # output side, continuous
    max_input_speed: Optional[float] = None


@dataclass
class Drive:
    joint: str
    motor: Lump
    motor_spec: MotorSpec
    gearbox: Lump
    gearbox_spec: GearboxSpec
    transmission_ratio: float = 1.0
    # Placeholder for joint + belt/cable/linkage friction.
    transmission_efficiency: float = 1.0
    # Filled in by ArmDescription once the whole tree is known.
    motor_offset: float = 0.0
    gearbox_offset: float = 0.0
    colocated: bool = True

    @property
    def total_ratio(self) -> float:
        """Motor turns per joint turn (or per metre for a prismatic joint)."""
        return self.gearbox_spec.ratio * self.transmission_ratio

    @property
    def total_efficiency(self) -> float:
        return self.gearbox_spec.efficiency * self.transmission_efficiency

    def set_efficiency(
        self, gearbox: Optional[float] = None, transmission: Optional[float] = None
    ) -> None:
        for label, value in (("gearbox", gearbox), ("transmission", transmission)):
            if value is not None:
                _check_efficiency(f"joint {self.joint!r} {label}", value)
        if gearbox is not None:
            self.gearbox_spec.efficiency = float(gearbox)
        if transmission is not None:
            self.transmission_efficiency = float(transmission)

    @property
    def reflected_inertia(self) -> float:
        """Motor-side spinning inertia seen at the joint: ``I * N^2``."""
        n = self.total_ratio
        return (self.motor_spec.rotor_inertia + self.gearbox_spec.input_inertia) * n * n

    def joint_speed_limit(self) -> float:
        """Highest joint speed the drive allows (``inf`` if undeclared)."""
        caps = [math.inf]
        if self.motor_spec.no_load_speed:
            caps.append(self.motor_spec.no_load_speed / self.total_ratio)
        if self.gearbox_spec.max_input_speed:
            caps.append(self.gearbox_spec.max_input_speed / self.total_ratio)
        return min(caps)

    def motor_torque_limit(
        self, motor_speed: np.ndarray | float, continuous: bool = False
    ) -> np.ndarray:
        """Motor-side torque envelope: the motor's own curve, capped by what its
        gearbox can pass (the output rating referred back to the input) and
        zeroed above the gearbox's maximum input speed."""
        w = np.abs(np.asarray(motor_speed, dtype=float))
        gs = self.gearbox_spec
        out = self.motor_spec.torque_limit(w, continuous)
        rating = gs.rated_torque if continuous else gs.peak_torque
        if rating is not None:
            out = np.minimum(out, rating / (gs.ratio * gs.efficiency))
        if gs.max_input_speed:
            out = np.where(w <= gs.max_input_speed, out, 0.0)
        return out

    def joint_torque_limit(
        self, joint_speed: np.ndarray | float, continuous: bool = False
    ) -> np.ndarray:
        """Joint-side torque envelope: motor torque through the total ratio and
        efficiency, capped by the gearbox output rating carried through the
        transmission (and its efficiency). Efficiencies are applied as when
        driving the load, the conservative direction. Coupled drives are
        handled by ``Coupling.joint_torque_limit`` instead."""
        w = np.abs(np.asarray(joint_speed, dtype=float))
        n = self.total_ratio
        return self.motor_torque_limit(w * n, continuous) * n * self.total_efficiency

    def to_dict(self) -> dict[str, Any]:
        def lump(l: Lump) -> dict[str, Any]:
            return {
                "name": l.name,
                "link": l.link,
                "T": l.T.tolist(),
                "mass": l.mass,
                "shape": l.shape.to_dict(),
            }

        m, g = self.motor_spec, self.gearbox_spec
        return {
            "joint": self.joint,
            "motor": {
                **lump(self.motor),
                "rotor_inertia": m.rotor_inertia,
                "peak_torque": m.peak_torque,
                "continuous_torque": m.continuous_torque,
                "stall_torque": m.stall_torque,
                "no_load_speed": m.no_load_speed,
                "torque_constant": m.torque_constant,
                "resistance": m.resistance,
                "offset": self.motor_offset,
            },
            "gearbox": {
                **lump(self.gearbox),
                "ratio": g.ratio,
                "efficiency": g.efficiency,
                "input_inertia": g.input_inertia,
                "peak_torque": g.peak_torque,
                "rated_torque": g.rated_torque,
                "max_input_speed": g.max_input_speed,
                "offset": self.gearbox_offset,
            },
            "transmission": {
                "ratio": self.transmission_ratio,
                "efficiency": self.transmission_efficiency,
            },
            "total_ratio": self.total_ratio,
            "total_efficiency": self.total_efficiency,
            "reflected_inertia": self.reflected_inertia,
            "colocated": self.colocated,
        }


# Barrett-style differential: two motors, two joints. Motor 1 turns with the
# sum of the joint angles, motor 2 with their difference, so pitch is carried by
# both motors together and roll by the two turning against each other.
DIFFERENTIAL = np.array([[1.0, 1.0], [1.0, -1.0]])


@dataclass
class Coupling:
    """Drives whose motors are geared to more than one joint at once.

    ``C`` is dimensionless and says how far each motor turns per unit of each
    joint: motor *i* (the motor of ``joints[i]``) turns
    ``N_i * sum_k C[i, k] * q_k``, with ``N_i`` that drive's own total ratio.
    Writing ``A = diag(N) @ C`` for the full map, everything else follows from
    it and from conservation of power:

    * motor speeds     ``w = A q'``
    * motor torques    ``tau_m = A^-T tau_joint``
    * reflected rotor inertia at the joints  ``A^T diag(I_spin) A``

    A differential therefore splits each joint's load over both motors, and
    each joint carries both rotors' inertia. ``C = [[1]]`` (one motor, one
    joint) reproduces a plain drive exactly.
    """

    name: str
    kind: str  # "differential" | "matrix"
    joints: list[str]
    C: np.ndarray
    drives: list[Drive] = field(default_factory=list)

    @property
    def A(self) -> np.ndarray:
        """Motor turns per joint turn, including each drive's own ratio."""
        return np.diag([d.total_ratio for d in self.drives]) @ self.C

    def motor_speeds(self, QD: np.ndarray) -> np.ndarray:
        """``(N, k)`` joint speeds (this group's joints) to motor speeds."""
        return np.asarray(QD, dtype=float) @ self.A.T

    def motor_torques(self, TAU: np.ndarray) -> np.ndarray:
        """``(N, k)`` joint torques to the motor torques that produce them."""
        return np.asarray(TAU, dtype=float) @ np.linalg.inv(self.A)

    def rotor_matrix(self) -> np.ndarray:
        """Reflected spinning inertia of the group's motors, at the joints."""
        spin = np.array(
            [d.motor_spec.rotor_inertia + d.gearbox_spec.input_inertia for d in self.drives]
        )
        A = self.A
        return A.T @ np.diag(spin) @ A

    def _gains(self, joint: str) -> np.ndarray:
        """Each motor's turns per turn of ``joint``, the others held still."""
        return self.A[:, self.joints.index(joint)]

    def joint_torque_limit(
        self, joint: str, joint_speed: np.ndarray | float, continuous: bool = False
    ) -> np.ndarray:
        """Torque available at ``joint`` with the group's other joints at rest.

        Every motor geared to it contributes, which is why a differential joint
        can be driven harder than either of its motors alone -- but only while
        its partner is unloaded, so this is an upper bound, not a simultaneous
        rating.
        """
        w = np.abs(np.asarray(joint_speed, dtype=float))
        total = np.zeros_like(w, dtype=float)
        for gain, d in zip(self._gains(joint), self.drives):
            if gain == 0.0:
                continue
            total = total + abs(gain) * d.total_efficiency * d.motor_torque_limit(
                abs(gain) * w, continuous
            )
        return total

    def joint_speed_limit(self, joint: str) -> float:
        """Highest speed for ``joint`` alone that no motor in the group exceeds."""
        caps = [math.inf]
        for gain, d in zip(self._gains(joint), self.drives):
            if gain == 0.0:
                continue
            motor_cap = [
                v
                for v in (d.motor_spec.no_load_speed, d.gearbox_spec.max_input_speed)
                if v
            ]
            if motor_cap:
                caps.append(min(motor_cap) / abs(gain))
        return min(caps)

    def partners(self, joint: str) -> list[str]:
        return [n for n in self.joints if n != joint]

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "type": self.kind,
            "joints": list(self.joints),
            "gains": self.C.tolist(),
            "matrix": self.A.tolist(),
        }


@dataclass
class ArmDescription:
    name: str
    root: str
    links: dict[str, Link]
    joints: dict[str, Joint]
    drives: dict[str, Drive]
    warnings: list[str] = field(default_factory=list)
    order: list[str] = field(default_factory=list)  # non-root links, base to tip
    urdf: str = ""  # source text; Pinocchio models are built from it
    couplings: list[Coupling] = field(default_factory=list)

    @property
    def actuated(self) -> list[str]:
        """Actuated joint names, base to tip. Mimic joints are passive."""
        by_child = {j.child: j for j in self.joints.values()}
        return [
            by_child[c].name
            for c in self.order
            if by_child[c].joint_type in ACTUATED_TYPES and by_child[c].mimic is None
        ]

    @property
    def passive(self) -> list[str]:
        """Joints that follow another joint through a linkage (``<mimic>``)."""
        return [n for n, j in self.joints.items() if j.mimic is not None]

    def lumps(self) -> list[Lump]:
        out: list[Lump] = []
        for d in self.drives.values():
            out.extend((d.motor, d.gearbox))
        return out

    def coupling(self, joint: str) -> Optional[Coupling]:
        """The drive coupling ``joint`` belongs to, if any."""
        for c in self.couplings:
            if joint in c.joints:
                return c
        return None

    def joint_torque_limit(
        self, joint: str, joint_speed: np.ndarray | float, continuous: bool = False
    ) -> np.ndarray:
        """Joint-side torque envelope of whatever drives ``joint``."""
        c = self.coupling(joint)
        if c is not None:
            return c.joint_torque_limit(joint, joint_speed, continuous)
        d = self.drives[joint]
        return d.joint_torque_limit(joint_speed, continuous)

    def drive_speed_limit(self, joint: str) -> float:
        """Highest joint speed the drive (or coupled group) allows."""
        c = self.coupling(joint)
        if c is not None:
            return c.joint_speed_limit(joint)
        d = self.drives.get(joint)
        return d.joint_speed_limit() if d is not None else math.inf

    def velocity_limit(self, joint: str) -> float:
        """Declared ``<limit velocity>``, else the drive's own speed limit."""
        j = self.joints[joint]
        caps = [v for v in (j.velocity,) if v]
        limit = self.drive_speed_limit(joint)
        if math.isfinite(limit):
            caps.append(limit)
        return min(caps) if caps else 3.0

    def effort_limit(self, joint: str) -> Optional[float]:
        """Declared ``<limit effort>``, else the drive's peak torque at stall."""
        j = self.joints[joint]
        if j.effort:
            return j.effort
        if joint in self.drives:
            v = float(self.joint_torque_limit(joint, 0.0))
            return v if math.isfinite(v) else None
        return None

    def mass_budget(self) -> dict[str, float]:
        structure = sum(l.inertial.mass for l in self.links.values())
        motors = sum(d.motor.mass for d in self.drives.values())
        gearboxes = sum(d.gearbox.mass for d in self.drives.values())
        return {
            "structure": structure,
            "motors": motors,
            "gearboxes": gearboxes,
            "total": structure + motors + gearboxes,
        }

    def to_dict(self) -> dict[str, Any]:
        """Pose-independent model for the browser (see ``web/kinematics.js``)."""
        return {
            "name": self.name,
            "root_link": self.root,
            "links": [
                {
                    "name": l.name,
                    "mass": l.inertial.mass,
                    "com": l.inertial.com.tolist(),
                    "inertia": l.inertial.inertia.tolist(),
                    "has_inertial": l.has_inertial,
                    "visuals": [v.to_dict() for v in l.visuals],
                    "tool_tips": l.tool_tips,
                }
                for l in (self.links[n] for n in [self.root] + self.order)
            ],
            "joints": [
                {
                    "name": j.name,
                    "type": j.joint_type,
                    "parent": j.parent,
                    "child": j.child,
                    "origin": j.T_origin.tolist(),
                    "axis": j.axis.tolist(),
                    "lower": j.lower,
                    "upper": j.upper,
                    "velocity_limit": self.velocity_limit(j.name)
                    if j.joint_type in ACTUATED_TYPES and j.mimic is None
                    else None,
                    "effort_limit": self.effort_limit(j.name)
                    if j.joint_type in ACTUATED_TYPES and j.mimic is None
                    else None,
                    "mimic": j.mimic.to_dict() if j.mimic is not None else None,
                }
                for j in self.joints.values()
            ],
            "actuated": self.actuated,
            "drives": [self.drives[n].to_dict() for n in self.actuated if n in self.drives],
            "couplings": [c.to_dict() for c in self.couplings],
            "mass_budget": self.mass_budget(),
            "warnings": list(self.warnings),
        }


# ----------------------------------------------------------------------------
# Parsing
# ----------------------------------------------------------------------------


def _parse_link(el: ET.Element, warnings: list[str]) -> Link:
    name = el.get("name")
    inertial_el = el.find("inertial")
    if inertial_el is None:
        warnings.append(f"link {name!r} has no <inertial>; its structure is treated as massless")
        inertial = MassProperties.zero()
    else:
        T = _origin_T(inertial_el, f"link {name!r} inertial")
        mass_el = inertial_el.find("mass")
        mass = _attr_float(mass_el, "value", f"link {name!r} mass", 0.0) or 0.0
        if mass < 0:
            raise ValueError(f"link {name!r}: mass must not be negative")
        I = _inertia_tensor(inertial_el.find("inertia"))
        if I is None:
            warnings.append(f"link {name!r} has mass but no <inertia>; treated as a point mass")
            I = np.zeros((3, 3))
        # URDF: inertia is about the COM in the inertial frame, which may be rotated.
        inertial = MassProperties(mass, np.zeros(3), I).transformed(T)

    visuals: list[Shape] = []
    for v in el.findall("visual"):
        geom = _parse_geometry(v.find("geometry"), f"link {name!r} visual")
        if geom is not None:
            color_el = v.find("material/color")
            color = (
                _floats(color_el.get("rgba"), 4, f"link {name!r} color")
                if color_el is not None
                else None
            )
            visuals.append(
                Shape(geom[0], _origin_T(v, f"link {name!r} visual"), geom[1], color)
            )

    tips = []
    for t in el.findall("tool_tip"):
        xyz = _floats(t.get("xyz"), 3, "tool_tip xyz")
        rpy = _floats(t.get("rpy"), 3, "tool_tip rpy")
        tips.append(
            {
                "name": t.get("name") or "tool_tip",
                "link": name,
                "T": make_T(rpy_to_matrix(*rpy), np.array(xyz)).tolist(),
            }
        )
    return Link(name, inertial, inertial_el is not None, visuals, tips)


def _parse_joint(el: ET.Element) -> Joint:
    name = el.get("name")
    if not name:
        raise ValueError("every <joint> needs a name")
    typ = (el.get("type") or "fixed").lower()
    continuous = typ == "continuous"
    if continuous:
        typ = "revolute"
    if typ not in ("revolute", "prismatic", "fixed"):
        raise ValueError(f"joint {name!r}: unsupported type {typ!r}")
    parent, child = el.find("parent"), el.find("child")
    if parent is None or child is None or not parent.get("link") or not child.get("link"):
        raise ValueError(f"joint {name!r} needs <parent link> and <child link>")

    axis = np.array(
        _floats((el.find("axis").get("xyz") if el.find("axis") is not None else None) or "0 0 1", 3,
                f"joint {name!r} axis"),
        dtype=float,
    )
    n = np.linalg.norm(axis)
    if n < 1e-9:
        raise ValueError(f"joint {name!r}: axis must be non-zero")

    j = Joint(
        name=name,
        joint_type=typ,
        parent=parent.get("link"),
        child=child.get("link"),
        T_origin=_origin_T(el, f"joint {name!r}"),
        axis=axis / n,
    )
    if typ in ACTUATED_TYPES:
        lim = el.find("limit")
        lo, hi = DEFAULT_RANGE[typ]
        if lim is not None and not continuous:
            lo = _attr_float(lim, "lower", f"joint {name!r} limit", lo)
            hi = _attr_float(lim, "upper", f"joint {name!r} limit", hi)
        if hi < lo:
            raise ValueError(f"joint {name!r}: upper limit is below lower limit")
        j.lower, j.upper = lo, hi
        j.velocity = _attr_float(lim, "velocity", f"joint {name!r} limit") or None
        j.effort = _attr_float(lim, "effort", f"joint {name!r} limit") or None
        mim = el.find("mimic")
        if mim is not None:
            src = mim.get("joint")
            if not src:
                raise ValueError(f"joint {name!r}: <mimic> needs joint=")
            j.mimic = Mimic(
                joint=src,
                multiplier=_attr_float(mim, "multiplier", f"joint {name!r} mimic", 1.0),
                offset=_attr_float(mim, "offset", f"joint {name!r} mimic", 0.0),
            )
    return j


def _parse_coupling(
    el: ET.Element, joints: dict[str, Joint], drives: dict[str, Drive]
) -> Coupling:
    name = el.get("name") or "coupling"
    what = f"<drive_coupling {name!r}>"
    kind = (el.get("type") or "differential").lower()
    members = (el.get("joints") or "").replace(",", " ").split()
    if len(members) != len(set(members)):
        raise ValueError(f"{what}: a joint may appear only once")
    for n in members:
        if n not in joints:
            raise ValueError(f"{what}: unknown joint {n!r}")
        if joints[n].mimic is not None:
            raise ValueError(f"{what}: joint {n!r} is passive (<mimic>) and has no drive")
        if n not in drives:
            raise ValueError(f"{what}: joint {n!r} has no <drive> to couple")

    if kind == "differential":
        if len(members) != 2:
            raise ValueError(f"{what}: a differential couples exactly two joints")
        C = DIFFERENTIAL.copy()
    elif kind == "matrix":
        rows = el.findall("motor")
        if len(rows) != len(members) or len(members) < 2:
            raise ValueError(
                f"{what}: needs one <motor joint= gains=> row per coupled joint"
            )
        C = np.zeros((len(members), len(members)))
        for row in rows:
            owner = row.get("joint")
            if owner not in members:
                raise ValueError(f"{what}: <motor joint={owner!r}> is not in joints=")
            C[members.index(owner)] = _floats(
                row.get("gains"), len(members), f"{what} motor {owner!r} gains"
            )
    else:
        raise ValueError(f"{what}: unsupported type {kind!r} (differential | matrix)")

    if abs(np.linalg.det(C)) < 1e-9:
        raise ValueError(f"{what}: the coupling matrix is singular (motors fight each other)")
    return Coupling(
        name=name,
        kind=kind,
        joints=members,
        C=C,
        drives=[drives[n] for n in members],
    )


def _parse_lump(el: ET.Element, kind: str, joint: str, links: dict[str, Link]) -> Lump:
    what = f"joint {joint!r} {kind}"
    host = el.get("link")
    if not host:
        raise ValueError(f"{what}: link= (the link it is mounted on) is required")
    if host not in links:
        raise ValueError(f"{what}: mounted on unknown link {host!r}")
    T = _origin_T(el, what)
    mass = _attr_float(el.find("mass"), "value", f"{what} mass")
    if mass is None or mass <= 0:
        raise ValueError(f"{what}: <mass value> is required and must be positive")

    geom = _parse_geometry(el.find("geometry"), what)
    declared = _inertia_tensor(el.find("inertia"))
    if declared is not None:
        inertia = declared
    elif geom is not None:
        inertia = unit_mass_inertia(*geom) * mass
    else:
        inertia = np.zeros((3, 3))
    if geom is None:
        geom = ("sphere", (sphere_radius_for_mass(mass),))
    return Lump(
        name=el.get("name") or f"{joint}_{kind}",
        kind=kind,
        link=host,
        T=T,
        mass=mass,
        inertia=inertia,
        shape=Shape(geom[0], T, geom[1]),
    )


def _check_efficiency(what: str, value: float) -> None:
    if not (0.0 < float(value) <= 1.0):
        raise ValueError(f"{what}: efficiency must be in (0, 1], got {value}")


def _parse_drive(el: ET.Element, joint: str, links: dict[str, Link]) -> Drive:
    motor_el, gb_el = el.find("motor"), el.find("gearbox")
    if motor_el is None or gb_el is None:
        raise ValueError(f"joint {joint!r}: <drive> needs both <motor> and <gearbox>")
    mw = f"joint {joint!r} motor"
    motor_spec = MotorSpec(
        rotor_inertia=_attr_float(motor_el, "rotor_inertia", mw, 0.0) or 0.0,
        peak_torque=_attr_float(motor_el, "peak_torque", mw, positive=True),
        continuous_torque=_attr_float(motor_el, "continuous_torque", mw, positive=True),
        stall_torque=_attr_float(motor_el, "stall_torque", mw, positive=True),
        no_load_speed=_speed_attr(motor_el, "no_load_speed", mw),
        torque_constant=_attr_float(motor_el, "torque_constant", mw, positive=True),
        resistance=_attr_float(motor_el, "resistance", mw, positive=True),
    )
    if motor_spec.rotor_inertia < 0:
        raise ValueError(f"{mw}: rotor_inertia must not be negative")

    gw = f"joint {joint!r} gearbox"
    gb_spec = GearboxSpec(
        ratio=_attr_float(gb_el, "ratio", gw, 1.0, positive=True),
        efficiency=_attr_float(gb_el, "efficiency", gw, 1.0),
        input_inertia=_attr_float(gb_el, "input_inertia", gw, 0.0) or 0.0,
        peak_torque=_attr_float(gb_el, "peak_torque", gw, positive=True),
        rated_torque=_attr_float(gb_el, "rated_torque", gw, positive=True),
        max_input_speed=_speed_attr(gb_el, "max_input_speed", gw),
    )
    tr = el.find("transmission")
    tw = f"joint {joint!r} transmission"
    t_ratio = _attr_float(tr, "ratio", tw, 1.0, positive=True)
    t_eff = _attr_float(tr, "efficiency", tw, 1.0)
    _check_efficiency(gw, gb_spec.efficiency)
    _check_efficiency(tw, t_eff)

    return Drive(
        joint=joint,
        motor=_parse_lump(motor_el, "motor", joint, links),
        motor_spec=motor_spec,
        gearbox=_parse_lump(gb_el, "gearbox", joint, links),
        gearbox_spec=gb_spec,
        transmission_ratio=t_ratio,
        transmission_efficiency=t_eff,
    )


def _link_order(joints: dict[str, Joint], root: str) -> list[str]:
    """Non-root links, breadth-first from ``root`` in declaration order."""
    children: dict[str, list[str]] = {}
    for j in joints.values():
        children.setdefault(j.parent, []).append(j.child)
    order: list[str] = []
    queue = [root]
    seen = {root}
    while queue:
        for child in children.get(queue.pop(0), []):
            if child not in seen:
                seen.add(child)
                order.append(child)
                queue.append(child)
    return order


def parse_arm(urdf_xml: str) -> ArmDescription:
    """Parse an arm URDF (with ``<drive>`` extensions) into an ``ArmDescription``."""
    try:
        robot = ET.fromstring(urdf_xml)
    except ET.ParseError as e:
        raise ValueError(f"invalid XML: {e}") from e
    if robot.tag != "robot":
        raise ValueError("expected a <robot> root element")

    warnings: list[str] = []
    links: dict[str, Link] = {}
    for el in robot.findall("link"):
        name = el.get("name")
        if not name:
            raise ValueError("every <link> needs a name")
        if name in links:
            raise ValueError(f"duplicate link name {name!r}")
        links[name] = _parse_link(el, warnings)

    joints: dict[str, Joint] = {}
    joint_els: dict[str, ET.Element] = {}
    children: set[str] = set()
    for el in robot.findall("joint"):
        j = _parse_joint(el)
        if j.name in joints:
            raise ValueError(f"duplicate joint name {j.name!r}")
        for ln in (j.parent, j.child):
            if ln not in links:
                raise ValueError(f"joint {j.name!r} references undeclared link {ln!r}")
        if j.child in children:
            raise ValueError(f"link {j.child!r} is the child of more than one joint")
        children.add(j.child)
        joints[j.name] = j
        joint_els[j.name] = el

    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        raise ValueError(f"expected exactly one root link, found {roots}")
    root = roots[0]
    order = _link_order(joints, root)
    unreachable = set(links) - set(order) - {root}
    if unreachable:
        raise ValueError(f"links not connected to {root!r}: {sorted(unreachable)}")

    drives: dict[str, Drive] = {}
    for name, j in joints.items():
        if joint_els[name].find("dynamics") is not None:
            warnings.append(
                f"joint {name!r}: <dynamics> friction is not modelled; "
                "use gearbox / transmission efficiency instead (ignored)"
            )
        if j.mimic is not None:
            src = j.mimic.joint
            if src not in joints:
                raise ValueError(f"joint {name!r}: <mimic> names unknown joint {src!r}")
            if joints[src].mimic is not None:
                raise ValueError(f"joint {name!r}: <mimic> may not follow another mimic joint")
            if joint_els[name].find("drive") is not None:
                raise ValueError(f"joint {name!r} follows {src!r} (<mimic>) and cannot have a drive")
            continue
        drive_el = joint_els[name].find("drive")
        if drive_el is None:
            if j.joint_type in ACTUATED_TYPES:
                warnings.append(
                    f"joint {name!r} has no <drive>; analysed as an ideal, massless actuator"
                )
            continue
        if j.joint_type not in ACTUATED_TYPES:
            raise ValueError(f"joint {name!r} is fixed but declares a <drive>")
        drives[name] = _parse_drive(drive_el, name, links)

    couplings: list[Coupling] = []
    coupled: set[str] = set()
    for el in robot.findall("drive_coupling"):
        c = _parse_coupling(el, joints, drives)
        clash = coupled & set(c.joints)
        if clash:
            raise ValueError(
                f"<drive_coupling {c.name!r}>: {sorted(clash)} already coupled elsewhere"
            )
        coupled |= set(c.joints)
        couplings.append(c)

    arm = ArmDescription(
        name=robot.get("name") or "robot",
        root=root,
        links=links,
        joints=joints,
        drives=drives,
        warnings=warnings,
        order=order,
        urdf=urdf_xml,
        couplings=couplings,
    )

    # Distance of each lump from the joint it drives, at the zero pose. This is
    # what "remotely located" means in the chain view.
    try:
        model = build_model(arm, with_drives=False)
    except Exception as e:  # urdfdom's own validation, beyond what we check
        raise ValueError(f"Pinocchio could not load this URDF: {e}") from e
    q0 = np.zeros(model.nq)
    data, Tw = frame_placements(model, q0)
    for d in drives.values():
        j = joints[d.joint]
        p_joint = data.oMi[model.getJointId(j.name)].translation
        for lump, attr in ((d.motor, "motor_offset"), (d.gearbox, "gearbox_offset")):
            p = (Tw[lump.link] @ lump.T)[:3, 3]
            setattr(d, attr, float(np.linalg.norm(p - p_joint)))
        d.colocated = (
            {d.motor.link, d.gearbox.link} <= {j.parent, j.child}
            and max(d.motor_offset, d.gearbox_offset) <= COLOCATED_TOLERANCE_M
        )
    return arm


def load_arm(path: Path | str) -> ArmDescription:
    return parse_arm(Path(path).read_text(encoding="utf-8"))
