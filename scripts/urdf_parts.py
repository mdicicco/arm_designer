"""Shared URDF text helpers for the example-robot generators.

The generators describe a robot as plain data (links, joints, drives) and this
module turns that into the URDF dialect ``arm_analyzer.robot`` reads. It is
deliberately small: primitives only, one inertia per link, and no attempt to be
a general URDF writer.

``make_simple_6dof.py``, ``make_kr_style.py`` and ``make_wam_style.py`` predate
it and carry their own ``drive_xml``; the newer generators share these. All of
them take a motor's mass from :func:`motor_body_for` and a gearbox's from
:func:`gearbox_body_for`.
"""

from __future__ import annotations

import math

from arm_analyzer.gearbox_mass import bulk_density as gearbox_bulk_density
from arm_analyzer.gearbox_mass import estimate_mass as estimate_gearbox_mass
from arm_analyzer.motor_mass import bulk_density as motor_bulk_density
from arm_analyzer.motor_mass import estimate_mass

ALONG_X = f"0 {math.pi / 2:.6f} 0"  # rpy turning a primitive's +Z onto +X
ALONG_Y = f"{-math.pi / 2:.6f} 0 0"  # rpy turning a primitive's +Z onto +Y
ALONG_Z = "0 0 0"
DOWN = f"{math.pi:.6f} 0 0"  # rpy turning +Z onto -Z
AXES = {"x": ("1 0 0", ALONG_X), "y": ("0 1 0", ALONG_Y), "z": ("0 0 1", ALONG_Z)}

DEFAULT_COLOR = "0.55 0.57 0.60 1"


def motor_mass_for(peak_torque: float, form: str = "frameless") -> float:
    """The mass a ``form`` motor of this rating gets in the examples.

    Torque is the **defined** parameter in these files and mass follows from
    it, so no example declares the two independently and they cannot drift
    apart. ``form`` carries the construction, which moves the answer more than
    the rating does -- an integrated servo is ~3.8x an outrunner at the same
    torque. See ``arm_analyzer.motor_mass``.
    """
    return estimate_mass(peak_torque, form)


def gearbox_body_for(
    rated_torque: float,
    ratio: float,
    radius: float,
    length: float,
    kind: str = "harmonic",
) -> tuple[float, float, float]:
    """``(mass, radius, length)`` for a reducer rated to ``rated_torque`` out.

    The same treatment the motors get: mass from the rating, then the housing
    resized to hold it at the bulk density measured for that type of reducer
    (a cycloidal frame is mostly housing, a planetary mostly steel, so this is
    per-type where the motors' was a single number). ``radius`` and ``length``
    give only the proportions.

    Ratio is passed through for completeness; the fit says it barely matters.
    """
    mass = estimate_gearbox_mass(rated_torque, ratio, kind)
    volume = mass / gearbox_bulk_density(kind)
    have = math.pi * radius * radius * length
    scale = (volume / have) ** (1.0 / 3.0)
    return mass, radius * scale, length * scale


def motor_body_for(
    peak_torque: float, radius: float, length: float, form: str = "frameless"
) -> tuple[float, float, float]:
    """``(mass, radius, length)`` for a ``form`` motor rated to ``peak_torque``.

    The mass comes from the rating and the form; the cylinder is then resized
    to hold it at that form's measured bulk density, scaling radius and length
    by the same factor so the motor keeps the proportions the generator drew it
    with -- a pancake stays a pancake. ``radius`` and ``length`` are therefore
    a *shape*, not a size, and only their ratio survives.

    Sizing it matters beyond looks: a lump with no ``<inertia>`` takes its
    inertia tensor from its geometry scaled to its mass, so an envelope too big
    for the mass would hand the motor an inertia it does not have.
    """
    mass = motor_mass_for(peak_torque, form)
    volume = mass / motor_bulk_density(form)
    have = math.pi * radius * radius * length
    scale = (volume / have) ** (1.0 / 3.0)
    return mass, radius * scale, length * scale


def g(v: float) -> str:
    return f"{v:.6g}"


def deg(v: float) -> float:
    return math.radians(v)


def xyz(v) -> str:
    return " ".join(g(c) for c in v)


def inertia(shape: str, params: tuple, mass: float) -> tuple[float, float, float]:
    """Principal inertia of a primitive whose own axis is +Z."""
    if shape == "box":
        x, y, z = params
        return (
            mass * (y * y + z * z) / 12,
            mass * (x * x + z * z) / 12,
            mass * (x * x + y * y) / 12,
        )
    r, h = params
    radial = mass * (3 * r * r + h * h) / 12
    return (radial, radial, mass * r * r / 2)


def geometry(shape: str, params: tuple) -> str:
    if shape == "box":
        return f'<box size="{" ".join(g(p) for p in params)}"/>'
    return f'<cylinder radius="{g(params[0])}" length="{g(params[1])}"/>'


def link_xml(
    name: str,
    mass: float,
    shapes: list[tuple],
    *,
    color: str = DEFAULT_COLOR,
    tool_tip: str = "",
) -> str:
    """One ``<link>``; the first shape also defines the inertia."""
    shape, params, center, axis = shapes[0]
    ixx, iyy, izz = inertia(shape, params, mass)
    if shape == "cylinder":
        radial, axial = ixx, izz
        ixx, iyy, izz = (
            (axial, radial, radial) if axis == "x" else
            (radial, axial, radial) if axis == "y" else
            (radial, radial, axial)
        )
    parts = [
        f'  <link name="{name}">',
        "    <inertial>",
        f'      <origin xyz="{xyz(center)}" rpy="0 0 0"/>',
        f'      <mass value="{g(mass)}"/>',
        f'      <inertia ixx="{g(ixx)}" ixy="0" ixz="0" iyy="{g(iyy)}" iyz="0" izz="{g(izz)}"/>',
        "    </inertial>",
    ]
    for shape, params, center, axis in shapes:
        rpy = AXES[axis][1] if shape == "cylinder" else "0 0 0"
        parts += [
            "    <visual>",
            f'      <origin xyz="{xyz(center)}" rpy="{rpy}"/>',
            f"      <geometry>{geometry(shape, params)}</geometry>",
            f'      <material name="{name}_mat"><color rgba="{color}"/></material>',
            "    </visual>",
        ]
    if tool_tip:
        parts.append(f"    {tool_tip}")
    parts.append("  </link>")
    return "\n".join(parts) + "\n"


def drive_xml(
    joint: str,
    motor: tuple,
    gearbox: tuple,
    comment: str,
    *,
    gearbox_name: str = "gearbox",
    motor_form: str = "frameless",
    gearbox_efficiency: float = 1.0,
    transmission_ratio: float = 1.0,
    transmission_efficiency: float = 1.0,
) -> str:
    """``<drive>`` for one joint.

    ``motor``   = ((radius, length, rotor inertia, peak, continuous, stall,
                   no-load rad/s, kt, R), host link, xyz, shaft axis) -- its
                  mass comes from ``peak`` and the no-load speed, and radius
                  and length give only its proportions (see
                  :func:`motor_body_for`)
    ``gearbox`` = (host link, xyz, axis, ratio, input inertia, peak out,
                   rated out, max input rad/s, radius, length) -- its mass
                  comes from the rated output torque and its type, and radius
                  and length give only its proportions
    """
    (spec, m_host, m_xyz, m_axis) = motor
    radius, length, rotor, peak, cont, stall, nl, kt, res = spec
    mass, radius, length = motor_body_for(peak, radius, length, motor_form)
    gb_host, gb_xyz, gb_axis, ratio, gb_in, gb_peak, gb_rated, gb_max, gb_r, gb_l = gearbox
    gb_mass, gb_r, gb_l = gearbox_body_for(gb_rated, ratio, gb_r, gb_l)
    return f"""    <drive>
      <motor name="{joint}_motor" link="{m_host}" form="{motor_form}" rotor_inertia="{rotor}"
             peak_torque="{g(peak)}" continuous_torque="{g(cont)}"
             stall_torque="{g(stall)}" no_load_speed="{g(nl)}"
             torque_constant="{g(kt)}" resistance="{g(res)}">
        <origin xyz="{xyz(m_xyz)}" rpy="{AXES[m_axis][1]}"/>
        <mass value="{g(mass)}"/>
        <geometry><cylinder radius="{g(radius)}" length="{g(length)}"/></geometry>
      </motor>
      <gearbox name="{joint}_{gearbox_name}" link="{gb_host}" ratio="{g(ratio)}" efficiency="{gearbox_efficiency:.2f}"
               input_inertia="{gb_in}" peak_torque="{g(gb_peak)}" rated_torque="{g(gb_rated)}"
               max_input_speed="{g(gb_max)}">
        <origin xyz="{xyz(gb_xyz)}" rpy="{AXES[gb_axis][1]}"/>
        <mass value="{g(gb_mass)}"/>
        <geometry><cylinder radius="{g(gb_r)}" length="{g(gb_l)}"/></geometry>
      </gearbox>
      <!-- {comment} -->
      <transmission ratio="{g(transmission_ratio)}" efficiency="{transmission_efficiency:.2f}"/>
    </drive>
"""


def joint_xml(
    name: str,
    parent: str,
    child: str,
    origin,
    axis: str,
    lo: float,
    hi: float,
    vel: float,
    *,
    body: str = "",
) -> str:
    """One revolute ``<joint>``; limits are given in degrees and deg/s."""
    return f"""  <joint name="{name}" type="revolute">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="{xyz(origin)}" rpy="0 0 0"/>
    <axis xyz="{AXES[axis][0]}"/>
    <!-- range {lo}..{hi} deg, {vel} deg/s; effort is derived from the drive -->
    <limit lower="{g(deg(lo))}" upper="{g(deg(hi))}" velocity="{g(deg(vel))}"/>
{body}  </joint>
"""


def mimic_xml(joint: str, multiplier: float = 1.0, offset: float = 0.0) -> str:
    """``<mimic>`` body for a passive linkage joint (no drive)."""
    return (
        f'    <mimic joint="{joint}" multiplier="{g(multiplier)}" offset="{g(offset)}"/>\n'
    )
