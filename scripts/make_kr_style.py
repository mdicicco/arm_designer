"""Generate ``examples/robots/kr_style_6dof.urdf``.

A KUKA KR-series-style arm: 6-axis, in-line wrist, with the **A4-A6 wrist
motors packed at the rear of the arm housing, behind the elbow (A3)**, where
they also counterbalance the forearm. The motors drive their gear units through
shafts inside the forearm: the A4 gearbox sits at the front of the arm housing,
the A5 gearbox in the wrist, the A6 gearbox at the flange.

Geometry follows the published KR 6 R900 (KR AGILUS) class as recalled from its
spec sheet -- A2 400 mm above the floor, 25 mm shoulder offset, 455 mm upper
arm, 35 mm elbow offset, 420 mm forearm, 80 mm to the flange, ~900 mm reach,
~52 kg -- and should be checked against the datasheet before relying on it.
Link masses, inertias and all motor / gearbox ratings are estimates sized to
that class, not KUKA data. Joint ranges are the KR 6 R900's, re-expressed for
this file's zero pose.

Zero pose: upper arm vertical, forearm horizontal pointing +X (KUKA's usual
home posture). All joints rotate about +Z (A1), +Y (A2, A3, A5) or +X (A4, A6).
Losses are placeholders (efficiency 1.0) as in the other examples.

Run from the project root:

    python scripts/make_kr_style.py
"""

from __future__ import annotations

import math
from pathlib import Path

from urdf_parts import gearbox_body_for, motor_body_for

OUT = Path(__file__).resolve().parents[1] / "examples" / "robots" / "kr_style_6dof.urdf"

# Construction of the real robot's motors, which sets their mass (see
# arm_analyzer.motor_mass -- form moves it more than the rating does).
MOTOR_FORM = "industrial"

GEARBOX_EFFICIENCY = 1.0
TRANSMISSION_EFFICIENCY = 1.0

ALONG_X = f"0 {math.pi / 2:.6f} 0"  # rpy turning a primitive's +Z onto +X
ALONG_Y = f"{-math.pi / 2:.6f} 0 0"  # rpy turning a primitive's +Z onto +Y
ALONG_Z = "0 0 0"
AXES = {"x": ("1 0 0", ALONG_X), "y": ("0 1 0", ALONG_Y), "z": ("0 0 1", ALONG_Z)}


def g(v: float) -> str:
    return f"{v:.6g}"


def deg(v: float) -> float:
    return math.radians(v)


# ---------------------------------------------------------------------------
# Links: name -> (mass, [(shape, params, xyz, axis)])  -- the first shape also
# defines the inertia (primitive at the link's mass, centred on its xyz).
# ---------------------------------------------------------------------------
LINKS = {
    "base_link": (12.0, [("cylinder", (0.10, 0.20), (0, 0, 0.10), "z")]),
    # Rotating column: carries the shoulder, offset 25 mm forward.
    "link1": (8.0, [("cylinder", (0.09, 0.20), (0, 0, 0.10), "z"),
                    ("box", (0.12, 0.16, 0.08), (0.025, 0, 0.18), "z")]),
    # Upper arm (rocker), 455 mm.
    "link2": (6.0, [("box", (0.09, 0.07, 0.455), (0, 0, 0.2275), "z")]),
    # Arm housing around the elbow; extends back to the wrist motor pack.
    "link3": (4.0, [("box", (0.30, 0.10, 0.10), (-0.03, 0, 0.035), "x")]),
    # Forearm tube from the A4 gear unit to the wrist.
    "link4": (1.8, [("cylinder", (0.04, 0.30), (0.15, 0, 0), "x")]),
    # In-line wrist housing (A5).
    "link5": (0.9, [("box", (0.07, 0.06, 0.07), (0.02, 0, 0), "x")]),
    # Flange.
    "link6": (0.2, [("cylinder", (0.032, 0.02), (0.01, 0, 0), "x")]),
}

COLORS = {"base_link": "0.30 0.33 0.38 1"}

# Motors: (mass, radius, length, rotor inertia, peak, continuous, stall, no-load rad/s, kt, R)
MOTOR_750W = (0.045, 0.14, "1.0e-4", 7.0, 2.4, 14.0, 838, 0.45, 0.8)
MOTOR_400W = (0.035, 0.12, "3.0e-5", 3.2, 1.1, 6.0, 838, 0.35, 1.6)
MOTOR_200W = (0.028, 0.10, "1.0e-5", 1.3, 0.45, 3.0, 838, 0.25, 3.0)
MOTOR_100W = (0.024, 0.09, "5.0e-6", 0.64, 0.22, 1.6, 838, 0.20, 5.0)

# Joints: name, parent, child, origin xyz, axis, lower, upper (deg), velocity (deg/s)
JOINTS = [
    ("j1", "base_link", "link1", (0, 0, 0.20), "z", -170, 170, 360),
    ("j2", "link1", "link2", (0.025, 0, 0.20), "y", -100, 135, 300),
    ("j3", "link2", "link3", (0, 0, 0.455), "y", -156, 120, 360),
    ("j4", "link3", "link4", (0.12, 0, 0.035), "x", -185, 185, 381),
    ("j5", "link4", "link5", (0.30, 0, 0), "y", -120, 120, 388),
    ("j6", "link5", "link6", (0.06, 0, 0), "x", -350, 350, 615),
]

# Drives: joint -> motor (spec, host, xyz, shaft axis), gearbox (host, xyz, axis,
# ratio, input inertia, peak, rated, max input rad/s, mass, radius, length),
# transmission comment.
DRIVES = {
    "j1": (
        (MOTOR_750W, "base_link", (-0.06, 0, 0.075), "z"),
        ("base_link", (0, 0, 0.16), "z", 120, "1.0e-5", 350, 150, 900, 0.08, 0.06),
        "joint elements (gear unit drives the column directly)",
    ),
    "j2": (
        (MOTOR_750W, "link1", (0.025, -0.19, 0.20), "y"),
        ("link1", (0.025, -0.10, 0.20), "y", 120, "1.0e-5", 350, 150, 900, 0.07, 0.05),
        "joint elements (gear unit drives the rocker directly)",
    ),
    "j3": (
        (MOTOR_400W, "link3", (-0.02, 0.17, 0.0), "y"),
        ("link3", (0, 0.08, 0), "y", 100, "5.0e-6", 200, 90, 900, 0.06, 0.04),
        "joint elements (A3 motor and gear unit ride on the arm housing at the elbow)",
    ),
    # Wrist motor pack at the rear of the arm housing, behind the elbow.
    "j4": (
        (MOTOR_200W, "link3", (-0.14, 0, 0.075), "x"),
        ("link3", (0.10, 0, 0.035), "x", 80, "2.0e-6", 60, 25, 1000, 0.045, 0.04),
        "drive shaft from the rear motor pack to the A4 gear unit",
    ),
    "j5": (
        (MOTOR_200W, "link3", (-0.14, 0.035, 0.0), "x"),
        ("link4", (0.28, 0, 0), "y", 80, "2.0e-6", 60, 25, 1000, 0.035, 0.03),
        "hollow shaft through the forearm + bevel stage to the A5 gear unit in the wrist",
    ),
    "j6": (
        (MOTOR_100W, "link3", (-0.14, -0.035, 0.0), "x"),
        ("link5", (0.04, 0, 0), "x", 50, "1.0e-6", 20, 8, 1200, 0.03, 0.03),
        "concentric shaft through the forearm and wrist to the A6 gear unit at the flange",
    ),
}


def inertia(shape: str, params: tuple, mass: float) -> tuple[float, float, float]:
    """Principal inertia of a primitive whose own axis is +Z."""
    if shape == "box":
        x, y, z = params
        return (mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12, mass * (x * x + y * y) / 12)
    r, h = params
    radial = mass * (3 * r * r + h * h) / 12
    return (radial, radial, mass * r * r / 2)


def geometry(shape: str, params: tuple) -> str:
    if shape == "box":
        return f'<box size="{" ".join(g(p) for p in params)}"/>'
    return f'<cylinder radius="{g(params[0])}" length="{g(params[1])}"/>'


def xyz(v) -> str:
    return " ".join(g(c) for c in v)


def link_xml(name: str) -> str:
    mass, shapes = LINKS[name]
    shape, params, center, axis = shapes[0]
    # Boxes are authored in link axes already; cylinders are turned onto `axis`.
    ixx, iyy, izz = inertia(shape, params, mass)
    if shape == "cylinder":
        # Rotate the principal moments with the cylinder (its own Z = link `axis`).
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
    # Neutral grey: the viewer already uses orange for gearboxes.
    color = COLORS.get(name, "0.55 0.57 0.60 1")
    for shape, params, center, axis in shapes:
        rpy = AXES[axis][1] if shape == "cylinder" else "0 0 0"
        parts += [
            "    <visual>",
            f'      <origin xyz="{xyz(center)}" rpy="{rpy}"/>',
            f"      <geometry>{geometry(shape, params)}</geometry>",
            f'      <material name="{name}_mat"><color rgba="{color}"/></material>',
            "    </visual>",
        ]
    if name == "link6":
        parts.append(f'    <tool_tip name="flange" xyz="0.02 0 0" rpy="{ALONG_X}"/>')
    parts.append("  </link>")
    return "\n".join(parts) + "\n"


def drive_xml(joint: str) -> str:
    (spec, m_host, m_xyz, m_axis), gb, comment = DRIVES[joint]
    radius, length, rotor, peak, cont, stall, nl, kt, res = spec
    mass, radius, length = motor_body_for(peak, radius, length, MOTOR_FORM)
    gb_host, gb_xyz, gb_axis, ratio, gb_in, gb_peak, gb_rated, gb_max, gb_r, gb_l = gb
    gb_mass, gb_r, gb_l = gearbox_body_for(gb_rated, ratio, gb_r, gb_l)
    return f"""    <drive>
      <motor name="{joint}_motor" link="{m_host}" form="{MOTOR_FORM}" rotor_inertia="{rotor}"
             peak_torque="{g(peak)}" continuous_torque="{g(cont)}"
             stall_torque="{g(stall)}" no_load_speed="{g(nl)}"
             torque_constant="{g(kt)}" resistance="{g(res)}">
        <origin xyz="{xyz(m_xyz)}" rpy="{AXES[m_axis][1]}"/>
        <mass value="{g(mass)}"/>
        <geometry><cylinder radius="{g(radius)}" length="{g(length)}"/></geometry>
      </motor>
      <gearbox name="{joint}_gearbox" link="{gb_host}" ratio="{g(ratio)}" efficiency="{GEARBOX_EFFICIENCY:.2f}"
               input_inertia="{gb_in}" peak_torque="{g(gb_peak)}" rated_torque="{g(gb_rated)}"
               max_input_speed="{g(gb_max)}">
        <origin xyz="{xyz(gb_xyz)}" rpy="{AXES[gb_axis][1]}"/>
        <mass value="{g(gb_mass)}"/>
        <geometry><cylinder radius="{g(gb_r)}" length="{g(gb_l)}"/></geometry>
      </gearbox>
      <!-- {comment} -->
      <transmission ratio="1" efficiency="{TRANSMISSION_EFFICIENCY:.2f}"/>
    </drive>
"""


def joint_xml(j) -> str:
    name, parent, child, origin, axis, lo, hi, vel = j
    return f"""  <joint name="{name}" type="revolute">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="{xyz(origin)}" rpy="0 0 0"/>
    <axis xyz="{AXES[axis][0]}"/>
    <!-- range {lo}..{hi} deg, {vel} deg/s; effort is derived from the drive -->
    <limit lower="{g(deg(lo))}" upper="{g(deg(hi))}" velocity="{g(deg(vel))}"/>
{drive_xml(name)}  </joint>
"""


def main() -> None:
    parts = [
        '<?xml version="1.0"?>\n',
        "<!-- Generated by scripts/make_kr_style.py; edit the script, not this file.\n"
        "     KUKA KR-series-style layout (wrist motors behind the elbow) at KR 6 R900 scale.\n"
        "     Geometry approximated from public specs; masses and drive ratings are estimates. -->\n",
        '<robot name="kr_style_6dof">\n',
    ]
    names = list(LINKS)
    for i, link in enumerate(names):
        parts.append(link_xml(link))
        if i < len(JOINTS):
            parts.append(joint_xml(JOINTS[i]))
    parts.append("</robot>\n")
    OUT.write_text("".join(parts), encoding="utf-8")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
