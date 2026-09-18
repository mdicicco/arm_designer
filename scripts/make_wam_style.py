"""Generate ``examples/robots/wam_style_7dof.urdf``.

A Barrett WAM-style 7-DOF arm: **cable driven, with the inner motors in the
base and two differentials**. It is the opposite of a conventional industrial
arm, and it is here to exercise the two mechanisms that go with that:

* **Remote drives.** M1, M2 and M3 sit in the base, below J1, so their 3.9 kg
  never appears as a load on any joint -- it is carried by the floor. M4 rides
  on the shoulder casting just outside J2, and the wrist motors M5-M7 sit at
  the front of the forearm rather than out at the wrist. Cables (modelled as a
  single reduction per drive) carry the torque to the joints.
* **Differentials.** J2/J3 (shoulder pitch and roll) are driven *together* by
  M2 and M3, and J5/J6 (wrist pitch and yaw) by M5 and M6::

      <drive_coupling type="differential" joints="j2 j3"/>

  Each motor then carries half the sum of its two joints' torques (or half
  their difference), which is what makes a differential worth the mechanism:
  shoulder pitch, the heaviest axis, is shared by two motors instead of one.
  In exchange each of those joints carries both rotors' inertia.

Geometry follows the published WAM 7-DOF kinematics as recalled from Barrett's
documentation -- shoulder 346 mm above the base mount, 550 mm upper arm with a
45 mm elbow offset, 300 mm forearm, 60 mm wrist-to-flange, about 1 m reach --
and should be checked against the real datasheet before relying on it. Link
masses, cable ratios and all motor ratings are estimates sized to a ~27 kg,
3 kg-payload arm, not Barrett data.

Zero pose: arm straight up, +X forward. Losses are placeholders (efficiency
1.0) as in the other examples.

Run from the project root:

    python scripts/make_wam_style.py
"""

from __future__ import annotations

import math
from pathlib import Path

OUT = Path(__file__).resolve().parents[1] / "examples" / "robots" / "wam_style_7dof.urdf"

GEARBOX_EFFICIENCY = 1.0
TRANSMISSION_EFFICIENCY = 1.0

ALONG_X = f"0 {math.pi / 2:.6f} 0"
ALONG_Y = f"{-math.pi / 2:.6f} 0 0"
ALONG_Z = "0 0 0"
AXES = {"x": ("1 0 0", ALONG_X), "y": ("0 1 0", ALONG_Y), "z": ("0 0 1", ALONG_Z)}

# Kinematics (m).
BASE_H = 0.346  # base mount to the shoulder point (J1/J2/J3 intersect)
UPPER = 0.550  # shoulder to elbow
ELBOW_OFF = 0.045  # elbow offset, forward of the upper arm axis
FOREARM = 0.300  # elbow to wrist
FLANGE = 0.060  # wrist to flange


def g(v: float) -> str:
    return f"{v:.6g}"


def deg(v: float) -> float:
    return math.radians(v)


# ---------------------------------------------------------------------------
# Links: name -> (mass, [(shape, params, xyz, axis)]). The first shape sets the
# inertia (primitive at the link's mass, centred on its xyz).
# ---------------------------------------------------------------------------
LINKS = {
    # The base shell holds M1-M3 and their capstans; it never moves.
    "base_link": (6.0, [("cylinder", (0.13, 0.30), (0, 0, 0.15), "z")]),
    # Yaw column up to the shoulder point.
    "link1": (4.0, [("cylinder", (0.10, 0.30), (0, 0, 0.19), "z")]),
    # Shoulder casting (pitch), carries M4.
    "link2": (3.0, [("box", (0.16, 0.20, 0.14), (0, 0, 0), "z")]),
    # Upper arm tube (roll axis), shoulder to elbow.
    "link3": (2.5, [("cylinder", (0.055, UPPER), (0, 0, UPPER / 2), "z")]),
    # Forearm: elbow to wrist, with the wrist motor pack at its inner end.
    "link4": (2.2, [("cylinder", (0.048, FOREARM), (0, 0, FOREARM / 2), "z")]),
    # Wrist pitch housing.
    "link5": (0.55, [("box", (0.08, 0.07, 0.07), (0, 0, 0), "z")]),
    # Wrist yaw yoke.
    "link6": (0.35, [("box", (0.06, 0.06, 0.06), (0, 0, 0), "z")]),
    # Tool flange (roll).
    "link7": (0.15, [("cylinder", (0.03, 0.02), (0, 0, 0.01), "z")]),
}

COLORS = {"base_link": "0.30 0.33 0.38 1"}

# Motors: (mass, radius, length, rotor inertia, peak, continuous, stall,
#          no-load rad/s, kt, R). Frameless brushless rotors, as a WAM uses.
MOTOR_INNER = (1.30, 0.042, 0.10, "6.0e-5", 2.0, 0.95, 4.0, 314, 0.28, 0.9)
MOTOR_ELBOW = (1.00, 0.038, 0.09, "4.0e-5", 1.6, 0.50, 3.2, 314, 0.26, 1.2)
MOTOR_WRIST = (0.45, 0.028, 0.07, "8.0e-6", 0.60, 0.20, 1.2, 419, 0.18, 3.0)

# Joints: name, parent, child, origin xyz, axis, lower, upper (deg), velocity (deg/s)
JOINTS = [
    ("j1", "base_link", "link1", (0, 0, 0.04), "z", -150, 150, 150),
    ("j2", "link1", "link2", (0, 0, BASE_H - 0.04), "y", -113, 113, 150),
    ("j3", "link2", "link3", (0, 0, 0), "z", -157, 157, 180),
    ("j4", "link3", "link4", (ELBOW_OFF, 0, UPPER), "y", -50, 180, 180),
    ("j5", "link4", "link5", (-ELBOW_OFF, 0, FOREARM), "z", -275, 75, 300),
    ("j6", "link5", "link6", (0, 0, 0), "y", -90, 90, 300),
    ("j7", "link6", "link7", (0, 0, 0), "z", -172, 172, 300),
]

# Drives: joint -> (motor spec, motor host, motor xyz, motor axis),
#                  (gearbox host, xyz, axis, ratio, input inertia, peak out,
#                   rated out, max input rad/s, mass, radius, length),
#                  what the cable run does.
#
# "gearbox" here is the capstan stage at the motor; the cable to the joint is
# the transmission. Ratios are the WAM's published cable reductions.
DRIVES = {
    "j1": (
        (MOTOR_INNER, "base_link", (0.09, 0, 0.10), "z"),
        ("base_link", (0.09, 0, 0.20), "z", 42, "8.0e-6", 90, 40, 340, 0.60, 0.05, 0.05),
        "cable from the base capstan up to the J1 pulley",
    ),
    # Shoulder differential: M2 and M3 both in the base, both feeding it.
    "j2": (
        (MOTOR_INNER, "base_link", (-0.065, 0.075, 0.10), "z"),
        ("base_link", (-0.065, 0.075, 0.20), "z", 28.25, "8.0e-6", 90, 40, 340, 0.60, 0.05, 0.05),
        "cable to the shoulder differential (with M3)",
    ),
    "j3": (
        (MOTOR_INNER, "base_link", (-0.065, -0.075, 0.10), "z"),
        ("base_link", (-0.065, -0.075, 0.20), "z", 28.25, "8.0e-6", 90, 40, 340, 0.60, 0.05, 0.05),
        "cable to the shoulder differential (with M2)",
    ),
    "j4": (
        (MOTOR_ELBOW, "link2", (-0.10, 0, 0.02), "z"),
        ("link2", (-0.05, 0, 0.06), "z", 18, "6.0e-6", 60, 26, 340, 0.45, 0.045, 0.045),
        "cable down the upper arm to the elbow pulley",
    ),
    # Wrist differential: M5 and M6 at the inner end of the forearm.
    "j5": (
        (MOTOR_WRIST, "link4", (0.055, 0.035, 0.05), "z"),
        ("link4", (0.03, 0.035, 0.11), "z", 9.7, "1.0e-6", 12, 5, 460, 0.18, 0.03, 0.03),
        "cable along the forearm to the wrist differential (with M6)",
    ),
    "j6": (
        (MOTOR_WRIST, "link4", (0.055, -0.035, 0.05), "z"),
        ("link4", (0.03, -0.035, 0.11), "z", 9.7, "1.0e-6", 12, 5, 460, 0.18, 0.03, 0.03),
        "cable along the forearm to the wrist differential (with M5)",
    ),
    "j7": (
        (MOTOR_WRIST, "link4", (-0.055, 0, 0.05), "z"),
        ("link5", (0, 0, 0.02), "z", 14.93, "1.0e-6", 12, 5, 460, 0.15, 0.025, 0.025),
        "cable through the wrist to the tool roll pulley",
    ),
}

COUPLINGS = [
    ("shoulder_differential", ("j2", "j3"),
     "M2 and M3 drive shoulder pitch and roll together"),
    ("wrist_differential", ("j5", "j6"),
     "M5 and M6 drive wrist pitch and yaw together"),
]


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
    if name == "link7":
        parts.append(f'    <tool_tip name="flange" xyz="0 0 {g(FLANGE)}"/>')
    parts.append("  </link>")
    return "\n".join(parts) + "\n"


def drive_xml(joint: str) -> str:
    (spec, m_host, m_xyz, m_axis), gb, comment = DRIVES[joint]
    mass, radius, length, rotor, peak, cont, stall, nl, kt, res = spec
    gb_host, gb_xyz, gb_axis, ratio, gb_in, gb_peak, gb_rated, gb_max, gb_mass, gb_r, gb_l = gb
    return f"""    <drive>
      <motor name="{joint}_motor" link="{m_host}" rotor_inertia="{rotor}"
             peak_torque="{g(peak)}" continuous_torque="{g(cont)}"
             stall_torque="{g(stall)}" no_load_speed="{g(nl)}"
             torque_constant="{g(kt)}" resistance="{g(res)}">
        <origin xyz="{xyz(m_xyz)}" rpy="{AXES[m_axis][1]}"/>
        <mass value="{g(mass)}"/>
        <geometry><cylinder radius="{g(radius)}" length="{g(length)}"/></geometry>
      </motor>
      <gearbox name="{joint}_capstan" link="{gb_host}" ratio="{g(ratio)}" efficiency="{GEARBOX_EFFICIENCY:.2f}"
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
        "<!-- Generated by scripts/make_wam_style.py; edit the script, not this file.\n"
        "     Barrett WAM-style 7-DOF cable arm: M1-M3 in the base, shoulder and\n"
        "     wrist differentials. Geometry approximated from public specs;\n"
        "     masses, cable ratios and drive ratings are estimates. -->\n",
        '<robot name="wam_style_7dof">\n',
    ]
    names = list(LINKS)
    for i, link in enumerate(names):
        parts.append(link_xml(link))
        if i < len(JOINTS):
            parts.append(joint_xml(JOINTS[i]))
    for name, joints, comment in COUPLINGS:
        parts.append(f"  <!-- {comment} -->\n")
        parts.append(
            f'  <drive_coupling name="{name}" type="differential" '
            f'joints="{" ".join(joints)}"/>\n'
        )
    parts.append("</robot>\n")
    OUT.write_text("".join(parts), encoding="utf-8")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
