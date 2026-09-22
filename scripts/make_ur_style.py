"""Generate ``examples/robots/ur_style_6dof.urdf``.

A UR5e-style collaborative arm: the **opposite extreme from the WAM**. Every
joint is a self-contained module -- motor, harmonic gear unit, brake and
encoder all inside the joint housing -- so the whole drivetrain rides on the
arm and every joint carries the mass of every joint outboard of it. Nothing is
remote, nothing is coupled; it is the baseline the cable-driven and
base-mounted layouts are worth comparing against.

The other UR signature is the **offset wrist**: J5 is offset from J4 and J6
from J5, instead of three axes meeting at a point. That buys a compact wrist
and a workspace with no shoulder singularity in front of the robot, at the cost
of a closed-form IK that no longer decouples.

Geometry follows the published UR5e link lengths as recalled from the data
sheet -- 163 mm base, 425 mm upper arm, 392 mm forearm, 850 mm reach, about
21 kg, 5 kg payload -- and should be checked against the real datasheet before
relying on it. Masses are split between structure and drive lumps to that
total; motor and gear ratings are estimates sized to the published joint torque
limits (150 N*m inner, 28 N*m wrist) and speeds (180 deg/s inner, 360 deg/s wrist).

Zero pose: arm straight up, +X forward, flange pointing +Y. Losses are
placeholders (efficiency 1.0) as in the other examples.

Run from the project root:

    python scripts/make_ur_style.py
"""

from __future__ import annotations

from pathlib import Path

from urdf_parts import ALONG_Y, drive_xml, g, joint_xml, link_xml

OUT = Path(__file__).resolve().parents[1] / "examples" / "robots" / "ur_style_6dof.urdf"

# Construction of the real robot's motors, which sets their mass (see
# arm_analyzer.motor_mass -- form moves it more than the rating does).
MOTOR_FORM = "integrated"

GEARBOX_EFFICIENCY = 1.0
TRANSMISSION_EFFICIENCY = 1.0

D1 = 0.1625  # base to shoulder
A2 = 0.425  # upper arm
A3 = 0.3922  # forearm
D4 = 0.1333  # wrist 1 offset
D5 = 0.0997  # wrist 2 offset
D6 = 0.0996  # wrist 3 (flange) offset

# name -> (mass, [(shape, params, xyz, axis)])
LINKS = {
    "base_link": (1.8, [("cylinder", (0.075, 0.09), (0, 0, 0.045), "z")]),
    # Shoulder housing: J1's module is inside it.
    "link1": (2.2, [("cylinder", (0.070, 0.12), (0, 0, 0.03), "z")]),
    # Upper arm tube.
    "link2": (4.6, [("cylinder", (0.056, A2), (0, 0.02, A2 / 2), "z")]),
    # Forearm tube.
    "link3": (2.4, [("cylinder", (0.045, A3), (0, 0, A3 / 2), "z")]),
    # Wrist 1 housing.
    "link4": (0.85, [("cylinder", (0.045, 0.10), (0, 0.05, 0), "y")]),
    # Wrist 2 housing.
    "link5": (0.85, [("cylinder", (0.045, 0.10), (0, 0, 0.05), "z")]),
    # Wrist 3 / tool flange.
    "link6": (0.25, [("cylinder", (0.032, 0.03), (0, 0.015, 0), "y")]),
}

COLORS = {"base_link": "0.30 0.33 0.38 1"}

# (mass, radius, length, rotor inertia, peak, continuous, stall, no-load rad/s, kt, R)
# Frameless servo rotors inside each joint module.
MOTOR_JOINT_BIG = (0.038, 0.075, "4.5e-5", 1.6, 0.58, 3.2, 480, 0.24, 1.1)
MOTOR_JOINT_MID = (0.032, 0.065, "2.2e-5", 1.4, 0.58, 2.8, 480, 0.22, 1.6)
MOTOR_JOINT_SML = (0.026, 0.050, "7.0e-6", 0.58, 0.21, 1.2, 620, 0.16, 3.2)

# name, parent, child, origin, axis, lower, upper (deg), velocity (deg/s)
JOINTS = [
    ("j1", "base_link", "link1", (0, 0, D1), "z", -360, 360, 180),
    ("j2", "link1", "link2", (0, 0.0455, 0), "y", -360, 360, 180),
    ("j3", "link2", "link3", (0, -0.0455, A2), "y", -160, 160, 180),
    ("j4", "link3", "link4", (0, 0, A3), "y", -360, 360, 360),
    ("j5", "link4", "link5", (0, D4, 0), "z", -360, 360, 360),
    ("j6", "link5", "link6", (0, 0, D5), "y", -360, 360, 360),
]

# joint -> (motor, gearbox, comment). Co-located: both lumps sit in the joint
# module itself, which is what "collaborative modular arm" means mechanically.
DRIVES = {
    "j1": (
        (MOTOR_JOINT_BIG, "base_link", (0, 0, 0.055), "z"),
        ("link1", (0, 0, 0.0), "z", 101, "8.0e-6", 210, 150, 500, 0.055, 0.035),
        "harmonic gear unit inside the shoulder module",
    ),
    "j2": (
        (MOTOR_JOINT_BIG, "link1", (0, 0.085, 0), "y"),
        ("link2", (0, 0.02, 0.0), "y", 101, "8.0e-6", 210, 150, 500, 0.055, 0.035),
        "harmonic gear unit inside the shoulder-lift module",
    ),
    "j3": (
        (MOTOR_JOINT_MID, "link2", (0, -0.025, A2 - 0.02), "y"),
        ("link3", (0, 0, 0.02), "y", 101, "5.0e-6", 210, 150, 500, 0.048, 0.03),
        "harmonic gear unit inside the elbow module",
    ),
    "j4": (
        (MOTOR_JOINT_SML, "link3", (0, 0.03, A3 - 0.03), "y"),
        ("link4", (0, 0.02, 0), "y", 50, "1.5e-6", 40, 28, 650, 0.04, 0.025),
        "harmonic gear unit inside the wrist-1 module",
    ),
    "j5": (
        (MOTOR_JOINT_SML, "link4", (0, D4 - 0.03, 0.03), "z"),
        ("link5", (0, 0, 0.02), "z", 50, "1.5e-6", 40, 28, 650, 0.04, 0.025),
        "harmonic gear unit inside the wrist-2 module",
    ),
    "j6": (
        (MOTOR_JOINT_SML, "link5", (0, 0.03, D5 - 0.03), "y"),
        ("link6", (0, 0.01, 0), "y", 50, "1.5e-6", 40, 28, 650, 0.035, 0.02),
        "harmonic gear unit inside the wrist-3 module",
    ),
}

TOOL_TIP = {"link6": f'<tool_tip name="flange" xyz="0 {g(D6)} 0" rpy="{ALONG_Y}"/>'}


def main() -> None:
    parts = [
        '<?xml version="1.0"?>\n',
        "<!-- Generated by scripts/make_ur_style.py; edit the script, not this file.\n"
        "     UR5e-style collaborative arm: co-located modular drives at every\n"
        "     joint and an offset wrist. Geometry approximated from public specs;\n"
        "     masses and drive ratings are estimates. -->\n",
        '<robot name="ur_style_6dof">\n',
    ]
    names = list(LINKS)
    for i, link in enumerate(names):
        mass, shapes = LINKS[link]
        parts.append(
            link_xml(
                link,
                mass,
                shapes,
                color=COLORS.get(link, "0.55 0.57 0.60 1"),
                tool_tip=TOOL_TIP.get(link, ""),
            )
        )
        if i >= len(JOINTS):
            continue
        name, parent, child, origin, axis, lo, hi, vel = JOINTS[i]
        motor, gearbox, comment = DRIVES[name]
        body = drive_xml(
            name,
            motor,
            gearbox,
            comment,
            gearbox_name="gear_unit",
            gearbox_efficiency=GEARBOX_EFFICIENCY,
            motor_form=MOTOR_FORM,
            transmission_efficiency=TRANSMISSION_EFFICIENCY,
        )
        parts.append(joint_xml(name, parent, child, origin, axis, lo, hi, vel, body=body))
    parts.append("</robot>\n")
    OUT.write_text("".join(parts), encoding="utf-8")
    print(f"wrote {OUT} (reach {g(A2 + A3 + D4 + D6)} m)")


if __name__ == "__main__":
    main()
