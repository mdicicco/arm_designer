"""Generate ``examples/robots/palletizer_4dof.urdf``.

A palletizer in the ABB IRB 460 mould, scaled down to a 0.95 m reach and a
20 kg payload so it works the same pick-and-place trajectory as the other
examples. Two things make it worth having:

* **All three heavy motors are in the base.** J1's drives the column directly;
  J2 and J3 push the arm through rods running up the outside of it, so the
  shoulder and elbow motors (13 kg of motor and gearbox between them) sit on
  the pedestal instead of out on the arm. Only the small tool-rotate motor
  rides along, at the elbow.
* **Two parallelogram linkages**, written as standard URDF ``<mimic>`` joints.
  ``level_elbow`` cancels J3 and ``level_shoulder`` cancels J2, so the tool
  plate stays level whatever the arm does -- the reason a palletizer needs no
  wrist pitch. They are passive: no drive, no degree of freedom, and they never
  appear in a trajectory. What they do carry is mass and inertia.

So this arm has 4 actuated joints, not 6: yaw, shoulder, elbow, tool rotate. It
can follow any path whose tool points straight down (the default pick-and-place
trajectory does), and cannot follow one that tips the tool over, which is
exactly the trade a real palletizer makes.

Geometry, masses and drive ratings are estimates for a machine of this class,
not ABB data. Zero pose: upper arm vertical, forearm horizontal along +X, tool
plate level with the flange pointing down.

Run from the project root:

    python scripts/make_palletizer.py
"""

from __future__ import annotations

from pathlib import Path

from urdf_parts import DOWN, drive_xml, g, joint_xml, link_xml, mimic_xml

OUT = Path(__file__).resolve().parents[1] / "examples" / "robots" / "palletizer_4dof.urdf"

# Construction of the real robot's motors, which sets their mass (see
# arm_analyzer.motor_mass -- form moves it more than the rating does).
MOTOR_FORM = "industrial"

GEARBOX_EFFICIENCY = 1.0
TRANSMISSION_EFFICIENCY = 1.0

COLUMN_H = 0.25  # pedestal to the yaw bearing
SHOULDER_H = 0.20  # yaw bearing to the shoulder pitch axis
UPPER = 0.55  # shoulder to elbow
FOREARM = 0.50  # elbow to the tool plate hanger
PLATE = 0.05  # hanger to the flange face

# name -> (mass, [(shape, params, xyz, axis)])
LINKS = {
    "base_link": (25.0, [("cylinder", (0.20, COLUMN_H), (0, 0, COLUMN_H / 2), "z")]),
    # Yaw column; the shoulder bearing and both push-rod pivots sit on top.
    "link1": (12.0, [("cylinder", (0.15, SHOULDER_H), (0, 0, SHOULDER_H / 2), "z")]),
    # Upper arm (rocker), shoulder to elbow.
    "link2": (8.0, [("box", (0.12, 0.14, UPPER), (0, 0, UPPER / 2), "z")]),
    # Forearm out to the tool hanger; carries the tool-rotate motor at its root.
    "link3": (6.0, [("box", (FOREARM, 0.12, 0.10), (FOREARM / 2, 0, 0), "z")]),
    # Parallelogram bracket (passive, driven by the elbow rod).
    "link4": (1.5, [("box", (0.10, 0.10, 0.12), (0, 0, -0.06), "z")]),
    # Tool plate: held level by the second linkage.
    "link5": (3.0, [("box", (0.20, 0.20, 0.05), (0, 0, -0.10), "z")]),
    # Rotating flange.
    "link6": (1.0, [("cylinder", (0.07, 0.04), (0, 0, -0.02), "z")]),
}

COLORS = {"base_link": "0.30 0.33 0.38 1"}

# (mass, radius, length, rotor inertia, peak, continuous, stall, no-load rad/s, kt, R)
MOTOR_700W = (0.050, 0.16, "1.6e-4", 6.0, 2.40, 12.0, 314, 0.45, 0.6)
MOTOR_400W = (0.032, 0.11, "2.0e-5", 1.3, 0.45, 2.6, 314, 0.30, 2.0)

# name, parent, child, origin, axis, lower, upper (deg), velocity (deg/s)
JOINTS = [
    ("j1", "base_link", "link1", (0, 0, COLUMN_H), "z", -165, 165, 150),
    ("j2", "link1", "link2", (0, 0, SHOULDER_H), "y", -25, 85, 110),
    ("j3", "link2", "link3", (0, 0, UPPER), "y", -20, 140, 130),
    # Passive: the rods that keep the tool plate level.
    ("level_elbow", "link3", "link4", (FOREARM, 0, 0), "y", -180, 180, 130),
    ("level_shoulder", "link4", "link5", (0, 0, 0), "y", -180, 180, 130),
    ("j4", "link5", "link6", (0, 0, -PLATE), "z", -300, 300, 300),
]

MIMICS = {"level_elbow": ("j3", -1.0), "level_shoulder": ("j2", -1.0)}

# joint -> (motor, gearbox, comment) -- see urdf_parts.drive_xml.
DRIVES = {
    "j1": (
        (MOTOR_700W, "base_link", (0.10, 0, 0.09), "z"),
        ("base_link", (0, 0, 0.20), "z", 100, "2.0e-5", 600, 250, 330, 0.10, 0.09),
        "gear unit drives the column directly",
    ),
    "j2": (
        (MOTOR_700W, "base_link", (-0.11, 0.09, 0.09), "z"),
        ("base_link", (-0.11, 0.09, 0.20), "z", 100, "2.0e-5", 600, 250, 330, 0.10, 0.09),
        "push rod from the base up to the shoulder rocker",
    ),
    "j3": (
        (MOTOR_700W, "base_link", (-0.11, -0.09, 0.09), "z"),
        ("base_link", (-0.11, -0.09, 0.20), "z", 100, "2.0e-5", 600, 250, 330, 0.10, 0.09),
        "push rod from the base, alongside the upper arm, to the elbow",
    ),
    "j4": (
        (MOTOR_400W, "link3", (0.10, 0, 0.09), "z"),
        ("link5", (0, 0, -0.06), "z", 50, "4.0e-6", 100, 40, 330, 0.05, 0.05),
        "belt down the forearm to the tool-rotate gear unit on the plate",
    ),
}

TOOL_TIP = {"link6": f'<tool_tip name="flange" xyz="0 0 -0.04" rpy="{DOWN}"/>'}


def main() -> None:
    parts = [
        '<?xml version="1.0"?>\n',
        "<!-- Generated by scripts/make_palletizer.py; edit the script, not this file.\n"
        "     Palletizer layout: J1-J3 motors on the pedestal, two parallelogram\n"
        "     linkages (<mimic>) keeping the tool plate level, 4 actuated axes.\n"
        "     Dimensions, masses and drive ratings are estimates, not ABB data. -->\n",
        '<robot name="palletizer_4dof">\n',
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
        if name in MIMICS:
            src, mult = MIMICS[name]
            body = mimic_xml(src, mult)
        else:
            motor, gearbox, comment = DRIVES[name]
            body = drive_xml(
                name,
                motor,
                gearbox,
                comment,
                gearbox_efficiency=GEARBOX_EFFICIENCY,
                motor_form=MOTOR_FORM,
                transmission_efficiency=TRANSMISSION_EFFICIENCY,
            )
        parts.append(joint_xml(name, parent, child, origin, axis, lo, hi, vel, body=body))
    parts.append("</robot>\n")
    OUT.write_text("".join(parts), encoding="utf-8")
    print(f"wrote {OUT} (reach {g(UPPER + FOREARM)} m)")


if __name__ == "__main__":
    main()
