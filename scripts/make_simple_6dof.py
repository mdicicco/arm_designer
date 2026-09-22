"""Generate the example arms in ``examples/robots/``.

``simple_6dof.urdf`` is a test arm with simple primitive geometry: a vertical
"candle" at the zero pose, ~1.0 m tall. Every drive is co-located with its
joint: gearbox centred on the joint axis, motor stacked behind it, both bolted
to the joint's parent link.

``simple_6dof_remote_elbow.urdf`` is the same arm with the elbow (j3) drive
moved back onto the shoulder link (link1), behind the j2 axis, driving the
elbow through a 1:1 belt (``<transmission>``). It shows what moving mass
toward the base does: j1 and j2 carry less, j3's own load is unchanged.

Link inertias are computed from each link's primitive at its declared mass,
so the numbers in the URDF are self-consistent. Run from the project root:

    python scripts/make_simple_6dof.py
"""

from __future__ import annotations

import math
from pathlib import Path

from urdf_parts import gearbox_body_for, motor_body_for

OUT_DIR = Path(__file__).resolve().parents[1] / "examples" / "robots"

HALF_PI = f"{-math.pi / 2:.6f}"  # rpy roll that turns local +Z onto +Y

# name, shape, params, centre-z, mass
LINKS = [
    ("base_link", "cylinder", (0.08, 0.14), 0.07, 2.5),
    ("link1", "cylinder", (0.06, 0.12), 0.06, 1.2),
    ("link2", "box", (0.06, 0.06, 0.35), 0.175, 1.8),
    ("link3", "box", (0.05, 0.05, 0.30), 0.15, 1.1),
    ("link4", "cylinder", (0.035, 0.08), 0.04, 0.45),
    ("link5", "box", (0.04, 0.04, 0.08), 0.04, 0.30),
    ("link6", "cylinder", (0.03, 0.02), 0.01, 0.10),
]

LARGE_MOTOR = dict(
    radius=0.035, length=0.07, rotor_inertia="4.0e-5", peak_torque=1.3,
    continuous_torque=0.45, stall_torque=3.2, no_load_speed=628, torque_constant=0.07,
    resistance=0.5,
)
MEDIUM_MOTOR = dict(
    radius=0.028, length=0.05, rotor_inertia="1.5e-5", peak_torque=0.64,
    continuous_torque=0.22, stall_torque=1.6, no_load_speed=680, torque_constant=0.05,
    resistance=1.1,
)
SMALL_MOTOR = dict(
    radius=0.02, length=0.04, rotor_inertia="4.0e-6", peak_torque=0.26,
    continuous_torque=0.09, stall_torque=0.7, no_load_speed=900, torque_constant=0.03,
    resistance=2.2,
)

# joint, parent, child, origin z, axis, lower, upper, velocity,
# motor, gearbox (ratio, input_inertia, peak, rated, max_in, r, L) -- the
# gearbox mass follows from its rated output torque, and r/L give its shape
JOINTS = [
    ("j1", "base_link", "link1", 0.14, "z", -2.97, 2.97, 3.0, LARGE_MOTOR,
     (100, "5e-6", 150, 60, 700, 0.045, 0.05)),
    ("j2", "link1", "link2", 0.12, "y", -2.0, 2.0, 2.5, LARGE_MOTOR,
     (120, "5e-6", 180, 80, 700, 0.045, 0.05)),
    ("j3", "link2", "link3", 0.35, "y", -2.5, 2.5, 3.0, MEDIUM_MOTOR,
     (100, "2e-6", 60, 25, 700, 0.038, 0.04)),
    ("j4", "link3", "link4", 0.30, "z", -3.0, 3.0, 5.0, SMALL_MOTOR,
     (80, "5e-7", 16, 7, 1000, 0.03, 0.03)),
    ("j5", "link4", "link5", 0.08, "y", -2.0, 2.0, 5.0, SMALL_MOTOR,
     (80, "5e-7", 16, 7, 1000, 0.028, 0.03)),
    ("j6", "link5", "link6", 0.08, "z", -3.14, 3.14, 6.0, SMALL_MOTOR,
     (80, "5e-7", 16, 7, 1000, 0.022, 0.025)),
]

# Loss placeholders, identical on every joint for now: gear friction goes into
# the gearbox efficiency; joint and belt/cable/linkage friction into the
# transmission efficiency. 1.0 = lossless.
# Construction of the motors, which sets their mass (see
# arm_analyzer.motor_mass -- form moves it more than the rating does).
MOTOR_FORM = "outrunner"

GEARBOX_EFFICIENCY = 1.0
TRANSMISSION_EFFICIENCY = 1.0

# Remote-elbow variant: j3's drive rides on link1, behind the j2 axis on the -Y
# side, and reaches the elbow through a 1:1 belt.
REMOTE_ELBOW = {
    "j3": {
        "host": "link1",
        "gb_xyz": "-0.03 -0.06 0.12",
        "m_xyz": "-0.03 -0.105 0.12",
        "transmission_comment": "belt from the shoulder-mounted gearbox to the elbow",
    }
}

COLORS = {
    "base_link": "0.30 0.33 0.38 1",
}


def g(v: float) -> str:
    return f"{v:.6g}"


def inertia(shape: str, params: tuple, mass: float) -> tuple[float, float, float]:
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


def link_xml(name, shape, params, cz, mass) -> str:
    ixx, iyy, izz = inertia(shape, params, mass)
    color = COLORS.get(name, "0.55 0.57 0.60 1")
    extra = ""
    if name == "link6":
        extra = """
    <visual>
      <origin xyz="0 0 0.04" rpy="0 0 0"/>
      <geometry><box size="0.015 0.05 0.04"/></geometry>
    </visual>
    <tool_tip name="tcp" xyz="0 0 0.06" rpy="0 0 0"/>"""
    return f"""  <link name="{name}">
    <inertial>
      <origin xyz="0 0 {g(cz)}" rpy="0 0 0"/>
      <mass value="{g(mass)}"/>
      <inertia ixx="{g(ixx)}" ixy="0" ixz="0" iyy="{g(iyy)}" iyz="0" izz="{g(izz)}"/>
    </inertial>
    <visual>
      <origin xyz="0 0 {g(cz)}" rpy="0 0 0"/>
      <geometry>{geometry(shape, params)}</geometry>
      <material name="{name}_mat"><color rgba="{color}"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 {g(cz)}" rpy="0 0 0"/>
      <geometry>{geometry(shape, params)}</geometry>
    </collision>{extra}
  </link>
"""


def drive_xml(name, parent, z, axis, motor, gb, remote=None) -> str:
    ratio, in_inertia, peak, rated, max_in, gb_r, gb_l = gb
    gb_mass, gb_r, gb_l = gearbox_body_for(rated, ratio, gb_r, gb_l)
    # Mass from the torque rating, envelope sized to hold it; the dict's
    # radius/length give only the motor's proportions.
    m_mass, m_radius, m_length = motor_body_for(
        motor["peak_torque"], motor["radius"], motor["length"], MOTOR_FORM
    )
    comment = "joint elements (gearbox output drives the joint directly)"
    if remote:
        parent = remote["host"]
        gb_xyz, m_xyz = remote["gb_xyz"], remote["m_xyz"]
        rpy = f"{HALF_PI} 0 0"
        comment = remote["transmission_comment"]
    elif axis == "z":
        # Stack below the joint, inside the parent link.
        gb_xyz = f"0 0 {g(z - gb_l / 2 - 0.005)}"
        m_xyz = f"0 0 {g(z - gb_l - 0.01 - m_length / 2)}"
        rpy = "0 0 0"
    else:
        # Beside the joint on the +Y side, shaft along the joint axis.
        side = 0.035
        gb_xyz = f"0 {g(side + gb_l / 2)} {g(z)}"
        m_xyz = f"0 {g(side + gb_l + m_length / 2)} {g(z)}"
        rpy = f"{HALF_PI} 0 0"
    return f"""    <drive>
      <motor name="{name}_motor" link="{parent}" form="{MOTOR_FORM}" rotor_inertia="{motor['rotor_inertia']}"
             peak_torque="{g(motor['peak_torque'])}" continuous_torque="{g(motor['continuous_torque'])}"
             stall_torque="{g(motor['stall_torque'])}" no_load_speed="{g(motor['no_load_speed'])}"
             torque_constant="{g(motor['torque_constant'])}" resistance="{g(motor['resistance'])}">
        <origin xyz="{m_xyz}" rpy="{rpy}"/>
        <mass value="{g(m_mass)}"/>
        <geometry><cylinder radius="{g(m_radius)}" length="{g(m_length)}"/></geometry>
      </motor>
      <gearbox name="{name}_gearbox" link="{parent}" ratio="{g(ratio)}" efficiency="{GEARBOX_EFFICIENCY:.2f}"
               input_inertia="{in_inertia}" peak_torque="{g(peak)}" rated_torque="{g(rated)}"
               max_input_speed="{g(max_in)}">
        <origin xyz="{gb_xyz}" rpy="{rpy}"/>
        <mass value="{g(gb_mass)}"/>
        <geometry><cylinder radius="{g(gb_r)}" length="{g(gb_l)}"/></geometry>
      </gearbox>
      <!-- {comment} -->
      <transmission ratio="1" efficiency="{TRANSMISSION_EFFICIENCY:.2f}"/>
    </drive>
"""


def joint_xml(j, remote=None) -> str:
    name, parent, child, z, axis, lo, hi, vel, motor, gb = j
    axis_xyz = {"y": "0 1 0", "z": "0 0 1"}[axis]
    return f"""  <joint name="{name}" type="revolute">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="0 0 {g(z)}" rpy="0 0 0"/>
    <axis xyz="{axis_xyz}"/>
    <!-- effort is omitted: it is derived from the drive (motor x ratio x efficiency, capped by the gearbox) -->
    <limit lower="{g(lo)}" upper="{g(hi)}" velocity="{g(vel)}"/>
{drive_xml(name, parent, z, axis, motor, gb, (remote or {}).get(name))}  </joint>
"""


def robot_xml(name: str, remote=None) -> str:
    parts = [
        '<?xml version="1.0"?>\n',
        "<!-- Generated by scripts/make_simple_6dof.py; edit the script, not this file. -->\n",
        f'<robot name="{name}">\n',
    ]
    for i, link in enumerate(LINKS):
        parts.append(link_xml(*link))
        if i < len(JOINTS):
            parts.append(joint_xml(JOINTS[i], remote))
    parts.append("</robot>\n")
    return "".join(parts)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, remote in (
        ("simple_6dof", None),
        ("simple_6dof_remote_elbow", REMOTE_ELBOW),
    ):
        out = OUT_DIR / f"{name}.urdf"
        out.write_text(robot_xml(name, remote), encoding="utf-8")
        print(f"wrote {out}")


if __name__ == "__main__":
    main()
