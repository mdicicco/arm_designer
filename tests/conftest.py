from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = PROJECT_ROOT / "examples"
SIMPLE_6DOF = EXAMPLES / "robots" / "simple_6dof.urdf"


@pytest.fixture(scope="session")
def simple_urdf() -> str:
    return SIMPLE_6DOF.read_text(encoding="utf-8")


@pytest.fixture(scope="session")
def simple_arm(simple_urdf):
    from arm_analyzer.robot import parse_arm

    return parse_arm(simple_urdf)


def one_joint_urdf(
    *,
    link_mass: float = 2.0,
    link_com_x: float = 0.5,
    motor_link: str = "arm",
    motor_xyz: str = "0.2 0 0",
    rotor_inertia: float = 0.0,
    ratio: float = 50.0,
    transmission: str = "",
    drive: bool = True,
    extra_motor_attrs: str = 'peak_torque="1.0" continuous_torque="0.4" stall_torque="2.0" no_load_speed="500"',
) -> str:
    """A base plus one arm link on a horizontal (Y-axis) revolute joint.

    At q = 0 the arm points along +X, so gravity torque is maximal there.
    """
    drive_xml = f"""
    <drive>
      <motor link="{motor_link}" rotor_inertia="{rotor_inertia}" {extra_motor_attrs}>
        <origin xyz="{motor_xyz}"/>
        <mass value="0.5"/>
      </motor>
      <gearbox link="base" ratio="{ratio}" peak_torque="60" rated_torque="25">
        <mass value="0.3"/>
      </gearbox>
      {transmission}
    </drive>""" if drive else ""
    return f"""<?xml version="1.0"?>
<robot name="one_joint">
  <link name="base">
    <inertial><origin xyz="0 0 0"/><mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/></inertial>
  </link>
  <joint name="shoulder" type="revolute">
    <parent link="base"/><child link="arm"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3" upper="3" velocity="2" effort="100"/>
    {drive_xml}
  </joint>
  <link name="arm">
    <inertial><origin xyz="{link_com_x} 0 0"/><mass value="{link_mass}"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial>
    <tool_tip name="tip" xyz="1 0 0"/>
  </link>
</robot>
"""


@pytest.fixture
def one_joint():
    return one_joint_urdf
