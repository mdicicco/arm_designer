"""Single-arm speed/torque analysis with lumped motor and gearbox masses.

Derived from the ``modular_robot`` framework, but self-contained: nothing here
imports ``modular_robot``.
"""

from arm_analyzer.robot import ArmDescription, load_arm, parse_arm

__all__ = ["ArmDescription", "load_arm", "parse_arm"]
