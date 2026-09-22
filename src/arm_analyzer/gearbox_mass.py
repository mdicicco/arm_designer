"""Estimate a gearbox's mass from the torque it is rated for and its type.

An empirical power law fitted to published reducer catalogues::

    log(m) = a[type] + b * log(T_out) + c * log(ratio)
           ->   m  ~  T_out^b * ratio^c,  scaled per type

with ``T_out`` the **rated output** torque in N*m (continuous, the way
catalogues quote a frame's capacity) and ``ratio`` the reduction.

Provenance
----------
Fitted in the ``robot_arm_data`` repository (``analyze_gearbox_mass.py``) over
``data/gearbox_data.csv``: 145 bare reducers -- not joint modules -- spanning
ratios 4:1 to 111:1 and rated output torques 0.4 to 784 N*m, from Harmonic
Drive CSF-2UH/LW, Nabtesco RV-E, Neugart PLE, Maxon GPX, NMRV worm boxes and
AliExpress HBK/SHF/FLE/GP equivalents. Ordinary least squares on the logs with
one intercept per type. The constants are copied here rather than refitted at
import so this package stays self-contained; ``COEFFICIENTS_SOURCE`` records
how to reproduce them.

Quality: in-sample R^2 0.92 on the logs, leave-one-out R^2 0.92, leave-one-out
mean absolute error 0.58 kg. That is a considerably tighter fit than the motor
one, because a reducer's mass really is close to a function of its frame size.

Ratio barely matters
--------------------
``c = -0.007``: mass is very nearly **independent of ratio**. That is the
dataset's main result rather than an artefact -- within one frame a harmonic
drive weighs the same at 30:1 as at 100:1, because the frame, not the gearing,
sets the mass. What a gearbox weighs is set by ``T_out`` (``b = 0.69``, so
doubling the rated torque costs about 1.6x the mass) and by its type.

Type matters a lot
------------------
At the same rating a worm box is roughly twice a planetary. The per-type
intercepts below carry that, and :data:`BULK_DENSITY` carries the matching
difference in how tightly each type packs its mass into its housing.

Scope
-----
This is the *gearbox* only -- not the motor driving it
(``arm_analyzer.motor_mass``) and not the bearings or housing of the joint it
sits in. Like the motor law it is a function of the rating a part is
*specified* to, not of the torque a trajectory happens to demand.
"""

from __future__ import annotations

import math
from typing import Any, Optional

# log(mass_kg) = A[type] + B_TORQUE * log(rated_out_Nm) + C_RATIO * log(ratio)
B_TORQUE = 0.6910439266074775
C_RATIO = -0.0067949979708556985

A = {
    "cycloidal": -1.9216305666561777,
    "harmonic": -2.161920843866195,
    "planetary": -2.3032801139847923,
    "spur": -1.5386292527589296,
    "worm": -1.5474191875573824,
}

TYPES = tuple(sorted(A))

# Harmonic drives are the most common reducer on an arm of this class and the
# best-represented type in the dataset (52 of 145 rows), so an undeclared
# gearbox is treated as one.
DEFAULT_TYPE = "harmonic"

COEFFICIENTS_SOURCE = (
    "robot_arm_data/analyze_gearbox_mass.py over data/gearbox_data.csv; "
    "n=145 reducers; R^2(log)=0.92, LOO R^2=0.92, LOO MAE=0.58 kg"
)
LOO_MAE_KG = 0.58

# Range of the fitted data.
TORQUE_RANGE_NM = (0.4, 784.0)
RATIO_RANGE = (4.0, 111.0)

# Bulk density of a reducer's envelope: mass over the cylinder that bounds it,
# median per type over the 106 rows listing both OD and length. Unlike a motor,
# which packs much the same way whatever it is, a reducer's density depends
# strongly on what is inside: a cycloidal frame is mostly housing, a planetary
# is mostly steel.
BULK_DENSITY = {
    "cycloidal": 2062.0,
    "harmonic": 3083.0,
    "planetary": 4885.0,
    "spur": 4623.0,
    "worm": 3758.0,
}


def normalize_type(gearbox_type: Optional[str]) -> str:
    """The dataset's name for ``gearbox_type``; unknown names are refused."""
    if not gearbox_type:
        return DEFAULT_TYPE
    key = str(gearbox_type).strip().lower()
    if key not in A:
        raise ValueError(f"unknown gearbox type {gearbox_type!r}; expected one of {TYPES}")
    return key


def estimate_mass(
    rated_torque: float, ratio: float, gearbox_type: Optional[str] = None
) -> float:
    """Mass (kg) of a ``gearbox_type`` reducer rated for ``rated_torque`` N*m
    at the output, reducing by ``ratio``."""
    key = normalize_type(gearbox_type)
    if not rated_torque or rated_torque <= 0:
        raise ValueError("rated_torque must be positive to estimate a gearbox mass")
    if not ratio or ratio <= 0:
        raise ValueError("ratio must be positive to estimate a gearbox mass")
    return math.exp(A[key] + B_TORQUE * math.log(rated_torque) + C_RATIO * math.log(ratio))


def bulk_density(gearbox_type: Optional[str] = None) -> float:
    """Envelope density (kg/m^3) used to size a housing for its mass."""
    return BULK_DENSITY[normalize_type(gearbox_type)]


def out_of_range(
    rated_torque: float, ratio: float, gearbox_type: Optional[str] = None
) -> Optional[str]:
    """Why this estimate is an extrapolation, or None when it is interpolation."""
    normalize_type(gearbox_type)
    lo, hi = TORQUE_RANGE_NM
    if rated_torque < lo:
        return f"{rated_torque:.3g} N·m is below the fitted range ({lo:g}–{hi:g} N·m)"
    if rated_torque > hi:
        return f"{rated_torque:.3g} N·m is above the fitted range ({lo:g}–{hi:g} N·m)"
    rlo, rhi = RATIO_RANGE
    if not (rlo <= ratio <= rhi):
        return f"{ratio:.3g}:1 is outside the fitted range ({rlo:g}–{rhi:g}:1)"
    return None


def describe(gearbox_type: Optional[str] = None) -> dict[str, Any]:
    """The constants behind one type, for reporting."""
    key = normalize_type(gearbox_type)
    return {
        "type": key,
        "intercept": A[key],
        "torque_exponent": B_TORQUE,
        "ratio_exponent": C_RATIO,
        "bulk_density": BULK_DENSITY[key],
    }
