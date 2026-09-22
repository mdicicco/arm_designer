"""Estimate a BLDC motor's mass from its torque rating and its construction.

An empirical power law fitted to published datasheet masses::

    log(m) = a[form] + b * log(tau_max)     ->     m  ~  tau^b, scaled per form

with ``tau_max`` the motor's peak torque in N*m. There is no speed term: see
*Why speed is gone* below.

Provenance
----------
Fitted in the ``robot_arm_data`` repository (``analyze_motor_mass.py``) over
``data/bldc_motor_data.csv``: 76 motors with a manufacturer-listed mass,
spanning **0.0034 to 98 N*m** and 8.3 g to 11 kg, from CubeMars, Maxon,
Faulhaber, Kollmorgen, Teknic, ODrive, T-Motor and OEM frameless / outrunner /
hub parts. Ordinary least squares on the logs with one intercept per form.
The constants are copied here rather than refitted at import so this package
stays self-contained; ``COEFFICIENTS_SOURCE`` records how to reproduce them.

Quality: in-sample R^2 0.95 on the logs, leave-one-out R^2 0.93, leave-one-out
mean absolute error 0.36 kg, leave-one-out **median relative error 30%**. Read
the relative figure, not the kilograms: this set spans four decades of mass, so
an average in kg is dominated by the largest motors.

Form is the dominant variable
-----------------------------
Construction matters more than anything else. Predicted against actual for a
*pooled* fit -- one intercept for everything -- the medians came out:

    frameless 0.57  outrunner 0.62  flat 0.76  inrunner 1.38
    hub 1.47        industrial 2.06  integrated 2.52

a 4.4x spread, which is why pooling left a 50% median error. Splitting the
intercept by form halves that. At the same torque rating an integrated
servo-plus-driver is about 3.8x a comparable outrunner, and *that* is the
number to know when sizing an arm: picking the construction moves the mass more
than picking the rating.

Each form is fitted over its own slice of the torque range
(:data:`TORQUE_RANGE`) and those slices barely overlap -- tiny inrunners at one
end, hub motors at the other. Asking for a 0.01 N*m frameless motor is
extrapolation even though 0.01 N*m is inside the dataset overall, so the range
check is per form.

Why speed is gone
-----------------
The earlier pooled fit carried a ``log(speed)`` term with a coefficient near
zero. On the enlarged set it is worse than useless: small motors in this
dataset are also fast (``corr(log tau, log speed) = -0.75``), so speed is
largely a proxy for size, and once ``form`` is in the model adding speed back
*raises* leave-one-out error (0.40 kg against 0.36 kg). One fewer input, and a
more robust fit.

Scope
-----
This is the *motor* only -- not its gearbox (``arm_analyzer.gearbox_mass``). It
is a function of the rating a motor is *specified* to, not of the torque a
trajectory happens to demand. And a power law has no floor: below the range for
its form the prediction keeps shrinking toward zero, where a real motor still
needs a housing, bearings and a connector. :data:`LIGHTEST_KG` records the
smallest real part in the set; ``estimate_mass`` does not clamp to it, but
:func:`out_of_range` says when you are past the edge.
"""

from __future__ import annotations

import math
from typing import Any, Optional

# log(mass_kg) = A[form] + B_TORQUE * log(peak_torque_Nm)
B_TORQUE = 0.6970294740298142

A = {
    "flat": -1.4997499248737194,
    "frameless": -1.6246025277668223,
    "hub": -0.9110476826416951,
    "industrial": -0.5407820490749299,
    "inrunner": -1.2088938950153576,
    "integrated": -0.4292261385671184,
    "outrunner": -1.7587905542954658,
}

FORMS = tuple(sorted(A))

# Frameless is the usual choice for a robot joint and the best-represented form
# in the set, so an undeclared motor is treated as one.
DEFAULT_FORM = "frameless"

COEFFICIENTS_SOURCE = (
    "robot_arm_data/analyze_motor_mass.py over data/bldc_motor_data.csv; "
    "n=76 listed-mass rows, per-form intercepts; R^2(log)=0.95, LOO R^2=0.93, "
    "LOO MAE=0.36 kg, LOO median relative error 30%"
)
LOO_MAE_KG = 0.36
LOO_MEDIAN_REL = 0.30

# Torque each form was actually fitted over. The slices barely overlap, so the
# range check has to be per form rather than against the union.
TORQUE_RANGE = {
    "flat": (0.0853, 9.95),
    "frameless": (1.35, 60.0),
    "hub": (14.3, 98.0),
    "industrial": (1.79, 27.1),
    "inrunner": (0.00344, 2.65),
    "integrated": (0.45, 13.02),
    "outrunner": (0.28, 17.0),
}

# Lightest motor in the set. A power law runs to zero; real parts do not.
LIGHTEST_KG = 0.0083

# Bulk density of a motor's envelope: mass over the cylinder bounding it.
# Per form where at least four rows list both OD and length, else the overall
# median. An inrunner is a dense little slug; an outrunner is mostly air.
OVERALL_DENSITY = 3063.0
DENSITY = {
    "frameless": 2583.0,
    "inrunner": 5329.0,
    "integrated": 3149.0,
    "outrunner": 2794.0,
}


def normalize_form(form: Optional[str]) -> str:
    """The dataset's name for ``form``; unknown names are refused."""
    if not form:
        return DEFAULT_FORM
    key = str(form).strip().lower()
    if key not in A:
        raise ValueError(f"unknown motor form {form!r}; expected one of {FORMS}")
    return key


def estimate_mass(peak_torque: float, form: Optional[str] = None) -> float:
    """Mass (kg) of a ``form`` motor rated for ``peak_torque`` N*m."""
    key = normalize_form(form)
    if not peak_torque or peak_torque <= 0:
        raise ValueError("peak_torque must be positive to estimate a motor mass")
    return math.exp(A[key] + B_TORQUE * math.log(peak_torque))


def bulk_density(form: Optional[str] = None) -> float:
    """Envelope density (kg/m^3) used to size a housing for its mass."""
    return DENSITY.get(normalize_form(form), OVERALL_DENSITY)


def out_of_range(peak_torque: float, form: Optional[str] = None) -> Optional[str]:
    """Why this estimate is an extrapolation, or None when it is interpolation."""
    key = normalize_form(form)
    lo, hi = TORQUE_RANGE[key]
    if peak_torque < lo:
        return f"{peak_torque:.3g} N·m is below the {key} range ({lo:g}–{hi:g} N·m)"
    if peak_torque > hi:
        return f"{peak_torque:.3g} N·m is above the {key} range ({lo:g}–{hi:g} N·m)"
    return None


def describe(form: Optional[str] = None) -> dict[str, Any]:
    """The constants behind one form, for reporting."""
    key = normalize_form(form)
    return {
        "form": key,
        "intercept": A[key],
        "torque_exponent": B_TORQUE,
        "torque_range": list(TORQUE_RANGE[key]),
        "bulk_density": bulk_density(key),
    }
