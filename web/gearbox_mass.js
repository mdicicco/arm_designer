// Gearbox mass from the torque rating — the browser's copy of the fitted
// reducer law, so the inspector can show the estimate live while you edit a
// rating.
//
// The constants mirror `src/arm_analyzer/gearbox_mass.py`, which holds the
// provenance and the caveats. `tests/test_frontend_gearbox_mass.py` runs this
// module against the Python one to catch the two drifting apart.

export const B_TORQUE = 0.6910439266074775;
export const C_RATIO = -0.0067949979708556985;

export const A = {
  cycloidal: -1.9216305666561777,
  harmonic: -2.161920843866195,
  planetary: -2.3032801139847923,
  spur: -1.5386292527589296,
  worm: -1.5474191875573824,
};

export const TYPES = Object.keys(A).sort();
export const DEFAULT_TYPE = "harmonic";
export const LOO_MAE_KG = 0.58;
export const TORQUE_RANGE_NM = [0.4, 784.0];
export const RATIO_RANGE = [4.0, 111.0];

export const BULK_DENSITY = {
  cycloidal: 2062.0,
  harmonic: 3083.0,
  planetary: 4885.0,
  spur: 4623.0,
  worm: 3758.0,
};

/** The dataset's name for `kind`, or null when it is not one of them. */
export function normalizeType(kind) {
  if (!kind) return DEFAULT_TYPE;
  const key = String(kind).trim().toLowerCase();
  return key in A ? key : null;
}

/** Mass (kg) of a reducer rated for `ratedTorque` N·m out at `ratio`, or null
 *  when there is no rating (or no known type) to predict from. */
export function estimateGearboxMass(ratedTorque, ratio, kind) {
  const key = normalizeType(kind);
  if (key === null || !(ratedTorque > 0) || !(ratio > 0)) return null;
  return Math.exp(A[key] + B_TORQUE * Math.log(ratedTorque) + C_RATIO * Math.log(ratio));
}

/** Why this estimate is extrapolation, or null when it is interpolation. */
export function gearboxMassOutOfRange(ratedTorque, ratio, kind) {
  if (normalizeType(kind) === null || !(ratedTorque > 0) || !(ratio > 0)) return null;
  const [lo, hi] = TORQUE_RANGE_NM;
  if (ratedTorque < lo) return `${ratedTorque} N·m is below the fitted range (${lo}–${hi} N·m)`;
  if (ratedTorque > hi) return `${ratedTorque} N·m is above the fitted range (${lo}–${hi} N·m)`;
  const [rlo, rhi] = RATIO_RANGE;
  if (ratio < rlo || ratio > rhi) {
    return `${ratio}:1 is outside the fitted range (${rlo}–${rhi}:1)`;
  }
  return null;
}
