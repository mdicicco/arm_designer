// Motor mass from torque rating and construction — the browser's copy of the
// fitted BLDC power law, so the inspector can show the estimate live while you
// edit a rating.
//
// The constants mirror `src/arm_analyzer/motor_mass.py`, which holds the
// provenance and the caveats. `tests/test_frontend_motor_mass.py` runs this
// module against the Python one to catch the two drifting apart.
//
// There is no speed term: once form is known, speed stops earning its place
// (it is largely a proxy for size in this dataset).

export const B_TORQUE = 0.6970294740298142;

export const A = {
  flat: -1.4997499248737194,
  frameless: -1.6246025277668223,
  hub: -0.9110476826416951,
  industrial: -0.5407820490749299,
  inrunner: -1.2088938950153576,
  integrated: -0.4292261385671184,
  outrunner: -1.7587905542954658,
};

export const FORMS = Object.keys(A).sort();
export const DEFAULT_FORM = "frameless";
export const LOO_MAE_KG = 0.36;
export const LOO_MEDIAN_REL = 0.30;
export const LIGHTEST_KG = 0.0083;

export const TORQUE_RANGE = {
  flat: [0.0853, 9.95],
  frameless: [1.35, 60.0],
  hub: [14.3, 98.0],
  industrial: [1.79, 27.1],
  inrunner: [0.00344, 2.65],
  integrated: [0.45, 13.02],
  outrunner: [0.28, 17.0],
};

export const OVERALL_DENSITY = 3063.0;
export const DENSITY = {
  frameless: 2583.0,
  inrunner: 5329.0,
  integrated: 3149.0,
  outrunner: 2794.0,
};

/** The dataset's name for `form`, or null when it is not one of them. */
export function normalizeForm(form) {
  if (!form) return DEFAULT_FORM;
  const key = String(form).trim().toLowerCase();
  return key in A ? key : null;
}

/** Mass (kg) of a `form` motor rated for `peakTorque` N·m, or null when there
 *  is no rating (or no known form) to predict from. */
export function estimateMotorMass(peakTorque, form) {
  const key = normalizeForm(form);
  if (key === null || !(peakTorque > 0)) return null;
  return Math.exp(A[key] + B_TORQUE * Math.log(peakTorque));
}

/** Envelope density (kg/m³) for sizing a housing to its mass. */
export function motorBulkDensity(form) {
  const key = normalizeForm(form);
  if (key === null) return OVERALL_DENSITY;
  return key in DENSITY ? DENSITY[key] : OVERALL_DENSITY;
}

/** Why this estimate is extrapolation, or null when it is interpolation.
 *  Each form covers its own slice of the torque range, so the check is per
 *  form rather than against the dataset as a whole. */
export function motorMassOutOfRange(peakTorque, form) {
  const key = normalizeForm(form);
  if (key === null || !(peakTorque > 0)) return null;
  const [lo, hi] = TORQUE_RANGE[key];
  if (peakTorque < lo) return `${peakTorque} N·m is below the ${key} range (${lo}–${hi} N·m)`;
  if (peakTorque > hi) return `${peakTorque} N·m is above the ${key} range (${lo}–${hi} N·m)`;
  return null;
}
