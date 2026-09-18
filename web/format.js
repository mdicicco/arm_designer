// Shared number formatting and the per-joint series palette.

export const SERIES_COLORS = [
  "#7ab8ff", "#f59e3b", "#6ab88a", "#d97a8c",
  "#b48bff", "#5fc6c0", "#e6c45f", "#9aa3b2",
];

export const WARN_FRACTION = 0.8;

export function fmt(v, digits = 2) {
  if (v == null || Number.isNaN(v)) return "—";
  if (!Number.isFinite(v)) return "∞";
  return Number(v).toFixed(digits);
}

export function fmtPct(v) {
  if (v == null || Number.isNaN(v)) return "—";
  if (!Number.isFinite(v)) return "∞";
  return `${Math.round(v * 100)}%`;
}

export function utilClass(v) {
  if (v == null || Number.isNaN(v)) return "u-na";
  if (v > 1) return "u-over";
  if (v > WARN_FRACTION) return "u-marginal";
  return "u-ok";
}

export function toRpm(w) {
  return (w * 60) / (2 * Math.PI);
}
