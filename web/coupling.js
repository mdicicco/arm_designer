// Joint-to-motor map for coupled drives (differentials).
//
// On a differential no motor owns a joint: each one carries a fixed share of
// both joint torques, so neither joint's speed-torque plot says what a motor
// must actually deliver. Two views of that, per coupling:
//
//   * the **torque plane** — the run traced in (τ_a, τ_b), with each motor's
//     rating drawn as a band across it. Two bands intersect in a diamond, and
//     that diamond, not the box of the two joint ratings, is what the pair can
//     hold: both joints can sit inside their own limits while their sum is not.
//   * the **split** — each motor's torque broken into what came from each
//     joint and what came from its own rotor, at the sample that sizes it.
//
// The drawn region is the stall region (zero motor speed), the largest it ever
// is; the sizing rows carry the real speed-dependent derating.

import { SERIES_COLORS, fmt, fmtPct, utilClass } from "./format.js";
import { LayeredCanvas, PAD, axes, niceMax, scale } from "./plots.js";

const $ = (sel) => document.querySelector(sel);

const REGION_PEAK = "#7ab8ff";
const REGION_CONT = "#6ab88a";
const BOX_COLOR = "rgba(154,163,178,0.45)";
const TRACE = "#e6c45f";

/** The four corners of `|n_j · τ| ≤ limit[j]`, as a closed polygon.
 *
 * The bands' corners are where both constraints are tight, so τ = Aᵀ s with
 * s the signed limits — the rating box mapped back into joint-torque space.
 * Taking the box corners in cyclic order keeps the polygon convex.
 */
export function regionCorners(A, limits) {
  if (limits.some((v) => v == null)) return null;
  const signs = [[1, 1], [1, -1], [-1, -1], [-1, 1]];
  return signs.map(([s0, s1]) => {
    const t = [s0 * limits[0], s1 * limits[1]];
    // Aᵀ t
    return [A[0][0] * t[0] + A[1][0] * t[1], A[0][1] * t[0] + A[1][1] * t[1]];
  });
}

function drawPolygon(c, pts, x, y, color, dashed) {
  if (!pts) return;
  c.strokeStyle = color;
  c.lineWidth = dashed ? 1.2 : 1.6;
  c.globalAlpha = dashed ? 0.65 : 0.9;
  c.setLineDash(dashed ? [5, 4] : []);
  c.beginPath();
  pts.forEach(([tx, ty], i) => (i ? c.lineTo(x.of(tx), y.of(ty)) : c.moveTo(x.of(tx), y.of(ty))));
  c.closePath();
  c.stroke();
  c.setLineDash([]);
  c.globalAlpha = 1;
}

/** Each joint's own rating as an axis-aligned box — the sizing you would get
 *  by reading the two joint plots separately. Drawn to be compared with the
 *  region, not to be believed. */
function jointBox(limits) {
  if (limits.some((v) => !(v > 0) || !Number.isFinite(v))) return null;
  const [a, b] = limits;
  return [[a, b], [a, -b], [-a, -b], [-a, b]];
}

function planeStatic(c, w, h, cmap, jointLimits, zoom) {
  const [na, nb] = cmap.joints;
  const ta = cmap.joint_torque[na];
  const tb = cmap.joint_torque[nb];
  const peak = regionCorners(cmap.A, cmap.motors.map((m) => m.stall_limit));
  const cont = regionCorners(cmap.A, cmap.motors.map((m) => m.continuous_limit));
  const box = jointBox(jointLimits);

  let xMax = 0;
  let yMax = 0;
  for (const v of ta) xMax = Math.max(xMax, Math.abs(v ?? 0));
  for (const v of tb) yMax = Math.max(yMax, Math.abs(v ?? 0));
  if (!zoom) {
    for (const pts of [peak, box]) {
      for (const [px, py] of pts || []) {
        xMax = Math.max(xMax, Math.abs(px));
        yMax = Math.max(yMax, Math.abs(py));
      }
    }
  }
  xMax = niceMax(Math.max(xMax * 1.1, 1e-6));
  yMax = niceMax(Math.max(yMax * 1.1, 1e-6));

  const plotW = w - PAD.l - PAD.r;
  const plotH = h - PAD.t - PAD.b;
  const x = scale(-xMax, xMax, PAD.l, PAD.l + plotW);
  const y = scale(-yMax, yMax, PAD.t + plotH, PAD.t);
  axes(c, w, h, x, y, `${na} τ (N·m)`, `${nb} τ (N·m)`);

  c.save();
  c.beginPath();
  c.rect(PAD.l, PAD.t, plotW, plotH);
  c.clip();

  // Zero crosshair — the quadrants matter here (same sign = both motors share
  // the load, opposite sign = they fight).
  c.strokeStyle = "rgba(255,255,255,0.10)";
  c.lineWidth = 1;
  c.beginPath();
  c.moveTo(PAD.l, y.of(0));
  c.lineTo(PAD.l + plotW, y.of(0));
  c.moveTo(x.of(0), PAD.t);
  c.lineTo(x.of(0), PAD.t + plotH);
  c.stroke();

  drawPolygon(c, box, x, y, BOX_COLOR, true);
  drawPolygon(c, cont, x, y, REGION_CONT, true);
  drawPolygon(c, peak, x, y, REGION_PEAK, false);

  // The run, in joint-torque space.
  c.strokeStyle = TRACE;
  c.lineWidth = 1.2;
  c.globalAlpha = 0.9;
  c.beginPath();
  let pen = false;
  for (let i = 0; i < ta.length; i++) {
    if (ta[i] == null || tb[i] == null) {
      pen = false;
      continue;
    }
    const px = x.of(ta[i]);
    const py = y.of(tb[i]);
    if (pen) c.lineTo(px, py);
    else c.moveTo(px, py);
    pen = true;
  }
  c.stroke();
  c.globalAlpha = 1;

  // Where each motor is worst off.
  cmap.motors.forEach((m, j) => {
    const i = m.sizing.index;
    if (ta[i] == null || tb[i] == null) return;
    const px = x.of(ta[i]);
    const py = y.of(tb[i]);
    c.strokeStyle = SERIES_COLORS[j % SERIES_COLORS.length];
    c.lineWidth = 1.6;
    c.beginPath();
    c.arc(px, py, 5, 0, Math.PI * 2);
    c.stroke();
    c.beginPath();
    c.moveTo(px - 7, py);
    c.lineTo(px + 7, py);
    c.moveTo(px, py - 7);
    c.lineTo(px, py + 7);
    c.stroke();
  });
  c.restore();
  return { x, y, ta, tb };
}

/** "τ_m = 0.0177·τ_j2 + 0.0177·τ_j3" — the row that names the split. */
function shareText(cmap, m) {
  return cmap.joints
    .map((n, i) => {
      const v = m.share[i];
      const sign = i === 0 ? (v < 0 ? "−" : "") : v < 0 ? " − " : " + ";
      return `${sign}${fmt(Math.abs(v), 4)}·τ(${n})`;
    })
    .join("");
}

function splitTable(cmap) {
  const rows = cmap.motors
    .map((m, j) => {
      const s = m.sizing;
      const color = SERIES_COLORS[j % SERIES_COLORS.length];
      const parts = cmap.joints
        .map((n) => `<td>${fmt(s.from_joints[n], 3)}</td>`)
        .join("");
      return `
        <tr>
          <td><span class="swatch" style="background:${color}"></span> ${m.joint}</td>
          <td class="share">${shareText(cmap, m)}</td>
          <td>${fmt(s.t, 2)}</td>
          ${parts}
          <td>${fmt(s.from_inertia, 3)}</td>
          <td><b>${fmt(s.motor_torque, 3)}</b></td>
          <td>${s.limit == null ? "—" : fmt(s.limit, 3)}</td>
          <td class="${utilClass(s.util)}">${fmtPct(s.util)}</td>
        </tr>`;
    })
    .join("");
  const jointHeads = cmap.joints.map((n) => `<th>from ${n}</th>`).join("");
  return `
    <table>
      <thead>
        <tr>
          <th>motor</th><th class="share">τ<sub>m</sub> =</th><th>at t</th>
          ${jointHeads}<th>rotor</th><th>τ<sub>m</sub></th><th>limit</th><th>util</th>
        </tr>
      </thead>
      <tbody>${rows}</tbody>
    </table>`;
}

export class CouplingPlots {
  constructor() {
    this.host = $("#coupling-grid");
    this.section = $("#coupling-section");
    this.cells = [];
    this.result = null;
    this.zoom = false;
    this.version = 0;
  }

  setResult(result) {
    this.result = result;
    this.version++;
    this.host.innerHTML = "";
    this.cells = [];
    const maps = result?.couplings?.filter((c) => c.plane) || [];
    this.section.hidden = maps.length === 0;
    if (!maps.length) return;

    const summaryOf = new Map((result.summary || []).map((s) => [s.name, s]));
    for (const cmap of maps) {
      // Each joint's own stall rating, for the box: the joint envelope at zero
      // speed, which for a coupled joint already accounts for both motors.
      const jointLimits = cmap.joints.map((n) => {
        const env = summaryOf.get(n)?.envelopes;
        return env?.joint_peak?.torque?.[0] ?? null;
      });
      const cell = document.createElement("div");
      cell.className = "cm-cell";
      cell.innerHTML = `
        <div class="st-head">
          <span class="name">${cmap.name}</span>
          <span class="badge coupled">${cmap.type}</span>
          <span class="nums">${cmap.joints.join(" ↔ ")}</span>
        </div>
        <canvas></canvas>
        <div class="cm-split">${splitTable(cmap)}</div>`;
      this.host.appendChild(cell);
      this.cells.push({
        cmap,
        jointLimits,
        layer: new LayeredCanvas(cell.querySelector("canvas")),
        geom: null,
      });
    }
  }

  setOptions({ zoom }) {
    if (zoom !== undefined) this.zoom = zoom;
    this.version++;
  }

  draw(idx) {
    if (!this.cells.length) return;
    const key = `${this.version}|${this.zoom}`;
    for (const c of this.cells) {
      const ctx = c.layer.draw(key, (sc, w, h) => {
        c.geom = planeStatic(sc, w, h, c.cmap, c.jointLimits, this.zoom);
      });
      if (idx == null || !c.geom) continue;
      const { x, y, ta, tb } = c.geom;
      const i = Math.min(idx, ta.length - 1);
      if (ta[i] == null || tb[i] == null) continue;
      ctx.fillStyle = "#fff";
      ctx.strokeStyle = "#14171c";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(x.of(ta[i]), y.of(tb[i]), 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
  }
}
