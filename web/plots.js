// Right panel and timeline: speed-torque plots, torque-vs-time, summary table.
//
// Every plot is drawn in two layers. The static layer (axes, envelopes, the
// whole operating trace, the RMS point) is rendered once into an offscreen
// canvas whenever the data, options or size change; each playback frame
// then only blits that and draws the playhead marker. Plot styling follows
// Modular Robot Studio's torque tab.

import { SERIES_COLORS, fmt, fmtPct, toRpm, utilClass } from "./format.js";

const $ = (sel) => document.querySelector(sel);

const PAD = { l: 44, r: 8, t: 8, b: 22 };
const AXIS_TEXT = "#7a8494";
const FONT = "10px system-ui, sans-serif";

// ---------------------------------------------------------------------------
// Canvas plumbing
// ---------------------------------------------------------------------------

class LayeredCanvas {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.cache = document.createElement("canvas");
    this.key = null;
    this.w = 0;
    this.h = 0;
    this.dpr = 1;
  }

  /** Resize to the CSS box; returns true if the backing store changed. */
  fit() {
    const w = this.canvas.clientWidth || 300;
    const h = this.canvas.clientHeight || 160;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    if (w === this.w && h === this.h && dpr === this.dpr) return false;
    this.w = w;
    this.h = h;
    this.dpr = dpr;
    for (const c of [this.canvas, this.cache]) {
      c.width = Math.max(1, Math.floor(w * dpr));
      c.height = Math.max(1, Math.floor(h * dpr));
    }
    return true;
  }

  /** Re-render the static layer if `key` or the size changed, then blit it. */
  draw(key, drawStatic) {
    const resized = this.fit();
    if (resized || key !== this.key) {
      const c = this.cache.getContext("2d");
      c.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
      c.clearRect(0, 0, this.w, this.h);
      drawStatic(c, this.w, this.h);
      this.key = key;
    }
    this.ctx.setTransform(1, 0, 0, 1, 0, 0);
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    this.ctx.drawImage(this.cache, 0, 0);
    this.ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    return this.ctx;
  }
}

function frame(c, w, h) {
  const plotW = w - PAD.l - PAD.r;
  const plotH = h - PAD.t - PAD.b;
  c.fillStyle = "rgba(255,255,255,0.025)";
  c.fillRect(PAD.l, PAD.t, plotW, plotH);
  c.strokeStyle = "rgba(255,255,255,0.10)";
  c.lineWidth = 1;
  c.strokeRect(PAD.l + 0.5, PAD.t + 0.5, plotW - 1, plotH - 1);
  return { plotW, plotH };
}

function emptyMessage(c, w, h, text) {
  const { plotW, plotH } = frame(c, w, h);
  c.fillStyle = AXIS_TEXT;
  c.font = "12px system-ui, sans-serif";
  c.textAlign = "center";
  c.textBaseline = "middle";
  c.fillText(text, PAD.l + plotW / 2, PAD.t + plotH / 2);
}

function niceMax(v) {
  if (!(v > 0)) return 1;
  const p = 10 ** Math.floor(Math.log10(v));
  for (const m of [1, 1.2, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10]) if (m * p >= v) return m * p;
  return 10 * p;
}

function tickLabel(v) {
  if (v === 0) return "0";
  const a = Math.abs(v);
  if (a >= 100) return v.toFixed(0);
  if (a >= 10) return v.toFixed(1).replace(/\.0$/, "");
  if (a >= 1) return v.toFixed(2).replace(/0$/, "");
  return v.toPrecision(2);
}

function axes(c, w, h, x, y, xLabel, yLabel) {
  const { plotW, plotH } = frame(c, w, h);
  c.font = FONT;
  c.fillStyle = AXIS_TEXT;
  c.strokeStyle = "rgba(255,255,255,0.06)";
  // Quarter gridlines.
  for (let i = 1; i < 4; i++) {
    const gx = PAD.l + (plotW * i) / 4;
    const gy = PAD.t + (plotH * i) / 4;
    c.beginPath();
    c.moveTo(gx, PAD.t);
    c.lineTo(gx, PAD.t + plotH);
    c.moveTo(PAD.l, gy);
    c.lineTo(PAD.l + plotW, gy);
    c.stroke();
  }
  c.strokeStyle = "rgba(255,255,255,0.2)";
  c.beginPath();
  c.moveTo(PAD.l, y.of(0));
  c.lineTo(PAD.l + plotW, y.of(0));
  c.moveTo(x.of(0), PAD.t);
  c.lineTo(x.of(0), PAD.t + plotH);
  c.stroke();

  c.textAlign = "right";
  c.textBaseline = "middle";
  c.fillText(tickLabel(y.max), PAD.l - 4, PAD.t + 4);
  c.fillText(tickLabel(y.min), PAD.l - 4, PAD.t + plotH - 4);
  if (y.min < 0) c.fillText("0", PAD.l - 4, y.of(0));
  c.textAlign = "center";
  c.textBaseline = "top";
  // The y-axis already labels a zero origin; only label x.min when it differs.
  if (x.min !== 0) c.fillText(tickLabel(x.min), PAD.l + 8, PAD.t + plotH + 4);
  c.fillText(tickLabel(x.max), PAD.l + plotW - 10, PAD.t + plotH + 4);
  if (x.min < 0) c.fillText("0", x.of(0), PAD.t + plotH + 4);
  c.fillText(xLabel, PAD.l + plotW / 2, PAD.t + plotH + 8);
  c.save();
  c.translate(10, PAD.t + plotH / 2);
  c.rotate(-Math.PI / 2);
  c.textBaseline = "middle";
  c.fillText(yLabel, 0, 0);
  c.restore();
  return { plotW, plotH };
}

function scale(min, max, p0, p1) {
  const span = max - min || 1;
  return { min, max, of: (v) => p0 + ((v - min) / span) * (p1 - p0) };
}

// ---------------------------------------------------------------------------
// Speed-torque plots
// ---------------------------------------------------------------------------

/** Pick the series / envelopes / units for one joint on the chosen side. */
function sideData(result, summary, side) {
  const s = result.series[summary.name];
  const env = summary.envelopes;
  if (side === "motor" && summary.drive) {
    return {
      speed: s.motor_speed.map(toRpm),
      torque: s.motor_torque,
      peak: env && { speed: env.motor_peak.speed.map(toRpm), torque: env.motor_peak.torque },
      cont: env && { speed: env.motor_continuous.speed.map(toRpm), torque: env.motor_continuous.torque },
      rms: [toRpm(summary.drive.motor.rms_speed), summary.drive.motor.rms_torque],
      xLabel: "motor speed (rpm)",
      yLabel: "motor τ (N·m)",
      util: s.util_motor,
      speedLimit: null,
    };
  }
  const prismatic = summary.type === "prismatic";
  return {
    speed: s.qd,
    torque: s.tau,
    peak: env && env.joint_peak,
    cont: env && env.joint_continuous,
    rms: [summary.joint.rms_speed, summary.joint.rms_torque],
    xLabel: prismatic ? "joint speed (m/s)" : "joint speed (rad/s)",
    yLabel: prismatic ? "joint force (N)" : "joint τ (N·m)",
    util: s.util_joint,
    speedLimit: summary.joint.velocity_limit,
  };
}

function drawEnvelope(c, env, x, y, fold, color, dashed) {
  if (!env) return;
  c.strokeStyle = color;
  c.lineWidth = dashed ? 1.2 : 1.6;
  c.globalAlpha = dashed ? 0.6 : 0.85;
  c.setLineDash(dashed ? [5, 4] : []);
  const quadrants = fold ? [[1, 1]] : [[1, 1], [-1, 1], [1, -1], [-1, -1]];
  for (const [sx, sy] of quadrants) {
    c.beginPath();
    let pen = false;
    for (let i = 0; i < env.speed.length; i++) {
      const tq = env.torque[i];
      if (tq == null) {
        pen = false;
        continue;
      }
      const px = x.of(sx * env.speed[i]);
      const py = y.of(sy * tq);
      if (pen) c.lineTo(px, py);
      else c.moveTo(px, py);
      pen = true;
    }
    // Close down to the speed axis at the envelope's end.
    const last = env.torque.length - 1;
    if (pen && env.torque[last] != null) c.lineTo(x.of(sx * env.speed[last]), y.of(0));
    c.stroke();
  }
  c.setLineDash([]);
  c.globalAlpha = 1;
}

function envMax(env) {
  let v = 0;
  for (const t of env?.torque || []) if (t != null) v = Math.max(v, t);
  return v;
}

function stStatic(c, w, h, result, summary, opts, color) {
  const { side, fold, zoom } = opts;
  const d = sideData(result, summary, side);
  let xMax = Math.abs(d.rms[0]);
  let yMax = Math.abs(d.rms[1]);
  for (const v of d.speed) xMax = Math.max(xMax, Math.abs(v));
  for (const v of d.torque) yMax = Math.max(yMax, Math.abs(v));
  if (zoom) {
    // Frame the operating trace; envelopes are clipped to the plot area.
    xMax = niceMax(Math.max(xMax * 1.15, 1e-6));
    yMax = niceMax(Math.max(yMax * 1.25, 1e-6));
  } else {
    if (d.peak) {
      xMax = Math.max(xMax, d.peak.speed[d.peak.speed.length - 1]);
      yMax = Math.max(yMax, envMax(d.peak));
    }
    xMax = niceMax(xMax * 1.04);
    yMax = niceMax(yMax * 1.08);
  }
  const plotW = w - PAD.l - PAD.r;
  const plotH = h - PAD.t - PAD.b;
  const x = scale(fold ? 0 : -xMax, xMax, PAD.l, PAD.l + plotW);
  const y = scale(fold ? 0 : -yMax, yMax, PAD.t + plotH, PAD.t);
  axes(c, w, h, x, y, d.xLabel, d.yLabel);
  c.save();
  c.beginPath();
  c.rect(PAD.l, PAD.t, plotW, plotH);
  c.clip();

  // Declared joint speed limit.
  if (d.speedLimit && d.speedLimit < xMax) {
    c.strokeStyle = "rgba(217,122,140,0.6)";
    c.setLineDash([2, 3]);
    c.beginPath();
    for (const sx of fold ? [1] : [1, -1]) {
      c.moveTo(x.of(sx * d.speedLimit), PAD.t);
      c.lineTo(x.of(sx * d.speedLimit), PAD.t + plotH);
    }
    c.stroke();
    c.setLineDash([]);
  }

  drawEnvelope(c, d.cont, x, y, fold, color, true);
  drawEnvelope(c, d.peak, x, y, fold, color, false);

  // Operating trace.
  c.strokeStyle = color;
  c.globalAlpha = 0.9;
  c.lineWidth = 1.2;
  c.beginPath();
  for (let i = 0; i < d.speed.length; i++) {
    const px = x.of(fold ? Math.abs(d.speed[i]) : d.speed[i]);
    const py = y.of(fold ? Math.abs(d.torque[i]) : d.torque[i]);
    if (i === 0) c.moveTo(px, py);
    else c.lineTo(px, py);
  }
  c.stroke();
  c.globalAlpha = 1;

  // Over-envelope samples in red.
  if (d.util) {
    c.fillStyle = "#f87171";
    for (let i = 0; i < d.util.length; i++) {
      if (!(d.util[i] > 1)) continue;
      const px = x.of(fold ? Math.abs(d.speed[i]) : d.speed[i]);
      const py = y.of(fold ? Math.abs(d.torque[i]) : d.torque[i]);
      c.fillRect(px - 1.5, py - 1.5, 3, 3);
    }
  }

  // RMS operating point.
  const [rx, ry] = [x.of(d.rms[0]), y.of(d.rms[1])];
  c.fillStyle = "#fff";
  c.strokeStyle = color;
  c.lineWidth = 1.5;
  c.beginPath();
  c.moveTo(rx, ry - 5);
  c.lineTo(rx + 5, ry);
  c.lineTo(rx, ry + 5);
  c.lineTo(rx - 5, ry);
  c.closePath();
  c.fill();
  c.stroke();
  c.restore();

  return { x, y, d };
}

export class SpeedTorquePlots {
  constructor({ onSelect }) {
    this.host = $("#st-grid");
    this.onSelect = onSelect;
    this.cells = [];
    this.result = null;
    this.side = "joint";
    this.fold = true;
    this.zoom = false;
    this.selected = null;
    this.version = 0;
  }

  setResult(result) {
    this.result = result;
    this.version++;
    this.host.innerHTML = "";
    this.cells = [];
    if (!result) return;
    result.summary.forEach((summary, idx) => {
      const cell = document.createElement("div");
      cell.className = "st-cell";
      cell.dataset.joint = summary.name;
      cell.innerHTML = `
        <div class="st-head">
          <span class="name" style="color:${SERIES_COLORS[idx % SERIES_COLORS.length]}">${summary.name}</span>
          <span class="badge ${summary.status}">${summary.status}</span>
          <span class="nums"></span>
        </div>
        <canvas></canvas>`;
      cell.addEventListener("click", () => this.onSelect(summary.name));
      this.host.appendChild(cell);
      this.cells.push({
        cell,
        summary,
        color: SERIES_COLORS[idx % SERIES_COLORS.length],
        nums: cell.querySelector(".nums"),
        layer: new LayeredCanvas(cell.querySelector("canvas")),
        geom: null,
      });
    });
    this.refreshHeaders();
    this.setSelected(this.selected);
  }

  setOptions({ side, fold, zoom }) {
    if (side !== undefined) this.side = side;
    if (fold !== undefined) this.fold = fold;
    if (zoom !== undefined) this.zoom = zoom;
    this.version++;
    this.refreshHeaders();
  }

  refreshHeaders() {
    for (const c of this.cells) {
      const s = c.summary;
      if (this.side === "motor" && s.drive) {
        const m = s.drive.motor;
        c.nums.textContent = `pk ${fmtPct(m.peak_util)} · rms ${fmtPct(m.rms_util)} · ${fmt(toRpm(m.peak_speed), 0)} rpm`;
      } else {
        const j = s.joint;
        c.nums.textContent = `pk ${fmt(j.peak_torque, 1)} · rms ${fmt(j.rms_torque, 1)} · env ${fmtPct(j.envelope_util)}`;
      }
    }
  }

  setSelected(name) {
    this.selected = name;
    for (const c of this.cells) c.cell.classList.toggle("selected", c.summary.name === name);
  }

  /** Redraw every plot with the playhead at sample index `idx`. */
  draw(idx) {
    if (!this.result) return;
    const opts = { side: this.side, fold: this.fold, zoom: this.zoom };
    const key = `${this.version}|${this.side}|${this.fold}|${this.zoom}`;
    for (const c of this.cells) {
      const ctx = c.layer.draw(key, (sc, w, h) => {
        c.geom = stStatic(sc, w, h, this.result, c.summary, opts, c.color);
      });
      if (idx == null || !c.geom) continue;
      const { x, y, d } = c.geom;
      const i = Math.min(idx, d.speed.length - 1);
      const sx = this.fold ? Math.abs(d.speed[i]) : d.speed[i];
      const sy = this.fold ? Math.abs(d.torque[i]) : d.torque[i];
      ctx.fillStyle = "#fff";
      ctx.strokeStyle = "#14171c";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(x.of(sx), y.of(sy), 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
  }
}

// ---------------------------------------------------------------------------
// Torque vs time
// ---------------------------------------------------------------------------

export class Timeline {
  constructor({ onSeek }) {
    this.layer = new LayeredCanvas($("#timeline-canvas"));
    this.legend = $("#timeline-legend");
    this.result = null;
    this.mode = "util";
    this.side = "joint";
    this.hidden = new Set();
    this.selected = null;
    this.version = 0;
    this.geom = null;

    this.layer.canvas.addEventListener("pointerdown", (e) => {
      const seek = (ev) => {
        if (!this.result || !this.geom) return;
        const rect = this.layer.canvas.getBoundingClientRect();
        const t = this.geom.x.inv(ev.clientX - rect.left);
        onSeek(Math.min(Math.max(t, 0), this.result.duration));
      };
      seek(e);
      const move = (ev) => seek(ev);
      window.addEventListener("pointermove", move);
      window.addEventListener("pointerup", () => window.removeEventListener("pointermove", move), { once: true });
    });
  }

  setResult(result) {
    this.result = result;
    this.version++;
    this.renderLegend();
  }

  setOptions({ mode, side }) {
    if (mode !== undefined) this.mode = mode;
    if (side !== undefined) this.side = side;
    this.version++;
  }

  setSelected(name) {
    this.selected = name;
    this.version++;
  }

  renderLegend() {
    const r = this.result;
    this.legend.innerHTML = "";
    if (!r) return;
    r.joints.forEach((n, i) => {
      const item = document.createElement("span");
      item.className = `item${this.hidden.has(n) ? " off" : ""}`;
      item.innerHTML = `<span class="sw" style="background:${SERIES_COLORS[i % SERIES_COLORS.length]}"></span>${n}`;
      item.addEventListener("click", () => {
        if (this.hidden.has(n)) this.hidden.delete(n);
        else this.hidden.add(n);
        this.version++;
        this.renderLegend();
      });
      this.legend.appendChild(item);
    });
  }

  _values(n) {
    const s = this.result.series[n];
    const motor = this.side === "motor" && s.motor_torque;
    if (this.mode === "util") return motor ? s.util_motor : s.util_joint;
    return motor ? s.motor_torque : s.tau;
  }

  _static(c, w, h) {
    const r = this.result;
    if (!r || !r.t.length) {
      emptyMessage(c, w, h, "no trajectory analysed");
      this.geom = null;
      return;
    }
    const names = r.joints.filter((n) => !this.hidden.has(n));
    const plotW = w - PAD.l - PAD.r;
    const plotH = h - PAD.t - PAD.b;
    const tMax = r.duration || 1;
    let yMin;
    let yMax;
    if (this.mode === "util") {
      yMin = 0;
      yMax = 1.1;
      for (const n of names) for (const v of this._values(n) || []) if (v != null) yMax = Math.max(yMax, Math.min(v, 3));
      yMax = niceMax(yMax);
    } else {
      yMax = 0;
      for (const n of names) for (const v of this._values(n)) yMax = Math.max(yMax, Math.abs(v));
      yMax = niceMax(yMax * 1.05);
      yMin = -yMax;
    }
    const x = scale(0, tMax, PAD.l, PAD.l + plotW);
    x.inv = (px) => ((px - PAD.l) / plotW) * tMax;
    const y = scale(yMin, yMax, PAD.t + plotH, PAD.t);
    const side = this.side === "motor" ? "motor" : "joint";
    axes(c, w, h, x, y, "time (s)", this.mode === "util" ? `${side} τ / envelope` : `${side} τ (N·m)`);

    if (this.mode === "util") {
      c.fillStyle = "rgba(248,113,113,0.08)";
      c.fillRect(PAD.l, PAD.t, plotW, Math.max(0, y.of(1) - PAD.t));
      c.strokeStyle = "rgba(248,113,113,0.6)";
      c.setLineDash([4, 3]);
      c.beginPath();
      c.moveTo(PAD.l, y.of(1));
      c.lineTo(PAD.l + plotW, y.of(1));
      c.stroke();
      c.strokeStyle = "rgba(245,158,59,0.35)";
      c.beginPath();
      c.moveTo(PAD.l, y.of(0.8));
      c.lineTo(PAD.l + plotW, y.of(0.8));
      c.stroke();
      c.setLineDash([]);
    }

    c.strokeStyle = "rgba(255,255,255,0.12)";
    for (const wt of r.plan.waypoint_times || []) {
      c.beginPath();
      c.moveTo(x.of(wt), PAD.t);
      c.lineTo(x.of(wt), PAD.t + plotH);
      c.stroke();
    }

    const n0 = r.t.length;
    // Selected joint drawn last and thicker.
    const ordered = [...names].sort((a, b) => (a === this.selected) - (b === this.selected));
    for (const n of ordered) {
      const vals = this._values(n);
      if (!vals) continue;
      const idx = r.joints.indexOf(n);
      c.strokeStyle = SERIES_COLORS[idx % SERIES_COLORS.length];
      c.lineWidth = n === this.selected ? 2.4 : 1.4;
      c.globalAlpha = this.selected && n !== this.selected ? 0.55 : 1;
      c.beginPath();
      let pen = false;
      for (let i = 0; i < n0; i++) {
        const v = vals[i];
        if (v == null) {
          pen = false;
          continue;
        }
        const px = x.of(r.t[i]);
        const py = y.of(Math.min(Math.max(v, yMin), yMax));
        if (pen) c.lineTo(px, py);
        else c.moveTo(px, py);
        pen = true;
      }
      c.stroke();
    }
    c.globalAlpha = 1;
    this.geom = { x, y, plotH };
  }

  draw(t) {
    const ctx = this.layer.draw(`${this.version}`, (c, w, h) => this._static(c, w, h));
    if (!this.geom || t == null) return;
    const px = this.geom.x.of(t);
    ctx.strokeStyle = "#fff";
    ctx.globalAlpha = 0.85;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(px, PAD.t);
    ctx.lineTo(px, PAD.t + this.geom.plotH);
    ctx.stroke();
    ctx.globalAlpha = 1;
  }
}

// ---------------------------------------------------------------------------
// Summary table
// ---------------------------------------------------------------------------

function td(v, cls = "") {
  return `<td class="${cls}">${v}</td>`;
}

function utd(v) {
  return td(fmtPct(v), utilClass(v));
}

export function renderSummary(result, { selected, onSelect }) {
  const host = $("#summary");
  if (!result) {
    host.innerHTML = `<p class="hint">Load a trajectory to analyse the drives.</p>`;
    return;
  }
  const rows = result.summary
    .map((s, i) => {
      const j = s.joint;
      const m = s.drive?.motor;
      const g = s.drive?.gearbox;
      const color = SERIES_COLORS[i % SERIES_COLORS.length];
      return `<tr data-joint="${s.name}" class="${s.name === selected ? "selected" : ""}">
        <td><span style="color:${color};font-weight:700">${s.name}</span></td>
        <td><span class="badge ${s.status}">${s.status}</span></td>
        ${td(fmt(j.peak_torque, 1))}${td(fmt(j.rms_torque, 1))}${utd(j.speed_util)}
        ${m ? utd(m.peak_util) + utd(m.rms_util) + utd(m.speed_util) : td("—", "u-na").repeat(3)}
        ${g ? utd(g.peak_util) + utd(g.rms_util) : td("—", "u-na").repeat(2)}
        ${m ? td(fmt(m.peak_power, 0)) : td("—", "u-na")}
        ${m?.mean_copper_loss != null ? td(fmt(m.mean_copper_loss, 1)) : td("—", "u-na")}
      </tr>`;
    })
    .join("");

  // Whole-arm totals from the time series.
  const n = result.t.length;
  let peakPower = 0;
  let meanCopper = 0;
  let energyOut = 0;
  const dt = n > 1 ? result.duration / (n - 1) : 0;
  for (let i = 0; i < n; i++) {
    let p = 0;
    for (const name of result.joints) {
      const s = result.series[name];
      if (s.motor_power) p += Math.max(0, s.motor_power[i]);
      if (s.copper_loss) meanCopper += s.copper_loss[i] / n;
    }
    peakPower = Math.max(peakPower, p);
    energyOut += p * dt;
  }

  host.innerHTML = `
    <table>
      <thead>
        <tr>
          <th></th><th></th>
          <th class="group" colspan="3">Joint</th>
          <th class="group" colspan="3">Motor</th>
          <th class="group" colspan="2">Gearbox</th>
          <th class="group" colspan="2">Power (W)</th>
        </tr>
        <tr>
          <th>Joint</th><th></th>
          <th title="peak |τ| at the joint (N·m)">τ pk</th>
          <th title="RMS τ at the joint (N·m)">τ rms</th>
          <th title="peak joint speed / declared velocity limit">spd</th>
          <th title="max |τ| / peak torque-speed envelope">pk</th>
          <th title="RMS τ / continuous torque">rms</th>
          <th title="peak speed / no-load speed">spd</th>
          <th title="peak output τ / peak rating">pk</th>
          <th title="RMS output τ / rated torque">rms</th>
          <th title="peak mechanical power at the motor shaft">mech</th>
          <th title="mean copper loss I²R">Cu</th>
        </tr>
      </thead>
      <tbody>${rows}</tbody>
    </table>
    <div class="totals">
      <span>Duration <b>${fmt(result.duration, 2)} s</b></span>
      <span>Peak arm power <b>${fmt(peakPower, 0)} W</b></span>
      <span>Motor energy out <b>${fmt(energyOut, 0)} J</b></span>
      <span>Mean copper loss <b>${fmt(meanCopper, 1)} W</b></span>
      ${result.payload_mass > 0 ? `<span>Payload <b>${fmt(result.payload_mass, 2)} kg</b></span>` : ""}
    </div>`;
  host.querySelectorAll("tbody tr").forEach((tr) => {
    tr.addEventListener("click", () => onSelect(tr.dataset.joint));
  });
}
