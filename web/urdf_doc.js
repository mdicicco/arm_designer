// The robot description as an editable XML document.
//
// The editor never builds a model itself: it edits this document, serializes
// it, and sends the text to /api/robot/model, so the server's parser stays the
// single authority on what a valid description is. Everything here is plain
// attribute plumbing in SI units; unit conversion for display lives in
// inspector.js.

const SIG_DIGITS = 7;

export function formatNumber(v) {
  if (!Number.isFinite(v)) return "0";
  const s = Number(v).toPrecision(SIG_DIGITS);
  return String(Number(s)); // drops trailing zeros and "e+0" noise
}

export class UrdfDoc {
  /** @throws {Error} if the text is not well-formed XML with a <robot> root */
  constructor(text) {
    this.doc = new DOMParser().parseFromString(text, "application/xml");
    const err = this.doc.getElementsByTagName("parsererror")[0];
    if (err) throw new Error(`invalid XML: ${err.textContent.split("\n")[0]}`);
    this.robot = this.doc.documentElement;
    if (this.robot.tagName !== "robot") throw new Error("expected a <robot> root element");
  }

  text() {
    return new XMLSerializer().serializeToString(this.doc);
  }

  // ---- lookup ----------------------------------------------------------

  _named(tag, name) {
    for (const el of this.robot.children) if (el.tagName === tag && el.getAttribute("name") === name) return el;
    return null;
  }

  link(name) {
    return this._named("link", name);
  }

  joint(name) {
    return this._named("joint", name);
  }

  jointForChild(linkName) {
    for (const el of this.robot.children) {
      if (el.tagName === "joint" && this.child(el, "child")?.getAttribute("link") === linkName) return el;
    }
    return null;
  }

  linkNames() {
    return [...this.robot.children].filter((e) => e.tagName === "link").map((e) => e.getAttribute("name"));
  }

  child(el, tag, create = false) {
    if (!el) return null;
    for (const c of el.children) if (c.tagName === tag) return c;
    if (!create) return null;
    const c = this.doc.createElement(tag);
    el.appendChild(c);
    return c;
  }

  drive(jointName, create = false) {
    return this.child(this.joint(jointName), "drive", create);
  }

  /** kind: "motor" | "gearbox" | "transmission" */
  part(jointName, kind, create = false) {
    return this.child(this.drive(jointName, create), kind, create);
  }

  // ---- attributes --------------------------------------------------------

  num(el, attr, fallback = null) {
    const raw = el?.getAttribute(attr);
    if (raw == null || raw.trim() === "") return fallback;
    const v = Number(raw);
    return Number.isFinite(v) ? v : fallback;
  }

  /** `null`/`""` removes the attribute (meaning "not declared"). */
  setNum(el, attr, value) {
    if (value == null || value === "" || !Number.isFinite(Number(value))) el.removeAttribute(attr);
    else el.setAttribute(attr, formatNumber(Number(value)));
  }

  vec(el, attr, n = 3) {
    const parts = (el?.getAttribute(attr) || "").trim().split(/[\s,]+/).filter(Boolean).map(Number);
    return Array.from({ length: n }, (_, i) => (Number.isFinite(parts[i]) ? parts[i] : 0));
  }

  setVec(el, attr, values) {
    el.setAttribute(attr, values.map(formatNumber).join(" "));
  }

  /** `<origin>` of `el`, created on demand. */
  origin(el, create = true) {
    return this.child(el, "origin", create);
  }

  // ---- links: visuals and inertial ---------------------------------------

  visuals(linkName) {
    const link = this.link(linkName);
    return link ? [...link.children].filter((c) => c.tagName === "visual") : [];
  }

  /** `{type, params}` of a <visual>/<motor>/<gearbox> geometry, or null. */
  geometry(el) {
    const g = this.child(el, "geometry");
    if (!g) return null;
    const box = this.child(g, "box");
    if (box) return { type: "box", params: this.vec(box, "size") };
    const cyl = this.child(g, "cylinder");
    if (cyl) return { type: "cylinder", params: [this.num(cyl, "radius", 0.05), this.num(cyl, "length", 0.1)] };
    const sph = this.child(g, "sphere");
    if (sph) return { type: "sphere", params: [this.num(sph, "radius", 0.05)] };
    return null;
  }

  setGeometry(el, type, params) {
    const g = this.child(el, "geometry", true);
    while (g.firstChild) g.removeChild(g.firstChild);
    const s = this.doc.createElement(type);
    if (type === "box") this.setVec(s, "size", params);
    else if (type === "cylinder") {
      this.setNum(s, "radius", params[0]);
      this.setNum(s, "length", params[1]);
    } else this.setNum(s, "radius", params[0]);
    g.appendChild(s);
    // Keep <geometry> directly after <origin>, as URDF authors expect.
    const o = this.child(el, "origin");
    if (o && o.nextSibling !== g) el.insertBefore(g, o.nextSibling);
  }

  addVisual(linkName) {
    const link = this.link(linkName);
    const v = this.doc.createElement("visual");
    const o = this.doc.createElement("origin");
    o.setAttribute("xyz", "0 0 0");
    o.setAttribute("rpy", "0 0 0");
    v.appendChild(o);
    link.appendChild(v);
    this.setGeometry(v, "box", [0.05, 0.05, 0.05]);
    return v;
  }

  remove(el) {
    el?.parentNode?.removeChild(el);
  }

  inertial(linkName, create = false) {
    const link = this.link(linkName);
    const el = this.child(link, "inertial", create);
    if (el && create) {
      this.child(el, "origin", true);
      const m = this.child(el, "mass", true);
      if (!m.hasAttribute("value")) m.setAttribute("value", "0");
      const i = this.child(el, "inertia", true);
      for (const k of ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]) if (!i.hasAttribute(k)) i.setAttribute(k, "0");
    }
    return el;
  }

  /**
   * Replace a link's COM and inertia with those of its visual shapes as solid
   * bodies sharing the link's current mass (density split by volume).
   * Returns false if the link has no usable shapes or no mass.
   */
  estimateInertial(linkName) {
    const inertial = this.inertial(linkName, true);
    const mass = this.num(this.child(inertial, "mass"), "value", 0);
    const shapes = [];
    for (const v of this.visuals(linkName)) {
      const g = this.geometry(v);
      if (!g) continue;
      const o = this.origin(v, false);
      const vol = volume(g);
      if (vol > 0) shapes.push({ g, vol, p: this.vec(o, "xyz"), R: rpyToR(this.vec(o, "rpy")) });
    }
    const total = shapes.reduce((s, x) => s + x.vol, 0);
    if (!(mass > 0) || total <= 0) return false;

    const com = [0, 0, 0];
    for (const s of shapes) for (let k = 0; k < 3; k++) com[k] += (s.vol / total) * s.p[k];
    const I = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (const s of shapes) {
      const m = (mass * s.vol) / total;
      const Ic = unitInertia(s.g).map((row) => row.map((x) => x * m));
      const Rot = mulMat(mulMat(s.R, Ic), transpose(s.R));
      const d = s.p.map((x, k) => x - com[k]);
      const dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
      for (let r = 0; r < 3; r++)
        for (let c = 0; c < 3; c++) I[r][c] += Rot[r][c] + m * ((r === c ? dd : 0) - d[r] * d[c]);
    }
    const o = this.origin(inertial, true);
    this.setVec(o, "xyz", com);
    o.setAttribute("rpy", "0 0 0");
    const ie = this.child(inertial, "inertia", true);
    const put = (k, v) => ie.setAttribute(k, formatNumber(Math.abs(v) < 1e-15 ? 0 : v));
    put("ixx", I[0][0]);
    put("iyy", I[1][1]);
    put("izz", I[2][2]);
    put("ixy", I[0][1]);
    put("ixz", I[0][2]);
    put("iyz", I[1][2]);
    return true;
  }

  // ---- joints ---------------------------------------------------------

  limit(jointName, create = false) {
    return this.child(this.joint(jointName), "limit", create);
  }

  // ---- efficiency, all drives ------------------------------------------

  drivenJoints() {
    return [...this.robot.children]
      .filter((e) => e.tagName === "joint" && this.child(e, "drive"))
      .map((e) => e.getAttribute("name"));
  }

  setEfficiencyAll(kind, value) {
    for (const j of this.drivenJoints()) this.setNum(this.part(j, kind, true), "efficiency", value);
  }
}

// ---- small maths for the inertia estimate ----------------------------------

function volume(g) {
  const p = g.params;
  if (g.type === "box") return Math.abs(p[0] * p[1] * p[2]);
  if (g.type === "cylinder") return Math.PI * p[0] * p[0] * p[1];
  return (4 / 3) * Math.PI * p[0] ** 3;
}

function unitInertia(g) {
  const p = g.params;
  let d;
  if (g.type === "box") {
    const [x, y, z] = p;
    d = [(y * y + z * z) / 12, (x * x + z * z) / 12, (x * x + y * y) / 12];
  } else if (g.type === "cylinder") {
    const [r, h] = p;
    const radial = (3 * r * r + h * h) / 12;
    d = [radial, radial, (r * r) / 2];
  } else {
    const i = (2 / 5) * p[0] * p[0];
    d = [i, i, i];
  }
  return [[d[0], 0, 0], [0, d[1], 0], [0, 0, d[2]]];
}

export function rpyToR([r, p, y]) {
  const [cr, sr, cp, sp, cy, sy] = [Math.cos(r), Math.sin(r), Math.cos(p), Math.sin(p), Math.cos(y), Math.sin(y)];
  // Rz(yaw) * Ry(pitch) * Rx(roll), matching the Python side.
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

function mulMat(a, b) {
  return a.map((row) => [0, 1, 2].map((c) => row[0] * b[0][c] + row[1] * b[1][c] + row[2] * b[2][c]));
}

function transpose(a) {
  return [0, 1, 2].map((r) => [0, 1, 2].map((c) => a[c][r]));
}
