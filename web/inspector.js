// Property panel for the URDF editor.
//
// Shows the element selected in the editor view -- a link, a motor, a gearbox
// or a transmission (with its joint) -- as editable fields, or the robot
// overview when nothing is selected. Every field writes straight into the
// UrdfDoc and calls `onEdit(commit)`: `commit=false` while typing/dragging
// (live preview), `commit=true` when the value is settled (undo history).
//
// Display units are engineering ones (mm, degrees, rpm, N·m); the document is
// always SI. Fields rebuild only when the selection changes or after a
// structural edit, never while the user is typing in them.

import { SERIES_COLORS, fmt, fmtPct } from "./format.js";

const RAD = Math.PI / 180;
const RPM = (2 * Math.PI) / 60;

const UNITS = {
  mm: { label: "mm", k: 1000, step: 1 },
  deg: { label: "°", k: 1 / RAD, step: 1 },
  degps: { label: "°/s", k: 1 / RAD, step: 5 },
  mmps: { label: "mm/s", k: 1000, step: 5 },
  rpm: { label: "rpm", k: 1 / RPM, step: 50 },
  kg: { label: "kg", k: 1, step: 0.01 },
  Nm: { label: "N·m", k: 1, step: 0.1 },
  N: { label: "N", k: 1, step: 1 },
  kgm2: { label: "kg·m²", k: 1, step: "any" },
  ratio: { label: ": 1", k: 1, step: 1 },
  Kt: { label: "N·m/A", k: 1, step: 0.01 },
  ohm: { label: "Ω", k: 1, step: 0.1 },
};

const AXES = { "+X": [1, 0, 0], "−X": [-1, 0, 0], "+Y": [0, 1, 0], "−Y": [0, -1, 0], "+Z": [0, 0, 1], "−Z": [0, 0, -1] };

function el(tag, attrs = {}, ...children) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") e.className = v;
    else if (k.startsWith("on")) e.addEventListener(k.slice(2), v);
    else if (v != null) e.setAttribute(k, v);
  }
  for (const c of children.flat()) if (c != null) e.append(c);
  return e;
}

function show(v, unit) {
  if (v == null) return "";
  const x = v * UNITS[unit].k;
  return String(Number(x.toPrecision(6)));
}

export class Inspector {
  /**
   * @param {HTMLElement} host
   * @param {{getDoc: () => import('./urdf_doc.js').UrdfDoc, getModel: () => object,
   *          onEdit: (commit: boolean) => void, onSelect: (sel: object|null) => void}} ctx
   */
  constructor(host, ctx) {
    this.host = host;
    this.ctx = ctx;
    this.selection = null;
    this.autoInertia = new Set(); // links whose inertia follows their shapes
    this.derived = []; // () => void, refreshed after each model update
  }

  get doc() {
    return this.ctx.getDoc();
  }

  get model() {
    return this.ctx.getModel();
  }

  // -------------------------------------------------------------------------
  // Field builders
  // -------------------------------------------------------------------------

  /** A numeric input bound to a getter/setter pair in SI units. */
  num(label, unit, get, set, { optional = false, placeholder = "", title = "", min = null } = {}) {
    const u = UNITS[unit];
    const input = el("input", {
      type: "number",
      step: u.step,
      value: show(get(), unit),
      placeholder,
      min,
    });
    const apply = (commit) => {
      const raw = input.value.trim();
      if (raw === "") {
        if (!optional) return;
        set(null);
      } else {
        const v = Number(raw);
        if (!Number.isFinite(v)) return;
        set(v / u.k);
      }
      this.ctx.onEdit(commit);
    };
    input.addEventListener("input", () => apply(false));
    input.addEventListener("change", () => apply(true));
    return el("label", { class: "f", title }, el("span", {}, label), input, el("em", {}, u.label));
  }

  /** Three inputs for an "x y z" style attribute. */
  vec3(label, unit, get, set, names = ["x", "y", "z"]) {
    const u = UNITS[unit];
    const values = get();
    const inputs = values.map((v, i) =>
      el("input", { type: "number", step: u.step, value: show(v, unit), title: names[i] })
    );
    const apply = (commit) => {
      const vs = inputs.map((x) => Number(x.value));
      if (vs.some((v) => !Number.isFinite(v))) return;
      set(vs.map((v) => v / u.k));
      this.ctx.onEdit(commit);
    };
    for (const i of inputs) {
      i.addEventListener("input", () => apply(false));
      i.addEventListener("change", () => apply(true));
    }
    return el("div", { class: "f f3" }, el("span", {}, label), el("div", { class: "trio" }, inputs), el("em", {}, u.label));
  }

  /** Efficiency slider (50-100 %) bound to `getPart(create)`'s `efficiency`. */
  efficiency(label, getPart, title) {
    const doc = this.doc;
    const value = doc.num(getPart(false), "efficiency", 1);
    const out = el("span", { class: "eff-val" }, fmtPct(value));
    const input = el("input", { type: "range", min: 0.5, max: 1, step: 0.01, value });
    const apply = (commit) => {
      doc.setNum(getPart(true), "efficiency", Number(input.value));
      out.textContent = fmtPct(Number(input.value));
      this.ctx.onEdit(commit);
    };
    input.addEventListener("input", () => apply(false));
    input.addEventListener("change", () => apply(true));
    return el("label", { class: "f f-eff", title }, el("span", {}, label), input, out);
  }

  select(label, options, value, onChange) {
    const s = el("select", {}, options.map(([v, text]) => el("option", { value: v }, text)));
    s.value = value;
    s.addEventListener("change", () => onChange(s.value));
    return el("label", { class: "f" }, el("span", {}, label), s, el("em"));
  }

  section(title, ...children) {
    return el("fieldset", { class: "sec" }, el("legend", {}, title), ...children);
  }

  derivedLine(fn) {
    const span = el("div", { class: "derived" });
    const update = () => (span.innerHTML = fn() || "");
    update();
    this.derived.push(update);
    return span;
  }

  navButton(text, sel) {
    return el("button", { type: "button", class: "ghost-btn small", onclick: () => this.ctx.onSelect(sel) }, text);
  }

  /** Origin + geometry editor for a <visual>, <motor> or <gearbox>. */
  shapeEditor(node, { onGeometry = null, removable = null } = {}) {
    const doc = this.doc;
    const origin = doc.origin(node, true);
    const geom = doc.geometry(node) || { type: "cylinder", params: [0.03, 0.05] };
    const box = el("div", { class: "shape" });
    const rebuild = () => {
      const fresh = this.shapeEditor(node, { onGeometry, removable });
      box.replaceWith(fresh);
    };
    // Numeric fields report their own edits; only a shape-type switch (which
    // rebuilds this block) reports here.
    const setGeom = (params, commit, type = geom.type) => {
      doc.setGeometry(node, type, params);
      if (type === geom.type) geom.params = [...params];
      onGeometry?.();
      if (commit) this.ctx.onEdit(true);
    };
    const typeSel = this.select(
      "Shape",
      [["box", "box"], ["cylinder", "cylinder (along z)"], ["sphere", "sphere"]],
      geom.type,
      (type) => {
        const p = geom.params;
        const size = geom.type === "box" ? Math.max(p[0], p[1]) / 2 : p[0];
        const len = geom.type === "box" ? p[2] : geom.type === "cylinder" ? p[1] : 2 * p[0];
        const params = type === "box" ? [2 * size, 2 * size, len] : type === "cylinder" ? [size, len] : [size];
        setGeom(params, true, type);
        rebuild();
      }
    );
    const p = [...geom.params];
    const dims =
      geom.type === "box"
        ? this.vec3("Size", "mm", () => p, (v) => setGeom(v, false), ["x", "y", "z"])
        : geom.type === "cylinder"
          ? [
              this.num("Radius", "mm", () => p[0], (v) => setGeom([(p[0] = v), p[1]], false)),
              this.num("Length", "mm", () => p[1], (v) => setGeom([p[0], (p[1] = v)], false)),
            ]
          : this.num("Radius", "mm", () => p[0], (v) => setGeom([(p[0] = v)], false));
    box.append(
      typeSel,
      ...[dims].flat(),
      this.vec3("Position", "mm", () => doc.vec(origin, "xyz"), (v) => {
        doc.setVec(origin, "xyz", v);
        onGeometry?.();
      }),
      this.vec3("Rotation", "deg", () => doc.vec(origin, "rpy"), (v) => {
        doc.setVec(origin, "rpy", v);
        onGeometry?.();
      }, ["roll", "pitch", "yaw"])
    );
    if (removable) {
      box.append(el("div", { class: "row-end" }, el("button", { type: "button", class: "ghost-btn small danger", onclick: removable }, "Remove shape")));
    }
    return box;
  }

  jointEditor(jointName) {
    const doc = this.doc;
    const joint = doc.joint(jointName);
    const type = joint.getAttribute("type");
    const origin = doc.origin(joint, true);
    const axisEl = doc.child(joint, "axis", true);
    const axis = doc.vec(axisEl, "xyz");
    const axisKey = Object.keys(AXES).find((k) => AXES[k].every((c, i) => Math.abs(c - axis[i]) < 1e-9));
    const angular = type !== "prismatic";
    const lim = () => doc.limit(jointName, true);
    const rows = [
      el("div", { class: "f" }, el("span", {}, "Type"), el("b", {}, type), el("em")),
      this.vec3("Offset", "mm", () => doc.vec(origin, "xyz"), (v) => doc.setVec(origin, "xyz", v)),
      this.vec3("Rotation", "deg", () => doc.vec(origin, "rpy"), (v) => doc.setVec(origin, "rpy", v), ["roll", "pitch", "yaw"]),
      this.select(
        "Axis",
        [...Object.keys(AXES).map((k) => [k, k]), ...(axisKey ? [] : [["custom", `custom (${axis.map((a) => fmt(a, 2)).join(" ")})`]])],
        axisKey || "custom",
        (k) => {
          if (k === "custom") return;
          doc.setVec(axisEl, "xyz", AXES[k]);
          this.ctx.onEdit(true);
        }
      ),
    ];
    if (type !== "fixed") {
      const lu = angular ? "deg" : "mm";
      rows.push(
        this.num("Lower limit", lu, () => doc.num(doc.limit(jointName), "lower"), (v) => doc.setNum(lim(), "lower", v)),
        this.num("Upper limit", lu, () => doc.num(doc.limit(jointName), "upper"), (v) => doc.setNum(lim(), "upper", v)),
        this.num("Max speed", angular ? "degps" : "mmps", () => doc.num(doc.limit(jointName), "velocity"), (v) => doc.setNum(lim(), "velocity", v), {
          optional: true,
          title: "Joint velocity limit. Blank: the drive's speed limit.",
        }),
        this.num("Max torque", angular ? "Nm" : "N", () => doc.num(doc.limit(jointName), "effort"), (v) => doc.setNum(lim(), "effort", v), {
          optional: true,
          placeholder: "from drive",
          title: "Joint effort limit. Blank: derived from motor × ratio × efficiency, capped by the gearbox.",
        }),
        this.derivedLine(() => {
          const j = this.model?.jointByName.get(jointName);
          if (!j || j.effort_limit == null) return "";
          const speed = angular ? `${fmt(j.velocity_limit / RAD, 0)} °/s` : `${fmt(j.velocity_limit * 1000, 0)} mm/s`;
          return `In effect: <b>${fmt(j.effort_limit, 1)} ${angular ? "N·m" : "N"}</b>, <b>${speed}</b>`;
        })
      );
    }
    return rows;
  }

  // -------------------------------------------------------------------------
  // Panels
  // -------------------------------------------------------------------------

  show(selection) {
    this.selection = selection;
    this.derived = [];
    const model = this.model;
    let body;
    try {
      if (!model) body = [el("p", { class: "hint" }, "Load a robot to edit it.")];
      else if (!selection) body = this.robotPanel();
      else if (selection.kind === "link") body = this.linkPanel(selection.id);
      else if (selection.kind === "transmission") body = this.transmissionPanel(selection.id);
      else body = this.lumpPanel(selection.kind, selection.id);
    } catch (e) {
      body = [el("p", { class: "hint error" }, `Cannot edit this element: ${e.message}`)];
    }
    this.host.replaceChildren(this.picker(), ...body);
  }

  refreshDerived() {
    for (const f of this.derived) f();
  }

  picker() {
    const model = this.model;
    const s = el("select", { class: "picker" });
    s.append(el("option", { value: "" }, "Robot overview"));
    if (model) {
      const g = (label, items) => el("optgroup", { label }, items.map(([v, t]) => el("option", { value: v }, t)));
      s.append(
        g("Links", model.links.map((l) => [`link:${l.name}`, l.name])),
        g("Transmissions / joints", model.actuated.map((n) => [`transmission:${n}`, `${n} transmission`])),
        g("Motors", model.drives.map((d) => [`motor:${d.joint}`, `${d.joint} motor (on ${d.motor.link})`])),
        g("Gearboxes", model.drives.map((d) => [`gearbox:${d.joint}`, `${d.joint} gearbox (on ${d.gearbox.link})`]))
      );
    }
    s.value = this.selection ? `${this.selection.kind}:${this.selection.id}` : "";
    s.addEventListener("change", () => {
      const cut = s.value.indexOf(":");
      this.ctx.onSelect(cut > 0 ? { kind: s.value.slice(0, cut), id: s.value.slice(cut + 1) } : null);
    });
    return el("div", { class: "picker-row" }, s);
  }

  robotPanel() {
    const model = this.model;
    const doc = this.doc;
    const b = model.massBudget;
    const pct = (v) => (b.total > 0 ? (100 * v) / b.total : 0);
    const budget = el("div", { class: "mass-budget" });
    budget.innerHTML = `
      <div class="bar">
        <span class="sw-link" style="width:${pct(b.structure)}%"></span>
        <span class="sw-motor" style="width:${pct(b.motors)}%"></span>
        <span class="sw-gearbox" style="width:${pct(b.gearboxes)}%"></span>
      </div>
      <div class="rows">
        <span class="sw sw-link"></span><span>Link structure (${model.links.length})</span><span class="v">${fmt(b.structure, 2)} kg</span>
        <span class="sw sw-motor"></span><span>Motors (${model.drives.length})</span><span class="v">${fmt(b.motors, 2)} kg</span>
        <span class="sw sw-gearbox"></span><span>Gearboxes (${model.drives.length})</span><span class="v">${fmt(b.gearboxes, 2)} kg</span>
        <span></span><span class="total">Total</span><span class="v total">${fmt(b.total, 2)} kg</span>
      </div>`;
    const warnings = model.warnings.length
      ? el("div", { class: "warnings" }, model.warnings.map((w) => el("div", {}, `⚠ ${w}`)))
      : null;

    const allEff = (kind, label, title) => {
      const vals = doc.drivenJoints().map((j) => doc.num(doc.part(j, kind), "efficiency", 1));
      const same = vals.every((v) => v === vals[0]);
      const out = el("span", { class: "eff-val" }, same ? fmtPct(vals[0] ?? 1) : "mixed");
      const input = el("input", { type: "range", min: 0.5, max: 1, step: 0.01, value: same ? vals[0] ?? 1 : 1 });
      const apply = (commit) => {
        doc.setEfficiencyAll(kind, Number(input.value));
        out.textContent = fmtPct(Number(input.value));
        this.ctx.onEdit(commit);
      };
      input.addEventListener("input", () => apply(false));
      input.addEventListener("change", () => apply(true));
      return el("label", { class: "f f-eff", title }, el("span", {}, label), input, out);
    };

    const joints = el(
      "div",
      { class: "joint-list" },
      model.actuated.map((name, i) => {
        const d = model.driveByJoint.get(name);
        const j = model.jointByName.get(name);
        const c = model.couplingByJoint.get(name);
        return el(
          "div",
          { class: "joint-item" },
          el("span", { class: "swatch", style: `background:${SERIES_COLORS[i % SERIES_COLORS.length]}` }),
          el("b", {}, name),
          el("span", { class: "meta" }, `${j.parent} → ${j.child}`),
          d && !d.colocated ? el("span", { class: "badge remote" }, "remote") : null,
          c
            ? el(
                "span",
                {
                  class: "badge coupled",
                  title: `${c.type} "${c.name}" with ${c.joints.filter((n) => n !== name).join(", ")}: `
                    + "both motors drive both joints",
                },
                c.type === "differential" ? "diff" : "coupled"
              )
            : null,
          el("span", { class: "grow" }),
          this.navButton("link", { kind: "link", id: j.child }),
          this.navButton("trans.", { kind: "transmission", id: name }),
          d ? this.navButton("motor", { kind: "motor", id: name }) : null,
          d ? this.navButton("gearbox", { kind: "gearbox", id: name }) : null
        );
      })
    );

    // Joints driven through a linkage instead of a motor: no DOF, no drive,
    // but they carry mass and their reaction lands on the joint they follow.
    const passive = model.joints.filter((j) => j.mimic);
    const linkages = !passive.length
      ? null
      : this.section(
          "Passive linkages",
          el(
            "div",
            { class: "joint-list" },
            passive.map((j) =>
              el(
                "div",
                { class: "joint-item" },
                el("b", {}, j.name),
                el("span", { class: "meta" }, `${j.parent} → ${j.child}`),
                el("span", { class: "badge passive" }, "passive"),
                el("span", { class: "grow" }),
                el(
                  "span",
                  { class: "meta" },
                  `= ${fmt(j.mimic.multiplier, 2)} × ${j.mimic.joint}`
                    + (j.mimic.offset ? ` + ${fmt(j.mimic.offset, 3)}` : "")
                ),
                this.navButton("link", { kind: "link", id: j.child })
              )
            )
          )
        );

    return [
      this.section("Mass budget", budget, warnings),
      this.section(
        "Losses, all joints",
        allEff("gearbox", "Gear efficiency", "Friction inside the gears, every joint"),
        allEff("transmission", "Joint + transmission", "Friction in the joints and belts/cables/linkages, every joint"),
        el("p", { class: "hint" }, "100% = lossless. Per-joint values are on each gearbox and transmission.")
      ),
      this.section("Joints", joints),
      linkages,
      el("p", { class: "hint" }, "Click a link, motor, gearbox or blue transmission ring in the view to edit it."),
    ];
  }

  linkPanel(name) {
    const doc = this.doc;
    const model = this.model;
    const link = model.linkByName.get(name);
    if (!link || !doc.link(name)) throw new Error(`no link ${name}`);
    const joint = model.jointByChild.get(name);
    const hosted = model.drives.flatMap((d) =>
      ["motor", "gearbox"].filter((k) => d[k].link === name).map((k) => this.navButton(`${d.joint} ${k}`, { kind: k, id: d.joint }))
    );

    const inertialBox = el("div");
    const auto = el("input", { type: "checkbox" });
    auto.checked = this.autoInertia.has(name);
    const syncInertia = () => {
      if (!this.autoInertia.has(name)) return;
      if (doc.estimateInertial(name)) renderInertial();
    };
    auto.addEventListener("change", () => {
      if (auto.checked) this.autoInertia.add(name);
      else this.autoInertia.delete(name);
      syncInertia();
      this.ctx.onEdit(true);
    });
    const renderInertial = () => {
      const inertial = doc.inertial(name, true);
      const massEl = doc.child(inertial, "mass", true);
      const origin = doc.origin(inertial, true);
      const I = doc.child(inertial, "inertia", true);
      const inertia = (k, label) => this.num(label, "kgm2", () => doc.num(I, k, 0), (v) => doc.setNum(I, k, v ?? 0));
      inertialBox.replaceChildren(
        this.num("Mass", "kg", () => doc.num(massEl, "value", 0), (v) => {
          doc.setNum(massEl, "value", v);
          syncInertia();
        }, { min: 0 }),
        this.vec3("COM", "mm", () => doc.vec(origin, "xyz"), (v) => doc.setVec(origin, "xyz", v)),
        el("div", { class: "grid2" }, inertia("ixx", "Ixx"), inertia("iyy", "Iyy"), inertia("izz", "Izz"), inertia("ixy", "Ixy"), inertia("ixz", "Ixz"), inertia("iyz", "Iyz")),
        el(
          "div",
          { class: "row-end" },
          el("label", { class: "check", title: "Recompute COM and inertia from the shapes whenever they or the mass change" }, auto, "keep in sync with shapes"),
          el("button", {
            type: "button",
            class: "ghost-btn small",
            title: "Solid shapes, uniform density, at the mass above",
            onclick: () => {
              if (doc.estimateInertial(name)) {
                renderInertial();
                this.ctx.onEdit(true);
              }
            },
          }, "Estimate from shapes")
        )
      );
    };
    renderInertial();

    const shapes = el("div");
    const renderShapes = () => {
      shapes.replaceChildren(
        ...doc.visuals(name).map((v) =>
          this.shapeEditor(v, {
            onGeometry: syncInertia,
            removable: () => {
              doc.remove(v);
              syncInertia();
              renderShapes();
              this.ctx.onEdit(true);
            },
          })
        ),
        el("button", {
          type: "button",
          class: "ghost-btn small",
          onclick: () => {
            doc.addVisual(name);
            syncInertia();
            renderShapes();
            this.ctx.onEdit(true);
          },
        }, "+ Add shape")
      );
    };
    renderShapes();

    return [
      el("h4", {}, "Link ", el("b", {}, name)),
      hosted.length ? el("div", { class: "navs" }, el("span", { class: "hint" }, "Mounted here:"), hosted) : null,
      this.section("Geometry", shapes),
      this.section("Mass properties", inertialBox),
      joint
        ? this.section(
            `Placement: joint ${joint.name} (on ${joint.parent})`,
            ...this.jointEditor(joint.name),
            el("div", { class: "navs" }, this.navButton(`${joint.name} transmission`, { kind: "transmission", id: joint.name }))
          )
        : el("p", { class: "hint" }, "Root link: fixed to the world."),
    ];
  }

  lumpPanel(kind, jointName) {
    const doc = this.doc;
    const node = doc.part(jointName, kind);
    if (!node) throw new Error(`joint ${jointName} has no ${kind}`);
    const massEl = doc.child(node, "mass", true);
    const links = doc.linkNames();
    const common = [
      this.select("Mounted on", links.map((l) => [l, l]), node.getAttribute("link"), (l) => {
        node.setAttribute("link", l);
        this.ctx.onEdit(true);
      }),
      this.num("Mass", "kg", () => doc.num(massEl, "value"), (v) => doc.setNum(massEl, "value", v), { min: 0 }),
      this.shapeEditor(node),
    ];
    const attr = (label, unit, name, opts = {}) =>
      this.num(label, unit, () => doc.num(node, name), (v) => doc.setNum(node, name, v), { optional: true, ...opts });
    const speed = (label, name) => {
      // Accept either spelling in the file; always write rad/s back.
      const get = () => doc.num(node, name) ?? (doc.num(node, `${name}_rpm`) != null ? doc.num(node, `${name}_rpm`) * RPM : null);
      return this.num(label, "rpm", get, (v) => {
        node.removeAttribute(`${name}_rpm`);
        doc.setNum(node, name, v);
      }, { optional: true });
    };
    const d = () => this.model?.driveByJoint.get(jointName);
    const nav = el(
      "div",
      { class: "navs" },
      this.navButton(`${jointName} transmission`, { kind: "transmission", id: jointName }),
      this.navButton(kind === "motor" ? `${jointName} gearbox` : `${jointName} motor`, { kind: kind === "motor" ? "gearbox" : "motor", id: jointName }),
      this.navButton(`host: ${node.getAttribute("link")}`, { kind: "link", id: node.getAttribute("link") })
    );

    if (kind === "motor") {
      return [
        el("h4", {}, "Motor ", el("b", {}, `${jointName}`)),
        nav,
        this.section("Body", ...common),
        this.section(
          "Ratings",
          attr("Peak torque", "Nm", "peak_torque", { title: "Current-limited peak torque" }),
          attr("Continuous torque", "Nm", "continuous_torque", { title: "Thermal (RMS) rating" }),
          attr("Stall torque", "Nm", "stall_torque", { title: "Voltage line intercept: τ = stall·(1 − ω/ω₀)" }),
          speed("Max speed (no-load)", "no_load_speed"),
          attr("Rotor inertia", "kgm2", "rotor_inertia", { optional: false }),
          attr("Torque constant", "Kt", "torque_constant"),
          attr("Resistance", "ohm", "resistance"),
          this.derivedLine(() => {
            const x = d();
            if (!x) return "";
            return `Reflected at the joint: <b>${fmt(x.reflected_inertia, 3)} kg·m²</b> (× ${fmt(x.total_ratio, 0)}²)`;
          }),
          this.derivedLine(() => {
            const c = this.model?.couplingByJoint.get(jointName);
            if (!c) return "";
            const others = c.joints.filter((n) => n !== jointName).join(", ");
            return c.type === "differential"
              ? `In the <b>${c.name}</b>: this motor and ${others}'s share both joints — `
                + `it carries half the sum of their torques, the other half the difference, `
                + `and each joint feels both rotors.`
              : `Coupled (<b>${c.name}</b>) with ${others}.`;
          })
        ),
      ];
    }
    return [
      el("h4", {}, "Gearbox ", el("b", {}, `${jointName}`)),
      nav,
      this.section("Body", ...common),
      this.section(
        "Ratings",
        attr("Gear ratio", "ratio", "ratio", { optional: false, min: 0 }),
        this.efficiency("Efficiency", () => node, "Gear friction as an efficiency (100% = lossless)"),
        attr("Peak output torque", "Nm", "peak_torque"),
        attr("Rated output torque", "Nm", "rated_torque", { title: "Continuous (RMS) rating, output side" }),
        speed("Max input speed", "max_input_speed"),
        attr("Input inertia", "kgm2", "input_inertia", { optional: false }),
        this.derivedLine(() => {
          const x = d();
          return x ? `Total ratio with transmission: <b>${fmt(x.total_ratio, 2)} : 1</b>` : "";
        })
      ),
    ];
  }

  transmissionPanel(jointName) {
    const doc = this.doc;
    const model = this.model;
    const j = model.jointByName.get(jointName);
    if (!j) throw new Error(`no joint ${jointName}`);
    const hasDrive = !!doc.drive(jointName);
    const tr = (create = true) => doc.part(jointName, "transmission", create);
    const rows = [];
    if (hasDrive) {
      rows.push(
        this.section(
          "Transmission",
          this.num("Ratio", "ratio", () => doc.num(doc.part(jointName, "transmission"), "ratio", 1), (v) => doc.setNum(tr(), "ratio", v ?? 1), {
            min: 0,
            title: "Extra reduction after the gearbox: belt, cable, linkage (1 = direct)",
          }),
          this.efficiency("Efficiency", tr, "Friction in the joint itself and any belt / cable / linkage (100% = lossless)")
        )
      );
    } else if (j.mimic) {
      rows.push(
        el(
          "p",
          { class: "hint" },
          `Passive linkage: this joint follows ${j.mimic.joint} `
            + `(× ${fmt(j.mimic.multiplier, 2)}${j.mimic.offset ? ` + ${fmt(j.mimic.offset, 3)}` : ""}). `
            + "It has no drive and no degree of freedom; its load goes back into the joint it follows."
        )
      );
    } else {
      rows.push(el("p", { class: "hint" }, "This joint has no <drive>; it is analysed as an ideal actuator."));
    }
    rows.push(this.section(`Joint ${jointName} (${j.parent} → ${j.child})`, ...this.jointEditor(jointName)));
    return [
      el("h4", {}, "Transmission ", el("b", {}, jointName)),
      el(
        "div",
        { class: "navs" },
        hasDrive ? this.navButton("motor", { kind: "motor", id: jointName }) : null,
        hasDrive ? this.navButton("gearbox", { kind: "gearbox", id: jointName }) : null,
        this.navButton(`link ${j.child}`, { kind: "link", id: j.child })
      ),
      ...rows,
    ];
  }
}
