// Cartesian trajectory editor (pop-out modal).
//
// Edits a working copy of a cartesian spec: waypoint list, the selected
// waypoint's pose / path type / timing, and the global speed limits. Waypoints
// can also be dragged and rotated in the 3-D view.
//
// The trajectory is robot-independent, so the view shows only the trajectory:
// the path, each waypoint's tool frame and the robot base frame it is
// expressed in. Every change asks /api/trajectory/cartesian/preview for the
// path shape (debounced). "Check reach" optionally adds a per-waypoint IK
// check against the robot currently loaded, without drawing it. Save hands
// the spec back to main.js, which solves IK for the loaded robot along the
// whole path through /api/analyze and only closes the editor if that works.
//
// Display units: mm, degrees, mm/s, deg/s. The spec stores metres and degrees.

import * as THREE from "three";
import { TransformControls } from "three/addons/controls/TransformControls.js";
import { CSS2DObject } from "three/addons/renderers/CSS2DRenderer.js";

import { api } from "./api.js";
import { ArmScene } from "./viewport.js";
import { fmt } from "./format.js";

const $ = (sel) => document.querySelector(sel);
const DEG = 180 / Math.PI;
const PREVIEW_DEBOUNCE_MS = 200;
const DEFAULT_LIMITS = { linear_speed: 0.15, smooth_speed: 0.5, angular_speed: 180 };
const COLORS = { ok: 0x8de08a, bad: 0xf87171, selected: 0xffd23f, path: 0x8de08a };

function el(tag, attrs = {}, ...children) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === "class") e.className = v;
    else if (k.startsWith("on")) e.addEventListener(k.slice(2), v);
    else if (v != null && v !== false) e.setAttribute(k, v === true ? "" : v);
  }
  for (const c of children.flat()) if (c != null) e.append(c);
  return e;
}

const round = (v, digits = 6) => Number(Number(v).toFixed(digits));

/** Deep copy with rpy in degrees and all optional blocks present. */
function normalize(spec) {
  const s = JSON.parse(JSON.stringify(spec));
  const toDeg = (s.units || "rad") === "rad" ? DEG : 1;
  s.format = "cartesian";
  s.units = "deg";
  s.limits = { ...DEFAULT_LIMITS, ...(s.limits || {}) };
  s.waypoints = (s.waypoints || []).map((w, i) => ({
    ...w,
    name: w.name || `wp${i}`,
    xyz: (w.xyz || [0, 0, 0]).map(Number),
    rpy: (w.rpy || [0, 0, 0]).map((v) => round(Number(v) * toDeg, 4)),
    path: i === 0 ? undefined : w.path || "smooth",
  }));
  return s;
}

export class TrajectoryEditor {
  /**
   * @param {{getUrdf: () => string, getModel: () => object|null,
   *          getToolPose: () => {xyz: number[], rpy: number[]}|null,
   *          onSave: (spec: object, label: string) => Promise<string|null>}} ctx
   *   onSave resolves to null on success or an error message.
   */
  constructor(ctx) {
    this.ctx = ctx;
    this.modal = $("#traj-modal");
    this.view = null;
    this.spec = null;
    this.sel = 0;
    this.preview = null;
    this.previewSeq = 0;
    this.history = [];
    this.future = [];
    this.dirty = false;
    this.markers = [];
    this.checkReach = false;
    this.wire();
  }

  get isOpen() {
    return !this.modal.hidden;
  }

  // -------------------------------------------------------------------------
  // Lifecycle
  // -------------------------------------------------------------------------

  open(spec, label) {
    this.spec = normalize(spec);
    this.label = label;
    $("#te-label").value = spec.label || label || "trajectory";
    this.sel = 0;
    this.history = [];
    this.future = [];
    this.dirty = false;
    this.modal.hidden = false;
    if (!this.view) this.createView();
    this.fitPending = true;
    const robot = this.ctx.getModel()?.name;
    $("#te-save").title = robot
      ? `Solve IK for ${robot} along the whole path and analyse it`
      : "Load a robot to solve IK and analyse";
    $("#te-check").title = robot
      ? `IK-check each waypoint against the loaded robot (${robot})`
      : "Load a robot to check reachability";
    this.renderTools();
    this.renderAll();
    this.setStatus("");
    this.requestPreview(true);
  }

  close(force = false) {
    if (!force && this.dirty && !confirm("Discard your trajectory edits?")) return;
    this.tc?.detach();
    this.modal.hidden = true;
  }

  createView() {
    const host = $("#te-view .view-host");
    // No robot model is ever set: the view shows only the trajectory.
    this.view = new ArmScene(host, { toggles: { path: true, com: false } });
    const { scene, camera, renderer, orbit } = this.view;
    this.group = new THREE.Group();
    scene.add(this.group);
    this.pathLine = null;

    this.tc = new TransformControls(camera, renderer.domElement);
    this.tc.setSize(0.8);
    this.tc.setSpace("local");
    scene.add(this.tc);
    this.tc.addEventListener("dragging-changed", (e) => {
      orbit.enabled = !e.value;
      if (!e.value) {
        this.commit();
        this.renderMarkers(); // deferred while dragging
      }
    });
    this.tc.addEventListener("objectChange", () => this.onGizmo());

    renderer.domElement.addEventListener("pointerdown", (down) => {
      const onUp = (up) => {
        if (up.pointerId !== down.pointerId) return;
        if ((up.clientX - down.clientX) ** 2 + (up.clientY - down.clientY) ** 2 > 25) return;
        if (this.tc.dragging || this.tc.axis) return;
        const rect = renderer.domElement.getBoundingClientRect();
        const ndc = new THREE.Vector2(
          ((up.clientX - rect.left) / rect.width) * 2 - 1,
          -((up.clientY - rect.top) / rect.height) * 2 + 1
        );
        const ray = new THREE.Raycaster();
        ray.setFromCamera(ndc, camera);
        const hit = ray.intersectObjects(this.markers.map((m) => m.userData.ball), false)[0];
        if (hit) this.select(hit.object.userData.index);
      };
      window.addEventListener("pointerup", onUp, { once: true });
    });
    // The robot base frame the poses are expressed in (the scene's world axes).
    const base = document.createElement("div");
    base.className = "tag tag-link tag-right";
    base.textContent = "robot base";
    scene.add(new CSS2DObject(base));
  }

  /** Frame the path and waypoints (plus the base frame). */
  fitToPath() {
    const box = new THREE.Box3(new THREE.Vector3(-0.05, -0.05, 0), new THREE.Vector3(0.05, 0.05, 0.05));
    for (const p of this.preview?.path || []) box.expandByPoint(new THREE.Vector3(...p));
    for (const w of this.spec?.waypoints || []) box.expandByPoint(new THREE.Vector3(...w.xyz));
    this.view.fitToBox(box);
  }

  // -------------------------------------------------------------------------
  // Edits and history
  // -------------------------------------------------------------------------

  /** Apply `fn` to the spec; `commit` records an undo step. */
  edit(fn, { commit = true, rerender = false } = {}) {
    if (commit) this.snapshot();
    fn(this.spec);
    this.dirty = true;
    if (rerender) this.renderAll();
    else this.renderMarkers();
    this.requestPreview();
  }

  snapshot() {
    this.history.push(JSON.stringify(this.spec));
    if (this.history.length > 200) this.history.shift();
    this.future = [];
  }

  /** Called when a drag ends: the drag itself was applied live without history. */
  commit() {
    if (this._dragStart) {
      this.history.push(this._dragStart);
      this.future = [];
      this._dragStart = null;
    }
  }

  undo() {
    if (!this.history.length) return;
    this.future.push(JSON.stringify(this.spec));
    this.spec = JSON.parse(this.history.pop());
    this.sel = Math.min(this.sel, this.spec.waypoints.length - 1);
    this.renderAll();
    this.requestPreview();
  }

  redo() {
    if (!this.future.length) return;
    this.history.push(JSON.stringify(this.spec));
    this.spec = JSON.parse(this.future.pop());
    this.sel = Math.min(this.sel, this.spec.waypoints.length - 1);
    this.renderAll();
    this.requestPreview();
  }

  select(i) {
    this.sel = Math.max(0, Math.min(i, this.spec.waypoints.length - 1));
    this.renderList();
    this.renderDetail();
    this.renderMarkers();
  }

  onGizmo() {
    const obj = this.tc.object;
    if (!obj) return;
    if (!this._dragStart) this._dragStart = JSON.stringify(this.spec);
    const i = obj.userData.index;
    const w = this.spec.waypoints[i];
    w.xyz = obj.position.toArray().map((v) => round(v));
    const e = new THREE.Euler().setFromQuaternion(obj.quaternion, "ZYX");
    w.rpy = [e.x, e.y, e.z].map((v) => round(v * DEG, 4));
    this.dirty = true;
    this.renderDetail();
    this.requestPreview();
  }

  // -------------------------------------------------------------------------
  // Preview
  // -------------------------------------------------------------------------

  requestPreview(immediate = false) {
    clearTimeout(this._previewTimer);
    this._previewTimer = setTimeout(() => this.runPreview(), immediate ? 0 : PREVIEW_DEBOUNCE_MS);
  }

  async runPreview() {
    const seq = ++this.previewSeq;
    let result;
    try {
      const urdf = this.checkReach ? this.ctx.getUrdf() : null;
      result = await api.cartesianPreview(urdf, this.spec);
    } catch (e) {
      if (seq !== this.previewSeq) return;
      this.preview = null;
      this.setStatus(e.message, "bad");
      this.renderPath();
      this.renderMarkers();
      this.renderList();
      return;
    }
    if (seq !== this.previewSeq) return;
    this.preview = result;
    const checked = result.waypoints.some((w) => w.reachable !== null);
    const bad = result.waypoints.filter((w) => w.reachable === false).map((w) => w.name);
    if (!checked) this.setStatus(`${fmt(result.duration, 2)} s`, "");
    else if (bad.length) this.setStatus(`Unreachable for ${this.ctx.getModel()?.name}: ${[...new Set(bad)].join(", ")}`, "bad");
    else this.setStatus(`${fmt(result.duration, 2)} s · all waypoints reachable for ${this.ctx.getModel()?.name}`, "good");
    $("#te-duration").textContent = `${fmt(result.duration, 2)} s`;
    this.renderPath();
    // Rebuilding markers detaches the gizmo; wait for the drag to finish.
    if (!this.tc.dragging) this.renderMarkers();
    this.renderList();
    this.renderReachHint();
    if (this.fitPending) {
      this.fitPending = false;
      this.fitToPath();
    }
  }

  setCheckReach(on) {
    this.checkReach = on && !!this.ctx.getModel();
    $("#te-check").dataset.on = this.checkReach ? "1" : "0";
    this.requestPreview(true);
  }

  setStatus(msg, kind = "") {
    const s = $("#te-status");
    s.textContent = msg;
    s.title = msg;
    s.className = `status-msg ${kind}`;
  }

  // -------------------------------------------------------------------------
  // Rendering
  // -------------------------------------------------------------------------

  renderAll() {
    this.renderSettings();
    this.renderList();
    this.renderDetail();
    this.renderMarkers();
    this.renderPath();
  }

  renderPath() {
    if (this.pathLine) {
      this.group.remove(this.pathLine);
      this.pathLine.geometry.dispose();
      this.pathLine = null;
    }
    const pts = this.preview?.path;
    if (!pts || pts.length < 2) return;
    this.pathLine = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pts.map((p) => new THREE.Vector3(...p))),
      new THREE.LineBasicMaterial({ color: COLORS.path, transparent: true, opacity: 0.8 })
    );
    this.group.add(this.pathLine);
  }

  renderMarkers() {
    const attachedIndex = this.tc.object?.userData.index;
    this.tc.detach();
    for (const m of this.markers) {
      this.group.remove(m);
      m.traverse((o) => {
        o.geometry?.dispose?.();
        o.material?.dispose?.();
        if (o.isCSS2DObject) o.element.remove();
      });
    }
    this.markers = [];
    const reach = this.preview?.waypoints || [];
    const drawn = new Map();
    this.spec.waypoints.forEach((w, i) => {
      const g = new THREE.Group();
      g.position.set(...w.xyz);
      g.quaternion.setFromEuler(new THREE.Euler(...w.rpy.map((v) => v / DEG), "ZYX"));
      g.userData.index = i;
      const selected = i === this.sel;
      const ok = reach[i]?.reachable !== false;
      const ball = new THREE.Mesh(
        new THREE.SphereGeometry(selected ? 0.013 : 0.009, 16, 12),
        new THREE.MeshBasicMaterial({
          color: selected ? COLORS.selected : ok ? COLORS.ok : COLORS.bad,
          depthTest: false,
          transparent: true,
        })
      );
      ball.renderOrder = 40;
      ball.userData.index = i;
      g.add(ball);
      g.userData.ball = ball;
      // Tool frame: z (blue) is the approach axis.
      const axes = new THREE.AxesHelper(selected ? 0.06 : 0.035);
      axes.material.depthTest = false;
      axes.renderOrder = 39;
      g.add(axes);
      // One label per distinct spot.
      const key = w.xyz.map((v) => v.toFixed(4)).join(",");
      if (drawn.has(key)) {
        drawn.get(key).element.textContent += ` / ${i + 1}`;
      } else {
        const tag = document.createElement("div");
        tag.className = `tag tag-waypoint tag-right${ok ? "" : " tag-bad"}`;
        tag.textContent = `${i + 1}`;
        const obj = new CSS2DObject(tag);
        obj.center.set(0, 0.5);
        g.add(obj);
        drawn.set(key, obj);
      }
      this.group.add(g);
      this.markers.push(g);
    });
    const target = this.markers[this.sel] ?? this.markers[attachedIndex];
    if (target) this.tc.attach(target);
  }

  /** Tool name is free text (the trajectory is robot-independent); the
   * loaded robot's tool tips are offered as suggestions. */
  renderTools() {
    const model = this.ctx.getModel();
    const tips = (model?.links || []).flatMap((l) => (l.tool_tips || []).map((t) => t.name));
    $("#te-tool-list").replaceChildren(...tips.map((t) => el("option", { value: t })));
    $("#te-tool").value = this.spec?.tool || "";
  }

  renderSettings() {
    const L = this.spec.limits;
    $("#te-linear").value = round(L.linear_speed * 1000, 3);
    $("#te-smooth").value = round(L.smooth_speed * 1000, 3);
    $("#te-angular").value = round(L.angular_speed, 3);
    $("#te-tool").value = this.spec.tool || "";
  }

  renderList() {
    const reach = this.preview?.waypoints || [];
    const list = $("#te-list");
    list.replaceChildren(
      ...this.spec.waypoints.map((w, i) => {
        const r = reach[i];
        const status = !r || r.reachable === null
          ? el("span", { class: "te-st" }, "")
          : r.reachable
            ? el("span", { class: "te-st ok", title: "reachable" }, "✓")
            : el("span", { class: "te-st bad", title: `missed by ${fmt(r.position_error * 1000, 1)} mm` }, "✗");
        return el(
          "div",
          { class: `te-row${i === this.sel ? " selected" : ""}`, onclick: () => this.select(i) },
          el("span", { class: "te-idx" }, `${i + 1}`),
          el("span", { class: "te-name" }, w.name),
          el("span", { class: `te-path ${w.path || "start"}` }, i === 0 ? "start" : w.path),
          el("span", { class: "te-xyz" }, w.xyz.map((v) => fmt(v * 1000, 0)).join(", ")),
          w.dwell ? el("span", { class: "te-dwell", title: "dwell" }, `⏸ ${fmt(w.dwell, 1)} s`) : null,
          status
        );
      })
    );
  }

  numField(label, unit, value, onValue, { step = 1, optional = false, placeholder = "", disabled = false } = {}) {
    const input = el("input", { type: "number", step, value: value ?? "", placeholder, disabled });
    let started = false;
    input.addEventListener("input", () => {
      const raw = input.value.trim();
      if (raw === "" && !optional) return;
      const v = raw === "" ? null : Number(raw);
      if (v !== null && !Number.isFinite(v)) return;
      this.edit((s) => onValue(v, s), { commit: !started });
      started = true;
    });
    input.addEventListener("change", () => (started = false));
    return el("label", { class: "f" }, el("span", {}, label), input, el("em", {}, unit));
  }

  trio(label, unit, values, onValue, names, step) {
    const inputs = values.map((v, k) => el("input", { type: "number", step, value: v, title: names[k] }));
    let started = false;
    for (const input of inputs) {
      input.addEventListener("input", () => {
        const vs = inputs.map((x) => Number(x.value));
        if (vs.some((v) => !Number.isFinite(v))) return;
        this.edit((s) => onValue(vs, s), { commit: !started });
        started = true;
      });
      input.addEventListener("change", () => (started = false));
    }
    return el("div", { class: "f f3" }, el("span", {}, label), el("div", { class: "trio" }, inputs), el("em", {}, unit));
  }

  renderDetail() {
    const host = $("#te-detail");
    const i = this.sel;
    const w = this.spec.waypoints[i];
    if (!w) {
      host.replaceChildren(el("p", { class: "hint" }, "No waypoints."));
      return;
    }
    const at = (s) => s.waypoints[i];
    const nameInput = el("input", { type: "text", value: w.name });
    nameInput.addEventListener("change", () => this.edit((s) => (at(s).name = nameInput.value.trim() || `wp${i}`), { rerender: true }));
    const pathSel = el(
      "select",
      { disabled: i === 0 },
      el("option", { value: "linear" }, "linear (straight line)"),
      el("option", { value: "smooth" }, "smooth (curved)")
    );
    pathSel.value = w.path || "smooth";
    pathSel.addEventListener("change", () => this.edit((s) => (at(s).path = pathSel.value), { rerender: true }));
    const r = this.preview?.waypoints?.[i];
    host.replaceChildren(
      el("div", { class: "f" }, el("span", {}, "Name"), nameInput, el("em")),
      el("div", { class: "f" }, el("span", {}, "Path to here"), pathSel, el("em")),
      this.trio("Position", "mm", w.xyz.map((v) => round(v * 1000, 3)), (vs, s) => (at(s).xyz = vs.map((v) => round(v / 1000))), ["x", "y", "z"], 1),
      this.trio("Orientation", "°", w.rpy, (vs, s) => (at(s).rpy = vs), ["roll", "pitch", "yaw"], 1),
      el(
        "div",
        { class: "row-end" },
        el("button", {
          type: "button",
          class: "ghost-btn small",
          title: "Tool z straight down, keeping the yaw",
          onclick: () => this.edit((s) => (at(s).rpy = [0, 180, at(s).rpy[2]]), { rerender: true }),
        }, "Point tool down")
      ),
      this.numField("Dwell", "s", w.dwell ?? "", (v, s) => (v ? (at(s).dwell = v) : delete at(s).dwell), { step: 0.1, optional: true, placeholder: "0" }),
      this.numField("Speed", "mm/s", w.speed != null ? round(w.speed * 1000, 3) : "", (v, s) => (v == null ? delete at(s).speed : (at(s).speed = v / 1000)), {
        step: 10,
        optional: true,
        placeholder: "limit",
        disabled: i === 0,
      }),
      this.numField("Duration", "s", w.duration ?? "", (v, s) => (v == null ? delete at(s).duration : (at(s).duration = v)), {
        step: 0.1,
        optional: true,
        placeholder: "auto",
        disabled: i === 0,
      }),
      el("p", { class: "hint", id: "te-reach-hint" })
    );
    this.renderReachHint();
  }

  renderReachHint() {
    const hint = $("#te-reach-hint");
    if (!hint) return;
    const r = this.preview?.waypoints?.[this.sel];
    const robot = this.ctx.getModel()?.name;
    hint.textContent =
      !r || r.reachable === null
        ? ""
        : r.reachable
          ? `Reachable by ${robot}.`
          : `Not reachable by ${robot}: missed by ${fmt(r.position_error * 1000, 1)} mm / ${fmt(r.orientation_error * DEG, 1)}°.`;
  }

  // -------------------------------------------------------------------------
  // Wiring
  // -------------------------------------------------------------------------

  wire() {
    $("#te-cancel").addEventListener("click", () => this.close());
    $("#te-save").addEventListener("click", () => this.save());
    $("#te-download").addEventListener("click", () => this.download());
    $("#te-undo").addEventListener("click", () => this.undo());
    $("#te-redo").addEventListener("click", () => this.redo());

    const limit = (id, key, k) =>
      $(id).addEventListener("input", (e) => {
        const v = Number(e.target.value);
        if (v > 0) this.edit((s) => (s.limits[key] = v * k));
      });
    limit("#te-linear", "linear_speed", 1 / 1000);
    limit("#te-smooth", "smooth_speed", 1 / 1000);
    limit("#te-angular", "angular_speed", 1);
    $("#te-tool").addEventListener("change", (e) => {
      const name = e.target.value.trim();
      this.edit((s) => (name ? (s.tool = name) : delete s.tool));
    });
    $("#te-check").addEventListener("click", () => this.setCheckReach(!this.checkReach));

    const insertAfter = (make) => {
      this.edit((s) => s.waypoints.splice(this.sel + 1, 0, make(s.waypoints[this.sel])), { rerender: true });
      this.select(this.sel + 1);
    };
    $("#te-add").addEventListener("click", () =>
      insertAfter((w) => ({ ...JSON.parse(JSON.stringify(w)), name: `wp${this.spec.waypoints.length}`, xyz: [w.xyz[0], w.xyz[1], round(w.xyz[2] + 0.05)], path: "smooth", dwell: undefined }))
    );
    $("#te-dup").addEventListener("click", () =>
      insertAfter((w) => ({ ...JSON.parse(JSON.stringify(w)), name: `${w.name}_copy`, path: w.path || "linear" }))
    );
    $("#te-del").addEventListener("click", () => {
      if (this.spec.waypoints.length <= 2) return this.setStatus("A trajectory needs at least two waypoints", "bad");
      this.edit((s) => {
        s.waypoints.splice(this.sel, 1);
        delete s.waypoints[0].path;
      }, { rerender: true });
      this.select(Math.min(this.sel, this.spec.waypoints.length - 1));
    });
    const move = (d) => {
      const j = this.sel + d;
      if (j < 0 || j >= this.spec.waypoints.length) return;
      this.edit((s) => {
        const w = s.waypoints;
        [w[this.sel], w[j]] = [w[j], w[this.sel]];
        delete w[0].path;
        for (let k = 1; k < w.length; k++) w[k].path ||= "smooth";
      }, { rerender: true });
      this.select(j);
    };
    $("#te-up").addEventListener("click", () => move(-1));
    $("#te-down").addEventListener("click", () => move(1));
    $("#te-from-pose").addEventListener("click", () => {
      const pose = this.ctx.getToolPose();
      if (!pose) return this.setStatus("No tool pose available", "bad");
      this.edit((s) => {
        s.waypoints[this.sel].xyz = pose.xyz.map((v) => round(v));
        s.waypoints[this.sel].rpy = pose.rpy.map((v) => round(v, 4));
      }, { rerender: true });
    });

    this.modal.querySelectorAll("[data-te-view]").forEach((b) =>
      b.addEventListener("click", () => this.view?.viewPreset(b.dataset.teView))
    );
    $("#te-fit").addEventListener("click", () => this.fitToPath());
    this.modal.querySelectorAll("[data-te-mode]").forEach((b) =>
      b.addEventListener("click", () => {
        this.tc.setMode(b.dataset.teMode);
        this.modal.querySelectorAll("[data-te-mode]").forEach((x) => (x.dataset.on = x === b ? "1" : "0"));
      })
    );

    window.addEventListener("keydown", (e) => {
      if (!this.isOpen) return;
      if (e.key === "Escape") {
        e.preventDefault();
        this.close();
        return;
      }
      if (e.target.closest("input, select, textarea")) return;
      const mod = e.ctrlKey || e.metaKey;
      if (mod && e.key.toLowerCase() === "z") {
        e.preventDefault();
        if (e.shiftKey) this.redo();
        else this.undo();
      } else if (e.key === "w" || e.key === "e") {
        this.modal.querySelector(`[data-te-mode="${e.key === "w" ? "translate" : "rotate"}"]`).click();
      }
    });
  }

  specForSave() {
    const s = JSON.parse(JSON.stringify(this.spec));
    s.label = $("#te-label").value.trim() || this.label;
    for (const w of s.waypoints) {
      for (const k of ["dwell", "speed", "duration"]) if (w[k] == null) delete w[k];
    }
    return s;
  }

  async save() {
    const spec = this.specForSave();
    const btn = $("#te-save");
    btn.disabled = true;
    this.setStatus("Solving IK along the path…", "busy");
    try {
      const error = await this.ctx.onSave(spec, spec.label);
      if (error) {
        this.setStatus(error, "bad");
        return;
      }
      this.dirty = false;
      this.close(true);
    } finally {
      btn.disabled = false;
    }
  }

  download() {
    const spec = this.specForSave();
    const name = (spec.label || "trajectory").replace(/[^A-Za-z0-9_-]+/g, "_").replace(/^_|_$/g, "") || "trajectory";
    const a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([JSON.stringify(spec, null, 2) + "\n"], { type: "application/json" }));
    a.download = `${name}.json`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  }
}
