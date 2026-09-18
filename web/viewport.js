// Three.js views of the arm. `ArmScene` is instantiated twice:
//
//   * the motion viewer: the arm posed along the trajectory, tool path, and a
//     dashed power-path line (motor -> gearbox -> joint) for remote drives;
//   * the URDF editor (`annotate: true`): the arm frozen at its zero pose with
//     link frames, joint axes, offset dimensions, COM markers with masses, and
//     a clickable ring per joint standing for its transmission.
//
// Scene setup (Z-up camera, lights, grid, ViewHelper cube, view presets and
// zoom-to-fit) follows Modular Robot Studio's viewport.js. The robot is built
// once per model as one Group per link; posing only rewrites those groups'
// matrices. Motors and gearboxes are children of the link they are mounted on.
//
// Selection is `{kind, id}`: kind is "link" (id = link name) or "motor",
// "gearbox", "transmission" (id = joint name).

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { ViewHelper } from "three/addons/helpers/ViewHelper.js";
import { CSS2DObject, CSS2DRenderer } from "three/addons/renderers/CSS2DRenderer.js";

import { matFromRowMajor, unionMeshBoundingBox } from "./math3.js";

export const COLORS = {
  link: 0x8b9099,
  motor: 0x5a9fd4,
  gearbox: 0xd9913a,
  transmission: 0x7ab8ff,
  selected: 0xffd23f,
  related: 0x7ab8ff,
  tip: 0x8de08a,
  path: 0x8de08a,
  axis: 0xffd23f,
  dim: 0x9aa3b2,
};

const RING_RADIUS = 0.03;

function geometryFor(shape) {
  if (shape.type === "box") {
    const [sx, sy, sz] = shape.params;
    return new THREE.BoxGeometry(sx, sy, sz);
  }
  if (shape.type === "sphere") return new THREE.SphereGeometry(shape.params[0], 24, 16);
  const [r, h] = shape.params;
  const geom = new THREE.CylinderGeometry(r, r, h, 32);
  geom.rotateX(Math.PI / 2); // URDF cylinders run along +Z; Three's along +Y.
  return geom;
}

function disposeTree(obj) {
  obj.traverse((o) => {
    o.geometry?.dispose?.();
    if (Array.isArray(o.material)) o.material.forEach((m) => m.dispose());
    else o.material?.dispose?.();
    if (o.isCSS2DObject) o.element.remove();
  });
}

function onTop(obj, order) {
  obj.traverse((o) => {
    if (o.material) {
      o.material.depthTest = false;
      o.material.transparent = true;
    }
    o.renderOrder = order;
  });
  return obj;
}

/** A screen-space text tag; `side` "right" starts at the anchor, "left" ends there. */
function label(text, cls = "", side = "right") {
  const el = document.createElement("div");
  el.className = `tag ${cls} tag-${side}`;
  el.textContent = text;
  const obj = new CSS2DObject(el);
  obj.center.set(side === "left" ? 1 : 0, 0.5);
  return obj;
}

const mm = (v) => `${Math.round(v * 1000)}`;

export class ArmScene {
  /**
   * @param {HTMLElement} host element the canvas (and labels) fill
   * @param {{annotate?: boolean, onPick?: (sel: object|null) => void, toggles?: object}} opts
   */
  constructor(host, { annotate = false, onPick = null, toggles = {} } = {}) {
    this.host = host;
    this.annotate = annotate;
    this.onPick = onPick;
    this.toggles = {
      grid: true,
      drives: true,
      xray: true,
      com: annotate,
      axes: annotate,
      path: true,
      dims: annotate,
      frames: annotate,
      names: annotate,
      ...toggles,
    };
    this.model = null;
    this.selection = null;
    this.related = null; // joint name whose parts get a softer highlight
    this.linkWorld = new Map();
    this._reset();

    const scene = (this.scene = new THREE.Scene());
    scene.background = new THREE.Color(0x14171c);
    const camera = (this.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100));
    camera.up.set(0, 0, 1);
    camera.position.set(1.7, 1.3, 1.1);

    const renderer = (this.renderer = new THREE.WebGLRenderer({ antialias: true }));
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.autoClear = false;
    host.appendChild(renderer.domElement);

    // Text labels: the editor's annotations, and trajectory waypoint names.
    this.labels = new CSS2DRenderer();
    Object.assign(this.labels.domElement.style, { position: "absolute", inset: "0", pointerEvents: "none" });
    host.appendChild(this.labels.domElement);

    const orbit = (this.orbit = new OrbitControls(camera, renderer.domElement));
    orbit.target.set(0, 0, 0.45);
    orbit.enableDamping = true;
    orbit.dampingFactor = 0.18;
    orbit.update();

    scene.add(new THREE.AmbientLight(0xffffff, 0.55));
    const key = new THREE.DirectionalLight(0xffffff, 0.85);
    key.position.set(2, 4, 3);
    scene.add(key);
    const fill = new THREE.DirectionalLight(0x9bc0ff, 0.35);
    fill.position.set(-3, -2, 2);
    scene.add(fill);

    this.grid = new THREE.GridHelper(2.4, 24, 0x3d4656, 0x2a303a);
    this.grid.rotation.x = Math.PI / 2;
    scene.add(this.grid);
    scene.add(new THREE.AxesHelper(0.12));

    this.raycaster = new THREE.Raycaster();
    // ViewHelper draws in the canvas's bottom-right corner and hit-tests
    // clicks against that same square, so it is given the canvas itself.
    this.viewHelper = new ViewHelper(camera, renderer.domElement);
    this.pendingZUpSnap = false;

    renderer.domElement.addEventListener("pointerdown", (down) => {
      const onUp = (up) => {
        if (up.pointerId !== down.pointerId) return;
        if ((up.clientX - down.clientX) ** 2 + (up.clientY - down.clientY) ** 2 > 25) return;
        this.viewHelper.center.copy(orbit.target);
        if (this.viewHelper.handleClick(up)) {
          this.pendingZUpSnap = true;
          return;
        }
        this.onPick?.(this._pick(up.clientX, up.clientY));
      };
      window.addEventListener("pointerup", onUp, { once: true });
    });

    new ResizeObserver(() => this._resize()).observe(host);
    this._resize();

    const clock = new THREE.Clock();
    const tick = () => {
      requestAnimationFrame(tick);
      const delta = clock.getDelta();
      if (this.viewHelper.animating) {
        this.viewHelper.update(delta);
      } else if (this.pendingZUpSnap) {
        this.pendingZUpSnap = false; // ViewHelper assumes Y-up; restore Z-up.
        camera.up.set(0, 0, 1);
        camera.lookAt(orbit.target);
      }
      orbit.update();
      renderer.clear();
      renderer.render(scene, camera);
      this.viewHelper.render(renderer);
      this.labels.render(scene, camera);
    };
    tick();
  }

  _reset() {
    this.root = null;
    this.linkGroups = new Map();
    this.pickables = [];
    this.parts = { link: [], motor: [], gearbox: [], transmission: [] };
    this.layers = { com: [], axes: [], dims: [], frames: [], names: [], drives: [], power: [] };
    this.powerLines = [];
  }

  _resize() {
    const w = this.host.clientWidth;
    const h = this.host.clientHeight;
    if (!w || !h) return;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
    this.labels.setSize(w, h);
  }

  _pick(clientX, clientY) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1
    );
    this.raycaster.setFromCamera(ndc, this.camera);
    const visible = this.pickables.filter((m) => {
      for (let o = m; o; o = o.parent) if (!o.visible) return false;
      return true;
    });
    const hits = this.raycaster.intersectObjects(visible, false);
    if (!hits.length) return null;
    // Drive parts and rings sit inside translucent links: prefer them.
    const hit = hits.find((h) => h.object.userData.kind !== "link") || hits[0];
    const { kind, id } = hit.object.userData;
    return { kind, id };
  }

  // -------------------------------------------------------------------------
  // Camera
  // -------------------------------------------------------------------------

  viewPreset(which) {
    const center = this.orbit.target.clone();
    let radius = this.camera.position.distanceTo(center);
    if (!isFinite(radius) || radius < 0.05) radius = 1.5;
    const dirs = {
      iso: new THREE.Vector3(1, 0.8, 0.6).normalize(),
      top: new THREE.Vector3(0, 0, 1),
      front: new THREE.Vector3(1, 0, 0),
      right: new THREE.Vector3(0, 1, 0),
    };
    this.camera.up.copy(which === "top" ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(0, 0, 1));
    this.camera.position.copy(center).addScaledVector(dirs[which] || dirs.iso, radius);
    this.camera.lookAt(center);
    this.orbit.update();
  }

  zoomToFit() {
    this.scene.updateMatrixWorld(true);
    this.fitToBox(unionMeshBoundingBox([...this.parts.link, ...this.parts.motor, ...this.parts.gearbox]));
  }

  /** Frame `box` (world coordinates); an empty or missing box frames a default volume. */
  fitToBox(box) {
    if (!box || box.isEmpty()) {
      box = new THREE.Box3(new THREE.Vector3(-0.3, -0.3, 0), new THREE.Vector3(0.3, 0.3, 0.9));
    }
    const center = box.getCenter(new THREE.Vector3());
    const r = Math.max(box.getBoundingSphere(new THREE.Sphere()).radius, 0.1);
    const dir = this.camera.position.clone().sub(this.orbit.target).normalize();
    if (!isFinite(dir.lengthSq()) || dir.lengthSq() === 0) dir.set(1, 0.8, 0.6).normalize();
    const fov = (this.camera.fov * Math.PI) / 180;
    this.orbit.target.copy(center);
    this.camera.position.copy(center).addScaledVector(dir, (r / Math.sin(fov / 2)) * 1.1);
    this.camera.near = Math.max(0.001, r * 0.02);
    this.camera.far = r * 100;
    this.camera.updateProjectionMatrix();
    this.orbit.update();
  }

  // -------------------------------------------------------------------------
  // Robot construction
  // -------------------------------------------------------------------------

  _mesh(shape, color, userData) {
    const mat = new THREE.MeshStandardMaterial({ color, metalness: 0.16, roughness: 0.68 });
    const mesh = new THREE.Mesh(geometryFor(shape), mat);
    mesh.matrixAutoUpdate = false;
    mesh.matrix.copy(matFromRowMajor(shape.T));
    mesh.userData = { ...userData, baseColor: color };
    this.pickables.push(mesh);
    this.parts[userData.kind].push(mesh);
    return mesh;
  }

  _com(parent, com, mass, color, text) {
    const r = Math.max(0.011 * Math.cbrt(Math.max(mass, 1e-3)), 0.004);
    const m = onTop(
      new THREE.Mesh(new THREE.SphereGeometry(r, 16, 12), new THREE.MeshBasicMaterial({ color, opacity: 0.9 })),
      20
    );
    m.position.set(com[0], com[1], com[2]);
    parent.add(m);
    this.layers.com.push(m);
    if (this.annotate && text) {
      const tag = label(text, "tag-com");
      tag.position.copy(m.position);
      parent.add(tag);
      this.layers.com.push(tag);
    }
  }

  /** Build the scene graph for a RobotModel; the camera is left where it is. */
  setModel(model) {
    if (this.root) {
      this.scene.remove(this.root);
      disposeTree(this.root);
    }
    this._reset();
    this.model = model;
    if (!model) return;
    const root = (this.root = new THREE.Group());

    for (const link of model.links) {
      const g = new THREE.Group();
      g.matrixAutoUpdate = false;
      for (const v of link.visuals) {
        const color = v.color ? new THREE.Color(v.color[0], v.color[1], v.color[2]).getHex() : COLORS.link;
        g.add(this._mesh(v, color, { kind: "link", id: link.name }));
      }
      if (link.mass > 0) this._com(g, link.com, link.mass, 0xc8d0e0, `${link.mass.toFixed(2)} kg`);
      for (const tip of link.tool_tips || []) {
        const s = new THREE.Mesh(new THREE.SphereGeometry(0.009, 16, 12), new THREE.MeshBasicMaterial({ color: COLORS.tip }));
        s.matrixAutoUpdate = false;
        s.matrix.copy(matFromRowMajor(tip.T));
        s.renderOrder = 15;
        g.add(s);
      }
      if (this.annotate) {
        const axes = onTop(new THREE.AxesHelper(0.06), 18);
        g.add(axes);
        this.layers.frames.push(axes);
        // Names hang left of the shape centre; COM masses sit to the right.
        const name = label(link.name, "tag-link", "left");
        const c = link.visuals[0]?.T;
        if (c) name.position.set(c[0][3], c[1][3], c[2][3]);
        g.add(name);
        this.layers.names.push(name);
      }
      this.linkGroups.set(link.name, g);
      root.add(g);
    }

    for (const j of model.joints) {
      if (j.type === "fixed") continue;
      const child = this.linkGroups.get(j.child);
      const arrow = onTop(
        new THREE.ArrowHelper(j.axisVec.clone(), new THREE.Vector3(), 0.12, COLORS.axis, 0.025, 0.014),
        25
      );
      child.add(arrow);
      this.layers.axes.push(arrow);
      if (this.annotate) {
        // Transmission ring: a clickable torus around the joint axis. A mimic
        // joint is a linkage, not a drive, so it gets no ring to click.
        if (!j.mimic) {
          const ring = new THREE.Mesh(
            new THREE.TorusGeometry(RING_RADIUS, 0.006, 10, 36),
            new THREE.MeshBasicMaterial({ color: COLORS.transmission })
          );
          ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), j.axisVec);
          ring.userData = { kind: "transmission", id: j.name, baseColor: COLORS.transmission };
          onTop(ring, 26);
          child.add(ring);
          this.pickables.push(ring);
          this.parts.transmission.push(ring);
        }
        const tag = label(j.name, "tag-joint");
        tag.position.copy(j.axisVec).multiplyScalar(0.13);
        child.add(tag);
        this.layers.names.push(tag);
        this._dimension(j);
      }
    }

    for (const d of model.drives) {
      for (const kind of ["motor", "gearbox"]) {
        const lump = d[kind];
        const host = this.linkGroups.get(lump.link);
        const mesh = this._mesh(lump.shape, COLORS[kind], { kind, id: d.joint });
        host.add(mesh);
        this.layers.drives.push(mesh);
        const T = lump.T;
        this._com(host, [T[0][3], T[1][3], T[2][3]], lump.mass, COLORS[kind], null);
      }
      if (!d.colocated) {
        const line = new THREE.Line(
          new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()]),
          new THREE.LineDashedMaterial({ color: COLORS.transmission, dashSize: 0.02, gapSize: 0.012, opacity: 0.85 })
        );
        onTop(line, 30);
        line.frustumCulled = false;
        root.add(line);
        this.layers.power.push(line);
        this.powerLines.push({
          line,
          drive: d,
          joint: model.jointByName.get(d.joint),
          motorT: matFromRowMajor(d.motor.T),
          gearboxT: matFromRowMajor(d.gearbox.T),
        });
      }
    }

    this.scene.add(root);
    this._applyToggles();
    this._applySelection();
    this.pose(model.defaultPose());
  }

  /** Offset legs (parent-frame X, then Y, then Z) from the parent link's
   * origin to a joint origin, labelled in millimetres. */
  _dimension(j) {
    const parent = this.linkGroups.get(j.parent);
    const o = j.origin;
    const t = [o[0][3], o[1][3], o[2][3]];
    const rpy = rpyOf(o);
    const rotated = rpy.some((v) => Math.abs(v) > 1e-6);
    if (Math.hypot(...t) < 1e-6 && !rotated) return;
    const pts = [new THREE.Vector3()];
    const p = new THREE.Vector3();
    for (let k = 0; k < 3; k++) {
      if (Math.abs(t[k]) < 1e-6) continue;
      p.setComponent(k, t[k]);
      pts.push(p.clone());
    }
    if (pts.length > 1) {
      const line = onTop(
        new THREE.Line(
          new THREE.BufferGeometry().setFromPoints(pts),
          new THREE.LineDashedMaterial({ color: COLORS.dim, dashSize: 0.012, gapSize: 0.008, opacity: 0.9 })
        ),
        17
      );
      line.computeLineDistances();
      parent.add(line);
      this.layers.dims.push(line);
    }
    const parts = ["x", "y", "z"].map((a, k) => (Math.abs(t[k]) >= 1e-6 ? `${a} ${mm(t[k])}` : null)).filter(Boolean);
    const rot = rotated ? ` · rpy ${rpy.map((v) => Math.round((v * 180) / Math.PI)).join("/")}°` : "";
    const tag = label(`${j.name}: ${parts.length ? parts.join(" · ") + " mm" : "0"}${rot}`, "tag-dim");
    tag.center.set(0, 1.4); // sit just above the leg, clear of COM labels on it
    // Label the longest leg, where there is most room for the text.
    let best = 1;
    for (let i = 2; i < pts.length; i++) {
      if (pts[i].distanceTo(pts[i - 1]) > pts[best].distanceTo(pts[best - 1])) best = i;
    }
    if (pts.length > 1) tag.position.copy(pts[best]).add(pts[best - 1]).multiplyScalar(0.5);
    parent.add(tag);
    this.layers.dims.push(tag);
  }

  // -------------------------------------------------------------------------
  // Posing
  // -------------------------------------------------------------------------

  pose(q) {
    if (!this.model) return;
    this.model.linkWorldTransforms(q, this.linkWorld);
    for (const [name, g] of this.linkGroups) {
      const T = this.linkWorld.get(name);
      if (T) {
        g.matrix.copy(T);
        g.matrixWorldNeedsUpdate = true;
      }
    }
    const p = new THREE.Vector3();
    const m = new THREE.Matrix4();
    for (const { line, drive, joint, motorT, gearboxT } of this.powerLines) {
      const pos = line.geometry.attributes.position;
      const set = (i, mat) => {
        p.setFromMatrixPosition(mat);
        pos.setXYZ(i, p.x, p.y, p.z);
      };
      set(0, m.multiplyMatrices(this.linkWorld.get(drive.motor.link), motorT));
      set(1, m.multiplyMatrices(this.linkWorld.get(drive.gearbox.link), gearboxT));
      set(2, this.linkWorld.get(joint.child));
      pos.needsUpdate = true;
      line.computeLineDistances();
    }
  }

  /** World pose of every tool tip at pose q: `{name, matrix}`. */
  toolTipPoses(q) {
    if (!this.model) return [];
    const Tw = this.model.linkWorldTransforms(q);
    const out = [];
    for (const link of this.model.links) {
      for (const tip of link.tool_tips || []) {
        out.push({ name: tip.name, matrix: matFromRowMajor(tip.T).premultiply(Tw.get(link.name)) });
      }
    }
    return out;
  }

  /** World position of every tool tip at pose q. */
  toolTipWorld(q) {
    if (!this.model) return [];
    const Tw = this.model.linkWorldTransforms(q);
    const out = [];
    for (const link of this.model.links) {
      for (const tip of link.tool_tips || []) {
        out.push(new THREE.Vector3().setFromMatrixPosition(matFromRowMajor(tip.T).premultiply(Tw.get(link.name))));
      }
    }
    return out;
  }

  /**
   * Tool path overlay: the traced path, plus (for cartesian trajectories)
   * labelled markers at the programmed waypoints.
   * @param {THREE.Vector3[]|null} points
   * @param {{name: string, xyz: number[]}[]} [waypoints]
   */
  setToolPath(points, waypoints = []) {
    if (this.toolPath) {
      this.scene.remove(this.toolPath);
      disposeTree(this.toolPath);
    }
    this.toolPath = null;
    if (!points || points.length < 2) return;
    const group = new THREE.Group();
    group.add(
      new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(points),
        new THREE.LineBasicMaterial({ color: COLORS.path, transparent: true, opacity: 0.7 })
      )
    );
    const seen = new Map();
    for (const w of waypoints) {
      const key = w.xyz.map((v) => v.toFixed(4)).join(",");
      if (seen.has(key)) {
        // Same spot visited twice (e.g. above_pick / retract_pick): one marker.
        const tag = seen.get(key);
        if (!tag.element.textContent.split(" / ").includes(w.name)) tag.element.textContent += ` / ${w.name}`;
        continue;
      }
      const m = new THREE.Mesh(
        new THREE.OctahedronGeometry(0.008),
        new THREE.MeshBasicMaterial({ color: COLORS.path })
      );
      m.position.set(...w.xyz);
      group.add(m);
      const tag = label(w.name, "tag-waypoint");
      tag.position.copy(m.position);
      group.add(tag);
      seen.set(key, tag);
    }
    this.toolPath = group;
    this.toolPath.visible = this.toggles.path;
    this.scene.add(this.toolPath);
  }

  // -------------------------------------------------------------------------
  // Toggles and selection
  // -------------------------------------------------------------------------

  setToggle(name, on) {
    this.toggles[name] = on;
    this._applyToggles();
  }

  _applyToggles() {
    const t = this.toggles;
    this.grid.visible = t.grid;
    const show = (list, on) => list.forEach((o) => (o.visible = on));
    show(this.layers.drives, t.drives);
    show(this.layers.power, t.drives);
    show(this.layers.com, t.com);
    show(this.layers.axes, t.axes);
    show(this.layers.dims, t.dims);
    show(this.layers.frames, t.frames);
    show(this.layers.names, t.names);
    if (this.toolPath) this.toolPath.visible = t.path;
    for (const m of this.parts.link) {
      m.material.transparent = t.xray;
      m.material.opacity = t.xray ? 0.35 : 1;
      m.material.depthWrite = !t.xray;
      m.material.needsUpdate = true;
    }
  }

  /** `selection` gets the strong highlight; parts of `relatedJoint` a soft one. */
  setSelection(selection, relatedJoint = null) {
    this.selection = selection;
    this.related = relatedJoint;
    this._applySelection();
  }

  _applySelection() {
    const sel = this.selection;
    const drivingJoint = (linkName) => this.model?.jointByChild.get(linkName)?.name;
    for (const m of this.pickables) {
      const { kind, id, baseColor } = m.userData;
      const exact = sel && sel.kind === kind && sel.id === id;
      const jointOfPart = kind === "link" ? drivingJoint(id) : id;
      const soft = !exact && this.related && jointOfPart === this.related;
      const color = exact ? COLORS.selected : soft && kind === "link" ? COLORS.related : baseColor;
      m.material.color.setHex(color);
      if (m.material.emissive) m.material.emissive.setHex(exact ? 0x4a3800 : soft ? 0x1a3050 : 0x000000);
    }
  }
}

function rpyOf(o) {
  // Inverse of R = Rz(yaw) Ry(pitch) Rx(roll), from a row-major 4x4.
  const sy = Math.hypot(o[0][0], o[1][0]);
  if (sy > 1e-8) return [Math.atan2(o[2][1], o[2][2]), Math.atan2(-o[2][0], sy), Math.atan2(o[1][0], o[0][0])];
  return [Math.atan2(-o[1][2], o[1][1]), Math.atan2(-o[2][0], sy), 0];
}
