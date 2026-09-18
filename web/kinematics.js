// Client-side forward kinematics and trajectory-plan evaluation.
//
// Derived from Modular Robot Studio's kinematics.js. The server sends one
// pose-independent model (/api/robot/model) and the piecewise-cubic plan
// (/api/analyze); posing and playback happen here, so scrubbing the
// timeline costs no requests. Frames match the Python side exactly: a joint's
// `origin` maps child coordinates into parent coordinates, and its axis is
// constant in the child link frame. tests/test_frontend_kinematics.py holds
// the two implementations together.

import * as THREE from "three";
import { matFromRowMajor } from "./math3.js";

export class RobotModel {
  /** @param {object} payload response from /api/robot/model */
  constructor(payload) {
    this.name = payload.name;
    this.rootLink = payload.root_link;
    this.links = payload.links || [];
    this.joints = (payload.joints || []).map((j) => ({
      ...j,
      originMat: matFromRowMajor(j.origin),
      axisVec: new THREE.Vector3(j.axis[0], j.axis[1], j.axis[2]).normalize(),
    }));
    this.actuated = payload.actuated || [];
    this.drives = payload.drives || [];
    this.couplings = payload.couplings || [];
    this.massBudget = payload.mass_budget || {};
    this.warnings = payload.warnings || [];

    this.jointByName = new Map(this.joints.map((j) => [j.name, j]));
    this.jointByChild = new Map(this.joints.map((j) => [j.child, j]));
    this.driveByJoint = new Map(this.drives.map((d) => [d.joint, d]));
    this.couplingByJoint = new Map();
    for (const c of this.couplings) for (const n of c.joints) this.couplingByJoint.set(n, c);
    this.linkByName = new Map(this.links.map((l) => [l.name, l]));
    this.order = this._topologicalOrder();
    this._scratch = new THREE.Matrix4();
    this._scratchQ = new THREE.Quaternion();
    this._scratchV = new THREE.Vector3();
  }

  _topologicalOrder() {
    const childrenOf = new Map();
    for (const j of this.joints) {
      if (!childrenOf.has(j.parent)) childrenOf.set(j.parent, []);
      childrenOf.get(j.parent).push(j);
    }
    const order = [];
    const seen = new Set([this.rootLink]);
    const queue = [this.rootLink];
    while (queue.length) {
      const cur = queue.shift();
      for (const j of childrenOf.get(cur) || []) {
        if (seen.has(j.child)) continue;
        seen.add(j.child);
        order.push(j.child);
        queue.push(j.child);
      }
    }
    return order;
  }

  /** Parent-link -> child-link matrix for one joint at position q. */
  _jointMatrix(j, q, out) {
    out.copy(j.originMat);
    if (j.type === "revolute") {
      this._scratchQ.setFromAxisAngle(j.axisVec, q);
      this._scratch.makeRotationFromQuaternion(this._scratchQ);
      out.multiply(this._scratch);
    } else if (j.type === "prismatic") {
      this._scratchV.copy(j.axisVec).multiplyScalar(q);
      this._scratch.makeTranslation(this._scratchV.x, this._scratchV.y, this._scratchV.z);
      out.multiply(this._scratch);
    }
    return out;
  }

  /**
   * World transform per link for a joint vector.
   * @param {Record<string, number>} q
   * @param {Map<string, THREE.Matrix4>} [out] reused between calls if given
   * @returns {Map<string, THREE.Matrix4>}
   */
  /** Joint value, following a <mimic> linkage back to the joint that drives it. */
  jointValue(j, q) {
    if (j.mimic) {
      const src = Number.isFinite(q[j.mimic.joint]) ? q[j.mimic.joint] : 0;
      return j.mimic.multiplier * src + j.mimic.offset;
    }
    return Number.isFinite(q[j.name]) ? q[j.name] : 0;
  }

  linkWorldTransforms(q = {}, out = new Map()) {
    const root = out.get(this.rootLink) || new THREE.Matrix4();
    root.identity();
    out.set(this.rootLink, root);
    const local = new THREE.Matrix4();
    for (const link of this.order) {
      const j = this.jointByChild.get(link);
      const parent = out.get(j.parent);
      const value = this.jointValue(j, q);
      this._jointMatrix(j, value, local);
      const m = out.get(link) || new THREE.Matrix4();
      m.multiplyMatrices(parent, local);
      out.set(link, m);
    }
    return out;
  }

  defaultPose() {
    const out = {};
    for (const name of this.actuated) {
      const j = this.jointByName.get(name);
      out[name] = Math.min(Math.max(0, j.lower), j.upper);
    }
    return out;
  }
}

// ---------------------------------------------------------------------------
// Trajectory plan evaluation: the server plans (or spline-fits) and hands back
// piecewise cubics; we only evaluate them.
// ---------------------------------------------------------------------------

export class PlanEvaluator {
  constructor(plan) {
    this.plan = plan || { segments: [], joints: [], duration: 0 };
    this.segments = this.plan.segments || [];
    this.joints = this.plan.joints || [];
    this.duration = this.plan.duration || 0;
    this.waypointTimes = this.plan.waypoint_times || [];
  }

  get isEmpty() {
    return this.segments.length === 0;
  }

  _segmentAt(t) {
    if (!this.segments.length) return null;
    if (t <= this.segments[0].t0) return this.segments[0];
    let lo = 0;
    let hi = this.segments.length - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (t <= this.segments[mid].t1) hi = mid;
      else lo = mid + 1;
    }
    return this.segments[lo];
  }

  /** @returns {{q: object, qd: object, qdd: object}} */
  sample(t) {
    const q = {};
    const qd = {};
    const qdd = {};
    const seg = this._segmentAt(t);
    if (!seg) return { q, qd, qdd };
    const span = seg.t1 - seg.t0;
    const dt = Math.min(Math.max(t - seg.t0, 0), Math.max(span, 0));
    for (const [name, c] of Object.entries(seg.coeffs)) {
      const [c0, c1, c2, c3] = c;
      q[name] = c0 + dt * (c1 + dt * (c2 + dt * c3));
      qd[name] = c1 + dt * (2 * c2 + 3 * c3 * dt);
      qdd[name] = 2 * c2 + 6 * c3 * dt;
    }
    return { q, qd, qdd };
  }

  positionsAt(t) {
    return this.sample(t).q;
  }
}
