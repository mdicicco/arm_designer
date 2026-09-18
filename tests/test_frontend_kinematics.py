"""Cross-check the browser's kinematics against the Python implementation.

``web/kinematics.js`` implements forward kinematics and plan evaluation so
the viewport can pose the arm without a round trip. That is only safe if it
agrees exactly with the Pinocchio model the torques are computed on,
otherwise the picture and the torque numbers describe different motions. These tests run the real JS module under Node
with a minimal Three.js stub, and skip when Node is not installed.

(Adapted from the modular_robot test of the same name.)
"""

import json
import math
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from arm_analyzer.pin_model import build_model, frame_placements
from arm_analyzer.trajectory import load_trajectory

WEB = Path(__file__).resolve().parents[1] / "web"

THREE_STUB = """
// Minimal Three.js stand-in: Matrix4.elements is column-major, as in three.js.
export class Vector3 {
  constructor(x=0,y=0,z=0){this.x=x;this.y=y;this.z=z;}
  set(x,y,z){this.x=x;this.y=y;this.z=z;return this;}
  copy(v){this.x=v.x;this.y=v.y;this.z=v.z;return this;}
  clone(){return new Vector3(this.x,this.y,this.z);}
  multiplyScalar(s){this.x*=s;this.y*=s;this.z*=s;return this;}
  length(){return Math.hypot(this.x,this.y,this.z);}
  normalize(){const l=this.length()||1;return this.multiplyScalar(1/l);}
  setFromMatrixPosition(m){const e=m.elements;this.x=e[12];this.y=e[13];this.z=e[14];return this;}
}
export class Quaternion {
  constructor(x=0,y=0,z=0,w=1){this.x=x;this.y=y;this.z=z;this.w=w;}
  setFromAxisAngle(a,angle){const h=angle/2,s=Math.sin(h);
    this.x=a.x*s;this.y=a.y*s;this.z=a.z*s;this.w=Math.cos(h);return this;}
}
export class Matrix4 {
  constructor(){this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];}
  set(n11,n12,n13,n14,n21,n22,n23,n24,n31,n32,n33,n34,n41,n42,n43,n44){
    const e=this.elements;
    e[0]=n11;e[4]=n12;e[8]=n13;e[12]=n14;
    e[1]=n21;e[5]=n22;e[9]=n23;e[13]=n24;
    e[2]=n31;e[6]=n32;e[10]=n33;e[14]=n34;
    e[3]=n41;e[7]=n42;e[11]=n43;e[15]=n44;
    return this;}
  identity(){this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];return this;}
  copy(m){this.elements=m.elements.slice();return this;}
  clone(){return new Matrix4().copy(this);}
  multiplyMatrices(a,b){
    const ae=a.elements,be=b.elements,te=this.elements;
    for(let c=0;c<4;c++)for(let r=0;r<4;r++){
      let s=0;for(let k=0;k<4;k++)s+=ae[k*4+r]*be[c*4+k];
      te[c*4+r]=s;}
    return this;}
  multiply(m){return this.multiplyMatrices(this.clone(),m);}
  makeRotationFromQuaternion(q){
    const {x,y,z,w}=q,x2=x+x,y2=y+y,z2=z+z,xx=x*x2,xy=x*y2,xz=x*z2,
      yy=y*y2,yz=y*z2,zz=z*z2,wx=w*x2,wy=w*y2,wz=w*z2;
    return this.set(1-(yy+zz),xy-wz,xz+wy,0, xy+wz,1-(xx+zz),yz-wx,0,
                    xz-wy,yz+wx,1-(xx+yy),0, 0,0,0,1);}
  makeTranslation(x,y,z){return this.set(1,0,0,x,0,1,0,y,0,0,1,z,0,0,0,1);}
  invert(){
    const m=[];for(let r=0;r<4;r++){m.push([]);for(let c=0;c<4;c++)m[r].push(this.elements[c*4+r]);}
    const inv=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
    for(let i=0;i<4;i++){
      let p=i;for(let r=i+1;r<4;r++)if(Math.abs(m[r][i])>Math.abs(m[p][i]))p=r;
      [m[i],m[p]]=[m[p],m[i]];[inv[i],inv[p]]=[inv[p],inv[i]];
      const d=m[i][i];for(let c=0;c<4;c++){m[i][c]/=d;inv[i][c]/=d;}
      for(let r=0;r<4;r++){if(r===i)continue;const f=m[r][i];
        for(let c=0;c<4;c++){m[r][c]-=f*m[i][c];inv[r][c]-=f*inv[i][c];}}}
    return this.set(inv[0][0],inv[0][1],inv[0][2],inv[0][3],
                    inv[1][0],inv[1][1],inv[1][2],inv[1][3],
                    inv[2][0],inv[2][1],inv[2][2],inv[2][3],
                    inv[3][0],inv[3][1],inv[3][2],inv[3][3]);}
}
export class Box3 {}
export class Sphere {}
"""


def _node_or_skip():
    node = shutil.which("node")
    if not node:
        pytest.skip("node is not installed; skipping the JS/Python equivalence check")
    return node


def _stage(tmp_path: Path) -> Path:
    """Copy the JS modules next to a Three.js stub and rewrite their imports."""
    # Mark the staged files as ES modules: Node versions without module-syntax
    # detection would otherwise load them as CommonJS.
    (tmp_path / "package.json").write_text('{"type": "module"}', encoding="utf-8")
    (tmp_path / "three.js").write_text(THREE_STUB, encoding="utf-8")
    for name in ("kinematics.js", "math3.js"):
        src = (WEB / name).read_text(encoding="utf-8")
        src = src.replace('from "three"', 'from "./three.js"')
        (tmp_path / name).write_text(src, encoding="utf-8")
    return tmp_path


def _run_node(node: str, workdir: Path, script: str, payload: dict) -> object:
    (workdir / "payload.json").write_text(json.dumps(payload), encoding="utf-8")
    (workdir / "run.mjs").write_text(script, encoding="utf-8")
    proc = subprocess.run([node, "run.mjs"], cwd=workdir, capture_output=True, text=True, timeout=60)
    if proc.returncode != 0:
        raise AssertionError(f"node failed:\n{proc.stdout}\n{proc.stderr}")
    return json.loads(proc.stdout)


FK_SCRIPT = """
import fs from "fs";
import { RobotModel } from "./kinematics.js";
const data = JSON.parse(fs.readFileSync("payload.json", "utf8"));
const model = new RobotModel(data.model);
const out = {};
for (const [link, m] of model.linkWorldTransforms(data.q)) {
  const rows = [];
  for (let r = 0; r < 4; r++) {
    const row = [];
    for (let c = 0; c < 4; c++) row.push(m.elements[c * 4 + r]);
    rows.push(row);
  }
  out[link] = rows;
}
console.log(JSON.stringify(out));
"""

PLAN_SCRIPT = """
import fs from "fs";
import { PlanEvaluator } from "./kinematics.js";
const data = JSON.parse(fs.readFileSync("payload.json", "utf8"));
const ev = new PlanEvaluator(data.plan);
console.log(JSON.stringify(data.times.map((t) => ev.sample(t))));
"""


def test_browser_fk_matches_python(tmp_path, simple_arm):
    node = _node_or_skip()
    q = dict(zip(simple_arm.actuated, (0.7, -1.1, 2.4, 0.3, -0.5, 1.9)))
    model = build_model(simple_arm, with_drives=False)
    qv = np.zeros(model.nq)
    for n, v in q.items():
        qv[model.joints[model.getJointId(n)].idx_q] = v
    _, expected = frame_placements(model, qv)
    got = _run_node(node, _stage(tmp_path), FK_SCRIPT, {"model": simple_arm.to_dict(), "q": q})
    assert set(got) == set(expected)
    for link, T in expected.items():
        assert np.allclose(np.array(got[link]), T, atol=1e-12), link


def test_browser_plan_evaluation_matches_python(tmp_path, simple_arm):
    node = _node_or_skip()
    plan = load_trajectory(
        simple_arm,
        {
            "format": "waypoints",
            "waypoints": [
                {"joints": [0, 0, 0, 0, 0, 0]},
                {"joints": [1.2, -0.8, 0.5, 0, 0, 0], "stop": False},
                {"joints": [-0.6, 1.0, -1.1, 0.5, 0.2, -1.0]},
            ],
        },
    )
    times = [plan.duration * i / 60 for i in range(61)]
    got = _run_node(node, _stage(tmp_path), PLAN_SCRIPT, {"plan": plan.to_dict(), "times": times})
    for t, sample in zip(times, got):
        q, qd, qdd = plan.sample(t)
        for n in simple_arm.actuated:
            assert q[n] == pytest.approx(sample["q"][n], abs=1e-12)
            assert qd[n] == pytest.approx(sample["qd"][n], abs=1e-12)
            assert qdd[n] == pytest.approx(sample["qdd"][n], abs=1e-12)
            assert math.isfinite(sample["q"][n])


def test_browser_fk_follows_a_mimic_linkage(tmp_path):
    """The palletizer's tool plate must come out level in the browser too."""
    from arm_analyzer.robot import load_arm

    from conftest import EXAMPLES

    node = _node_or_skip()
    arm = load_arm(EXAMPLES / "robots" / "palletizer_4dof.urdf")
    assert arm.passive  # the linkage the browser has to reproduce
    q = dict(zip(arm.actuated, (0.5, 0.6, -0.3, 1.0)))
    model = build_model(arm, with_drives=False)
    qv = np.zeros(model.nq)
    for n, v in q.items():
        qv[model.joints[model.getJointId(n)].idx_q] = v
    _, expected = frame_placements(model, qv)
    got = _run_node(node, _stage(tmp_path), FK_SCRIPT, {"model": arm.to_dict(), "q": q})
    for link, T in expected.items():
        assert np.allclose(np.array(got[link]), T, atol=1e-12), link
    # And the plate really is level, so the check above means something.
    assert np.allclose(np.array(got["link5"])[:3, 2], [0, 0, 1], atol=1e-9)
