// Matrix helpers shared by the kinematics and the viewport.
//
// The server sends 4x4 transforms as row-major nested lists. THREE.Matrix4.set
// takes row-major arguments but stores column-major, so this is the one
// place that bridges the two.

import * as THREE from "three";

export function matFromRowMajor(m) {
  return new THREE.Matrix4().set(
    m[0][0], m[0][1], m[0][2], m[0][3],
    m[1][0], m[1][1], m[1][2], m[1][3],
    m[2][0], m[2][1], m[2][2], m[2][3],
    m[3][0], m[3][1], m[3][2], m[3][3]
  );
}

export function unionMeshBoundingBox(meshes) {
  const box = new THREE.Box3();
  let any = false;
  for (const m of meshes) {
    if (!m.geometry || !m.visible) continue;
    if (!m.geometry.boundingBox) m.geometry.computeBoundingBox();
    const b = m.geometry.boundingBox.clone();
    m.updateMatrixWorld(true);
    b.applyMatrix4(m.matrixWorld);
    if (any) box.union(b);
    else {
      box.copy(b);
      any = true;
    }
  }
  return any ? box : null;
}
