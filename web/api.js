// API client for the Arm Analyzer backend (FastAPI under /api/*).

async function call(path, body) {
  const init = body === undefined
    ? { method: "GET" }
    : { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) };
  const res = await fetch(path, init);
  if (!res.ok) {
    let msg = res.statusText;
    try {
      const txt = await res.text();
      try {
        msg = JSON.parse(txt).detail || txt;
      } catch {
        msg = txt;
      }
    } catch {
      /* keep statusText */
    }
    throw new Error(msg || `HTTP ${res.status}`);
  }
  return res.json();
}

export const api = {
  robots: () => call("/api/robots"),
  robotFile: (file) => call(`/api/robots/${encodeURIComponent(file)}`),
  trajectories: () => call("/api/trajectories"),
  trajectoryFile: (file) => call(`/api/trajectories/${encodeURIComponent(file)}`),
  // Pose-independent model; the browser poses it (kinematics.js).
  model: (urdf) => call("/api/robot/model", { urdf }),
  // Cartesian editor: path shape and timing; with `urdf`, also a per-waypoint
  // IK check against that robot (no full solve).
  cartesianPreview: (urdf, trajectory) =>
    call("/api/trajectory/cartesian/preview", urdf ? { urdf, trajectory } : { trajectory }),
  // Plan + inverse dynamics + per-drive summaries.
  analyze: (body) => call("/api/analyze", body),
};
