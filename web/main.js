// Arm Analyzer — bootstrap and app wiring.
//
//   api.js         HTTP client
//   urdf_doc.js    the robot description as an editable XML document
//   kinematics.js  client-side FK + piecewise-cubic plan evaluation
//   viewport.js    ArmScene: the Three.js view (used for editor and motion)
//   inspector.js   property panel for the selected link / motor / gearbox / transmission
//   jog.js         joint jog pop-over in the motion view
//   traj_editor.js cartesian trajectory editor (pop-out)
//   plots.js       speed-torque grid, torque timeline, summary table
//   format.js      number formatting + series palette
//
// Data flow: the URDF lives here as a UrdfDoc. Every edit re-serializes it and
// asks /api/robot/model to parse it (debounced); a good parse rebuilds both
// views and re-runs /api/analyze, a bad one is reported and the last good
// model stays on screen. Playback only evaluates the returned plan locally.

import { api } from "./api.js";
import { PlanEvaluator, RobotModel } from "./kinematics.js";
import { ArmScene } from "./viewport.js";
import { UrdfDoc } from "./urdf_doc.js";
import { Inspector } from "./inspector.js";
import { JogPanel } from "./jog.js";
import { TrajectoryEditor } from "./traj_editor.js";
import * as THREE from "three";
import { SpeedTorquePlots, Timeline, renderSummary } from "./plots.js";
import { fmt } from "./format.js";

const $ = (sel) => document.querySelector(sel);
const $$ = (sel) => Array.from(document.querySelectorAll(sel));

const PATH_SAMPLES = 240;
const IMPORTED = "__imported__";
const EDITED = "__edited__";
const DEFAULT_CARTESIAN = "default_pick_place.json";
const MODEL_DEBOUNCE_MS = 150;
const ANALYZE_DEBOUNCE_MS = 350;
const HISTORY_LIMIT = 100;

const state = {
  // Robot description
  doc: null, // UrdfDoc being edited
  label: "", // file name it came from
  pristine: "", // serialized doc as loaded (for "modified" / revert)
  committed: "", // serialized doc at the last committed edit
  history: [],
  future: [],
  urdf: null, // last text the server parsed successfully
  model: null,
  modelSeq: 0,
  // Trajectory + analysis
  traj: null, // {text, kind, label}
  importedTraj: null,
  editedTraj: null,
  result: null,
  plan: null,
  analyzeSeq: 0,
  // Playback
  t: 0,
  playing: false,
  speed: 1,
  loop: true,
  q: {},
  // Selection: {kind: "link"|"motor"|"gearbox"|"transmission", id} or null
  selection: null,
};

// ---------------------------------------------------------------------------
// Status
// ---------------------------------------------------------------------------

let statusTimer = null;
function setStatus(msg, kind = "") {
  const el = $("#status-msg");
  el.textContent = msg;
  el.title = msg;
  el.className = `status-msg ${kind}`;
  clearTimeout(statusTimer);
  if (kind === "good") statusTimer = setTimeout(() => (el.textContent = ""), 4000);
}

// ---------------------------------------------------------------------------
// Views
// ---------------------------------------------------------------------------

const editorView = new ArmScene($("#editor-view .view-host"), {
  annotate: true,
  onPick: (sel) => select(sel),
});
const motionView = new ArmScene($("#motion-view .view-host"), {
  onPick: (sel) => select(sel),
});
const inspector = new Inspector($("#inspector"), {
  getDoc: () => state.doc,
  getModel: () => state.model,
  onEdit: (commit) => onEdit(commit),
  onSelect: (sel) => select(sel),
});
const jog = new JogPanel($("#jog-panel"), { onJog: (name, v) => jogJoint(name, v) });
const plots = new SpeedTorquePlots({ onSelect: (joint) => select({ kind: "transmission", id: joint }) });
const trajEditor = new TrajectoryEditor({
  getUrdf: () => state.urdf,
  getModel: () => state.model,
  getToolPose: () => currentToolPose(),
  onSave: (spec, label) => saveEditedTrajectory(spec, label),
});
const timeline = new Timeline({
  onSeek: (t) => {
    togglePlay(false);
    setTime(t);
  },
});

// ---------------------------------------------------------------------------
// Robot loading and editing
// ---------------------------------------------------------------------------

async function loadRobot(text, label) {
  let doc;
  try {
    doc = new UrdfDoc(text);
  } catch (e) {
    setStatus(`Robot: ${e.message}`, "bad");
    return;
  }
  state.doc = doc;
  state.label = label;
  state.pristine = state.committed = doc.text();
  state.history = [];
  state.future = [];
  state.selection = null;
  state.q = {};
  setStatus(`Loading ${label}…`, "busy");
  await refreshModel({ fit: true });
}

let modelTimer = null;
function scheduleModel() {
  clearTimeout(modelTimer);
  modelTimer = setTimeout(() => refreshModel(), MODEL_DEBOUNCE_MS);
}

async function refreshModel({ fit = false } = {}) {
  clearTimeout(modelTimer);
  const text = state.doc.text();
  const seq = ++state.modelSeq;
  let model;
  try {
    model = new RobotModel(await api.model(text));
  } catch (e) {
    if (seq !== state.modelSeq) return;
    setStatus(`URDF: ${e.message}`, "bad");
    $("#inspector").classList.add("invalid");
    return;
  }
  if (seq !== state.modelSeq) return;
  $("#inspector").classList.remove("invalid");
  const firstLoad = !state.model || fit;
  state.model = model;
  state.urdf = text;

  // Keep the current pose for joints that still exist; default the rest.
  state.q = { ...model.defaultPose(), ...pick(state.q, model.actuated) };
  editorView.setModel(model);
  motionView.setModel(model);
  motionView.pose(state.q);
  jog.setModel(model, state.q);
  if (fit) {
    editorView.zoomToFit();
    motionView.zoomToFit();
  }

  if (state.selection && !selectionExists(state.selection)) state.selection = null;
  applySelection({ rebuildInspector: firstLoad });
  inspector.refreshDerived();
  updateDocChrome();
  if (firstLoad) setStatus(`Loaded ${model.name}: ${model.actuated.length} joints`, "good");
  analyzeSoon();
}

function pick(obj, keys) {
  const out = {};
  for (const k of keys) if (k in obj) out[k] = obj[k];
  return out;
}

/** Called by the inspector for every field change. */
function onEdit(commit) {
  if (commit) commitHistory();
  updateDocChrome();
  scheduleModel();
}

function commitHistory() {
  const now = state.doc.text();
  if (now === state.committed) return;
  state.history.push(state.committed);
  if (state.history.length > HISTORY_LIMIT) state.history.shift();
  state.future = [];
  state.committed = now;
}

function restore(text) {
  state.doc = new UrdfDoc(text);
  state.committed = text;
  updateDocChrome();
  refreshModel().then(() => inspector.show(state.selection));
}

function undo() {
  commitHistory(); // fold in any uncommitted live edit first
  if (!state.history.length) return;
  state.future.push(state.committed);
  restore(state.history.pop());
}

function redo() {
  if (!state.future.length) return;
  state.history.push(state.committed);
  restore(state.future.pop());
}

function revert() {
  commitHistory();
  if (state.committed === state.pristine) return;
  state.history.push(state.committed);
  state.future = [];
  restore(state.pristine);
}

function updateDocChrome() {
  const text = state.doc ? state.doc.text() : "";
  $("#brand-doc").textContent = state.model ? `${state.model.name} · ${state.label}` : "no robot";
  $("#brand-dirty").hidden = !state.doc || text === state.pristine;
  $("#btn-undo").disabled = !state.history.length && text === state.committed;
  $("#btn-redo").disabled = !state.future.length;
  $("#btn-revert").disabled = !state.doc || text === state.pristine;
}

function downloadUrdf() {
  if (!state.doc) return;
  const text = `<?xml version="1.0"?>\n${state.doc.text().replace(/^<\?xml[^>]*>\s*/, "")}\n`;
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([text], { type: "application/xml" }));
  a.download = state.label?.endsWith(".urdf") ? state.label : `${state.model?.name || "robot"}.urdf`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

// ---------------------------------------------------------------------------
// Selection
// ---------------------------------------------------------------------------

function selectionExists(sel) {
  const m = state.model;
  if (sel.kind === "link") return m.linkByName.has(sel.id);
  if (sel.kind === "transmission") return m.jointByName.has(sel.id);
  return m.driveByJoint.has(sel.id);
}

function jointOf(sel) {
  if (!sel || !state.model) return null;
  if (sel.kind === "link") return state.model.jointByChild.get(sel.id)?.name || null;
  return sel.id;
}

function select(sel) {
  if (sel && state.selection && sel.kind === state.selection.kind && sel.id === state.selection.id) return;
  if (sel && state.model && !selectionExists(sel)) return;
  commitHistory();
  state.selection = sel;
  applySelection({ rebuildInspector: true });
}

function applySelection({ rebuildInspector }) {
  const sel = state.selection;
  const joint = jointOf(sel);
  editorView.setSelection(sel, joint);
  motionView.setSelection(sel, joint);
  plots.setSelected(joint);
  timeline.setSelected(joint);
  $$("#summary tbody tr").forEach((el) => el.classList.toggle("selected", el.dataset.joint === joint));
  if (rebuildInspector) inspector.show(sel);
  requestDraw();
}

// ---------------------------------------------------------------------------
// Analysis
// ---------------------------------------------------------------------------

function clearResult() {
  state.result = null;
  state.plan = null;
  state.t = 0;
  state.playing = false;
  plots.setResult(null);
  timeline.setResult(null);
  renderSummary(null, {});
  motionView.setToolPath(null);
  updateTransport();
  requestDraw();
}

/** Run the analysis; resolves to null on success or the error message. */
async function analyze() {
  clearTimeout(analyzeTimer);
  if (!state.urdf || !state.traj) return null;
  const seq = ++state.analyzeSeq;
  setStatus(`Analysing ${state.traj.label}…`, "busy");
  let result;
  try {
    result = await api.analyze({
      urdf: state.urdf,
      trajectory: state.traj.text,
      trajectory_kind: state.traj.kind,
      units: $("#opt-units").value,
      smoothing: $("#opt-smoothing").value,
      rate_hz: Number($("#opt-rate").value) || 200,
      payload: { mass: Math.max(0, Number($("#opt-payload").value) || 0) },
    });
  } catch (e) {
    if (seq !== state.analyzeSeq) return null;
    clearResult();
    setStatus(`Analysis: ${e.message}`, "bad");
    return e.message;
  }
  if (seq !== state.analyzeSeq) return null; // superseded

  state.result = result;
  state.plan = new PlanEvaluator(result.plan);
  state.t = Math.min(state.t, result.duration);
  plots.setResult(result);
  timeline.setResult(result);
  renderSummary(result, {
    selected: jointOf(state.selection),
    onSelect: (joint) => select({ kind: "transmission", id: joint }),
  });
  applySelection({ rebuildInspector: false });
  buildToolPath();
  updateTransport();
  setTime(state.t);
  $("#motion-hint").textContent = describeTrajectory(result);
  $("#motion-hint").title = $("#motion-hint").textContent;

  const over = result.summary.filter((s) => s.status === "over").map((s) => s.name);
  setStatus(
    over.length ? `Out of spec: ${over.join(", ")}` : "All drives within spec",
    over.length ? "bad" : "good"
  );
  return null;
}

let analyzeTimer = null;
function analyzeSoon() {
  clearTimeout(analyzeTimer);
  analyzeTimer = setTimeout(analyze, ANALYZE_DEBOUNCE_MS);
}

function buildToolPath() {
  if (!state.plan || state.plan.isEmpty) return motionView.setToolPath(null);
  const pts = [];
  for (let i = 0; i <= PATH_SAMPLES; i++) {
    const tip = motionView.toolTipWorld(state.plan.positionsAt((state.plan.duration * i) / PATH_SAMPLES))[0];
    if (tip) pts.push(tip);
  }
  // Label only the working spots (waypoints with a dwell) when there are any;
  // via poses (approach, stage, swing...) would bury them.
  const wps = state.result?.plan?.meta?.waypoints || [];
  const spots = wps.filter((w) => w.dwell > 0);
  motionView.setToolPath(pts, spots.length ? spots : wps);
}

function describeTrajectory(result) {
  const meta = result.plan.meta || {};
  const base = `${state.traj.label} · ${fmt(result.duration, 2)} s · moving mass ${fmt(result.moving_mass, 2)} kg`;
  if (meta.kind !== "cartesian") return base;
  const ik = meta.ik;
  return `${base} · cartesian → IK on "${ik.tool}", ${ik.samples} poses, max error ${fmt(ik.max_position_error * 1e6, 1)} µm`;
}

// ---------------------------------------------------------------------------
// Trajectories
// ---------------------------------------------------------------------------

function kindFor(name) {
  return /\.csv$|\.txt$/i.test(name) ? "csv" : "json";
}

async function selectTrajectory(value) {
  if (value === IMPORTED) {
    state.traj = state.importedTraj;
  } else if (value === EDITED) {
    state.traj = state.editedTraj;
  } else if (value) {
    try {
      const f = await api.trajectoryFile(value);
      state.traj = { text: f.text, kind: kindFor(f.file), label: f.file };
    } catch (e) {
      setStatus(`Trajectory: ${e.message}`, "bad");
      return;
    }
  } else {
    state.traj = null;
  }
  state.t = 0;
  state.playing = false;
  if (state.traj) await analyze();
  else clearResult();
}

// ---------------------------------------------------------------------------
// Trajectory editor
// ---------------------------------------------------------------------------

function parseCartesian(traj) {
  if (!traj || traj.kind !== "json") return null;
  try {
    const spec = JSON.parse(traj.text);
    return spec?.format === "cartesian" ? spec : null;
  } catch {
    return null;
  }
}

async function editTrajectory() {
  if (!state.model) return setStatus("Load a robot first", "bad");
  let spec = parseCartesian(state.traj);
  let label = state.traj?.label || DEFAULT_CARTESIAN;
  if (!spec) {
    const what = state.traj ? `"${state.traj.label}" is a joint-space trajectory and can't be edited here.` : "No trajectory is loaded.";
    if (!confirm(`${what}\n\nStart a new cartesian trajectory from the default pick and place?`)) return;
    try {
      const f = await api.trajectoryFile(DEFAULT_CARTESIAN);
      spec = JSON.parse(f.text);
      label = f.file;
    } catch (e) {
      return setStatus(`Trajectory: ${e.message}`, "bad");
    }
  }
  if (state.playing) togglePlay(false);
  trajEditor.open(spec, spec.label || label);
}

/** Editor "Save": solve IK along the whole path via the analysis; keep the
 * previous trajectory if that fails. Resolves to null or an error message. */
async function saveEditedTrajectory(spec, label) {
  const previous = state.traj;
  state.editedTraj = { text: `${JSON.stringify(spec, null, 2)}\n`, kind: "json", label: `${label} (edited)` };
  state.traj = state.editedTraj;
  state.t = 0;
  const error = await analyze();
  if (error) {
    state.traj = previous;
    if (previous) await analyze();
    return error;
  }
  const sel = $("#traj-select");
  sel.querySelector(`option[value="${EDITED}"]`)?.remove();
  sel.add(new Option(state.editedTraj.label, EDITED), 0);
  sel.value = EDITED;
  setStatus(`Saved ${label}: IK solved along the whole path`, "good");
  return null;
}

/** Tool pose shown in the motion view (trajectory time or jog), for the editor. */
function currentToolPose() {
  const tip = motionView.toolTipPoses(state.q)[0];
  if (!tip) return null;
  const p = new THREE.Vector3();
  const qn = new THREE.Quaternion();
  tip.matrix.decompose(p, qn, new THREE.Vector3());
  const e = new THREE.Euler().setFromQuaternion(qn, "ZYX");
  return { xyz: p.toArray(), rpy: [e.x, e.y, e.z].map((v) => (v * 180) / Math.PI) };
}

// ---------------------------------------------------------------------------
// Playback
// ---------------------------------------------------------------------------

function sampleIndex(t) {
  const r = state.result;
  if (!r || r.t.length < 2) return 0;
  return Math.max(0, Math.min(r.t.length - 1, Math.round((t / r.duration) * (r.t.length - 1))));
}

function setTime(t) {
  state.t = t;
  if (state.plan && !state.plan.isEmpty) {
    state.q = { ...state.q, ...state.plan.positionsAt(t) };
    motionView.pose(state.q);
    jog.sync(state.q);
  }
  requestDraw();
}

function jogJoint(name, value) {
  if (state.playing) togglePlay(false);
  state.q = { ...state.q, [name]: value };
  motionView.pose(state.q);
}

function updateTransport() {
  const has = !!state.result;
  $("#btn-play").disabled = !has;
  $("#btn-rewind").disabled = !has;
  $("#scrub").disabled = !has;
  $("#btn-play").textContent = state.playing ? "⏸" : "▶";
  if (has) $("#scrub").max = String(state.result.duration);
}

function togglePlay(force) {
  if (!state.result) return;
  const next = force ?? !state.playing;
  if (next && state.t >= state.result.duration - 1e-9) setTime(0);
  state.playing = next;
  updateTransport();
}

let drawPending = true;
function requestDraw() {
  drawPending = true;
}

let lastFrame = performance.now();
function frame(now) {
  requestAnimationFrame(frame);
  const dt = Math.min(0.1, (now - lastFrame) / 1000);
  lastFrame = now;
  if (state.playing && state.result) {
    let t = state.t + dt * state.speed;
    if (t >= state.result.duration) {
      if (state.loop) t %= Math.max(state.result.duration, 1e-6);
      else {
        t = state.result.duration;
        state.playing = false;
        updateTransport();
      }
    }
    setTime(t);
  }
  if (!drawPending) return;
  drawPending = false;
  const has = !!state.result;
  plots.draw(has ? sampleIndex(state.t) : null);
  timeline.draw(has ? state.t : null);
  $("#scrub").value = String(state.t);
  $("#time-readout").textContent = has ? `${fmt(state.t, 2)} / ${fmt(state.result.duration, 2)} s` : "— / — s";
}

// ---------------------------------------------------------------------------
// UI wiring
// ---------------------------------------------------------------------------

function segmented(id, onChange) {
  const host = $(id);
  host.querySelectorAll("button").forEach((b) => {
    b.addEventListener("click", () => {
      host.querySelectorAll("button").forEach((x) => x.setAttribute("aria-pressed", String(x === b)));
      onChange(b.dataset.value);
    });
  });
}

async function readFile(input) {
  const file = input.files?.[0];
  input.value = "";
  if (!file) return null;
  return { name: file.name, text: await file.text() };
}

function wireView(container, view) {
  container.querySelectorAll("[data-view]").forEach((b) => b.addEventListener("click", () => view.viewPreset(b.dataset.view)));
  container.querySelector("[data-fit]")?.addEventListener("click", () => view.zoomToFit());
  container.querySelectorAll("[data-toggle]").forEach((b) => {
    view.setToggle(b.dataset.toggle, b.dataset.on === "1");
    b.addEventListener("click", () => {
      const on = b.dataset.on !== "1";
      b.dataset.on = on ? "1" : "0";
      view.setToggle(b.dataset.toggle, on);
    });
  });
}

/**
 * Optional view state from the URL, e.g. `?robot=simple_6dof.urdf&traj=sine_sweep.csv
 * &side=motor&timeline=torque&zoom=1&select=motor:j2&t=1.5&view=right`, so a
 * view can be bookmarked. `joint=j2` is shorthand for `select=transmission:j2`.
 */
function applyViewParams(params) {
  const click = (sel) => document.querySelector(sel)?.click();
  if (params.get("side")) click(`#plot-side [data-value="${CSS.escape(params.get("side"))}"]`);
  if (params.get("timeline")) click(`#timeline-mode [data-value="${CSS.escape(params.get("timeline"))}"]`);
  for (const [key, id] of [["zoom", "#opt-zoom"], ["fold", "#opt-fold"]]) {
    if (!params.has(key)) continue;
    const el = $(id);
    el.checked = params.get(key) !== "0";
    el.dispatchEvent(new Event("change"));
  }
  if (params.get("view")) {
    for (const v of [editorView, motionView]) {
      v.viewPreset(params.get("view"));
      v.zoomToFit();
    }
  }
  const sel = params.get("select") || (params.get("joint") ? `transmission:${params.get("joint")}` : null);
  if (sel && sel.includes(":")) {
    const cut = sel.indexOf(":");
    select({ kind: sel.slice(0, cut), id: sel.slice(cut + 1) });
  }
  if (params.get("jog") === "1") click("#btn-jog");
  if (params.get("edit") === "1") editTrajectory();
  if (params.get("t") && state.result) setTime(Math.min(Number(params.get("t")) || 0, state.result.duration));
}

function wireUi() {
  wireView($("#editor-view"), editorView);
  wireView($("#motion-view"), motionView);

  $("#btn-jog").addEventListener("click", (e) => {
    const on = e.currentTarget.dataset.on !== "1";
    e.currentTarget.dataset.on = on ? "1" : "0";
    $("#jog-panel").hidden = !on;
    jog.sync(state.q);
  });

  $("#robot-select").addEventListener("change", async (e) => {
    const f = await api.robotFile(e.target.value).catch((err) => setStatus(err.message, "bad"));
    if (f) await loadRobot(f.text, f.file);
  });
  $("#btn-open-urdf").addEventListener("click", () => $("#file-urdf").click());
  $("#file-urdf").addEventListener("change", async (e) => {
    const f = await readFile(e.target);
    if (!f) return;
    const sel = $("#robot-select");
    sel.querySelector(`option[value="${IMPORTED}"]`)?.remove();
    sel.add(new Option(`${f.name} (opened)`, IMPORTED), 0);
    sel.value = IMPORTED;
    await loadRobot(f.text, f.name);
  });
  $("#btn-download-urdf").addEventListener("click", downloadUrdf);
  $("#btn-undo").addEventListener("click", undo);
  $("#btn-redo").addEventListener("click", redo);
  $("#btn-revert").addEventListener("click", revert);

  $("#traj-select").addEventListener("change", (e) => selectTrajectory(e.target.value));
  $("#btn-edit-traj").addEventListener("click", editTrajectory);
  $("#btn-import-traj").addEventListener("click", () => $("#file-traj").click());
  $("#file-traj").addEventListener("change", async (e) => {
    const f = await readFile(e.target);
    if (!f) return;
    state.importedTraj = { text: f.text, kind: kindFor(f.name), label: f.name };
    const sel = $("#traj-select");
    sel.querySelector(`option[value="${IMPORTED}"]`)?.remove();
    sel.add(new Option(`${f.name} (imported)`, IMPORTED), 0);
    sel.value = IMPORTED;
    await selectTrajectory(IMPORTED);
  });

  $("#btn-analyze").addEventListener("click", analyze);
  for (const id of ["#opt-payload", "#opt-rate"]) $(id).addEventListener("input", analyzeSoon);
  for (const id of ["#opt-units", "#opt-smoothing"]) $(id).addEventListener("change", analyze);

  segmented("#plot-side", (side) => {
    plots.setOptions({ side });
    timeline.setOptions({ side });
    requestDraw();
  });
  $("#opt-fold").addEventListener("change", (e) => {
    plots.setOptions({ fold: e.target.checked });
    requestDraw();
  });
  $("#opt-zoom").addEventListener("change", (e) => {
    plots.setOptions({ zoom: e.target.checked });
    requestDraw();
  });
  segmented("#timeline-mode", (mode) => {
    timeline.setOptions({ mode });
    requestDraw();
  });

  $("#btn-play").addEventListener("click", () => togglePlay());
  $("#btn-rewind").addEventListener("click", () => setTime(0));
  $("#scrub").addEventListener("input", (e) => {
    togglePlay(false);
    setTime(Number(e.target.value));
  });
  $("#play-speed").addEventListener("change", (e) => (state.speed = Number(e.target.value)));
  $("#btn-loop").addEventListener("click", (e) => {
    state.loop = !state.loop;
    e.currentTarget.dataset.on = state.loop ? "1" : "0";
  });

  window.addEventListener("keydown", (e) => {
    if (trajEditor.isOpen) return; // the editor has its own shortcuts
    if (e.target.closest("input, select, textarea")) return;
    const mod = e.ctrlKey || e.metaKey;
    if (mod && e.key.toLowerCase() === "z") {
      e.preventDefault();
      if (e.shiftKey) redo();
      else undo();
    } else if (mod && e.key.toLowerCase() === "y") {
      e.preventDefault();
      redo();
    } else if (e.code === "Space") {
      e.preventDefault();
      togglePlay();
    } else if (e.key === "Home") {
      setTime(0);
    } else if (e.key === "Escape") {
      select(null);
    }
  });
  window.addEventListener("beforeunload", (e) => {
    if (state.doc && state.doc.text() !== state.pristine) e.preventDefault();
  });

  new ResizeObserver(requestDraw).observe($("#plots-col"));
  new ResizeObserver(requestDraw).observe($("#timeline-canvas"));
}

async function boot() {
  wireUi();
  inspector.show(null);
  requestAnimationFrame(frame);

  const [robots, trajs] = await Promise.all([
    api.robots().catch(() => []),
    api.trajectories().catch(() => []),
  ]);
  const rsel = $("#robot-select");
  for (const r of robots) rsel.add(new Option(r.label, r.file));
  const tsel = $("#traj-select");
  tsel.add(new Option("— none —", ""));
  for (const t of trajs) tsel.add(new Option(t.label, t.file));

  const params = new URLSearchParams(location.search);
  const choose = (list, wanted) => list.find((x) => x.file === wanted) || list[0];
  const traj = choose(trajs, params.get("traj"));
  if (traj) {
    tsel.value = traj.file;
    const f = await api.trajectoryFile(traj.file).catch(() => null);
    if (f) state.traj = { text: f.text, kind: kindFor(f.file), label: f.file };
  }
  const robot = choose(robots, params.get("robot"));
  if (robot) {
    rsel.value = robot.file;
    const f = await api.robotFile(robot.file).catch((e) => setStatus(e.message, "bad"));
    if (f) await loadRobot(f.text, f.file);
    await analyze();
  } else {
    setStatus("No robots in examples/robots — open a URDF", "bad");
  }
  applyViewParams(params);
}

boot();
