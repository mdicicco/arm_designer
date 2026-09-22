// The link-mass sliders.
//
// Each link is sized as a tube spanning its own joints, plus a collar around
// every motor or gearbox mounted on it — see `src/arm_analyzer/link_mass.py`,
// which owns the model and the limits mirrored below. This module only turns
// those parameters into controls and reports what came back.
//
// Nothing is computed here: moving a slider re-runs the analysis, because a
// link's mass changes the torque the motion needs and that is the whole point
// of the panel.

import { fmt } from "./format.js";

const $ = (sel) => document.querySelector(sel);

// label, key, min, max, step, and how the value reads (SI in, display out).
const FIELDS = [
  {
    key: "baseline_diameter", label: "Baseline Ø", min: 0.02, max: 0.4, step: 0.005,
    scale: 1000, unit: "mm", digits: 0,
    title: "Outside diameter of the base link's tube. An absolute size, so it needs setting per robot.",
  },
  {
    key: "taper", label: "Distal taper", min: 0.1, max: 1.5, step: 0.05,
    scale: 1, unit: "×", digits: 2,
    title: "Tip diameter ÷ base diameter, applied geometrically down the chain. 1.0 is a constant-diameter arm; below 1 the wrist is skinnier than the shoulder.",
  },
  {
    key: "wall", label: "Wall", min: 0.0005, max: 0.05, step: 0.0005,
    scale: 1000, unit: "mm", digits: 1,
    title: "Tube wall thickness. A wall at least as thick as the radius makes the link solid.",
  },
  {
    key: "density", label: "Density", min: 500, max: 9000, step: 50,
    scale: 1, unit: "kg/m³", digits: 0,
    title: "Structural material. 2700 is aluminium; raise it to stand in for ribs, castings and hardware the tube does not model.",
  },
  {
    key: "actuator_mass", label: "Per actuator", min: 0, max: 5, step: 0.05,
    scale: 1, unit: "kg", digits: 2,
    title: "Collar mass added for each motor or gearbox mounted on the link — the part that does not scale with the actuator.",
  },
  {
    key: "actuator_fraction", label: "× actuator mass", min: 0, max: 2, step: 0.05,
    scale: 1, unit: "×", digits: 2,
    title: "Plus this much of the actuator's own mass, because a bigger actuator needs a bigger boss.",
  },
];

export class LinkMassControls {
  /** `onChange` re-runs the analysis (debounced by the caller). */
  constructor({ onChange }) {
    this.onChange = onChange;
    this.mode = "declared";
    this.values = Object.fromEntries(FIELDS.map((f) => [f.key, null]));
    this.inputs = new Map();
    this._build();
  }

  _build() {
    const host = $("#lm-sliders");
    for (const f of FIELDS) {
      const row = document.createElement("label");
      row.className = "lm-f";
      row.title = f.title;
      row.innerHTML = `<span>${f.label}</span>
        <input type="range" min="${f.min}" max="${f.max}" step="${f.step}" />
        <span class="val"></span>`;
      const input = row.querySelector("input");
      const out = row.querySelector(".val");
      const show = () => {
        out.textContent = `${fmt(Number(input.value) * f.scale, f.digits)} ${f.unit}`;
      };
      input.addEventListener("input", () => {
        this.values[f.key] = Number(input.value);
        show();
        // Only a derived arm cares; don't re-analyse while it is off.
        if (this.mode === "derived") this.onChange();
      });
      host.appendChild(row);
      this.inputs.set(f.key, { input, show });
    }

    const mode = $("#lm-mode");
    mode.addEventListener("change", () => {
      this.mode = mode.value;
      this._refreshSummary();
      this.onChange();
    });
  }

  /** The body for `/api/analyze`. */
  request() {
    return { mode: this.mode, ...this.values };
  }

  /** Adopt the parameters the server echoed, so the sliders start where the
   *  Python defaults are rather than at a guess duplicated in JS. */
  setResult(result) {
    const lm = result?.link_mass;
    if (!lm) {
      $("#lm-note").textContent = "";
      return;
    }
    for (const f of FIELDS) {
      const v = lm.params?.[f.key];
      if (v == null) continue;
      this.values[f.key] = v;
      const { input, show } = this.inputs.get(f.key);
      // Don't fight the user mid-drag.
      if (document.activeElement !== input) {
        input.value = String(v);
        show();
      }
    }
    this._refreshSummary(lm);

    const ratio = lm.ratio;
    const note =
      this.mode === "derived"
        ? `Structure <b>${fmt(lm.derived_total, 2)} kg</b> from geometry`
          + (ratio ? ` — <b>${fmt(ratio, 2)}×</b> the file's ${fmt(lm.declared_total, 2)} kg.` : ".")
          + " Tube plus collars only: no joint housings, covers or cabling, and no stress check."
        : `The file declares <b>${fmt(lm.declared_total, 2)} kg</b> of structure;`
          + ` this geometry would give <b>${fmt(lm.derived_total, 2)} kg</b>`
          + (ratio ? ` (${fmt(ratio, 2)}×).` : ".");
    $("#lm-note").innerHTML = note;
  }

  _refreshSummary(lm) {
    const el = $("#lm-summary");
    if (this.mode !== "derived") {
      el.textContent = "declared";
      return;
    }
    el.textContent = lm ? `derived · ${fmt(lm.derived_total, 2)} kg` : "derived";
  }
}
