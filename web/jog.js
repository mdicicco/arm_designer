// Jog pop-over in the motion view: one slider per actuated joint.

import { SERIES_COLORS, fmt } from "./format.js";

const toDisplay = (j, v) => (j.type === "revolute" ? `${fmt((v * 180) / Math.PI, 1)}°` : `${fmt(v * 1000, 0)} mm`);

export class JogPanel {
  constructor(host, { onJog }) {
    this.host = host;
    this.onJog = onJog;
    this.model = null;
    this.rows = new Map();
  }

  setModel(model, q) {
    this.model = model;
    this.rows.clear();
    this.host.replaceChildren();
    if (!model) return;
    model.actuated.forEach((name, i) => {
      const j = model.jointByName.get(name);
      const input = document.createElement("input");
      Object.assign(input, { type: "range", min: j.lower, max: j.upper, step: (j.upper - j.lower) / 1000 });
      input.value = q[name] ?? 0;
      const value = document.createElement("span");
      value.className = "val";
      value.textContent = toDisplay(j, q[name] ?? 0);
      input.addEventListener("input", () => {
        value.textContent = toDisplay(j, Number(input.value));
        this.onJog(name, Number(input.value));
      });
      const row = document.createElement("label");
      row.className = "jog-row";
      const tag = document.createElement("b");
      tag.textContent = name;
      tag.style.color = SERIES_COLORS[i % SERIES_COLORS.length];
      row.append(tag, input, value);
      this.host.append(row);
      this.rows.set(name, { input, value, joint: j });
    });
  }

  sync(q) {
    if (this.host.hidden) return;
    for (const [name, { input, value, joint }] of this.rows) {
      if (document.activeElement === input) continue;
      const v = q[name] ?? 0;
      input.value = v;
      value.textContent = toDisplay(joint, v);
    }
  }
}
