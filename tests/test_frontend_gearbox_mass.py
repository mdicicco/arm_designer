"""Keep the browser's gearbox-mass model identical to the Python one.

``web/gearbox_mass.js`` carries its own copy of the fitted constants so the
inspector can show an estimate while a rating is being edited. Two copies of a
fit drift, and a browser that disagreed with the analysis would show one mass
while the torque curves used another -- so this runs the real JS under Node and
checks it against ``arm_analyzer.gearbox_mass``. Skips without Node.
"""

import json
import shutil
import subprocess
from pathlib import Path

import pytest

from arm_analyzer import gearbox_mass

WEB = Path(__file__).resolve().parents[1] / "web"

CASES = [
    [40, 50, "harmonic"],
    [40, 50, "planetary"],
    [40, 50, "cycloidal"],
    [40, 50, "worm"],
    [40, 50, "spur"],
    [4, 30, "harmonic"],
    [4, 100, "harmonic"],      # ratio barely matters
    [784, 111, "cycloidal"],   # top of the fitted range
    [0.1, 50, "harmonic"],     # below it
    [40, 200, "worm"],         # ratio outside it
    [40, 50, None],            # untyped -> harmonic
]

SCRIPT = """
import fs from "fs";
import {
  A, B_TORQUE, C_RATIO, BULK_DENSITY, DEFAULT_TYPE, LOO_MAE_KG,
  estimateGearboxMass, gearboxMassOutOfRange, normalizeType,
} from "./gearbox_mass.js";
const cases = JSON.parse(fs.readFileSync("payload.json", "utf8"));
console.log(JSON.stringify({
  constants: { A, B_TORQUE, C_RATIO, BULK_DENSITY, DEFAULT_TYPE, LOO_MAE_KG },
  results: cases.map(([t, r, k]) => ({
    mass: estimateGearboxMass(t, r, k),
    note: gearboxMassOutOfRange(t, r, k),
  })),
  unknown_type: normalizeType("belt"),
  case_insensitive: normalizeType(" WORM "),
  no_rating: estimateGearboxMass(0, 50, "harmonic"),
}));
"""


@pytest.fixture(scope="module")
def js(tmp_path_factory):
    node = shutil.which("node")
    if node is None:
        pytest.skip("node is not installed")
    work = tmp_path_factory.mktemp("gearbox_mass_js")
    (work / "package.json").write_text('{"type": "module"}', encoding="utf-8")
    (work / "gearbox_mass.js").write_text(
        (WEB / "gearbox_mass.js").read_text(encoding="utf-8"), encoding="utf-8"
    )
    (work / "payload.json").write_text(json.dumps(CASES), encoding="utf-8")
    (work / "run.mjs").write_text(SCRIPT, encoding="utf-8")
    proc = subprocess.run(
        [node, "run.mjs"], cwd=work, capture_output=True, text=True, timeout=60
    )
    if proc.returncode != 0:
        raise AssertionError(f"node failed:\n{proc.stdout}\n{proc.stderr}")
    return json.loads(proc.stdout)


def test_browser_constants_match_python(js):
    c = js["constants"]
    assert c["B_TORQUE"] == gearbox_mass.B_TORQUE
    assert c["C_RATIO"] == gearbox_mass.C_RATIO
    assert c["DEFAULT_TYPE"] == gearbox_mass.DEFAULT_TYPE
    assert c["LOO_MAE_KG"] == gearbox_mass.LOO_MAE_KG
    assert c["A"] == gearbox_mass.A
    assert c["BULK_DENSITY"] == gearbox_mass.BULK_DENSITY


def test_browser_estimates_match_python(js):
    for (torque, ratio, kind), got in zip(CASES, js["results"]):
        assert got["mass"] == pytest.approx(
            gearbox_mass.estimate_mass(torque, ratio, kind), rel=1e-12
        ), f"T={torque}, ratio={ratio}, type={kind}"


def test_browser_agrees_on_what_is_extrapolation(js):
    for (torque, ratio, kind), got in zip(CASES, js["results"]):
        expected = gearbox_mass.out_of_range(torque, ratio, kind)
        assert (got["note"] is None) == (expected is None), (torque, ratio, kind)


def test_browser_handles_types_the_same_way(js):
    """Python raises on an unknown type; the browser has nothing to show and
    must return null rather than render NaN into the inspector."""
    assert js["unknown_type"] is None
    with pytest.raises(ValueError):
        gearbox_mass.normalize_type("belt")
    assert js["case_insensitive"] == gearbox_mass.normalize_type(" WORM ")
    assert js["no_rating"] is None
