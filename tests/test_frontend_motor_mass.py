"""Keep the browser's motor-mass model identical to the Python one.

``web/motor_mass.js`` carries its own copy of the fitted constants so the
inspector can show an estimate while a rating is being edited. Two copies of a
fit drift, and a browser that disagreed with the analysis would show one mass
while the torque curves used another -- so this runs the real JS under Node and
checks it against ``arm_analyzer.motor_mass``. Skips without Node.
"""

import json
import shutil
import subprocess
from pathlib import Path

import pytest

from arm_analyzer import motor_mass

WEB = Path(__file__).resolve().parents[1] / "web"

CASES = [
    [1.0, "outrunner"],
    [1.0, "frameless"],
    [1.0, "industrial"],
    [1.0, "integrated"],
    [0.0102, "inrunner"],   # smallest real part in the set
    [98.0, "hub"],          # top of the range
    [0.01, "frameless"],    # below this form's range
    [500.0, "frameless"],   # above it
    [5.0, "hub"],           # below the hub range
    [2.0, None],            # untyped -> frameless
]

SCRIPT = """
import fs from "fs";
import {
  A, B_TORQUE, DEFAULT_FORM, DENSITY, LIGHTEST_KG, LOO_MAE_KG, LOO_MEDIAN_REL,
  OVERALL_DENSITY, TORQUE_RANGE,
  estimateMotorMass, motorBulkDensity, motorMassOutOfRange, normalizeForm,
} from "./motor_mass.js";
const cases = JSON.parse(fs.readFileSync("payload.json", "utf8"));
console.log(JSON.stringify({
  constants: {
    A, B_TORQUE, DEFAULT_FORM, DENSITY, LIGHTEST_KG, LOO_MAE_KG,
    LOO_MEDIAN_REL, OVERALL_DENSITY, TORQUE_RANGE,
  },
  results: cases.map(([t, f]) => ({
    mass: estimateMotorMass(t, f),
    note: motorMassOutOfRange(t, f),
    density: motorBulkDensity(f),
  })),
  unknown_form: normalizeForm("pancake"),
  case_insensitive: normalizeForm(" INDUSTRIAL "),
  no_rating: estimateMotorMass(0, "frameless"),
}));
"""


@pytest.fixture(scope="module")
def js(tmp_path_factory):
    node = shutil.which("node")
    if node is None:
        pytest.skip("node is not installed")
    work = tmp_path_factory.mktemp("motor_mass_js")
    (work / "package.json").write_text('{"type": "module"}', encoding="utf-8")
    (work / "motor_mass.js").write_text(
        (WEB / "motor_mass.js").read_text(encoding="utf-8"), encoding="utf-8"
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
    assert c["B_TORQUE"] == motor_mass.B_TORQUE
    assert c["A"] == motor_mass.A
    assert c["DEFAULT_FORM"] == motor_mass.DEFAULT_FORM
    assert c["LOO_MAE_KG"] == motor_mass.LOO_MAE_KG
    assert c["LOO_MEDIAN_REL"] == motor_mass.LOO_MEDIAN_REL
    assert c["LIGHTEST_KG"] == motor_mass.LIGHTEST_KG
    assert c["OVERALL_DENSITY"] == motor_mass.OVERALL_DENSITY
    assert c["DENSITY"] == motor_mass.DENSITY
    assert {k: tuple(v) for k, v in c["TORQUE_RANGE"].items()} == motor_mass.TORQUE_RANGE
    assert "C_SPEED" not in c, "the speed term should be gone from both copies"


def test_browser_estimates_match_python(js):
    for (torque, form), got in zip(CASES, js["results"]):
        assert got["mass"] == pytest.approx(
            motor_mass.estimate_mass(torque, form), rel=1e-12
        ), f"tau={torque}, form={form}"


def test_browser_agrees_on_what_is_extrapolation(js):
    for (torque, form), got in zip(CASES, js["results"]):
        expected = motor_mass.out_of_range(torque, form)
        assert (got["note"] is None) == (expected is None), (torque, form)


def test_browser_agrees_on_density(js):
    for (torque, form), got in zip(CASES, js["results"]):
        assert got["density"] == pytest.approx(motor_mass.bulk_density(form))


def test_browser_handles_forms_the_same_way(js):
    """Python raises on an unknown form; the browser has nothing to show and
    must return null rather than render NaN into the inspector."""
    assert js["unknown_form"] is None
    with pytest.raises(ValueError):
        motor_mass.normalize_form("pancake")
    assert js["case_insensitive"] == motor_mass.normalize_form(" INDUSTRIAL ")
    assert js["no_rating"] is None
