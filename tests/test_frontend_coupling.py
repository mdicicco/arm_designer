"""Cross-check the browser's feasible-region geometry against the analysis.

``web/coupling.js`` draws the region a coupled pair of motors can hold as a
polygon in the plane of the two joint torques. That polygon is the whole point
of the view -- it is what shows a differential's limit to be a diamond rather
than the box of the two joint ratings -- so its corners have to be the real
solution of ``|A^-1[:, j] . tau| <= T_j``, not a lookalike. These tests run the
real JS under Node and check the corners against the same constraint the
Python side reports, and skip when Node is not installed.
"""

import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from arm_analyzer.analysis import analyze
from arm_analyzer.robot import parse_arm

from test_coupling import DIFFERENTIAL, _plan, two_joint_urdf

WEB = Path(__file__).resolve().parents[1] / "web"


def _node_or_skip() -> str:
    node = shutil.which("node")
    if node is None:
        pytest.skip("node is not installed")
    return node


def _stage(tmp_path: Path) -> Path:
    """Copy coupling.js and the modules it imports, as ES modules."""
    (tmp_path / "package.json").write_text('{"type": "module"}', encoding="utf-8")
    for name in ("coupling.js", "plots.js", "format.js"):
        (tmp_path / name).write_text((WEB / name).read_text(encoding="utf-8"), encoding="utf-8")
    return tmp_path


SCRIPT = """
import fs from "fs";
import { regionCorners } from "./coupling.js";
const data = JSON.parse(fs.readFileSync("payload.json", "utf8"));
console.log(JSON.stringify(regionCorners(data.A, data.limits)));
"""


def _corners(node: str, tmp_path: Path, A, limits):
    work = _stage(tmp_path)
    (work / "payload.json").write_text(
        json.dumps({"A": A, "limits": limits}), encoding="utf-8"
    )
    (work / "run.mjs").write_text(SCRIPT, encoding="utf-8")
    proc = subprocess.run(
        [node, "run.mjs"], cwd=work, capture_output=True, text=True, timeout=60
    )
    if proc.returncode != 0:
        raise AssertionError(f"node failed:\n{proc.stdout}\n{proc.stderr}")
    return json.loads(proc.stdout)


@pytest.fixture(scope="module")
def diff_map():
    """The coupling map of the two-joint differential, as the browser gets it."""
    arm = parse_arm(two_joint_urdf(DIFFERENTIAL))
    plan = _plan(arm, [0.3, -0.2], [0.4, 0.1], [1.0, -1.0])
    return analyze(arm, plan, rate_hz=100)["couplings"][0]


def test_browser_region_corners_satisfy_both_motor_limits(tmp_path, diff_map):
    node = _node_or_skip()
    limits = [m["stall_limit"] for m in diff_map["motors"]]
    corners = _corners(node, tmp_path, diff_map["A"], limits)
    assert len(corners) == 4
    A_inv = np.array(diff_map["A_inv"])
    for tau in corners:
        # Every corner is where both constraints are exactly tight.
        assert np.allclose(np.abs(np.array(tau) @ A_inv), limits)


def test_browser_region_is_a_diamond_not_a_box(tmp_path, diff_map):
    """The corners lie on the joint axes, so the region is the box rotated 45
    degrees: a torque inside both joint ratings can still overload a motor."""
    node = _node_or_skip()
    limits = [m["stall_limit"] for m in diff_map["motors"]]
    corners = np.array(_corners(node, tmp_path, diff_map["A"], limits))
    # One coordinate of each corner is zero for an equal-ratio differential.
    assert np.allclose(np.min(np.abs(corners), axis=1), 0.0)
    reach = np.max(np.abs(corners))
    # The box's corner (both joints at full reach) is outside the diamond.
    assert np.max(np.abs(np.array([reach, reach]) @ np.array(diff_map["A_inv"]))) > max(limits)


def test_browser_region_is_omitted_when_a_motor_is_unrated(tmp_path, diff_map):
    node = _node_or_skip()
    assert _corners(node, tmp_path, diff_map["A"], [None, 1.0]) is None
