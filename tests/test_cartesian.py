import json

import numpy as np
import pinocchio as pin
import pytest

from arm_analyzer.cartesian import (
    MIN_JERK_PEAK,
    TOOL_FRAME,
    build_path,
    ik_model,
    plan_from_cartesian,
    sample_path,
)
from arm_analyzer.robot import load_arm, parse_arm
from arm_analyzer.trajectory import load_trajectory

from conftest import EXAMPLES

DEFAULT = EXAMPLES / "trajectories" / "default_pick_place.json"
# Every example arm can follow the default pick-and-place path: its tool points
# straight down throughout, which even the 4-DOF palletizer can do.
ROBOTS = [
    "simple_6dof",
    "simple_6dof_remote_elbow",
    "kr_style_6dof",
    "wam_style_7dof",
    "ur_style_6dof",
    "palletizer_4dof",
]
# The workspace tour also tips the tool out, up and sideways. The palletizer's
# plate is always level and the WAM's wrist stops at +/-90 deg, so those two
# cannot follow it -- that is a property of the machines, not of the tour.
TOUR_ROBOTS = ["simple_6dof", "simple_6dof_remote_elbow", "kr_style_6dof"]


@pytest.fixture(scope="module")
def default_spec():
    return json.loads(DEFAULT.read_text())


def _segment_to(segments, name):
    return next(s for s in segments if s.end.name == name and s.start is not s.end)


def test_linear_segments_are_straight(default_spec):
    _, segments = build_path(default_spec)
    seg = _segment_to(segments, "pick")
    a, b = seg.start.p, seg.end.p
    for t in np.linspace(seg.t0, seg.t1, 9):
        p, _ = seg.pose(t)
        # Distance from the a-b line is zero.
        assert np.linalg.norm(np.cross(p - a, b - a)) < 1e-12


def test_smooth_segments_join_linear_ones_tangentially(default_spec):
    _, segments = build_path(default_spec)
    retract = _segment_to(segments, "retract_pick")
    transfer = _segment_to(segments, "above_place")
    approach = _segment_to(segments, "place")
    up = retract.end.p - retract.start.p
    down = approach.end.p - approach.start.p
    leave = np.asarray(transfer.position.derivate(0.0, 1)).ravel()
    arrive = np.asarray(transfer.position.derivate(1.0, 1)).ravel()
    unit = lambda v: v / np.linalg.norm(v)  # noqa: E731
    assert np.allclose(unit(leave), unit(up))
    assert np.allclose(unit(arrive), unit(down))
    # And it is a genuine curve, not the straight chord.
    mid, _ = transfer.pose(0.5 * (transfer.t0 + transfer.t1))
    chord_mid = 0.5 * (transfer.start.p + transfer.end.p)
    assert mid[2] > chord_mid[2] + 0.01


def test_min_jerk_timing_rests_at_every_waypoint_and_respects_speed(default_spec):
    _, segments = build_path(default_spec)
    limits = default_spec["limits"]
    for seg in segments:
        dt = 1e-5
        for t in (seg.t0, seg.t1):
            p_a, _ = seg.pose(t - dt)
            p_b, _ = seg.pose(t + dt)
            assert np.linalg.norm(p_b - p_a) / (2 * dt) < 1e-3
        if seg.start is seg.end:
            continue
        cap = limits["linear_speed" if seg.end.path == "linear" else "smooth_speed"]
        ts = np.linspace(seg.t0, seg.t1, 401)
        ps = np.array([seg.pose(t)[0] for t in ts])
        speed = np.linalg.norm(np.diff(ps, axis=0), axis=1) / np.diff(ts)
        assert speed.max() <= cap * 1.01
        rot_limited = MIN_JERK_PEAK * np.linalg.norm(pin.log3(seg.start.R.T @ seg.end.R)) / np.radians(limits["angular_speed"])
        if rot_limited < seg.duration * 0.99 and seg.duration > 0.11:
            assert speed.max() == pytest.approx(cap, rel=0.02)  # speed-limited, not padded


def test_dwell_holds_the_pose(default_spec):
    _, segments = build_path(default_spec)
    dwell = next(s for s in segments if s.start is s.end and s.end.name == "pick")
    assert dwell.duration == pytest.approx(0.3)
    p0, _ = dwell.pose(dwell.t0)
    p1, _ = dwell.pose(0.5 * (dwell.t0 + dwell.t1))
    assert np.allclose(p0, p1)


def test_waypoint_orientation_is_reached(default_spec):
    wps, segments = build_path(default_spec)
    times, poses, labels = sample_path(segments, 100.0)
    assert labels[0] == "home"
    assert np.allclose(poses[0].rotation, wps[0].R)
    # Tool z points down at every sample of the default path.
    for pose in poses:
        assert pose.rotation[:, 2] == pytest.approx([0, 0, -1], abs=1e-9)


@pytest.mark.parametrize("robot", ROBOTS)
def test_ik_joint_plan_reproduces_the_tool_path(robot, default_spec):
    arm = load_arm(EXAMPLES / "robots" / f"{robot}.urdf")
    plan = plan_from_cartesian(arm, default_spec)
    assert plan.meta["kind"] == "cartesian"
    assert plan.meta["ik"]["max_position_error"] < 1e-5

    model, _ = ik_model(arm, None)
    data = model.createData()
    frame = model.getFrameId(TOOL_FRAME)
    _, segments = build_path(default_spec)
    _, poses, _ = sample_path(segments, 1000.0)
    times = np.linspace(0, plan.duration, len(poses))
    for i in range(0, len(poses), 97):  # includes points between IK samples
        q, _, _ = plan.sample(times[i])
        qv = np.array([q[n] for n in arm.actuated])
        pin.framesForwardKinematics(model, data, qv)
        err = np.linalg.norm(data.oMf[frame].translation - poses[i].translation)
        assert err < 5e-4, (robot, times[i], err)
    # Segment boundaries are marked for the timeline.
    assert len(plan.waypoint_times) >= len(default_spec["waypoints"])


def test_joint_velocity_is_zero_at_stops(default_spec):
    arm = load_arm(EXAMPLES / "robots" / "kr_style_6dof.urdf")
    plan = plan_from_cartesian(arm, default_spec)
    for t in plan.waypoint_times[1:-1]:
        _, qd, _ = plan.sample(t)
        assert max(abs(v) for v in qd.values()) < 0.02


def test_unreachable_pose_names_the_waypoint(default_spec):
    arm = load_arm(EXAMPLES / "robots" / "simple_6dof.urdf")
    spec = json.loads(json.dumps(default_spec))
    spec["waypoints"][2]["xyz"] = [2.0, 0.3, 0.2]  # "pick", far out of reach
    with pytest.raises(ValueError, match="pick.*not reachable|not reachable.*pick"):
        plan_from_cartesian(arm, spec)


def test_needs_a_tool_tip(one_joint, default_spec):
    arm = parse_arm(one_joint().replace('<tool_tip name="tip" xyz="1 0 0"/>', ""))
    with pytest.raises(ValueError, match="tool_tip"):
        plan_from_cartesian(arm, default_spec)


@pytest.mark.parametrize(
    "mutate, message",
    [
        (lambda s: s.update(waypoints=s["waypoints"][:1]), "at least two"),
        (lambda s: s["waypoints"][1].update(path="arc"), "path"),
        (lambda s: s["waypoints"][1].update(xyz=[0, 0]), "xyz"),
        (lambda s: s["waypoints"][1].update(duration=0), "duration"),
        (lambda s: s["limits"].update(linear_speed=-1), "linear_speed"),
        (lambda s: s.update(units="grad"), "units"),
    ],
)
def test_bad_cartesian_specs(default_spec, mutate, message):
    spec = json.loads(json.dumps(default_spec))
    mutate(spec)
    with pytest.raises(ValueError, match=message):
        build_path(spec)


def test_dispatch_from_text(default_spec):
    arm = load_arm(EXAMPLES / "robots" / "kr_style_6dof.urdf")
    plan = load_trajectory(arm, DEFAULT.read_text())
    assert plan.meta["kind"] == "cartesian"
    json.dumps(plan.to_dict(), allow_nan=False)


def test_waypoint_check_follows_the_path_branch(default_spec):
    from arm_analyzer.cartesian import build_path, solve_waypoints

    arm = load_arm(EXAMPLES / "robots" / "kr_style_6dof.urdf")
    wps, _ = build_path(default_spec)
    tool, results = solve_waypoints(arm, wps, None)
    assert tool == "flange"
    assert all(r["reachable"] for r in results)
    # Same configuration as the full path solve at the waypoint times.
    plan = plan_from_cartesian(arm, default_spec)
    stops = [s for s in plan.meta["segments"] if s["path"] != "dwell"]
    times = [0.0] + [s["t1"] for s in stops]
    for r, t in zip(results, times):
        q, _, _ = plan.sample(t)
        assert max(abs(q[n] - r["q"][n]) for n in arm.actuated) < 1e-3


def test_preview_reports_unreachable_waypoints_without_failing(default_spec):
    from arm_analyzer.cartesian import preview

    arm = load_arm(EXAMPLES / "robots" / "simple_6dof.urdf")
    spec = json.loads(json.dumps(default_spec))
    spec["waypoints"][2]["xyz"] = [2.0, 0.3, 0.2]
    out = preview(arm, spec)
    flags = [w["reachable"] for w in out["waypoints"]]
    assert flags[2] is False and all(flags[:2]) and all(flags[3:])
    assert out["waypoints"][2]["q"] is None
    assert out["waypoints"][2]["position_error"] > 1.0
    assert out["tools"] == ["tcp"]
    assert len(out["path"]) > 100 and out["duration"] > 0
    json.dumps(out, allow_nan=False)


TOUR = EXAMPLES / "trajectories" / "workspace_tour.json"


def test_workspace_tour_covers_the_workspace():
    spec = json.loads(TOUR.read_text())
    wps, _ = build_path(spec)
    spots = [w for w in wps if w.dwell > 0]
    assert len(spots) == 10
    # Tool approach directions: down, up, horizontal (out and sideways) and tilted.
    dirs = np.array([w.R[:, 2] for w in spots])
    radial = np.array([w.p[:2] / np.linalg.norm(w.p[:2]) for w in spots])
    vertical = dirs[:, 2]
    assert (vertical < -0.99).sum() >= 2  # down
    assert (vertical > 0.99).sum() >= 1  # up
    horizontal = np.abs(vertical) < 1e-3  # rpy is stored to 1e-4 deg
    along = np.einsum("ij,ij->i", dirs[:, :2], radial)
    assert (horizontal & (along > 0.99)).sum() >= 2  # out
    assert (horizontal & (np.abs(along) < 1e-3)).sum() >= 2  # sideways
    assert ((np.abs(vertical) > 0.5) & (np.abs(vertical) < 0.9)).sum() >= 2  # tilted
    # All around the base, near and far, low and high.
    az = np.degrees(np.arctan2([w.p[1] for w in spots], [w.p[0] for w in spots]))
    assert az.min() < -120 and az.max() > 120
    heights = [w.p[2] for w in spots]
    assert min(heights) <= 0.15 and max(heights) >= 0.85


@pytest.mark.parametrize("robot", TOUR_ROBOTS)
def test_workspace_tour_is_feasible_for_every_example_arm(robot):
    from arm_analyzer.analysis import analyze

    arm = load_arm(EXAMPLES / "robots" / f"{robot}.urdf")
    plan = plan_from_cartesian(arm, json.loads(TOUR.read_text()))
    assert plan.meta["ik"]["max_position_error"] < 5e-5
    result = analyze(arm, plan, rate_hz=100)
    for s in result["summary"]:
        assert s["status"] == "ok", (robot, s["name"], s["joint"], s["drive"])


@pytest.mark.parametrize("robot", ["palletizer_4dof", "wam_style_7dof"])
def test_arms_that_cannot_tip_the_tool_over_are_rejected_by_name(robot):
    """The tour asks for tool-out and tool-up poses; say which pose fails."""
    arm = load_arm(EXAMPLES / "robots" / f"{robot}.urdf")
    with pytest.raises(ValueError, match="not reachable"):
        plan_from_cartesian(arm, json.loads(TOUR.read_text()))


def test_ur_style_reaches_the_whole_tour():
    """It has the range for every pose; the wrist just runs out of speed."""
    from arm_analyzer.analysis import analyze

    arm = load_arm(EXAMPLES / "robots" / "ur_style_6dof.urdf")
    plan = plan_from_cartesian(arm, json.loads(TOUR.read_text()))
    assert plan.meta["ik"]["max_position_error"] < 5e-5
    result = analyze(arm, plan, rate_hz=100)
    over = {s["name"] for s in result["summary"] if s["status"] == "over"}
    assert over == {"j4"}  # 106% of its declared 360 deg/s, no torque problem
    for s in result["summary"]:
        assert s["drive"]["motor"]["rms_util"] < 0.5
