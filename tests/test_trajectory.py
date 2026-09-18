import json
import math

import numpy as np
import pytest

from arm_analyzer.trajectory import load_trajectory, parse_csv_samples, plan_from_samples

from conftest import EXAMPLES


def test_example_waypoint_file(simple_arm):
    text = (EXAMPLES / "trajectories" / "pick_and_place.json").read_text()
    plan = load_trajectory(simple_arm, text)
    spec = json.loads(text)
    assert len(plan.waypoint_times) == len(spec["waypoints"])
    # Degrees in the file, radians in the plan.
    q, qd, _ = plan.sample(plan.waypoint_times[2])
    assert q["j2"] == pytest.approx(math.radians(spec["waypoints"][2]["joints"][1]), abs=1e-9)
    assert all(abs(v) < 1e-9 for v in qd.values())  # "pick" is a stop


def test_example_csv_file(simple_arm):
    text = (EXAMPLES / "trajectories" / "sine_sweep.csv").read_text()
    names, t, q = parse_csv_samples(text)
    plan = load_trajectory(simple_arm, text, kind="csv")
    assert names == simple_arm.actuated
    assert plan.duration == pytest.approx(t[-1] - t[0])
    for i in (0, 137, 400, len(t) - 1):
        got, _, _ = plan.sample(t[i] - t[0])
        for k, n in enumerate(names):
            assert got[n] == pytest.approx(q[i, k], abs=1e-9)


def test_spline_reproduces_a_cubic_exactly():
    # A not-a-knot cubic spline is exact for cubic data, so derivatives are too.
    t = np.linspace(0.5, 2.5, 21)
    f = lambda x: 0.3 - 0.2 * x + 0.7 * x**2 - 0.25 * x**3  # noqa: E731
    plan = plan_from_samples(["a"], t, f(t)[:, None], ["a", "b"])
    for x in (0.5, 0.93, 1.71, 2.5):
        q, qd, qdd = plan.sample(x - 0.5)
        assert q["a"] == pytest.approx(f(x), abs=1e-10)
        assert qd["a"] == pytest.approx(-0.2 + 1.4 * x - 0.75 * x**2, abs=1e-9)
        assert qdd["a"] == pytest.approx(1.4 - 1.5 * x, abs=1e-8)
        assert q["b"] == 0.0  # joints not in the file are held at zero


def test_smoothing_reduces_noise_in_acceleration():
    rng = np.random.default_rng(3)
    t = np.linspace(0, 4, 401)
    clean = np.sin(t)
    noisy = clean + rng.normal(0, 2e-3, t.shape)
    exact = plan_from_samples(["a"], t, noisy[:, None], ["a"])
    smooth = plan_from_samples(["a"], t, noisy[:, None], ["a"], smoothing="auto")

    def accel_error(plan):
        errs = [plan.sample(x)[2]["a"] + math.sin(x) for x in np.linspace(0.5, 3.5, 60)]
        return float(np.sqrt(np.mean(np.square(errs))))

    assert accel_error(smooth) < 0.2 * accel_error(exact)


def test_waypoint_list_form_and_units(simple_arm):
    plan = load_trajectory(
        simple_arm,
        {
            "format": "waypoints",
            "units": "deg",
            "waypoints": [
                {"joints": [0, 0, 0, 0, 0, 0]},
                {"joints": [90, 0, 0, 0, 0, 0]},
            ],
        },
    )
    q, _, _ = plan.sample(plan.duration)
    assert q["j1"] == pytest.approx(math.pi / 2)


def test_json_samples(simple_arm):
    plan = load_trajectory(
        simple_arm,
        json.dumps(
            {
                "format": "samples",
                "units": "deg",
                "joint_names": ["j2", "j3"],
                "t": [0, 1, 2, 3],
                "q": [[0, 0], [10, 20], [20, 10], [30, 0]],
            }
        ),
    )
    q, _, _ = plan.sample(1.0)
    assert q["j2"] == pytest.approx(math.radians(10))
    assert q["j1"] == 0.0


@pytest.mark.parametrize(
    "content, kind, message",
    [
        ("t,j1\n0,0\n0,1\n", "csv", "strictly increasing"),
        ("t,zz\n0,0\n1,1\n", "csv", "not an actuated joint"),
        ("time_ms,j1\n0,0\n1,1\n", "csv", "time column"),
        ("t,j1\n0,0\n1,abc\n", "csv", "non-numeric"),
        ("t,j1\n0,0\n", "csv", "at least two samples"),
        ('{"format": "waypoints", "waypoints": []}', None, "non-empty"),
        ('{"format": "waypoints", "waypoints": [{"joints": [1, 2]}]}', None, "values for 6 joints"),
        ('{"format": "bogus"}', None, "unknown trajectory format"),
        ('{"waypoints": [{"joints": {"j1": 0}}], "units": "grad"}', None, "units"),
        ("{not json", "json", "invalid JSON"),
    ],
)
def test_bad_trajectories(simple_arm, content, kind, message):
    with pytest.raises(ValueError, match=message):
        load_trajectory(simple_arm, content, kind=kind)
