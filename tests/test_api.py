import json

import pytest

pytest.importorskip("fastapi")
pytest.importorskip("httpx")

from fastapi.testclient import TestClient  # noqa: E402

from arm_analyzer.server import app  # noqa: E402


@pytest.fixture(scope="module")
def client():
    return TestClient(app)


def test_index_and_static(client):
    assert client.get("/").status_code == 200
    r = client.get("/static/main.js")
    assert r.status_code == 200
    assert r.headers["cache-control"].startswith("no-store")


def test_listings(client):
    robots = client.get("/api/robots").json()
    assert {"file": "simple_6dof.urdf", "label": "simple 6dof"} in robots
    files = {t["file"] for t in client.get("/api/trajectories").json()}
    assert {"default_pick_place.json", "pick_and_place.json", "sine_sweep.csv"} <= files


@pytest.mark.parametrize(
    "path",
    [
        "/api/robots/..%2Fpyproject.toml",
        "/api/robots/simple_6dof.json",
        "/api/trajectories/..%2F..%2Fpyproject.toml",
        "/api/trajectories/.hidden.json",
    ],
)
def test_file_access_is_contained(client, path):
    assert client.get(path).status_code in (400, 404)


def test_missing_file(client):
    assert client.get("/api/robots/nope.urdf").status_code == 404


def test_model(client, simple_urdf):
    r = client.post("/api/robot/model", json={"urdf": simple_urdf})
    assert r.status_code == 200
    body = r.json()
    assert body["actuated"] == ["j1", "j2", "j3", "j4", "j5", "j6"]
    assert len(body["drives"]) == 6


def test_model_errors(client):
    assert client.post("/api/robot/model", json={}).status_code == 400
    r = client.post("/api/robot/model", json={"urdf": "<robot><joint/></robot>"})
    assert r.status_code == 400
    assert "name" in r.json()["detail"]


@pytest.mark.parametrize("name", ["pick_and_place.json", "sine_sweep.csv"])
def test_analyze_examples(client, simple_urdf, name):
    traj = client.get(f"/api/trajectories/{name}").json()["text"]
    r = client.post(
        "/api/analyze",
        json={
            "urdf": simple_urdf,
            "trajectory": traj,
            "trajectory_kind": "csv" if name.endswith(".csv") else "json",
            "rate_hz": 100,
            "payload": {"mass": 0.5},
        },
    )
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["payload_mass"] == 0.5
    assert [s["name"] for s in body["summary"]] == body["joints"]
    assert body["plan"]["segments"]
    json.dumps(body, allow_nan=False)


def test_analyze_cartesian_trajectory(client, simple_urdf):
    traj = client.get("/api/trajectories/default_pick_place.json").json()["text"]
    r = client.post("/api/analyze", json={"urdf": simple_urdf, "trajectory": traj, "rate_hz": 100})
    assert r.status_code == 200, r.text
    meta = r.json()["plan"]["meta"]
    assert meta["kind"] == "cartesian"
    assert meta["ik"]["tool"] == "tcp"
    assert meta["ik"]["max_position_error"] < 1e-5
    assert [w["name"] for w in meta["waypoints"]][:3] == ["home", "above_pick", "pick"]


def test_cartesian_preview_endpoint(client, simple_urdf):
    spec = json.loads(client.get("/api/trajectories/default_pick_place.json").json()["text"])
    r = client.post("/api/trajectory/cartesian/preview", json={"urdf": simple_urdf, "trajectory": spec})
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["tool"] == "tcp"
    assert all(w["reachable"] for w in body["waypoints"])
    # Malformed or non-cartesian specs are client errors.
    bad = dict(spec, waypoints=spec["waypoints"][:1])
    assert client.post("/api/trajectory/cartesian/preview", json={"urdf": simple_urdf, "trajectory": bad}).status_code == 400
    # Without a robot: the path only, nothing robot-specific.
    r = client.post("/api/trajectory/cartesian/preview", json={"trajectory": spec})
    assert r.status_code == 200, r.text
    plain = r.json()
    assert plain["path"] == body["path"] and plain["duration"] == body["duration"]
    assert plain["tools"] == []
    assert all(w["reachable"] is None for w in plain["waypoints"])
    joint = {"format": "waypoints", "waypoints": []}
    r = client.post("/api/trajectory/cartesian/preview", json={"urdf": simple_urdf, "trajectory": joint})
    assert r.status_code == 400 and "cartesian" in r.json()["detail"]


def test_payload_increases_torque(client, simple_urdf):
    traj = client.get("/api/trajectories/pick_and_place.json").json()["text"]

    def peak_j2(mass):
        r = client.post(
            "/api/analyze",
            json={"urdf": simple_urdf, "trajectory": traj, "rate_hz": 50, "payload": {"mass": mass}},
        )
        return next(s for s in r.json()["summary"] if s["name"] == "j2")["joint"]["peak_torque"]

    assert peak_j2(3.0) > peak_j2(0.0) + 3.0 * 9.8 * 0.3


def test_efficiency_override(client, simple_urdf):
    traj = client.get("/api/trajectories/pick_and_place.json").json()["text"]

    def j2(efficiency):
        r = client.post(
            "/api/analyze",
            json={"urdf": simple_urdf, "trajectory": traj, "rate_hz": 50, "efficiency": efficiency},
        )
        assert r.status_code == 200, r.text
        return next(s for s in r.json()["summary"] if s["name"] == "j2")

    base = j2({})
    lossy = j2({"j2": {"gearbox": 0.8, "transmission": 0.9}})
    assert lossy["drive"]["efficiency"]["total"] == pytest.approx(0.72)
    assert lossy["joint"]["peak_torque"] == pytest.approx(base["joint"]["peak_torque"])
    assert lossy["drive"]["motor"]["rms_torque"] > base["drive"]["motor"]["rms_torque"]
    assert lossy["joint"]["effort_limit"] == pytest.approx(base["joint"]["effort_limit"] * 0.72)


@pytest.mark.parametrize(
    "patch, message",
    [
        ({"trajectory": "t,j9\n0,0\n1,1\n", "trajectory_kind": "csv"}, "j9"),
        ({"gravity": [0, 0]}, "gravity"),
        ({"payload": {"mass": -1}}, "payload"),
        ({"rate_hz": 1e6}, "rate_hz"),
        ({"efficiency": {"j9": {"gearbox": 0.9}}}, "j9"),
        ({"efficiency": {"j1": {"gearbox": 1.5}}}, "efficiency"),
        ({"efficiency": {"j1": {"belt": 0.9}}}, "gearbox"),
        ({"trajectory": ""}, "trajectory"),
    ],
)
def test_analyze_errors(client, simple_urdf, patch, message):
    body = {
        "urdf": simple_urdf,
        "trajectory": '{"waypoints": [{"joints": {"j1": 0}}, {"joints": {"j1": 1}}]}',
        **patch,
    }
    r = client.post("/api/analyze", json=body)
    assert r.status_code == 400
    assert message in r.json()["detail"]
