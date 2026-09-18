"""Generate the example trajectories in ``examples/trajectories/``.

* ``pick_and_place.json`` -- waypoints (degrees), timed by the planner.
* ``sine_sweep.csv``      -- 100 Hz timed samples (radians): every joint on its
  own sinusoid, so each drive sweeps through a range of speeds and torques.
* ``default_pick_place.json`` -- a robot-independent CARTESIAN tool path:
  straight vertical approach / retract at the pick and place poses, smooth
  curved transfers everywhere else, tool pointing down. Joint motion comes from
  IK against whichever robot is loaded.
* ``workspace_tour.json`` -- a robot-independent CARTESIAN test tour:
  pick-and-place moves at 10 spots all around the base, with the tool
  pointing down, out, up, sideways and tilted in between.
* ``kr_pick_and_place.json`` -- waypoints for ``kr_style_6dof`` (zero pose:
  upper arm vertical, forearm horizontal). A5 keeps the flange pointing down
  (j2 + j3 + j5 = 90 deg) at every pose; picks ~0.23 m above the floor.

Run from the project root:

    python scripts/make_example_trajectories.py
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parents[1] / "examples" / "trajectories"
JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]


def pick_and_place() -> dict:
    poses = [
        ("home", [0, 0, 0, 0, 0, 0], True),
        ("above_pick", [45, 35, 70, 0, 60, 0], False),
        ("pick", [45, 50, 75, 0, 55, 0], True),
        ("lift", [30, 30, 80, 30, 60, 45], False),
        ("above_place", [-60, 35, 70, -20, 70, -90], False),
        ("place", [-60, 55, 60, -20, 65, -90], True),
        ("home", [0, 0, 0, 0, 0, 0], True),
    ]
    return {
        "format": "waypoints",
        "label": "Pick and place",
        "units": "deg",
        "joint_order": JOINTS,
        # 1.3x slower than the planner's fastest timing, so joints peak at
        # ~77% of their velocity limits instead of exactly 100%.
        "limits": {"accel_ratio": 4.0, "jerk_ratio": 20.0, "time_scale": 1.3},
        "waypoints": [{"name": n, "joints": q, "stop": s} for n, q, s in poses],
    }


def default_pick_place() -> dict:
    # Tool z pointing at the floor. Ry(180) rather than Rx(180): it is reached
    # with the wrist roll joints near zero on in-line wrists, well inside limits.
    down = [0, 180, 0]
    down_turned = [0, 180, 90]  # same, part rotated 90 deg about the vertical
    home = [0.40, 0.0, 0.55]
    pick = [0.40, 0.30, 0.20]
    place = [0.40, -0.30, 0.20]
    clearance = 0.12  # approach / retract height above pick and place

    def above(p):
        return [p[0], p[1], p[2] + clearance]

    return {
        "format": "cartesian",
        "label": "Default pick and place (cartesian)",
        "units": "deg",
        "limits": {"linear_speed": 0.15, "smooth_speed": 0.5, "angular_speed": 180},
        "waypoints": [
            {"name": "home", "xyz": home, "rpy": down},
            {"name": "above_pick", "xyz": above(pick), "rpy": down, "path": "smooth"},
            {"name": "pick", "xyz": pick, "rpy": down, "path": "linear", "dwell": 0.3},
            {"name": "retract_pick", "xyz": above(pick), "rpy": down, "path": "linear"},
            {"name": "above_place", "xyz": above(place), "rpy": down_turned, "path": "smooth"},
            {"name": "place", "xyz": place, "rpy": down_turned, "path": "linear", "dwell": 0.3},
            {"name": "retract_place", "xyz": above(place), "rpy": down_turned, "path": "linear"},
            {"name": "home", "xyz": home, "rpy": down, "path": "smooth"},
        ],
    }


def _rot_z(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def _rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def _rot_x(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def _rpy_deg(R: np.ndarray) -> list[float]:
    """URDF roll/pitch/yaw (R = Rz Ry Rx) in degrees, valid at gimbal lock too."""
    sy = math.hypot(R[0, 0], R[1, 0])
    if sy > 1e-9:
        rpy = (math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy), math.atan2(R[1, 0], R[0, 0]))
    else:  # pitch = +/-90: fold yaw into roll
        rpy = (math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0)
    out = [round(math.degrees(v), 4) for v in rpy]
    back = _rot_z(out[2]) @ _rot_y(out[1]) @ _rot_x(out[0])
    assert np.allclose(back, R, atol=1e-5), "rpy round-trip failed"
    return out


# Spots of the workspace tour, all around the base:
#   azimuth (deg, about the base Z axis), radius (m), height (m),
#   pitch (180 = tool down, 90 = out, 0 = up), yaw offset (+/-90 turns the
#   tool sideways), spin about the tool axis (deg).
TOUR_SPOTS = [
    ("down_front", 0, 0.50, 0.15, 180, 0, 0),
    ("out_front_left", 35, 0.60, 0.40, 90, 0, 0),
    ("up_left_overhead", 70, 0.40, 0.90, 0, 0, 0),
    ("side_left", 105, 0.40, 0.25, 90, 90, 0),
    ("down_out_rear_left", 140, 0.40, 0.55, 135, 0, 0),
    ("down_rear_right_low", -140, 0.45, 0.10, 180, 0, 0),
    ("out_right", -105, 0.55, 0.50, 90, 0, 0),
    ("up_out_front_right", -70, 0.55, 0.65, 45, 0, 0),
    ("side_front_right", -35, 0.45, 0.30, 90, -90, 0),
    ("down_far_front_spun", 15, 0.60, 0.20, 180, 0, 90),
]
TOUR_HOME = {"xyz": [0.40, 0.0, 0.55], "rpy": [0, 180, 0]}
TOUR_SWING = (0.40, 0.55)  # radius, height of the tool-down swing circle
TOUR_STAGE = (0.50, 0.65)  # radius, height of the horizontal staging pose
TOUR_SWING_STEP = 45.0  # max azimuth change between swing poses (deg)


def workspace_tour(spots=None, stage=TOUR_STAGE, swing=TOUR_SWING, limits=None) -> dict:
    """Ten pick-and-place visits around the base.

    The spots are visited in one sweep around the base (positive azimuths out,
    negative ones on the way back, never crossing behind the robot), linked
    by tool-down "swing" poses on a circle, at most ``TOUR_SWING_STEP`` apart,
    so transfers never cut across the base axis. Spots that point out or up
    are reached by pitching the tool in their own vertical plane through a
    horizontal "stage" pose; sideways spots rotate straight from tool-down.
    Keeping reorientations planar avoids twisting in-line wrists through their
    singularity, which small arms cannot follow at speed. Approach and retract
    run straight along the tool axis.
    """
    spots = TOUR_SPOTS if spots is None else spots
    clearance = 0.08

    def at(az, r, z):
        a = math.radians(az)
        return [round(r * math.cos(a), 4), round(r * math.sin(a), 4), round(z, 4)]

    def swing_pose(name, az):
        return {"name": name, "xyz": at(az, *swing), "rpy": _rpy_deg(_rot_z(az) @ _rot_y(180)), "path": "smooth"}

    ordered = sorted((s for s in spots if s[1] >= 0), key=lambda s: s[1]) + sorted(
        (s for s in spots if s[1] < 0), key=lambda s: -s[1]
    )
    waypoints = [{"name": "home", **TOUR_HOME}]
    current = 0.0
    for i, (label, az, r, z, pitch, yaw, spin) in enumerate(ordered, 1):
        tag = f"{i:02d}_{label}"
        # Swing along the circle to face the spot.
        steps = max(1, math.ceil(abs(az - current) / TOUR_SWING_STEP))
        for k in range(1, steps + 1):
            a = current + (az - current) * k / steps
            waypoints.append(swing_pose(f"{tag}_face" if k == steps else f"{tag}_swing{k}", round(a, 4)))
        current = az
        R = _rot_z(az + yaw) @ _rot_y(pitch) @ _rot_z(spin)
        staged = pitch < 135 and yaw == 0
        lead_in = []
        if staged:
            lead_in.append({"name": f"{tag}_stage", "xyz": at(az, *stage), "rpy": _rpy_deg(_rot_z(az) @ _rot_y(90)), "path": "smooth"})
        spot = np.array(at(az, r, z))
        approach = spot - clearance * R[:, 2]  # back off along the tool axis
        rpy = _rpy_deg(R)
        waypoints += lead_in + [
            {"name": f"{tag}_approach", "xyz": [round(v, 4) for v in approach], "rpy": rpy, "path": "smooth"},
            {"name": tag, "xyz": [round(v, 4) for v in spot], "rpy": rpy, "path": "linear", "dwell": 0.3},
            {"name": f"{tag}_retract", "xyz": [round(v, 4) for v in approach], "rpy": rpy, "path": "linear"},
        ] + [dict(w, path="smooth") for w in reversed(lead_in)] + [swing_pose(f"{tag}_leave", az)]
    steps = max(1, math.ceil(abs(current) / TOUR_SWING_STEP))
    for k in range(1, steps):
        waypoints.append(swing_pose(f"return_swing{k}", round(current * (1 - k / steps), 4)))
    waypoints.append({"name": "home", **TOUR_HOME, "path": "smooth"})
    return {
        "format": "cartesian",
        "label": "Workspace tour: 10-spot pick and place (cartesian)",
        "units": "deg",
        # Chosen so every example arm stays under ~70 % of its joint speed limits.
        "limits": limits or {"linear_speed": 0.15, "smooth_speed": 0.5, "angular_speed": 120},
        "waypoints": waypoints,
    }


def kr_pick_and_place() -> dict:
    poses = [
        ("home", [0, 0, 0, 0, 90, 0], True),
        ("above_pick", [40, 35, 30, 0, 25, 0], False),
        ("pick", [40, 45, 35, 0, 10, 0], True),
        ("lift", [0, 20, 10, 0, 60, 45], False),
        ("above_place", [-70, 35, 30, 0, 25, 90], False),
        ("place", [-70, 45, 35, 0, 10, 90], True),
        ("home", [0, 0, 0, 0, 90, 0], True),
    ]
    return {
        "format": "waypoints",
        "label": "KR pick and place",
        "robot_hint": "kr_style_6dof",
        "units": "deg",
        "joint_order": JOINTS,
        "limits": {"accel_ratio": 4.0, "jerk_ratio": 20.0, "time_scale": 1.3},
        "waypoints": [{"name": n, "joints": q, "stop": s} for n, q, s in poses],
    }


def sine_sweep(rate_hz: float = 100.0, duration: float = 8.0) -> str:
    # amplitude (rad), period (s), offset (rad)
    params = {
        "j1": (1.2, 8.0, 0.0),
        "j2": (0.7, 4.0, 0.3),
        "j3": (0.9, 4.0, 0.8),
        "j4": (1.5, 2.67, 0.0),
        "j5": (1.0, 2.0, 0.4),
        "j6": (2.0, 1.6, 0.0),
    }
    lines = [
        "# Sine sweep: each joint follows q = offset + A*sin(2*pi*t/T) * ramp(t).",
        "# Units: radians. Generated by scripts/make_example_trajectories.py.",
        "t," + ",".join(JOINTS),
    ]
    n = int(round(duration * rate_hz)) + 1
    for i in range(n):
        t = i / rate_hz
        # Smooth start/stop so the trajectory begins and ends at rest.
        ramp = math.sin(math.pi * t / duration) ** 2
        row = [f"{t:.3f}"]
        for j in JOINTS:
            a, period, off = params[j]
            row.append(f"{off * ramp + a * ramp * math.sin(2 * math.pi * t / period):.6f}")
        lines.append(",".join(row))
    return "\n".join(lines) + "\n"


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "pick_and_place.json").write_text(
        json.dumps(pick_and_place(), indent=2) + "\n", encoding="utf-8"
    )
    (OUT / "sine_sweep.csv").write_text(sine_sweep(), encoding="utf-8")
    (OUT / "default_pick_place.json").write_text(
        json.dumps(default_pick_place(), indent=2) + "\n", encoding="utf-8"
    )
    (OUT / "workspace_tour.json").write_text(
        json.dumps(workspace_tour(), indent=2) + "\n", encoding="utf-8"
    )
    (OUT / "kr_pick_and_place.json").write_text(
        json.dumps(kr_pick_and_place(), indent=2) + "\n", encoding="utf-8"
    )
    print(f"wrote trajectories to {OUT}")


if __name__ == "__main__":
    main()
