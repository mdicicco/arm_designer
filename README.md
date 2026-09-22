# Arm Analyzer

Speed–torque analysis for **one serial robot arm** whose every joint is driven by
a motor and a gearbox that you place and size yourself. Load the arm, import a
trajectory, and see — for every joint — where its operating points fall
relative to the motor's and gearbox's ratings, so you can resize drives, change
ratios, or move actuators to cut mass.

Derived from the Modular Robot Studio framework (same GUI stack, colour scheme
and jerk-limited planner) but self-contained: nothing here imports
`modular_robot`. Kinematics and rigid-body dynamics are
[Pinocchio](https://github.com/stack-of-tasks/pinocchio) (C++); Cartesian
paths use [ndcurves](https://github.com/loco-3d/ndcurves) and IK uses
[pink](https://github.com/stephane-caron/pink), all from conda-forge.

The environment is Python 3.14 with Pinocchio 4.0: conda-forge's current
ndcurves builds require exactly that Pinocchio, and its Linux/Windows Python
bindings are built for 3.14 only.

```bash
pixi install
pixi run gui          # http://127.0.0.1:8766
pixi run -e dev test
```

or with pip:

```bash
python -m venv .venv && source .venv/bin/activate
pip install -e ".[gui,dev,kinematics]"
arm-analyzer-gui
```

## The GUI

One page, three equal columns:

| Column | Shows |
|---|---|
| **URDF editor** | A 3-D view of the arm frozen at its zero pose, annotated with joint offsets (dimension legs in each parent link's frame, mm), link frames, joint axes, centres of mass with their masses, link/joint names, and a blue ring per joint for its transmission. Click a link, motor, gearbox or ring — or use the picker — to edit it in the property panel below. With nothing selected the panel shows the mass budget, warnings, losses for all joints, and a joint list. |
| **Motion** | The arm posed along the trajectory with the tool path and, for remote drives, a dashed motor → gearbox → joint line. Transport bar for playback and a *Jog* pop-over for moving joints by hand. Below it, torque vs time for all joints, either as raw torque or as a fraction of each drive's envelope. |
| **Speed–torque** | Analysis options, one speed–torque plot per joint, and a summary table. Plots switch between **joint side** (joint torque vs joint speed, against the drive envelope referred to the joint) and **motor side** (motor torque vs rpm, against the motor's own curves). |

### Editing

| Selected | Editable |
|---|---|
| Link | Shapes (box / cylinder / sphere, size, position, rotation; add / remove), mass, COM, inertia — with *Estimate from shapes* and an optional *keep in sync* — and the placement of the joint that carries it (offset, rotation, axis, limits, max speed, max torque) |
| Motor | Mounting link, position, rotation, mass, shape; peak / continuous / stall torque, no-load speed (rpm), rotor inertia, torque constant, resistance |
| Gearbox | Mounting link, position, rotation, mass, shape; ratio, efficiency, peak / rated output torque, max input speed (rpm), input inertia |
| Transmission | Ratio and efficiency, plus the joint's placement and limits |

Values are shown in mm, degrees, rpm and N·m and stored in SI. Every edit is
re-parsed by the server as you type, so both views and the analysis update
live; an invalid value is reported in the status bar and the last valid model
stays on screen. **Undo / Redo** (Ctrl+Z / Ctrl+Shift+Z) step through edits,
**Revert** returns to the file as loaded, and **Download** saves the edited
URDF — nothing is written to disk by the server.

In each speed–torque plot the solid curve is the peak envelope, the dashed curve is
the continuous one, ◆ is the RMS operating point, ● is the playhead, and red
dots are samples outside the peak envelope. *Fold* plots magnitudes in one
quadrant; *Zoom to data* frames the trace instead of the whole envelope.

A selection is highlighted everywhere: the part in both views, and its joint's
plot, timeline trace and summary row. Clicking a plot or summary row selects
that joint's transmission. Keys: **Space** play/pause, **Home** rewind,
**Esc** clear selection.

Views can be bookmarked with query parameters:
`/?robot=kr_style_6dof.urdf&traj=kr_pick_and_place.json&select=motor:j5&side=motor&zoom=1&t=2.5`
(also `joint=j3` for `select=transmission:j3`, `timeline=torque|util`,
`fold=0|1`, `view=iso|top|front|right`, `jog=1`, `edit=1` to open the trajectory editor).

## Robot description

Standard URDF for the structure — link `<visual>` geometry (box, cylinder,
sphere) and an `<inertial>` block with mass, COM and inertia per link — plus a
`<drive>` element on every actuated joint:

```xml
<joint name="j3" type="revolute">
  <parent link="link2"/><child link="link3"/>
  <origin xyz="0 0 0.35"/><axis xyz="0 1 0"/>
  <limit lower="-2.5" upper="2.5" velocity="3"/>        <!-- effort optional -->
  <drive>
    <motor name="j3_motor" link="link1"
           rotor_inertia="1.5e-5" peak_torque="0.64" continuous_torque="0.22"
           stall_torque="1.6" no_load_speed="680"
           torque_constant="0.05" resistance="1.1">
      <origin xyz="-0.03 -0.105 0.12" rpy="-1.5708 0 0"/>
      <mass value="0.30"/>
      <geometry><cylinder radius="0.028" length="0.05"/></geometry>
    </motor>
    <gearbox name="j3_gearbox" link="link1" ratio="100" efficiency="1.00"
             input_inertia="2e-6" peak_torque="60" rated_torque="25"
             max_input_speed="700">
      <origin xyz="-0.03 -0.06 0.12" rpy="-1.5708 0 0"/>
      <mass value="0.35"/>
      <geometry><cylinder radius="0.038" length="0.04"/></geometry>
    </gearbox>
    <transmission ratio="1" efficiency="1.00"/>         <!-- joint + belt / cable / linkage -->
  </drive>
</joint>
```

**Motors and gearboxes are lumps of mass.** Each is rigidly attached to the link
named by `link=`, at `<origin>` (its centre of mass; local +Z is the shaft). That
link can be anywhere in the arm. Put it on the joint's parent for a co-located
drive, or further back (as above) for a remote actuator. Inertia comes from
`<inertia>` if given, otherwise from `<geometry>` scaled to the declared mass,
otherwise the lump is treated as a point mass. `<geometry>` is also what the
viewer draws.

| Attribute | Meaning |
|---|---|
| motor `rotor_inertia` | spinning inertia (kg·m²), reflected as `I·N²` |
| motor `peak_torque`, `continuous_torque` | current-limited ratings (N·m) |
| motor `stall_torque`, `no_load_speed` | the voltage line `τ = stall·(1 − ω/ω₀)`; speeds in rad/s, or `no_load_speed_rpm` |
| motor `torque_constant`, `resistance` | optional; enable current and I²R copper loss |
| gearbox `ratio` | reduction |
| gearbox `efficiency` | placeholder for gear friction, (0, 1], default 1 |
| gearbox `input_inertia` | motor-side inertia, reflected like the rotor |
| gearbox `peak_torque`, `rated_torque` | **output-side** ratings (N·m) |
| gearbox `max_input_speed` | rad/s, or `max_input_speed_rpm` |
| transmission `ratio` | extra reduction stage after the gearbox (default 1) |
| transmission `efficiency` | placeholder for friction in the joint and any belt / cable / linkage, (0, 1], default 1 |

### Differentials and linkages

Two mechanisms break the one-motor-one-joint rule, and both are declared
outside `<drive>`.

**A differential** gears two motors to two joints at once. Declare it at robot
level and both drives are carried back through it together:

```xml
<drive_coupling name="shoulder" type="differential" joints="j2 j3"/>
```

Motor 1 then turns with `N·(q2 + q3)` and motor 2 with `N·(q2 − q3)`, so each
carries **half the sum** of the two joint torques and half their difference —
the shoulder's weight is shared by both motors instead of resting on one — and
each joint carries **both** rotors' reflected inertia. Either joint on its own
can be driven twice as hard as one motor allows, but only while its partner is
unloaded, which is what the joint-side envelope shows. `type="matrix"` with one
`<motor joint="…" gains="…"/>` row per joint states an arbitrary map instead.

**A passive linkage** is a standard URDF `<mimic>`: the joint follows another
one through a rod or belt rather than a motor.

```xml
<joint name="level_elbow" type="revolute">
  …
  <mimic joint="j3" multiplier="-1"/>          <!-- no <drive> -->
</joint>
```

It is not a degree of freedom, never appears in a trajectory, and carries no
drive — but it does carry mass, and its load lands on the joint it follows.
Chaining two of them is how a palletizer keeps its tool plate level.

**Losses are placeholders.** There is no friction model: a joint's URDF
`<dynamics>` is ignored with a warning. All losses are meant to be lumped into
the two efficiencies above, which the examples set to 1.0 (lossless), so the
default results are exactly the torque the motion requires. The chain panel has
a slider for each (per joint, and one pair for all joints) to see what losses
would do without editing the file; *Reset* returns to the file's values.

All ratings are optional; anything left out is simply not checked (shown as
"—"). Units are SI. If `<limit effort>` is absent, it is derived from the drive.
An actuated joint with no `<drive>`, or a link with no `<inertial>`, is
analysed anyway and listed as a warning. `<tool_tip name="tcp" xyz="…"/>` inside
a link marks where payloads attach and what the path overlay traces.

## Trajectories

**Cartesian (JSON)** — robot-independent tool paths, turned into joint motion
by IK against whichever robot is loaded. This is the default:
`default_pick_place.json`.

```json
{
  "format": "cartesian",
  "units": "deg",
  "tool": "tcp",
  "limits": {"linear_speed": 0.15, "smooth_speed": 0.5, "angular_speed": 180},
  "waypoints": [
    {"name": "home",       "xyz": [0.40, 0.00, 0.55], "rpy": [0, 180, 0]},
    {"name": "above_pick", "xyz": [0.40, 0.30, 0.32], "rpy": [0, 180, 0], "path": "smooth"},
    {"name": "pick",       "xyz": [0.40, 0.30, 0.20], "rpy": [0, 180, 0], "path": "linear", "dwell": 0.3},
    {"name": "retract",    "xyz": [0.40, 0.30, 0.32], "rpy": [0, 180, 0], "path": "linear"}
  ]
}
```

Poses are the tool tip (`<tool_tip>`; `tool` picks one by name) in the robot
base frame; `rpy [0, 180, 0]` points the tool straight down. Each waypoint
says how the tool gets there:

- `linear` — a straight line (approach / retract);
- `smooth` — a cubic Bézier that leaves along the previous linear segment and
  arrives along the next, so transfers curve into and out of the straight
  moves without a corner.

Orientation is slerped. Every segment runs rest-to-rest on a **minimum-jerk**
time law (ndcurves `MinimumJerk`), lasting the shortest time that keeps the
peak tool speed and rotation rate within `limits` — or `duration` / `speed`
per waypoint. `dwell` pauses at a waypoint.

**Editing.** *Edit…* (next to the trajectory picker) opens the trajectory
editor over the page:

- its own 3-D view of the trajectory alone — the path, every waypoint's tool
  frame (blue = approach axis) and the robot base frame the poses are given in;
  no robot is drawn, since the same trajectory serves any robot. Click a
  waypoint to select it, then drag it with the **Move** / **Rotate** gizmo
  (W / E);
- the waypoint list (add, duplicate, reorder, delete, and *From motion pose*
  to copy the tool pose currently shown in the Motion view — position it with
  Jog first), a form for the selected waypoint (position in mm, orientation in
  degrees, linear / smooth, dwell, speed or duration), and the trajectory's
  tool and speed limits;
- a live preview of the path and its duration on every edit, and an optional
  **Check reach** toggle that IK-checks each waypoint against the robot
  currently loaded (unreachable ones turn red) without drawing it.

**Save & solve IK** solves the whole path for the loaded robot and re-runs the analysis on the
resulting joint angles and velocities. If any pose along the path can't be
reached, the editor stays open with the error and the previous trajectory stays
loaded; otherwise the edited trajectory is selected as "… (edited)".
**Download** keeps it as a JSON file (the server never writes). A joint-space
trajectory can't be edited; *Edit…* then offers to start from the default
cartesian one. Undo / Redo (Ctrl+Z / Ctrl+Shift+Z) work inside the editor;
Esc cancels.

**`workspace_tour.json`** is a 72 s test tour: pick-and-place moves at 10
spots all around the base — tool down (one with the tool spun 90°), out, up
(under an overhead surface), sideways both ways, and tilted down-out / up-out —
from 0.10 m to 0.90 m high. It is generated by
`scripts/make_example_trajectories.py` (edit `TOUR_SPOTS` there). Spots are
visited in one sweep around the base (never across the back), joined by
tool-down poses on a circle; out/up spots are reached by pitching the tool in
their own vertical plane through a horizontal staging pose. Twisting an
in-line wrist through its singularity while also turning the base is what
small arms cannot follow, and this layout avoids it. All three example arms
run the tour with every joint "ok".

On analysis the path is sampled at 100 Hz (`ik_rate`) and each pose solved
with pink under the joint position limits, warm-started from the previous
sample so the arm stays on one configuration branch; the start pose tries a
few seeds and keeps the one closest to the zero pose. An unreachable pose or
a sudden joint jump (a singularity or limit on the path) is reported with the
waypoint and time. While tracking, the solver is gently pulled toward the
previous sample's joint angles: near a wrist singularity the tool pose barely
constrains the wrist rolls, and without that pull they swing back and forth
to chase micron-level errors, which appears as huge accelerations and
torques. The cost is a few microns of tool error at such poses (the IK error
is reported). The joint samples are then spline-fitted like any timed
trajectory. The motion view marks the working spots (waypoints with a dwell) and shows
the IK accuracy.

**Waypoints (JSON)** are timed by the jerk-limited planner. `stop: false` points
are splined through. Joints can be given as an object or as a list in
`joint_order`:

```json
{
  "format": "waypoints",
  "units": "deg",
  "joint_order": ["j1", "j2", "j3", "j4", "j5", "j6"],
  "limits": {"accel_ratio": 4, "jerk_ratio": 20, "time_scale": 1.3,
             "per_joint": {"j1": {"max_velocity": 1.0}}},
  "waypoints": [
    {"name": "home", "joints": [0, 0, 0, 0, 0, 0], "stop": true},
    {"name": "via",  "joints": {"j1": 30, "j2": 20}, "stop": false},
    {"name": "pick", "joints": [45, 50, 75, 0, 55, 0], "stop": true}
  ]
}
```

**Timed samples (CSV)**: a `t` column, then one column per joint. `#` lines are
comments. Angle units come from the *Import units* selector.

```csv
t,j1,j2,j3,j4,j5,j6
0.00,0,0,0,0,0,0
0.01,0.0001,0.0002,...
```

**Timed samples (JSON)**: `{"format": "samples", "units": "rad", "joint_names": [...], "t": [...], "q": [[...], ...]}`.

Samples are fitted with a C² cubic spline, so velocity and acceleration are
continuous. For **measured** data choose *Smoothing: auto*, which fits a smoothing
spline chosen by generalized cross-validation. Differentiating raw sensor noise
twice otherwise swamps the inertial torque. Joints a trajectory doesn't mention
are held at zero; prismatic positions are always metres.

## What is computed

Dynamics run in **Pinocchio** (`pin.rnea`, `computeGeneralizedGravity`, `crba`).
`src/arm_analyzer/pin_model.py` turns the description into a Pinocchio model:

- The URDF is normalized so urdfdom accepts it: extension tags are removed,
  missing `effort`/`velocity` are filled in, and `continuous` joints become
  `revolute` so each joint stays a single angle.
- Every motor and gearbox is attached to its host link with
  `Model.appendBodyToJoint`, plus a body frame of the same name.
- Each drive's reflected inertia is stored in `Model.armature`, which RNEA and
  CRBA both include.

Per sample (at the chosen rate):

```
τ_joint = τ_link + τ_rotor = pin.rnea(q, q̇, q̈)   (links carry their structure + every lump on them + payload)
τ_rotor = armature · q̈ = (I_rotor + I_gb,in) · N² · q̈      N = gearbox ratio × transmission ratio
```

and the drive side:

```
ω_motor = N · q̇
τ_motor       = τ_link / (N·η) + (I_rotor + I_gb,in)·N·q̈   while driving the load (τ_link·q̇ ≥ 0)
              = τ_link · η / N + …                          while the load back-drives it
τ_gearbox,out = τ_link / (N_tr·η_tr)   (· η_tr / N_tr when back-driven)
η = η_gearbox · η_transmission
```

Joint torque never includes losses: it is what the motion requires. Losses show
up on the motor and gearbox side and shrink the joint-side envelope
(`τ_motor,max · N · η`). With both efficiencies at 1.0, `τ_motor · N = τ_joint`
exactly.

Per joint, the summary reports:

- **Peak utilization**: `max |τ| / envelope(|ω|)`. A point above the curve at
  high speed counts even if its torque is below the stall rating.
- **RMS utilization**: RMS torque against the continuous rating.
- **Speed utilization**: peak speed against the no-load speed, the gearbox's
  max input speed, and the joint's velocity limit.
- **Power**: peak mechanical power and mean copper loss.

A joint is **marginal** above 80% and **over** above 100%.

### Gearbox mass from the torque rating

The same treatment for the reducers, from a second fit in `robot_arm_data`
(`analyze_gearbox_mass.py`, `data/gearbox_data.csv`): **145 bare reducers** —
not joint modules — spanning 4:1 to 111:1 and 0.4 to 784 N·m rated output, from
Harmonic Drive CSF-2UH/LW, Nabtesco RV-E, Neugart PLE, Maxon GPX, NMRV worm
boxes and AliExpress equivalents.

```
m = exp(a[type] + b·ln T_out + c·ln ratio)        m ∝ T_out^0.69 · ratio^-0.007
```

Two results carry the model:

- **Ratio barely matters** (`c = −0.007`). Within a frame a harmonic drive
  weighs the same at 30:1 as at 100:1 — the frame sets the mass, not the
  gearing. Doubling the *rated output torque* costs about 1.6×.
- **Type matters a lot.** At the same rating a worm box is roughly twice a
  planetary. `<gearbox>` takes a `type=` attribute — `harmonic` (the default),
  `cycloidal`, `planetary`, `worm` or `spur` — and each type carries its own
  intercept and its own housing density (cycloidal 2062, harmonic 3083, worm
  3758, spur 4623, planetary 4885 kg/m³, medians over the 106 rows listing OD
  and length).

This fit is much tighter than the motor one: **R² 0.92 on the logs, LOO R² 0.92,
LOO MAE 0.58 kg** — a reducer's mass really is close to a function of its frame
size. It covers the **gearbox only**, and like the motor law it reads the rating
a part is *specified* to, not the torque a trajectory demands.

`rated_torque` must be a **continuous** rating. Feeding it a peak figure sizes
the frame far too large — a harmonic drive's peak output is typically 2–3× its
continuous rating, so a `peak_torque / rated_torque` near 1 is the tell. See the
validation section: this is exactly what the `ur_style_6dof` example does.

### Link mass from geometry

Link mass can likewise be an output rather than an input. The **Link mass**
panel in the analysis options switches between the file's `<inertial>` blocks
and a derived sizing, and carries sliders for the factors behind it. Each link
becomes **one tube plus a collar per mounted actuator**:

- **The tube** spans the link — from the joint attaching it to its parent (its
  own frame origin) to the joint carrying the next link, or to the tool tip on
  a tip link. That length comes out of the URDF kinematics; nothing invents it.
  Its diameter tapers down the chain, because a shoulder carries the whole arm
  and a wrist carries a gripper:

  ```
  d(link i of n) = baseline_diameter · taper^(i / (n−1))
  ```

  `taper = 1` is a constant-diameter arm; `taper = 0.4` gives a wrist tube 40%
  of the base. The wall is `wall` thick, and a wall at least as thick as the
  radius makes the link solid.
- **A collar per actuator**, for the flanges and bearing seats that carry a
  motor or gearbox: each lump mounted on the link adds
  `actuator_mass + actuator_fraction · lump.mass` at that lump's own position,
  taking the lump's shape — a cylinder around the mount point.
  `actuator_fraction` is there because a bigger actuator needs a bigger boss.

Mass follows from geometry and inertia with it: the tube's tensor is a hollow
cylinder, each collar's is its shape scaled to its mass, and they combine
through the parallel-axis theorem. The result replaces the link's `<inertial>`
**in the Pinocchio model as well as in the budget** — link inertias otherwise
reach Pinocchio through the URDF text, so a derived mass that only moved the
budget would leave the torque curves describing a different robot.

| Slider | Default | What it sets |
|---|---|---|
| Baseline Ø | 120 mm | Outside diameter of the base link's tube |
| Distal taper | 0.50 | Tip diameter ÷ base diameter |
| Wall | 8 mm | Tube wall; ≥ radius means solid |
| Density | 2700 kg/m³ | Structural material (aluminium) |
| Per actuator | 0.30 kg | Collar mass for each mounted motor or gearbox |
| × actuator mass | 0.50 | Plus this much of the actuator's own mass |

**The defaults are a starting point, not a calibration.** `baseline_diameter`
is absolute, so one set of numbers cannot fit a 7 kg desktop arm and an 80 kg
palletizer of the same reach at once — on the bundled examples the default
lands between 0.36× and 1.55× the declared structure:

| arm | declared | derived at defaults | ratio |
|---|---|---|---|
| `simple_6dof` | 7.45 kg | 11.58 | 1.55× |
| `ur_style_6dof` | 12.95 kg | 13.92 | 1.07× |
| `wam_style_7dof` | 18.75 kg | 14.47 | 0.77× |
| `kr_style_6dof` | 32.90 kg | 18.07 | 0.55× |
| `palletizer_4dof` | 56.50 kg | 20.16 | 0.36× |

Tune per robot; the panel reports that ratio either way, so *declared* mode
still tells you what the geometry would give. This is a **sizing** model, not a
stress model: no load path, no buckling check, no joint housings, covers or
cabling, and the drawn `<visual>` geometry is left alone — the tube is a
structural idealisation of a link, not a claim about how it looks.

### Motor mass from the torque rating

A motor's mass is largely set by the torque it is rated for and by how it is
built, so the arm's mass can be an output of the drive sizing rather than a
fixed input. The **Motor mass** control switches between the `<mass>` in the
URDF and this fit, re-running the dynamics either way.

```
m = exp(a[form] + b·ln τ_peak)              m ∝ τ^0.697, scaled per form
```

Fitted by ordinary least squares over **76 motors** with manufacturer-listed
masses spanning **0.0034 to 98 N·m** (8.3 g to 11 kg) — CubeMars, Maxon,
Faulhaber, Kollmorgen, Teknic, ODrive, T-Motor and OEM parts — in the separate
`robot_arm_data` repository (`analyze_motor_mass.py`,
`data/bldc_motor_data.csv`). R²(log) 0.95, LOO R² 0.93, **LOO median relative
error 30%**. Read the relative figure: the set spans four decades of mass, so an
average in kilograms is dominated by the largest motors.

**Construction dominates.** `<motor>` takes a `form=` attribute — `frameless`
(the default), `outrunner`, `inrunner`, `flat`, `industrial`, `integrated` or
`hub` — and it moves the answer more than the rating does. At 1 N·m:

| form | mass | fitted over |
|---|---|---|
| outrunner | 172 g | 0.28–17 N·m |
| frameless | 197 g | 1.35–60 N·m |
| flat | 223 g | 0.085–9.9 N·m |
| inrunner | 299 g | 0.0034–2.65 N·m |
| hub | 402 g | 14.3–98 N·m |
| industrial | 582 g | 1.79–27.1 N·m |
| integrated | 651 g | 0.45–13 N·m |

An integrated servo-plus-driver is **3.8×** an outrunner at the same torque,
where doubling the rating costs only 1.62×. A single pooled fit was 4.4× biased
across forms (frameless 0.57×, integrated 2.52×) and carried a 50% median error;
splitting the intercept halves that.

Each form covers its own slice of the range and the slices barely overlap, so
the extrapolation check is **per form** — a 0.01 N·m `frameless` motor is
extrapolation even though 0.01 N·m is inside the dataset overall.

**There is no speed term.** The earlier fit had one with a coefficient near
zero; on the enlarged set adding it back *raises* leave-one-out error (0.40 kg
against 0.36 kg), because small motors here are also fast
(`corr(ln τ, ln ω) = −0.75`) and speed is largely a proxy for size once form is
known.

**No floor.** A power law runs to zero as torque does, where a real motor still
needs a housing, bearings and a connector. The lightest part in the set is
**8.3 g**; below a form's range the estimate keeps shrinking and only the
extrapolation warning tells you.

### Link mass from geometry

Link mass can likewise be an output rather than an input. The **Link mass**
panel in the analysis options switches between the file's `<inertial>` blocks
and a derived sizing, and carries sliders for the factors behind it. Each link
becomes **one tube plus a collar per mounted actuator**:

- **The tube** spans the link — from the joint attaching it to its parent (its
  own frame origin) to the joint carrying the next link, or to the tool tip on
  a tip link. That length comes out of the URDF kinematics; nothing invents it.
  Its diameter tapers down the chain, because a shoulder carries the whole arm
  and a wrist carries a gripper:

  ```
  d(link i of n) = baseline_diameter · taper^(i / (n−1))
  ```

  `taper = 1` is a constant-diameter arm; `taper = 0.4` gives a wrist tube 40%
  of the base. The wall is `wall` thick, and a wall at least as thick as the
  radius makes the link solid.
- **A collar per actuator**, for the flanges and bearing seats that carry a
  motor or gearbox: each lump mounted on the link adds
  `actuator_mass + actuator_fraction · lump.mass` at that lump's own position,
  taking the lump's shape — a cylinder around the mount point.
  `actuator_fraction` is there because a bigger actuator needs a bigger boss.

Mass follows from geometry and inertia with it: the tube's tensor is a hollow
cylinder, each collar's is its shape scaled to its mass, and they combine
through the parallel-axis theorem. The result replaces the link's `<inertial>`
**in the Pinocchio model as well as in the budget** — link inertias otherwise
reach Pinocchio through the URDF text, so a derived mass that only moved the
budget would leave the torque curves describing a different robot.

| Slider | Default | What it sets |
|---|---|---|
| Baseline Ø | 120 mm | Outside diameter of the base link's tube |
| Distal taper | 0.50 | Tip diameter ÷ base diameter |
| Wall | 8 mm | Tube wall; ≥ radius means solid |
| Density | 2700 kg/m³ | Structural material (aluminium) |
| Per actuator | 0.30 kg | Collar mass for each mounted motor or gearbox |
| × actuator mass | 0.50 | Plus this much of the actuator's own mass |

**The defaults are a starting point, not a calibration.** `baseline_diameter`
is absolute, so one set of numbers cannot fit a 7 kg desktop arm and an 80 kg
palletizer of the same reach at once — on the bundled examples the default
lands between 0.36× and 1.55× the declared structure:

| arm | declared | derived at defaults | ratio |
|---|---|---|---|
| `simple_6dof` | 7.45 kg | 11.58 | 1.55× |
| `ur_style_6dof` | 12.95 kg | 13.92 | 1.07× |
| `wam_style_7dof` | 18.75 kg | 14.47 | 0.77× |
| `kr_style_6dof` | 32.90 kg | 18.07 | 0.55× |
| `palletizer_4dof` | 56.50 kg | 20.16 | 0.36× |

Tune per robot; the panel reports that ratio either way, so *declared* mode
still tells you what the geometry would give. This is a **sizing** model, not a
stress model: no load path, no buckling check, no joint housings, covers or
cabling, and the drawn `<visual>` geometry is left alone — the tube is a
structural idealisation of a link, not a claim about how it looks.

### Motor mass from the torque rating

A motor's mass is largely set by the torque it is rated for, so the arm's mass
can be treated as an output of the drive sizing rather than a fixed input. The
**Motor mass** control in the analysis options switches between:

- **declared** — each motor weighs the `<mass>` in the URDF (the default);
- **from torque rating** — each motor weighs what an empirical BLDC power law
  predicts from its `peak_torque` (and `no_load_speed`), and the dynamics are
  re-run on that arm, so every torque curve reflects the change.

```
m = exp(a + b·ln τ_peak + c·ln ω_max)        m ∝ τ^0.82 · ω^-0.04
```

Fitted by ordinary least squares over 51 motors with manufacturer-listed
masses — CubeMars, Maxon, Faulhaber, Kollmorgen, Teknic, ODrive, T-Motor and
OEM frameless/outrunner/hub parts — in the separate `robot_arm_data`
repository (`analyze_motor_mass.py`, `data/bldc_motor_data.csv`). The constants
live in `src/arm_analyzer/motor_mass.py` rather than being refitted at import,
so this package stays self-contained; `web/motor_mass.js` mirrors them for the
inspector and a test runs the two against each other.

`b = 0.82` is the useful number: doubling a motor's torque rating costs about
**1.8×** its mass, not 2×. The speed exponent is near zero on this dataset, so
a motor that declares no speed costs little accuracy.

**It is a frameless-motor law.** The fit pools every construction in the
dataset, and they disagree: actual ÷ predicted runs about **0.57× for frameless**
(18 of 51 rows), 0.70–0.80× for outrunner/flat/inrunner, ~1.0× for hub, but
**1.8× for industrial** and **2.3× for integrated** servos. Most of the data is
at the frameless end, so that is where the law sits — an arm built from housed
industrial servos will come out roughly half its real motor mass. The example
arms here declare masses 1.6–3× the prediction, which is consistent with them
being specified as industrial parts. Splitting the fit by form factor belongs
in `robot_arm_data`, where the data is.

The example arms declare **no independent motor mass**: their generators set a
torque rating and derive the mass from it, so the two modes give the same answer
on them and there is no circular reference to resolve. Switching to
*from torque rating* only changes something for a URDF that declares a mass of
its own.

**Read it as a ranking, not a specification.** Leave-one-out error is **±0.66 kg**
per motor (R² 0.75 on the logs), which is enough to compare two layouts and not
enough to choose a part. It covers the **motor only** — gearboxes keep their
declared mass, since the dataset does not cover them. Ratings outside the
fitted range (0.15–98 N·m, 400–97 000 rpm) are flagged as extrapolation. When
the model is applied, a lump's inertia tensor is scaled by the mass ratio,
holding its shape and size fixed.

Note this is a one-way calculation: mass follows from the rating you set, not
from the torque the trajectory demands. Sizing a motor to its demanded torque
changes the arm's mass, which changes the demand — that loop is yours to close
by editing ratings and re-analysing.

**Modelling limits.** Losses are only the two efficiency placeholders (no
friction model). The rotor is a reflected inertia on its own axis; its
gyroscopic coupling with the host link is ignored. Transmissions are ideal
and decoupled: a belt routed through an intermediate joint doesn't make the
driven joint depend on that joint's angle. Only box, cylinder and sphere
geometry is drawn; mesh visuals are skipped.

## Layout

| Path | Role |
|---|---|
| `src/arm_analyzer/robot.py` | URDF + `<drive>` / `<drive_coupling>` / `<mimic>` parser, mass budget, drive envelopes |
| `src/arm_analyzer/pin_model.py` | URDF normalization + Pinocchio model with drive lumps and armature |
| `src/arm_analyzer/dynamics.py` | Torque terms, gravity, mass matrix via Pinocchio |
| `src/arm_analyzer/motor_mass.py` | Empirical BLDC mass-from-torque power law |
| `src/arm_analyzer/gearbox_mass.py` | Empirical reducer mass-from-rating power law, per type |
| `src/arm_analyzer/link_mass.py` | Link sizing: a tube per link plus a collar per mounted actuator |
| `src/arm_analyzer/trajectory.py` | Waypoint / CSV / JSON import → piecewise cubics |
| `src/arm_analyzer/cartesian.py` | Cartesian paths (ndcurves), minimum-jerk timing, IK (pink) |
| `src/arm_analyzer/analysis.py` | Joint-, gearbox- and motor-side series, utilizations |
| `src/arm_analyzer/profile.py` | Jerk-limited planner (from `modular_robot`) |
| `src/arm_analyzer/transforms.py`, `mass_properties.py` | Parsing helpers (rpy, primitive inertia) |
| `src/arm_analyzer/server.py` | FastAPI app (stateless; the browser sends URDF + trajectory text) |
| `web/` | Browser GUI (module map at the top of `main.js`); `urdf_doc.js` + `inspector.js` are the URDF editor, `traj_editor.js` the trajectory editor |
| `examples/robots/` | Example arms, see below |
| `examples/trajectories/` | `default_pick_place.json` and `workspace_tour.json` (cartesian, any robot), `pick_and_place.json` and `kr_pick_and_place.json` (joint waypoints), `sine_sweep.csv` (100 Hz samples) |
| `scripts/` | Generators for the example robots and trajectories (`pixi run examples`); `urdf_parts.py` holds the shared URDF writers |

### Example arms

| File | What it shows |
|---|---|
| `simple_6dof.urdf` | ~1 m test arm, primitive geometry, every drive co-located on its joint's parent link |
| `simple_6dof_remote_elbow.urdf` | The same arm with the elbow drive moved back onto the shoulder link, driving through a belt |
| `kr_style_6dof.urdf` | KUKA KR-series-style layout at KR 6 R900 (KR AGILUS) scale, ~46 kg: the A4–A6 motors sit at the rear of the arm housing behind the elbow and reach their gear units (front of the housing, wrist, flange) through shafts in the forearm. Use it with `kr_pick_and_place.json` (zero pose: upper arm vertical, forearm horizontal). Geometry is approximated from public specs; drive ratings are estimates, not KUKA data. |
| `wam_style_7dof.urdf` | Barrett WAM-style cable arm, 7 axes, ~24 kg: M1–M3 sit **in the base**, so 9 kg of the robot is carried by the floor and not by any joint. J2/J3 and J5/J6 are driven through **differentials** (`<drive_coupling>`), which is the layout to look at if you want to see two motors sharing one heavy axis. Its ±90° wrist cannot reach the tool-out and tool-up poses of `workspace_tour.json`. |
| `ur_style_6dof.urdf` | UR5e-style collaborative arm, ~19 kg, 5 kg payload: an offset wrist and a self-contained module at **every** joint, so the whole drivetrain rides on the arm. The baseline the two remote-drive layouts above are worth comparing against. |
| `palletizer_4dof.urdf` | Palletizer, ~73 kg, 20 kg payload: J1–J3's motors all sit on the pedestal and push the arm through rods, and **two parallelogram linkages** (`<mimic>`) keep the tool plate level, so there is no wrist pitch and only 4 actuated axes. It can follow any tool-down path, and nothing else. |

Geometry for the last four is approximated from public specs; drive ratings are
estimates, not manufacturer data. **Every example motor's mass is derived from its
own `peak_torque` and `form`, and every gearbox's from its own `rated_torque`
and type**, so no example declares a mass that
contradicts its rating — change a rating in a generator and `pixi run examples`
re-derives the mass. Each motor's cylinder is then **resized to hold that
mass** at 3063 kg/m³, scaling radius and length together so the motor keeps its
proportions — so in the generators `radius` and `length` set a motor's shape,
not its size. That density is the median of the 27 motors in
`bldc_motor_data.csv` that list both OD and length (IQR 2456–3632): a motor is
copper, laminated steel, magnets and an aluminium housing around an air gap and
a bore, so its bulk density sits between aluminium and steel. Sizing matters
beyond looks — a lump with no `<inertia>` takes its tensor from its geometry
scaled to its mass, so an oversized envelope would hand the motor an inertia it
does not have.

Each example declares the `form=` its real counterpart uses — `industrial` for
the KUKA and palletizer, `integrated` for the UR, `frameless` for the WAM,
`outrunner` for the small test arm — which is what brings the KR and WAM within
a couple of percent of their datasheets. Every transmission is modelled as
ideal and decoupled unless a `<drive_coupling>` says otherwise: on a real
in-line wrist like the KR's, A4 rotation also turns the A5 and A6 shafts, so
those motor angles are coupled in a way this file does not describe.
| `tests/` | pytest: hand-calculated torques, an energy balance, RNEA/CRBA consistency, JS-vs-Pinocchio kinematics, and JS-vs-Python for the differential region and the motor-mass model |

### Validation against the real robots

Three examples imitate machines with published datasheets, so the models can be
checked rather than trusted. `tests/reference_specs.py` holds the published
numbers and their sources; `tests/test_reference_specs.py` asserts against
them, with tolerances set at **where the models actually are** so that a change
which drifts further fails loudly. Retrieved 2026-09-20.

| | model | real | |
|---|---|---|---|
| **KUKA KR 6 R900 sixx** — mass | 52.70 kg | 52 kg | ✅ **+1%** |
| reach (flange, geometric) | 961 mm | 901 mm rated | envelope vs geometry |
| joint range widths | 340/235/276/370/240/700° | identical | ✅ exact |
| **Barrett WAM 7-DOF** — mass | 26.82 kg | 27.4 kg | ✅ **−2%** |
| reach (to plate) | 915 mm | 910 mm | ✅ +0.5% |
| link lengths | 346 / 550 / 45 / 300 / 60 mm | identical | ✅ exact |
| joint ranges | 7 axes | 6 of 7 exact, J5 rounded | ✅ |
| **UR5e** — mass | 30.95 kg | 20.6 kg | ❌ **+50%** |
| reach (flange, geometric) | 925 mm | 850 mm rated | envelope vs geometry |
| link lengths | d1 162.5, a2 425, a3 392.2, d4 133.3, d5 99.7, d6 99.6 mm | identical | ✅ exact |
| joint ranges | ±360° (elbow ±160°) | ±360° all | elbow deliberately tighter |
| joint speeds | 180°/s inner, **360°/s wrist** | **180°/s all six** | ❌ wrist 2× too fast |

What the comparison says, with motors sized by rating **and construction** and
gearboxes by rating:

- **Geometry is solid.** The UR5e and WAM link lengths reproduce the published
  DH parameters exactly, and all three arms' joint travel matches. The KR's
  limits are offset from KUKA's because this example uses a different zero
  pose, but every *range width* agrees, so the test compares widths.
- **Mass now validates.** The KR lands at **+1%** and the WAM at **−2%**. Two
  changes got there: deriving gearbox mass from its rated output torque, which
  raised it above the hand-set values, and giving each motor its real `form`
  — the KUKA's housed industrial servos are 2.9× a frameless motor of the same
  rating, which is most of the gap the earlier pooled fit left open.
- **The UR5e is the exception, and it is a fault in that example.** Its
  gearboxes declare `rated_torque = 150 N·m` on the inner joints — the UR5e's
  *peak* joint torque, not a reducer's continuous rating. The giveaway is
  `peak_torque / rated_torque = 1.40`, where the KR and WAM sit at 2.3. The
  mass law reads `rated_torque` as a frame rating, sizes those three joints as
  3.5 kg reducers, and the drives end up 58% of the whole arm.
  `tests/test_reference_specs.py` marks this `xfail(strict=True)` under
  `OVERWEIGHT_BY_RATING`, so it will flag the moment the ratings are corrected
  in `scripts/make_ur_style.py`.

Reach is quoted differently by different manufacturers — to the flange, to the
mounting plate, or as a rated working envelope — so the reference records which
frame each published figure refers to and the test measures that frame. Only
the WAM's is a like-for-like comparison; UR's and KUKA's rated envelopes are
smaller than the geometric maximum and are bounded rather than matched.

### HTTP API

| Endpoint | Purpose |
|---|---|
| `GET /api/robots`, `/api/robots/{file}` | Example URDFs |
| `GET /api/trajectories`, `/api/trajectories/{file}` | Example trajectories |
| `POST /api/robot/model` `{urdf}` | Pose-independent model for the viewer |
| `POST /api/trajectory/cartesian/preview` `{trajectory, urdf?}` | Path and timing for the editor; per-waypoint IK only when `urdf` is given |
| `POST /api/analyze` `{urdf, trajectory, trajectory_kind?, units?, smoothing?, rate_hz?, gravity?, payload?, efficiency?, motor_mass?, link_mass?}` | Plan (incl. IK for cartesian paths) + series + summaries. `motor_mass` and `gearbox_mass` are `"declared"` (default) or `"model"`; `link_mass` is `{mode, baseline_diameter, taper, wall, density, actuator_mass, actuator_fraction}` with `mode` `"declared"` (default) or `"derived"`. The result's `motor_mass`, `gearbox_mass` and `link_mass` blocks report declared and derived masses either way |

The server only reads from `examples/`, never writes, and accepts CORS only from
localhost. It is a local tool; don't expose it.
