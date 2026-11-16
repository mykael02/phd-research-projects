# MuJoCo balancing robot starter

This lab runs a free-body robot with two driven wheels and native floor contact.
It is a separate environment from the publication tools. Michael Agbo's existing
planar experiments and reports are unchanged.

## Open the simulation

Double-click **Launch MuJoCo.cmd**. The robot starts paused at a 5-degree tilt.
Click the viewer to give it keyboard focus.

| Key | Action |
| --- | --- |
| Space | Run / pause |
| J | Apply a 3 N push in world -X for 0.2 seconds |
| L | Apply a 3 N push in world +X for 0.2 seconds |
| R | Reset and pause |

Drag the mouse to orbit the camera; use the wheel to zoom. Pushes follow the
ground X axis, independent of the camera. A new push replaces the current pulse.
A push entered while paused starts when simulation resumes. The simulation
pauses at a 45-degree pitch or roll; press R to reset.

## Reproduce the installation check

From the project root in PowerShell:

```powershell
mujoco_lab/.venv/Scripts/python.exe mujoco_lab/run.py --smoke
mujoco_lab/.venv/Scripts/python.exe -m pip check
```

The smoke check loads the model, checks mass and degrees of freedom, renders two
1280 by 900 images, simulates up to 10 seconds with a push at 2 seconds, and checks
finite states, contacts and engine warnings. Its measurements are saved in
`output/setup_check.json`; images are `output/preview.png` and
`output/after_smoke.png`. A smoke check establishes that the installation works;
it is not a controller validation study.

To recreate the environment on another Windows computer with Python 3.14:

```powershell
powershell -ExecutionPolicy Bypass -File mujoco_lab/setup.ps1
```

`requirements.txt` pins the installed dependencies. The environment itself is
excluded from the research ZIP. The launcher requires a working graphics driver
and desktop session.

## Model scope and assumptions

The starter uses the nominal LQR gains from `../config.json`, a 200 Hz control
loop, a 1 ms physics step, and a total motor torque limit of 0.70 N m split equally
between the wheels. Feedback is ideal. External pushes act at the torso COM.

The original body mass (1.2 kg), COM height above the axle (0.18 m), pitch inertia
(0.012 kg m2), wheel radius (0.045 m), and wheel spin inertia (0.00012 kg m2 each)
are retained. The original 0.35 kg base mass is split into a 0.11 kg chassis and
two 0.12 kg wheels. Total mass is 1.55 kg.

The chassis inertia, wheel transverse inertia, torso roll/yaw inertia, wheel
spacing, visual geometry, and contact/friction parameters are starter assumptions
specified in `balancing_robot.xml`. Unlike the planar model, this model permits
roll, yaw, vertical motion and wheel slip. Contact compliance replaces ideal
rolling constraints. It has no payload or sensor noise yet. Body appearance does
not infer mass properties: explicit inertial values are used.

The comparison below now checks conventions, ideal-rolling equations, trajectories
and contact/step-size sensitivity. Physical calibration and a fall-boundary study
remain outstanding. The viewer still uses the original 1 ms starter configuration;
the research runner separately evaluates finer steps and contact variants.

## Completed nominal comparison

Open **View validation report.cmd**, or the
[HTML report](../results/mujoco/report.html) and
[PDF supplement](../publication/output/pdf/mujoco_report.pdf).

- 48 state/input combinations verify accelerations and motor-force conventions.
- 15 ideal-rolling trajectories agree with the original equations within the
  declared 1e-7 componentwise tolerance, in each coordinate's native units.
- 150 study integrations cover original RK4, reduced MuJoCo, native wheel contact,
  three physics steps, and contact-parameter variations.
- At 0.25 ms, all 15 nominal native-contact cases recover, matching the original.
  Maximum differences are 0.89 mm position and 0.114 degrees pitch.
- All nominal cases recover; the study does not establish fall-boundary agreement.
  Lower sliding friction (0.5) causes 5/15 cases to fail the trajectory screen at
  0.5 ms even though all recover. No gains or contact parameters were fitted.

Reproduce the complete study and report using the existing two environments:

```powershell
powershell -ExecutionPolicy Bypass -File mujoco_lab/run_validation.ps1
```

The roughly nine-minute study on this computer writes `results/mujoco/study.json`
and all 150 sampled trajectories in `trajectories.npz`. Column names and units are
in the JSON metadata. `validation_protocol.json` records the grid and criteria
fixed before execution. Research source hashes are checked during report creation.
`check_reference.py` also checks one trajectory and recovery time against the
existing `balance.simulate` entry point.

The reduced model uses a slide/hinge assembly with reflected wheel inertia and
generalized motor forces. It independently checks the same ideal equations; it
is not a wheel-contact or hardware model. The native model uses the physical
wheels in `balancing_robot.xml`. Its rolling residual includes contact-free wheel
spin; the report distinguishes that from samples with active contact. The native
runner refreshes derived states each physics step for feedback, drag and telemetry.

## Fall-boundary and payload extension

Open **View boundary and payload report.cmd** or the
[combined report](../results/mujoco_extension/report.html).
The [PDF](../publication/output/pdf/mujoco_boundary_payload_report.pdf) includes
eight figures and three native 3D payload illustrations.

The coarse 88-condition grid agrees between engines. A separately frozen,
exploratory zoom of every observed transition interval finds two persistent
disagreements: +/-18.2 N at a 0.5 N m torque limit recovers in the planar model
but falls in MuJoCo. No outcome changes occur as the physics step is refined to
0.0625 ms for those cases. The remaining 86 zoom conditions agree. These are
finite sampled intervals, not exact critical forces or hardware thresholds.

The payload extension tests all nine prior mass/height combinations at four
force/torque conditions, with fixed and exact-load LQR, ideal sensing and an
unloaded control. Both engines agree on all 36 loaded cases for each controller:
19 recoveries and 17 falls. Redesign does not change the fall labels on this grid.
It lowers median peak tilt by about 0.81 degrees on the 19 common survivors;
the report also gives paired control-effort differences.

```powershell
powershell -ExecutionPolicy Bypass -File mujoco_lab/run_extension_study.ps1
```

The full sequence takes roughly 20 minutes of simulation on this computer,
plus report generation. It uses two worker processes and source-keyed local
checkpoints. Checkpoints and environments are excluded from the research ZIP.
Data and protocol files are retained separately:

- `extension_protocol.json`: original boundary and payload grids, audit rules.
- `boundary_zoom_protocol.json`: exploratory follow-up fixed after coarse results.
- `results/mujoco_boundary`: 352 integrations on 88 coarse conditions.
- `results/mujoco_boundary_zoom`: 356 integrations on 88 zoom conditions,
  including finer checks of the two disagreements; 16 conditions repeat coarse endpoints.
- `results/mujoco_payload`: 340 integrations, including 20 ideal-rolling checks.
- `results/mujoco_extension`: combined report, 3D renders and data checks.

Every study stores raw 200 Hz trajectories, units, outcomes and source hashes.
The primary payload controller comparison uses a common 0.25 ms physics step;
finer audits are reported separately. No gains, contact coefficients or inertias
were fitted to make the engines agree. The native chassis inertia remains additional.
Startup/chassis-inertia ablations and measured hardware parameters are still needed
to explain physical accuracy. The noisy sensing study has since been extended to
MuJoCo (below); the load-estimate error variants have not.

[Official MuJoCo Python documentation](https://mujoco.readthedocs.io/en/stable/python.html)

## Follow-up: boundary diagnostics and noisy payload repeat

```powershell
powershell -ExecutionPolicy Bypass -File mujoco_lab/run_followup_study.ps1 -Workers 8
```

- `disagreement_protocol.json`, `run_disagreement.py`: 25 models (two references,
  22 single-assumption variants, one combined), bisection of the critical push.
  Output in `results/mujoco_disagreement/study.json` (488 integrations).
- `run_disagreement_addendum.py`: post hoc check at 0.3 and 0.7 N m (168).
- `noisy_payload_protocol.json`, `run_noisy_payload.py`: Python port of the
  JavaScript sensing pipeline, verified on all 3,240 archived planar records, then
  run on the contact model at 0.25 and 0.5 ms. Output in `results/mujoco_noisy_payload`.
- `run_noisy_controls.py`: post hoc exact-feedback control, repeatability check and
  rule-selected illustration (`controls.json`, `illustration.npz`).
- `run_exact_boundary.py`: post hoc exact-feedback critical push of both gains at all
  18 noisy conditions in all three plants (`exact_boundary.json`, 2,988 integrations).
- `hostperf.py`: opts worker processes out of Windows power throttling.

All scripts record source hashes, and the report builders refuse to run if a hashed
source has changed since the data were produced.
