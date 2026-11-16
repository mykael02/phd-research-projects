# Balancing robot — Balance Lab

A first runnable balancing-robot project: nonlinear planar dynamics, a PID-based
controller, a designed LQR, reproducible experiments, and an interactive browser
simulation. This is a research starter, not validated hardware control software.

## Open the animation

New: [live 3D simulation and replay](visualisation/index.html),
[3D experiment map](visualisation/experiments.html), and
[animated preview](visualisation/robot-3d-preview.gif).
The [research package](publication/README.md) contains the journal-neutral
manuscript under Michael Agbo's name, a detailed report, 16 print-quality figures,
editable LaTeX, analysis data and reproduction scripts. The 3D view displays
planar dynamics; yaw, lateral balance and steering are not modelled.

In 3D, click **Run simulation**, then **Push left** or **Push right**.
The force slider sets a 0.2-second pulse on both upright robots. Pause/resume,
reset, initial tilt, torque limit and simulation speed are available. Choose
**Recorded experiments** to return to the saved study replays.

Double-click **index.html** in this folder. No server, installation, or internet
connection is needed for the browser simulation. Keep `config.js`, `model.js`,
`app.js`, and `style.css` alongside the page.

1. Click **Run simulation**.
2. Click **Push left** or **Push right** to apply the same force to both robots.
3. Compare tilt, displacement, motor torque, and the time trace.
4. Increase push strength or reduce the torque limit to explore failure.
5. Click **Reset** for a fresh run. Initial tilt and torque-limit changes reset
   automatically; push-strength changes affect subsequent pushes.

The two panes are side views; one wheel represents the coaxial wheel pair. The
camera follows each axle. Ground labels are in metres relative to its starting
point. Runs stop at 20 simulated seconds or when both robots have fallen. A robot
that falls stops individually at the fall threshold. Repeated push clicks replace
the active force and restart its 0.2-second duration; they do not add impulses.
The tab pauses when hidden, and slow rendering slows simulated time rather than
skipping physics steps. The browser does not automatically save interactive runs.

## Robustness study — step 2 is built

Open **results/robustness/report.html**, or follow the robustness-study link on
the simulation page. Choose a motor torque limit, inspect the PID and LQR maps,
then select a cell to see its metrics and replay that condition. Replay links
load both robots with the same tilt and torque limit, schedule the signed push
at 2.0 seconds, and stop at 10 seconds. Click **Run simulation** to begin; avoid
manual pushes or parameter changes if reproducing the recorded trial.

Run the full study from this folder:

```powershell
python run_robustness.py
node tests/test_study_ui.js
```

There are **1,638 trials**, with both controllers evaluated on every combination:

- Initial tilt: -20 to +20 degrees, in 5-degree increments (9 values).
- Horizontal push: -30 to +30 N, in 5 N increments (13 values), applied for
  0.2 seconds from t = 2.0 s. Negative means left; zero means no push.
- Combined motor torque limit: 0.1, 0.2, 0.3, 0.4, 0.5, 0.7, and 1.0 N m.

All other body parameters, controller gains, exact state feedback, and control
timing stay fixed. Each trial runs for 10 seconds or until it falls. No-push
trials use time zero as their recovery reference; other trials use t = 2.2 s.
Outcomes are **recovered**, **upright but unsettled**, or **fallen**, using the
existing recovery and fall definitions below. Falls before the push are marked
separately; they are not evidence of surviving or failing an applied push.

Default study results:

| Controller | Recovered | Upright, unsettled | Fallen | Total |
| --- | ---: | ---: | ---: | ---: |
| PID | 215 | 220 | 384 | 819 |
| LQR | 357 | 0 | 462 | 819 |

PID remained upright in more sampled conditions, but many did not return to the
settling bounds. LQR recovered in more conditions and also fell in more. These
counts reflect this particular parameter grid; they are not success probabilities
or a general controller ranking. No limit on horizontal travel is imposed.

Maps show tested points, not continuous guaranteed operating regions. Adjacent
different outcomes bracket a change at the grid resolution; they do not prove
monotonicity or locate an exact boundary. The runner finds neighboring outcome
changes along all three axes and audits 32 deterministically selected cells with
two RK4 physics substeps per control step. The controller and fall-detection rate
remain 200 Hz. In this run, all 32 classifications agreed. This sampled check is
not a complete convergence proof.

Outputs are a standalone `report.html`, per-trial `results.csv`, and `results.json`
containing metrics, axes, definitions, configuration snapshot, source hashes, and
integration-audit results. Trajectories are regenerated by replay rather than
stored for all 1,638 runs. Use `--output PATH` to preserve another run. Replays use
the current local model/configuration: rerun the study after changing either.

The Node study test checks report selection and replay URLs and verifies five
scheduled replay conditions against saved Python final states for both controllers.
It uses DOM/canvas mocks, so it does not verify visual layout in a real browser.

## Reproducible experiments

From this folder:

```powershell
python run_experiments.py
python -m unittest discover -s tests -v
node tests/test_ui.js
```

The experiment runner uses Python's standard library and the saved `config.json`.
It compares PID, LQR, and uncontrolled motion for five cases: initial tilt, pushes
in both directions, a large push, and weak motors. It writes `results/summary.csv`,
`results/summary.json`, and full state/control histories in
`results/trajectories.json`. Rerunning replaces these outputs; use `--output PATH`
to keep a separate experiment. Each scripted run lasts up to 10 seconds, with a
push scheduled at 2.0–2.2 seconds. All default settings are deterministic.

For controller design and the numerical test suite, NumPy and SciPy are required:

```powershell
python -m pip install -r requirements.txt
python design_controller.py
```

The design command regenerates both `config.json` and `config.js` from the same
parameters. Do not edit only one config file. The cross-runtime test also uses
Node.js, and explicitly skips that check if Node is unavailable.

## Physical model

State is `[x, v, theta, omega]`: axle position and velocity, forward body tilt
from upright, and angular velocity. SI units throughout. Positive x and tilt
are rightwards in the side view. Motor command `tau` is the **combined torque
of the two wheel motors**, with equal and opposite reaction on the body.

Parameters: wheel/base translation mass M = 0.35 kg; body mass m = 1.2 kg;
axle-to-body centre-of-mass distance l = 0.18 m; body inertia about its centre
I = 0.012 kg m²; each wheel's axle inertia Iw = 0.00012 kg m²; wheel radius
r = 0.045 m; gravity g = 9.81 m/s². Default combined torque limit is 0.7 N m.
Linear translation and rotation drag coefficients are 0.08 N s/m and
0.002 N m s/rad respectively. These are illustrative parameters, not measurements.

The Lagrangian gives the coupled acceleration equations:

```text
A = M + m + 2 Iw / r²
B = m l cos(theta)
D = I + m l²

A x_ddot + B theta_ddot = tau/r - bx v + m l sin(theta) omega² + F
B x_ddot + D theta_ddot = m g l sin(theta) - tau - bt omega + F l cos(theta)
```

F is an external horizontal force at the body's centre of mass. The equations
include wheel rotational inertia and motor reaction torque; this is not merely
a cart-pole with a robot drawing. Assumptions: rigid body, planar motion, ideal
rolling, unlimited ground traction, no motor electrical dynamics, and exact
state measurements in the original baseline. The live 3D lab now adds Step 3
measurement noise/delay and Step 4 rigid payload aggregation. Yaw, wheel slip and
motor electrical dynamics remain unmodelled. Body appearance is schematic.

Control is held constant over each 0.005-second step (200 Hz). Physics uses RK4.
A fall is declared when |theta| reaches 45 degrees at a step endpoint. Impact
and subsequent ground contact are not simulated. Browser and Python implement
the same model, and their final states are checked against each other.

## Controllers

**PID:** tilt proportional, integral, and angular-rate feedback, plus position
and velocity feedback:

```text
tau_raw = 14 theta + 0.4 integral(theta dt) + 2 omega + 2 x + 3 v
```

The integral is clamped to ±0.25 rad s, with conditional integration to prevent
windup under saturation. Position feedback signs follow this motor/body model's
coordinate convention. This is a manually tuned PID-based baseline, not a
pure tilt-only PID and not an optimized or universally robust controller. The
tilt integrator can leave a small position offset after a disturbance.

**LQR:** numerical linearization about upright, continuous-time Riccati design,
Q = diag(12, 2, 120, 2), R = 1, and `tau_raw = -K state`. The generated gains
are stored in config. Tests verify both continuous design stability and local
sampled-data stability at 200 Hz. Actual simulation is nonlinear and saturated;
local LQR stability does not guarantee recovery from arbitrary pushes.

Both controllers use the same torque limit. Altering that limit does not redesign
the gains. No noise is injected, so derivatives come from the true simulated state.

## Metrics and initial findings

Recovery time is measured after the scheduled push ends (or from time zero in
the no-push case). A settled robot must remain within **1° tilt, 0.03 m position,
and 0.03 m/s speed**, with at least one second of evidence at the end of the run.
The reported time is the first step after the final threshold violation; it is
an offline finite-run measure. `null` means it did not meet this definition.

Also report peak absolute tilt, maximum displacement from the origin, final
position and tilt, saturation steps, and integral of squared torque. Squared
torque integral is an effort proxy, not physical energy consumption. The terminal
trajectory row has torque and push set to zero as an end-of-run marker.

On the default 3 N rightward push, the current PID recovered in 3.51 s and LQR
in 2.78 s. PID had lower peak tilt (5.17° versus 6.16°), while LQR had lower maximum
displacement (0.187 m versus 0.224 m). Both fell under the 30 N push and the weak
motor case. These illustrative cases are not evidence of general superiority.

## Verification and research next steps

The test suite checks equilibrium, mechanical power balance, unforced energy
conservation, numerical convergence, invalid parameters, LQR sample stability,
recovery, falls, anti-windup, and Python/JavaScript agreement. The separate Node
test checks UI control logic using a mock canvas; it does not verify pixels or
browser layout. Live browser inspection was unavailable during initial creation.

**Step 3, implemented — imperfect measurements.** The live 3D page now includes
perfect-state, direct noisy sensing and Kalman-filtered sensing modes. Angle and
gyro noise, packet delay and seed are configurable. Purple body outlines and
four selectable state plots compare truth with delivered measurements and the
controller estimate. Run export preserves the full history and force events.
The original 2D interface remains the perfect-state baseline.

The [Step 3 report](results/sensing/report.html) documents 8,340 unique sensing
trials, six ideal baselines, 30 seeds per noisy condition and five new scientific
figures. [PDF supplement](publication/output/pdf/sensing_report.pdf).
Run `node run_sensing.js`, then
`publication/.venv/Scripts/python.exe publication/build_sensing_report.py`
to reproduce it. The fixed protocol is in `sensing_protocol.json`.
The filter estimates attitude at the packet timestamp; it does not compensate
delay. Position and velocity channels are ideal but delayed. This is not yet a
physical IMU/accelerometer model. Details are in [SENSING.md](SENSING.md).

**Step 4, implemented - payload variation.** The live 3D lab now has added-mass
and mounting-height controls. A rigid centred cuboid changes mass, COM and pitch
inertia together, with the original controller gains frozen. The push acts at
the combined loaded-body COM. See [PAYLOAD.md](PAYLOAD.md) for assumptions.

The [Step 4 report](results/payload/report.html) contains 4,176 trials, seven new
figures and a separate refinement audit of all 2,016 deterministic trials.
[PDF supplement](publication/output/pdf/payload_report.pdf).
Run `node run_payload.js`, `node run_payload_audit.js`, then
`publication/.venv/Scripts/python.exe publication/build_payload_report.py`.
The fixed protocol is in `payload_protocol.json`.

The original manuscript and Step 3 results remain separate archived studies.
A fixed-cost payload-aware comparison is now implemented (see below). Further tuning,
fresh held-out conditions, hardware validation,
motor electrical response and wheel slip remain future work. The implemented load
is fixed at initialisation; online pickup and off-axis loads require additional
physics. Adding payload variation alone does not establish research novelty.

Background references (not a claim that this implementation reproduces either):

- [Albert et al., wheeled inverted pendulum trajectory planning and control](https://arxiv.org/abs/1811.12819)
- [SciPy continuous algebraic Riccati equation solver](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_continuous_are.html)

## Payload-aware LQR comparison

The [live 3D lab](visualisation/index.html) now defaults to **Fixed vs payload-aware
LQR**. Add a payload to compare nominal and redesigned gains. Expand the estimate
controls to test incorrect assumed mass or height. The original PID/LQR view is
still selectable. See [COMPARISON.md](COMPARISON.md).

The [comparison report](results/comparison/report.html) contains 16 development
checks, 11,016 reserved test runs and six figures. Gains use the same original Q/R,
with no performance-based parameter search. A half-step audit and separate
component-matrix/DOP853 check support numerical correctness. Full multibody
validation of the sensing study and hardware validation remain outstanding;
the focused ideal-sensing MuJoCo extension is linked below.

## MuJoCo starter

The [MuJoCo lab](mujoco_lab/README.md) adds a native wheel/contact model in its own
Python environment. Open [Launch MuJoCo.cmd](mujoco_lab/Launch%20MuJoCo.cmd), then
press Space to run, J/L to push, or R to reset. It starts paused. This installation
and model smoke check is preparation for independent validation; its contact and
three-dimensional inertial assumptions have not yet been calibrated.

The [MuJoCo comparison report](results/mujoco/report.html) now contains 150
deterministic integrations and six figures. A separately built
ideal-rolling MuJoCo model passes all 15 trajectory checks. At 0.25 ms physics
resolution, the native-contact model agrees with the original recovery labels
in all 15 nominal cases, with maximum differences of 0.89 mm and 0.114 degrees.
That first grid contains only recoveries; the follow-up below tests fall boundaries.
Reducing sliding friction to 0.5 preserves recovery but fails the declared
trajectory screen in 5 of 15 cases at 0.5 ms. See the
[PDF supplement](publication/output/pdf/mujoco_report.pdf) for methods and limits.
This is simulation verification and discrepancy analysis, not hardware validation.

## Fall boundaries and loaded MuJoCo comparison

The [boundary and payload report](results/mujoco_extension/report.html) adds
1,048 integrations, eight figures and native 3D payload views.

- The coarse boundary grid agrees in all 88 conditions (32 recoveries, 56 falls).
- An exploratory 0.05 N / 0.05 degree zoom agrees in 86 of 88 conditions. At
  +/-18.2 N with a 0.5 N m limit, the planar model recovers and MuJoCo falls.
  These two mirror cases retain their outcomes at 0.0625 ms physics resolution.
- Both engines agree for all 36 loaded conditions under each controller. Fixed
  and payload-aware LQR each give 19 recoveries and 17 falls in this ideal-sensing
  subset. There is no fall-count improvement on this grid.
- Among the 19 four-way survivor cases, redesign lowers median peak tilt by
  about 0.81 degrees and modestly lowers the squared-torque measure in both engines.

The nine prior payload mass/height combinations are represented as welded
MuJoCo bodies with matched combined COM and pitch inertia. The comparison also
includes an unloaded control, 20 reduced-model trajectory checks, 480 distinct
acceleration checks, and 32 bitwise-repeatable boundary endpoint trajectories.
The exploratory zoom is outcome-selected and shares endpoints with the coarse
grid; its cases are not independent statistical samples.

[PDF supplement](publication/output/pdf/mujoco_boundary_payload_report.pdf).
Run `powershell -ExecutionPolicy Bypass -File mujoco_lab/run_extension_study.ps1`
to reproduce the studies, data audit, 3D renders and report in order. The original
nominal viewer remains available; the new work is a separate research comparison.

## Boundary diagnostics, noisy MuJoCo repeat and manuscript version 2

The [follow-up supplement](results/mujoco_followup/report.html)
([PDF](publication/output/pdf/mujoco_followup_report.pdf)) and the
[integrated manuscript, version 2](results/manuscript_v2/manuscript.html)
([PDF](publication/output/pdf/manuscript_v2.pdf)) add four studies. Version 1 of
the manuscript and all earlier datasets are unchanged.

- Boundary disagreement (656 integrations): planar 18.2271 N against contact
  18.1887 N at 0.5 N m. The contact model's extra 0.0002 kg m² of chassis pitch
  inertia lowers its boundary by 44.6 mN. With it matched, a residual of 3 to 6 mN
  in the opposite direction remains at all three torque limits. Settling has no
  effect; contact settings move the boundary by at most 14 mN unless friction is 0.5.
- Noisy payload repeat (9,720 trials): the fall reduction persists in MuJoCo,
  508 to 334 (planar) and 511 to 343 (contact, 0.25 ms) of 1,620 paired trials.
  Every fall is at 0.3 N m.
- Exact-feedback control (108 runs, post hoc): the redesign changes the fall label
  in 2 of 18 conditions.
- Exact-feedback boundary (2,988 integrations, post hoc, added after an internal
  review): the redesign raises the critical push by 4-9% at 0.26 m and 10-21% at
  0.36 m and changes it by at most 1% at 0.14 m, in both models. Each gain's noisy
  fall count follows its distance to its own boundary. Noise reveals a boundary
  shift; counted per controller, noise-caused falls are not reduced (241 against 254).

The manuscript's hardware plan measures the boundary shift directly with
staircases (228 trials) before any noise-injection stage. Run
`mujoco_lab/run_followup_study.ps1` to repeat everything, then
`publication/verify_followup.py` to check hashes, links and PDF text.
