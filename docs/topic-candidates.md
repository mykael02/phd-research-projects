# Provisional robotics topic candidates

> **Status: provisional.** These candidates were written from general
> background knowledge. **No live literature search has been done.** Each
> "gap" below is a hypothesis to check, not an established open problem. No
> papers, authors or labs are cited. The search is tracked as OUTSTANDING in
> [literature-search-todo.md](literature-search-todo.md), with one check for
> each hypothesis ID (H1-H6).

Use [scoring-rubric.md](scoring-rubric.md) to compare candidates **after** the
hypotheses have been checked.

---

## T1. Navigation under localization uncertainty

**Problem statement.** Mobile robots usually plan as if their pose estimate
were exact, then rely on feedback to absorb the error. In narrow passages,
featureless corridors or under sensor degradation, the estimate can be
uncertain enough that a nominally collision-free plan is unsafe or fails. The
question is how to plan paths that trade path length against the chance of
staying well localized and collision-free.

**Gap - Hypothesis - unverified, requires literature check (H1).** Belief-space
or uncertainty-aware planners may still be too costly or too conservative for
real-time use on low-cost platforms. A lightweight method that adds a
localizability cost to a standard grid or sampling planner, with a measurable
effect on collision rate, may not yet be well characterized.

**Required skills.** Probabilistic state estimation (Kalman and particle
filters), graph search and sampling-based planning, Python/C++, basic ROS or
an equivalent middleware, experimental statistics.

**First small experiment.** Extend `projects/nav-baseline` with a per-cell
"localizability" penalty (for example, low in open areas with no nearby
landmarks). Compare plain A* with penalized A* on the seeded maps, reporting
path length and a simulated pose-error proxy along each path. All of this can
be done in the standard library on a laptop.

**Compute / hardware.** Laptop for the first stage. Later: a 2D simulator, then
a small differential-drive robot with a lidar or camera.

**Risks.** The area may be crowded, so verifying the gap matters most here (H1).
Simulated uncertainty models may not transfer to real sensors, and real-robot
evaluation takes a lot of time.

---

## T2. Learned traversability for off-road or unstructured terrain

**Problem statement.** Outdoor robots must decide which terrain is safe to
drive over (grass, mud, gravel, slopes) from onboard sensing. Hand-built cost
maps generalize poorly, while learned ones need labels and can be
overconfident on unfamiliar terrain.

**Gap - Hypothesis - unverified, requires literature check (H2).** Learned
traversability estimators may lack calibrated uncertainty, and there may be no
standard way to measure how planner safety degrades when the terrain is out of
distribution. Self-supervised labels from the robot's own driving experience
may be under-used for calibration specifically.

**Required skills.** Deep learning (PyTorch or similar), computer vision and
point clouds, mapping, uncertainty calibration, field-robotics practice.

**First small experiment.** On synthetic grids, give each cell a "true"
traversal cost and a noisy predicted cost with a stated confidence. Plan on the
predicted map, execute on the true one, and measure the gap between expected
and realized cost as the noise grows. This can reuse the grid planner with a
weighted-cost extension.

**Compute / hardware.** A GPU is needed for any real perception work. Field
data needs a rugged platform or an existing dataset (dataset availability is
part of H2).

**Risks.** Data collection is expensive and seasonal, results may be
site-specific, and the work depends on access to hardware or data.

---

## T3. Multi-robot path planning robust to execution delays

**Problem statement.** Multi-agent path finding gives collision-free plans for
many robots on a shared graph, but real robots slow down, stop and drift. Plans
that assume perfect timing can deadlock or collide once execution deviates.

**Gap - Hypothesis - unverified, requires literature check (H3).** Methods that
give delay-robust guarantees may scale poorly with the number of agents, while
scalable methods may give no guarantees under delays. Little may be known about
which of the two matters more in realistic warehouse-like delay distributions.

**Required skills.** Combinatorial search, discrete optimization, simulation,
probability, and some systems engineering for multi-robot experiments.

**First small experiment.** Build on the 4-connected grid: plan two to four
agents with prioritized A* (reserving cells in time), then inject random delays
from a seeded `random.Random` and count conflicts and deadlocks. Compare this
with a simple "wait at conflict" execution policy.

**Compute / hardware.** A laptop is enough for simulation. Hardware validation
needs several small robots, which costs more.

**Risks.** This is likely an active and competitive area, so a contribution may
be incremental. The step from a grid abstraction to continuous robot motion
needs care.

---

## T4. Tactile feedback for contact-rich manipulation

**Problem statement.** Tasks such as insertion, cable routing or handling
deformable objects depend on contact forces that vision cannot see well. Tactile
sensors give rich contact signals, but turning them into robust control
policies is still hard.

**Gap - Hypothesis - unverified, requires literature check (H4).** Tactile
policies may generalize poorly across sensor units, wear and object materials.
There may be no agreed benchmark for measuring robustness to sensor variation.

**Required skills.** Manipulation and grasping, control (impedance and force),
signal processing, machine learning, and lab work with hardware.

**First small experiment.** Simulate a 1-D peg-insertion contact model with
noisy force readings. Compare a fixed-gain impedance controller with a simple
force-threshold adaptive controller as sensor noise and bias change. Report
success rate and peak force.

**Compute / hardware.** Needs a robot arm and tactile sensors for real
contributions; simulation alone may not convince reviewers. A GPU is needed for
learning-based policies.

**Risks.** Depends heavily on hardware access, contact is hard to simulate
faithfully, and sensors can be fragile and expensive.

---

## T5. Safety filters for learning-based control

**Problem statement.** Learned controllers can perform well but give no safety
guarantees. Safety filters (for example, control barrier functions or
reachability-based shields) minimally change a nominal command to keep the
system in a safe set, but they rely on a model of the dynamics.

**Gap - Hypothesis - unverified, requires literature check (H5).** Safety
filters built on learned or uncertain dynamics may be either over-conservative
or unsafe when the model error is mis-estimated. There may be no simple,
reproducible benchmark that measures the trade-off between conservativeness and
violations across model-error levels.

**Required skills.** Nonlinear control, convex optimization (QPs), dynamical
systems, machine learning basics, careful numerical experiments.

**First small experiment.** Take a 2-D single-integrator (later a unicycle)
avoiding one circular obstacle. Apply a barrier-function filter with an
injected model error, sweep the assumed error bound, and plot the violation
rate against the deviation from the nominal command.

**Compute / hardware.** A laptop for the first studies. Hardware (a mobile base
or a drone) is useful later but not needed at first.

**Risks.** The topic is mathematically demanding, and theory contributions may
need strong control background. The area may be crowded (H5).

---

## T6. Detecting execution failures for language-instructed robot tasks

**Problem statement.** Robots that take natural-language instructions often use
large learned models to break tasks into steps. These systems can produce
plausible plans that fail during execution, and they may not notice the failure.

**Gap - Hypothesis - unverified, requires literature check (H6).** Failure
detection and recovery for such systems may be evaluated mostly on success rate,
with little measurement of how well the system detects its own failures
(precision/recall of failure detection). A cheap, verifiable check on each step
(for example, pre/post-condition checks on symbolic state) may be under-explored.

**Required skills.** Task planning (symbolic and/or learned), machine learning,
software engineering, evaluation design, and an understanding of the
limitations of large models.

**First small experiment.** Define a toy symbolic world (blocks or grid rooms)
with pre/post-conditions. Inject step failures at a seeded rate into a scripted
"planner" and measure how a simple condition checker detects them. No learned
model is needed at first.

**Compute / hardware.** Early stages need only a laptop. Later stages may need
API access to or a GPU for large models, plus a manipulation or mobile platform.

**Risks.** The field moves fast, so results may date quickly. It depends on
external models that can change. Evaluation can be hard to reproduce.

---

## Notes on use

- Navigation (T1, and partly T2/T3) is closest to the existing
  `projects/nav-baseline` code. That makes it cheap to start, but it is **not** a
  reason to pick it as the research focus.
- Before any topic is chosen, run the matching check in
  [literature-search-todo.md](literature-search-todo.md) and update the gap text
  with what the search actually found.
