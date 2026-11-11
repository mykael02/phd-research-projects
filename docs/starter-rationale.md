# Starter rationale: why a grid A* baseline

## Why this starter

`projects/nav-baseline` is a **provisional** starting point. It is not a
decision that the PhD will focus on navigation. It was chosen because:

- **It needs nothing to run.** It uses only the Python 3.8+ standard library,
  with no simulator, GPU or robot, so work can start while the topic is still
  open and the literature search is OUTSTANDING.
- **Correctness can be checked.** On a 4-connected, unit-cost grid,
  breadth-first search (BFS) gives the true shortest path length, so A* can be
  checked against it exactly.
- **It is reproducible.** Fixed hand-made maps and seeded random grids give an
  experiment report that is identical on every run. This sets the habit of
  deterministic experiments early.
- **It is reusable.** Several candidates in
  [topic-candidates.md](topic-candidates.md) have first experiments that extend
  a grid planner (T1 localizability cost, T2 weighted costs, T3 multi-agent
  reservations).

## Baseline rules (as built)

These rules define what the baseline does and does not claim:

- **Map:** `.` is a free cell and `#` is a blocked cell. Ragged, empty or
  unknown-character maps raise `ValueError`.
- **Coordinates:** `(row, col)`.
- **Robot model:** a point robot that occupies exactly one cell. There is no
  footprint, orientation or dynamics.
- **Moves:** 4-connected (up, down, left, right) with unit cost. No diagonal
  moves. Path length is `len(path) - 1`.
- **Heuristic:** Manhattan distance, with deterministic tie-breaking.
- **Endpoint-check order**, applied before any search:
  1. bounds: start out of bounds raises `ValueError`, then goal out of bounds
     raises `ValueError`;
  2. blocked start fails with reason `start_blocked`;
  3. blocked goal fails with reason `goal_blocked`;
  4. `start == goal` succeeds with path `[start]` and length 0. This is only
     reachable for an in-bounds free cell.

  Otherwise the planner searches, and an unreachable goal fails with `no_path`.
- **Planning success is not execution success.** "Success" means the planner
  returned a path that passes `validate_path` (all cells in bounds and free,
  correct endpoints, consecutive cells 4-adjacent). It says nothing about
  whether a real robot with size, dynamics, sensing noise or control error
  could follow that path.
- **Timing is reported separately.** The default report contains no wall-clock
  times and is byte-identical across runs. Timing appears only with `--timing`,
  in a separate section labelled non-deterministic.

## Deferred work (later slices)

These items are **not** implemented. Each is listed with ideas for how it could
be accepted when built.

### D1. Sampling-based planning (RRT, and possibly RRT*)
- **Scope:** continuous 2-D configuration space with obstacles taken from the
  grid maps (each blocked cell becomes an axis-aligned square), point robot
  first.
- **Acceptance ideas:**
  - Uses a seeded `random.Random`, so the same seed gives the same tree and path.
  - A continuous path validator checks every segment against the obstacles,
    not just the vertices.
  - On maps where grid A* finds a path, RRT finds a path within a fixed
    iteration budget on a stated fraction of seeds. Failures are reported with a
    reason, not hidden.
  - If RRT* is added, path cost does not increase as iterations grow, and is
    compared with the grid optimum as a reference (noting that the two use
    different metrics).

### D2. Differential-drive robot model
- **Scope:** unicycle/differential-drive kinematics (x, y, heading) with
  velocity and turn-rate limits, integrated at a fixed time step.
- **Acceptance ideas:**
  - Unit tests check analytic cases: straight-line motion, turning in place,
    and a constant-curvature arc that returns to its start after a full circle
    within a stated tolerance.
  - Commands outside the limits are clipped or rejected, as documented and
    tested.
  - The simulation is deterministic for a fixed time step and fixed inputs.
  - A footprint radius is introduced, with tests showing that obstacle
    inflation makes free-but-narrow passages correctly fail.

### D3. Path tracking
- **Scope:** a controller (for example, pure pursuit) that drives the D2 model
  along a path from A* or RRT.
- **Acceptance ideas:**
  - Reports **execution** metrics separately from planning metrics: reached
    goal yes/no, maximum and mean cross-track error, collisions, time to goal in
    simulated steps.
  - Shows on at least one case that a valid plan can still fail in execution
    (for example, a sharp grid corner with a turn-rate limit). This makes the
    planning/execution distinction concrete.
  - Deterministic output for fixed inputs. Optional sensor or actuation noise
    comes only from a seeded generator.
  - Tests cover a straight path (near-zero cross-track error) and a path the
    controller cannot follow within limits (reported as an execution failure,
    not a crash).

## What would change this plan

Once the literature search is done and a topic is scored with
[scoring-rubric.md](scoring-rubric.md), the deferred items should be
re-prioritized. If the chosen focus is not navigation (for example, T4
manipulation or T6 task-level failure detection), D1-D3 may be dropped in
favour of a starter suited to that topic.
