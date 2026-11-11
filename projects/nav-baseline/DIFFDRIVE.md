# D2: differential-drive (unicycle) model

`diffdrive/` adds a kinematic unicycle model of a differential-drive robot with
a disc footprint, on the same ASCII maps as `gridnav/`. It uses only the Python
3.8+ standard library and is deterministic. It is a kinematic model only: there
is no mass, wheel slip, actuator lag, sensing or noise, and no controller. Path
tracking (D3) is still deferred.

## Shared geometry rule G

D1 (`rrtplan/`, see `RRT.md`) and D2 each implement and test this rule
independently.

- **G1** Grid cell `(r, c)` (gridnav's `(row, col)`) maps to the continuous
  point `(x, y) = (c + 0.5, r + 0.5)`: `cell_center(cell)`.
- **G2** The map area is `[0, width] x [0, height]`. Blocked cell `(r, c)` is
  the closed square `[c, c+1] x [r, r+1]`.
- **G3** A point is free only if `0 < x < width`, `0 < y < height` and it lies
  in no closed blocked square. Touching an obstacle or the outer edge is a
  collision: `point_in_collision(grid, point)`.
- **G4** A disc with centre `p` and radius `rho >= 0` collides if the Euclidean
  distance from `p` to any blocked square is `<= rho`, or if `x - rho <= 0`,
  `x + rho >= width`, `y - rho <= 0` or `y + rho >= height`:
  `disc_in_collision(grid, centre, rho)`. With `rho = 0` this is G3; the two
  functions are written separately and a test checks they agree.
- **G5** Out-of-range or blocked cells are in collision:
  `cell_in_collision(grid, cell, rho=0.0)`.

A negative or non-finite radius raises `ValueError`. Non-finite coordinates
are treated as collisions.

Reference cases on `maps/detour.txt` (width 3, height 5, column 1 blocked for
rows 0-3), all tested in `tests/test_diffdrive.py`: `(0.5, 2.5)` free;
`(1.0, 2.5)` in collision (touches the wall); `(0.0, 2.5)` and `(3.0, 2.5)` in
collision (outer edge); `(1.5, 4.5)` free; a disc at `(0.5, 2.5)` is free at
`rho = 0.4` and in collision at `rho = 0.5` and `rho = 0.6`. A disc at
`(1.5, 4.5)` in the one-unit passage below the wall is free at `rho = 0.4` and
in collision at `rho = 0.5`.

## Model

- **Frame:** the same frame as G. `x` grows with the column, `y` with the row,
  so `y` points down in the printed map. The heading `theta` is measured from
  `+x` towards `+y`, so a positive turn rate looks clockwise in the printed map.
- **Pose:** `Pose(x, y, theta)`. Returned headings are wrapped to `[-pi, pi]`.
- **Limits:** `Limits(v_min, v_max, omega_max)`, inclusive:
  `v` in `[v_min, v_max]` and `omega` in `[-omega_max, omega_max]`. Non-finite
  values, `v_min > v_max` or `omega_max < 0` raise `ValueError`. Wheel-level
  limits (track width, per-wheel speed) are not modelled.
- **Clipping:** `clip_command(v, omega, limits)` returns
  `ClippedCommand(v, omega, clipped)`. `clipped` is true if either value was
  changed; values exactly at a limit are not clipped. Non-finite commands raise
  `ValueError`.
- **Integration:** `step(pose, v, omega, dt)` holds `(v, omega)` constant for a
  fixed `dt > 0`. If `omega == 0` it uses the straight-line update
  `x += v dt cos(theta)`, `y += v dt sin(theta)`. Otherwise it uses the exact
  arc: `theta' = theta + omega dt`,
  `x += (v/omega)(sin theta' - sin theta)`,
  `y -= (v/omega)(cos theta' - cos theta)`. It is computed in the equivalent
  form `x += c cos(theta + omega dt/2)`, `y += c sin(theta + omega dt/2)` with
  chord `c = v dt sin(omega dt/2)/(omega dt/2)`, which avoids subtracting
  nearly equal sines when `omega` is tiny but nonzero. `step` applies no limits.

## Rollout and the collision-check limitation

`rollout(grid, start, commands, dt, limits, radius=0.0)` clips each command,
applies one `step` per command and returns `RolloutResult(poses,
collision_step, clipped_steps)`:

- `poses[0]` is the start and `poses[i]` the pose after step `i`.
- The disc footprint (G4) is checked at the start pose and at the pose after
  each step. The rollout stops at the first pose in collision;
  `collision_step` is its index in `poses` (0 if the start collides), or `None`.
- `clipped_steps` lists the 1-based steps whose command was clipped.

**Limitation: the motion between steps is not checked.** Only the poses at the
step boundaries are tested against G4. Checking the swept disc along the
segment or arc between steps is not implemented. With a large `dt` the robot
can pass through an obstacle without being flagged. A test demonstrates this on
`maps/detour.txt`: from `(0.5, 2.5)` heading `+x` with `v = 2` and
`radius = 0.2`, one step of `dt = 1.0` lands at `(2.5, 2.5)`, on the far side
of the one-unit wall in column 1, and is reported collision-free; the same
motion with `dt = 0.1` is flagged at step 2. A small `dt` reduces the chance
of a missed collision but does not remove it.

## What the tests check

`tests/test_diffdrive.py` checks the G reference cases above, that `rho = 0`
matches the point rule over a quarter-unit sweep of three maps, G5 for blocked
and out-of-range cells, straight-line motion, turning on the spot with the
position unchanged, one full circle (`v = omega = 1`, 100 steps) returning to
its start within `1e-9` in `x`, `y` and wrapped `theta`, that one arc step
matches ten smaller ones, that the arc update matches the textbook
`(v/omega)` formula for moderate turn rates and stays within `1e-8` of the
straight-line update for tiny positive and negative turn rates (including a
rollout on `detour.txt` that must flag the wall), clipping and its flag, the step-boundary collision
gap above, and that repeated rollouts are identical. These are checks of the
kinematic equations and collision rule only; they make no claim about how a
real robot would move.
