# Seeded continuous RRT (D1)

D1 implements a point-robot RRT using Python 3.8+ and only the standard library.
It is a planning baseline, with no robot tracking or execution guarantee. D3
(tracking) remains deferred. The live literature search is still **OUTSTANDING**;
this slice uses no literature-search results. The differential-drive D2 slice is
separate; this planner does not model a disc footprint or vehicle dynamics.

## Shared geometry rule G

- G1: a grid cell `(row, col)` maps to `(x, y) = (col + 0.5, row + 0.5)`.
  Positive x follows columns and positive y follows rows.
- G2: the map area is `[0, width] x [0, height]`. A blocked cell `(r, c)`
  occupies the closed square `[c, c+1] x [r, r+1]`.
- G3: a point is free only when `0 < x < width`, `0 < y < height` and it
  belongs to no closed blocked square. Touching obstacles or outer edges collides.
- G4 (shared disc convention, implemented separately by D2): a disc of radius
  `rho >= 0` collides when its centre's Euclidean distance to a blocked square
  is `<= rho`, or `x-rho <= 0`, `x+rho >= width`, `y-rho <= 0`, or
  `y+rho >= height`. At `rho = 0` this reduces to G3.
- G5: blocked and out-of-range cells map to points in collision.

On `maps/detour.txt`, `(0.5, 2.5)` and `(1.5, 4.5)` are free;
`(1.0, 2.5)`, `(0.0, 2.5)` and `(3.0, 2.5)` collide. D1 tests all these
point cases. The shared disc examples at `(0.5, 2.5)` (free at radius 0.4,
colliding at 0.5 and 0.6) belong to D2's footprint checks.

`segment_is_free` rejects any intersection with a closed blocked square,
including a single corner touch. It intersects parameter intervals analytically,
using `fractions.Fraction` for exact arithmetic on the supplied binary float
coordinates. There is no sampling or collision tolerance. Coordinates generated
by the planner still use floating-point arithmetic. Both segment endpoints must
be free; convexity of the open map rectangle then excludes outer-edge contact
along the whole segment. Non-finite point coordinates are rejected.

## API and deterministic settings

With `projects/nav-baseline` on the import path:

```python
from gridnav import load_grid
from rrtplan import rrt, validate_continuous_path

grid = load_grid("projects/nav-baseline/maps/detour.txt")
result = rrt(grid, (0, 0), (0, 2), seed=0)
if result.success:
    assert validate_continuous_path(grid, result.path, (0, 0), (0, 2))
```

`rrt(grid, start_cell, goal_cell, seed=0, max_iterations=2000, step=0.5,
goal_bias=0.1)` creates only a local `random.Random(seed)`. Samples are the
goal with probability 0.1, otherwise uniform in the map rectangle. Each iteration
chooses the nearest existing node (insertion order breaks ties), steers at most
0.5 units and checks the full extension segment. Rejected samples still consume
an iteration. Every accepted new node attempts a direct, checked connection to
the exact goal point; this final connection may exceed the steering step.
The planner stops at its first success and does not optimize or smooth the path.

The immutable `RRTResult` contains `success`, `reason`, `path`, `nodes`,
`parents` and `iterations`. Points, paths and trees use tuples; `parents[i]`
is the parent index for `nodes[i]`, with `None` for the root. `length` is the
sum of Euclidean segment lengths on success and `None` on failure. Identical
inputs and seed produce identical trees and paths in the same Python runtime.

Endpoint checks occur before search, checking start before goal. Blocked or
out-of-range cells return `start_in_collision` or `goal_in_collision` without
raising. Equal free endpoints succeed with reason `ok`, one path point, length
zero and zero iterations. Search exhaustion returns `budget_exhausted`, an empty
path and the explored tree. Zero iterations is a valid budget. Invalid settings
(negative/noninteger budget, nonpositive/nonfinite step, or bias outside `[0,1]`)
raise `ValueError` when search is needed; these are configuration errors rather
than planning failures. The cell API expects integer `(row, col)` tuples.

`validate_continuous_path(grid, path, start_cell, goal_cell)` requires a nonempty
path, exact start and goal centres, and every point and segment free under G.
It accepts a one-point free path for identical endpoints. Planning success does
not imply physical execution success, clearance for a nonzero radius, or optimality.

## Fixed acceptance experiments

From the repository root, run separately:

```text
python -m unittest discover -s projects/nav-baseline/tests -t projects/nav-baseline -v
python projects/nav-baseline/run_rrt_experiments.py
python projects/nav-baseline/run_rrt_experiments.py --timing
```

Seeds are exactly 0 through 19, with the defaults above. The tests require:

| Map | Start → goal cells | Required successes |
| --- | --- | --- |
| open | `(0,0)` → `(3,4)` | at least 18/20 |
| detour | `(0,0)` → `(0,2)` | at least 18/20 |
| disconnected | `(0,0)` → `(0,4)` | exactly 0/20, all `budget_exhausted` |

Tests validate every successful path for these seeds, repeat each run to compare
the complete result including its tree, and check that global random state is
unchanged. Geometry tests include wall crossings with free endpoints, corner
touches, outer-edge segments, and tiny offsets on either side of a corner touch.

`run_rrt_experiments.generate_report()` reports successes out of 20, median
Euclidean path length over successes (six decimal places, or `-` when absent),
failure-reason counts, thresholds and PASS/FAIL. Each successful path is validated
before reporting. The A* length is labelled **grid reference only**: four-connected
grid distance is not a continuous shortest-path reference. These small fixed
cases are acceptance checks, not evidence of general success rates or performance.

Two independent script runs must produce byte-identical output. Tests also vary
`PYTHONHASHSEED`. Timing appears only with `--timing`, in a separate labelled
section after the unchanged deterministic report. It measures report generation,
including loading, planning, the A* reference and path validation.
