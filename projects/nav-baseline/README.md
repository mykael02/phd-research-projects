# Minimal grid navigation baseline

This is a provisional robotics research starter, not a selected PhD focus or
evidence of a research contribution. It implements planning for a point robot
occupying one cell, with four-connected, unit-cost moves. Planning success is
not execution success: dynamics, robot footprint, sensing and tracking are absent.
No live literature search was performed for this slice; that remains outstanding.
RRT, differential-drive dynamics and path tracking are deferred to later work.

Python 3.8+ and the standard library are sufficient; install nothing. From the
repository root, run these commands separately:

```text
python -m unittest discover -s projects/nav-baseline/tests -t projects/nav-baseline -v
python projects/nav-baseline/run_experiments.py
python projects/nav-baseline/run_experiments.py --timing
```

## Layout and API

- `gridnav/`: strict grid parsing/loading, A*, BFS reference and path validation.
- `maps/`: hand-made ASCII maps (`.` free, `#` blocked).
- `run_experiments.py`: fixed cases, seeded random cases and report function.
- `tests/`: unittest checks, including independent script invocations.

With `projects/nav-baseline` on the Python import path:

```python
from gridnav import astar, parse_grid, validate_path

grid = parse_grid("...\n.#.\n...")
result = astar(grid, (0, 0), (2, 2))
assert result.success and validate_path(grid, result.path, (0, 0), (2, 2))
assert result.length == 4
```

Coordinates are integer `(row, col)` tuples. `parse_grid(text)` and
`load_grid(path)` reject empty, ragged and unknown-character maps with
`ValueError`. A final newline is accepted; spaces and blank rows are not trimmed.
`Grid` validates direct construction too. Both planners check endpoints in this
exact order:

1. Start out of bounds: raise `ValueError("start out of bounds")`.
2. Goal out of bounds: raise `ValueError("goal out of bounds")`.
3. Blocked start: fail with `start_blocked`.
4. Blocked goal: fail with `goal_blocked`.
5. Equal endpoints: succeed with `[start]`, length zero, only for a free cell.
6. Search: succeed with `ok`, or fail with `no_path` if unreachable.

Results contain `success`, `reason`, `path`, `length` and `nodes_expanded`.
Failed paths are empty and their length is `None`. Successful length is
`len(path) - 1`. `validate_path(grid, path, start, goal)` returns a boolean,
checking endpoints, free in-bounds cells and consecutive four-adjacent cells.

A* uses Manhattan distance and increasing insertion order to break equal
priority ties. Neighbors are visited up, right, down, left. BFS independently
uses a FIFO queue to obtain the shortest-path reference. Expanded nodes count
non-stale queue removals, including the goal; endpoint-only results count zero.

## Reproducible experiments

`generate_report(cases=None, seeds=DEFAULT_SEEDS)` returns a string with no timing.
The default cases include open space, a hand-made obstacle map, disconnection,
equal endpoints, blocked endpoints, and five random grids. Random grids are
12 by 16, with obstacle probability 0.25, using a fresh `random.Random(seed)`;
start `(0, 0)` and goal `(11, 15)` are forced free. Default seeds are
7, 19, 42, 73 and 101. No global random state is used.

Explicit `Case` sequences replace all defaults; `seeds` applies only when
`cases` is `None`. The same ordered cases/seeds produce identical reports.
Per-case fields are success/reason, A* length, BFS length, optimal yes/no and
A* nodes expanded. `optimal=yes` means validated success with the same length
as BFS, or matching failure reasons from both planners. Missing lengths are `-`.
Every successful path from either planner is validated before reporting.
The aggregate is the fraction of cases with successful A* plans, including the
intentionally failing cases in the denominator; it is not a performance benchmark.

`--timing` appends a separate section labelled `Timing (non-deterministic)`.
It measures full report generation, including loading, both searches and validation,
and leaves the deterministic report prefix unchanged.
