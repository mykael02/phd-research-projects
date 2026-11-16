# Find the exit

Runnable first experiment: a point robot explores an unfamiliar static grid and
tries to reach a marked exit. Python standard library only; no installation needed.

From the repository root:

```powershell
python projects/exit-finding/run_experiments.py
python -m unittest discover -s projects/exit-finding -p "test_*.py" -v
```

The runner produces `results/summary.csv`, full trajectories in
`results/results.json`, and SVG route diagrams. Open the SVGs in a browser.
Use `--output PATH` to keep another run and `--max-steps 1000` to change the limit.
The default output files are replaced when rerun.

## Experiment

- Compare seeded random walking with nearest-frontier exploration on a branching
  map, an open room, and an unreachable-exit negative control.
- The robot knows the rectangular map dimensions, its exact position, and the
  current cell plus four adjacent cells. It does not know walls or the exit
  location beyond this observation. The exit is a marked goal cell, not necessarily
  a boundary opening. Observation reveals an exit marker only within sensor range.
- Unknown cells are blocked for planning. A frontier is an observed free cell
  adjacent to unobserved space. Breadth-first search chooses the nearest reachable
  frontier. Once the exit is seen, the existing A* planner routes to it.
- Both policies receive the same observations and use A* once the exit is seen.
- Each move takes one simulated time unit. Report exit success, move count,
  collisions, observed cells (including walls), and the full-map shortest-path
  reference. The reference is calculated after execution and cannot guide policies.

Five seeds are used per policy and map. The frontier policy is deterministic;
its repeated seeds are identical, not independent experimental evidence. Random
walking can hit the step limit even when an exit is reachable. An unreachable
case is deliberately included; do not treat the pooled success fraction as a
measure of navigation quality. SVGs show full ground truth for review only.

## Scope and next experiments

This is a discrete grid simulation with perfect localization and local sensing.
No wall-following baseline, moving obstacles, sensor noise, footprint, dynamics,
or physical path tracking is implemented. Zero collisions follows from this
idealized movement model; it is not a real-robot safety result.

Next: add generated held-out mazes and a wall-following baseline, then investigate
whether information-aware frontier selection reduces exit-search distance.
Add blocked corridors and noisy sensing as separate controlled experiments.

Research question: how should an explorer balance discovering space against
following routes likely to lead to an exit? Novelty is unverified. Existing
frontier research: https://arxiv.org/abs/2011.05323 . A systematic literature
review is still required before making a PhD contribution claim.

This folder imports `projects/nav-baseline/gridnav` and report helpers from
`projects/simulation-support`; keep the repository layout when moving it.
