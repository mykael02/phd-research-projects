# Brain for robots in radioactive plants

Runnable first experiment: a mission planner chooses inspections and returns to
base while respecting synthetic battery and radiation-dose budgets. Python
standard library only; no installation or robot required.

From the repository root:

```powershell
python projects/radiation-aware-inspection/run_experiments.py
python -m unittest discover -s projects/radiation-aware-inspection -p "test_*.py" -v
```

Outputs: `results/summary.csv`, inspection order and full movement trajectories in
`results/results.json`, and radiation heatmaps with routes in `results/*.svg`.
Open SVGs in a browser. Use `--output PATH` to preserve another run; default
output files are replaced when rerun.

## Experiment

Compare two greedy mission policies on the same known synthetic plant:

- `shortest`: existing A* routes and a preference for the cheapest next inspection
  in time units.
- `dose_aware`: Dijkstra routes minimizing `movement time + weight * dose`, with
  default weight 3, and the same weighted preference for the next inspection.

Both enforce identical battery and dose limits, including the cost of returning
to base. Before accepting an inspection, each checks the outward route, one
stationary inspection time unit, and a specific reserved return route. When no
next inspection passes, the robot follows the last reserved route home.

Every move takes one time unit and one battery unit. Movement dose is the average
of the two endpoint dose rates times that duration; inspection adds one battery
unit and the target's dose rate. Rates and doses are arbitrary simulation units,
not calibrated radiation measurements. The field is `0.2 + 18/(1 + squared
distance to hotspot)` and does not model shielding or equipment degradation.

Run four scenarios: standard budgets (80 battery / 80 dose), tight dose (80 / 35),
tight battery (30 / 80), and zero budget (0 / 0). These illustrative limits were
chosen to exercise full, partial, and refused missions; they are not a benchmark.
Report completed inspections, return status, elapsed time, movement steps, dose,
and budget compliance. A robot staying at base counts as returned but completes
zero work; consider these metrics together. Passing a target does not inspect it.

## Limits and next experiments

The map and entire radiation field are known exactly and remain static. Routes
and inspections execute perfectly on a point-robot grid. Return feasibility is
only assured under those assumptions. This is not a controller for a nuclear site.

Weighted routes and greedy task selection are baselines, not optimal constrained
mission planning: a selected route may fail the budget check even when a different
route could fit, and task order can miss better combinations. The reason
`no_feasible_next_inspection` means none of the routes this policy considered fit;
it is not a proof that every possible mission is infeasible. Unreachable targets
produce the same stop reason. No uncertainty, online radiation mapping, sensor
faults, dynamic replanning, or radiation-dependent failure model is implemented.

Next: expose only local noisy radiation readings; maintain an estimated field;
compare conservative return reserves under uncertainty with this known-field
reference. Then introduce faults and measure recovery and failed returns.

Research question: when should a robot inspect, gather information, or retreat
under uncertain radiation exposure? Novelty remains unverified. Radiation-aware
navigation already exists, for example:
https://pmc.ncbi.nlm.nih.gov/articles/PMC8968874/ . A systematic literature review
is still required before making a PhD contribution claim.

This folder imports `projects/nav-baseline/gridnav` and report helpers from
`projects/simulation-support`; keep the repository layout when moving it.
