# Simulation support

Shared, standard-library-only report helpers used by `exit-finding` and
`radiation-aware-inspection`:

- `write_results(output, records)` writes `results.json` (full records) and
  `summary.csv` (records without path / inspection-order fields).
- `draw_grid(...)` renders a grid map, route and labels (optionally a radiation
  heatmap) to SVG.

It has no runner or tests of its own; it is exercised by those two projects'
experiment runners, which add this folder to `sys.path` by its location
relative to themselves.
