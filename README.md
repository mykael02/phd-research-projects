# Robotics PhD research projects

This repository is the starting point for choosing and beginning a robotics PhD
research focus. It holds provisional topic notes in `docs/` and several small,
runnable research projects in `projects/`. None of them is a chosen focus.

## Status: no live literature search has been done

The topic notes in `docs/` were written from general background knowledge
without any live literature search. No web, database or library search was run
for them. As a result:

- Every "gap" in `docs/topic-candidates.md` is labelled
  **Hypothesis - unverified, requires literature check**. None of them is an
  established open problem until it has been checked.
- No papers, authors, labs or results are cited, and none should be inferred.
- The live search is recorded as **OUTSTANDING** in
  `docs/literature-search-todo.md`. A topic should not be chosen until that
  search is done.

## Projects

| Project | Question it starts on | Runs on | Status |
|---|---|---|---|
| [balancing-robot](projects/balancing-robot/) | PID and LQR balancing control, sensing, payloads, and MuJoCo simulation studies | Python, NumPy/SciPy, Node.js; browser visualisations | Simulation studies and draft research reports; no hardware validation |
| [reinforcement-learning](projects/reinforcement-learning/) | Reserved for reinforcement learning experiments | Not yet implemented | Placeholder; original folder was empty |
| [nav-baseline](projects/nav-baseline/) | Grid A*, seeded RRT and a differential-drive model as a testable navigation base | Python 3.8+, stdlib | D1, D2 built; D3 path tracking deferred |
| [exit-finding](projects/exit-finding/) | How a robot with only local sensing finds a marked exit in an unknown grid (random walk vs. nearest-frontier) | Python 3.8+, stdlib | First experiment runs |
| [radiation-aware-inspection](projects/radiation-aware-inspection/) | Planning inspections under battery and radiation-dose budgets (shortest vs. dose-aware routing) | Python 3.8+, stdlib | First experiment runs; novelty unverified |
| [sparse-rgb-starter](projects/sparse-rgb-starter/) | Human motion from sparse RGB views; currently a 2D-keypoint triangulation and evaluation pipeline | Python 3.11+, numpy | Synthetic pilot runs; RGB detection etc. planned |
| [simulation-support](projects/simulation-support/) | Shared results/SVG helpers for exit-finding and radiation-aware-inspection | Python 3.8+, stdlib | Library only |

`exit-finding` and `radiation-aware-inspection` import `nav-baseline/gridnav`
and `simulation-support` by their position under `projects/`; keep these
folders side by side.

## Navigation is a provisional starter, not the chosen focus

`projects/nav-baseline/` contains a minimal 4-connected grid A* planner. It is
there to give a working, testable code base to build on while the topic is
still open. It is **not** a decision that the PhD will focus on navigation. See
`docs/starter-rationale.md` for the reasoning and the deferred work.

Two of the deferred items are now built, both under one shared geometry rule
(G) for continuous positions on the grid maps:

- **D1 seeded RRT: built** (`rrtplan/`, `run_rrt_experiments.py`, `RRT.md`).
- **D2 differential-drive model: built** (`diffdrive/`, `DIFFDRIVE.md`). It is
  kinematic only, and its collision check covers the poses at step boundaries,
  not the motion between them.
- **D3 path tracking: still deferred.**

The literature search remains **OUTSTANDING**. Building D1 and D2 does not
change that, and no topic has been chosen.

## Layout

```
README.md                         This file: index of projects
.github/workflows/tests.yml       CI: starter tests/runners and balancing-robot tests
docs/
  topic-candidates.md             Provisional robotics topics, each with a hypothesis-labelled gap
  scoring-rubric.md               Weighted criteria for choosing a research focus
  literature-search-todo.md       OUTSTANDING live search: venues, query terms, check per hypothesis
  starter-rationale.md            Why grid A* is the starter, baseline rules, deferred roadmap
  visualizations/                 Interactive HTML replays of the exit and plant experiments
projects/
  balancing-robot/                Balancing simulations, controller studies, visualisations and reports
  reinforcement-learning/         Reserved for future reinforcement learning projects
  nav-baseline/                   Grid A*, BFS, D1 seeded RRT, D2 diff-drive (see its README)
  exit-finding/                   Exploration toward an unknown exit
  radiation-aware-inspection/     Budgeted inspection missions on a synthetic radiation field
  sparse-rgb-starter/             Sparse-view triangulation pilot (installable package)
  simulation-support/             Shared report helpers
```

Each project has a `README.md` describing its current scope and status.
The navigation starters keep small result files (summary CSV and SVG); their
full `results.json` trajectories and `runs/` outputs are regenerated and ignored.
The balancing-robot project also preserves its archived study data, figures,
and draft reports. Local Python environments, report-rendering tools and previews,
logs, and the generated release ZIP stay on disk but are excluded from Git.

## Commands

Run from the repository root:

```
# balancing-robot (Python, NumPy/SciPy and Node.js)
python -m pip install -r projects/balancing-robot/requirements.txt
python -m unittest discover -s projects/balancing-robot/tests -v
# Browser entry point: projects/balancing-robot/index.html
# Additional Node.js tests and MuJoCo setup: see the project's README files.

# nav-baseline (Python 3.8+, no third-party packages)
python -m unittest discover -s projects/nav-baseline/tests -t projects/nav-baseline -v
python projects/nav-baseline/run_experiments.py            # add --timing for wall-clock section
python projects/nav-baseline/run_rrt_experiments.py

# exit-finding and radiation-aware-inspection (Python 3.8+, no third-party packages)
python -m unittest discover -s projects/exit-finding -p "test_*.py" -v
python projects/exit-finding/run_experiments.py
python -m unittest discover -s projects/radiation-aware-inspection -p "test_*.py" -v
python projects/radiation-aware-inspection/run_experiments.py

# sparse-rgb-starter (Python 3.11+, numpy); run inside its folder
cd projects/sparse-rgb-starter
python -m pip install -e .
python -m unittest discover -s tests -v
python -m sparse_rgb pilot --config configs/pilot.toml --output runs/pilot
```

The nav-baseline experiment report is deterministic and identical on every run;
`--timing` adds a separate, non-deterministic wall-clock section. The exit and
plant runners rewrite their `results/` folders; pass `--output PATH` to keep a
separate run.
