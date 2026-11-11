# Robotics PhD research projects

This repository is the starting point for choosing and beginning a robotics PhD
research focus. It has two parts: provisional topic notes and a small,
deterministic navigation baseline to experiment with.

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
README.md                         This file
docs/
  topic-candidates.md             Provisional robotics topics, each with a hypothesis-labelled gap
  scoring-rubric.md               Weighted criteria for choosing a research focus
  literature-search-todo.md       OUTSTANDING live search: venues, query terms, check per hypothesis
  starter-rationale.md            Why grid A* is the starter, baseline rules, deferred roadmap
projects/
  nav-baseline/                   Standard-library-only Python 3.8+ baseline
    gridnav/                      Grid loading, 4-connected A*, BFS reference, validate_path
    rrtplan/                      D1: seeded RRT in continuous space (rule G)
    diffdrive/                    D2: unicycle kinematics, limits, disc collision (rule G)
    maps/                         Hand-made '.'/'#' text maps
    run_experiments.py            Deterministic experiment report (timing only with --timing)
    run_rrt_experiments.py        Deterministic D1 RRT report
    tests/                        unittest suite
    README.md                     Baseline details
    RRT.md                        D1 rules and fixed settings
    DIFFDRIVE.md                  D2 model, rule G, collision-check limitation
```

## Commands

Run from the repository root (Python 3.8+, no third-party packages):

```
python -m unittest discover -s projects/nav-baseline/tests -t projects/nav-baseline -v
python projects/nav-baseline/run_experiments.py
python projects/nav-baseline/run_experiments.py --timing
python projects/nav-baseline/run_rrt_experiments.py
```

The first runs the test suite. The second prints the deterministic experiment
report, which is identical on every run. The third adds a separate wall-clock
timing section, labelled non-deterministic. The fourth prints the D1 RRT report
(see `projects/nav-baseline/RRT.md`).
