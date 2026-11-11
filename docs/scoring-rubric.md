# Scoring rubric for choosing a research focus

Use this rubric to compare the candidates in
[topic-candidates.md](topic-candidates.md). Do **not** score a topic until its
hypothesis (H1-H6) has been checked by the live search in
[literature-search-todo.md](literature-search-todo.md), which is currently
OUTSTANDING.

## Gate (pass/fail, before scoring)

A topic can only be scored if both are true:

1. **Gap checked.** Its hypothesis has been checked by a recorded literature
   search, and the result was "supported" or "partly supported" (see the
   outcome categories in the search to-do).
2. **Feasible to start.** A first experiment can begin within about a month
   with the resources actually available.

Topics that fail the gate are set aside, or rewritten and checked again.

## Weighted criteria

Score each criterion from 1 (poor) to 5 (strong). Weighted score = sum of
(weight x score) / 5, giving a total out of 100.

| # | Criterion | Weight | 1 means | 5 means |
|---|-----------|-------:|---------|---------|
| C1 | Verified novelty / gap strength | 20 | Gap mostly addressed already | Clear, checked, unaddressed gap |
| C2 | Personal interest and motivation | 15 | Would struggle to stay engaged for years | Strong, lasting interest |
| C3 | Supervision and community fit | 15 | No available advisor expertise or community | Strong advisor fit and active community |
| C4 | Feasibility with available resources | 15 | Needs hardware/compute that is not available | Everything needed is available |
| C5 | Skill match and growth | 10 | Large skill gaps with no plan to close them | Good base with clear growth path |
| C6 | Path to a first result | 10 | No publishable result likely within a year | Small, clear first result within a year |
| C7 | Impact and breadth | 10 | Narrow interest | Wide relevance across robotics or beyond |
| C8 | Risk profile | 5 | Single point of failure (one dataset, one robot, one vendor) | Several fallback directions |
|   | **Total** | **100** | | |

## Scoring sheet

Fill this in only after the gate has passed. Leave cells blank until then.

| Topic | Gate | C1 (20) | C2 (15) | C3 (15) | C4 (15) | C5 (10) | C6 (10) | C7 (10) | C8 (5) | Total /100 |
|-------|------|----|----|----|----|----|----|----|----|-----|
| T1 Navigation under localization uncertainty | | | | | | | | | | |
| T2 Learned traversability | | | | | | | | | | |
| T3 Delay-robust multi-robot planning | | | | | | | | | | |
| T4 Tactile contact-rich manipulation | | | | | | | | | | |
| T5 Safety filters for learned control | | | | | | | | | | |
| T6 Failure detection for language-instructed tasks | | | | | | | | | | |

## Guidance

- If two topics are within about 5 points, prefer the one with the cheaper
  first experiment (C6) and lower risk (C8).
- C2 and C3 depend on the person and the institution. Only the PhD candidate
  and their advisors can score them.
- Re-score after the first small experiment. Early results often change C4 and
  C6.
- Being close to the existing navigation baseline is **not** a criterion. The
  baseline is a provisional starter only.
