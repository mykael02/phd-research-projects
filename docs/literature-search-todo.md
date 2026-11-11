# Literature search to-do

## Status: OUTSTANDING

**No live literature search has been done yet.** The topic candidates were
written without network access, from general background knowledge only. Every
gap in [topic-candidates.md](topic-candidates.md) stays labelled
"Hypothesis - unverified, requires literature check" until its check below is
done and recorded.

This file deliberately names no specific papers, labs, people or deadlines.

## Where to search

**Indexes and databases:** general academic search engines, the arXiv
robotics category (cs.RO) and related categories (cs.LG, cs.AI, eess.SY), the
IEEE Xplore and ACM Digital Library databases, and institutional library
access.

**Conference venues:** ICRA, IROS, RSS, CoRL, CDC/ACC (control topics),
AAAI/IJCAI and SoCS (search and multi-agent topics), WAFR (algorithmic
foundations), ISER (experimental robotics), HRI (where human interaction is
involved).

**Journal venues:** IEEE Transactions on Robotics (T-RO), IEEE Robotics and
Automation Letters (RA-L), The International Journal of Robotics Research
(IJRR), Autonomous Robots, Journal of Field Robotics, Science Robotics.

**Also check:** recent survey and tutorial articles, workshop programmes at the
major robotics conferences, and public benchmark and dataset pages.

## General procedure (for each hypothesis)

1. Run the query terms on at least two indexes. Restrict to roughly the last
   five years first, then widen.
2. Find recent surveys first and read their "open problems" sections.
3. Record every relevant work found in the log below (title, venue, year, one
   line on its relevance). Only record works that were actually opened and read.
4. Decide one outcome:
   - **Supported**: the gap appears real; no work found closes it.
   - **Partly supported**: some work addresses it; a narrower gap remains (write
     down the narrower gap).
   - **Refuted**: existing work closes the gap. Rewrite the topic or drop it.
5. Update the gap text in `topic-candidates.md`. Replace the hypothesis label
   only when the outcome is recorded here with its evidence.

## Checks per hypothesis

### H1 - Navigation under localization uncertainty (T1)
- **Query terms:** "belief space planning", "uncertainty-aware path planning",
  "localizability", "perception-aware planning", "active localization",
  "chance-constrained motion planning", "planning under localization uncertainty
  real-time".
- **Check:** Are there lightweight (real-time, low-compute) planners that add a
  localization-quality cost to grid or sampling planners **and** report the
  effect on collision or failure rate on real robots? If several do, H1 is
  refuted or must be narrowed.
- **Outcome:** OUTSTANDING

### H2 - Learned traversability (T2)
- **Query terms:** "traversability estimation", "self-supervised
  traversability", "off-road navigation learning", "terrain cost map
  uncertainty", "out-of-distribution terrain", "calibrated traversability".
- **Check:** Do existing traversability methods report calibrated uncertainty
  and measure planner safety under out-of-distribution terrain? Are public
  off-road datasets available that would allow this without collecting new
  field data?
- **Outcome:** OUTSTANDING

### H3 - Delay-robust multi-robot planning (T3)
- **Query terms:** "multi-agent path finding", "MAPF delays", "robust MAPF",
  "k-robust", "execution uncertainty multi-robot", "action dependency graph",
  "lifelong MAPF".
- **Check:** Is there a method that both scales to hundreds of agents **and**
  gives delay-robustness guarantees? Has anyone compared methods under
  realistic, measured delay distributions rather than synthetic ones?
- **Outcome:** OUTSTANDING

### H4 - Tactile contact-rich manipulation (T4)
- **Query terms:** "tactile sensing manipulation", "visuotactile policy",
  "tactile sim-to-real", "contact-rich insertion", "tactile sensor
  variability", "tactile benchmark".
- **Check:** Is there an accepted benchmark or protocol for robustness of
  tactile policies to sensor-unit variation and wear? Do existing methods report
  cross-sensor generalization?
- **Outcome:** OUTSTANDING

### H5 - Safety filters for learned control (T5)
- **Query terms:** "control barrier function learned dynamics", "safety
  filter", "robust CBF model uncertainty", "Hamilton-Jacobi reachability
  learning", "safe reinforcement learning shield", "safety filter benchmark".
- **Check:** Is there a reproducible benchmark that measures the trade-off
  between conservativeness and safety violations across levels of model error?
  How crowded is the area of robust/learned barrier functions?
- **Outcome:** OUTSTANDING

### H6 - Failure detection for language-instructed tasks (T6)
- **Query terms:** "LLM robot task planning failure detection", "language
  model robot failure recovery", "execution monitoring robot", "pre-condition
  post-condition checking", "vision-language model success detection".
- **Check:** Do existing systems report failure-detection precision and recall,
  not only task success rate? Are symbolic pre/post-condition checks combined
  with learned planners in prior work, and how are they evaluated?
- **Outcome:** OUTSTANDING

## Search log

No entries. Add entries only for searches that have actually been run.

| Date | Hypothesis | Index / venue | Query | Relevant works found (title, venue, year) | Notes |
|------|------------|---------------|-------|-------------------------------------------|-------|
| | | | | | |
