"""Fixed, reproducible planning experiments. No dependencies beyond Python."""

import argparse
from dataclasses import dataclass
from pathlib import Path
import random
import time
from typing import Iterable, List, Optional, Sequence

from gridnav import Cell, Grid, astar, bfs, load_grid, parse_grid, validate_path

DEFAULT_SEEDS = (7, 19, 42, 73, 101)
MAPS = Path(__file__).resolve().parent / "maps"


@dataclass(frozen=True)
class Case:
    name: str
    grid: Grid
    start: Cell
    goal: Cell


def seeded_case(seed: int) -> Case:
    rng = random.Random(seed)
    height, width = 12, 16
    rows = [
        ["#" if rng.random() < 0.25 else "." for _ in range(width)]
        for _ in range(height)
    ]
    rows[0][0] = rows[-1][-1] = "."
    return Case(
        "seed_{}".format(seed), Grid(tuple("".join(row) for row in rows)),
        (0, 0), (height - 1, width - 1),
    )


def default_cases(seeds: Iterable[int] = DEFAULT_SEEDS) -> List[Case]:
    cases = [
        Case("open", load_grid(MAPS / "open.txt"), (0, 0), (3, 4)),
        Case("detour", load_grid(MAPS / "detour.txt"), (0, 0), (0, 2)),
        Case("disconnected", load_grid(MAPS / "disconnected.txt"), (0, 0), (0, 4)),
        Case("same_cell", parse_grid("."), (0, 0), (0, 0)),
        Case("blocked_start", parse_grid("#."), (0, 0), (0, 1)),
        Case("blocked_goal", parse_grid(".#"), (0, 0), (0, 1)),
    ]
    return cases + [seeded_case(seed) for seed in seeds]


def generate_report(
    cases: Optional[Sequence[Case]] = None,
    seeds: Iterable[int] = DEFAULT_SEEDS,
) -> str:
    """Return deterministic text. Explicit cases replace the entire default set.

    Seeds select random cases only when cases is None. Timing is never included.
    """
    selected = default_cases(seeds) if cases is None else cases
    lines = [
        "4-connected unit-cost grid planning (point robot)",
        "case | success | reason | length | BFS length | optimal | nodes expanded",
    ]
    successes = 0
    total = 0
    for case in selected:
        result = astar(case.grid, case.start, case.goal)
        reference = bfs(case.grid, case.start, case.goal)
        for planned in (result, reference):
            if planned.success and not validate_path(
                case.grid, planned.path, case.start, case.goal
            ):
                raise RuntimeError("invalid planner path in case " + case.name)
        optimal = (
            result.success == reference.success
            and result.length == reference.length
            and result.reason == reference.reason
        )
        successes += int(result.success)
        total += 1
        lines.append("{} | {} | {} | {} | {} | {} | {}".format(
            case.name, "yes" if result.success else "no", result.reason,
            result.length if result.length is not None else "-",
            reference.length if reference.length is not None else "-",
            "yes" if optimal else "no", result.nodes_expanded,
        ))
    rate = 100.0 * successes / total if total else 0.0
    lines.append("Planning success rate: {}/{} ({:.1f}%)".format(successes, total, rate))
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timing", action="store_true", help="append non-deterministic elapsed time")
    args = parser.parse_args()
    if args.timing:
        started = time.perf_counter()
    report = generate_report()
    if args.timing:
        elapsed = time.perf_counter() - started
    print(report, end="")
    if args.timing:
        print("\nTiming (non-deterministic)")
        print("Full report generation, including grid loading, A*, BFS and validation: {:.6f} s".format(elapsed))


if __name__ == "__main__":
    main()
