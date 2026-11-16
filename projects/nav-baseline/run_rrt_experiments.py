"""Fixed seeded RRT acceptance report; timing is opt-in."""

import argparse
from collections import Counter
from pathlib import Path
from statistics import median
import time

from gridnav import astar, load_grid
from rrtplan import rrt, validate_continuous_path

DEFAULT_SEEDS = tuple(range(20))
MAPS = Path(__file__).resolve().parent / "maps"
CASES = (
    ("open", (0, 0), (3, 4), 18, 20),
    ("detour", (0, 0), (0, 2), 18, 20),
    ("disconnected", (0, 0), (0, 4), 0, 0),
)


def generate_report() -> str:
    lines = [
        "Continuous point-robot RRT under rule G; seeds 0-19",
        "Settings: max_iterations=2000, step=0.5, goal_bias=0.1",
        "A* length is a 4-connected grid reference only, not a continuous optimum.",
        "map | successes | median successful length | failure reasons | A* length (grid reference only) | success threshold | check",
    ]
    for name, start, goal, minimum, maximum in CASES:
        grid = load_grid(MAPS / (name + ".txt"))
        results = [rrt(grid, start, goal, seed=seed) for seed in DEFAULT_SEEDS]
        for result in results:
            if result.success and not validate_continuous_path(grid, result.path, start, goal):
                raise RuntimeError("invalid continuous path in case " + name)
        lengths = [result.length for result in results if result.success]
        failures = Counter(result.reason for result in results if not result.success)
        reference = astar(grid, start, goal)
        passed = minimum <= len(lengths) <= maximum
        if name == "disconnected":
            passed = passed and failures == {"budget_exhausted": 20}
        lines.append("{} | {}/20 | {} | {} | {} | {} | {}".format(
            name, len(lengths), "{:.6f}".format(median(lengths)) if lengths else "-",
            ", ".join("{}={}".format(reason, failures[reason]) for reason in sorted(failures)) or "none",
            reference.length if reference.success else "-",
            ">=18/20" if minimum else "0/20; budget_exhausted=20",
            "PASS" if passed else "FAIL",
        ))
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
        print("Full report generation, including loading, RRT, A* and validation: {:.6f} s".format(elapsed))


if __name__ == "__main__":
    main()
