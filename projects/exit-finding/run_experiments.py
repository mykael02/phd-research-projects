"""Run from any working directory; all paths resolve relative to this file."""
import argparse
from pathlib import Path
import sys
from exit_sim import scenarios, simulate

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "simulation-support"))
from sim_support import draw_grid, write_results


def main():
    parser = argparse.ArgumentParser(description="Compare local-observation exit-search policies")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "results")
    parser.add_argument("--max-steps", type=int, default=500)
    args = parser.parse_args()
    if args.max_steps < 0:
        parser.error("--max-steps must be nonnegative")
    records = []
    for name, world in scenarios().items():
        for policy in ("random", "frontier"):
            for seed in (7, 19, 42, 73, 101):
                result = {"scenario": name, **simulate(world, policy, seed, args.max_steps)}
                records.append(result)
                print(f"{name:12} {policy:8} seed={seed:3} {result['reason']:22} steps={result['steps']:3}")
    write_results(args.output, records)
    for name, world in scenarios().items():
        row = next(r for r in records if r["scenario"] == name and r["policy"] == "frontier")
        draw_grid(args.output / f"{name}.svg", world.grid, row["path"],
                  {world.start: "S", world.exit: "E"}, f"Find the exit: {name} / frontier / {row['reason']}")
    print(f"Reports: {args.output.resolve()}")


if __name__ == "__main__":
    main()
