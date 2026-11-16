import argparse
from pathlib import Path
import sys
from plant_sim import demo_plant, simulate

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "simulation-support"))
from sim_support import draw_grid, write_results


def main():
    parser = argparse.ArgumentParser(description="Compare inspection policies with return reserves")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "results")
    args = parser.parse_args()
    plant, records = demo_plant(), []
    for name, battery, budget in (("standard", 80, 80), ("tight_dose", 80, 35), ("tight_battery", 30, 80), ("zero_budget", 0, 0)):
        for policy in ("shortest", "dose_aware"):
            row = {"scenario": name, **simulate(plant, policy, battery, budget)}
            records.append(row)
            print(f"{name:14} {policy:10} inspections={row['completed']}/3 dose={row['dose']:7.2f} time={row['elapsed_steps']:3} returned={row['returned']}")
    write_results(args.output, records)
    labels = {plant.base: "B", **{cell: str(i+1) for i, cell in enumerate(plant.inspections)}}
    for row in records:
        draw_grid(args.output / f"{row['scenario']}_{row['policy']}.svg", plant.grid, row["path"], labels,
                  f"Plant inspection: {row['scenario']} / {row['policy']} / {row['completed']} inspected", plant.rates)
    print(f"Reports: {args.output.resolve()}")


if __name__ == "__main__":
    main()
