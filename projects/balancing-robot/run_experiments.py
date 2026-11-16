"""Repeatable nominal and stress cases, with full trajectories and CSV metrics."""
import argparse
import csv
from dataclasses import replace
import json
from pathlib import Path
from balance import Parameters, simulate


def main():
    root = Path(__file__).resolve().parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=root/"results")
    args=parser.parse_args()
    config=json.loads((root/"config.json").read_text())
    scenarios=[("initial_tilt",5,0,.7), ("push_right",5,3,.7), ("push_left",5,-3,.7),
               ("large_push",5,30,.7), ("weak_motors",15,3,.1)]
    summaries, trajectories = [], {}
    for scenario, tilt, force, limit in scenarios:
        for controller in ("pid","lqr","off"):
            params=replace(Parameters(**config["parameters"]),torque_limit=limit)
            metrics, rows=simulate(controller,config,initial_degrees=tilt,push_force=force,params=params)
            summary={"scenario":scenario,"initial_tilt_deg":tilt,"push_force_n":force,"torque_limit_nm":limit,**metrics}
            summaries.append(summary)
            trajectories[f"{scenario}_{controller}"]=rows
            print(f"{scenario:14} {controller:3} {'FALL' if metrics['fallen'] else 'upright':7} peak={metrics['peak_tilt_deg']:6.2f} deg  recovery={metrics['recovery_s']} s")
    args.output.mkdir(parents=True,exist_ok=True)
    with (args.output/"summary.csv").open("w",newline="",encoding="utf-8") as handle:
        writer=csv.DictWriter(handle,fieldnames=list(summaries[0]));writer.writeheader();writer.writerows(summaries)
    (args.output/"summary.json").write_text(json.dumps(summaries,indent=2)+"\n",encoding="utf-8")
    (args.output/"trajectories.json").write_text(json.dumps(trajectories,separators=(',',':'))+"\n",encoding="utf-8")
    print("Results:",args.output.resolve())


if __name__=="__main__":
    main()
