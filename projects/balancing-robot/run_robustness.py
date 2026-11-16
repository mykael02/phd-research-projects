"""Deterministic three-dimensional controller sweep, with sampled boundaries."""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
import csv
import math
import os
from pathlib import Path
import platform

from balance import Parameters, simulate

ROOT = Path(__file__).resolve().parent
TILTS = tuple(range(-20, 21, 5))
PUSHES = tuple(range(-30, 31, 5))
TORQUES = (.1, .2, .3, .4, .5, .7, 1.0)
CONTROLLERS = ("pid", "lqr")


def outcome(metrics):
    if metrics["fallen"]:
        return "fallen"
    return "recovered" if metrics["recovery_s"] is not None else "unsettled"


def evaluate(config, controller, tilt, push, torque, physics_substeps=1):
    parameters=replace(Parameters(**config["parameters"]), torque_limit=torque)
    metrics, _ = simulate(controller, config, initial_degrees=tilt, push_force=push,
                          params=parameters, dt=config["dt"], physics_substeps=physics_substeps)
    return {"initial_tilt_deg":tilt, "push_force_n":push, "torque_limit_nm":torque,
            **metrics, "outcome":outcome(metrics),
            "push_started":push != 0 and metrics["duration_s"] > 2.0,
            "push_completed":push != 0 and metrics["duration_s"] >= 2.2,
            "saturation_s":round(metrics["saturated_steps"]*config["dt"],6)}


def run_grid(config, tilts=TILTS, pushes=PUSHES, torques=TORQUES, progress=None):
    axes = (tuple(tilts),tuple(pushes),tuple(torques))
    for axis in axes:
        if not axis or len(set(axis))!=len(axis) or not all(math.isfinite(v) for v in axis):
            raise ValueError("grid axes must be nonempty, finite and unique")
    if any(abs(v)>=45 for v in axes[0]) or any(v<=0 for v in axes[2]):
        raise ValueError("initial tilts must be below 45 degrees and torque limits positive")
    records=[]
    for torque in axes[2]:
        for tilt in axes[0]:
            for push in axes[1]:
                for controller in CONTROLLERS:
                    records.append(evaluate(config,controller,tilt,push,torque))
        if progress:
            progress(torque,len(records))
    return records


def summarize(records):
    summaries=[]
    for controller in CONTROLLERS:
        selected=[r for r in records if r["controller"]==controller]
        counts=Counter(r["outcome"] for r in selected)
        summaries.append({"controller":controller,"tested":len(selected),
                          **{name:counts[name] for name in ("recovered","unsettled","fallen")},
                          "fell_before_push":sum(r["fallen"] and r["push_force_n"]!=0 and not r["push_started"] for r in selected)})
    return summaries


def boundary_candidates(records):
    """Cells with unlike adjacent outcomes on any axis, without monotonicity assumptions."""
    keys=("initial_tilt_deg","push_force_n","torque_limit_nm")
    axes=[sorted({r[k] for r in records}) for k in keys]
    lookup={(r["controller"],*(r[k] for k in keys)):r for r in records}
    candidates=[]
    for row in records:
        coords=[row[k] for k in keys]
        different=False
        for dim, axis in enumerate(axes):
            index=axis.index(coords[dim])
            for other in (index-1,index+1):
                if 0<=other<len(axis):
                    neighbor=coords.copy();neighbor[dim]=axis[other]
                    found=lookup.get((row["controller"],*neighbor))
                    if found is not None and found["outcome"]!=row["outcome"]:
                        different=True
        if different:
            candidates.append(row)
    return candidates


def audit_boundaries(config, records, limit=32):
    candidates=boundary_candidates(records)
    count=min(limit,len(candidates))
    indices=sorted({round(i*(len(candidates)-1)/max(1,count-1)) for i in range(count)})
    audits=[]
    for i in indices:
        original=candidates[i]
        refined=evaluate(config,original["controller"],original["initial_tilt_deg"],
                         original["push_force_n"],original["torque_limit_nm"],physics_substeps=2)
        audits.append({k:original[k] for k in ("controller","initial_tilt_deg","push_force_n","torque_limit_nm")})
        audits[-1].update(original_outcome=original["outcome"],refined_outcome=refined["outcome"],
                          outcome_agrees=original["outcome"]==refined["outcome"],
                          peak_tilt_difference_deg=round(abs(original["peak_tilt_deg"]-refined["peak_tilt_deg"]),6),
                          original_recovery_s=original["recovery_s"],refined_recovery_s=refined["recovery_s"])
    return {"boundary_candidate_count":len(candidates),"audited":len(audits),
            "disagreements":sum(not r["outcome_agrees"] for r in audits),
            "control_step_s":config["dt"],"refined_physics_step_s":config["dt"]/2,"records":audits}


def save_report(output, config, records, audit):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    keys=("initial_tilt_deg","push_force_n","torque_limit_nm")
    config_text=json.dumps(config,sort_keys=True,separators=(',',':'))
    metadata={"study":"nominal-state-observation-v1","duration_s":10,"push_time_s":2,"push_duration_s":.2,
              "control_step_s":config["dt"],"fall_angle_deg":45,
              "recovery_thresholds":{"tilt_deg":1,"position_m":.03,"speed_m_s":.03,"final_dwell_s":1},
              "axes":{k:sorted({r[k] for r in records}) for k in keys},
              "config_sha256":hashlib.sha256(config_text.encode()).hexdigest(),
              "source_sha256":{name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                                for name in ("balance.py","run_robustness.py")},
              "python":platform.python_version(),"config":config}
    payload={"metadata":metadata,"summary":summarize(records),"audit":audit,"records":records}
    (output/"results.json").write_text(json.dumps(payload,indent=2)+"\n",encoding="utf-8")
    with (output/"results.csv").open('w',newline='',encoding='utf-8') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)
    # A standalone report with embedded data also works directly from file://.
    try:
        replay=os.path.relpath(ROOT/"index.html",output).replace('\\','/')
    except ValueError:
        replay=(ROOT/"index.html").as_uri()
    template=(ROOT/"templates"/"robustness-report.html").read_text(encoding='utf-8')
    report=template.replace('DATA_PAYLOAD',json.dumps(payload,separators=(',',':')).replace('</','<\\/'))
    report=report.replace('REPLAY_URL_JSON',json.dumps(replay))
    (output/"report.html").write_text(report,encoding='utf-8')
    return payload


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'results'/'robustness')
    args=parser.parse_args()
    config=json.loads((ROOT/'config.json').read_text())
    total=len(TILTS)*len(PUSHES)*len(TORQUES)*len(CONTROLLERS)
    print(f'Running {total} trials: {len(TILTS)} tilts x {len(PUSHES)} signed pushes x {len(TORQUES)} torque limits x 2 controllers.',flush=True)
    records=run_grid(config,progress=lambda torque,n:print(f'Torque {torque:.2f} N m complete: {n}/{total}',flush=True))
    print('Auditing 32 sampled boundary cells with half-sized physics steps; control remains 200 Hz.',flush=True)
    audit=audit_boundaries(config,records)
    payload=save_report(args.output,config,records,audit)
    for summary in payload['summary']:print(summary)
    print(f"Audit: {audit['audited']} trials, {audit['disagreements']} outcome disagreements.")
    print('Open:',(args.output/'report.html').resolve())


if __name__=='__main__':main()
