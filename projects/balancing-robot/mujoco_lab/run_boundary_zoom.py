"""Exploratory fixed-resolution zoom of every observed coarse transition."""
import argparse
from collections import defaultdict
import hashlib
import json
import time
import numpy as np
import run_extension as run
import extension_core as core

ROOT, HERE = core.ROOT, core.HERE
Z = json.loads((HERE/'boundary_zoom_protocol.json').read_text())


def selected_cases(source):
    cases = {c['case_id']: c for c in source['cases']}
    groups = defaultdict(list)
    for pair in source['selected_pairs']:
        c = cases[pair['case_id']]
        value = c['push_force_n'] if c['scenario']=='force' else c['initial_tilt_deg']
        groups[c['scenario'], c['torque_limit_nm'], int(np.sign(value))].append((abs(value), c, pair))
    selected, intervals = {}, []
    for (scenario, limit, sign), group in sorted(groups.items()):
        ordered = sorted(group, key=lambda x: x[0])
        for left, right in zip(ordered, ordered[1:]):
            if all(left[2][k]==right[2][k] for k in ['planar_outcome', 'native_outcome']): continue
            intervals.append(dict(scenario=scenario, torque_limit_nm=limit, sign=sign, lower=left[0], upper=right[0]))
            for j in range(11):
                value = round(sign*(left[0]+j*(right[0]-left[0])/10), 8)
                c = dict(left[1])
                c['push_force_n' if scenario=='force' else 'initial_tilt_deg'] = value
                c['parent_interval'] = [left[0], right[0]]
                selected[scenario, limit, value] = c
    cases = list(selected.values())
    for i, c in enumerate(cases): c['case_id'] = f'zoom_{i:03d}'
    return cases, intervals


def main(workers):
    start = time.perf_counter()
    source = json.loads((ROOT/Z['source']).read_text())
    cases, intervals = selected_cases(source)
    hashes = run.sources()
    for f in ['mujoco_lab/run_boundary_zoom.py', 'mujoco_lab/boundary_zoom_protocol.json', Z['source']]:
        hashes[f] = hashlib.sha256((ROOT/f).read_bytes()).hexdigest()
    fingerprint = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()[:20]
    found = run.run_batch([dict(c, controller='fixed', engine=e, physics_step_s=h, round='zoom_base')
                           for c in cases for h in Z['physics_steps_s'] for e in ['planar', 'native']], fingerprint, workers, 'boundary zoom')
    def index(): return {(r['case_id'], r['engine'], r['physics_step_s']): (r, path) for r, path in found}
    lookup = index()
    flagged = [c for c in cases if len({lookup[c['case_id'], e, h][0]['outcome'] for e in ['planar', 'native'] for h in Z['physics_steps_s']}) > 1]
    if flagged:
        found.extend(run.run_batch([dict(c, controller='fixed', engine=e, physics_step_s=.0000625, round='zoom_audit_1') for c in flagged for e in ['planar', 'native']], fingerprint, workers, 'zoom first audit'))
    lookup = index()
    again = [c for c in flagged if any(lookup[c['case_id'], e, .000125][0]['outcome'] != lookup[c['case_id'], e, .0000625][0]['outcome'] for e in ['planar', 'native'])]
    if again:
        found.extend(run.run_batch([dict(c, controller='fixed', engine=e, physics_step_s=.00003125, round='zoom_audit_2') for c in again for e in ['planar', 'native']], fingerprint, workers, 'zoom second audit'))
    found.sort(key=lambda item: (item[0]['case_id'], item[0]['engine'], -item[0]['physics_step_s']))
    lookup = index()
    records = [r for r, _ in found]
    arrays = {r['id']: np.load(path, allow_pickle=False) for r, path in found}
    pairs, selected, refinement = [], [], []
    for c in cases:
        candidates = []
        for h in [*Z['physics_steps_s'], *Z['audit_steps_s']]:
            if (c['case_id'], 'planar', h) not in lookup: continue
            a, b = [lookup[c['case_id'], e, h][0] for e in ['planar', 'native']]
            diff = core.previous.difference(arrays[b['id']], arrays[a['id']])
            diff.update(case_id=c['case_id'], controller='fixed', physics_step_s=h, planar_id=a['id'], native_id=b['id'], planar_outcome=a['outcome'], native_outcome=b['outcome'], same_outcome=a['outcome']==b['outcome'], same_fall_label=(a['outcome']=='fallen')==(b['outcome']=='fallen'), fall_time_difference_s=b['duration_s']-a['duration_s'] if a['outcome']==b['outcome']=='fallen' else None)
            candidates.append(diff)
        pairs.extend(candidates)
        selected.append(min(candidates, key=lambda r: r['physics_step_s']))
        for e in ['planar', 'native']:
            group = sorted([r for r in records if r['case_id']==c['case_id'] and r['engine']==e], key=lambda r: -r['physics_step_s'])
            for a, b in zip(group, group[1:]):
                diff = core.previous.difference(arrays[a['id']], arrays[b['id']])
                diff.update(case_id=c['case_id'], controller='fixed', engine=e, coarse_step_s=a['physics_step_s'], fine_step_s=b['physics_step_s'], same_outcome=a['outcome']==b['outcome'], coarse_outcome=a['outcome'], fine_outcome=b['outcome'])
                refinement.append(diff)
    result = {'metadata': {'phase': 'boundary_zoom', 'protocol': core.PROTOCOL, 'zoom_protocol': Z, 'source_sha256': hashes,
                           'mujoco': run.mujoco.__version__, 'numpy': np.__version__, 'python': run.platform.python_version(),
                           'trajectory_columns': core.previous.COLUMNS, 'runtime_s': time.perf_counter()-start},
              'cases': cases, 'intervals': intervals, 'records': records, 'pairs': pairs, 'selected_pairs': selected, 'refinement': refinement,
              'audit': {'first_round_pairs': len(flagged), 'second_round_pairs': len(again)}}
    out = ROOT/'results/mujoco_boundary_zoom'; out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out/'trajectories.npz', **arrays)
    (out/'study.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'conditions': len(cases), 'intervals': len(intervals), 'integrations': len(records), 'outcome_disagreements': sum(not p['same_outcome'] for p in selected), 'audit': result['audit']}, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(); parser.add_argument('--workers', type=int, default=2)
    args = parser.parse_args(); main(args.workers)
