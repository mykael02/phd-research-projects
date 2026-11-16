"""Run the frozen boundary study, then the loaded-controller comparison."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import itertools
import json
import platform
import time
from pathlib import Path
import numpy as np
import mujoco
import extension_core as core

ROOT, HERE, PROTOCOL = core.ROOT, core.HERE, core.PROTOCOL
CACHE = HERE/'_qa/extension_cache'


def sources():
    names = ['balance.py', 'config.json', 'mujoco_lab/validate.py', 'mujoco_lab/extension_core.py',
             'mujoco_lab/run_extension.py', 'mujoco_lab/extension_protocol.json',
             'mujoco_lab/balancing_robot.xml', 'mujoco_lab/requirements.txt',
             'results/comparison/gain_design.json', 'comparison_protocol.json',
             'results/robustness/results.json']
    return {n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in names}


def conditions(phase):
    cases = []
    if phase == 'boundary':
        b = PROTOCOL['boundary']
        for row in b['force_slices']:
            count = round((row['positive_force_end_n']-row['positive_force_start_n'])/b['force_spacing_n'])+1
            for sign in b['force_signs']:
                for j in range(count):
                    force = sign*(row['positive_force_start_n']+j*b['force_spacing_n'])
                    cases.append(dict(scenario='force', initial_tilt_deg=0, push_force_n=force,
                                      torque_limit_nm=row['torque_limit_nm'], mass_kg=0, height_m=.18))
        s = b['startup_slice']
        for sign in s['signs']:
            for j in range(round((s['positive_tilt_end_deg']-s['positive_tilt_start_deg'])/s['spacing_deg'])+1):
                cases.append(dict(scenario='startup', initial_tilt_deg=sign*(s['positive_tilt_start_deg']+j*s['spacing_deg']),
                                  push_force_n=0, torque_limit_nm=s['torque_limit_nm'], mass_kg=0, height_m=.18))
    else:
        loads = [(0, .18)]+list(itertools.product(PROTOCOL['payload']['masses_kg'], PROTOCOL['payload']['heights_m']))
        for mass, height in loads:
            for s in PROTOCOL['payload']['scenarios']:
                cases.append(dict(scenario=s['id'], initial_tilt_deg=s['initial_tilt_deg'], push_force_n=s['push_force_n'],
                                  torque_limit_nm=s['torque_limit_nm'], mass_kg=mass, height_m=height))
    for i, row in enumerate(cases): row['case_id'] = f'{phase}_{i:03d}'
    return cases


def work(job, fingerprint):
    encoded = json.dumps(job, sort_keys=True, separators=(',', ':')).encode()
    key = hashlib.sha256(fingerprint.encode()+encoded).hexdigest()
    folder = CACHE/fingerprint
    folder.mkdir(parents=True, exist_ok=True)
    jpath, npath = folder/f'{key}.json', folder/f'{key}.npy'
    if jpath.exists() and npath.exists():
        return json.loads(jpath.read_text()), str(npath)
    metrics, values = core.execute(job)
    result = dict(job, **metrics, id=key[:20])
    np.save(npath, values, allow_pickle=False)
    jpath.write_text(json.dumps(result, indent=2)+'\n')
    return result, str(npath)


def run_batch(jobs, fingerprint, workers, label):
    found = []
    started = time.perf_counter()
    with ProcessPoolExecutor(max_workers=workers) as pool:
        pending = [pool.submit(work, j, fingerprint) for j in jobs]
        for future in as_completed(pending):
            found.append(future.result())
            if len(found)%20 == 0 or len(found) == len(jobs):
                print(f'{label}: {len(found)}/{len(jobs)} integrations ({time.perf_counter()-started:.1f} s)', flush=True)
    return found


def main(phase, workers):
    started = time.perf_counter()
    out = ROOT/f'results/mujoco_{phase}'
    out.mkdir(parents=True, exist_ok=True)
    hashes = sources()
    fingerprint = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()[:20]
    checks = core.verify_loads()
    print(f'Compiled load and force checks passed: {len(checks)} loads, {sum(r["acceleration_checks"] for r in checks)} acceleration cases.', flush=True)
    cases = conditions(phase)
    controllers = ['fixed'] if phase == 'boundary' else ['fixed', 'exact']
    jobs = [dict(c, controller=controller, engine=engine, physics_step_s=step, round='base')
            for c in cases for controller in controllers for step in PROTOCOL['physics_steps_s'] for engine in ['planar', 'native']]
    found = run_batch(jobs, fingerprint, workers, phase+' base')
    def idx():
        return {(r['case_id'], r['controller'], r['engine'], r['physics_step_s']): (r, path) for r, path in found}
    lookup = idx()
    flagged = []
    for c in cases:
        for controller in controllers:
            group = [lookup[c['case_id'], controller, e, h][0] for e in ['planar', 'native'] for h in PROTOCOL['physics_steps_s']]
            if len({r['outcome'] for r in group}) > 1:
                flagged.append((c, controller))
    audit_jobs = [dict(c, controller=v, engine=e, physics_step_s=.000125, round='audit_1')
                  for c, v in flagged for e in ['planar', 'native']]
    if audit_jobs: found.extend(run_batch(audit_jobs, fingerprint, workers, phase+' first audit'))
    lookup = idx()
    again = [(c, v) for c, v in flagged if any(lookup[c['case_id'], v, e, .00025][0]['outcome'] != lookup[c['case_id'], v, e, .000125][0]['outcome'] for e in ['planar', 'native'])]
    extra_jobs = [dict(c, controller=v, engine=e, physics_step_s=.0000625, round='audit_2')
                  for c, v in again for e in ['planar', 'native']]
    if extra_jobs: found.extend(run_batch(extra_jobs, fingerprint, workers, phase+' second audit'))
    if phase == 'payload':
        verify_jobs = [dict(c, controller=v, engine='reduced', physics_step_s=.0005, round='equation_trajectory')
                       for c in cases if c['scenario']=='opposing_standard' for v in controllers]
        found.extend(run_batch(verify_jobs, fingerprint, workers, 'loaded ideal-rolling check'))
    found.sort(key=lambda item: (item[0]['case_id'], item[0]['controller'], item[0]['engine'], -item[0]['physics_step_s']))
    lookup = idx()
    records = [r for r, _ in found]
    trajectories = {r['id']: np.load(path, allow_pickle=False) for r, path in found}
    pairs = []
    for c in cases:
        for v in controllers:
            keys = [(c['case_id'], v, 'planar', h) for h in [.0005, .00025, .000125, .0000625]]
            for key in keys:
                if key not in lookup: continue
                original = lookup[key][0]
                native = lookup[key[0], key[1], 'native', key[3]][0]
                diff = core.previous.difference(trajectories[native['id']], trajectories[original['id']])
                diff.update(case_id=c['case_id'], controller=v, physics_step_s=key[3],
                            planar_id=original['id'], native_id=native['id'],
                            planar_outcome=original['outcome'], native_outcome=native['outcome'],
                            same_outcome=original['outcome']==native['outcome'],
                            same_fall_label=(original['outcome']=='fallen')==(native['outcome']=='fallen'),
                            fall_time_difference_s=native['duration_s']-original['duration_s'] if original['outcome']==native['outcome']=='fallen' else None)
                pairs.append(diff)
    selected = []
    for c in cases:
        for v in controllers:
            candidates = [p for p in pairs if p['case_id']==c['case_id'] and p['controller']==v]
            selected.append(min(candidates, key=lambda p: p['physics_step_s']))
    refinement = []
    for c in cases:
        for v in controllers:
            for e in ['planar', 'native']:
                candidates = sorted([r for r in records if r['case_id']==c['case_id'] and r['controller']==v and r['engine']==e], key=lambda r: -r['physics_step_s'])
                for a, b in zip(candidates, candidates[1:]):
                    diff = core.previous.difference(trajectories[a['id']], trajectories[b['id']])
                    diff.update(case_id=c['case_id'], controller=v, engine=e, coarse_step_s=a['physics_step_s'], fine_step_s=b['physics_step_s'],
                                same_outcome=a['outcome']==b['outcome'], coarse_outcome=a['outcome'], fine_outcome=b['outcome'])
                    refinement.append(diff)
    reduced_checks = []
    for r in records:
        if r['engine'] != 'reduced': continue
        original = lookup[r['case_id'], r['controller'], 'planar', .0005][0]
        diff = core.previous.difference(trajectories[r['id']], trajectories[original['id']])
        diff.update(case_id=r['case_id'], controller=r['controller'], same_outcome=r['outcome']==original['outcome'])
        assert diff['same_outcome'] and max(diff['max_state_errors_native_units']) < 1e-7
        reduced_checks.append(diff)
    # Zero-load fixed and exact gains must produce identical trajectories in each engine.
    zero_checks = []
    if phase == 'payload':
        for c in cases:
            if c['mass_kg'] != 0: continue
            for e in ['planar', 'native']:
                for h in PROTOCOL['physics_steps_s']:
                    a, b = [lookup[c['case_id'], v, e, h][0] for v in controllers]
                    assert np.array_equal(trajectories[a['id']], trajectories[b['id']])
                    zero_checks.append(dict(case_id=c['case_id'], engine=e, physics_step_s=h, result='bitwise identical'))
    result = {'metadata': {'phase': phase, 'protocol': PROTOCOL, 'source_sha256': hashes,
                           'mujoco': mujoco.__version__, 'numpy': np.__version__, 'python': platform.python_version(),
                           'trajectory_columns': core.previous.COLUMNS, 'workers': workers, 'runtime_s': time.perf_counter()-started},
              'cases': cases, 'load_checks': checks, 'records': records, 'pairs': pairs, 'selected_pairs': selected,
              'refinement': refinement, 'reduced_checks': reduced_checks, 'zero_load_checks': zero_checks,
              'audit': {'first_round_pairs': len(flagged), 'second_round_pairs': len(again)}}
    np.savez_compressed(out/'trajectories.npz', **trajectories)
    (out/'study.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'phase': phase, 'conditions': len(cases), 'integrations': len(records),
                      'selected_pairs': len(selected), 'outcome_disagreements': sum(not p['same_outcome'] for p in selected),
                      'audit': result['audit'], 'runtime_s': result['metadata']['runtime_s']}, indent=2), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('phase', choices=['boundary', 'payload'])
    parser.add_argument('--workers', type=int, default=2)
    args = parser.parse_args()
    if not 1 <= args.workers <= 4: parser.error('Use 1 to 4 worker processes')
    main(args.phase, args.workers)
