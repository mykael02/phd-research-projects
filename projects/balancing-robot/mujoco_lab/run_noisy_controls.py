"""Two controls for the noisy payload comparison, added after its results were known.

1. Ideal-feedback control. The archived ideal-sensing null (216 planar and 36
   contact conditions) used different tilts, forces and limits from the two
   noisy scenarios. This runs the noisy scenarios themselves, at all nine loads,
   with exact state feedback (zero noise, zero delay), in the planar model and
   both contact replicates. The planar runs are also checked against the
   archived JavaScript trial function.
2. Repeatability. Twelve native condition groups (360 trials) are recomputed
   from scratch in fresh processes and compared field by field with study.json,
   because the checkpoint cache was filled by more than one process.
3. Illustration. One noise-induced fall that the redesign prevents, chosen by a
   stated rule (not by inspection) and re-run with full trajectories.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import itertools
import json
import platform
import subprocess
import time
import mujoco
import numpy as np
import hostperf
import run_noisy_payload as rn

ROOT = rn.ROOT
IDEAL = {'id': 'ideal', 'mode': 'direct', 'angleNoiseDeg': 0, 'gyroNoiseDegS': 0, 'delayMs': 0}
FILES = rn.FILES+['mujoco_lab/run_noisy_controls.py', 'run_comparison.js', 'visualisation/payload-lqr.js', 'visualisation/payload-gains.js', 'model.js']
KEYS = ['engine_id', 'mass_kg', 'height_m', 'scenario', 'profile', 'seed', 'controller']


def one(job):
    metrics, _ = rn.trial(job)
    return dict({k: v for k, v in job.items() if k != 'profile_settings'}, **metrics)


def ideal_jobs():
    jobs = []
    for e in rn.PROTOCOL['engines']:
        for m, h in itertools.product(rn.SOURCE['test_masses_kg'], rn.SOURCE['test_heights_m']):
            for s in rn.SOURCE['sensing_test']:
                for c in rn.PROTOCOL['controllers']:
                    jobs.append(dict(engine_id=e['id'], engine=e['engine'], physics_step_s=e['physics_step_s'], mass_kg=m, height_m=h,
                                     scenario=s['id'], initial_tilt_deg=s['initial_tilt_deg'], push_force_n=s['push_force_n'],
                                     torque_limit_nm=s['torque_limit_nm'], profile='ideal', profile_settings=IDEAL, controller=c, seed=0, seed_index=0))
    return jobs


def javascript_reference():
    script = ("const {trial}=require('./run_comparison.js');const p=require('./comparison_protocol.json');const out=[];"
              "for(const m of p.test_masses_kg)for(const h of p.test_heights_m)for(const s of p.sensing_test)for(const v of ['fixed','exact']){"
              "const {id,...condition}=s;const r=trial(v,condition,{mass_kg:m,height_m:h}).metrics;"
              "out.push({mass_kg:m,height_m:h,scenario:id,controller:v,outcome:r.outcome,peak_tilt_deg:r.peak_tilt_deg,duration_s:r.duration_s,torque_squared_integral:r.torque_squared_integral});}"
              "console.log(JSON.stringify(out));")
    return json.loads(subprocess.check_output(['node', '-e', script], cwd=ROOT, text=True))


def summarise(records):
    out = {}
    for e in rn.PROTOCOL['engines']:
        rows = {(r['mass_kg'], r['height_m'], r['scenario'], r['controller']): r for r in records if r['engine_id'] == e['id']}
        cells = sorted({k[:3] for k in rows})
        pairs = [(rows[k+('fixed',)], rows[k+('exact',)]) for k in cells]
        out[e['id']] = dict(cells=len(pairs), outcomes={c: dict(sorted(__import__('collections').Counter(p[i]['outcome'] for p in pairs).items())) for i, c in enumerate(['fixed', 'exact'])},
                            fixed_falls=sum(a['outcome'] == 'fallen' for a, _ in pairs), exact_falls=sum(b['outcome'] == 'fallen' for _, b in pairs),
                            different_fall_labels=sum((a['outcome'] == 'fallen') != (b['outcome'] == 'fallen') for a, b in pairs),
                            different_outcomes=sum(a['outcome'] != b['outcome'] for a, b in pairs),
                            max_peak_tilt_deg=max(max(a['peak_tilt_deg'], b['peak_tilt_deg']) for a, b in pairs),
                            max_saturation_s=max(max(a['saturation_s'], b['saturation_s']) for a, b in pairs),
                            by_scenario={s: dict(fixed_falls=sum(a['outcome'] == 'fallen' for (a, b), k in zip(pairs, cells) if k[2] == s),
                                                 exact_falls=sum(b['outcome'] == 'fallen' for (a, b), k in zip(pairs, cells) if k[2] == s),
                                                 median_peak_tilt_fixed_deg=float(np.median([a['peak_tilt_deg'] for (a, b), k in zip(pairs, cells) if k[2] == s])),
                                                 median_peak_tilt_exact_deg=float(np.median([b['peak_tilt_deg'] for (a, b), k in zip(pairs, cells) if k[2] == s])))
                                         for s in sorted({k[2] for k in cells})})
    return out


def main(workers):
    started = time.perf_counter()
    hashes = {n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES}
    study = json.loads((ROOT/'results/mujoco_noisy_payload/study.json').read_text())
    for name, digest in study['metadata']['source_sha256'].items():
        assert hashes[name] == digest, f'{name} changed since the noisy study was run'
    jobs = ideal_jobs()
    profiles = {p['id']: p for p in rn.SOURCE['profiles']}
    native_groups = sorted({tuple(r[k] for k in ['engine_id', 'mass_kg', 'height_m', 'scenario', 'profile', 'controller'])
                            for r in study['records'] if r['engine_id'] != 'planar_port'})
    chosen = native_groups[::18]
    engines = {e['id']: e for e in rn.PROTOCOL['engines']}
    scenarios = {s['id']: s for s in rn.SOURCE['sensing_test']}
    repeat_jobs = []
    for engine_id, m, h, s, f, c in chosen:
        for i in range(rn.SOURCE['repetitions']):
            sc = scenarios[s]
            repeat_jobs.append(dict(engine_id=engine_id, engine='native', physics_step_s=engines[engine_id]['physics_step_s'], mass_kg=m, height_m=h, scenario=s,
                                    initial_tilt_deg=sc['initial_tilt_deg'], push_force_n=sc['push_force_n'], torque_limit_nm=sc['torque_limit_nm'],
                                    profile=f, profile_settings=profiles[f], controller=c, seed=5003+104729*i, seed_index=i))
    with ProcessPoolExecutor(max_workers=workers, initializer=hostperf.full_speed) as pool:
        ideal = list(pool.map(one, jobs))
        print(f'Ideal-feedback control: {len(ideal)} runs ({time.perf_counter()-started:.0f} s)', flush=True)
        repeats = list(pool.map(one, repeat_jobs))
        print(f'Repeatability: {len(repeats)} runs ({time.perf_counter()-started:.0f} s)', flush=True)

    reference = {(r['mass_kg'], r['height_m'], r['scenario'], r['controller']): r for r in javascript_reference()}
    planar = [r for r in ideal if r['engine_id'] == 'planar_port']
    js_check = dict(records=len(planar), outcome_matches=sum(r['outcome'] == reference[r['mass_kg'], r['height_m'], r['scenario'], r['controller']]['outcome'] for r in planar),
                    max_peak_tilt_difference_deg=max(abs(r['peak_tilt_deg']-reference[r['mass_kg'], r['height_m'], r['scenario'], r['controller']]['peak_tilt_deg']) for r in planar))
    assert js_check['outcome_matches'] == len(planar) == 36 and js_check['max_peak_tilt_difference_deg'] < 1e-6, js_check

    stored = {tuple(r[k] for k in KEYS): r for r in study['records']}
    fields = [k for k in repeats[0] if k not in ('profile_settings',)]
    mismatched = [tuple(r[k] for k in KEYS) for r in repeats if any(r[k] != stored[tuple(r[k] for k in KEYS)][k] for k in fields)]
    repeatability = dict(groups=len(chosen), trials=len(repeats), fields_compared=len(fields), identical=len(repeats)-len(mismatched),
                         mismatches=[list(m) for m in mismatched], selection='Every 18th of the 216 native condition groups in sorted order')
    assert not mismatched, mismatched[:5]

    # Illustration rule: among cells that both gains survive under exact feedback in every plant, take the cell
    # (load, scenario, profile) with the most falls prevented in the contact model at 0.25 ms; then the lowest seed
    # index at which the fixed gain falls and the payload-aware gain survives in both the planar and that contact run.
    outcome = {(r['engine_id'], r['mass_kg'], r['height_m'], r['scenario'], r['controller']): r['outcome'] for r in ideal}
    both_recover = lambda m, h, s: all(outcome[e['id'], m, h, s, c] == 'recovered' for e in rn.PROTOCOL['engines'] for c in rn.PROTOCOL['controllers'])
    fallen = {tuple(r[k] for k in KEYS): r['outcome'] == 'fallen' for r in study['records']}
    def prevented(engine_id, m, h, s, f, seed):
        return fallen[engine_id, m, h, s, f, seed, 'fixed'] and not fallen[engine_id, m, h, s, f, seed, 'exact']
    seeds = [5003+104729*i for i in range(rn.SOURCE['repetitions'])]
    cells = sorted({(r['mass_kg'], r['height_m'], r['scenario'], r['profile']) for r in study['records']})
    ranked = sorted([c for c in cells if both_recover(*c[:3])], key=lambda c: (-sum(prevented('native_0p25ms', *c, s) for s in seeds), c))
    m, h, s, f = ranked[0]
    index = next(i for i, seed in enumerate(seeds) if all(prevented(e, m, h, s, f, seed) for e in ['planar_port', 'native_0p25ms']))
    arrays, shown = {}, []
    for e in rn.PROTOCOL['engines'][:2]:
        for c in rn.PROTOCOL['controllers']:
            sc = scenarios[s]
            job = dict(engine_id=e['id'], engine=e['engine'], physics_step_s=e['physics_step_s'], mass_kg=m, height_m=h, scenario=s,
                       initial_tilt_deg=sc['initial_tilt_deg'], push_force_n=sc['push_force_n'], torque_limit_nm=sc['torque_limit_nm'],
                       profile=f, profile_settings=profiles[f], controller=c, seed=seeds[index], seed_index=index)
            metrics, rows = rn.trial(job, capture=True)
            assert metrics == {k: stored[e['id'], m, h, s, f, seeds[index], c][k] for k in metrics}
            arrays[f"{e['id']}__{c}"] = rows
            shown.append(dict(id=f"{e['id']}__{c}", engine_id=e['id'], controller=c, **metrics))
    np.savez_compressed(ROOT/'results/mujoco_noisy_payload/illustration.npz', **arrays)
    illustration = dict(rule='Cell (load, scenario, profile) with the most falls prevented in the contact model at 0.25 ms among cells that both gains survive under exact feedback in all three plants; then the lowest seed index at which the fixed gain falls and the payload-aware gain survives in both the planar and the 0.25 ms contact model.',
                        mass_kg=m, height_m=h, scenario=s, profile=f, seed_index=index, seed=seeds[index],
                        prevented_in_cell={e['id']: sum(prevented(e['id'], m, h, s, f, sd) for sd in seeds) for e in rn.PROTOCOL['engines']},
                        columns=rn.COLUMNS, runs=shown)

    result = {'metadata': {'status': 'Post hoc controls, designed after the noisy results were known. Deterministic; no tuning.',
                           'source_sha256': hashes, 'mujoco': mujoco.__version__, 'numpy': np.__version__, 'python': platform.python_version(),
                           'runtime_s': time.perf_counter()-started, 'workers': workers},
              'ideal_control': dict(profile=IDEAL, summary=summarise(ideal), javascript_check=js_check, records=ideal),
              'repeatability': repeatability, 'illustration': illustration}
    (ROOT/'results/mujoco_noisy_payload/controls.json').write_text(json.dumps(result, indent=2)+'\n')
    for name, s in result['ideal_control']['summary'].items():
        print(f"{name:14s} ideal feedback: fixed {s['outcomes']['fixed']}, aware {s['outcomes']['exact']}, different fall labels {s['different_fall_labels']}, "
              f"max peak tilt {s['max_peak_tilt_deg']:.1f} deg, max saturation {s['max_saturation_s']:.2f} s")
    print('JavaScript check:', js_check)
    print('Repeatability:', {k: v for k, v in repeatability.items() if k != 'mismatches'})
    print('Illustration:', {k: v for k, v in illustration.items() if k not in ('runs', 'columns', 'rule')})


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--workers', type=int, default=12)
    main(parser.parse_args().workers)
