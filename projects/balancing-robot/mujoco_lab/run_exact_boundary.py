"""Exact-feedback fall boundary of both gains at the noisy conditions (post hoc).

Added after an internal review of manuscript version 2. The exact-feedback
control (run_noisy_controls.py) gives one fall label per gain and condition, at
the noisy scenarios' 12 N push. A label cannot show how far that push is from
each gain's fall boundary. This study measures the boundary directly: for every
load, both noisy scenarios, both gains and all three plants, bisection over the
push magnitude (sign as in the scenario, 0 to 40 N, 12 halvings, 9.8 mN) finds
the smallest push that produces a fall with exact state feedback (zero noise,
zero delay). Everything else is as in the noisy study: initial tilt, torque
limit, pulse timing, gains, fall and recovery rules.

Planar runs also get a 1 N scan from 0 to 40 N to check for a single fall
transition, and every plant must reproduce the archived exact-feedback label
at 12 N from controls.json.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import itertools
import json
import platform
import time
import mujoco
import numpy as np
import hostperf
import run_noisy_payload as rn

ROOT = rn.ROOT
IDEAL = {'id': 'ideal', 'mode': 'direct', 'angleNoiseDeg': 0, 'gyroNoiseDegS': 0, 'delayMs': 0}
FILES = rn.FILES+['mujoco_lab/run_exact_boundary.py']
LOW, HIGH, HALVINGS = 0.0, 40.0, 12
OUT = ROOT/'results'/'mujoco_noisy_payload'/'exact_boundary.json'


def falls(engine, scenario, mass, height, controller, magnitude):
    sign = 1 if scenario['push_force_n'] > 0 else -1
    job = dict(engine_id=engine['id'], engine=engine['engine'], physics_step_s=engine['physics_step_s'], mass_kg=mass, height_m=height,
               scenario=scenario['id'], initial_tilt_deg=scenario['initial_tilt_deg'], push_force_n=sign*magnitude,
               torque_limit_nm=scenario['torque_limit_nm'], profile='ideal', profile_settings=IDEAL, controller=controller, seed=0, seed_index=0)
    return rn.trial(job)[0]['outcome'] == 'fallen'


def bracket(args):
    engine, scenario, mass, height, controller = args
    lo, hi, trace = LOW, HIGH, []
    for x in (lo, hi):
        trace.append([x, falls(engine, scenario, mass, height, controller, x)])
    if trace[0][1] or not trace[1][1]:
        lo = hi = None
    else:
        for _ in range(HALVINGS):
            mid = (lo+hi)/2
            fell = falls(engine, scenario, mass, height, controller, mid)
            trace.append([mid, fell])
            if fell: hi = mid
            else: lo = mid
    scan = None
    if engine['engine'] == 'planar':
        scan = [falls(engine, scenario, mass, height, controller, float(x)) for x in range(int(LOW), int(HIGH)+1)]
    return dict(engine_id=engine['id'], scenario=scenario['id'], mass_kg=mass, height_m=height, controller=controller,
                last_not_fallen_n=lo, first_fallen_n=hi, critical_push_n=None if lo is None else (lo+hi)/2, trace=trace,
                scan_single_transition=None if scan is None else sum(a != b for a, b in zip(scan, scan[1:])) == 1)


def main(workers):
    started = time.perf_counter()
    hashes = {n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES}
    controls = json.loads((ROOT/'results'/'mujoco_noisy_payload'/'controls.json').read_text())
    archived = {(r['engine_id'], r['scenario'], r['mass_kg'], r['height_m'], r['controller']): r['outcome'] == 'fallen'
                for r in controls['ideal_control']['records']}
    jobs = [(e, s, m, h, c) for e in rn.PROTOCOL['engines'] for s in rn.SOURCE['sensing_test']
            for m, h in itertools.product(rn.SOURCE['test_masses_kg'], rn.SOURCE['test_heights_m']) for c in rn.PROTOCOL['controllers']]
    with ProcessPoolExecutor(max_workers=workers, initializer=hostperf.full_speed) as pool:
        records = list(pool.map(bracket, jobs))
    push = {s['id']: abs(s['push_force_n']) for s in rn.SOURCE['sensing_test']}
    for r in records:
        assert r['critical_push_n'] is not None, r
        assert r['scan_single_transition'] in (None, True), r
        key = (r['engine_id'], r['scenario'], r['mass_kg'], r['height_m'], r['controller'])
        assert (r['first_fallen_n'] <= push[r['scenario']]) == archived[key] or r['last_not_fallen_n'] < push[r['scenario']] < r['first_fallen_n'], key
        r['margin_at_noisy_push_n'] = r['critical_push_n']-push[r['scenario']]
    cells = []
    index = {(r['engine_id'], r['scenario'], r['mass_kg'], r['height_m'], r['controller']): r for r in records}
    for e, s, m, h in sorted({k[:4] for k in index}):
        f, a = index[e, s, m, h, 'fixed']['critical_push_n'], index[e, s, m, h, 'exact']['critical_push_n']
        cells.append(dict(engine_id=e, scenario=s, mass_kg=m, height_m=h, fixed_critical_n=f, exact_critical_n=a,
                          shift_n=a-f, shift_percent=100*(a-f)/f, noisy_push_n=push[s]))
    result = {'metadata': {'status': 'Post hoc, added after an internal review of manuscript version 2 and after every noisy result was known. Deterministic; no tuning.',
                           'source_sha256': hashes, 'mujoco': mujoco.__version__, 'numpy': np.__version__, 'python': platform.python_version(),
                           'bracket_n': [LOW, HIGH], 'halvings': HALVINGS, 'width_n': (HIGH-LOW)/2**HALVINGS,
                           'integrations': sum(len(r['trace'])+(41 if r['scan_single_transition'] is not None else 0) for r in records),
                           'elapsed_s': round(time.perf_counter()-started, 1)},
              'records': records, 'cells': cells}
    OUT.write_text(json.dumps(result, indent=1)+'\n')
    for c in cells:
        print(f"{c['engine_id']:14} {c['scenario']:17} {c['mass_kg']:.2f} kg {c['height_m']:.2f} m  fixed {c['fixed_critical_n']:6.2f}  aware {c['exact_critical_n']:6.2f}  {c['shift_percent']:+5.1f}%")
    print(f"{result['metadata']['integrations']} integrations in {result['metadata']['elapsed_s']} s -> {OUT}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--workers', type=int, default=8)
    main(parser.parse_args().workers)
