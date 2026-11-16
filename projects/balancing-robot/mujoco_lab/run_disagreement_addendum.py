"""Post hoc consistency check of the chassis-inertia explanation.

Written after the main diagnostics. If the extra chassis pitch inertia is why
two cells disagree at 0.5 N m, the same four models should (a) show a similar
gap at the other two force boundaries and (b) reproduce every archived zoom
label there, including the archived agreement. Nothing is fitted.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import platform
import time
import numpy as np
import mujoco
import hostperf
import run_disagreement as rd

ROOT, HERE = rd.ROOT, rd.HERE
SPEC = {
    'study': 'mujoco-boundary-disagreement-addendum-v1',
    'date': '2026-11-13',
    'status': 'Post hoc. Designed after the main diagnostics identified chassis pitch inertia; exploratory consistency check, not a confirmatory test.',
    'variants': ['planar', 'native', 'planar_plus_chassis', 'chassis_removed'],
    'boundaries': [{'torque_limit_nm': 0.3, 'low_n': 10.0, 'high_n': 13.5}, {'torque_limit_nm': 0.5, 'low_n': 17.0, 'high_n': 19.5},
                   {'torque_limit_nm': 0.7, 'low_n': 20.0, 'high_n': 23.5}],
    'halvings': 12,
    'physics_step_s': rd.STEP,
    'prediction': 'For every archived zoom force in the slice, a model falls exactly when the force is at or above its first-fallen bracket end. Compared with the archived finest-step labels of the planar and contact models.',
    'scope': 'Force slices only (zero initial tilt). The unforced start-up slice at 0.1 N m is not covered.'}
SOURCES = rd.SOURCES+['mujoco_lab/run_disagreement_addendum.py', 'results/mujoco_boundary_zoom/study.json']


def bracket(args):
    variant, limit, low, high = args
    rd.LIMIT = limit  # the trial loop, torque clamp and motor range all read this module value
    def fallen(force):
        return rd.execute(dict(variant=variant, physics_step_s=rd.STEP, push_force_n=force))[0]['outcome'] == 'fallen'
    assert not fallen(low) and fallen(high), (variant, limit)
    for _ in range(SPEC['halvings']):
        mid = (low+high)/2
        if fallen(mid): high = mid
        else: low = mid
    return dict(variant=variant, torque_limit_nm=limit, last_not_fallen_n=low, first_fallen_n=high, midpoint_n=(low+high)/2, width_n=high-low)


def main(workers):
    started = time.perf_counter()
    hashes = {n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in SOURCES}
    jobs = [(v, b['torque_limit_nm'], b['low_n'], b['high_n']) for b in SPEC['boundaries'] for v in SPEC['variants']]
    with ProcessPoolExecutor(max_workers=workers, initializer=hostperf.full_speed) as pool:
        brackets = list(pool.map(bracket, jobs))
    zoom = json.loads((ROOT/'results/mujoco_boundary_zoom/study.json').read_text())
    cases = {c['case_id']: c for c in zoom['cases']}
    main_study = json.loads((ROOT/'results/mujoco_disagreement/study.json').read_text())
    boundaries = []
    for b in SPEC['boundaries']:
        limit = b['torque_limit_nm']
        found = {r['variant']: r for r in brackets if r['torque_limit_nm'] == limit}
        if limit == rd.PROTOCOL['torque_limit_nm']:
            for name in SPEC['variants']:
                reference = next(x for x in main_study['brackets'] if x['variant'] == name)
                assert (found[name]['last_not_fallen_n'], found[name]['first_fallen_n']) == (reference['last_not_fallen_n'], reference['first_fallen_n']), name
        checks, wrong = [], 0
        for pair in zoom['selected_pairs']:
            c = cases[pair['case_id']]
            if c['scenario'] != 'force' or c['torque_limit_nm'] != limit: continue
            force = abs(c['push_force_n'])
            predicted = {e: force >= found[e]['first_fallen_n'] for e in ['planar', 'native']}
            archived = {e: pair[e+'_outcome'] == 'fallen' for e in ['planar', 'native']}
            ok = predicted == archived
            wrong += not ok
            checks.append(dict(push_force_n=c['push_force_n'], predicted_fallen=predicted, archived_fallen=archived, matches=ok))
        low, high = sorted([found['native']['midpoint_n'], found['planar']['midpoint_n']])
        grid = sorted({abs(c['push_force_n']) for c in checks})
        boundaries.append(dict(torque_limit_nm=limit, brackets=found,
                               gap_n=found['planar']['midpoint_n']-found['native']['midpoint_n'],
                               gap_percent=100*(found['planar']['midpoint_n']-found['native']['midpoint_n'])/found['planar']['midpoint_n'],
                               gap_with_inertia_matched_n=found['planar']['midpoint_n']-found['chassis_removed']['midpoint_n'],
                               planar_shift_from_added_inertia_n=found['planar_plus_chassis']['midpoint_n']-found['planar']['midpoint_n'],
                               contact_shift_from_chassis_inertia_n=found['native']['midpoint_n']-found['chassis_removed']['midpoint_n'],
                               archived_grid_forces_inside_gap_n=[f for f in grid if low < f < high],
                               archived_zoom_cells=len(checks), archived_labels_reproduced=len(checks)-wrong,
                               archived_disagreements=sum(c['archived_fallen']['planar'] != c['archived_fallen']['native'] for c in checks), checks=checks))
    result = {'metadata': {'specification': SPEC, 'source_sha256': hashes, 'mujoco': mujoco.__version__, 'numpy': np.__version__,
                           'python': platform.python_version(), 'workers': workers, 'runtime_s': time.perf_counter()-started,
                           'integrations': len(jobs)*(SPEC['halvings']+2)},
              'boundaries': boundaries}
    (ROOT/'results/mujoco_disagreement/addendum.json').write_text(json.dumps(result, indent=2)+'\n')
    for b in boundaries:
        print(f"limit {b['torque_limit_nm']:g} N m: planar {b['brackets']['planar']['midpoint_n']:.4f}, contact {b['brackets']['native']['midpoint_n']:.4f}, gap {b['gap_n']*1000:+.1f} mN, "
              f"inertia matched {b['gap_with_inertia_matched_n']*1000:+.1f} mN, grid forces inside gap {b['archived_grid_forces_inside_gap_n']}, "
              f"archived labels reproduced {b['archived_labels_reproduced']}/{b['archived_zoom_cells']}, archived disagreements {b['archived_disagreements']}", flush=True)
    print(json.dumps({'integrations': result['metadata']['integrations'], 'runtime_s': result['metadata']['runtime_s']}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--workers', type=int, default=8)
    main(parser.parse_args().workers)
