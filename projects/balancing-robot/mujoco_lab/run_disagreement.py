"""Diagnose the two archived fall-boundary disagreements (+/-18.2 N, 0.5 N m).

One declared modelling assumption is changed per variant; nothing is fitted.
The archived plant, protocols and results are read-only.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
import platform
import time
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import extension_core as core

previous = core.previous
ROOT, HERE = core.ROOT, core.HERE
PROTOCOL = json.loads((HERE/'disagreement_protocol.json').read_text())
VARIANTS = PROTOCOL['variants']
DT, LIMIT = PROTOCOL['control_period_s'], PROTOCOL['torque_limit_nm']
STEP, FINE = PROTOCOL['physics_step_s'], PROTOCOL['confirmation_step_s']
OUT = ROOT/'results/mujoco_disagreement'
COLUMNS = previous.COLUMNS+['normal_force_n', 'friction_utilisation']
SOURCES = ['balance.py', 'config.json', 'mujoco_lab/validate.py', 'mujoco_lab/extension_core.py',
           'mujoco_lab/run_disagreement.py', 'mujoco_lab/disagreement_protocol.json',
           'mujoco_lab/balancing_robot.xml', 'results/comparison/gain_design.json']


def native_xml(v):
    root = ET.parse(HERE/'balancing_robot.xml').getroot()
    root.find('default/motor').set('ctrlrange', f'{-LIMIT/2} {LIMIT/2}')
    geom = root.find('default/geom')
    if 'friction' in v: geom.set('friction', f"{v['friction']} 0.005 0.0001")
    if 'time_constant_s' in v: geom.set('solref', f"{v['time_constant_s']} 1")
    if 'solimp' in v: geom.set('solimp', v['solimp'])
    if 'condim' in v: geom.set('condim', str(v['condim']))
    for key in ['cone', 'integrator', 'noslip_iterations']:
        if key in v: root.find('option').set(key, str(v[key]))
    base = root.find('.//body[@name="base"]')
    if 'initial_height_m' in v: base.set('pos', f"0 0 {v['initial_height_m']}")
    if 'chassis_inertia' in v: base.find('inertial').set('diaginertia', ' '.join([str(v['chassis_inertia'])]*3))
    if 'torso_pitch_inertia' in v:
        root.find('.//body[@name="torso"]/inertial').set('diaginertia', f"0.012 {v['torso_pitch_inertia']} 0.004")
    return ET.tostring(root, encoding='unicode')


class VariantEngine(core.LoadedEngine):
    """Unloaded native plant with one or more declared assumption changes."""
    def __init__(self, step, v):
        self.kind, self.step = 'native', step
        self.substeps = round(DT/step)
        assert abs(self.substeps*step-DT) < 1e-14
        self.p, _, _ = core.load_properties(0, .18, LIMIT)
        self.mass, self.payload = 0, None
        self.friction = v.get('friction', 1)
        self.model = mujoco.MjModel.from_xml_string(native_xml(v))
        self.model.opt.timestep = step
        self.data = mujoco.MjData(self.model)
        self.base, self.torso = [self.model.body(n).id for n in ['base', 'torso']]
        self.wheels = [self.model.body(n).id for n in ['left_wheel', 'right_wheel']]
        self.vel, self.wrench = np.zeros(6), np.zeros(6)
        assert abs(self.model.body_mass.sum()-1.55) < 1e-12

    def contact(self):
        normal, use = 0.0, 0.0
        for i in range(self.data.ncon):
            mujoco.mj_contactForce(self.model, self.data, i, self.wrench)
            normal += self.wrench[0]
            if self.wrench[0] > 1e-9:
                use = max(use, math.hypot(self.wrench[1], self.wrench[2])/(self.friction*self.wrench[0]))
        return [normal, use]


def execute(job):
    """Same trial loop as extension_core.execute, plus the declared start-up options."""
    v, step, force = dict(VARIANTS[job['variant']], **job.get('resolved', {})), job['physics_step_s'], job['push_force_n']
    p, _, _ = core.load_properties(0, .18, LIMIT)
    if 'planar_body_inertia' in v:
        p = previous.Parameters(**{**core.CONFIG['parameters'], 'body_inertia': v['planar_body_inertia'], 'torque_limit': LIMIT})
    k = core.gain('fixed', 0, .18)
    engine = VariantEngine(step, v) if v['engine'] == 'native' else None
    state = np.zeros(4)
    if engine:
        engine.reset(0)
        for _ in range(round(v.get('warmup_s', 0)/DT)):
            engine.advance(max(-LIMIT, min(LIMIT, -float(k@engine.state()))), 0)
    rows = []
    for tick in range(2001):
        if tick == 400:
            if engine and v.get('reset_at_push'):
                d = engine.data
                d.qpos[0:2] = 0; d.qpos[3:7] = [1, 0, 0, 0]; d.qvel[:] = 0
                mujoco.mj_forward(engine.model, d)
            if engine is None and 'injected_state' in v:
                state = np.array(v['injected_state'])
        if engine:
            state, extras = engine.state(), engine.extras()+engine.contact()
        else:
            extras = [0, p.radius, 0, 0, 0, 0, 0, 0, 0]
        assert np.all(np.isfinite(state)) and np.all(np.isfinite(extras))
        fallen = abs(state[2]) >= math.pi/4 or abs(extras[0]) >= math.pi/4
        terminal = fallen or tick == 2000
        push = force if 400 <= tick < 440 and not terminal else 0
        torque = max(-LIMIT, min(LIMIT, -float(k@state))) if not terminal else 0
        rows.append([tick*DT, *state, torque, push, *extras])
        if terminal: break
        if engine:
            engine.advance(torque, push)
        else:
            for _ in range(round(DT/step)):
                state = np.array(previous.rk4(state, torque, push, step, p))
    a = np.asarray(rows)
    result = previous.metrics(a, fallen, force)
    result['saturation_s'] = float(np.count_nonzero(np.abs(a[:-1, 5]) >= LIMIT-1e-12)*DT)
    result['fall_time_s'] = float(a[-1, 0]) if fallen else None
    result['pulse_onset_state'] = a[min(400, len(a)-1), 1:5].tolist()
    result['pulse_onset_axle_height_m'] = float(a[min(400, len(a)-1), 8])
    window = a[400:441] if len(a) > 440 else a[400:]
    result['pulse_max_slip_m_s'] = float(np.max(np.abs(window[:, 9:11]))) if len(window) else None
    result['pulse_max_friction_utilisation'] = float(window[:, 15].max()) if len(window) else None
    result['pulse_min_normal_force_n'] = float(window[:, 14].min()) if len(window) else None
    if engine:
        assert not any(int(w.number) for w in engine.data.warning)
    return dict(job, **result), a


def label(variant, step, force, resolved):
    return execute(dict(variant=variant, physics_step_s=step, push_force_n=force, resolved=resolved))[0]


def critical(args):
    """Bisection bracket [not fallen, fallen] on positive force."""
    variant, resolved = args
    b = PROTOCOL['primary']
    low, high, trace = 17.0, 19.5, []
    ends = [label(variant, STEP, f, resolved) for f in (low, high)]
    trace += [[f, r['outcome']] for f, r in zip((low, high), ends)]
    if ends[0]['outcome'] == 'fallen' or ends[1]['outcome'] != 'fallen':
        return dict(variant=variant, transition_in_bracket=False, trace=trace)
    for _ in range(12):
        mid = (low+high)/2
        outcome = label(variant, STEP, mid, resolved)['outcome']
        trace.append([mid, outcome])
        if outcome == 'fallen': high = mid
        else: low = mid
    return dict(variant=variant, transition_in_bracket=True, last_not_fallen_n=low, first_fallen_n=high,
                midpoint_n=(low+high)/2, width_n=high-low, trace=trace)


def one(args):
    variant, step, force, resolved, keep = args
    result, a = execute(dict(variant=variant, physics_step_s=step, push_force_n=force, resolved=resolved))
    return result, (a if keep else None)


def traction(a, p, force_sign):
    """Ground reaction the planar ideal-rolling model implicitly demands."""
    ratios = []
    for row in a[:-1]:
        s, u, f = row[1:5], row[5], row[6]
        _, ax, _, alpha = previous.derivative(s, u, f, p)
        sin, cos = math.sin(s[2]), math.cos(s[2])
        ml = p.body_mass*p.length
        horizontal = (p.base_mass+p.body_mass)*ax+ml*(alpha*cos-s[3]**2*sin)-f+p.translation_drag*s[1]
        normal = (p.base_mass+p.body_mass)*p.gravity-ml*(alpha*sin+s[3]**2*cos)
        ratios.append([row[0], horizontal, normal, abs(horizontal)/normal])
    r = np.array(ratios)
    i = int(np.argmax(r[:, 3]))
    return dict(peak_required_friction_coefficient=float(r[i, 3]), time_s=float(r[i, 0]),
                horizontal_ground_force_n=float(r[i, 1]), normal_force_n=float(r[i, 2]),
                samples_above_1=int(np.count_nonzero(r[:, 3] > 1)),
                seconds_above_1=float(np.count_nonzero(r[:, 3] > 1)*DT),
                min_normal_force_n=float(r[:, 2].min()))


def main(workers):
    started = time.perf_counter()
    OUT.mkdir(parents=True, exist_ok=True)
    hashes = {n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in SOURCES}

    # Stage 0: regression against the archived zoom trajectory, and measured start-up quantities.
    zoom = json.loads((ROOT/'results/mujoco_boundary_zoom/study.json').read_text())
    case = next(c for c in zoom['cases'] if c['scenario'] == 'force' and c['torque_limit_nm'] == .5 and c['push_force_n'] == 18.2)
    archived = np.load(ROOT/'results/mujoco_boundary_zoom/trajectories.npz')
    regression = []
    for engine, variant in [('planar', 'planar'), ('native', 'native')]:
        record = next(r for r in zoom['records'] if r['case_id'] == case['case_id'] and r['engine'] == engine and r['physics_step_s'] == STEP)
        result, a = execute(dict(variant=variant, physics_step_s=STEP, push_force_n=18.2))
        assert np.array_equal(a[:, :14], archived[record['id']]), engine
        regression.append(dict(engine=engine, archived_id=record['id'], outcome=result['outcome'], result='bitwise identical to archived zoom trajectory'))
    print('Archived trajectories reproduced bitwise:', [r['engine'] for r in regression], flush=True)

    resolved, startup = {}, {}
    for step in [STEP, FINE]:
        _, a = execute(dict(variant='native', physics_step_s=step, push_force_n=0))
        rest = float(a[300:400, 8].mean())
        startup[str(step)] = dict(resting_axle_height_m=rest, initial_axle_height_m=float(a[0, 8]),
                                  initial_drop_m=float(a[0, 8]-rest), pulse_onset_state=a[400, 1:5].tolist(),
                                  max_abs_pitch_before_pulse_deg=float(np.max(np.abs(a[:401, 3]))*180/math.pi),
                                  max_abs_position_before_pulse_m=float(np.max(np.abs(a[:401, 1]))),
                                  max_slip_before_pulse_m_s=float(np.max(np.abs(a[:401, 9:11]))),
                                  first_contact_time_s=float(a[np.argmax(a[:, 13] > 0), 0]))
        resolved[step] = {name: ({'initial_height_m': rest} if v.get('initial_height') == 'measured' else
                                 {'injected_state': a[400, 1:5].tolist()} if v.get('inject_native_state_at_push') else {})
                          for name, v in VARIANTS.items()}
    print('Start-up measurements:', json.dumps(startup[str(STEP)]), flush=True)

    names = list(VARIANTS)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        brackets = list(pool.map(critical, [(n, resolved[STEP][n]) for n in names]))
        print(f'Bisection complete ({time.perf_counter()-started:.0f} s)', flush=True)
        label_jobs = [(n, step, sign*18.2, resolved[step][n], True) for n in names for step in [STEP, FINE] for sign in [1, -1]]
        labels = list(pool.map(one, label_jobs))
        print(f'Labels at +/-18.2 N complete ({time.perf_counter()-started:.0f} s)', flush=True)
        forces = [round(18.10+.01*i, 2) for i in range(21)]
        scan_jobs = [(n, STEP, f, {}, False) for n in ['planar', 'native'] for f in forces]
        scans = [r for r, _ in pool.map(one, scan_jobs)]
        print(f'Monotonicity scans complete ({time.perf_counter()-started:.0f} s)', flush=True)

    reference = next(b for b in brackets if b['variant'] == 'planar')['midpoint_n']
    native = next(b for b in brackets if b['variant'] == 'native')['midpoint_n']
    for b in brackets:
        b.update(group=VARIANTS[b['variant']]['group'], label=VARIANTS[b['variant']]['label'], engine=VARIANTS[b['variant']]['engine'])
        if b['transition_in_bracket']:
            b['shift_from_planar_n'] = b['midpoint_n']-reference
            b['shift_from_native_n'] = b['midpoint_n']-native
            b['fraction_of_gap_closed'] = (b['midpoint_n']-native)/(reference-native) if b['engine'] == 'native' else None
    records, trajectories = [], {}
    for (result, a), job in zip(labels, label_jobs):
        result['id'] = f"{job[0]}__{job[1]:g}__{job[2]:+g}"
        records.append(result); trajectories[result['id']] = a
    monotonic = {}
    for name in ['planar', 'native']:
        seq = [r['outcome'] == 'fallen' for r in scans if r['variant'] == name]
        monotonic[name] = dict(forces_n=forces, fallen=seq, single_transition=sum(a != b for a, b in zip(seq, seq[1:])) == 1,
                               last_not_fallen_n=max(f for f, s in zip(forces, seq) if not s), first_fallen_n=min(f for f, s in zip(forces, seq) if s))
    p, _, _ = core.load_properties(0, .18, LIMIT)
    demand = {}
    for force in [17.0, 18.0, 18.2, 19.0]:
        _, a = execute(dict(variant='planar', physics_step_s=STEP, push_force_n=force))
        demand[str(force)] = traction(a, p, 1)
    result = {'metadata': {'protocol': PROTOCOL, 'source_sha256': hashes, 'mujoco': mujoco.__version__, 'numpy': np.__version__,
                           'python': platform.python_version(), 'trajectory_columns': COLUMNS, 'workers': workers,
                           'runtime_s': time.perf_counter()-started,
                           'integrations': sum(len(b['trace']) for b in brackets)+len(records)+len(scans)+len(demand)+4},
              'regression': regression, 'startup': startup, 'brackets': brackets, 'labels': records,
              'monotonicity': monotonic, 'planar_traction_demand': demand,
              'planar_midpoint_n': reference, 'native_midpoint_n': native}
    np.savez_compressed(OUT/'trajectories.npz', **trajectories)
    (OUT/'study.json').write_text(json.dumps(result, indent=2)+'\n')
    print(f"{'variant':24s} {'bracket midpoint':>16s} {'shift vs planar':>16s}  +18.2  -18.2 (fine)")
    for b in brackets:
        tags = [next(r['outcome'][:3] for r in records if r['variant'] == b['variant'] and r['physics_step_s'] == FINE and r['push_force_n'] == s*18.2) for s in [1, -1]]
        mid = f"{b['midpoint_n']:.4f}" if b['transition_in_bracket'] else 'none'
        shift = f"{b['shift_from_planar_n']:+.4f}" if b['transition_in_bracket'] else ''
        print(f"{b['variant']:24s} {mid:>16s} {shift:>16s}  {tags[0]}    {tags[1]}")
    print(json.dumps({'integrations': result['metadata']['integrations'], 'runtime_s': result['metadata']['runtime_s']}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--workers', type=int, default=8)
    main(parser.parse_args().workers)
