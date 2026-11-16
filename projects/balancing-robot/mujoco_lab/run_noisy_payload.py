"""Repeat the noisy fixed-vs-exact payload comparison on the native MuJoCo plant.

Conditions, seeds, gains and noise profiles come unchanged from
comparison_protocol.json. sensing.js is ported line for line; the port must
reproduce the archived planar outcomes before native results are interpreted.
"""
import argparse
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import itertools
import json
import math
import platform
import time
import numpy as np
import mujoco
import extension_core as core

previous = core.previous
ROOT, HERE = core.ROOT, core.HERE
PROTOCOL = json.loads((HERE/'noisy_payload_protocol.json').read_text())
SOURCE = json.loads((ROOT/'comparison_protocol.json').read_text())
DT, RAD = SOURCE['dt_s'], math.pi/180
OUT = ROOT/'results/mujoco_noisy_payload'
CACHE = HERE/'_qa/noisy_cache'
FILES = ['balance.py', 'config.json', 'comparison_protocol.json', 'visualisation/sensing.js',
         'mujoco_lab/validate.py', 'mujoco_lab/extension_core.py', 'mujoco_lab/extension_protocol.json',
         'mujoco_lab/run_noisy_payload.py', 'mujoco_lab/noisy_payload_protocol.json',
         'mujoco_lab/balancing_robot.xml', 'results/comparison/gain_design.json']
COLUMNS = ['time_s', 'x_m', 'vx_m_s', 'pitch_rad', 'pitch_rate_rad_s', 'estimate_x_m', 'estimate_vx_m_s',
           'estimate_pitch_rad', 'estimate_pitch_rate_rad_s', 'next_interval_torque_nm', 'next_interval_push_n']


class Random:
    """sensing.js Random: 32-bit LCG and Box-Muller pairs."""
    def __init__(self, seed): self.state = seed & 0xffffffff
    def uniform(self):
        self.state = (1664525*self.state+1013904223) & 0xffffffff
        return (self.state+.5)/4294967296
    def pair(self):
        radius = math.sqrt(-2*math.log(self.uniform()))
        phase = 2*math.pi*self.uniform()
        return radius*math.cos(phase), radius*math.sin(phase)


class AttitudeKalman:
    def __init__(self, angle_sigma, gyro_sigma, accel_sigma):
        self.r0, self.r1, self.q, self.ready = angle_sigma**2, gyro_sigma**2, accel_sigma**2, False
    def update(self, theta, omega, dt):
        if not self.ready or (self.r0 == 0 and self.r1 == 0):
            self.theta, self.omega, self.a, self.b, self.c, self.ready = theta, omega, self.r0, 0.0, self.r1, True
            return theta, omega
        self.theta += dt*self.omega
        a = self.a+2*dt*self.b+dt*dt*self.c+self.q*dt**4/4
        b = self.b+dt*self.c+self.q*dt**3/2
        c = self.c+self.q*dt*dt
        s = a+self.r0
        if s > 0:
            error = theta-self.theta
            self.theta += a/s*error; self.omega += b/s*error
            c = max(0, c-b*b/s); b = b*self.r0/s; a = a*self.r0/s
        s = c+self.r1
        if s > 0:
            error = omega-self.omega
            self.theta += b/s*error; self.omega += c/s*error
            a = max(0, a-b*b/s); b = b*self.r1/s; c = c*self.r1/s
        self.a, self.b, self.c = a, b, c
        return self.theta, self.omega


class Observation:
    """Causal packet pipeline: noise on pitch and gyro, whole-packet delay, optional filter."""
    def __init__(self, profile, seed):
        self.mode = profile['mode']
        self.angle, self.gyro = profile['angleNoiseDeg']*RAD, profile['gyroNoiseDegS']*RAD
        self.delay = round(profile['delayMs']/(DT*1000))
        assert abs(self.delay-profile['delayMs']/(DT*1000)) < 1e-8
        self.random, self.queue, self.last_source = Random(seed), [], -1
        self.filter = AttitudeKalman(self.angle, self.gyro, 50)
    def observe(self, truth, tick):
        n = self.random.pair()
        self.queue.append((tick, [truth[0], truth[1], truth[2]+self.angle*n[0], truth[3]+self.gyro*n[1]]))
        if len(self.queue) > self.delay+1: self.queue.pop(0)
        source, measured = self.queue[0]
        if source != self.last_source:
            attitude = measured[2:]
            if self.mode == 'kalman': attitude = self.filter.update(measured[2], measured[3], DT)
            self.estimate = [measured[0], measured[1], *attitude]
            self.last_source = source
        return self.estimate


def trial(job, capture=False):
    mass, height, limit = job['mass_kg'], job['height_m'], job['torque_limit_nm']
    p, _, _ = core.load_properties(mass, height, limit)
    k = [float(x) for x in core.gain(job['controller'], mass, height)]
    native = job['engine'] == 'native'
    step = job['physics_step_s']
    engine = core.LoadedEngine('native', step, mass, height, limit) if native else None
    state = [0.0, 0.0, job['initial_tilt_deg']*RAD, 0.0]
    if native:
        engine.reset(job['initial_tilt_deg'])
        state = [float(x) for x in engine.state()]
    sensor = Observation(job['profile_settings'], job['seed'])
    estimate = sensor.observe(state, 0)
    tick, fallen, last_bad, tail, n = 0, False, None, [0.0, 0.0, 0.0], 0
    peak, travel, effort, saturated, roll, slip = abs(job['initial_tilt_deg']), 0.0, 0.0, 0, 0.0, 0.0
    rows = []

    def record(torque=0.0, push=0.0):
        nonlocal last_bad, n
        t = tick*DT
        if t >= 2.2-1e-10 and (abs(state[0]) > .03 or abs(state[1]) > .03 or abs(state[2]) > RAD): last_bad = t
        if t >= 9-1e-10:
            for i in range(3): tail[i] += state[i]**2
            n += 1
        if capture: rows.append([t, *state, *estimate, torque, push])

    while True:
        terminal = tick >= 2000 or fallen
        push = job['push_force_n'] if 400 <= tick < 440 and not terminal else 0.0
        raw = -sum(g*y for g, y in zip(k, estimate))
        torque = max(-limit, min(limit, raw)) if not terminal else 0.0
        record(torque, push)
        if terminal: break
        saturated += abs(raw) > limit
        if native:
            engine.advance(torque, push)
            state = [float(x) for x in engine.state()]
            extras = engine.extras()
            roll, slip = max(roll, abs(extras[0])), max(slip, abs(extras[2]), abs(extras[3]))
            fallen = abs(state[2]) >= math.pi/4 or abs(extras[0]) >= math.pi/4
        else:
            for _ in range(round(DT/step)):
                state = list(previous.rk4(state, torque, push, step, p))
            fallen = abs(state[2]) >= math.pi/4
        assert all(math.isfinite(x) for x in state)
        tick += 1
        effort += torque*torque*DT
        peak, travel = max(peak, abs(state[2]/RAD)), max(travel, abs(state[0]))
        estimate = sensor.observe(state, tick)
    duration = tick*DT
    settled = max(2.2, 2.2 if last_bad is None else last_bad+DT)
    recovery = settled-2.2 if not fallen and duration-settled >= 1-1e-10 else None
    full = not fallen
    if native: assert not any(int(w.number) for w in engine.data.warning)
    metrics = dict(outcome='fallen' if fallen else 'unsettled' if recovery is None else 'recovered',
                   duration_s=duration, recovery_s=recovery, peak_tilt_deg=peak, max_displacement_m=travel,
                   torque_squared_integral=effort, saturation_s=saturated*DT,
                   tail_tilt_rms_deg=math.sqrt(tail[2]/n)/RAD if full else None,
                   tail_position_rms_m=math.sqrt(tail[0]/n) if full else None,
                   max_roll_deg=roll/RAD, max_sampled_slip_m_s=slip)
    return metrics, np.asarray(rows)


def group_jobs(group):
    return [dict(group, seed=5003+104729*i, seed_index=i) for i in range(SOURCE['repetitions'])]


def work(group, fingerprint):
    key = hashlib.sha256((fingerprint+json.dumps(group, sort_keys=True)).encode()).hexdigest()
    path = CACHE/fingerprint/f'{key}.json'
    if path.exists(): return json.loads(path.read_text())
    records = [dict({k: v for k, v in job.items() if k != 'profile_settings'}, **trial(job)[0]) for job in group_jobs(group)]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(records))
    return records


def sign_test(prevented, introduced):
    """Exact two-sided binomial test on discordant pairs."""
    n, low = prevented+introduced, min(prevented, introduced)
    if n == 0: return 1.0
    return min(1.0, 2*sum(math.comb(n, i) for i in range(low+1))/2**n)


def summarise(records, engine_id):
    rows = {(r['mass_kg'], r['height_m'], r['scenario'], r['profile'], r['seed'], r['controller']): r for r in records if r['engine_id'] == engine_id}
    keys = sorted({k[:5] for k in rows})
    pairs = [(rows[k+('fixed',)], rows[k+('exact',)]) for k in keys]
    falls = lambda sel, i: sum(p[i]['outcome'] == 'fallen' for p in sel)
    def block(sel):
        prevented = sum(a['outcome'] == 'fallen' and b['outcome'] != 'fallen' for a, b in sel)
        introduced = sum(a['outcome'] != 'fallen' and b['outcome'] == 'fallen' for a, b in sel)
        return dict(pairs=len(sel), fixed_falls=falls(sel, 0), exact_falls=falls(sel, 1), prevented=prevented, introduced=introduced,
                    both_fall=sum(a['outcome'] == b['outcome'] == 'fallen' for a, b in sel),
                    both_survive=sum(a['outcome'] != 'fallen' and b['outcome'] != 'fallen' for a, b in sel))
    total = block(pairs)
    total['sign_test_p'] = sign_test(total['prevented'], total['introduced'])
    total['outcomes'] = {c: dict(Counter(p[i]['outcome'] for p in pairs)) for i, c in enumerate(['fixed', 'exact'])}
    survivors = [(a, b) for a, b in pairs if a['outcome'] != 'fallen' and b['outcome'] != 'fallen']
    if survivors:
        for name in ['peak_tilt_deg', 'torque_squared_integral', 'tail_tilt_rms_deg']:
            delta = np.array([b[name]-a[name] for a, b in survivors])
            total[f'survivor_{name}_median_delta'] = float(np.median(delta))
            total[f'survivor_{name}_iqr'] = [float(x) for x in np.percentile(delta, [25, 75])]
    by = {}
    for name, index in [('scenario', 2), ('profile', 3), ('mass_kg', 0), ('height_m', 1)]:
        by[name] = {str(v): block([p for k, p in zip(keys, pairs) if k[index] == v]) for v in sorted({k[index] for k in keys})}
    by['load'] = {f'{m:g} kg at {h:g} m': block([p for k, p in zip(keys, pairs) if k[:2] == (m, h)]) for m, h in sorted({k[:2] for k in keys})}
    by['cell'] = [dict(mass_kg=m, height_m=h, scenario=s, profile=f, **block([p for k, p in zip(keys, pairs) if k[:4] == (m, h, s, f)]))
                  for m, h, s, f in sorted({k[:4] for k in keys})]
    return dict(total, by=by)


def main(workers):
    started = time.perf_counter()
    OUT.mkdir(parents=True, exist_ok=True)
    hashes = {n: hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in FILES}
    fingerprint = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()[:20]
    profiles = {p['id']: p for p in SOURCE['profiles']}
    groups = []
    for e in PROTOCOL['engines']:
        for m, h in itertools.product(SOURCE['test_masses_kg'], SOURCE['test_heights_m']):
            for s in SOURCE['sensing_test']:
                for f in SOURCE['profiles']:
                    for controller in PROTOCOL['controllers']:
                        groups.append(dict(engine_id=e['id'], engine=e['engine'], physics_step_s=e['physics_step_s'], mass_kg=m, height_m=h,
                                           scenario=s['id'], initial_tilt_deg=s['initial_tilt_deg'], push_force_n=s['push_force_n'],
                                           torque_limit_nm=s['torque_limit_nm'], profile=f['id'], profile_settings=profiles[f['id']], controller=controller))
    records = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        pending = [pool.submit(work, g, fingerprint) for g in groups]
        for i, future in enumerate(as_completed(pending), 1):
            records.extend(future.result())
            if i % 20 == 0 or i == len(groups):
                print(f'{i}/{len(groups)} condition groups, {len(records)} trials ({time.perf_counter()-started:.0f} s)', flush=True)
    records.sort(key=lambda r: (r['engine_id'], r['mass_kg'], r['height_m'], r['scenario'], r['profile'], r['seed_index'], r['controller']))

    # Port verification against every archived JavaScript record for these two controllers.
    archive = json.loads((ROOT/'results/comparison/test.json').read_text())['records']
    old = {(r['mass_kg'], r['height_m'], r['scenario'], r['profile'], r['seed'], r['variant']): r
           for r in archive if r['phase'] == 'sensing_test' and r['variant'] in PROTOCOL['controllers']}
    port = [r for r in records if r['engine_id'] == 'planar_port']
    assert len(port) == len(old) == 3240
    mismatched, worst = [], dict(peak_tilt_deg=0.0, duration_s=0.0, torque_squared_integral=0.0)
    for r in port:
        a = old[r['mass_kg'], r['height_m'], r['scenario'], r['profile'], r['seed'], r['controller']]
        if a['outcome'] != r['outcome']: mismatched.append({k: r[k] for k in ['mass_kg', 'height_m', 'scenario', 'profile', 'seed', 'controller']})
        for name in worst: worst[name] = max(worst[name], abs(a[name]-r[name]))
    verification = dict(archived_records=len(old), outcome_matches=len(port)-len(mismatched), outcome_mismatches=mismatched,
                        max_absolute_differences=worst)
    print('Port verification:', json.dumps({k: v for k, v in verification.items() if k != 'outcome_mismatches'}), flush=True)

    summary = {e['id']: summarise(records, e['id']) for e in PROTOCOL['engines']}
    index = {(r['engine_id'], r['mass_kg'], r['height_m'], r['scenario'], r['profile'], r['seed'], r['controller']): r['outcome'] == 'fallen' for r in records}
    agreement = {}
    for a, b in [('planar_port', 'native_0p25ms'), ('native_0p25ms', 'native_0p5ms'), ('planar_port', 'native_0p5ms')]:
        same = [index[(a,)+k[1:]] == index[(b,)+k[1:]] for k in index if k[0] == a]
        agreement[f'{a} vs {b}'] = dict(trials=len(same), same_fall_label=sum(same))
    native = [summary[i] for i in ['native_0p25ms', 'native_0p5ms']]
    persists = all(s['exact_falls'] < s['fixed_falls'] and s['prevented'] > s['introduced'] for s in native)

    e = PROTOCOL['example']
    scenario = next(s for s in SOURCE['sensing_test'] if s['id'] == e['scenario'])
    examples, arrays = [], {}
    for engine in PROTOCOL['engines'][:2]:
        for controller in PROTOCOL['controllers']:
            job = dict(engine_id=engine['id'], engine=engine['engine'], physics_step_s=engine['physics_step_s'], mass_kg=e['mass_kg'], height_m=e['height_m'],
                       scenario=scenario['id'], initial_tilt_deg=scenario['initial_tilt_deg'], push_force_n=scenario['push_force_n'],
                       torque_limit_nm=scenario['torque_limit_nm'], profile=e['profile'], profile_settings=profiles[e['profile']],
                       controller=controller, seed=5003+104729*e['seed_index'], seed_index=e['seed_index'])
            metrics, a = trial(job, capture=True)
            name = f"{engine['id']}__{controller}"
            assert metrics['outcome'] == next(r for r in records if all(r[k] == job[k] for k in ['engine_id', 'mass_kg', 'height_m', 'scenario', 'profile', 'seed', 'controller']))['outcome']
            examples.append(dict(id=name, engine_id=engine['id'], controller=controller, **metrics)); arrays[name] = a
    result = {'metadata': {'protocol': PROTOCOL, 'source_protocol': SOURCE, 'source_sha256': hashes, 'mujoco': mujoco.__version__,
                           'numpy': np.__version__, 'python': platform.python_version(), 'workers': workers,
                           'runtime_s': time.perf_counter()-started, 'example_columns': COLUMNS},
              'port_verification': verification, 'summary': summary, 'fall_label_agreement': agreement,
              'archived_planar': dict(pairs=1620, fixed_falls=508, exact_falls=334, prevented=186, introduced=12),
              'reduction_persists': persists, 'examples': examples, 'records': records}
    np.savez_compressed(OUT/'examples.npz', **arrays)
    (OUT/'study.json').write_text(json.dumps(result)+'\n')
    keys = [k for k in records[0]]
    (OUT/'trials.csv').write_text(','.join(keys)+'\n'+'\n'.join(','.join('' if r[k] is None else str(r[k]) for k in keys) for r in records)+'\n')
    for name, s in summary.items():
        print(f"{name:14s} fixed falls {s['fixed_falls']:4d}  exact falls {s['exact_falls']:4d}  prevented {s['prevented']:4d}  introduced {s['introduced']:4d}  p={s['sign_test_p']:.3g}")
    print(json.dumps({'reduction_persists': persists, 'agreement': agreement, 'runtime_s': result['metadata']['runtime_s']}, indent=1))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--workers', type=int, default=8)
    main(parser.parse_args().workers)
