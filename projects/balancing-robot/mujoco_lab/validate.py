"""Reproducible MuJoCo comparison; original model and experiment outputs are read-only."""
import hashlib
import itertools
import json
import math
import platform
import sys
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT / 'results/mujoco'
sys.path.insert(0, str(ROOT))
from balance import Parameters, derivative, rk4

CONFIG = json.loads((ROOT / 'config.json').read_text())
PROTOCOL = json.loads((HERE / 'validation_protocol.json').read_text())
P = Parameters(**CONFIG['parameters'])
K = np.array(CONFIG['lqr_gain'])
DT = PROTOCOL['control_period_s']
COLUMNS = ['time_s', 'x_m', 'vx_m_s', 'pitch_rad', 'pitch_rate_rad_s',
           'next_interval_torque_nm', 'next_interval_push_n', 'roll_rad',
           'axle_height_m', 'left_slip_m_s', 'right_slip_m_s', 'y_m', 'yaw_rad', 'contact_count']


def reduced_xml():
    # Slide armature represents wheel spin energy after imposing ideal rolling.
    # Chassis never rotates; the original model has no chassis pitch inertia.
    return f'''<mujoco model="ideal rolling equivalence check">
      <compiler angle="radian" inertiafromgeom="false"/>
      <option timestep="0.001" integrator="RK4" gravity="0 0 {-P.gravity}"/>
      <worldbody><body name="base" pos="0 0 {P.radius}">
        <joint name="travel" type="slide" axis="1 0 0" damping="{P.translation_drag}" armature="{2*P.wheel_inertia/P.radius**2}"/>
        <inertial pos="0 0 0" mass="{P.base_mass}" diaginertia=".001 .001 .001"/>
        <body name="torso"><joint name="pitch" type="hinge" axis="0 1 0" damping="{P.rotation_drag}"/>
          <inertial pos="0 0 {P.length}" mass="{P.body_mass}" diaginertia="{P.body_inertia} {P.body_inertia} .004"/>
        </body>
      </body></worldbody>
      <actuator><motor joint="travel" gear="{1/P.radius}"/><motor joint="pitch" gear="-1"/></actuator>
    </mujoco>'''


class Engine:
    def __init__(self, kind, step, variant=None):
        self.kind, self.step = kind, step
        self.substeps = round(DT / step)
        assert abs(self.substeps * step - DT) < 1e-14
        if kind == 'reduced':
            self.model = mujoco.MjModel.from_xml_string(reduced_xml())
        else:
            tree = ET.parse(HERE / 'balancing_robot.xml')
            default = tree.getroot().find('default/geom')
            default.set('friction', f"{variant['friction']} 0.005 0.0001")
            default.set('solref', f"{variant['time_constant_s']} 1")
            self.model = mujoco.MjModel.from_xml_string(ET.tostring(tree.getroot(), encoding='unicode'))
        self.model.opt.timestep = step
        self.data = mujoco.MjData(self.model)
        self.base = self.model.body('base').id
        self.torso = self.model.body('torso').id
        self.vel = np.zeros(6)
        assert abs(self.model.body_mass.sum() - 1.55) < 1e-12
        if kind == 'native':
            self.wheels = [self.model.body(n).id for n in ('left_wheel', 'right_wheel')]

    def reset(self, tilt):
        mujoco.mj_resetData(self.model, self.data)
        angle = math.radians(tilt)
        if self.kind == 'reduced':
            self.data.qpos[:] = [0, angle]
        else:
            self.data.qpos[3:7] = [math.cos(angle / 2), 0, math.sin(angle / 2), 0]
        mujoco.mj_forward(self.model, self.data)

    def state(self):
        d = self.data
        if self.kind == 'reduced':
            return np.array([d.qpos[0], d.qvel[0], d.qpos[1], d.qvel[1]])
        rot = d.xmat[self.base].reshape(3, 3)
        mujoco.mj_objectVelocity(self.model, d, mujoco.mjtObj.mjOBJ_BODY, self.base, self.vel, 0)
        return np.array([d.qpos[0], d.qvel[0], math.atan2(rot[0, 2], rot[2, 2]), self.vel[1]])

    def extras(self):
        if self.kind == 'reduced':
            return [0, P.radius, 0, 0, 0, 0, 0]
        d = self.data
        rot = d.xmat[self.base].reshape(3, 3)
        slips = []
        for wheel in self.wheels:
            mujoco.mj_objectVelocity(self.model, d, mujoco.mjtObj.mjOBJ_BODY, wheel, self.vel, 0)
            slips.append(float(self.vel[3] - P.radius * self.vel[1]))
        return [math.atan2(rot[2, 1], rot[2, 2]), d.qpos[2], *slips,
                d.qpos[1], math.atan2(rot[1, 0], rot[0, 0]), d.ncon]

    def advance(self, torque, push):
        m, d = self.model, self.data
        d.ctrl[:] = torque if self.kind == 'reduced' else torque / 2
        for _ in range(self.substeps):
            d.xfrc_applied[:] = 0
            d.xfrc_applied[self.torso, 0] = push
            if self.kind == 'native':
                d.xfrc_applied[self.base, 0] = -P.translation_drag * d.qvel[0]
                d.xfrc_applied[self.torso, 4] = -P.rotation_drag * self.state()[3]
            mujoco.mj_step(m, d)
            # Needed for endpoint telemetry and state-dependent forces on next step.
            mujoco.mj_forward(m, d)


def verify_equations():
    engine = Engine('reduced', .001)
    m, d = engine.model, engine.data
    errors = []
    for angle, velocity, omega, torque, push in itertools.product(
            [-.3, 0, .3], [-.2, .2], [-.4, .4], [-.2, .2], [-3, 3]):
        d.qpos[:] = [0, angle]
        d.qvel[:] = [velocity, omega]
        d.ctrl[:] = torque
        d.xfrc_applied[:] = 0
        d.xfrc_applied[engine.torso, 0] = push
        mujoco.mj_forward(m, d)
        expected = np.array(derivative([0, velocity, angle, omega], torque, push, P))[[1, 3]]
        error = np.abs(d.qacc - expected)
        np.testing.assert_allclose(d.qfrc_actuator, [torque/P.radius, -torque], atol=1e-12, rtol=0)
        errors.append(error)
    maximum = np.max(errors, axis=0)
    assert np.max(maximum) < PROTOCOL['reduced_acceptance']['max_acceleration_error_each_native_unit']
    return {'conditions': len(errors), 'max_acceleration_error_m_s2': float(maximum[0]),
            'max_angular_acceleration_error_rad_s2': float(maximum[1]),
            'actuator_generalized_force_sign_and_scale': 'passed'}


def metrics(a, fallen, push):
    state = a[:, 1:5]
    reference = 2.2 if push else 0
    bad = (np.abs(state[:, 2]) > math.radians(1)) | (np.abs(state[:, 0]) > .03) | (np.abs(state[:, 1]) > .03)
    indices = np.where((a[:, 0] >= reference - 1e-12) & bad)[0]
    settled = max(reference, a[indices[-1], 0] + DT if len(indices) else reference)
    recovered = not fallen and a[-1, 0] - settled >= 1 - 1e-10
    return {'outcome': 'fallen' if fallen else ('recovered' if recovered else 'unsettled'),
            'duration_s': float(a[-1, 0]), 'recovery_s': float(settled-reference) if recovered else None,
            'peak_pitch_deg': float(np.max(np.abs(state[:, 2]))*180/math.pi),
            'max_travel_m': float(np.max(np.abs(state[:, 0]))),
            'squared_torque_integral_nm2_s': float(np.sum(a[:-1, 5]**2)*DT),
            'saturation_s': float(np.sum(np.abs(a[:-1, 5]) >= P.torque_limit-1e-12)*DT),
            'max_sampled_slip_m_s': float(np.max(np.abs(a[:, 9:11]))),
            'max_sampled_roll_deg': float(np.max(np.abs(a[:, 7]))*180/math.pi),
            'max_sampled_y_m': float(np.max(np.abs(a[:, 11]))),
            'min_sampled_axle_height_m': float(a[:, 8].min()),
            'max_sampled_axle_height_m': float(a[:, 8].max()),
            'pulse_samples': int(np.count_nonzero(a[:-1, 6]))}


def trial(kind, step, tilt, push, variant=None):
    engine = None if kind == 'planar' else Engine(kind, step, variant)
    state = np.array([0, 0, math.radians(tilt), 0])
    if engine:
        engine.reset(tilt)
    rows, fallen = [], False
    for tick in range(round(PROTOCOL['duration_s']/DT)+1):
        t = tick*DT
        if engine:
            state, extras = engine.state(), engine.extras()
        else:
            extras = [0, P.radius, 0, 0, 0, 0, 0]
        assert np.all(np.isfinite(state)) and np.all(np.isfinite(extras))
        fallen = abs(state[2]) >= math.pi/4 or abs(extras[0]) >= math.pi/4
        terminal = fallen or tick == round(PROTOCOL['duration_s']/DT)
        force = push if 400 <= tick < 440 and not terminal else 0
        torque = float(np.clip(-K@state, -P.torque_limit, P.torque_limit)) if not terminal else 0
        rows.append([t, *state, torque, force, *extras])
        if terminal:
            break
        if engine:
            engine.advance(torque, force)
        else:
            for _ in range(round(DT/step)):
                state = rk4(state, torque, force, step, P)
    a = np.asarray(rows)
    result = metrics(a, fallen, push)
    if not fallen and push:
        assert result['pulse_samples'] == 40
    if engine:
        warnings = [int(w.number) for w in engine.data.warning]
        assert not any(warnings), warnings
        if kind == 'native':
            assert np.max(a[:, 13]) > 0
    return result, a


def difference(a, b):
    n = min(len(a), len(b))
    np.testing.assert_allclose(a[:n, 0], b[:n, 0], atol=1e-12, rtol=0)
    delta = a[:n, 1:5] - b[:n, 1:5]
    return {'common_horizon_s': float(a[n-1, 0]),
            'max_position_error_m': float(np.max(np.abs(delta[:, 0]))),
            'max_pitch_error_deg': float(np.max(np.abs(delta[:, 2]))*180/math.pi),
            'rms_position_error_m': float(np.sqrt(np.mean(delta[:, 0]**2))),
            'rms_pitch_error_deg': float(np.sqrt(np.mean(delta[:, 2]**2))*180/math.pi),
            'max_state_errors_native_units': np.max(np.abs(delta), axis=0).tolist()}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    source_files = ['config.json', 'balance.py', 'mujoco_lab/validate.py',
                    'mujoco_lab/balancing_robot.xml', 'mujoco_lab/validation_protocol.json', 'mujoco_lab/requirements.txt']
    hashes = {f: hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in source_files}
    equation = verify_equations()
    print('Equation checks:', json.dumps(equation), flush=True)
    (OUT/'reduced_model.xml').write_text(reduced_xml())
    records, trajectories, index = [], {}, {}
    conditions = list(itertools.product(PROTOCOL['initial_tilts_deg'], PROTOCOL['push_forces_n']))
    jobs = []
    for kind in ['planar', 'reduced', 'native']:
        steps = [.001] if kind == 'reduced' else PROTOCOL['physics_steps_s']
        for step in steps:
            for tilt, force in conditions:
                jobs.append((kind, step, tilt, force, PROTOCOL['native_contact_variants'][0]))
    for variant in PROTOCOL['native_contact_variants'][1:]:
        for tilt, force in conditions:
            jobs.append(('native', PROTOCOL['sensitivity_step_s'], tilt, force, variant))
    for i, (kind, step, tilt, force, variant) in enumerate(jobs):
        result, a = trial(kind, step, tilt, force, variant)
        key = f'r{i:03d}'
        result.update(id=key, engine=kind, physics_step_s=step, initial_tilt_deg=tilt,
                      push_force_n=force, variant=variant['name'] if kind == 'native' else 'ideal')
        records.append(result)
        trajectories[key] = a
        index[kind, step, tilt, force, result['variant']] = result
        if (i+1)%15 == 0:
            print(f'{i+1}/{len(jobs)} trials complete ({time.perf_counter()-started:.1f} s)', flush=True)
    pairs = []
    for r in records:
        if r['engine'] == 'planar':
            continue
        step = r['physics_step_s']
        b = index['planar', step, r['initial_tilt_deg'], r['push_force_n'], 'ideal']
        diff = difference(trajectories[r['id']], trajectories[b['id']])
        diff.update(id=r['id'], reference_id=b['id'], same_outcome=r['outcome']==b['outcome'])
        if r['engine'] == 'reduced':
            diff['passes'] = diff['same_outcome'] and max(diff['max_state_errors_native_units']) < 1e-7
            assert diff['passes'], diff
        else:
            screen = PROTOCOL['native_screen']
            diff['passes'] = (diff['same_outcome'] and diff['common_horizon_s'] >= 10-1e-12
                              and diff['max_position_error_m'] <= screen['max_position_error_m']
                              and diff['max_pitch_error_deg'] <= screen['max_pitch_error_deg'])
        pairs.append(diff)
    refinement = []
    for kind in ['planar', 'native']:
        for coarse, fine in zip(PROTOCOL['physics_steps_s'], PROTOCOL['physics_steps_s'][1:]):
            for tilt, force in conditions:
                variant = 'nominal' if kind == 'native' else 'ideal'
                a = index[kind, coarse, tilt, force, variant]
                b = index[kind, fine, tilt, force, variant]
                diff = difference(trajectories[a['id']], trajectories[b['id']])
                diff.update(engine=kind, coarse_step_s=coarse, fine_step_s=fine, id=a['id'],
                            reference_id=b['id'], same_outcome=a['outcome']==b['outcome'])
                refinement.append(diff)
    metadata = {'protocol': PROTOCOL, 'source_sha256': hashes, 'mujoco_version': mujoco.__version__,
                'numpy_version': np.__version__, 'python': platform.python_version(),
                'platform': platform.platform(), 'runtime_s': time.perf_counter()-started,
                'trajectory_columns': COLUMNS, 'control_updates_per_second': 1/DT,
                'notes': 'Deterministic simulations; no hardware calibration. Errors use common pre-fall horizons. Terminal torque and force are zero placeholders.'}
    result = {'metadata': metadata, 'equation_checks': equation, 'records': records,
              'pairs': pairs, 'refinement': refinement}
    np.savez_compressed(OUT/'trajectories.npz', **trajectories)
    (OUT/'study.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'trials': len(records), 'reduced_passes': sum(p['passes'] for p in pairs if records[int(p['id'][1:])]['engine']=='reduced'),
                      'runtime_s': metadata['runtime_s']}, indent=2), flush=True)


if __name__ == '__main__':
    main()
