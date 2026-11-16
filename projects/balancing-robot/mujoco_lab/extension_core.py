"""Loaded plant assembly and matched trials; keeps the first MuJoCo study frozen."""
from dataclasses import replace
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
import validate as previous

ROOT = previous.ROOT
HERE = previous.HERE
CONFIG = previous.CONFIG
PROTOCOL = json.loads((HERE/'extension_protocol.json').read_text())
BANK = json.loads((ROOT/'results/comparison/gain_design.json').read_text())['bank']
DT = PROTOCOL['control_period_s']


def load_properties(mass, height, limit):
    b = CONFIG['parameters']
    m0, l0 = b['body_mass'], b['length']
    total = m0+mass
    centre = (m0*l0+mass*height)/total
    x, y, z = PROTOCOL['payload']['shape_xyz_m']
    own = mass*np.array([y*y+z*z, x*x+z*z, x*x+y*y])/12
    parallel = m0*(l0-centre)**2 + mass*(height-centre)**2
    inertia = np.array([.012, b['body_inertia'], .004])+own+np.array([parallel, parallel, 0])
    p = previous.Parameters(**{**b, 'body_mass': total, 'length': centre,
                               'body_inertia': float(inertia[1]), 'torque_limit': limit})
    return p, inertia, own


def gain(controller, mass, height):
    key = '0.0000/0.1800' if controller == 'fixed' or mass == 0 else f'{mass:.4f}/{height:.4f}'
    return np.array(BANK[key]['gain'])


class LoadedEngine(previous.Engine):
    def __init__(self, kind, step, mass, height, limit):
        self.kind, self.step = kind, step
        self.substeps = round(DT/step)
        assert abs(self.substeps*step-DT) < 1e-14
        self.p, inertia, own = load_properties(mass, height, limit)
        self.mass = mass
        if kind == 'reduced':
            root = ET.fromstring(previous.reduced_xml())
            torso = root.find('.//body[@name="torso"]/inertial')
            torso.set('mass', str(self.p.body_mass))
            torso.set('pos', f'0 0 {self.p.length}')
            torso.set('diaginertia', ' '.join(map(str, inertia)))
        else:
            root = ET.parse(HERE/'balancing_robot.xml').getroot()
            root.find('default/motor').set('ctrlrange', f'{-limit/2} {limit/2}')
            if mass > 0:
                base = root.find('.//body[@name="base"]')
                payload = ET.SubElement(base, 'body', name='payload', pos=f'0 0 {height}')
                ET.SubElement(payload, 'inertial', pos='0 0 0', mass=str(mass), diaginertia=' '.join(map(str, own)))
                ET.SubElement(payload, 'geom', name='payload_box', type='box', size='.04 .06 .03', rgba='0.95 0.65 0.12 1')
        self.xml = ET.tostring(root, encoding='unicode')
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.model.opt.timestep = step
        self.data = mujoco.MjData(self.model)
        self.base, self.torso = [self.model.body(n).id for n in ['base', 'torso']]
        self.payload = self.model.body('payload').id if kind == 'native' and mass > 0 else None
        self.vel = np.zeros(6)
        if kind == 'native':
            self.wheels = [self.model.body(n).id for n in ['left_wheel', 'right_wheel']]
        assert abs(self.model.body_mass.sum()-(1.55+mass)) < 1e-12
        self.verify_compiled_load()

    def verify_compiled_load(self):
        m = self.model
        if self.kind == 'reduced':
            assert abs(m.body_mass[self.torso]-self.p.body_mass) < 1e-12
            assert abs(m.body_ipos[self.torso, 2]-self.p.length) < 1e-12
            assert abs(m.body_inertia[self.torso, 1]-self.p.body_inertia) < 1e-12
            return
        ids = [self.torso]+([] if self.payload is None else [self.payload])
        heights = [m.body_pos[i, 2]+m.body_ipos[i, 2] for i in ids]
        centre = sum(m.body_mass[i]*h for i, h in zip(ids, heights))/self.p.body_mass
        moment = sum(m.body_inertia[i, 1]+m.body_mass[i]*(h-centre)**2 for i, h in zip(ids, heights))
        for i in ids:
            np.testing.assert_allclose(m.body_iquat[i], [1, 0, 0, 0], atol=1e-12)
        assert abs(centre-self.p.length) < 1e-12
        assert abs(moment-self.p.body_inertia) < 1e-12

    def apply_force(self, force):
        d = self.data
        d.xfrc_applied[:] = 0
        if self.kind == 'reduced' or self.payload is None:
            d.xfrc_applied[self.torso, 0] = force
        else:
            d.xfrc_applied[self.torso, 0] = force*1.2/self.p.body_mass
            d.xfrc_applied[self.payload, 0] = force*self.mass/self.p.body_mass
        if self.kind == 'native':
            d.xfrc_applied[self.base, 0] = -self.p.translation_drag*d.qvel[0]
            d.xfrc_applied[self.torso, 4] = -self.p.rotation_drag*self.state()[3]

    def advance(self, torque, push):
        d, m = self.data, self.model
        d.ctrl[:] = torque if self.kind == 'reduced' else torque/2
        for _ in range(self.substeps):
            self.apply_force(push)
            mujoco.mj_step(m, d)
            mujoco.mj_forward(m, d)


def execute(job):
    kind, step = job['engine'], job['physics_step_s']
    mass, height, limit = job['mass_kg'], job['height_m'], job['torque_limit_nm']
    p, _, _ = load_properties(mass, height, limit)
    k = gain(job['controller'], mass, height)
    engine = None if kind == 'planar' else LoadedEngine(kind, step, mass, height, limit)
    state = np.array([0, 0, math.radians(job['initial_tilt_deg']), 0])
    if engine: engine.reset(job['initial_tilt_deg'])
    rows = []
    for tick in range(2001):
        if engine:
            state, extras = engine.state(), engine.extras()
        else:
            extras = [0, p.radius, 0, 0, 0, 0, 0]
        assert np.all(np.isfinite(state)) and np.all(np.isfinite(extras))
        fallen = abs(state[2]) >= math.pi/4 or abs(extras[0]) >= math.pi/4
        terminal = fallen or tick == 2000
        force = job['push_force_n'] if 400 <= tick < 440 and not terminal else 0
        raw = -float(k@state)
        torque = max(-limit, min(limit, raw)) if not terminal else 0
        rows.append([tick*DT, *state, torque, force, *extras])
        if terminal: break
        if engine:
            engine.advance(torque, force)
        else:
            for _ in range(round(DT/step)):
                state = previous.rk4(state, torque, force, step, p)
    a = np.asarray(rows)
    result = previous.metrics(a, fallen, job['push_force_n'])
    result['saturation_s'] = float(np.count_nonzero(np.abs(a[:-1, 5]) >= limit-1e-12)*DT)
    result['push_started'] = bool(np.count_nonzero(a[:, 6]))
    result['push_completed'] = bool(np.count_nonzero(a[:, 6]) == 40) if job['push_force_n'] else None
    result['fall_time_s'] = float(a[-1, 0]) if fallen else None
    result['gain'] = k.tolist()
    result['warnings'] = [] if engine is None else [int(w.number) for w in engine.data.warning]
    assert not any(result['warnings'])
    if not fallen and job['push_force_n']: assert result['pulse_samples'] == 40
    if engine and kind == 'native': assert a[:, 13].max() > 0
    return result, a


def verify_loads():
    checks = []
    loads = [(0, .18)]+list(itertools.product(PROTOCOL['payload']['masses_kg'], PROTOCOL['payload']['heights_m']))
    for mass, height in loads:
        p, _, _ = load_properties(mass, height, .6)
        stored = BANK['0.0000/0.1800' if mass == 0 else f'{mass:.4f}/{height:.4f}']['parameters']
        for name in ['body_mass', 'length', 'body_inertia']:
            assert abs(getattr(p, name)-stored[name]) < 1e-12
        e = LoadedEngine('reduced', .0005, mass, height, .6)
        native = LoadedEngine('native', .0005, mass, height, .6)
        errors = []
        for theta, v, w, u, force in itertools.product([-.3, 0, .3], [-.2, .2], [-.4, .4], [-.2, .2], [-3, 3]):
            e.data.qpos[:] = [0, theta]; e.data.qvel[:] = [v, w]
            e.data.ctrl[:] = u; e.apply_force(force)
            mujoco.mj_forward(e.model, e.data)
            expected = np.array(previous.derivative([0, v, theta, w], u, force, p))[[1, 3]]
            errors.append(np.abs(e.data.qacc-expected))
        error = np.max(errors, axis=0)
        assert np.max(error) < 1e-9
        # Native split forces must give the correct resultant and moment about axle.
        native.reset(17); native.apply_force(3)
        ids = [native.torso]+([] if native.payload is None else [native.payload])
        total = np.sum(native.data.xfrc_applied[ids, :3], axis=0)
        moment = sum(np.cross(native.data.xipos[i]-native.data.xpos[native.base], native.data.xfrc_applied[i, :3])[1] for i in ids)
        np.testing.assert_allclose(total, [3, 0, 0], atol=1e-12)
        assert abs(moment-3*p.length*math.cos(math.radians(17))) < 1e-12
        checks.append({'mass_kg': mass, 'height_m': height, 'body_mass_kg': p.body_mass,
                       'body_com_m': p.length, 'body_pitch_inertia_kg_m2': p.body_inertia,
                       'acceleration_checks': len(errors), 'max_acceleration_errors': error.tolist(),
                       'compiled_mass_com_inertia_and_force_moment': 'passed'})
    return checks
