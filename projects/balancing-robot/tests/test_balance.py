import json
import math
from dataclasses import replace
from pathlib import Path
import shutil
import subprocess
import sys
import unittest
import numpy as np
from scipy.linalg import expm

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from balance import Controller, Parameters, derivative, energy, rk4, simulate
from design_controller import design

CONFIG=json.loads((ROOT/"config.json").read_text())


class PhysicsTests(unittest.TestCase):
    def test_upright_equilibrium(self):
        self.assertEqual(derivative((0,0,0,0),0),(0,0,0,0))
        self.assertEqual(rk4((0,0,0,0),0,0,.005),(0,0,0,0))

    def test_mechanical_power_balance(self):
        p=Parameters();s=np.array([.1,.3,.2,-.5]);torque=.12;push=2
        f=np.array(derivative(s,torque,push,p));eps=1e-7
        numerical=(energy(s+eps*f,p)-energy(s-eps*f,p))/(2*eps)
        expected=torque*(s[1]/p.radius-s[3])+push*(s[1]+p.length*math.cos(s[2])*s[3])-p.translation_drag*s[1]**2-p.rotation_drag*s[3]**2
        self.assertAlmostEqual(numerical,expected,places=7)

    def test_unforced_energy_conservation(self):
        p=replace(Parameters(),translation_drag=0,rotation_drag=0)
        state=(0,.1,.1,.2);initial=energy(state,p)
        for _ in range(1000):state=rk4(state,0,0,.001,p)
        self.assertAlmostEqual(energy(state,p),initial,places=7)

    def test_rk4_converges_under_step_halving(self):
        final=[]
        for dt in (.01,.005,.000625):
            state=(0,.1,.1,0)
            for _ in range(round(.1/dt)):state=rk4(state,.05,1,dt)
            final.append(np.array(state))
        self.assertLess(np.linalg.norm(final[1]-final[2]),np.linalg.norm(final[0]-final[2])/8)

    def test_invalid_parameters(self):
        for kwargs in ({"radius":0},{"body_mass":-1},{"gravity":float('nan')}):
            with self.assertRaises(ValueError):Parameters(**kwargs)
        with self.assertRaises(ValueError):rk4((0,0,0,0),0,0,-.1)


class ControllerTests(unittest.TestCase):
    def test_lqr_design_and_sampled_stability(self):
        fresh=design()
        np.testing.assert_allclose(fresh['lqr_gain'],CONFIG['lqr_gain'],rtol=1e-10)
        a=np.array(fresh['linear_a']);b=np.array(fresh['linear_b']);k=np.array(fresh['lqr_gain']).reshape(1,4)
        block=np.zeros((5,5));block[:4,:4]=a;block[:4,4:]=b
        discrete=expm(block*CONFIG['dt'])
        poles=np.linalg.eigvals(discrete[:4,:4]-discrete[:4,4:]@k)
        self.assertTrue(np.all(np.abs(poles)<1))

    def test_standard_push_recovers(self):
        for name in ('pid','lqr'):
            metrics,rows=simulate(name,CONFIG,push_force=3)
            self.assertFalse(metrics['fallen'])
            self.assertIsNotNone(metrics['recovery_s'])
            self.assertLess(abs(metrics['final_tilt_deg']),1)
            self.assertLess(abs(metrics['final_position_m']),.03)
            self.assertTrue(all(abs(row['torque'])<=.7 for row in rows))

    def test_uncontrolled_and_overwhelming_push_fail(self):
        self.assertTrue(simulate('off',CONFIG)[0]['fallen'])
        for name in ('pid','lqr'):
            metrics,_=simulate(name,CONFIG,push_force=100)
            self.assertTrue(metrics['fallen'])
            self.assertIsNone(metrics['recovery_s'])

    def test_initial_fall_and_short_run(self):
        metrics,rows=simulate('lqr',CONFIG,initial_degrees=45)
        self.assertTrue(metrics['fallen']);self.assertEqual(len(rows),1)
        self.assertIsNone(simulate('lqr',CONFIG,duration=.1)[0]['recovery_s'])

    def test_anti_windup(self):
        control=Controller('pid',CONFIG)
        for _ in range(500):
            torque,clipped=control.command((0,0,.2,0),.005,.1)
            self.assertEqual(torque,.1);self.assertTrue(clipped)
        self.assertEqual(control.integral,0)

    @unittest.skipUnless(shutil.which('node'),'Node required for cross-runtime check')
    def test_browser_matches_python(self):
        script="""
const {Robot}=require('./model.js'), cfg=require('./config.js');
const output={};
for(const name of ['pid','lqr','off']) {
  const robot=new Robot(name,cfg);
  for(let i=0;i<2000&&!robot.fallen;i++)robot.step(i>=400&&i<440?3:0);
  output[name]={state:robot.state,time:robot.time,fallen:robot.fallen,effort:robot.effort};
}
console.log(JSON.stringify(output));
"""
        result=subprocess.run(['node','-e',script],cwd=ROOT,capture_output=True,text=True,check=True,timeout=20)
        browser=json.loads(result.stdout)
        for name in ('pid','lqr','off'):
            metrics,rows=simulate(name,CONFIG,push_force=3)
            np.testing.assert_allclose(browser[name]['state'],rows[-1]['state'],atol=1e-11)
            self.assertEqual(browser[name]['fallen'],metrics['fallen'])
            self.assertAlmostEqual(browser[name]['time'],rows[-1]['time'],places=10)
            self.assertAlmostEqual(browser[name]['effort'],metrics['torque_squared_integral'],places=5)


if __name__=='__main__':unittest.main()
