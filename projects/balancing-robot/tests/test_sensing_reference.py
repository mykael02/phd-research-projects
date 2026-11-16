"""Independent dense-matrix Kalman reference and archived Python plant comparison."""
import json,subprocess,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
class SensingReference(unittest.TestCase):
    def test_dense_matrix_filter_reference(self):
        values=[[.1,.2],[.11,.15],[.09,.25],[.13,.18],[.08,.24]]
        js="const {AttitudeKalman}=require('./visualisation/sensing.js');const f=new AttitudeKalman(.02,.05,50);console.log(JSON.stringify("+json.dumps(values)+".map(y=>({state:f.update(...y,.005),P:[[f.a,f.b],[f.b,f.c]]}))));"
        actual=json.loads(subprocess.check_output(['node','-e',js],cwd=ROOT,text=True))
        dt=.005;F=np.array([[1,dt],[0,1]]);G=np.array([[dt*dt/2],[dt]]);Q=50**2*G@G.T;R=np.diag([.02**2,.05**2]);x=np.array(values[0]);P=R.copy()
        for i,y in enumerate(values):
            if i:
                x=F@x;P=F@P@F.T+Q;K=P@np.linalg.inv(P+R);x=x+K@(np.array(y)-x);P=(np.eye(2)-K)@P@(np.eye(2)-K).T+K@R@K.T
            np.testing.assert_allclose(actual[i]['state'],x,atol=1e-12);np.testing.assert_allclose(actual[i]['P'],P,atol=1e-12)
    def test_ideal_against_archived_python_conditions(self):
        protocol=json.loads((ROOT/'sensing_protocol.json').read_text());baseline=json.loads((ROOT/'results/robustness/results.json').read_text())['records']
        js="const {trial}=require('./run_sensing.js'),p=require('./sensing_protocol.json');console.log(JSON.stringify(p.scenarios.flatMap(s=>p.controllers.map(c=>trial(c,s,{mode:'ideal',seed:1001}).metrics))));"
        actual=json.loads(subprocess.check_output(['node','-e',js],cwd=ROOT,text=True))
        for row in actual:
            scenario=next(s for s in protocol['scenarios'] if s['id']==row['scenario'])
            old=next(r for r in baseline if r['controller']==row['controller'] and all(r[k]==scenario[k] for k in ['initial_tilt_deg','push_force_n','torque_limit_nm']))
            self.assertEqual(row['outcome'],old['outcome'])
            for key in ['peak_tilt_deg','max_displacement_m','final_position_m','torque_squared_integral']:self.assertAlmostEqual(row[key],old[key],places=5)
if __name__=='__main__':unittest.main()
