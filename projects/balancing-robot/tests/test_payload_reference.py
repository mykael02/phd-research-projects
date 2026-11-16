"""Independent component-wise dynamics and archived baseline checks."""
import json,math,subprocess,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]

class PayloadReference(unittest.TestCase):
    def test_component_mass_matrix(self):
        script="""const P=require('./visualisation/payload.js'),B=require('./model.js'),c=require('./config.js');let rows=[];for(const m of [0,.25,.75,1.5])for(const h of [.1,.18,.3,.4]){const s=[.2,-.3,.27,-.4],p=P.combine(c.parameters,m,h).parameters;rows.push({m,h,p,s,d:B.derivative(s,.17,6,p)});}console.log(JSON.stringify(rows));"""
        rows=json.loads(subprocess.check_output(['node','-e',script],cwd=ROOT,text=True))
        for row in rows:
            m,h,p=row['m'],row['h'],row['p'];x,v,t,w=row['s']
            first=1.2*.18+m*h
            axle=.012+1.2*.18**2+m*h*h+m*(.08**2+.06**2)/12
            matrix=np.array([[.35+1.2+m+2*.00012/.045**2,first*math.cos(t)],[first*math.cos(t),axle]])
            force=np.array([.17/.045-.08*v+first*math.sin(t)*w*w+6,first*9.81*math.sin(t)-.17-.002*w+6*(first/(1.2+m))*math.cos(t)])
            acc=np.linalg.solve(matrix,force)
            np.testing.assert_allclose([row['d'][1],row['d'][3]],acc,rtol=1e-13,atol=1e-13)

    def test_selected_python_trajectories(self):
        import sys
        sys.path.insert(0,str(ROOT))
        from balance import Parameters,simulate
        script="""const P=require('./visualisation/payload.js'),c=require('./config.js'),{trial}=require('./run_payload.js');let r=[];for(const controller of ['pid','lqr'])for(const m of [0,.75,1.5]){const load={mass_kg:m,height_m:.4},condition={initial_tilt_deg:5,push_force_n:5,torque_limit_nm:.7};r.push({controller,m,config:P.configure(c,m,.4).config,result:trial(controller,condition,load).metrics});}console.log(JSON.stringify(r));"""
        rows=json.loads(subprocess.check_output(['node','-e',script],cwd=ROOT,text=True))
        for row in rows:
            metrics,_=simulate(row['controller'],row['config'],initial_degrees=5,push_force=5,params=Parameters(**row['config']['parameters']))
            expected='fallen' if metrics['fallen'] else 'unsettled' if metrics['recovery_s'] is None else 'recovered'
            self.assertEqual(row['result']['outcome'],expected)
            for field in ['peak_tilt_deg','max_displacement_m','final_position_m','torque_squared_integral']:
                self.assertAlmostEqual(row['result'][field],metrics[field],places=5)

if __name__=='__main__':unittest.main()
