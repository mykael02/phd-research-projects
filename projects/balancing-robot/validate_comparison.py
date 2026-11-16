"""Independent component mass matrix + DOP853; held 200 Hz saturated LQR."""
import json,math,subprocess
from pathlib import Path
import numpy as np
from scipy.integrate import solve_ivp
from design_payload_lqr import linear,parameters
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'results/comparison'
def main():
    # Check the analytical linearisation against finite differences of the archived model.
    from balance import Parameters,derivative
    for m,h in [(0,.18),(.35,.14),(.65,.26),(1.15,.36)]:
        p=parameters(m,h);A,B=linear(p);eps=1e-6
        fd=np.column_stack([(np.array(derivative(eps*v,0,p=Parameters(**p)))-np.array(derivative(-eps*v,0,p=Parameters(**p))))/(2*eps) for v in np.eye(4)])
        np.testing.assert_allclose(A,fd,atol=1e-9,rtol=1e-10)
    script="""const {trial}=require('./run_comparison.js');let r=[];for(const [m,h] of [[.35,.14],[.65,.26],[1.15,.36]])for(const v of ['fixed','exact'])r.push(trial(v,{initial_tilt_deg:12,push_force_n:-15,torque_limit_nm:.6},{mass_kg:m,height_m:h},undefined,5003,true));console.log(JSON.stringify(r));"""
    records=json.loads(subprocess.check_output(['node','-e',script],cwd=ROOT,text=True));bank=json.loads((OUT/'gain_design.json').read_text())['bank'];checks=[]
    for record in records:
        r=record['metrics'];m,h=r['mass_kg'],r['height_m'];em,eh=r['estimated_mass_kg'],r['estimated_height_m'];K=np.array(bank[f'{em:.4f}/{(.18 if em==0 else eh):.4f}']['gain']);first=1.2*.18+m*h;M=1.2+m;L=first/M;J=.012+1.2*.18**2+m*h*h+m*(.08**2+.06**2)/12;A=.35+M+2*.00012/.045**2
        def rhs(t,s,u,f):
            x,v,theta,w=s;b=first*math.cos(theta);force=u/.045-.08*v+first*math.sin(theta)*w*w+f;moment=first*9.81*math.sin(theta)-u-.002*w+f*L*math.cos(theta)
            acc=np.linalg.solve([[A,b],[b,J]],[force,moment]);return [v,acc[0],w,acc[1]]
        s=np.array([0,0,math.radians(12),0]);states=[s.copy()];last_bad=None;fall=False
        for i in range(2000):
            t=i*.005;u=float(np.clip(-K@s,-.6,.6));f=-15 if 400<=i<440 else 0;s=solve_ivp(rhs,(t,t+.005),s,args=(u,f),method='DOP853',rtol=1e-11,atol=1e-13).y[:,-1];states.append(s.copy());end=(i+1)*.005
            if end>=2.2-1e-10 and (abs(s[0])>.03 or abs(s[1])>.03 or abs(s[2])>math.pi/180):last_bad=end
            if abs(s[2])>=math.pi/4:fall=True;break
        settled=max(2.2,2.2 if last_bad is None else last_bad+.005);outcome='fallen' if fall else 'recovered' if end-settled>=1-1e-10 else 'unsettled'
        ref=np.array([x['state'] for x in record['rows']]);assert len(ref)==len(states);error=float(np.max(np.abs(ref-np.array(states))));assert error<1e-4,error;assert outcome==r['outcome']
        checks.append(dict(mass_kg=m,height_m=h,variant=r['variant'],outcome=outcome,max_absolute_state_component_difference=error))
    (OUT/'independent_check.json').write_text(json.dumps({'method':'Independent component mass matrix, DOP853 rtol=1e-11 atol=1e-13; mixed state components in SI units','linearisation_check':'passed','checks':checks,'scope':'Numerical cross-check only; not hardware or independent multibody validation'},indent=2)+'\n');print('Independent dynamics/DOP853 check passed for six selected trajectories.')
if __name__=='__main__':main()
