"""Reproduce the original study and add explicitly post hoc mechanism checks."""
from collections import Counter
from copy import deepcopy
from dataclasses import replace
import csv
import hashlib
import json
import math
from pathlib import Path
import platform
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from balance import Controller, Parameters, simulate
from run_robustness import outcome

OUT=ROOT/'publication'/'data'


def summarize_rows(rows, config, controller):
    tail=[r['state'] for r in rows if r['time']>=rows[-1]['time']-1-1e-9]
    control=Controller(controller,config)
    for row in rows[:-1]:control.command(row['state'],config['dt'],config['parameters']['torque_limit'])
    # The caller supplies the actual trial torque limit in its config copy.
    return {'final_integral_rad_s':control.integral,
            'tail_max_position_m':max(abs(s[0]) for s in tail),
            'tail_max_speed_m_s':max(abs(s[1]) for s in tail),
            'tail_max_tilt_deg':max(abs(math.degrees(s[2])) for s in tail),
            'final_speed_m_s':rows[-1]['state'][1],
            'final_omega_rad_s':rows[-1]['state'][3]}


def trial(original, config, duration=10):
    p=replace(Parameters(**config['parameters']),torque_limit=original['torque_limit_nm'])
    m,rows=simulate(original['controller'],config,initial_degrees=original['initial_tilt_deg'],
                    push_force=original['push_force_n'],params=p,duration=duration,dt=config['dt'])
    replay=deepcopy(config);replay['parameters']['torque_limit']=p.torque_limit
    return m,rows,summarize_rows(rows,replay,original['controller'])


def csv_write(name, records):
    with (OUT/name).open('w',newline='',encoding='utf-8') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(records[0]));writer.writeheader();writer.writerows(records)


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    source=ROOT/'results'/'robustness'/'results.json'
    data=json.loads(source.read_text());config=data['metadata']['config']
    assert hashlib.sha256((ROOT/'balance.py').read_bytes()).hexdigest()==data['metadata']['source_sha256']['balance.py'],'Original model changed'
    diagnostics=[];extensions=[];ablation=[];examples={}
    example_conditions=[(5,5,.7),(-20,-10,.2),(0,30,.7)]
    for index,old in enumerate(data['records']):
        metrics,rows,extra=trial(old,config)
        for key,value in metrics.items():assert value==old[key],(index,key,value,old[key])
        diagnostics.append({**old,**extra})
        condition=(old['initial_tilt_deg'],old['push_force_n'],old['torque_limit_nm'])
        if condition in example_conditions:
            examples[f"{condition}_{old['controller']}"]={'condition':condition,'controller':old['controller'],'metrics':metrics,'rows':rows}
        if (index+1)%234==0:print(f'Reproduced {index+1}/1638 original trials',flush=True)
    csv_write('baseline_diagnostics.csv',diagnostics)
    for index,old in enumerate(r for r in diagnostics if r['outcome']=='unsettled'):
        metrics,rows,extra=trial(old,config,duration=30)
        prediction=-config['pid']['ki']/config['pid']['kx']*extra['final_integral_rad_s']
        extensions.append({**{k:old[k] for k in ('initial_tilt_deg','push_force_n','torque_limit_nm','controller')},
                           **metrics,'outcome':outcome(metrics),**extra,'equilibrium_prediction_m':prediction,
                           'equilibrium_residual_m':rows[-1]['state'][0]-prediction})
        if index%50==49:print(f'Extended {index+1}/220 unsettled trials to 30 seconds',flush=True)
    csv_write('horizon_extension.csv',extensions)
    ablated=deepcopy(config);ablated['pid']['ki']=0.0
    for index,old in enumerate(r for r in diagnostics if r['controller']=='pid'):
        metrics,rows,extra=trial(old,ablated)
        ablation.append({**{k:old[k] for k in ('initial_tilt_deg','push_force_n','torque_limit_nm')},
                         **metrics,'outcome':outcome(metrics),**extra})
        if (index+1)%117==0:print(f'Integral ablation {index+1}/819 trials',flush=True)
    csv_write('integral_ablation.csv',ablation)
    a=np.array(config['linear_a']);b=np.array(config['linear_b']);g=config['pid']
    matrix=np.zeros((5,5));matrix[:4,:4]=a+b@np.array([[g['kx'],g['kv'],g['kp'],g['kd']]])
    matrix[:4,4]=b[:,0]*g['ki'];matrix[4,2]=1
    eigenvalues=np.linalg.eigvals(matrix)
    sensitivity=[]
    for tolerance in (.01,.02,.03,.04,.05,.06):
        for name in ('pid','lqr'):
            selected=[r for r in diagnostics if r['controller']==name]
            count=sum(not r['fallen'] and r['tail_max_position_m']<=tolerance and r['tail_max_tilt_deg']<=1 and r['tail_max_speed_m_s']<=.03 for r in selected)
            sensitivity.append({'controller':name,'position_tolerance_m':tolerance,'recovered':count,'tested':len(selected)})
            if tolerance==.03:assert count==sum(r['outcome']=='recovered' for r in selected)
    csv_write('tolerance_sensitivity.csv',sensitivity)
    paired=[]
    lqr={(r['initial_tilt_deg'],r['push_force_n'],r['torque_limit_nm']):r for r in diagnostics if r['controller']=='lqr'}
    for pid in (r for r in diagnostics if r['controller']=='pid'):
        l=lqr[(pid['initial_tilt_deg'],pid['push_force_n'],pid['torque_limit_nm'])]
        paired.append({'pid':pid,'lqr':l})
    analysis={'baseline':diagnostics,'extension':extensions,'ablation':ablation,'paired':paired,'examples':examples,
              'tolerance_sensitivity':sensitivity,'pid_linear_poles':[[float(x.real),float(x.imag)] for x in eigenvalues],
              'metadata':{'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'source':str(source.relative_to(ROOT)),
                          'python':platform.python_version(),'numpy':np.__version__,
                          'analysis_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                          'baseline_reproduced':len(diagnostics),'extension_design':'Post hoc: all 220 baseline PID unsettled trials extended to 30 s',
                          'ablation_design':'Post hoc: ki=0 only, other PID gains unchanged, all 819 conditions, 10 s'}}
    (OUT/'analysis.json').write_text(json.dumps(analysis,separators=(',',':'))+'\n',encoding='utf-8')
    print('Extension:',dict(Counter(r['outcome'] for r in extensions)),flush=True)
    print('Ablation:',dict(Counter(r['outcome'] for r in ablation)),flush=True)
    print('Maximum 30 s equilibrium residual (m):',max(abs(r['equilibrium_residual_m']) for r in extensions),flush=True)
    print('Paired:',dict(Counter((r['pid']['outcome'],r['lqr']['outcome']) for r in paired)),flush=True)


if __name__=='__main__':main()
