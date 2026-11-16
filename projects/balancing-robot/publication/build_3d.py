"""Export unmodified recorded samples for offline 3D views and a static 3D figure."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];HERE=ROOT/'publication';OUT=ROOT/'visualisation'

def main():
    analysis=json.loads((HERE/'data/analysis.json').read_text())
    baseline=json.loads((ROOT/'results/robustness/results.json').read_text())
    scenarios=[]
    conditions=[((5,5,.7),'Nominal recovery · 5° / +5 N / 0.70 N·m'),((-20,-10,.2),'Different outcomes · −20° / −10 N / 0.20 N·m'),((0,30,.7),'Strong pulse · 0° / +30 N / 0.70 N·m')]
    for condition,label in conditions:
        scenario={'condition':condition,'label':label}
        for controller in ('pid','lqr'):
            ex=next(e for e in analysis['examples'].values() if e['controller']==controller and tuple(e['condition'])==condition)
            record=next(r for r in baseline['records'] if r['controller']==controller and (r['initial_tilt_deg'],r['push_force_n'],r['torque_limit_nm'])==condition)
            scenario[controller]={**ex,'outcome':record['outcome']}
        scenarios.append(scenario)
    (OUT/'replay-data.js').write_text('window.REPLAY_DATA='+json.dumps({'scenarios':scenarios},separators=(',',':'))+';\n',encoding='utf-8')
    (OUT/'experiment-data.js').write_text('window.EXPERIMENT_DATA='+json.dumps(baseline['records'],separators=(',',':'))+';\n',encoding='utf-8')
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'pdf.fonttype':42,'svg.fonttype':'none'})
    fig=plt.figure(figsize=(7.5,8.8))
    for i,scenario in enumerate(scenarios):
        ax=fig.add_subplot(3,1,i+1,projection='3d')
        for controller,color,style in [('pid','#bd531e','-'),('lqr','#087b88','--')]:
            trial=scenario[controller];s=np.array([row['state'] for row in trial['rows']]);x=s[:,0];v=s[:,1];theta=np.rad2deg(s[:,2]);ax.plot(x,v,theta,color=color,ls=style,lw=1.2,label='PID-type' if controller=='pid' else 'LQR');ax.scatter([x[0]],[v[0]],[theta[0]],marker='o',s=24,color=color);ax.scatter([x[-1]],[v[-1]],[theta[-1]],marker='x',s=35,color=color)
        ax.set_xlabel('Displacement (m)',labelpad=7);ax.set_ylabel('Speed (m/s)',labelpad=7);ax.set_zlabel('Tilt (deg)',labelpad=6);ax.set_title(f"({chr(97+i)}) Initial tilt {scenario['condition'][0]} deg, push {scenario['condition'][1]} N, limit {scenario['condition'][2]} N m",pad=4,fontsize=9);ax.view_init(elev=22,azim=-58);ax.set_box_aspect((1.9,1,1));ax.tick_params(labelsize=7,pad=1)
        if i==0:ax.legend(loc='upper left',bbox_to_anchor=(-.6,1.05),frameon=False)
    fig.subplots_adjust(left=.16,right=.83,bottom=.06,top=.96,hspace=.25)
    name='fig09_3d_state_trajectories'
    for extension in ('pdf','svg','png'):fig.savefig(HERE/'figures'/f'{name}.{extension}',dpi=600,bbox_inches='tight',pad_inches=.14)
    plt.close(fig)
    manifest=json.loads((HERE/'figure_manifest.json').read_text());manifest=[m for m in manifest if m['name']!=name];manifest.append({'name':name,'caption':'Three-dimensional state trajectories in displacement, speed and tilt for three explicitly selected examples. Solid orange: PID-type; dashed teal: LQR. Circles mark initial states and crosses final states. Paths stop at 10 s or the declared fall, with no ground-impact extrapolation. These are planar-model state-space curves, not three-dimensional rigid-body motion; each panel uses its own axis scale.'});(HERE/'figure_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('Exported six recorded trajectories, 1,638 grid records and static 3D state figure.')

if __name__=='__main__':main()
