"""Create print-quality vector and raster figures from verified numerical records."""
from collections import Counter
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.patches import Patch

HERE=Path(__file__).resolve().parent
FIG=HERE/'figures'
STATES=('recovered','unsettled','fallen')
LABELS=('Recovered','Upright, unsettled','Fallen')
COLORS=('#009E73','#E69F00','#CC79A7')
PID='#D55E00';LQR='#0072B2';ABL='#6B5CA5'
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'axes.titlesize':10,'axes.labelsize':9,
 'xtick.labelsize':8,'ytick.labelsize':8,'legend.fontsize':8,'axes.spines.top':False,'axes.spines.right':False,
 'pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none','savefig.dpi':600,'axes.linewidth':.6,
 'grid.alpha':.25,'grid.linewidth':.5})


def save(fig,name,caption,manifest):
    for ext in ('pdf','svg','png'):fig.savefig(FIG/f'{name}.{ext}',bbox_inches='tight',pad_inches=.08)
    manifest.append({'name':name,'caption':caption})
    plt.close(fig)


def outcome_map(ax,records,torque,controller,tilts,pushes,annotate=True):
    lookup={(r['initial_tilt_deg'],r['push_force_n']):r for r in records if r['controller']==controller and r['torque_limit_nm']==torque}
    z=np.array([[STATES.index(lookup[(t,p)]['outcome']) for p in pushes] for t in tilts])
    ax.imshow(z,origin='lower',aspect='auto',cmap=ListedColormap(COLORS),norm=BoundaryNorm([-.5,.5,1.5,2.5],3),interpolation='none')
    ax.set_xticks(range(0,len(pushes),2),pushes[::2]);ax.set_yticks(range(0,len(tilts),2),tilts[::2])
    ax.set_xlabel('Horizontal push (N)');ax.set_ylabel('Initial tilt (deg)')
    ax.set_title(f'{"PID-type" if controller=="pid" else "LQR"} | limit {torque:.1f} N m')
    if annotate:
        for iy in range(len(tilts)):
            for ix in range(len(pushes)):ax.text(ix,iy,'RUF'[z[iy,ix]],ha='center',va='center',fontsize=6.5,color='#101010')


def main():
    FIG.mkdir(parents=True,exist_ok=True)
    data=json.loads((HERE/'data'/'analysis.json').read_text());original=json.loads((HERE.parent/'results'/'robustness'/'results.json').read_text())
    records=data['baseline'];axes=original['metadata']['axes'];torques=axes['torque_limit_nm'];tilts=axes['initial_tilt_deg'];pushes=axes['push_force_n'];manifest=[]
    fig,axs=plt.subplots(1,2,figsize=(7.2,3.4),sharey=True,layout='constrained')
    for ax,name,title in zip(axs,('pid','lqr'),('PID-type','LQR')):
        bottom=np.zeros(len(torques))
        for state,label,color,hatch in zip(STATES,LABELS,COLORS,('', '//','xx')):
            counts=np.array([sum(r['controller']==name and r['torque_limit_nm']==t and r['outcome']==state for r in records) for t in torques])
            ax.bar(range(len(torques)),counts,bottom=bottom,color=color,hatch=hatch,label=label,edgecolor='white',linewidth=.3)
            bottom+=counts
        ax.set_xticks(range(len(torques)),[f'{t:g}' for t in torques]);ax.set_ylim(0,125);ax.set_title(title);ax.set_xlabel('Total torque limit (N m)');ax.set_ylabel('Conditions (117 per torque limit)')
    axs[1].legend(loc='upper left',bbox_to_anchor=(0,1.26),ncol=1,frameon=False)
    save(fig,'fig01_outcomes_by_torque','Outcome counts at each torque limit. Each bar contains 117 conditions: nine initial tilts and thirteen signed pushes. Colours and hatching distinguish recovery, upright non-recovery, and fall. All other model and gain settings are fixed.',manifest)
    fig,axs=plt.subplots(2,3,figsize=(7.2,5.3))
    for row,controller in enumerate(('pid','lqr')):
        for col,torque in enumerate((.1,.3,.7)):outcome_map(axs[row,col],records,torque,controller,tilts,pushes)
    fig.subplots_adjust(wspace=.48,hspace=.55,top=.91,bottom=.1,left=.075,right=.99)
    fig.legend([Patch(facecolor=c) for c in COLORS],LABELS,ncol=3,loc='upper center',frameon=False)
    save(fig,'fig02_selected_failure_maps','Sampled outcome maps for three torque limits. R denotes recovered, U upright but unsettled, and F fallen. The cells show evaluated points; boundaries between colours are not certified continuous operating limits. Complete maps for all seven limits appear in the technical report.',manifest)
    matrix=np.zeros((3,3),int)
    for pair in data['paired']:matrix[STATES.index(pair['pid']['outcome']),STATES.index(pair['lqr']['outcome'])]+=1
    fig,ax=plt.subplots(figsize=(5.2,4),layout='constrained');ax.imshow(matrix,cmap='Blues',vmin=0,vmax=max(1,matrix.max()))
    for (i,j),n in np.ndenumerate(matrix):ax.text(j,i,str(n),ha='center',va='center',fontsize=15,color='white' if n>matrix.max()*.55 else '#111111')
    ax.set_xticks(range(3),['Recovered','Unsettled','Fallen']);ax.set_yticks(range(3),['Recovered','Unsettled','Fallen']);ax.set_xlabel('LQR outcome');ax.set_ylabel('PID-type outcome')
    save(fig,'fig03_paired_outcomes','Paired outcomes across the same 819 physical conditions. Rows are PID-type outcomes and columns are LQR outcomes. Off-diagonal entries reveal disagreements that marginal success counts conceal.',manifest)
    common=[p for p in data['paired'] if p['pid']['outcome']==p['lqr']['outcome']=='recovered']
    metrics=[('recovery_s','Recovery time (s)'),('peak_tilt_deg','Peak absolute tilt (deg)'),('max_displacement_m','Maximum displacement (m)'),('torque_squared_integral',r'Torque effort ((N m)$^2$ s)')]
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.5),layout='constrained')
    for ax,(key,label) in zip(axs.flat,metrics):
        x=np.array([p['pid'][key] for p in common]);y=np.array([p['lqr'][key] for p in common]);lim=max(x.max(),y.max())*1.06
        ax.plot([0,lim],[0,lim],color='.5',ls='--',lw=.8);ax.scatter(x,y,s=13,color=LQR,alpha=.55,edgecolors='none')
        ax.set_xlim(-.015*lim,lim);ax.set_ylim(-.015*lim,lim);ax.set_xlabel('PID-type '+label);ax.set_ylabel('LQR '+label);ax.grid()
    save(fig,'fig04_matched_recovery_metrics',f'Paired metrics restricted to the {len(common)} conditions in which both controllers recovered. Points below the dashed identity line have a lower LQR value. This matched subset excludes every fall and non-recovery and must not be interpreted as an unconditional controller ranking.',manifest)
    conditions=[(5,5,.7),(-20,-10,.2)]
    fig,axs=plt.subplots(3,2,figsize=(7.2,6),sharex=True)
    for col,condition in enumerate(conditions):
        for controller,color,style in [('pid',PID,'-'),('lqr',LQR,'--')]:
            ex=data['examples'][f'{condition}_{controller}'];times=np.array([r['time'] for r in ex['rows']]);states=np.array([r['state'] for r in ex['rows']]);torque=np.array([r['torque'] for r in ex['rows']])
            for ax,values in zip(axs[:,col],[np.rad2deg(states[:,2]),states[:,0],torque]):ax.plot(times,values,color=color,ls=style,lw=1.25,label='PID-type' if controller=='pid' else 'LQR')
        axs[0,col].set_title(f'Tilt {condition[0]} deg, push {condition[1]} N, limit {condition[2]} N m',fontsize=9)
        for row,(label,tolerance) in enumerate([('Tilt (deg)',1),('Position (m)',.03),('Torque (N m)',condition[2])]):
            ax=axs[row,col];ax.axvspan(2,2.2,color='.8',alpha=.45);ax.axhline(tolerance,color='.5',ls=':',lw=.7);ax.axhline(-tolerance,color='.5',ls=':',lw=.7);ax.set_ylabel(label);ax.grid();ax.set_xlim(0,10)
        axs[-1,col].set_xlabel('Time (s)')
    axs[0,0].legend(frameon=False,fontsize=8);fig.subplots_adjust(wspace=.38,hspace=.3,left=.09,right=.99,bottom=.08,top=.94)
    save(fig,'fig05_response_trajectories','Illustrative trajectories for a nominal recovering condition (left) and the first PID-type unsettled condition in the original sweep order (right). Grey bands mark the force pulse. Dotted lines show the tilt and position recovery tolerances and the motor torque limits. These post hoc examples are explanatory, not representative statistical samples.',manifest)
    ext=data['extension'];fig,axs=plt.subplots(1,2,figsize=(7.2,3.4),layout='constrained')
    predicted=np.array([r['equilibrium_prediction_m'] for r in ext]);actual=np.array([r['final_position_m'] for r in ext]);axs[0].plot([-.05,.05],[-.05,.05],ls='--',lw=.8,color='.5');axs[0].scatter(predicted,actual,s=16,facecolors='none',edgecolors=PID,linewidths=.7)
    axs[0].set_xlabel(r'Predicted $-k_i z/k_x$ (m)');axs[0].set_ylabel('Final position at 30 s (m)');axs[0].grid();axs[0].set_xlim(-.05,.05);axs[0].set_ylim(-.05,.05)
    before=[abs(r['final_position_m'])*100 for r in records if r['outcome']=='unsettled'];after=[abs(r['final_position_m'])*100 for r in ext];bins=np.linspace(2.8,4.4,17)
    axs[1].hist(before,bins=bins,histtype='step',lw=1.3,label='At 10 s',color=LQR);axs[1].hist(after,bins=bins,histtype='step',lw=1.3,ls='--',label='At 30 s',color=PID);axs[1].axvline(3,color='.35',ls=':',label='Recovery limit');axs[1].set_xlabel('Absolute final position (cm)');axs[1].set_ylabel('Conditions');axs[1].legend(frameon=False)
    save(fig,'fig06_integral_equilibrium','Post hoc extension of all 220 originally unsettled PID-type trials to 30 s. Left: the measured position is compared with the equilibrium relation x = -(ki/kx) z. Right: the final-position distributions at 10 and 30 s relative to the 3 cm recovery limit. No controller gains or physical parameters change.',manifest)
    fig,axs=plt.subplots(1,2,figsize=(7.2,3.5),layout='constrained')
    groups=[[r for r in records if r['controller']=='pid'],data['ablation'],[r for r in records if r['controller']=='lqr']]
    bottom=np.zeros(3)
    for state,label,color,hatch in zip(STATES,LABELS,COLORS,('','//','xx')):
        counts=np.array([sum(r['outcome']==state for r in group) for group in groups]);axs[0].barh(range(3),counts,left=bottom,color=color,hatch=hatch,label=label,edgecolor='white',linewidth=.3)
        for y,(n,start) in enumerate(zip(counts,bottom)):
            if n>35:axs[0].text(start+n/2,y,str(n),ha='center',va='center',fontsize=8)
        bottom+=counts
    axs[0].set_yticks(range(3),['PID-type','PID-type, ki = 0','LQR']);axs[0].invert_yaxis();axs[0].set_xlabel('Conditions (819 per controller)');axs[0].legend(frameon=False,loc='upper center',bbox_to_anchor=(.5,1.3),fontsize=7)
    transition=np.zeros((3,3),int);base=[r for r in records if r['controller']=='pid']
    for old,new in zip(base,data['ablation']):transition[STATES.index(old['outcome']),STATES.index(new['outcome'])]+=1
    axs[1].imshow(transition,cmap='Blues',vmin=0,vmax=max(1,transition.max()))
    for (i,j),n in np.ndenumerate(transition):axs[1].text(j,i,str(n),ha='center',va='center',color='white' if n>transition.max()*.55 else '#111111',fontsize=11)
    axs[1].set_xticks(range(3),['R','U','F']);axs[1].set_yticks(range(3),['R','U','F']);axs[1].set_xlabel('Integral removed');axs[1].set_ylabel('Original PID-type')
    save(fig,'fig07_integral_ablation','Post hoc integral ablation on the full original grid. Only ki is changed from 0.4 to zero; all other PID-type gains, model parameters and limits remain fixed. The right matrix pairs original and ablated outcomes (R recovered, U unsettled, F fallen). This isolates an implementation choice and is not a newly optimized controller comparison.',manifest)
    fig,ax=plt.subplots(figsize=(6.5,3.4),layout='constrained')
    for name,color,style in [('pid',PID,'-'),('lqr',LQR,'--')]:
        vals=[r for r in data['tolerance_sensitivity'] if r['controller']==name];ax.plot([r['position_tolerance_m']*100 for r in vals],[r['recovered'] for r in vals],marker='o',color=color,ls=style,label='PID-type' if name=='pid' else 'LQR')
    tolerance=[.01,.02,.03,.04,.05,.06]
    counts=[sum(not r['fallen'] and r['tail_max_position_m']<=tol and r['tail_max_tilt_deg']<=1 and r['tail_max_speed_m_s']<=.03 for r in data['ablation']) for tol in tolerance]
    ax.plot(np.array(tolerance)*100,counts,marker='s',color=ABL,ls=':',label='PID-type, ki = 0');ax.axvline(3,color='.5',ls=':',lw=.8);ax.set_xlabel('Position recovery tolerance (cm)');ax.set_ylabel('Recovered conditions out of 819');ax.set_ylim(0,819);ax.set_xticks(range(1,7));ax.grid();ax.legend(frameon=False)
    save(fig,'fig08_tolerance_sensitivity','Post hoc sensitivity of the recovery label to the position tolerance. The final one-second dwell, 1 degree tilt bound, 0.03 m/s speed bound and 10-second horizon remain fixed. The dotted vertical line marks the original 3 cm tolerance. Curves are deterministic grid counts, without probability or confidence-interval interpretation.',manifest)
    for i,torque in enumerate(torques,1):
        fig,axs=plt.subplots(1,2,figsize=(7.2,3.35))
        for ax,controller in zip(axs,('pid','lqr')):outcome_map(ax,records,torque,controller,tilts,pushes)
        fig.subplots_adjust(wspace=.26,left=.08,right=.99,bottom=.16,top=.81)
        fig.legend([Patch(facecolor=c) for c in COLORS],LABELS,ncol=3,loc='upper center',frameon=False)
        save(fig,f'sfig{i:02d}_torque_{str(torque).replace(".","p")}',f'Complete paired outcome maps at a total torque limit of {torque:g} N m. Each map contains 117 conditions. R, U, and F label recovery, upright non-recovery, and fall respectively. Each trial has exact measurements and the same nominal body parameters.',manifest)
    (HERE/'figure_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    print(f'Saved {len(manifest)} figures in PDF, SVG and 600 dpi PNG formats.')


if __name__=='__main__':main()
