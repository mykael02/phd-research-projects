"""Step 4 frozen-gain payload study: figures, summaries and separate report."""
from collections import Counter,defaultdict
import csv,hashlib,html,json,math,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent;OUT=ROOT/'results/payload';FIG=HERE/'figures'
sys.path.insert(0,str(HERE))
from build_reports import setup_fonts,build_pdf,build_tex
from build_sensing_report import interval
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'axes.labelsize':9,'axes.titlesize':10,'legend.fontsize':8,'xtick.labelsize':8,'ytick.labelsize':8,'pdf.fonttype':42,'svg.fonttype':'none','axes.spines.top':False,'axes.spines.right':False,'grid.alpha':.2})
HEIGHTS=[.1,.18,.3,.4];MASSES=[0,.25,.5,.75,1,1.5];COLORS=['#176b87','#558444','#c66427','#76499e'];PROFILES=['direct_0ms','kalman_0ms','kalman_10ms'];PLABELS=['Direct, 0 ms','Kalman, 0 ms','Kalman, 10 ms']
LOADS=[(0,.18),(.75,.18),(.75,.4),(1.5,.4)];LLABELS=['0 kg','0.75 kg\n0.18 m','0.75 kg\n0.40 m','1.50 kg\n0.40 m']
def sub(rows,**kw):return [r for r in rows if all(r[k]==v for k,v in kw.items())]
def load_rows(rows,m,h,**kw):return sub(rows,mass_kg=m,height_m=.18 if m==0 else h,**kw)
def save_csv(name,rows):
    with (OUT/name).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)

def main():
    data=json.loads((OUT/'results.json').read_text());audit=json.loads((OUT/'integration_audit.json').read_text());rows=data['records'];assert len(rows)==4176
    for f,digest in data['metadata']['source_sha256'].items():assert hashlib.sha256((ROOT/f).read_bytes()).hexdigest()==digest,f
    ideal=sub(rows,phase='ideal_grid');noisy=sub(rows,phase='sensing');assert len(ideal)==2016 and len(noisy)==2160
    assert audit['trials']==2016
    groups=defaultdict(list)
    for r in noisy:groups[(r['controller'],r['scenario'],r['mass_kg'],r['height_m'],r['profile'])].append(r)
    summary=[]
    for (c,s,m,h,p),g in groups.items():
        assert len(g)==30 and len({r['seed'] for r in g})==30
        counts=Counter(r['outcome'] for r in g);lo,hi=interval(counts['fallen'],30)
        summary.append(dict(controller=c,scenario=s,mass_kg=m,height_m=h,profile=p,n=30,recovered=counts['recovered'],unsettled=counts['unsettled'],fallen=counts['fallen'],fall_ci_low=lo,fall_ci_high=hi))
    assert len(summary)==72;save_csv('sensing_summary.csv',summary)
    grid=[]
    for c in ['pid','lqr']:
        baseline=load_rows(ideal,0,.18,controller=c)
        key=lambda r:(r['initial_tilt_deg'],r['push_force_n'],r['torque_limit_nm'])
        base={key(r):r for r in baseline}
        for m in MASSES:
            for h in ([.18] if m==0 else HEIGHTS):
                g=load_rows(ideal,m,h,controller=c);assert len(g)==48
                counts=Counter(r['outcome'] for r in g)
                grid.append(dict(controller=c,mass_kg=m,height_m=h,n=48,recovered=counts['recovered'],unsettled=counts['unsettled'],fallen=counts['fallen'],baseline_fall_to_upright=sum(base[key(r)]['outcome']=='fallen' and r['outcome']!='fallen' for r in g),baseline_upright_to_fall=sum(base[key(r)]['outcome']!='fallen' and r['outcome']=='fallen' for r in g)))
    save_csv('ideal_summary.csv',grid);(OUT/'summary.json').write_text(json.dumps({'ideal':grid,'sensing':summary},indent=2)+'\n')
    manifest={}
    def save(fig,name,caption):
        for ext in ['pdf','svg','png']:
            target=FIG/f'{name}.{ext}'
            if '--reuse-figures' not in sys.argv or not target.exists() or ('--refresh-geometry' in sys.argv and name=='step4_fig07_payload_geometry'):fig.savefig(target,dpi=600,bbox_inches='tight',pad_inches=.12)
        plt.close(fig);manifest[name]={'name':name,'caption':caption}
    fig,axs=plt.subplots(1,2,figsize=(7.2,3.1),layout='constrained')
    for h,col in zip(HEIGHTS,COLORS):
        a=[load_rows(ideal,m,h)[0] for m in MASSES]
        axs[0].plot(MASSES,[r['com_m'] for r in a],'o-',color=col,label=f'h = {h:.2f} m')
        axs[1].plot(MASSES,[r['axle_inertia_kg_m2'] for r in a],'o-',color=col)
    axs[0].set_ylabel('Combined COM distance from axle (m)');axs[1].set_ylabel('Pitch inertia about axle (kg m$^2$)');axs[0].legend()
    for ax in axs:ax.set_xlabel('Added payload mass (kg)');ax.grid()
    save(fig,'step4_fig01_mass_properties','Rigid-body aggregation across the evaluated payload masses and mounting heights h. COM is measured along the body from the axle. The payload is a centred cuboid, 0.08 x 0.06 x 0.12 m; its own pitch inertia is included. Curves connect evaluated masses. The unloaded robot is the same at every nominal mounting height.')
    fig,axs=plt.subplots(2,3,figsize=(7.4,5.1),layout='constrained');outcomes=['recovered','unsettled','fallen'];cmap=ListedColormap(['#b9dfcf','#f2d4a4','#d8afbc'])
    for i,c in enumerate(['pid','lqr']):
        for j,limit in enumerate([.2,.4,.7]):
            ax=axs[i,j];a=[[outcomes.index(load_rows(ideal,m,h,controller=c,initial_tilt_deg=0,push_force_n=20,torque_limit_nm=limit)[0]['outcome']) for m in MASSES] for h in HEIGHTS]
            ax.imshow(a,origin='lower',aspect='auto',cmap=cmap,norm=BoundaryNorm([-.5,.5,1.5,2.5],3))
            for y in range(4):
                for x in range(6):ax.text(x,y,['R','U','F'][a[y][x]],ha='center',va='center',fontsize=8)
            ax.set(xticks=range(6),xticklabels=MASSES,yticks=range(4),yticklabels=HEIGHTS,xlabel='Added mass (kg)',ylabel='Height h (m)',title=f'{c.upper()} | limit {limit:.1f} N m')
    fig.legend(handles=[Patch(color=cmap(i),label=label) for i,label in enumerate(['R: recovered','U: upright, unsettled','F: fallen'])],loc='outside upper center',ncol=3)
    save(fig,'step4_fig02_outcome_map','Deterministic outcomes for zero initial tilt and a 20 N pulse, separated by controller and torque limit. Each distinct loaded cell is one trial. The zero-mass column repeats the same unloaded trial across heights for reference; those repetitions are not additional trials. Masses and heights are discrete categories, not equally spaced numerical axes. These maps do not fit a stability boundary.')
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.2),layout='constrained')
    for i,c in enumerate(['pid','lqr']):
        for h,col in zip(HEIGHTS,COLORS):
            vals=[load_rows(grid,m,h,controller=c)[0] for m in MASSES]
            for j,metric in enumerate(['recovered','fallen']):axs[i,j].plot(MASSES,[r[metric]/48 for r in vals],'o-',color=col,label=f'h = {h:.2f} m')
        for j in range(2):axs[i,j].set(title=c.upper(),xlabel='Added mass (kg)',ylabel=['Recovered / 48 conditions','Fallen / 48 conditions'][j],ylim=(-.03,1.03));axs[i,j].grid()
    axs[0,0].legend(fontsize=7,ncol=2)
    save(fig,'step4_fig03_grid_fractions','Fractions of the same 48 deterministic initial-tilt/force/torque conditions at each load: four tilts, four force levels and three torque limits. The denominator includes zero-force cases. These equally weighted grid summaries are descriptive, not success probabilities for an operating population. No sampling confidence intervals apply. Lines only connect evaluated masses.')
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.0),layout='constrained')
    for j,c in enumerate(['pid','lqr']):
        for m,col in zip([0,.75,1.5],[COLORS[0],COLORS[2],COLORS[3]]):
            trace=data['examples'][c+'_'+str(m if m else 0)]['rows'];t=[r['time'] for r in trace]
            axs[0,j].plot(t,[r['state'][2]*180/math.pi for r in trace],color=col,label=f'{m:g} kg')
            axs[1,j].plot(t,[r['state'][0] for r in trace],color=col)
        axs[0,j].set_title(c.upper());axs[0,j].set_ylabel('True tilt (deg)');axs[1,j].set_ylabel('Position (m)')
        for ax in axs[:,j]:ax.axvspan(2,2.2,color='#d8a742',alpha=.18);ax.grid();ax.set_xlabel('Time (s)')
    axs[0,0].legend()
    save(fig,'step4_fig04_trajectories','Protocol-selected ideal-sensing examples: 5 deg initial tilt, 5 N pulse and 0.7 N m torque limit. Loaded examples mount their payload at h=0.40 m. The shaded interval is the pulse. All six trials last 10 s and recover. These traces illustrate one condition and do not establish that increasing payload always improves balance.')
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.4),layout='constrained')
    for i,s in enumerate(['large_push','low_torque']):
        for j,c in enumerate(['pid','lqr']):
            ax=axs[i,j]
            for k,(profile,col) in enumerate(zip(PROFILES,[COLORS[2],COLORS[3],COLORS[0]])):
                a=[sub(summary,controller=c,scenario=s,mass_kg=m,height_m=h,profile=profile)[0] for m,h in LOADS];y=np.array([r['fallen']/30 for r in a]);lo=np.array([r['fall_ci_low'] for r in a]);hi=np.array([r['fall_ci_high'] for r in a]);ax.errorbar(np.arange(4)+(k-1)*.19,y,yerr=[np.maximum(0,y-lo),np.maximum(0,hi-y)],fmt='o',ms=4,capsize=2,color=col,label=PLABELS[k])
            ax.set(xticks=range(4),xticklabels=LLABELS,ylim=(-.04,1.04),ylabel='Falls / 30 seeds',title=c.upper()+' | '+s.replace('_',' '));ax.grid(axis='y')
    axs[0,0].legend(fontsize=7)
    save(fig,'step4_fig05_sensing_falls','Fall fractions and Wilson 95% intervals for fixed noisy conditions. Each point uses 30 paired seeds, angle-noise SD 0.5 deg and gyro-noise SD 2 deg/s. The first load is the unloaded reference; labels for loaded cases give mass and mounting height. Large-push and low-torque scenarios retain their original definitions. Zero observed falls does not imply zero failure probability. Profiles share seeds and are not independent groups.')
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.1),layout='constrained')
    for j,c in enumerate(['pid','lqr']):
        for i,(metric,label) in enumerate([('tail_tilt_rms_deg','Final-second tilt RMS (deg)'),('tail_position_rms_m','Final-second position RMS (m)')]):
            ax=axs[i,j]
            for k,(profile,col) in enumerate(zip(PROFILES,[COLORS[2],COLORS[3],COLORS[0]])):
                g=[load_rows(noisy,m,h,controller=c,scenario='nominal',profile=profile) for m,h in LOADS];assert all(len(a)==30 and all(r['duration_s']==10 and r['outcome']!='fallen' for r in a) for a in g)
                q=np.array([np.quantile([r[metric] for r in a],[.25,.5,.75]) for a in g]);ax.errorbar(np.arange(4)+(k-1)*.19,q[:,1],yerr=[q[:,1]-q[:,0],q[:,2]-q[:,1]],fmt='o',capsize=2,color=col,label=PLABELS[k])
            ax.set(xticks=range(4),xticklabels=LLABELS,ylabel=label,title=c.upper());ax.grid(axis='y')
    axs[0,0].legend(fontsize=7)
    save(fig,'step4_fig06_tail_rms','Nominal-condition final-one-second RMS metrics, sampled at 200 Hz including both endpoints from 9 to 10 s. Points show medians and bars the interquartile range across 30 seeds, not confidence intervals. Every plotted trial survives for 10 s but fails the strict continuous-dwell recovery criterion. These additional metrics describe persistent motion without redefining the recovery labels. Full-duration-only speed RMS is also supplied in the CSV.')
    fig=plt.figure(figsize=(8,3.7),layout='constrained')
    def box(ax,centre,size,color):
        x,y,z=centre;w,d,h=size;verts=np.array([[x+a*w/2,y+b*d/2,z+c*h/2] for a,b,c in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]])
        ax.add_collection3d(Poly3DCollection([[verts[i] for i in f] for f in [(0,1,2,3),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]],facecolors=color,edgecolors='#456070',linewidths=.4,alpha=.65))
    for i,(m,h) in enumerate([(0,.18),(.75,.18),(.75,.4)]):
        ax=fig.add_subplot(1,3,i+1,projection='3d',computed_zorder=False);p=load_rows(ideal,m,h)[0];R=.045
        box(ax,(0,0,R+.18),(.085,.145,.20),'#72a9bb');ax.plot([0,0],[0,0],[R,R+(max(.18,h) if m else .18)],color='#627888',lw=2)
        if m:box(ax,(0,0,R+h),(.08,.12,.06),'#daa83a')
        t=np.linspace(0,2*np.pi,80)
        for y in [-.115,.115]:ax.plot(R*np.sin(t),np.full_like(t,y),R+R*np.cos(t),color='#263e4b',lw=3)
        ax.scatter([0],[0],[R+p['com_m']],color='#f9ca31',s=30,edgecolor='#7d4b00',depthshade=False,zorder=10)
        ax.set(xlim=(-.15,.15),ylim=(-.17,.17),zlim=(0,.5),xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)',title='Unloaded' if not m else f'{m:g} kg at {h:.2f} m');ax.set_xticks([-.1,0,.1]);ax.set_yticks([-.1,0,.1]);ax.set_zticks([0,.2,.4]);ax.set_box_aspect((.30,.34,.50));ax.view_init(elev=19,azim=-56);ax.tick_params(labelsize=6)
    save(fig,'step4_fig07_payload_geometry','Three-dimensional schematic at the upright pose, not a simulated response or a hardware design. Gold cuboid: payload; gold point: combined body COM; blue shell: illustrative original body envelope. Payload dimensions are those used for its pitch inertia. The original shell is only a drawing, and ballast may overlap its envelope. All three panels share the same metre scale. Dynamics remain planar, with no yaw, roll, slip or collision simulation.')
    (HERE/'payload_figure_manifest.json').write_text(json.dumps(list(manifest.values()),indent=2)+'\n')
    blocks=[]
    def h(t):blocks.append(('heading',t))
    def p(t):blocks.append(('paragraph',t))
    def page(t,figure=None):blocks.append(('pagebreak',));h(t);blocks.append(('figure',figure)) if figure else None
    title='Payload mass and mounting height in a balancing robot'
    h('Scope and main findings')
    p('This Step 4 research supplement extends the existing planar balancing-robot model with a rigid, centred payload. It evaluates frozen nominal PID-type and LQR gains over 2,016 deterministic trials and 2,160 trials combining loading with noisy sensing. The 4,176 primary records are accompanied by a separate integration audit. This is an exploratory simulation study, not a hardware experiment, a held-out benchmark or a matched-tuning comparison of controller classes.')
    counts=[Counter(r['outcome'] for r in sub(ideal,controller=c)) for c in ['pid','lqr']]
    blocks.append(('table','Deterministic inventory: 1,008 trials per controller',['Controller','Recovered','Unsettled','Fallen'],[[c.upper(),a['recovered'],a['unsettled'],a['fallen']] for c,a in zip(['pid','lqr'],counts)],[.28,.24,.24,.24]))
    p('The inventory equally weights the 21 distinct load configurations, each with 48 initial-tilt/force/torque conditions. It is not a deployment probability. Payload does not produce a uniform deterioration: the selected nominal high-mounted examples show smaller peak tilt with added mass. That finding depends on the geometry, frozen gains and disturbance definition. Mass, COM and inertia all change, and the push acts at the loaded-body COM, so this study does not isolate mass alone.')
    p('In the noisy subset, PID-type has 1,080 upright-but-unsettled trials. LQR has 1,026 upright-but-unsettled trials and 54 falls. No noisy trial satisfies the original final-one-second dwell requirement. Conditional tail RMS metrics therefore describe the remaining motion separately from recovery. The selected nominal noisy comparisons all survive the full 10 seconds.')
    h('1 Rigid payload model')
    p('The original body has mass m0=1.2 kg, axle-to-COM distance l0=0.18 m and pitch inertia I0=0.012 kg m^2 about its own COM. A payload of mass mp is fixed on the body centreline, with COM distance h from the axle. Its rectangular dimensions are width w=0.08 m in the fore-aft direction, height b=0.06 m along the body and lateral depth 0.12 m. Density varies with the selected mass; dimensions stay fixed. The attachment is assumed massless and rigid. No payload is added or removed during a trial.')
    p('The combined mass M, COM distance L and COM pitch inertia I follow component mass weighting and the parallel-axis theorem [1]. For rotation about the lateral axle, payload depth does not enter its own pitch inertia Ip. No horizontal COM offset is introduced, so the upright equilibrium angle remains zero.')
    blocks.append(('equation',r'M=m_0+m_p,\quad L=(m_0l_0+m_ph)/M,\quad I_p=m_p(w^2+b^2)/12'))
    blocks.append(('equation',r'I=I_0+m_0(l_0-L)^2+I_p+m_p(h-L)^2'))
    p('The independent identity I + M L^2 = I0 + m0 l0^2 + Ip + mp h^2 verifies inertia about the axle. Substituting M, L and I into the existing nonlinear plant updates translational inertia, gravity, translation-pitch coupling and rotational inertia together. Zero payload returns the original parameter values exactly.')
    page('2 Experimental protocol')
    p('Added masses are 0, 0.25, 0.50, 0.75, 1.00 and 1.50 kg. Nonzero masses use h=0.10, 0.18, 0.30 and 0.40 m. Zero mass is evaluated once, at the bookkeeping height 0.18 m, giving 21 unique loads. The deterministic grid combines initial tilts 0, 5, 10 and 15 deg; horizontal forces 0, 5, 10 and 20 N; and total motor-torque limits 0.2, 0.4 and 0.7 N m. There are 48 conditions per load and controller, hence 2,016 trials. Only nonnegative initial tilts and force directions are covered; opposite-direction disturbances remain untested here.')
    p('The sensing subset uses four loads: unloaded, 0.75 kg at 0.18 m, 0.75 kg at 0.40 m, and 1.50 kg at 0.40 m. Three physical scenarios retain the Step 3 definitions below. Each is evaluated with direct measurements at zero delay, Kalman-filtered measurements at zero delay, and Kalman-filtered measurements at 10 ms delay. Angle-noise SD is 0.5 deg and gyro-noise SD is 2 deg/s. Thirty seeds per combination produce 2,160 trials.')
    blocks.append(('table','Physical scenarios for the sensing subset',['Scenario','Initial tilt (deg)','Force (N)','Limit (N m)'],[['Nominal',5,5,.7],['Large push',0,20,.7],['Low torque',0,5,.2]],[.28,.24,.24,.24]))
    p('Every run lasts at most 10 s. A nonzero force is held from 2.00 to 2.20 s. It acts horizontally at the combined loaded-body COM; its pitch moment is F L cos(theta). Thus the application point moves with COM as payload changes. This preserves the original COM-force convention, but is not equivalent to pushing a fixed chassis attachment. Base/wheel properties, gravity and drag are unchanged. Control and sensing run at 200 Hz; nominal plant integration uses one fourth-order Runge-Kutta step per control interval. Motor torque is saturated at the selected total two-motor limit.')
    p('Both original controllers and the Step 3 Kalman estimator are frozen. The estimator still processes tilt and angular rate only, with ideal but delayed position and velocity channels, fixed per-interval angular-acceleration SD 50 rad/s^2, and no delay prediction. True plant states are used only for physics, visualisation and evaluation. Matching seed 1001+104729*i, i=0..29, supplies paired innovations across controllers, loads and sensing profiles. The protocol was written before this sweep but uses previously explored scenarios; it is not preregistered or an independent generalisation test.')
    h('3 Outcomes and uncertainty')
    p('A fall occurs when absolute true tilt reaches 45 deg at a control endpoint. Recovery requires a final continuous one-second dwell within absolute tilt 1 deg, position 0.03 m and speed 0.03 m/s, after the pulse ends; for zero-force cases the reference is t=0. Surviving runs that miss this criterion are unsettled. Persistent noise can prevent recovery under this strict rule even when posture remains bounded.')
    p('Final-second RMS tilt, position and speed use the 201 control samples from 9 to 10 s, including both endpoints. They are null for falls and are compared only in groups where every trial runs for 10 s. Peak tilt, displacement, saturation time and integrated squared torque use the available pre-termination record; different fall times therefore imply different horizons. Squared torque is a control-effort measure, not electrical energy. Wilson 95% intervals apply only to noisy fixed-condition fall fractions. IQR bars describe seed variation and are not confidence intervals. Deterministic grid fractions carry no sampling intervals; shared seeds do not make observer or load groups independent.')
    page('4 How the mass distribution changes','step4_fig01_mass_properties')
    p('Moving a fixed mass above or below the original COM shifts the combined COM in the expected direction. Axle inertia increases for every positive added mass, while the rate of increase depends strongly on mounting height. These simultaneous changes explain why added load cannot be represented by changing body_mass alone.')
    page('5 Loaded-body geometry in three dimensions','step4_fig07_payload_geometry')
    p('The interactive lab now exposes payload mass and height alongside sensing settings. A gold cuboid shows the payload and the yellow point tracks the combined COM. The force arrow starts at that COM. The purple outline represents the estimated body and payload pose; changing load resets the run. JSON exports retain nominal and loaded parameters, load geometry, sensing settings, force events and trajectories. Recorded original experiments retain their original geometry and parameters.')
    page('6 Deterministic outcomes under a large push','step4_fig02_outcome_map')
    p('Read each cell as one evaluated condition. The outcomes reflect nonlinear response and saturation under frozen gains. The map deliberately retains both upright-but-unsettled and recovered states, because falling and position recovery answer different questions. It is not a load rating or a certified operating envelope.')
    page('7 Summaries over the common condition grid','step4_fig03_grid_fractions')
    p('For each loaded case, ideal_summary.csv also pairs its 48 outcomes with the same unloaded conditions. It records baseline-fall-to-upright and baseline-upright-to-fall transitions, so improvements and regressions can be inspected separately. Aggregate fractions can conceal which individual conditions changed. No claim of monotonic safety improvement, optimal mounting height or superiority of PID or LQR as a class is justified by this design.')
    page('8 Selected time responses','step4_fig04_trajectories')
    example_table=[]
    for c in ['pid','lqr']:
        for m in [0,.75,1.5]:
            r=data['examples'][c+'_'+str(m if m else 0)]['metrics'];example_table.append([c.upper(),m,f"{r['peak_tilt_deg']:.3f}",f"{r['recovery_s']:.3f}"])
    blocks.append(('table','Selected nominal examples, h=0.40 m when loaded',['Controller','Added mass (kg)','Peak tilt (deg)','Recovery after pulse (s)'],example_table,[.22,.24,.25,.29]))
    page('9 Loading combined with noisy sensing','step4_fig05_sensing_falls')
    p('For LQR in the large-push scenario with Kalman sensing and 10 ms delay, falls are 23/30 unloaded, 0/30 at 0.75 kg and h=0.18 m, 30/30 at 0.75 kg and h=0.40 m, and 1/30 at 1.50 kg and h=0.40 m. This nonmonotone response is specific to those fixed conditions; more load is neither uniformly better nor uniformly worse.')
    p('This small sensing subset is a joint exploratory comparison, not a full mass-height-noise-delay factorial study. Noise may excite motion that is absent under exact state feedback. The filter smooths measurements at the delivered packet time; 10 ms delay remains uncompensated. Thirty seeds give limited resolution for uncommon failures. Zero observed falls in a cell has a Wilson upper bound of approximately 11.35%.')
    page('10 Persistent motion among full-duration survivors','step4_fig06_tail_rms')
    p('The nominal sensing trials are all upright but unsettled. Tail RMS reveals differences hidden by that shared label without silently changing the original recovery criterion. A low RMS tilt alone does not guarantee low positional error or satisfy a continuous dwell condition. Tolerance choices should be tied to an eventual hardware task, then fixed before testing new controllers.')
    page('11 Verification, limitations and next experiments')
    p(f"All {audit['trials']:,} deterministic trials were repeated with two RK4 substeps per control interval, retaining the original 200 Hz sensing and controller update. {audit['changed_outcomes']} outcome labels changed. Numerical agreement of labels at this refinement does not prove global convergence or physical validity. Per-trial metric differences are retained in integration_audit.json; the stochastic subset was not subjected to the full refinement audit.")
    p('Automated checks verify zero-payload trajectory parity, the component-wise inertia identity, a positive mass matrix over selected poses, no mutation of the nominal configuration, input bounds and repeatable seeded trials. A separate NumPy component mass-matrix calculation agrees with the JavaScript dynamics. Six selected loaded ideal trajectories also agree with the existing Python simulator. The 3D UI tests exercise resets, payload geometry, replay isolation and exports with a renderer mock. Browser access was unavailable, so interactive WebGL pixels were not visually verified.')
    p('The model omits wheel slip, roll, yaw, collisions, structural flexibility, motor electrical dynamics and hardware sensor calibration. Payload positions are centred, static and rigid; this study does not estimate an unknown payload, adapt gains, move a load online or compensate an off-axis equilibrium shift. The schematic shell is not a complete CAD mass distribution. Literature already studies unknown-payload estimation and physical wheeled-pendulum validation [2], so adding payload controls alone is not a research novelty claim.')
    p('The next defensible comparison is frozen gains versus a payload-aware redesign, with a declared tuning budget, separate development conditions and held-out mass-height/force combinations. A fixed attachment-point disturbance study would separate the effects of moving COM from the changing force moment. Independent simulator or hardware measurements should then test the chosen claims. Keep these supplements and their frozen datasets separate from the original manuscript until the author reviews the contribution, citations and AI-assistance disclosure.')
    h('Reproduction and files')
    p('From the project root, run node run_payload.js, then node run_payload_audit.js, then publication/.venv/Scripts/python.exe publication/build_payload_report.py. The protocol snapshot records settings and source hashes before execution. results/payload contains per-trial JSON/CSV, deterministic and sensing summaries, selected traces and the integration audit. Seven new figures are exported as vector PDF, SVG and 600 dpi PNG. The report is provided as HTML and PDF, with editable LaTeX source; LaTeX was not compiled here. The original manuscript and Step 3 datasets remain archived baselines.')
    h('References')
    blocks.append(('reference','[1] MIT OpenCourseWare. Classical Mechanics, Fall 2016. Lesson 29.4: Parallel Axis Theorem.','https://ocw.mit.edu/courses/8-01sc-classical-mechanics-fall-2016/resources/29-3/'))
    blocks.append(('reference','[2] D. Baek, Y. Sim, A. Purushottam, S. Gupta and J. Ramos. Toward Control of Wheeled Humanoid Robots with Unknown Payloads: Equilibrium Point Estimation via Real-to-Sim Adaptation. arXiv:2403.10948v2, 2024.','https://arxiv.org/abs/2403.10948v2'))
    setup_fonts();build_pdf(HERE/'output/pdf/payload_report.pdf',title,blocks,manifest,'Step 4 research supplement | 12 November 2026');build_tex(HERE/'output/latex/payload_report.tex',title,blocks,manifest)
    (OUT/'report_content.json').write_text(json.dumps(blocks,indent=2)+'\n')
    pieces=[]
    for b in blocks:
        if b[0]=='heading':pieces.append('<h2>'+html.escape(b[1])+'</h2>')
        elif b[0]=='paragraph':pieces.append('<p>'+html.escape(b[1])+'</p>')
        elif b[0]=='equation':pieces.append('<pre>'+('M = m0 + mp; L = (m0*l0 + mp*h) / M; Ip = mp*(w^2 + b^2) / 12' if 'M=m_0' in b[1] else 'I = I0 + m0*(l0 - L)^2 + Ip + mp*(h - L)^2')+'</pre>')
        elif b[0]=='figure':pieces.append('<figure><img loading="lazy" src="../../publication/figures/'+b[1]+'.svg" alt="'+html.escape(manifest[b[1]]['caption'],quote=True)+'"><figcaption>'+html.escape(manifest[b[1]]['caption'])+'</figcaption></figure>')
        elif b[0]=='table':pieces.append('<div class="table-wrap"><table><caption>'+html.escape(b[1])+'</caption><thead><tr>'+''.join('<th>'+html.escape(str(x))+'</th>' for x in b[2])+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in r)+'</tr>' for r in b[3])+'</tbody></table></div>')
        elif b[0]=='reference':pieces.append('<p>'+html.escape(b[1])+' <a href="'+html.escape(b[2],quote=True)+'">Source</a></p>')
    css='body{margin:0;background:#f3f2ed;color:#18364d;font:16px/1.7 Segoe UI,Arial,sans-serif}main{max-width:1000px;margin:auto;padding:40px 30px}h1{font-size:42px;line-height:1.15}h2{margin-top:40px}a{color:#176779}nav{display:flex;gap:20px;flex-wrap:wrap}figure{margin:30px 0;padding:18px;background:white;border:1px solid #d3dde2;border-radius:12px}img{width:100%;height:auto}figcaption{font-size:13px;line-height:1.6;color:#536474}table{border-collapse:collapse;width:100%;font-size:13px;background:white}th,td{padding:9px;border:1px solid #d3dde2;text-align:left}th{background:#e5ecef}caption{text-align:left;font-weight:600;margin-bottom:8px}.table-wrap,pre{overflow-x:auto}'
    page_html='<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Step 4 | Payload variation</title><style>'+css+'</style></head><body><main><nav><a href="../../visualisation/index.html">Live 3D payload lab</a><a href="../../publication/output/pdf/payload_report.pdf">PDF report</a><a href="results.csv">All trials CSV</a><a href="ideal_summary.csv">Ideal summaries</a><a href="sensing_summary.csv">Sensing summaries</a></nav><p>MICHAEL AGBO / STEP 4</p><h1>'+title+'</h1><p>4,176 trials. Seven figures. Frozen nominal controller gains.</p>'+''.join(pieces)+'</main></body></html>'
    (OUT/'report.html').write_text(page_html,encoding='utf-8');print('Created seven figures, summaries, HTML report, PDF and LaTeX source.')

if __name__=='__main__':main()
