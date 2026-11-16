"""Paired fixed/payload-aware LQR results, with explicit estimate mismatch."""
from collections import Counter,defaultdict
import csv,hashlib,html,json,math,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent;OUT=ROOT/'results/comparison';FIG=HERE/'figures'
sys.path.insert(0,str(HERE))
from build_reports import setup_fonts,build_pdf,build_tex
from build_sensing_report import interval
V=['fixed','exact','mass_low','mass_high','height_low','height_high'];LABELS=['Fixed','Exact load','Mass -20%','Mass +20%','Height -4 cm','Height +4 cm'];COLORS=['#c65d1e','#087b88','#5a6d99','#78863e','#a35c90','#6742a0'];PROFILES=['direct_0ms','kalman_0ms','kalman_10ms'];PNAMES=['Direct, 0 ms','Kalman, 0 ms','Kalman, 10 ms']
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'axes.labelsize':9,'axes.titlesize':10,'legend.fontsize':8,'xtick.labelsize':8,'ytick.labelsize':8,'pdf.fonttype':42,'svg.fonttype':'none','axes.spines.top':False,'axes.spines.right':False,'grid.alpha':.2})
def sub(rows,**kw):return [r for r in rows if all(r[k]==v for k,v in kw.items())]
def pairkey(r):return tuple(r[k] for k in ['phase','mass_kg','height_m','initial_tilt_deg','push_force_n','torque_limit_nm','profile','seed'])
def csvout(name,rows):
    with (OUT/name).open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
def main():
    data=json.loads((OUT/'test.json').read_text());rows=data['records'];assert len(rows)==11016
    for f,digest in data['metadata']['source_sha256'].items():assert hashlib.sha256((ROOT/f).read_bytes()).hexdigest()==digest,f
    audit=json.loads((OUT/'integration_audit.json').read_text());independent=json.loads((OUT/'independent_check.json').read_text());design=json.loads((OUT/'gain_design.json').read_text());protocol=data['metadata']['protocol'];base={pairkey(r):r for r in rows if r['variant']=='fixed'};assert len(base)==1836
    groups=defaultdict(list)
    for r in sub(rows,phase='sensing_test'):groups[(r['mass_kg'],r['height_m'],r['scenario'],r['profile'],r['variant'])].append(r)
    summary=[]
    for (m,h,s,p,v),a in groups.items():
        assert len(a)==30 and len({r['seed'] for r in a})==30;count=Counter(r['outcome'] for r in a);lo,hi=interval(count['fallen'],30)
        summary.append(dict(mass_kg=m,height_m=h,scenario=s,profile=p,variant=v,n=30,recovered=count['recovered'],unsettled=count['unsettled'],fallen=count['fallen'],fall_ci_low=lo,fall_ci_high=hi))
    assert len(summary)==324;csvout('sensing_summary.csv',summary)
    paired=[]
    for phase in ['ideal_test','sensing_test']:
        for v in V[1:]:
            a=sub(rows,phase=phase,variant=v);paired.append(dict(phase=phase,variant=v,n=len(a),fixed_fall_to_upright=sum(base[pairkey(r)]['outcome']=='fallen' and r['outcome']!='fallen' for r in a),fixed_upright_to_fall=sum(base[pairkey(r)]['outcome']!='fallen' and r['outcome']=='fallen' for r in a)))
    csvout('paired_transitions.csv',paired);(OUT/'summary.json').write_text(json.dumps({'sensing':summary,'paired':paired},indent=2)+'\n')
    manifest={}
    def save(fig,name,caption):
        for ext in ['pdf','svg','png']:
            target=FIG/f'{name}.{ext}'
            if '--reuse-figures' not in sys.argv or not target.exists() or ('--refresh-gains' in sys.argv and name=='compare_fig01_gains') or ('--refresh-layout' in sys.argv and name in ['compare_fig04_estimate_errors','compare_fig05_paired_metrics']):fig.savefig(target,dpi=600,bbox_inches='tight',pad_inches=.12)
        plt.close(fig);manifest[name]={'name':name,'caption':caption}
    fig,axs=plt.subplots(1,3,figsize=(7.3,2.8),layout='constrained')
    for h,col in zip(protocol['test_heights_m'],['#087b88','#6742a0','#c65d1e']):
        gains=[design['bank'][f'{m:.4f}/{h:.4f}']['gain'] for m in protocol['test_masses_kg']]
        for j,index in enumerate([1,2,3]):axs[j].plot(protocol['test_masses_kg'],[g[index] for g in gains],'o-',color=col,label=f'h={h:.2f} m')
    for j,index in enumerate([1,2,3]):axs[j].axhline(design['bank']['0.0000/0.1800']['gain'][index],color='#344b5d',ls='--',label='Nominal');axs[j].set(xlabel='Payload mass (kg)',ylabel=['Velocity gain (N s)','Tilt gain (N m/rad)','Angular-rate gain (N m s/rad)'][j]);axs[j].grid()
    axs[0].legend(fontsize=6.5)
    save(fig,'compare_fig01_gains','Gains K in u=-Kx for the reserved test masses and heights, using identical Q and R for every design. Dashed lines are nominal gains. The position gain remains -sqrt(12), to numerical precision, and is omitted. These are local continuous-time designs implemented with 200 Hz sampling and actuator saturation. Connecting lines do not imply interpolation in the implementation.')
    outcomes=['recovered','unsettled','fallen'];fig,axs=plt.subplots(1,2,figsize=(7.2,3.2),layout='constrained')
    matrices={}
    for ax,phase,title in zip(axs,['ideal_test','sensing_test'],['Perfect sensing: 216 pairs','Noisy sensing: 1,620 pairs']):
        matrix=np.zeros((3,3),int)
        for r in sub(rows,phase=phase,variant='exact'):matrix[outcomes.index(base[pairkey(r)]['outcome']),outcomes.index(r['outcome'])]+=1
        matrices[phase]=matrix.tolist();ax.imshow(matrix,cmap='Blues')
        for y in range(3):
            for x in range(3):ax.text(x,y,str(matrix[y,x]),ha='center',va='center',color='white' if matrix[y,x]>matrix.max()*.5 else '#18364d')
        ax.set(xticks=range(3),xticklabels=['Recovered','Unsettled','Fallen'],yticks=range(3),yticklabels=['Recovered','Unsettled','Fallen'],xlabel='Payload-aware, exact load',ylabel='Fixed nominal LQR',title=title)
    save(fig,'compare_fig02_paired_outcomes','Paired outcomes for exact-load redesign versus fixed nominal LQR. Every pair shares the true load, initial state, force, torque limit, sensing profile and random seed. Counts are descriptive inventories. In the noisy subset 186 fixed-controller falls become upright trials, while 12 fixed-controller survivors fall after redesign. No noisy run meets the strict recovery dwell criterion. The ideal comparison has identical outcome labels for every pair.')
    fig,axs=plt.subplots(2,3,figsize=(7.4,5.2),layout='constrained')
    for i,scenario in enumerate(['opposing_push','restricted_torque']):
        for j,profile in enumerate(PROFILES):
            ax=axs[i,j];a=np.zeros((3,3));labels={}
            for y,h in enumerate(protocol['test_heights_m']):
                for x,m in enumerate(protocol['test_masses_kg']):
                    f=sub(summary,mass_kg=m,height_m=h,scenario=scenario,profile=profile,variant='fixed')[0]['fallen'];e=sub(summary,mass_kg=m,height_m=h,scenario=scenario,profile=profile,variant='exact')[0]['fallen'];a[y,x]=(e-f)/30;labels[y,x]=f'{f}/{e}'
            im=ax.imshow(a,origin='lower',vmin=-1,vmax=1,cmap='BrBG_r',aspect='auto')
            for (y,x),label in labels.items():ax.text(x,y,label,ha='center',va='center',fontsize=8,color='white' if abs(a[y,x])>.65 else '#14283b')
            ax.set(xticks=range(3),xticklabels=protocol['test_masses_kg'],yticks=range(3),yticklabels=protocol['test_heights_m'],xlabel='True mass (kg)',ylabel='True height (m)',title=('Opposing push' if i==0 else 'Restricted torque')+'\n'+PNAMES[j])
    fig.colorbar(im,ax=axs,shrink=.8,label='Change in fall fraction: aware - fixed')
    save(fig,'compare_fig03_condition_falls','Each cell prints fixed/aware fall counts out of 30 paired seeds; colour is (aware-fixed)/30, so negative values favour redesign. Separate panels retain each scenario and sensing profile. Axes are discrete test levels, not a continuous stability map. Per-controller Wilson intervals are in sensing_summary.csv; this colour scale is not a confidence interval for a paired difference.')
    fig,axs=plt.subplots(1,2,figsize=(7.2,3.7),layout='constrained')
    for ax,phase,title in zip(axs,['ideal_test','sensing_test'],['Perfect sensing','Noisy sensing']):
        a=[sub(paired,phase=phase,variant=v)[0] for v in V[1:]];x=np.arange(5);positive=[r['fixed_fall_to_upright'] for r in a];negative=[-r['fixed_upright_to_fall'] for r in a]
        ax.barh(x,positive,color='#18846e',label='Fixed fall becomes upright');ax.barh(x,negative,color='#c65d1e',label='Fixed survivor falls');ax.axvline(0,color='#243e51',lw=.8)
        for y,(pos,neg) in enumerate(zip(positive,negative)):ax.text(pos+(.15 if phase=='ideal_test' else 3),y,str(pos),va='center',fontsize=8);ax.text(neg-(.15 if phase=='ideal_test' else 3),y,str(-neg),va='center',ha='right',fontsize=8)
        ax.set(yticks=x,yticklabels=LABELS[1:],xlabel='Number of paired transitions',title=title);ax.invert_yaxis();ax.margins(x=.2)
    fig.legend(*axs[1].get_legend_handles_labels(),fontsize=7,loc='outside upper center',ncol=2)
    save(fig,'compare_fig04_estimate_errors','Paired fall-to-upright improvements and upright-to-fall regressions relative to fixed nominal LQR, separately for 216 ideal pairs and 1,620 noisy pairs per variant. Four mismatch designs perturb one assumed parameter at a time. Heights refer to axle-to-payload COM distance. Counts across conditions are not operational probabilities. The different panel scales reflect their different inventories. A beneficial bias on this grid is not a recommendation to misestimate real payloads.')
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.0),layout='constrained');metrics=['tail_tilt_rms_deg','tail_position_rms_m','torque_squared_integral','saturation_s'];ylabs=['Change in tilt RMS (deg)','Change in position RMS (m)','Change in squared torque\n((N m)$^2$s)','Change in saturation fraction'];metric_rows=[]
    for ax,metric,label in zip(axs.flat,metrics,ylabs):
        for j,profile in enumerate(['ideal',*PROFILES]):
            a=[r for r in rows if r['variant']=='exact' and r['profile']==profile and r['outcome']!='fallen' and base[pairkey(r)]['outcome']!='fallen'];values=[(r[metric]-base[pairkey(r)][metric])/(10 if metric=='saturation_s' else 1) for r in a];q=np.quantile(values,[.25,.5,.75]);ax.errorbar(j,q[1],yerr=[[q[1]-q[0]],[q[2]-q[1]]],fmt='o',color='#087b88',capsize=3);metric_rows.append(dict(metric=metric,profile=profile,paired_survivors=len(a),q25=float(q[0]),median=float(q[1]),q75=float(q[2])));ax.text(j,.97,f'n={len(a)}',transform=ax.get_xaxis_transform(),ha='center',va='top',fontsize=7)
        ax.axhline(0,color='#637587',ls='--',lw=.8);ax.set(xticks=range(4),xticklabels=['Ideal','Direct\n0 ms','Kalman\n0 ms','Kalman\n10 ms'],ylabel=label,xlim=(-.35,3.35));ax.grid(axis='y');ax.margins(y=.22)
    csvout('paired_metrics.csv',metric_rows)
    save(fig,'compare_fig05_paired_metrics','Exact-load redesign minus fixed-gain metric differences, restricted to pairs in which both robots survive the full 10 s. Each point is the median paired difference and each bar the interquartile range, not a confidence interval. n is the number of survivor pairs, pooled descriptively over the listed test conditions; different columns select different survivor populations. Negative values mean a lower metric. Saturation fraction is saturated time / 10 s. This conditional view must be read alongside the fall counts; it does not measure unconditional performance.')
    fig,axs=plt.subplots(3,1,figsize=(7.2,5.6),layout='constrained')
    for variant,col in zip(['fixed','exact'],COLORS[:2]):
        a=data['examples'][variant]['rows'];t=[r['time'] for r in a]
        for ax,values in zip(axs,[[r['state'][2]*180/math.pi for r in a],[r['state'][0] for r in a],[r['torque'] for r in a]]):ax.plot(t,values,color=col,label='Fixed nominal' if variant=='fixed' else 'Payload-aware')
    for ax,label in zip(axs,['Tilt (deg)','Position (m)','Previous-interval torque (N m)']):ax.set(xlabel='Time (s)',ylabel=label);ax.axvspan(2,2.2,color='#e0b050',alpha=.2);ax.grid()
    axs[0].legend()
    save(fig,'compare_fig06_example','Protocol-selected ideal example: true payload 0.65 kg at 0.26 m, initial tilt 12 deg, force -15 N from 2.00 to 2.20 s, and torque limit 0.6 N m. Both controllers recover. Torque stored at each endpoint is the command held during the preceding interval; its initial sample is zero. This example was specified before test execution and is not a best-case selection.')
    (HERE/'comparison_figure_manifest.json').write_text(json.dumps(list(manifest.values()),indent=2)+'\n')
    b=[]
    def h(t):b.append(('heading',t))
    def p(t):b.append(('paragraph',t))
    def page(t,figure=None):b.append(('pagebreak',));h(t);b.append(('figure',figure)) if figure else None
    title='Payload-aware LQR under sensing uncertainty and load-estimate error'
    h('Scope and main result')
    p('This study compares the original fixed nominal LQR with a payload-conditioned LQR using exactly the same state and control-effort weights. Gains are designed from an assumed load before each run and remain fixed during the run. No payload estimator or online adaptation is implemented. Six variants are tested on reserved payload/force conditions: fixed gains, exact load, mass underestimated or overestimated by 20%, and height underestimated or overestimated by 4 cm. There are 16 development checks and 11,016 test trials; development results are excluded from the test summaries.')
    inventory=[]
    for v,label in zip(V,LABELS):
        a=Counter(r['outcome'] for r in sub(rows,phase='ideal_test',variant=v));c=Counter(r['outcome'] for r in sub(rows,phase='sensing_test',variant=v));inventory.append([label,a['recovered'],a['fallen'],c['unsettled'],c['fallen']])
    b.append(('table','Per-variant test inventory: 216 ideal and 1,620 noisy runs',['Design','Ideal recovered','Ideal fallen','Noisy unsettled','Noisy fallen'],inventory,[.28,.18,.18,.18,.18]))
    p('Exact-load redesign leaves every ideal-test outcome unchanged: 166 recoveries and 50 falls. In noisy tests, falls decrease from 508 to 334. Paired inspection reveals 186 prevented falls and 12 introduced falls, giving a net reduction of 174 on this finite grid. None of the noisy runs satisfies the original strict recovery dwell. These are simulation inventories, not estimates of deployment success or proof that payload-aware control is uniformly safer.')
    p('Assumption error is not uniformly harmful on this grid. Overestimating height gives fewer falls than exact-load design, while underestimating height introduces more regressions. The original Q/R objective, saturation and sensing imperfections can make a physically exact design suboptimal for the chosen nonlinear finite-horizon outcomes. This post-result observation is not used to retune the design or recommend a favourable bias.')
    page('1 Controller construction and comparison contract')
    p('The rigid payload model, motor reaction torque, COM-force convention and sensing model are inherited from Steps 3 and 4. True payload parameters always determine plant dynamics. Only the controller gain uses the assumed mass and height. For each assumed payload, the plant is linearised analytically at the upright zero-velocity equilibrium. The continuous algebraic Riccati equation is solved with SciPy [1], retaining Q=diag(12,2,120,2) and R=1. The applied torque is u=clip(-K*xhat, limit), sampled and held at 200 Hz. The true loaded COM is the force application point, irrespective of estimation error.')
    b.append(('equation',r'A^T P+PA-PBR^{-1}B^TP+Q=0,\qquad K=R^{-1}B^TP'))
    p('The fixed controller retains the original nominal gain. Zero-payload redesign returns that exact stored gain after a numerical agreement check. The browser uses precomputed CARE solutions at supported settings with exact lookup, not interpolation. The study also includes the exact mismatch settings. All 949 matched designs satisfy the tested CARE residual tolerance and have stable unsaturated, perfect-state sampled linear dynamics. This is not a nonlinear, saturated or delayed-system stability guarantee.')
    h('2 Development and reserved test protocol')
    p('The tuning budget is zero performance-based parameter searches. Q/R and the Step 3 filter covariance are frozen before execution. Sixteen development runs use the four previously studied loads and the original nominal/large-push conditions; they check implementation only. The new test masses are 0.35, 0.65 and 1.15 kg; heights are 0.14, 0.26 and 0.36 m. None of these mass-height combinations was evaluated in the prior payload grid. These are reserved model conditions, not new hardware or out-of-model physics. The protocol is locally recorded before execution, not externally preregistered.')
    p('The ideal test crosses nine loads with initial tilts -12, 0 and 12 deg, forces -15, -7, 7 and 15 N, and torque limits 0.3 and 0.6 N m: 216 conditions per variant and 1,296 total trials. Both force directions are included, with aligned and opposing initial tilts. Mismatch variants change one assumed parameter at a time; true payloads and every disturbance stay paired.')
    b.append(('table','Noisy test scenarios, each at all nine loads',['Scenario','Initial tilt (deg)','Force (N)','Limit (N m)'],[['Opposing push',7,-12,.6],['Restricted torque',-7,12,.3]],[.31,.23,.23,.23]))
    p('Three sensing profiles use angle-noise SD 0.75 deg and gyro-noise SD 3 deg/s: direct observations at zero delay, Kalman estimates at zero delay, and Kalman estimates at 10 ms delay. Thirty seeds per condition use 5003+104729*i, i=0..29, distinct from prior studies and paired across variants. Nine loads x two scenarios x three profiles x 30 seeds gives 1,620 runs per variant and 9,720 noisy trials. Each run lasts at most 10 s, with a 0.2 s force pulse starting at 2 s. The filter does not predict delayed packets to the present.')
    page('3 Outcomes, pairing and statistical scope')
    p('The primary outcome is a fall when absolute true tilt reaches 45 deg at a control endpoint. Recovery retains the final continuous one-second dwell within 1 deg tilt, 0.03 m position and 0.03 m/s speed. Upright survivors that miss it are unsettled. Final-second RMS uses all 201 endpoint samples from 9 to 10 s. Torque squared integral is a control-effort measure, not electrical energy. Saturation fraction is saturated control time divided by 10 s for full-duration runs.')
    p('Paired outcomes match load, disturbance, torque limit, observer and seed. Continuous comparisons use only pairs where both variants survive for 10 s; this is an explicitly conditional population. Medians and IQR describe the finite paired grid, not confidence in an operational mean. Per-condition fall fractions have Wilson 95% intervals for each variant; no unpaired significance test or pooled deployment probability is reported. Conditions are not independent random draws from a real-world operating distribution, and shared seeds induce dependence across variants.')
    h('4 Gain changes at the reserved loads');b.append(('figure','compare_fig01_gains'))
    p('The redesign changes velocity, tilt and angular-rate gains while retaining the state and torque cost weights. It is a model-based baseline, not a tuned robust controller. The experiment tests whether these gain changes help under the existing saturation and sensing conditions; an affirmative answer in some cells does not establish optimality.')
    page('5 Exact-load design: paired outcomes','compare_fig02_paired_outcomes')
    p('The perfect-sensing subset shows no change in recovery/fall labels, although continuous responses can differ. In noisy tests, the net benefit hides regressions. The complete trial CSV and paired transition file preserve both directions of change. Persistent sensor noise prevents every noisy trial from meeting the strict recovery dwell, so staying upright cannot be reported as recovery.')
    page('6 Where the fall counts change','compare_fig03_condition_falls')
    p('These maps retain the load geometry, sensing profile and disturbance scenario. A lower overall fall count need not imply improvement in every cell. The 30 seeds per cell give limited resolution for rare failures; zero falls still has a Wilson upper bound of approximately 11.35%. The colour represents an observed paired-grid difference, with no claim that every coloured cell is statistically significant.')
    page('7 Sensitivity to incorrect payload estimates','compare_fig04_estimate_errors')
    p('The errors are deliberately structured assumptions, not samples from a calibrated payload estimator. Mass and height errors are tested separately; joint errors, drift, unknown payload shape and off-axis loads are not covered. Gains use the erroneous model while physics retains the true model. The favourable aggregate result for overestimated height is a hypothesis for a future controller-design study, not a parameter selected here using test outcomes.')
    page('8 Residual motion, control effort and saturation','compare_fig05_paired_metrics')
    p('These differences compare the same surviving condition and noise seed. Selecting only mutual survivors excludes falls and can change which conditions contribute in each profile. Consequently the fall maps and this conditional metric figure answer complementary questions. The broad ranges across pooled conditions should not be read as uncertainty bounds for a universal controller effect.')
    page('9 Preselected ideal trajectory','compare_fig06_example')
    p('Both designs recover in this example. The curves show how a redesign may alter displacement, tilt and actuator use without changing a discrete outcome. This condition was specified before the test sweep, preventing selection of a visually favourable trajectory after inspecting the results.')
    page('10 Verification and practical limits')
    p(f"All {audit['trials']:,} ideal test records were repeated with two RK4 substeps per control interval while preserving 200 Hz feedback and sensing. {audit['changed_outcomes']} outcome labels changed. The audit is a numerical refinement check, not a physical validation or a full stochastic convergence study.")
    maximum=max(r['max_absolute_state_component_difference'] for r in independent['checks'])
    p(f"Six preselected ideal trajectories were independently integrated using a component-wise mass matrix and SciPy DOP853, with torque held over each 5 ms interval (rtol=1e-11, atol=1e-13). All selected outcome labels agree. The largest absolute component difference is {maximum:.3g}, comparing each state component in its SI unit (metres, metres/second, radians or radians/second). This mixed-unit maximum is a numerical discrepancy diagnostic, not a physically meaningful norm. The analytical linearisation also agrees with finite differences of the archived nonlinear model.")
    p('Automated checks cover original fixed-controller parity, zero-load equivalence, seeded repeatability, estimate settings, gain lookup and the live comparison controls. UI tests use real Three.js geometry with a mocked renderer. Browser access remains unavailable, so live WebGL pixels were not visually verified. PDF pages and static figures are checked separately. The compact robot-view layout is retained, and changing gains or assumed payload resets a live run.')
    p('No independent multibody simulator or hardware validation has been completed. The separate component/solver check reduces implementation risk but shares the same modelling assumptions. Real wheel traction, motor dynamics, calibration error, load attachment compliance, roll, yaw and online pickup remain outside scope. Estimated parameters are supplied to the controller; the system does not learn them. The 54 true-load/variant sampled linear diagnostics have spectral radii between '+f"{min(r['sampled_radius'] for r in design['test_linear_diagnostics']):.6f} and {max(r['sampled_radius'] for r in design['test_linear_diagnostics']):.6f}"+', all below one, but that local perfect-state result does not cover delayed observation or saturation.')
    p('Payload-sensitive gain scheduling and observer-based compensation already exist in the literature; Song et al. [2], for example, study a more complex variable-configuration robot with a gain-scheduled robust controller and equilibrium-offset observer in simulation. The present work does not reproduce that method. A publishable contribution needs a precise new claim and external validation, not simply adding another LQR. The present evidence supports a narrower statement: on this reserved simulation grid, fixed-cost payload redesign improves aggregate noisy fall counts while preserving identifiable regressions and sensitivity to parameter assumptions.')
    h('Reproduction and next experiment')
    p('Run publication/.venv/Scripts/python.exe design_payload_lqr.py; node run_comparison.js --development; node run_comparison.js --test; node audit_comparison.js; publication/.venv/Scripts/python.exe validate_comparison.py; then publication/.venv/Scripts/python.exe publication/build_comparison_report.py. Data and source hashes are in results/comparison. The report has six vector PDF/SVG and 600 dpi PNG figures, a PDF and HTML version, and editable LaTeX (not compiled here). Prior manuscript and Steps 3/4 datasets remain unchanged.')
    p('For a follow-up, specify a redesigned cost or delay-aware controller using a separate development set, then create a fresh test set: the present test outcomes are now known. Use a fixed physical force attachment point and an independent multibody or hardware implementation before extending these conclusions to a physical robot. Do not retrospectively promote the favourable height bias into a validated design.')
    h('References')
    b.append(('reference','[1] SciPy Developers. scipy.linalg.solve_continuous_are. SciPy 1.17 documentation.','https://docs.scipy.org/doc/scipy-1.17.0/reference/generated/scipy.linalg.solve_continuous_are.html'))
    b.append(('reference','[2] Y. Song, G. Wan and X. Cao. Gain-Scheduled Control of a Wheeled Inverted-Pendulum Robot with Load-Induced Equilibrium Drift Compensation. Applied Sciences 16(10), 4876 (2026). DOI 10.3390/app16104876.','https://www.mdpi.com/2076-3417/16/10/4876'))
    setup_fonts();build_pdf(HERE/'output/pdf/comparison_report.pdf',title,b,manifest,'Controller comparison supplement | 12 November 2026');build_tex(HERE/'output/latex/comparison_report.tex',title,b,manifest);(OUT/'report_content.json').write_text(json.dumps(b,indent=2)+'\n')
    pieces=[]
    for block in b:
        kind=block[0]
        if kind=='heading':pieces.append('<h2>'+html.escape(block[1])+'</h2>')
        elif kind=='paragraph':pieces.append('<p>'+html.escape(block[1])+'</p>')
        elif kind=='equation':pieces.append('<pre>A^T P + P A - P B R^-1 B^T P + Q = 0; K = R^-1 B^T P</pre>')
        elif kind=='figure':pieces.append('<figure><img loading="lazy" src="../../publication/figures/'+block[1]+'.svg" alt="'+html.escape(manifest[block[1]]['caption'],quote=True)+'"><figcaption>'+html.escape(manifest[block[1]]['caption'])+'</figcaption></figure>')
        elif kind=='table':pieces.append('<div class="table-wrap"><table><caption>'+html.escape(block[1])+'</caption><thead><tr>'+''.join('<th>'+html.escape(str(x))+'</th>' for x in block[2])+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in r)+'</tr>' for r in block[3])+'</tbody></table></div>')
        elif kind=='reference':pieces.append('<p>'+html.escape(block[1])+' <a href="'+block[2]+'">Source</a></p>')
    source=(ROOT/'results/payload/report.html').read_text(encoding='utf-8');css=source.split('<style>')[1].split('</style>')[0]
    page_html='<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Payload-aware LQR comparison</title><style>'+css+'</style></head><body><main><nav><a href="../../visualisation/index.html">Live 3D comparison</a><a href="../../publication/output/pdf/comparison_report.pdf">PDF report</a><a href="test.csv">All test records</a><a href="sensing_summary.csv">Condition summaries</a><a href="paired_metrics.csv">Paired metrics</a></nav><p>MICHAEL AGBO / CONTROLLER COMPARISON</p><h1>'+title+'</h1><p>16 development checks. 11,016 reserved test trials. Six figures.</p>'+''.join(pieces)+'</main></body></html>';(OUT/'report.html').write_text(page_html,encoding='utf-8');print('Created comparison summaries, six figures, HTML/PDF report and editable LaTeX.')
if __name__=='__main__':main()
