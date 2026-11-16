"""Summarise the frozen sensing sweep, plot its data, and write a separate supplement."""
from collections import Counter,defaultdict
import csv,hashlib,html,json,math,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize

HERE=Path(__file__).resolve().parent;ROOT=HERE.parent;OUT=ROOT/'results/sensing';FIG=HERE/'figures'
sys.path.insert(0,str(HERE))
from build_reports import setup_fonts,build_pdf,build_tex

DIRECT='#c65d1e';KALMAN='#6742a0';TRUTH='#18364d'
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'axes.labelsize':9,'axes.titlesize':10,'legend.fontsize':8,'xtick.labelsize':8,'ytick.labelsize':8,'pdf.fonttype':42,'svg.fonttype':'none','axes.spines.top':False,'axes.spines.right':False,'grid.alpha':.2})

def interval(k,n):
    z=1.959963984540054;p=k/n;denom=1+z*z/n;centre=(p+z*z/(2*n))/denom;half=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/denom
    return max(0,centre-half),min(1,centre+half)

def subset(records,**kwargs):return [r for r in records if all(r[k]==v for k,v in kwargs.items())]

def main():
    data=json.loads((OUT/'results.json').read_text());records=data['records'];protocol=data['metadata']['protocol'];assert len(records)==8340
    for name,digest in data['metadata']['source_sha256'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,(name,'source changed since study')
    grouped=defaultdict(list)
    for r in records:grouped[(r['scenario'],r['controller'],r['observer'],r['angleNoiseDeg'],r['gyroNoiseDegS'],r['delayMs'])].append(r)
    summary=[]
    for (scenario,controller,observer,angle,gyro,delay),rows in grouped.items():
        expected=30 if angle or gyro else 1;assert len(rows)==expected and len({r['seed'] for r in rows})==expected
        c=Counter(r['outcome'] for r in rows);lo,hi=interval(c['fallen'],expected) if expected>1 else (None,None)
        summary.append(dict(scenario=scenario,controller=controller,observer=observer,angle_sigma_deg=angle,gyro_sigma_deg_s=gyro,delay_ms=delay,n=expected,recovered=c['recovered'],unsettled=c['unsettled'],fallen=c['fallen'],fall_ci_low=lo,fall_ci_high=hi,median_current_rmse_deg=float(np.median([r['estimate_current_rmse_deg'] for r in rows])),median_source_rmse_deg=float(np.median([r['estimate_source_rmse_deg'] for r in rows])),median_duration_s=float(np.median([r['duration_s'] for r in rows]))))
    assert len(summary)==336
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    with (OUT/'summary.csv').open('w',newline='') as file:w=csv.DictWriter(file,fieldnames=summary[0].keys());w.writeheader();w.writerows(summary)
    manifest={}
    def save(fig,name,caption):
        for ext in ('pdf','svg','png'):fig.savefig(FIG/f'{name}.{ext}',dpi=600,bbox_inches='tight',pad_inches=.1)
        plt.close(fig);manifest[name]={'name':name,'caption':caption}
    # Independent noise channels; uncertainty intervals apply to repeated seeds only.
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.4),layout='constrained')
    for row,controller in enumerate(('pid','lqr')):
        for col,(family,key,label) in enumerate([('angle_only','angleNoiseDeg','Angle noise SD (deg)'),('gyro_only','gyroNoiseDegS','Gyro noise SD (deg/s)')]):
            ax=axs[row,col]
            for mode,color,style in [('direct',DIRECT,'-'),('kalman',KALMAN,'--')]:
                source=[r for r in records if r['scenario']=='nominal' and r['controller']==controller and r['observer']==mode and family in r['families']];xs=sorted({r[key] for r in source});ys=[]
                for x in xs:
                    sample=[r for r in source if r[key]==x];n=len(sample);k=sum(r['outcome']=='recovered' for r in sample);ys.append(k/n)
                    if n>1:
                        lo,hi=interval(k,n);ax.vlines(x,lo,hi,color=color,alpha=.6,lw=1.4)
                ax.plot(xs,ys,marker='o',ms=4,color=color,ls=style,label='Direct' if mode=='direct' else 'Kalman')
            ax.set_title(controller.upper());ax.set_xlabel(label);ax.set_ylabel('Recovered fraction');ax.set_ylim(-.04,1.04);ax.set_yticks([0,.25,.5,.75,1]);ax.grid()
    axs[0,0].legend(loc='lower left',frameon=False)
    save(fig,'step3_fig01_independent_noise','Independent angle-noise and gyro-noise sweeps in the nominal condition (5 deg initial tilt, 5 N push, 0.7 N m limit), with zero packet delay and the other noise channel set to zero. Each noisy point uses 30 paired seeds. Vertical lines are 95% Wilson intervals for the recovered fraction at that fixed condition. The zero-noise point is a single deterministic evaluation with no uncertainty interval. Lines connect evaluated levels only.')
    # A clearly identified stress condition, not an aggregate across conditions.
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.7),layout='constrained');levels=protocol['combined_angle_sigma_deg'];delays=protocol['delay_ms']
    for row,controller in enumerate(('pid','lqr')):
        for col,mode in enumerate(('direct','kalman')):
            ax=axs[row,col];values=np.zeros((len(delays),len(levels)))
            for iy,delay in enumerate(delays):
                for ix,level in enumerate(levels):
                    rows=subset(records,scenario='large_push',controller=controller,observer=mode,angleNoiseDeg=level,gyroNoiseDegS=level*4,delayMs=delay);n=len(rows);fallen=sum(r['fallen'] for r in rows);values[iy,ix]=fallen/n
                    ax.text(ix,iy,f'{fallen}/{n}',ha='center',va='center',fontsize=9,color='white' if fallen/n>.55 else '#102839')
            im=ax.imshow(values,origin='lower',cmap='magma_r',vmin=0,vmax=1,aspect='auto');ax.set_xticks(range(len(levels)),levels);ax.set_yticks(range(len(delays)),delays);ax.set_title(f'{controller.upper()} | {mode.capitalize()}');ax.set_xlabel('Angle noise SD (deg)');ax.set_ylabel('Delay (ms; discrete levels)')
    fig.colorbar(im,ax=axs.ravel().tolist(),label='Fallen / evaluated',shrink=.85)
    save(fig,'step3_fig02_noise_delay_falls','Noise-delay outcome map for the large-push condition (0 deg initial tilt, 20 N pulse, 0.7 N m limit). Gyro SD is four times angle SD numerically, in deg/s. Each cell gives falls/evaluations: n=30 for noisy cells and n=1 for deterministic zero-noise cells. Rows are discrete delay levels, not equally spaced numeric time intervals. Shading is the evaluated fraction; it is not a fitted stability boundary. Full results for all three scenarios are supplied in summary.csv.')
    fig,axs=plt.subplots(2,2,figsize=(7.2,5.7),sharex=True,layout='constrained')
    for col,controller in enumerate(('pid','lqr')):
        rows=data['examples'][controller+'_kalman']['rows'];t=np.array([r['time'] for r in rows]);mask=(t>=1.5)&(t<=3.5)
        for row,index in enumerate((2,3)):
            ax=axs[row,col]
            for key,color,style,width,label in [('measurement','#b4bdc5','-',.55,'Measured'),('state',TRUTH,'-',1.5,'True'),('estimate',KALMAN,'--',1.2,'Estimated')]:
                values=np.rad2deg([r[key][index] for r in rows]);ax.plot(t[mask],values[mask],color=color,ls=style,lw=width,label=label)
            ax.axvspan(2,2.2,color='#deb566',alpha=.18);ax.grid();ax.set_ylabel('Tilt (deg)' if index==2 else 'Angular rate (deg/s)')
        axs[0,col].set_title(controller.upper());axs[1,col].set_xlabel('Simulation time (s)')
    axs[0,0].legend(frameon=False,fontsize=8)
    save(fig,'step3_fig03_true_measured_estimated','Protocol-selected example: nominal condition, angle SD 1 deg, gyro SD 4 deg/s, zero delay and seed 1001. Both controllers use Kalman estimates. The shaded interval is the 5 N pulse. A fixed 1.5-3.5 s window shows the transient and sensor noise; the complete 10 s traces are retained in results.json. Angular-rate estimates remain close to noisy gyro readings with the chosen process covariance. This is an illustrative trace, not independent statistical replication.')
    fig,axs=plt.subplots(1,2,figsize=(7.2,3.7),layout='constrained')
    for ax,controller in zip(axs,('pid','lqr')):
        for mode,color in [('direct',DIRECT),('kalman',KALMAN)]:
            for metric,style,suffix in [('estimate_current_rmse_deg','-','current truth'),('estimate_source_rmse_deg','--','packet-time truth')]:
                med=[];low=[];high=[]
                for delay in delays:
                    rows=subset(records,scenario='nominal',controller=controller,observer=mode,angleNoiseDeg=1,gyroNoiseDegS=4,delayMs=delay);assert len(rows)==30 and all(r['duration_s']==10 for r in rows)
                    values=[r[metric] for r in rows];med.append(np.median(values));low.append(np.quantile(values,.25));high.append(np.quantile(values,.75))
                ax.plot(delays,med,marker='o',ms=3,color=color,ls=style,label=f'{mode.capitalize()}, {suffix}')
                if style=='-':ax.fill_between(delays,low,high,color=color,alpha=.12)
        ax.set_yscale('log');ax.set_title(controller.upper());ax.set_xlabel('Measurement delay (ms)');ax.set_ylabel('Tilt estimation RMSE (deg)');ax.grid()
    axs[0].legend(frameon=False,fontsize=6.7,loc='upper left')
    save(fig,'step3_fig04_delay_estimation_error','Nominal condition, angle SD 1 deg and gyro SD 4 deg/s; all 30 seeds in every plotted group run for the full 10 s. Solid curves compare estimates with current true tilt; dashed curves compare with truth at the delivered packet timestamp. Points are medians, and shading is the interquartile range for current-state error, not a confidence interval. The log scale separates denoising from the remaining delay mismatch. No prediction to the present is applied.')
    fig,axs=plt.subplots(1,2,figsize=(7.2,3.5),layout='constrained')
    for ax,controller in zip(axs,('pid','lqr')):
        for mode,color,style in [('direct',DIRECT,'-'),('kalman',KALMAN,'--')]:
            med=[];lo=[];hi=[]
            for level in levels:
                rows=subset(records,scenario='nominal',controller=controller,observer=mode,angleNoiseDeg=level,gyroNoiseDegS=4*level,delayMs=0);assert all(r['duration_s']==10 for r in rows);vals=[r['torque_squared_integral'] for r in rows];med.append(np.median(vals));lo.append(np.quantile(vals,.25));hi.append(np.quantile(vals,.75))
            ax.plot(levels,med,color=color,ls=style,marker='o',label=mode.capitalize());ax.fill_between(levels,lo,hi,color=color,alpha=.15)
        ax.set_xlabel('Angle noise SD (deg); gyro SD = 4 x level');ax.set_ylabel(r'Squared torque integral ($(N\,m)^2 s$)');ax.set_title(controller.upper());ax.grid();ax.legend(frameon=False)
    save(fig,'step3_fig05_control_effort','Nominal condition with zero delay and paired angle/gyro noise levels. Curves show median integrated squared total motor torque over the full 10 s; shading is the interquartile range across 30 seeds at each noisy level. Zero noise is one deterministic trial. All runs shown last 10 s. This is a control-effort measure, not electrical energy or hardware battery consumption.')
    (HERE/'sensing_figure_manifest.json').write_text(json.dumps(list(manifest.values()),indent=2)+'\n')
    blocks=[]
    def h(t):blocks.append(('heading',t))
    def p(t):blocks.append(('paragraph',t))
    def f(n):blocks.append(('figure',n))
    def table(title,headers,rows,widths):blocks.append(('table',title,headers,rows,widths))
    title='Noisy sensing and measurement delay in a balancing robot'
    h('Scope and main findings')
    p('This Step 3 supplement extends the original planar balancing-robot study with noisy angle and gyro measurements, delayed observation packets, and a causal two-state Kalman estimator. It reports 8,340 unique closed-loop trials and six perfect-observation baselines. The original physics, PID-type gains, LQR gains, torque limits and recovery definition remain fixed. It is a within-controller sensing experiment, not a matched-tuning comparison of controller classes.')
    p('The central distinction is between estimating a past measurement accurately and knowing the current robot state. Filtering reduces tilt noise in the tested nominal setting, but does not compensate for packet delay. Persistent measurement noise can also prevent the original strict final-window recovery criterion from being met even when the robot remains upright. Outcomes therefore separate recovery, upright-but-unsettled behaviour, and falls.')
    rows=[]
    for controller in ('pid','lqr'):
        for mode in ('direct','kalman'):
            for delay in (0,40):
                ss=subset(records,scenario='nominal',controller=controller,observer=mode,angleNoiseDeg=1,gyroNoiseDegS=4,delayMs=delay)
                rows.append([controller.upper(),mode,str(delay),f"{np.median([r['estimate_current_rmse_deg'] for r in ss]):.3f}",f"{np.median([r['estimate_source_rmse_deg'] for r in ss]):.3f}",str(sum(r['fallen'] for r in ss))+'/30'])
    table('Nominal condition: angle SD 1 deg, gyro SD 4 deg/s; all trials last 10 s',['Control','Input','Delay ms','Current RMSE deg','Packet-time RMSE deg','Falls'],rows,[.14,.15,.13,.20,.24,.14])
    p('Every trial in that table remains upright but is labelled unsettled: none achieves the final one-second dwell within all original bounds. The roughly sevenfold reduction in zero-delay median tilt RMSE therefore does not by itself establish task recovery. At 40 ms, the Kalman estimate still tracks the old packet-time tilt closely while its mismatch with current tilt is several degrees.')
    h('1 Observation model and causality')
    p('At each 5 ms sample, independent zero-mean Gaussian noise is added to true tilt and true angular rate. These are explicit angle and gyro surrogates. They do not model accelerometer-specific force, sensor bias, drift, clipping, quantisation or dropout. Position and translational speed are ideal encoder-like channels with no additive noise. The entire four-component packet receives the same configured delay.')
    p('With D delay steps, the packet index delivered at step k is max(0,k-D). A noisy packet acquired at t=0 is available for initialisation and is held while the pipeline fills; no negative-time trajectory or future observation is invented. Each newly delivered packet updates the filter once. Repeated startup packets are held without repeated correction or time propagation. The controller receives only the delivered vector or its filtered counterpart. Ground truth is used separately for plant dynamics, displays and outcome evaluation.')
    h('2 Kalman estimator')
    p('The estimator state is tilt and angular rate. Its transition assumes constant angular rate between measurement timestamps. F has rows [1,dt] and [0,1]; H is the identity. A per-interval random angular acceleration with standard deviation 50 rad/s² gives Q = sigma_a² G G-transpose, with G = [dt²/2, dt]-transpose. This engineering setting was fixed before the sweep and was not optimised on its results. R is diagonal, using the configured angle and gyro noise variances in radians. The first estimate equals the first measured angle/rate, with initial covariance R.')
    p('Prediction is followed by scalar angle and gyro corrections; independent measurement channels make this equivalent to a joint linear Kalman update. Zero noise in both channels returns the measurements exactly. Ideal position and velocity packet values are combined with the estimated attitude to form the four-component control input. No motor model, external-force input or truth-fed correction enters the estimator. It does not extrapolate from a delayed packet to the current control time.')
    p('The recursive prediction/correction structure follows the standard linear Kalman formulation [1]. The robot is nonlinear and its real angular acceleration is not the assumed white random process, so the filter does not carry a general optimality or stability guarantee here. Delay-aware balancing controllers are an established research area [2]; this supplement measures an uncompensated-delay baseline rather than claiming a new delay-compensation method.')
    h('3 Frozen experimental protocol')
    table('Three physical conditions, each with a 0.2 s pulse at 2 s',['Condition','Initial tilt deg','Force N','Torque limit N m'],[[s['id'],s['initial_tilt_deg'],s['push_force_n'],s['torque_limit_nm']] for s in protocol['scenarios']],[.34,.22,.18,.26])
    p('The angle-only sweep uses SDs 0, 0.25, 0.5, 1 and 2 deg, with zero gyro noise and delay. The gyro-only sweep uses SDs 0, 1, 2, 5 and 10 deg/s, with zero angle noise and delay. The combined sweep uses angle SDs 0, 0.5, 1 and 2 deg, gyro SD four times those numerical values, and delays 0, 5, 10, 20 and 40 ms. Coincident configurations across sweep families are deduplicated.')
    p('Each noisy configuration uses 30 seeds, seed_i = 1001 + 104729 i for i=0..29. The same random-number streams are used across controller and observer alternatives. A 32-bit linear-congruential generator and Box-Muller transform produce the angle/gyro innovations. Zero-noise configurations are deterministic and run once, irrespective of seed. There are 28 unique sensing settings, 695 runs per controller-observer-scenario combination, and 8,340 total sensing runs. Six additional ideal baselines reproduce the corresponding archived Python outcomes and metrics.')
    p('The protocol was written before executing this sweep but is not externally preregistered. Conditions were chosen with knowledge of the earlier nominal study; this is exploratory work, not an independent held-out benchmark. Controller gains are frozen to isolate observation changes. Neither gains nor the recovery thresholds were adjusted after viewing these results.')
    h('4 Outcomes, error measures and uncertainty')
    p('A fall is declared at a control endpoint when true absolute tilt reaches 45 deg. A run otherwise stops at 10 s. Recovery requires at least the final one-second dwell within 1 deg tilt, 0.03 m position, and 0.03 m/s speed, all from true state. Recovery time is measured from the end of the pulse. Upright trials that miss this criterion are unsettled. Persistent noise makes this all-samples dwell test demanding; that limitation is reported rather than hidden by redefining success.')
    p('Current-state tilt RMSE compares the controller estimate with true tilt at the same simulation time. Packet-time RMSE compares it with true tilt at the packet acquisition timestamp. Trial RMSE includes samples from initialisation through termination. An early fall shortens that window, so RMSE from different fall times must not be interpreted as an unconditional performance ranking. The nominal RMSE and effort figures explicitly use groups in which every run lasts 10 s.')
    p('Wilson 95% intervals describe event fractions across the 30 pseudorandom noise realisations at one fixed condition. Deterministic cells receive no interval. Interquartile shading describes between-seed variation, not uncertainty in a median. Paired alternatives share seeds; their outcomes are not independent groups. No significance test, multiple-comparison claim, or pooled deployment-success probability is reported. In particular, zero falls in 30 trials does not establish zero failure probability.')
    blocks.append(('pagebreak',));h('5 Independent noise channels');f('step3_fig01_independent_noise')
    p('The two sweeps separate the effect of angle noise from gyro noise. The estimator processes both channels, but its process covariance lets angular rate follow the gyro closely during fast balancing motion. Better angle estimates therefore do not guarantee that velocity fluctuations remain within the strict recovery dwell. These findings are specific to the selected measurement surrogates, gains and filter covariance.')
    blocks.append(('pagebreak',));h('6 Combined noise and delay');f('step3_fig02_noise_delay_falls')
    p('The large-push condition exposes interaction between packet delay, torque saturation and the two fixed feedback laws. The table below retains complete counts across all sensing settings in each scenario. These totals weight each noisy setting by 30 seeds and each deterministic setting once. They are a descriptive inventory only, not an equal-weight average over settings or a probability for an operational population.')
    inventory=[]
    for scenario in protocol['scenarios']:
        for ctrl in ('pid','lqr'):
            for mode in ('direct','kalman'):
                c=Counter(r['outcome'] for r in subset(records,scenario=scenario['id'],controller=ctrl,observer=mode));inventory.append([scenario['id'],ctrl.upper(),mode,c['recovered'],c['unsettled'],c['fallen']])
    table('Inventory: 695 trials in each row',['Scenario','Control','Input','R','U','F'],inventory,[.26,.15,.17,.14,.14,.14])
    blocks.append(('pagebreak',));h('7 True, measured and estimated trajectories');f('step3_fig03_true_measured_estimated')
    p('The interactive 3D page now offers perfect state, direct noisy sensing and Kalman-filtered sensing; angle-noise, gyro-noise and delay controls; a seed field; and a purple estimate outline. The live traces can display tilt, angular rate, position or speed. Solid robot geometry always follows true plant state. Manual force pulses remain available and use the same physics as the original 2D view. Live-run JSON export preserves settings, force events, true states, delivered observations and estimates.')
    blocks.append(('pagebreak',));h('8 Noise reduction versus delay mismatch');f('step3_fig04_delay_estimation_error')
    p('The separation between packet-time and current-state error is the main diagnostic result. Filtering removes part of the acquisition noise, while an old but clean estimate still drives a rapidly moving plant. Adding a predictor could address that mismatch, but its design and independent evaluation are future work. The present data do not establish that the filter is generally safer or more robust than direct sensing.')
    blocks.append(('pagebreak',));h('9 Control effort under persistent noise');f('step3_fig05_control_effort')
    h('10 Verification and limitations')
    p('Automated checks establish exact perfect-sensing state/torque agreement with the existing JavaScript model for both controllers; agreement of selected ideal trials with archived Python results; equality of the scalar Kalman implementation with an independent NumPy dense-matrix/Joseph-covariance reference; causal packet timing and startup holding; seed repeatability; noise-statistics sanity; and positive-semidefinite filter covariance. UI tests cover mode changes, seeded resets, delay readout, state selection and the existing live-force controls. The tests use a renderer mock and do not verify WebGL pixels. Browser access was unavailable for visual inspection.')
    p('This remains a planar model without wheel slip, motor electrical dynamics, realistic IMU acceleration, noisy wheel encoders, packet loss or hardware calibration. Sensor noise magnitudes are illustrative, not measured device specifications. Thirty seeds give limited resolution for rare failures. The filter assumes configured sensor variances are known, and only one process-covariance setting is evaluated. A matched tuning study, additional held-out conditions and hardware or independent-simulator validation are still required for stronger research claims.')
    blocks.append(('pagebreak',));h('11 Reproduction and next step')
    p('Run node run_sensing.js from the project root, then publication/.venv/Scripts/python.exe publication/build_sensing_report.py. The JSON protocol, protocol snapshot, original settings and source hashes are archived. Per-trial results and per-cell summaries are in results/sensing; vector PDF/SVG and 600 dpi PNG figures are in publication/figures. This report is a separate Step 3 supplement: the original manuscript and its deterministic data remain unchanged. Step 4, physically consistent payload variation, has not been implemented.')
    p('The next sensing extension should compare a causal delay predictor against these frozen baselines, using a declared tuning budget and held-out conditions. Before combining sensing with payload changes, define performance measures appropriate to persistent noise alongside the strict original dwell criterion. The author should review the scientific claims, software and AI-assisted drafting before integrating the supplement into a submission.')
    h('References')
    blocks.append(('reference','[1] G. Welch and G. Bishop. An Introduction to the Kalman Filter. UNC Technical Report TR 95-041, version updated 1 March 2004.','https://www.cs.utexas.edu/~pstone/Courses/393Rfall15/readings/Welch%2BBishop-TR-95.pdf'))
    blocks.append(('reference','[2] Z. Music et al. Design of a Networked Controller for a Two-Wheeled Inverted Pendulum Robot. arXiv:1812.03071v2, 2019; related publication DOI 10.1016/j.ifacol.2019.12.153.','https://arxiv.org/abs/1812.03071v2'))
    setup_fonts();build_pdf(HERE/'output/pdf/sensing_report.pdf',title,blocks,manifest,'Step 3 research supplement | Michael Agbo | 12 November 2026');build_tex(HERE/'output/latex/sensing_report.tex',title,blocks,manifest)
    (OUT/'report_content.json').write_text(json.dumps(blocks,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    # Human-readable local report, using the same content and figures as the PDF.
    pieces=[]
    for block in blocks:
        kind=block[0]
        if kind=='heading':pieces.append('<h2>'+html.escape(block[1])+'</h2>')
        elif kind=='paragraph':pieces.append('<p>'+html.escape(block[1])+'</p>')
        elif kind=='figure':pieces.append('<figure><img loading="lazy" src="../../publication/figures/'+block[1]+'.svg" alt="'+html.escape(manifest[block[1]]['caption'],quote=True)+'"><figcaption>'+html.escape(manifest[block[1]]['caption'])+'</figcaption></figure>')
        elif kind=='table':
            _,title_,headers,rows,_=block;pieces.append('<div class="table-wrap"><table><caption>'+html.escape(title_)+'</caption><thead><tr>'+''.join('<th>'+html.escape(str(x))+'</th>' for x in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in row)+'</tr>' for row in rows)+'</tbody></table></div>')
        elif kind=='reference':pieces.append('<p>'+html.escape(block[1])+' <a href="'+html.escape(block[2],quote=True)+'">Source</a></p>')
    page='<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Step 3 | Sensing and delay</title><style>body{margin:0;background:#f3f2ed;color:#18364d;font:16px/1.7 Segoe UI,Arial,sans-serif}main{max-width:1000px;margin:auto;padding:40px 30px}h1{font-size:42px;line-height:1.15}h2{margin-top:40px}a{color:#176779}nav{display:flex;gap:20px;flex-wrap:wrap}figure{margin:30px 0;padding:18px;background:white;border:1px solid #d3dde2;border-radius:12px}img{width:100%;height:auto}figcaption{font-size:13px;line-height:1.6;color:#536474}table{border-collapse:collapse;width:100%;font-size:13px;background:white}th,td{padding:9px;border:1px solid #d3dde2;text-align:left}th{background:#e5ecef}caption{text-align:left;font-weight:600;margin-bottom:8px}.table-wrap{overflow-x:auto}.tag{font-size:12px;letter-spacing:2px}footer{margin-top:40px;border-top:1px solid #cbd5dc;padding-top:20px}</style></head><body><main><nav><a href="../../visualisation/index.html">Live 3D sensing lab</a><a href="../../publication/output/pdf/sensing_report.pdf">Download PDF</a><a href="results.csv">All trials CSV</a><a href="summary.csv">All cells CSV</a></nav><p class="tag">MICHAEL AGBO / STEP 3</p><h1>Noisy sensing and measurement delay</h1><p>8,340 trials. Three physical conditions. Direct and Kalman-filtered observations.</p>'+''.join(pieces)+'<footer>Local research supplement. Simulation results; no hardware experiments.</footer></main></body></html>'
    (OUT/'report.html').write_text(page,encoding='utf-8');print('Created five figures, summary files, HTML report, PDF supplement and editable LaTeX.')

if __name__=='__main__':main()
