"""Boundary and payload evidence, with exploratory zoom and numerical audit disclosure."""
from collections import Counter, defaultdict
import hashlib
import html
import json
import math
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch
from matplotlib.ticker import MaxNLocator
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT/'results/mujoco_extension'
FIG = HERE/'figures'
sys.path.insert(0, str(HERE))
from build_reports import setup_fonts, build_pdf, build_tex
OUTCOMES = ['recovered', 'unsettled', 'fallen']
COLOURS = ['#087d88', '#d4ad38', '#c45b21']
ENGINE_COLOURS = {'planar':'#c45b21', 'native':'#087d88'}
CODE = {'recovered':'R', 'unsettled':'U', 'fallen':'F'}
plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':9, 'axes.titlesize':10,
                     'legend.fontsize':8, 'pdf.fonttype':42, 'svg.fonttype':'none',
                     'axes.spines.top':False, 'axes.spines.right':False, 'grid.alpha':.2})


def load(name):
    folder = ROOT/f'results/mujoco_{name}'
    data = json.loads((folder/'study.json').read_text())
    for f, digest in data['metadata']['source_sha256'].items():
        assert hashlib.sha256((ROOT/f).read_bytes()).hexdigest() == digest, f
    data['by_id'] = {r['id']:r for r in data['records']}
    data['case_lookup'] = {c['case_id']:c for c in data['cases']}
    data['arrays'] = np.load(folder/'trajectories.npz')
    return data


def status(data):
    chosen = data['selected_pairs']
    last_changes = []
    groups = defaultdict(list)
    for r in data['refinement']: groups[r['case_id'], r['controller'], r['engine']].append(r)
    for key, values in groups.items():
        finest = min(values, key=lambda r:r['fine_step_s'])
        if not finest['same_outcome']: last_changes.append(dict(case_id=key[0], controller=key[1], engine=key[2], **{k:finest[k] for k in ['coarse_step_s','fine_step_s','coarse_outcome','fine_outcome']}))
    return {'conditions':len(data['cases']), 'integrations':len(data['records']), 'pairs':len(chosen),
            'outcome_agreement':sum(p['same_outcome'] for p in chosen),
            'fall_agreement':sum(p['same_fall_label'] for p in chosen),
            'planar':dict(Counter(p['planar_outcome'] for p in chosen)),
            'native':dict(Counter(p['native_outcome'] for p in chosen)),
            'last_step_label_changes':last_changes, 'audit':data['audit']}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    coarse, zoom, payload = [load(n) for n in ['boundary','boundary_zoom','payload']]
    summaries = {n:status(d) for n,d in [('coarse',coarse),('zoom',zoom),('payload',payload)]}
    protocol = payload['metadata']['protocol']
    lookup = {(r['case_id'],r['controller'],r['engine'],r['physics_step_s']):r for r in payload['records']}
    loaded_cases = [c for c in payload['cases'] if c['mass_kg'] > 0]
    def record(case, controller, engine, step=.00025): return lookup[case['case_id'],controller,engine,step]
    def matrix_plot(ax, matrix, title, xlabel, ylabel):
        ax.imshow(matrix, cmap='Blues', vmin=0, vmax=max(1,matrix.max()))
        for y in range(3):
            for x in range(3): ax.text(x,y,str(matrix[y,x]),ha='center',va='center',color='white' if matrix[y,x]>matrix.max()*.55 else '#183449')
        ax.set(xticks=range(3),xticklabels=['R','U','F'],yticks=range(3),yticklabels=['R','U','F'],title=title,xlabel=xlabel,ylabel=ylabel)
    manifest = {}
    def save(fig, name, caption):
        for ext in ['png','svg','pdf']:
            target=FIG/f'{name}.{ext}'
            refresh = ('--refresh-audit-layout' in sys.argv and name=='ext_fig03_refinement') or ('--refresh-layout' in sys.argv and name in ['ext_fig06_payload_atlas','ext_fig08_paired_metrics'])
            if '--reuse-figures' not in sys.argv or not target.exists() or refresh: fig.savefig(target,dpi=400,bbox_inches='tight',pad_inches=.12)
        plt.close(fig); manifest[name]={'name':name,'caption':caption}

    fig, axs = plt.subplots(2,2,figsize=(7.4,4.7),layout='constrained')
    for ax,(scenario,limit) in zip(axs.flat,[('force',.3),('force',.5),('force',.7),('startup',.1)]):
        rows=[]
        for sign in [-1,1]:
            selected=[]
            for pair in zoom['selected_pairs']:
                c=zoom['case_lookup'][pair['case_id']]
                value=c['push_force_n'] if scenario=='force' else c['initial_tilt_deg']
                if c['scenario']==scenario and c['torque_limit_nm']==limit and np.sign(value)==sign:
                    selected.append((abs(value),pair))
            selected.sort()
            x=[v for v,_ in selected]
            for engine in ['planar','native']: rows.append([OUTCOMES.index(p[engine+'_outcome']) for _,p in selected])
        values=np.array(rows); ax.imshow(values,cmap=ListedColormap(COLOURS),vmin=0,vmax=2,aspect='auto')
        for y in range(values.shape[0]):
            for j in range(values.shape[1]): ax.text(j,y,['R','U','F'][values[y,j]],ha='center',va='center',color='white' if values[y,j]!=1 else '#183449',fontsize=7)
        ticks=list(range(0,len(x),2))
        ax.set(xticks=ticks,xticklabels=[f'{x[j]:g}' for j in ticks],yticks=range(4),yticklabels=['Planar -','Native -','Planar +','Native +'],
               xlabel='Force magnitude (N)' if scenario=='force' else 'Initial tilt magnitude (deg)',title=f'{"Force" if scenario=="force" else "Startup"}: torque limit {limit:g} N m')
        ax.tick_params(labelsize=7)
    fig.legend(handles=[Patch(color=c,label=n.capitalize()) for c,n in zip(COLOURS,OUTCOMES)],loc='outside upper center',ncol=3)
    save(fig,'ext_fig01_boundary_zoom','All cells of the exploratory transition zoom. Signs refer to force or initial tilt, as appropriate. The coarse grid located each interval; the follow-up uses 0.05 N or 0.05 deg spacing and the finest tested outcome for each cell. R/U/F mean recovered, upright-unsettled and fallen. Endpoints repeat coarse conditions. No interpolation between cells or population probability is implied; audit resolutions are retained in the data.')

    diagnostic = sorted(zoom['selected_pairs'],key=lambda p:(p['same_outcome'],-p['max_pitch_error_deg'],p['case_id']))[0]
    dc = zoom['case_lookup'][diagnostic['case_id']]
    fig,axs=plt.subplots(3,1,figsize=(7.2,5.1),layout='constrained')
    for engine in ['planar','native']:
        a=zoom['arrays'][diagnostic[engine+'_id']]
        for ax,col,factor in zip(axs,[3,1,5],[180/math.pi,1,1]):
            ax.plot(a[:,0],a[:,col]*factor,label=engine.capitalize(),color=ENGINE_COLOURS[engine],ls='-' if engine=='planar' else '--')
    axs[0].axhline(45,color='#7b7471',ls=':',lw=.8);axs[0].axhline(-45,color='#7b7471',ls=':',lw=.8);axs[0].legend()
    for ax,label in zip(axs,['Pitch (deg)','Position (m)','Total torque (N m)']):
        if dc['push_force_n']: ax.axvspan(2,2.2,color='#d1a21b',alpha=.18)
        ax.set(xlabel='Time (s)',ylabel=label);ax.grid()
    save(fig,'ext_fig02_boundary_example',f'Post hoc diagnostic: tilt {dc["initial_tilt_deg"]:g} deg, force {dc["push_force_n"]:g} N, torque limit {dc["torque_limit_nm"]:g} N m, physics step {diagnostic["physics_step_s"]*1000:g} ms. The selection prioritizes outcome disagreement, then largest common-horizon pitch difference, with case ID breaking ties. This is a discrepancy illustration, not a representative or favourable example. Lines stop at the fall endpoint; the terminal torque zero is a placeholder. Fall time is resolved only at 5 ms control endpoints.')

    fig,axs=plt.subplots(2,2,figsize=(6.6,4.1),layout='constrained')
    for i,(data,label,h) in enumerate([(zoom,'Boundary zoom',.00025),(payload,'Payload + zero-load',.0005)]):
        for j,engine in enumerate(['planar','native']):
            matrix=np.zeros((3,3),int)
            for p in data['selected_pairs']:
                before=next(r for r in data['records'] if r['case_id']==p['case_id'] and r['controller']==p['controller'] and r['engine']==engine and r['physics_step_s']==h)
                after=data['by_id'][p[engine+'_id']]
                matrix[OUTCOMES.index(before['outcome']),OUTCOMES.index(after['outcome'])]+=1
            matrix_plot(axs[i,j],matrix,label+' / '+engine,'Finest tested outcome','Coarsest outcome')
    save(fig,'ext_fig03_refinement','Coarsest-to-finest outcome changes for each engine. R/U/F denote recovered, upright-unsettled and fallen. Boundary zoom begins at 0.25 ms; payload begins at 0.5 ms. The finest step can vary because disagreement-triggered audits are selective. Matrices include all 88 zoom cells or all 80 payload/controller pairs (including the unloaded control), not only audited cases. Persistent last-two-step changes are reported separately rather than declared resolved.')

    renders=json.loads((OUT/'render_metadata.json').read_text())['models']
    fig,axs=plt.subplots(1,3,figsize=(7.2,4.0),layout='constrained')
    for ax,r in zip(axs,renders):
        image=plt.imread(OUT/r['image']);ax.imshow(image[140:820,:,:]);ax.axis('off')
        ax.set_title(f'{r["mass_kg"]:g} kg at {r["height_m"]:g} m',fontsize=10)
        ax.text(.5,-.035,f'COM {r["combined_com_m"]:.3f} m\nPitch inertia {r["pitch_inertia_kg_m2"]:.5f} kg m²',ha='center',va='top',transform=ax.transAxes,fontsize=8)
    save(fig,'ext_fig04_payload_models','Native MuJoCo renders of three of the nine tested payloads. Gold cuboids use the original 0.08 x 0.12 x 0.06 m shape; heights are measured along the body from the axle. The shell is transparent and a massless noncolliding mast is added for presentation only. Low payload placements intersect the illustrative shell: this is a centred rigid mass arrangement, not a mountable hardware design. Printed COM and inertia refer to combined torso and payload, excluding the translating base and additional native chassis inertia.')

    transitions={}
    fig,axs=plt.subplots(1,2,figsize=(7.0,3.3),layout='constrained')
    for ax,engine in zip(axs,['planar','native']):
        matrix=np.zeros((3,3),int)
        for c in loaded_cases:
            a,b=[record(c,v,engine) for v in ['fixed','exact']]
            matrix[OUTCOMES.index(a['outcome']),OUTCOMES.index(b['outcome'])]+=1
        matrix_plot(ax,matrix,engine.capitalize()+' (36 loaded cases)','Payload-aware LQR','Fixed nominal LQR')
        transitions[engine]={'matrix':matrix.tolist(), 'fall_to_upright':int(matrix[2,:2].sum()),'upright_to_fall':int(matrix[:2,2].sum())}
    save(fig,'ext_fig05_controller_outcomes','Paired fixed-to-payload-aware outcomes at the common primary physics step of 0.25 ms. Each matrix contains 36 loaded conditions (nine mass/height combinations and four force/torque scenarios). Unloaded controls are excluded. R/U/F mean recovered, upright-unsettled and fallen. Both designs use the same Q/R, ideal measurements and true plant; only the gain differs. Counts describe this selected finite grid. Finer-step audits assess sensitivity separately and do not silently change the primary comparison resolution.')

    fig,axs=plt.subplots(2,4,figsize=(7.5,4.7),layout='constrained')
    for i,engine in enumerate(['planar','native']):
        for j,scenario in enumerate(protocol['payload']['scenarios']):
            ax=axs[i,j];values=np.zeros((3,3));labels={}
            for y,height in enumerate(protocol['payload']['heights_m']):
                for x,mass in enumerate(protocol['payload']['masses_kg']):
                    c=next(c for c in loaded_cases if c['mass_kg']==mass and c['height_m']==height and c['scenario']==scenario['id'])
                    a,b=[record(c,v,engine)['outcome'] for v in ['fixed','exact']]
                    labels[y,x]=CODE[a]+'/'+CODE[b]
                    values[y,x]=OUTCOMES.index(a) if a==b else (3 if a=='fallen' else 4)
            ax.imshow(values,origin='lower',vmin=0,vmax=4,cmap=ListedColormap([*COLOURS,'#31558a','#79549b']),aspect='auto')
            for (y,x),label in labels.items():ax.text(x,y,label,ha='center',va='center',fontsize=8,color='#203c4f' if values[y,x]==1 else 'white')
            ax.set(xticks=range(3),xticklabels=protocol['payload']['masses_kg'],yticks=range(3),yticklabels=protocol['payload']['heights_m'],xlabel='Mass (kg)',ylabel='Height (m)' if j==0 else '',title=f'{engine.capitalize()}\n{scenario["push_force_n"]:+g} N / {scenario["torque_limit_nm"]:g} N m')
            ax.tick_params(labelsize=7)
    fig.legend(handles=[Patch(color=COLOURS[0],label='Both recovered'),Patch(color=COLOURS[2],label='Both fallen')],loc='outside upper center',ncol=2)
    save(fig,'ext_fig06_payload_atlas','Every loaded case at 0.25 ms. Text gives fixed/aware outcomes (R recovered, U unsettled, F fallen). Teal marks recovery under both designs and orange marks falls under both; there are no differing paired labels on this grid. Initial tilt is +12 deg in every panel. Positive and negative pushes are world-X forces at the combined COM. All nine mass/height combinations and four prescribed force/torque scenarios remain visible.')

    example=protocol['payload']['example']
    ec=next(c for c in loaded_cases if all(c[k]==example[k] for k in ['mass_kg','height_m','scenario']))
    fig,axs=plt.subplots(3,1,figsize=(7.2,5.0),layout='constrained')
    for engine in ['planar','native']:
        for controller,color in [('fixed','#c45b21'),('exact','#087d88')]:
            r=record(ec,controller,engine);a=payload['arrays'][r['id']]
            for ax,col,factor in zip(axs,[3,1,5],[180/math.pi,1,1]):
                ax.plot(a[:,0],a[:,col]*factor,color=color,ls='-' if engine=='planar' else '--',lw=1.2,label=f'{engine.capitalize()} / {"fixed" if controller=="fixed" else "aware"}')
    for ax,label in zip(axs,['Pitch (deg)','Position (m)','Total torque (N m)']):
        ax.axvspan(2,2.2,color='#d1a21b',alpha=.18);ax.set(xlabel='Time (s)',ylabel=label);ax.grid()
    axs[0].legend(ncol=2,fontsize=7)
    save(fig,'ext_fig07_payload_example','Protocol-selected example: 0.65 kg payload at 0.26 m, initial tilt +12 deg, -15 N force for 0.2 s from t=2 s, and torque limit 0.6 N m. Fixed and payload-aware LQR are compared in both engines at 0.25 ms physics resolution with 200 Hz control. Each line uses its own feedback trajectory under the same physical condition. Overlap indicates close trajectory agreement. Terminal torque is a zero placeholder; no continuation past a fall is plotted.')

    fig,axs=plt.subplots(1,2,figsize=(7.2,3.5),layout='constrained')
    survivor_cases=[c for c in loaded_cases if all(record(c,v,e)['outcome']!='fallen' for v in ['fixed','exact'] for e in ['planar','native'])]
    metric_summary=[]
    for ax,key,label in zip(axs,['squared_torque_integral_nm2_s','peak_pitch_deg'],['Change in squared torque integral ((N m)² s)','Change in peak absolute pitch (deg)']):
        points=[]
        for c in survivor_cases:
            points.append([record(c,'exact',e)[key]-record(c,'fixed',e)[key] for e in ['planar','native']])
        points=np.asarray(points).reshape(-1,2)
        if len(points):
            ax.scatter(points[:,0],points[:,1],color='#087d88',s=25,alpha=.7)
            lo,hi=float(points.min()),float(points.max());span=max(hi-lo,1e-5);lo-=span*.12;hi+=span*.12
            ax.plot([lo,hi],[lo,hi],':',color='#647586');ax.set_xlim(lo,hi);ax.set_ylim(lo,hi)
            metric_summary.append({'metric':key,'n':len(points),'planar_median_delta':float(np.median(points[:,0])),'native_median_delta':float(np.median(points[:,1]))})
        ax.axhline(0,color='#c2c9cf',lw=.6);ax.axvline(0,color='#c2c9cf',lw=.6)
        ax.set(xlabel='Planar: aware minus fixed',ylabel='Native: aware minus fixed',title=label+'\n'+f'{len(survivor_cases)} common survivor cases');ax.grid()
        ax.xaxis.set_major_locator(MaxNLocator(nbins=4))
        ax.yaxis.set_major_locator(MaxNLocator(nbins=4))
    save(fig,'ext_fig08_paired_metrics','Controller metric changes at the matched 0.25 ms step, restricted to loaded conditions surviving under both controllers in both engines. Each dot is one condition; coordinates are aware-minus-fixed metric differences. The dotted diagonal denotes agreement about the size of the controller effect. Negative values mean a lower metric. This four-way survivor restriction is reported explicitly and must be read with the full fall inventory; it is not an unconditional ranking or statistical confidence statement.')

    # Text and report assembly are kept separate from the plotting calculations.
    summary={'coarse':summaries['coarse'],'zoom':summaries['zoom'],'payload':summaries['payload'],
             'payload_primary_controller_transitions':transitions,'paired_survivor_metrics':metric_summary,
             'boundary_diagnostic':{'case':dc,'pair':diagnostic},'payload_example':ec}
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (HERE/'mujoco_extension_figure_manifest.json').write_text(json.dumps(list(manifest.values()),indent=2)+'\n')
    build_text(coarse,zoom,payload,summary,manifest,lookup,loaded_cases)


def build_text(coarse,zoom,payload,summary,manifest,lookup,loaded_cases):
    protocol=payload['metadata']['protocol']
    cs,zs,ps=[summary[k] for k in ['coarse','zoom','payload']]
    b=[]
    def h(t):b.append(('heading',t))
    def p(t):b.append(('paragraph',t))
    def page(t,figure=None):
        b.append(('pagebreak',));h(t)
        if figure:b.append(('figure',figure))
    def get(c,v,e):return lookup[c['case_id'],v,e,.00025]
    primary_agreement={v:sum(get(c,v,'planar')['outcome']==get(c,v,'native')['outcome'] for c in loaded_cases) for v in ['fixed','exact']}
    effects=[]
    for c in loaded_cases:
        values={e:int(get(c,'fixed',e)['outcome']=='fallen')-int(get(c,'exact',e)['outcome']=='fallen') for e in ['planar','native']}
        effects.append(dict(case_id=c['case_id'],**values,agrees=values['planar']==values['native']))
    summary['payload_primary_engine_agreement']=primary_agreement
    summary['payload_fall_effect_agreement']=sum(r['agrees'] for r in effects)
    summary['payload_fall_effects']=effects
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    title='Fall-boundary agreement and payload-aware control in MuJoCo'
    h('Scope and principal findings')
    p('This supplement extends the nominal simulator check to fall-adjacent conditions and rigid payloads. It compares the existing analytical planar model with a free-body MuJoCo wheel-contact model under matched controllers, sampling periods and prescribed disturbances. Each engine closes its own feedback loop. Neither engine is treated as ground truth. The work is computational verification and discrepancy analysis; no physical robot or hardware calibration is included.')
    p(f'The coarse boundary grid contains {cs["conditions"]} conditions: the two engines agree on every outcome, including {cs["planar"].get("recovered",0)} recoveries and {cs["planar"].get("fallen",0)} falls. An exploratory follow-up subdivides every observed transition interval to 0.05 N or 0.05 deg. In those {zs["conditions"]} zoom conditions, {zs["outcome_agreement"]} finest-tested three-way outcomes agree and {zs["conditions"]-zs["outcome_agreement"]} disagree. The zoom is selected from coarse results, not a new independent held-out sample.')
    p(f'The payload extension uses nine loaded mass/height combinations, four force/torque scenarios, two fixed gain designs and both engines, plus an unloaded control. At the common primary 0.25 ms step, engines agree in {primary_agreement["fixed"]}/36 loaded fixed-LQR cases and {primary_agreement["exact"]}/36 loaded payload-aware cases. Their conclusions about whether redesign prevents or introduces a fall agree in {summary["payload_fall_effect_agreement"]}/36 conditions.')
    b.append(('table','Study inventory; repetitions are not independent experimental samples',['Stage','Conditions','Integrations','Finest-pair agreement'],[
        ['Coarse boundary',cs['conditions'],cs['integrations'],f'{cs["outcome_agreement"]}/{cs["pairs"]}'],
        ['Exploratory boundary zoom',zs['conditions'],zs['integrations'],f'{zs["outcome_agreement"]}/{zs["pairs"]}'],
        ['Payload + unloaded controls',f'{len(payload["cases"])} x 2 gains',ps['integrations'],f'{ps["outcome_agreement"]}/{ps["pairs"]}']],[.38,.18,.18,.26]))
    p('Payload integration counts include the reduced-model trajectory checks and any selective audits. Zoom endpoints repeat coarse physical conditions. Outcome counts are descriptive properties of these specified grids; they are not deployment probabilities or statistical estimates of success.')

    page('1 Protocol, comparability and numerical controls')
    p('The coarse force slices start upright and apply signed forces with magnitudes 10-15 N at a 0.3 N m torque limit, 15-20 N at 0.5 N m, or 20-25 N at 0.7 N m. Spacing is 0.5 N. The unforced startup slice uses signed initial tilts 10-15 deg at a 0.1 N m limit, with 0.5 deg spacing. Both signs are evaluated. These intervals were selected from the archived planar robustness study, before running the new MuJoCo comparisons.')
    p('After the coarse grid completed, a separate exploratory protocol was fixed: take every adjacent three-way outcome transition in either engine, subdivide each interval into ten equal parts and evaluate all eleven values. This procedure does not assume a monotonic boundary and retains shared endpoints. The follow-up keeps gains, friction, inertia and classification thresholds fixed. No settings were fitted to remove disagreement.')
    p('All experiments last at most 10 s. Controls and ideal feedback update every 5 ms. A nonzero horizontal pulse acts from 2.00 through 2.20 s, exactly 40 control intervals unless the robot falls first. Total two-motor torque is clipped to the prescribed limit and split equally between the native wheel motors. Gains are reused from the existing CARE bank with Q = diag(12,2,120,2), R = 1. Fixed LQR uses the nominal gain; payload-aware LQR uses the true load, fixed throughout each run.')
    p('A fall is declared at a control endpoint when absolute pitch reaches 45 deg; native roll is also checked. Recovery requires a final continuous dwell of at least one second with |pitch| <=1 deg, |x| <=0.03 m and |vx| <=0.03 m/s, after the pulse ends when nonzero. Native lateral/yaw recovery is not tested. Fall times are quantized to the 5 ms detection interval. Trajectory differences are restricted to the common pre-fall horizon, with no extrapolation after either trial ends.')
    p('The planar solver uses RK4; MuJoCo uses implicitfast. The original native contact model remains: sliding friction 1.0, contact time constant 5 ms, torsional friction 0.005, condim 4 and no initial contact-settling warm-up. The axle begins 0.2 mm above the radius. Native chassis pitch inertia of 0.0002 kg m² remains additional to the ideal planar assumptions. Thus any discrepancy can involve contact, startup or extra inertia; it cannot be attributed solely to slip [1,2].')
    checks=json.loads((OUT/'data_checks.json').read_text())
    p(f'All {checks["checked_integrations"]} stored integrations pass source-hash, finite-state, timing, torque-limit, pulse-window and engine-warning checks. The {len(checks["repeated_coarse_zoom_trajectories"])} coarse/zoom endpoint trajectories repeated at the same physics step are bitwise identical. Both archived JavaScript payload examples reproduce their outcome, with state errors below 1e-4 in each coordinate\'s native units despite using a smaller integration step in the new Python runner.')

    page('2 Fine boundary map and observed transitions','ext_fig01_boundary_zoom')
    groups=defaultdict(list)
    for pair in zoom['selected_pairs']:
        c=zoom['case_lookup'][pair['case_id']]
        value=c['push_force_n'] if c['scenario']=='force' else c['initial_tilt_deg']
        groups[c['scenario'],c['torque_limit_nm'],int(np.sign(value))].append((abs(value),pair))
    table=[]
    for (scenario,limit,sign),group in sorted(groups.items()):
        group.sort();entries=[]
        for e in ['planar','native']:
            changes=[]
            for a,bp in zip(group,group[1:]):
                left,right=a[1][e+'_outcome'],bp[1][e+'_outcome']
                if left!=right:changes.append(f'{a[0]:g}{CODE[left]} to {bp[0]:g}{CODE[right]}')
            entries.append('; '.join(changes) or 'No transition in zoom')
        table.append([f'{scenario}, {limit:g}',f'{sign:+d}',*entries])
    b.append(('table','Adjacent sampled transitions: force magnitudes in N, startup angles in deg',['Slice / limit N m','Sign','Planar','Native'],table,[.23,.10,.335,.335]))
    p('These are adjacent observed labels, not exact critical values. A recovery/fall interval locates a transition only at the evaluated resolution. It does not guarantee that every intermediate point follows a monotonic ordering. The full coarser map and all per-step classifications are archived.')

    page('3 Boundary discrepancy diagnostic','ext_fig02_boundary_example')
    diagnostic=summary['boundary_diagnostic']['pair']
    p(f'The illustrated pair is {diagnostic["planar_outcome"]} in the planar engine and {diagnostic["native_outcome"]} in MuJoCo. Its shared-horizon maximum differences are {diagnostic["max_position_error_m"]:.4g} m and {diagnostic["max_pitch_error_deg"]:.4g} deg. Its common observation horizon is {diagnostic["common_horizon_s"]:.3f} s. The selection is explicitly post hoc and designed to expose disagreement; the complete grid is the primary evidence.')
    disagreements=[pair for pair in zoom['selected_pairs'] if not pair['same_outcome']]
    if disagreements:
        details=[]
        for pair in disagreements:
            c=zoom['case_lookup'][pair['case_id']]
            details.append(f'{c["push_force_n"]:+g} N, {c["initial_tilt_deg"]:+g} deg, limit {c["torque_limit_nm"]:g} N m: planar {pair["planar_outcome"]}, native {pair["native_outcome"]}')
        p('Disagreeing zoom cells: '+'; '.join(details)+'. Positive/negative mirror cases are symmetry checks, not independent stochastic evidence.')
    both_fall=[r for r in zoom['selected_pairs'] if r['fall_time_difference_s'] is not None]
    p(f'Among the {len(both_fall)} zoom pairs where both engines fall, the largest absolute fall-time difference is {max((abs(r["fall_time_difference_s"]) for r in both_fall),default=0):.4g} s. Matching a binary fall label does not require matching trajectories or the time of threshold crossing. Where only one engine falls, a fall-time difference is undefined and is not reported as zero.')

    page('4 Step refinement and numerical sensitivity','ext_fig03_refinement')
    p('The coarse boundary and payload grids use 0.5 and 0.25 ms physics steps. A pair is flagged if engine outcomes differ at the finer step or if either engine changes label when the step is halved. Flagged pairs are repeated in both engines at 0.125 ms; a further repeat at 0.0625 ms is triggered only by another within-engine label change. The exploratory zoom begins at 0.25 and 0.125 ms, with analogous selective steps of 0.0625 and 0.03125 ms. Control and event-detection intervals stay at 5 ms throughout.')
    b.append(('table','Selective numerical audit inventory',['Stage','First-round pairs','Second-round pairs','Last-step changes'],[[name,s['audit']['first_round_pairs'],s['audit']['second_round_pairs'],len(s['last_step_label_changes'])] for name,s in [('Coarse boundary',cs),('Boundary zoom',zs),('Payload',ps)]],[.31,.23,.23,.23]))
    changed=sum(len(s['last_step_label_changes']) for s in [cs,zs,ps])
    p(f'{changed} engine/condition/controller sequences change outcome between their last two tested steps. The cross-engine zoom disagreements persist after refinement. This supports reporting a model discrepancy at the tested resolutions; it does not identify which model is physically correct or establish an exact limiting boundary. Full records retain every tested resolution.')

    page('5 Rigid payload construction and independent checks','ext_fig04_payload_models')
    p('The loaded body combines nominal torso mass m0 = 1.2 kg at l0 = 0.18 m with payload mass mp at height h. Its mass is M = m0 + mp and COM is L = (m0 l0 + mp h)/M. Pitch inertia about that COM is I0 + m0(l0-L)² + mp(h-L)² + mp(0.08²+0.06²)/12, with I0 = 0.012 kg m². Native payload roll and yaw inertias also use the cuboid dimensions. The translating base/wheel mass remains 0.35 kg.')
    p('MuJoCo represents torso and payload as separate rigidly welded bodies. A total world-X force F is split as F m0/M on the torso and F mp/M on the payload. The sum and its moment about the axle therefore equal a single force at the combined COM. This mapping is verified directly from compiled mass properties and transformed body positions. The native additional chassis inertia is retained rather than silently absorbed into the planar body.')
    check_count=sum(r['acceleration_checks'] for r in payload['load_checks'])
    p(f'All ten load configurations, including zero payload, pass mass, COM, pitch-inertia and resultant-force/moment checks. {check_count} reduced-model acceleration cases agree within 1e-9 in their stated acceleration units. All {len(payload["reduced_checks"])} selected loaded ideal-rolling trajectories match the planar outcomes and satisfy a 1e-7 componentwise trajectory tolerance. All {len(payload["zero_load_checks"])} unloaded fixed/aware comparisons are bitwise identical at the two base steps. The previous nominal implementation regression also matches exactly.')

    page('6 Fixed versus payload-aware outcomes','ext_fig05_controller_outcomes')
    inventory=[]
    for e in ['planar','native']:
        for v in ['fixed','exact']:
            counts=Counter(get(c,v,e)['outcome'] for c in loaded_cases)
            inventory.append([e,'Fixed' if v=='fixed' else 'Aware',*[counts[x] for x in OUTCOMES]])
    b.append(('table','Primary loaded comparison at 0.25 ms; 36 cases per row',['Engine','Controller','Recovered','Unsettled','Fallen'],inventory,[.22,.22,.2,.18,.18]))
    for e in ['planar','native']:
        t=summary['payload_primary_controller_transitions'][e]
        p(f'In {e}, redesign turns {t["fall_to_upright"]} fixed-controller falls into upright trials and turns {t["upright_to_fall"]} fixed-controller upright trials into falls. These paired counts retain the same true load, disturbance and torque budget. They do not rank all LQR designs or all payload conditions.')
    p(f'At the primary matched step, fixed-controller engine labels agree for {primary_agreement["fixed"]}/36 loaded conditions and payload-aware labels for {primary_agreement["exact"]}/36. The engines agree on the direction of the redesign effect on falling for {summary["payload_fall_effect_agreement"]}/36 cases. Finer audit labels remain available alongside this uniform-step primary analysis.')
    loaded_ids={c['case_id'] for c in loaded_cases}
    survivor_pairs=[pair for pair in payload['pairs'] if pair['case_id'] in loaded_ids and pair['physics_step_s']==.00025 and pair['planar_outcome']!='fallen' and pair['native_outcome']!='fallen']
    if survivor_pairs:
        p(f'Among the {len(survivor_pairs)} loaded controller/condition pairs surviving in both engines for 10 s, maximum engine-to-engine differences are {max(r["max_position_error_m"] for r in survivor_pairs):.4g} m and {max(r["max_pitch_error_deg"] for r in survivor_pairs):.4g} deg. This full-horizon subset excludes any fallen pair; its trajectory agreement cannot replace the complete fall comparison.')

    page('7 Complete payload condition atlas','ext_fig06_payload_atlas')
    p('This focused extension evaluates the nine masses/heights from the previous controller study at four of its 24 ideal force/tilt/torque conditions per load: +12 deg initial tilt, pushes -15 or +15 N, and torque limits 0.3 or 0.6 N m. It includes an unloaded control at the same four conditions. It does not repeat the previous noisy-sensing grid, delayed filters or incorrect load estimates. Consequently conclusions are limited to ideal measurements and exact assumed payloads.')
    early=sum(r['outcome']=='fallen' and not r['push_started'] for r in payload['records'] if r['engine'] in ['planar','native'] and r['physics_step_s']==.00025 and r['mass_kg']>0)
    early_text=(f'There are {early} primary loaded engine/controller records that fall before receiving a scheduled push. They are retained in the inventory.' if early else 'Every primary loaded trial received the scheduled pulse; no pre-push fall occurred.')
    p(early_text+' The native model uses a centred rigid payload. Off-axis mounting, pickup during motion, support flexibility, motor electrical dynamics and identified wheel-floor traction remain outside this experiment.')

    page('8 Prespecified loaded trajectory','ext_fig07_payload_example')
    p('This example was specified in the extension protocol before execution. Its force acts at the true combined COM under both controllers. Gains are selected before a run and held constant; there is no online payload estimator or adaptive gain update. The ideal-rolling dynamics check isolates mass-property implementation, while the native model adds wheel contact and extra degrees of freedom.')

    page('9 Paired continuous metrics','ext_fig08_paired_metrics')
    n=summary['paired_survivor_metrics'][0]['n'] if summary['paired_survivor_metrics'] else 0
    p(f'There are {n} loaded conditions surviving in all four engine/controller combinations. Restricting continuous full-horizon metrics to this common set avoids comparing a short fallen trajectory with a ten-second recovery, but it excludes failures and must not replace the complete outcome analysis. Squared torque is the time integral of total commanded motor torque squared; it is a control-effort proxy, not electrical energy consumption.')
    metric_labels={'squared_torque_integral_nm2_s':'Squared torque integral ((N m)² s)','peak_pitch_deg':'Peak absolute pitch (deg)'}
    b.append(('table',f'Median paired aware-minus-fixed differences on {n} common survivors',['Metric','Planar','Native'],[[metric_labels[r['metric']],f'{r["planar_median_delta"]:.5g}',f'{r["native_median_delta"]:.5g}'] for r in summary['paired_survivor_metrics']],[.50,.25,.25]))
    p('The plotted differences compare aware and fixed designs within each engine at the same 0.25 ms physics step. Agreement about continuous metrics can be assessed separately from agreement about falling. No confidence intervals or significance tests are assigned to this deterministic, selected set of conditions.')

    page('10 Reproduction, limits and use in the manuscript')
    p(f'Engine versions: MuJoCo {payload["metadata"]["mujoco"]}, NumPy {payload["metadata"]["numpy"]}, Python {payload["metadata"]["python"]}. Run mujoco_lab/run_extension_study.ps1 from the project root to reproduce the coarse boundary study, exploratory zoom, payload study, renders and report in order. Each phase stores source SHA-256 hashes, the applicable protocol, every trial metric and all 200 Hz trajectories. Reporting checks recorded source hashes before using the data. Cached trial results are keyed by source content and complete job settings.')
    p('Results are stored separately in results/mujoco_boundary, results/mujoco_boundary_zoom and results/mujoco_payload. Each study.json names the trajectory columns and physical units used in its trajectories.npz. The combined HTML report, summary and rendered payload illustrations are in results/mujoco_extension. Editable LaTeX, standalone vector figures and the PDF supplement are included in the research archive. Baseline data and earlier reports are retained as distinct studies.')
    p('These checks strengthen the numerical evidence for the stated nominal and loaded equations. The exploratory zoom is outcome-conditioned, shares coarse endpoints and has finite force/angle resolution. Numerical refinement cannot identify physical parameters or turn a model comparison into a hardware experiment. Additional chassis inertia, contact compliance, the initial contact gap and geometric assumptions remain potential discrepancy sources. General 3D robustness is untested: forces and controller action are sagittal, with no steering or roll controller.')
    p('Use the results as a bounded methods and validation supplement, including disagreements and resolution limits. The next research step is to identify actual mass properties, actuator response and traction from a physical design or robot, and to separate calibration conditions from a held-out experimental validation set. Any claim of payload-aware superiority must retain the sensing conditions and controller budgets under which it was observed. Hardware validation remains a separate source of evidence.')
    h('References')
    for number,label,url in [(1,'MuJoCo XML Reference: inertial properties, joints, actuators and contacts.','https://mujoco.readthedocs.io/en/stable/XMLreference.html'),(2,'MuJoCo Computation: rigid-body dynamics, contact constraints and numerical integration.','https://mujoco.readthedocs.io/en/stable/computation/'),(3,'MuJoCo API: body velocities, applied forces and simulation state.','https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html')]:
        b.append(('reference',f'[{number}] {label} Accessed 12 November 2026.',url))
    setup_fonts()
    build_pdf(HERE/'output/pdf/mujoco_boundary_payload_report.pdf',title,b,manifest,'Boundary and loaded-control supplement | 12 November 2026')
    build_tex(HERE/'output/latex/mujoco_boundary_payload_report.tex',title,b,manifest)
    (OUT/'report_content.json').write_text(json.dumps(b,indent=2)+'\n')
    parts=[]
    for block in b:
        if block[0]=='heading':parts.append('<h2>'+html.escape(block[1])+'</h2>')
        elif block[0]=='paragraph':parts.append('<p>'+html.escape(block[1])+'</p>')
        elif block[0]=='figure':
            name=block[1];parts.append('<figure><img src="../../publication/figures/'+name+'.svg" alt="'+html.escape(manifest[name]['caption'],quote=True)+'"><figcaption>'+html.escape(manifest[name]['caption'])+'</figcaption></figure>')
        elif block[0]=='table':parts.append('<div class="table-wrap"><table><caption>'+html.escape(block[1])+'</caption><tr>'+''.join('<th>'+html.escape(str(x))+'</th>' for x in block[2])+'</tr>'+''.join('<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in row)+'</tr>' for row in block[3])+'</table></div>')
        elif block[0]=='reference':parts.append('<p>'+html.escape(block[1])+' <a href="'+block[2]+'">Source</a></p>')
    css=(ROOT/'results/mujoco/report.html').read_text(encoding='utf-8').split('<style>')[1].split('</style>')[0]
    (OUT/'report.html').write_text('<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+title+'</title><style>'+css+'</style></head><body><main><nav><a href="../../publication/output/pdf/mujoco_boundary_payload_report.pdf">PDF report</a><a href="../mujoco_boundary/study.json">Coarse boundary data</a><a href="../mujoco_boundary_zoom/study.json">Boundary zoom data</a><a href="../mujoco_payload/study.json">Payload data</a></nav><p>MICHAEL AGBO / BOUNDARY AND PAYLOAD COMPARISON</p><h1>'+title+'</h1>'+''.join(parts)+'</main></body></html>',encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k not in ['boundary_diagnostic','payload_fall_effects']},indent=2))


if __name__=='__main__':main()
