"""Scientific figures and journal-neutral MuJoCo verification supplement."""
from collections import Counter
import hashlib
import html
import json
import math
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT/'results/mujoco'
FIG = HERE/'figures'
sys.path.insert(0, str(HERE))
from build_reports import setup_fonts, build_pdf, build_tex

COLORS = {'planar': '#c45b21', 'reduced': '#7561aa', 'native': '#087d88'}
LABELS = {'planar': 'Original planar', 'reduced': 'MuJoCo ideal rolling', 'native': 'MuJoCo wheel contact'}
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9, 'axes.titlesize': 10,
                     'legend.fontsize': 8, 'pdf.fonttype': 42, 'svg.fonttype': 'none',
                     'axes.spines.top': False, 'axes.spines.right': False, 'grid.alpha': .2})


def main():
    data = json.loads((OUT/'study.json').read_text())
    for name, digest in data['metadata']['source_sha256'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == digest, name
    rows = data['records']
    assert len(rows) == 150
    arrays = np.load(OUT/'trajectories.npz')
    protocol = data['metadata']['protocol']
    lookup = {r['id']: r for r in rows}
    pairs = {r['id']: r for r in data['pairs']}
    def select(**kwargs):
        return [r for r in rows if all(r[k] == v for k, v in kwargs.items())]
    def nominal(step=.00025):
        return select(engine='native', variant='nominal', physics_step_s=step)
    def contact_residual(record):
        values = arrays[record['id']]
        return float(np.max(np.abs(values[values[:, 13] > 0, 9:11])))
    fine = nominal()
    reduced = select(engine='reduced')
    native_pass = sum(pairs[r['id']]['passes'] for r in fine)
    agreements = sum(pairs[r['id']]['same_outcome'] for r in fine)
    reduced_max = np.max([pairs[r['id']]['max_state_errors_native_units'] for r in reduced], axis=0)
    native_max_x = max(pairs[r['id']]['max_position_error_m'] for r in fine)
    native_max_pitch = max(pairs[r['id']]['max_pitch_error_deg'] for r in fine)
    native_ref = [r for r in data['refinement'] if r['engine']=='native' and r['fine_step_s']==.00025]
    native_ref_changed = sum(not r['same_outcome'] for r in native_ref)
    manifest = {}
    def save(fig, name, caption):
        for ext in ['png', 'svg', 'pdf']:
            target = FIG/f'{name}.{ext}'
            if '--reuse-figures' not in sys.argv or not target.exists():
                fig.savefig(target, dpi=400, bbox_inches='tight', pad_inches=.12)
        plt.close(fig)
        manifest[name] = {'name': name, 'caption': caption}

    # Each unit gets its own axis; residuals are numerical verification, not uncertainty.
    fig, axs = plt.subplots(2, 2, figsize=(7.2, 4.7), layout='constrained')
    for j, (ax, label) in enumerate(zip(axs.flat, ['Position error (m)', 'Velocity error (m/s)', 'Pitch error (rad)', 'Pitch-rate error (rad/s)'])):
        values = [pairs[r['id']]['max_state_errors_native_units'][j] for r in reduced]
        ax.semilogy(range(1, 16), np.maximum(values, 1e-17), 'o', color=COLORS['reduced'], ms=4)
        ax.axhline(1e-7, color='#8a4a38', ls='--', label='Declared tolerance')
        ax.set(xlabel='Condition number', ylabel=label, xticks=[1, 5, 10, 15], ylim=(3e-18, 1e-6))
        ax.grid()
    axs[0, 0].legend()
    save(fig, 'mujoco_fig01_equivalence', 'Maximum trajectory residuals between the independently assembled MuJoCo ideal-rolling model and the original equations. Both use RK4 with 1 ms physics steps and 5 ms held controls. Conditions are ordered by increasing initial tilt (0, 5, 15 deg), then push (-15, -3, 0, 3, 15 N). Each coordinate is compared in its stated units over the shared pre-fall horizon. Exact zeros are displayed at 1e-17 for logarithmic plotting. This checks implementation equivalence of the same assumptions, not physical accuracy.')

    fig, axs = plt.subplots(3, 1, figsize=(7.2, 5.4), layout='constrained')
    example = protocol['example']
    for kind, step, style in [('planar', .00025, '-'), ('reduced', .001, ':'), ('native', .00025, '--')]:
        r = select(engine=kind, physics_step_s=step, initial_tilt_deg=example['initial_tilt_deg'], push_force_n=example['push_force_n'])[0]
        a = arrays[r['id']]
        for ax, col, factor in zip(axs, [3, 1, 5], [180/math.pi, 1, 1]):
            if col == 5:
                ax.step(a[:, 0], a[:, col]*factor, where='post', color=COLORS[kind], ls=style, label=LABELS[kind], lw=1.3)
            else:
                ax.plot(a[:, 0], a[:, col]*factor, color=COLORS[kind], ls=style, label=LABELS[kind], lw=1.3)
    for ax, label in zip(axs, ['Pitch (deg)', 'Position (m)', 'Total torque (N m)']):
        ax.axvspan(2, 2.2, alpha=.18, color='#d1a21b')
        ax.set(xlabel='Time (s)', ylabel=label)
        ax.grid()
    axs[0].legend(ncol=3, fontsize=7, loc='upper right')
    save(fig, 'mujoco_fig02_trajectory', 'Protocol-selected trajectory: initial tilt 5 deg, +3 N COM force from 2.00 to 2.20 s, and total torque limit 0.70 N m. All models use the original fixed LQR gain and 200 Hz ideal feedback. Original and native models use 0.25 ms physics steps; the verified reduced model uses 1 ms RK4. The reduced and original curves may overlap. Torque is the command held over the following interval; the terminal zero is a plotting placeholder.')

    fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.3), layout='constrained')
    for ax, metric, label in zip(axs, ['max_position_error_m', 'max_pitch_error_deg'], ['Maximum position difference (mm)', 'Maximum pitch difference (deg)']):
        factor = 1000 if metric == 'max_position_error_m' else 1
        matrix = factor*np.array([pairs[r['id']][metric] for r in fine]).reshape(3, 5)
        im = ax.imshow(matrix, origin='lower', aspect='auto', cmap='YlOrBr')
        for i, r in enumerate(fine):
            y, x = divmod(i, 5)
            ax.text(x, y, f'{matrix[y,x]:.2g}', ha='center', va='center', fontsize=7, color='white' if matrix[y,x] > matrix.max()*.65 else '#182e40')
        ax.set(xticks=range(5), xticklabels=protocol['push_forces_n'], yticks=range(3), yticklabels=protocol['initial_tilts_deg'], xlabel='Scheduled push (N)', ylabel='Initial tilt (deg)', title=label)
        fig.colorbar(im, ax=ax, shrink=.7)
    save(fig, 'mujoco_fig03_discrepancy', 'Maximum absolute native-contact minus original-planar trajectory differences at 0.25 ms physics resolution. Each cell is one deterministic condition, not a probability or interpolated stability region. Differences stop at the earlier fall endpoint. The full record includes the common horizon and both outcome labels; a short-horizon fall is not treated as a successful agreement. The native model retains its additional chassis inertia, contact compliance and slip.')

    fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.7), layout='constrained')
    for ax, metric, label in zip(axs, ['max_position_error_m', 'max_pitch_error_deg'], ['Position difference (m)', 'Pitch difference (deg)']):
        for kind in ['planar', 'native']:
            for index in range(15):
                sets = [[r for r in data['refinement'] if r['engine']==kind and r['coarse_step_s']==h][index] for h in [.001, .0005]]
                ax.semilogy([0, 1], np.maximum([r[metric] for r in sets], 1e-17), 'o-', alpha=.4, ms=3, lw=.8, color=COLORS[kind], label=LABELS[kind] if index==0 else None)
        ax.set(xticks=[0, 1], xticklabels=['1 vs 0.5 ms', '0.5 vs 0.25 ms'], xlabel='Physics-step comparison', ylabel=label)
        ax.grid()
    axs[0].legend(fontsize=7)
    save(fig, 'mujoco_fig04_refinement', 'Maximum trajectory changes when halving physics steps, with the control interval and fall-detection interval fixed at 5 ms. Each line is a condition, with original RK4 in orange and native implicitfast in teal. Shared pre-fall horizons are used. Zeros are placed at 1e-17 for display. A downward line indicates a smaller successive-step discrepancy; contact transitions can prevent monotonic convergence. These are empirical resolution checks, not formal error bounds.')

    example_native = select(engine='native', variant='nominal', physics_step_s=.00025, initial_tilt_deg=5, push_force_n=3)[0]
    a = arrays[example_native['id']]
    fig, axs = plt.subplots(3, 1, figsize=(7.2, 5.1), layout='constrained')
    for j, label, style in [(9, 'Left wheel', '-'), (10, 'Right wheel', '--')]:
        axs[0].plot(a[:, 0], a[:, j], color=COLORS['native'] if j==9 else '#695493', ls=style, label=label)
    axs[0].legend()
    axs[1].plot(a[:, 0], 1000*(a[:, 8]-.045), color=COLORS['native'])
    axs[2].plot(a[:, 0], a[:, 7]*180/math.pi, color=COLORS['native'])
    for ax, label in zip(axs, ['Rolling residual (m/s)', 'Axle height - radius (mm)', 'Roll (deg)']):
        ax.axvspan(2, 2.2, color='#d1a21b', alpha=.18)
        ax.set(xlabel='Time (s)', ylabel=label)
        ax.grid()
    axs[0].set_xlim(-.003, .08)
    axs[1].set_xlim(-.003, .08)
    axs[0].set_title('Initial wheel/contact engagement')
    axs[2].set_ylim(-1e-6, 1e-6)
    axs[2].text(.03, .88, f'Peak |roll| = {np.max(np.abs(a[:,7]))*180/math.pi:.2g} deg', transform=axs[2].transAxes, fontsize=8)
    save(fig, 'mujoco_fig05_contact', 'Preselected 5 deg, +3 N example at 0.25 ms. The upper panels show the first 80 ms; roll shows the full run on a fixed scale, without amplifying round-off fluctuations. Rolling residual is wheel-centre world X velocity minus radius times absolute world Y angular velocity. This nearly planar diagnostic includes contact-free wheel spin and is not general 3D contact-point slip. Telemetry is sampled every 5 ms. The axle starts 0.2 mm above the radius with no warm-up, so the transients include contact engagement. Roll is not controlled.')

    variants = [v['name'] for v in protocol['native_contact_variants']]
    short_names = ['Nominal', 'Friction 0.5', 'Friction 1.5', 'Contact 10 ms']
    fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.8), layout='constrained')
    for i, (name, color) in enumerate(zip(['recovered', 'unsettled', 'fallen'], ['#087d88', '#d0a333', '#c45b21'])):
        counts = [Counter(r['outcome'] for r in select(engine='native', variant=v, physics_step_s=.0005)) for v in variants]
        bottom = [sum(c[k] for k in ['recovered', 'unsettled', 'fallen'][:i]) for c in counts]
        axs[0].bar(range(4), [c[name] for c in counts], bottom=bottom, color=color, label=name.capitalize())
    axs[0].set(ylabel='Conditions (n = 15 each)', ylim=(0, 18), yticks=[0, 5, 10, 15])
    axs[0].legend(fontsize=7, ncol=3, loc='upper center')
    for j, variant in enumerate(variants):
        r = select(engine='native', variant=variant, physics_step_s=.0005)
        vals = [contact_residual(p) for p in r]
        axs[1].scatter(np.full(15, j)+np.linspace(-.16, .16, 15), vals, s=16, color='#087d88', alpha=.65)
        axs[1].plot([j-.22, j+.22], [np.median(vals)]*2, color='#162f43', lw=2)
    axs[1].set(ylabel='Peak residual with contact (m/s)')
    for ax in axs:
        ax.set(xticks=range(4), xticklabels=short_names)
        ax.tick_params(axis='x', labelrotation=25)
        ax.grid(axis='y')
    save(fig, 'mujoco_fig06_sensitivity', 'Contact sensitivity at 0.5 ms. Nominal sliding friction is 1.0 and contact time constant is 5 ms; each variant changes one parameter. Left: outcomes over the same 15 conditions. Right: peak longitudinal rolling residual per trial, using only 5 ms samples with nonzero total contact count. This supplementary diagnostic excludes contact-free wheel spin but is not a general contact-point slip measurement. Every condition contributes; horizontal offsets distinguish conditions and black bars show medians. These are deterministic inventories, not confidence intervals.')

    b = []
    def h(text): b.append(('heading', text))
    def p(text): b.append(('paragraph', text))
    def page(title, figure=None):
        b.append(('pagebreak',)); h(title)
        if figure: b.append(('figure', figure))
    title = 'MuJoCo verification and wheel-contact discrepancy in a balancing robot'
    h('Scope and principal findings')
    p('This supplement adds an independently assembled MuJoCo model to the existing nominal, unloaded balancing-robot study. It distinguishes equation verification under ideal rolling from the discrepancy introduced by a free-body robot with wheel-ground contact. No controller tuning or parameter fitting is performed. The study is deterministic and simulation-only; it does not validate the robot against hardware.')
    p(f"All {len(reduced)} ideal-rolling trajectory checks pass the declared componentwise tolerance of 1e-7 in each coordinate's native units. A separate 48-condition acceleration check has maximum errors {data['equation_checks']['max_acceleration_error_m_s2']:.3g} m/s² and {data['equation_checks']['max_angular_acceleration_error_rad_s2']:.3g} rad/s². The independently compiled multibody model therefore reproduces the stated equations to numerical precision for the evaluated cases.")
    p(f'At the finest native-contact resolution (0.25 ms), {agreements}/15 outcome labels match the original planar model and {native_pass}/15 conditions pass the declared full-horizon discrepancy screen. Across common pre-fall horizons, maximum position and pitch differences are {native_max_x:.4g} m and {native_max_pitch:.4g} deg. These differences are model-discrepancy measurements, not measured physical errors.')
    counts_table = []
    for kind, step in [('planar', .00025), ('reduced', .001), ('native', .00025)]:
        selected = select(engine=kind, physics_step_s=step)
        c = Counter(r['outcome'] for r in selected)
        counts_table.append([LABELS[kind], c['recovered'], c['unsettled'], c['fallen'], len(selected)])
    b.append(('table', 'Nominal model outcome inventory', ['Model', 'Recovered', 'Unsettled', 'Fallen', 'Total'], counts_table, [.4, .17, .17, .13, .13]))
    p('The 150 integrations comprise 45 original-planar runs, 15 reduced MuJoCo runs, 45 nominal native-contact runs and 45 additional contact-sensitivity runs. Repeated step sizes and contact variants are numerical/configuration repetitions of 15 conditions, not independent experimental samples. The original manuscript and payload/noise supplements are preserved.')
    p('All 15 nominal conditions recover in both models. This grid therefore checks agreement within the tested recovery region; it does not test agreement at a fall boundary or validate prediction of failure. Boundary-focused conditions must be evaluated separately.')

    page('1 Matched protocol and model construction')
    p('The full Cartesian grid combines initial tilt 0, 5 and 15 deg with horizontal COM pushes -15, -3, 0, 3 and 15 N. Each nonzero force is held from 2.00 to 2.20 s. The maximum duration is 10 s. Gains are copied from config.json without redesign, ideal state feedback updates every 5 ms, and total two-motor torque is clipped to +/-0.70 N m.')
    b.append(('table', 'Shared nominal parameters', ['Quantity', 'Value', 'Interpretation'], [
        ['Body mass / base mass', '1.2 / 0.35 kg', 'Base mass includes both wheels'],
        ['Body COM above axle', '0.18 m', 'Push acts at body COM'],
        ['Body pitch inertia', '0.012 kg m²', 'About body COM'],
        ['Wheel radius / spin inertia', '0.045 m / 0.00012 kg m²', 'Inertia is per wheel'],
        ['Translation / rotation drag', '0.08 / 0.002', 'N s/m and N m s/rad'],
        ['Gravity / total mass', '9.81 m/s² / 1.55 kg', 'Same in each model']], [.37, .29, .34]))
    p('The reduced MuJoCo assembly has a base X slide and a torso Y hinge. The slide carries a reflected wheel-spin mass 2 Iw/r² through its armature. A common motor command produces generalized forces [u/r, -u]; a horizontal force on the torso COM supplies both translation and pitch moment. Joint damping implements the original drag terms. This uses MuJoCo rigid-body dynamics to check the original implementation, while deliberately retaining its assumptions.')
    p('The native model instead has a floating base and two wheel hinges. Its base mass is split into 0.11 kg chassis and two 0.12 kg wheels. It adds chassis pitch inertia 0.0002 kg m², uncalibrated roll/yaw inertias and 0.23 m wheel-centre spacing. Newton contact solving, an elliptic cone, condim 4, sliding friction 1.0, torsional friction 0.005 and solref [0.005, 1] define the nominal contact model. The rolling-friction entry is inactive with condim 4. These are explicit assumptions, not identified material properties [1,2].')
    p('The native axle starts at z = 0.0452 m with zero velocity and no warm-up. World X is forward and positive pitch is forward lean. Both wheel motors receive u/2. Native drag acts as a world-X force on the base and world-Y torque on the torso. Its state-dependent drag is refreshed each physics step. The launcher smoke test is not reused as research data.')

    page('2 Equation and ideal-rolling trajectory verification', 'mujoco_fig01_equivalence')
    p(f'The acceleration test spans 3 angles, 2 translational speeds, 2 pitch rates, 2 motor torques and 2 push forces (48 combinations). Motor generalized-force signs and scaling are checked separately. The largest componentwise trajectory errors are [{", ".join(f"{v:.3g}" for v in reduced_max)}] in [m, m/s, rad, rad/s]. Every reduced/original pair has the same outcome.')
    p('Independent assembly helps detect algebra, inertia, force-location and actuator-convention errors. It cannot validate omitted physics: both ideal-rolling implementations intentionally share the same mechanical assumptions and parameter values. Passing this check supports numerical implementation correctness within this scope.')

    page('3 Preselected closed-loop example', 'mujoco_fig02_trajectory')
    p('The original and reduced models use RK4. The native model uses implicitfast with compliant contacts. All controller updates remain at 200 Hz when physics resolution changes. A force pulse is indexed by integer control ticks, avoiding floating-point ambiguity at its start and end. A completed nonzero-push run contains exactly 40 forced control intervals.')

    page('4 Cross-model discrepancy and outcome agreement', 'mujoco_fig03_discrepancy')
    p('Before executing the grid, the engineering screen was set to maximum position difference <=0.01 m, maximum pitch difference <=0.5 deg, matching three-way outcomes, and a full shared 10 s horizon. These limits are practical study criteria; they are not established physical-accuracy standards. Every planned condition remains in the denominator, including any fall before the scheduled push.')
    p(f'{native_pass}/15 finest-resolution nominal contact cases pass that screen; {15-native_pass} do not. Outcome agreement alone ({agreements}/15) does not imply trajectory equivalence. A failed screen is retained as a result, without retuning gains or fitting friction after seeing the output. For any early fall, errors are explicitly restricted to the common observed horizon.')
    p('Fall detection occurs at 5 ms endpoints when absolute pitch reaches 45 deg; the native model also checks roll. A surviving run is recovered only if its final continuous dwell of at least 1 s satisfies |pitch| <=1 deg, |x| <=0.03 m and |vx| <=0.03 m/s, after the pulse ends when nonzero. These rules reproduce the existing planar study. Native lateral drift and yaw are monitored but are not recovery criteria, so this label does not certify general 3D pose recovery.')

    page('5 Physics-step sensitivity', 'mujoco_fig04_refinement')
    p(f'Halving native physics steps from 0.5 to 0.25 ms changes {native_ref_changed}/15 outcome labels. The largest shared-horizon changes are {max(r["max_position_error_m"] for r in native_ref):.4g} m and {max(r["max_pitch_error_deg"] for r in native_ref):.4g} deg. The 1-to-0.5 ms comparison and both original RK4 refinements are also archived. Fixed control timing separates physics-step effects from changes in feedback bandwidth.')
    p('Contact switching and saturation may amplify small trajectory changes. The smallest tested step is a numerical reference for this study, not an exact solution. The analysis does not infer that smaller integration error eliminates model discrepancy or that a contact parameter is physically correct because it agrees with an ideal-rolling model.')

    page('6 Contact, slip and out-of-plane diagnostics', 'mujoco_fig05_contact')
    p(f'Across the 15 nominal native runs at 0.25 ms, the largest sampled longitudinal rolling residual is {max(r["max_sampled_slip_m_s"] for r in fine):.4g} m/s, including contact-free wheel spin. Restricting samples to nonzero total contact count gives {max(contact_residual(r) for r in fine):.4g} m/s. Maximum sampled roll is {max(r["max_sampled_roll_deg"] for r in fine):.4g} deg and lateral travel is {max(r["max_sampled_y_m"] for r in fine):.4g} m. These are 200 Hz sampled maxima, not substep bounds. World-frame wheel velocities use the MuJoCo object-velocity API [3].')
    p('This grid excites sagittal balancing with symmetric wheel commands. Small roll or lateral motion in these trials is expected from symmetry and does not demonstrate lateral robustness. Off-axis forces, unequal wheel torque, uneven ground and payloads require additional experiments. The ideal model has no normal contact compliance, vertical motion or wheel slip with which to compare these channels.')

    page('7 Contact-parameter sensitivity', 'mujoco_fig06_sensitivity')
    sensitivity = []
    baseline = {(r['initial_tilt_deg'], r['push_force_n']): r for r in nominal(.0005)}
    for variant, label in zip(variants, short_names):
        selected = select(engine='native', variant=variant, physics_step_s=.0005)
        changed = sum(r['outcome'] != baseline[r['initial_tilt_deg'], r['push_force_n']]['outcome'] for r in selected)
        sensitivity.append([label, changed, sum(pairs[r['id']]['passes'] for r in selected), f'{max(pairs[r["id"]]["max_position_error_m"] for r in selected):.4g}', f'{max(pairs[r["id"]]["max_pitch_error_deg"] for r in selected):.4g}'])
    b.append(('table', 'At 0.5 ms: label changes vs nominal; trajectory differences vs planar', ['Variant', 'Changed labels', 'Screen passes /15', 'Max position diff. (m)', 'Max pitch diff. (deg)'], sensitivity, [.23, .18, .18, .21, .20]))
    p('Only one contact parameter changes in each variant. Interaction effects, solver-tolerance sensitivity, alternative geometries, startup warm-up and mass-property uncertainty remain untested. No variant is selected as the physical truth or promoted as an improved controller.')

    page('8 Reproducibility, interpretation and next experiments')
    p(f"Software: MuJoCo {data['metadata']['mujoco_version']}, NumPy {data['metadata']['numpy_version']}, Python {data['metadata']['python']}, {data['metadata']['platform']}. validation_protocol.json fixes the grid and criteria. study.json contains settings, per-trial metrics, pairwise errors, step refinements and source SHA-256 hashes. trajectories.npz retains all 150 sampled trajectories with explicit column units; report_content.json and the LaTeX source preserve editable prose.")
    p('Reproduction from the project root: run mujoco_lab/.venv/Scripts/python.exe mujoco_lab/validate.py, followed by publication/.venv/Scripts/python.exe publication/build_mujoco_report.py. The second command verifies the recorded source hashes before building figures and reports. The MuJoCo and publication environments stay separate. The archive excludes virtual environments and transient viewer state.')
    p('The results support a bounded statement: the original nominal equations pass an independent MuJoCo assembly check on the declared cases, and a specific free-body/contact realization has a measured discrepancy under the same fixed controller. They do not support hardware validation, a general stability guarantee, a full 3D controller claim, payload robustness, or a new control-method contribution. No measurements or experimental results have been invented.')
    p('Next, extend the simulation comparison to prespecified conditions near the fall boundary. Identify wheel radius, masses, COM, pitch inertia and torque limits from a physical design or robot; characterize motor response and wheel-floor traction; then freeze a held-out validation protocol. Add explicit chassis-inertia and startup-contact ablations before attributing any native/planar difference solely to slip. After the nominal mapping is understood, repeat the payload-aware comparison with matched loaded mass properties. Keep calibration cases separate from validation cases and report disagreements.')
    h('References')
    for number, label, url in [(1, 'MuJoCo XML Reference: joint armature, actuators and contact dimensions.', 'https://mujoco.readthedocs.io/en/stable/XMLreference.html'),
                               (2, 'MuJoCo Computation: rigid-body dynamics, contact constraints and integrators.', 'https://mujoco.readthedocs.io/en/stable/computation/'),
                               (3, 'MuJoCo API Functions: mj_objectVelocity.', 'https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html')]:
        b.append(('reference', f'[{number}] {label} Accessed 12 November 2026.', url))

    setup_fonts()
    build_pdf(HERE/'output/pdf/mujoco_report.pdf', title, b, manifest, 'Independent implementation check and native-contact supplement | 12 November 2026')
    build_tex(HERE/'output/latex/mujoco_report.tex', title, b, manifest)
    (OUT/'report_content.json').write_text(json.dumps(b, indent=2)+'\n')
    (HERE/'mujoco_figure_manifest.json').write_text(json.dumps(list(manifest.values()), indent=2)+'\n')
    summary = {'integrations': len(rows), 'equation_checks': data['equation_checks'], 'reduced_trajectory_passes': len(reduced),
               'native_fine_outcome_agreement': agreements, 'native_fine_screen_passes': native_pass,
               'native_fine_max_position_error_m': native_max_x, 'native_fine_max_pitch_error_deg': native_max_pitch,
               'native_final_refinement_changed_outcomes': native_ref_changed, 'sensitivity_rows': sensitivity}
    (OUT/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    parts = []
    for block in b:
        if block[0] == 'heading': parts.append('<h2>'+html.escape(block[1])+'</h2>')
        elif block[0] == 'paragraph': parts.append('<p>'+html.escape(block[1])+'</p>')
        elif block[0] == 'figure':
            name = block[1]
            parts.append('<figure><img src="../../publication/figures/'+name+'.svg" alt="'+html.escape(manifest[name]['caption'], quote=True)+'"><figcaption>'+html.escape(manifest[name]['caption'])+'</figcaption></figure>')
        elif block[0] == 'table':
            parts.append('<div class="table-wrap"><table><caption>'+html.escape(block[1])+'</caption><tr>'+''.join('<th>'+html.escape(str(x))+'</th>' for x in block[2])+'</tr>'+''.join('<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in row)+'</tr>' for row in block[3])+'</table></div>')
        elif block[0] == 'reference': parts.append('<p>'+html.escape(block[1])+' <a href="'+block[2]+'">Source</a></p>')
    css = (ROOT/'results/comparison/report.html').read_text(encoding='utf-8').split('<style>')[1].split('</style>')[0]
    (OUT/'report.html').write_text('<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+title+'</title><style>'+css+'</style></head><body><main><nav><a href="../../publication/output/pdf/mujoco_report.pdf">PDF report</a><a href="study.json">Results and provenance</a><a href="trajectories.npz">All trajectories</a></nav><p>MICHAEL AGBO / MUJOCO VERIFICATION</p><h1>'+title+'</h1>'+''.join(parts)+'</main></body></html>', encoding='utf-8')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
