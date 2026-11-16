"""Follow-up supplement: why two boundary cells disagree, and whether the noisy
payload-aware fall reduction survives native wheel contact.

Reads results/mujoco_disagreement and results/mujoco_noisy_payload (read-only),
writes five figures, results/mujoco_followup/summary.json and the supplement.
"""
from collections import Counter, defaultdict
import hashlib
import html
import itertools
import json
import math
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import FancyBboxPatch, Patch
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
OUT = ROOT/'results/mujoco_followup'
FIG = HERE/'figures'
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))
from build_reports import setup_fonts, build_pdf, build_tex
from balance import Parameters, derivative

# Categorical slots 1-3 of the validated reference palette (pass all-pairs on a light surface).
BLUE, ORANGE, AQUA, YELLOW = '#2a78d6', '#eb6834', '#1baf7a', '#eda100'
INK, INK2, MUTED, GRID, QUIET = '#0b0b0b', '#52514e', '#52514e', '#e1e0d9', '#a3a29b'
# Plants are drawn in dark grey and green so that orange and blue always mean the two controllers.
PLANT_C = {'planar': INK2, 'native': AQUA}
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 8.5, 'axes.titlesize': 9.5, 'axes.titleweight': 'bold',
                     'axes.labelsize': 8.5, 'legend.fontsize': 8, 'legend.frameon': False, 'pdf.fonttype': 42, 'svg.fonttype': 'none',
                     'axes.spines.top': False, 'axes.spines.right': False, 'axes.edgecolor': '#b9b8b2', 'axes.linewidth': .6,
                     'xtick.color': MUTED, 'ytick.color': MUTED, 'xtick.labelcolor': MUTED, 'ytick.labelcolor': MUTED,
                     'axes.labelcolor': INK, 'text.color': INK, 'axes.titlecolor': INK, 'grid.color': GRID, 'grid.linewidth': .6,
                     'grid.linestyle': '-', 'lines.linewidth': 1.6, 'lines.solid_capstyle': 'round', 'axes.axisbelow': True})
ENGINE_NAMES = {'archived': 'Planar\n(archived)', 'planar_port': 'Planar\n(port)', 'native_0p25ms': 'Contact\n0.25 ms', 'native_0p5ms': 'Contact\n0.5 ms'}
PROFILE_NAMES = {'direct_0ms': 'Direct', 'kalman_0ms': 'Kalman', 'kalman_10ms': 'Kalman, 10 ms delay'}
SCENARIO_NAMES = {'opposing_push': '0.6 N m limit', 'restricted_torque': '0.3 N m limit'}
GROUPS = [('reference', 'Reference models'), ('inertia', 'Chassis pitch inertia'), ('settling', 'Start-up and settling'),
          ('contact', 'Contact and friction'), ('combined', 'Combined')]
CODE = {'recovered': 'R', 'unsettled': 'U', 'fallen': 'F'}


def load(name):
    folder = ROOT/f'results/{name}'
    data = json.loads((folder/'study.json').read_text())
    for f, digest in data['metadata']['source_sha256'].items():
        assert hashlib.sha256((ROOT/f).read_bytes()).hexdigest() == digest, f'{f} changed since {name} was run'
    return data


def rounded_bars(ax, xs, heights, width, colour, horizontal=False):
    """Bars with a 3 pt rounded data end and a square baseline (the baseline corners are clipped at zero)."""
    fig = ax.figure
    fig.canvas.draw()
    box = ax.get_window_extent()
    (x0, x1), (y0, y1) = ax.get_xlim(), ax.get_ylim()
    xu, yu = abs(x1-x0)/(box.width/fig.dpi*72), abs(y1-y0)/(box.height/fig.dpi*72)
    for x, h in zip(xs, heights):
        if h <= 0: continue
        if horizontal:
            radius = min(3*xu, (width/2)*xu/yu, h/2)
            patch = FancyBboxPatch((-2*radius, x-width/2), h+2*radius, width, boxstyle=f'round,pad=0,rounding_size={radius}', mutation_aspect=yu/xu, fc=colour, ec='none', clip_on=True)
        else:
            radius = min(3*xu, width/2)
            patch = FancyBboxPatch((x-width/2, -2*radius*yu/xu), width, h+2*radius*yu/xu, boxstyle=f'round,pad=0,rounding_size={radius}', mutation_aspect=yu/xu, fc=colour, ec='none', clip_on=True)
        ax.add_patch(patch)


def display_label(label):
    return (label.replace('Native contact, as archived', 'Contact model, as archived').replace('Native, ', 'Contact, ')
            .replace('kg m2', 'kg m²').replace('reduced by 0.0002', 'reduced by 0.0002 kg m²')
            .replace('native pulse-onset state injected', "contact model's pulse-onset state injected"))


def sign_power(n, p_prevent, p_introduce, alpha=.05):
    """Exact power of the two-sided sign test on discordant pairs among n paired trials."""
    discordant = p_prevent+p_introduce
    if discordant <= 0: return 0.0
    share = p_prevent/discordant
    power = 0.0
    for d in range(1, n+1):
        weight = math.comb(n, d)*discordant**d*(1-discordant)**(n-d)
        if weight < 1e-14: continue
        pmf = [math.comb(d, k)/2**d for k in range(d+1)]
        lower, upper = list(itertools.accumulate(pmf)), list(itertools.accumulate(pmf[::-1]))[::-1]
        reject = [k for k in range(d+1) if min(1.0, 2*min(lower[k], upper[k])) <= alpha]
        power += weight*sum(math.comb(d, k)*share**k*(1-share)**(d-k) for k in reject)
    return power


def pairs_needed(p_prevent, p_introduce, target=.9, ceiling=400):
    for n in range(4, ceiling+1):
        if sign_power(n, p_prevent, p_introduce) >= target: return n
    return None


def staircase_sd(trials, scatter, step, discard=6, repeats=4000, seed=20261114):
    """Standard deviation of a one-up one-down staircase estimate of a fall boundary, as a fraction of it.

    The commanded push starts one step below the boundary and moves down one step after a fall and up one
    after a recovery; the realised push is the commanded one times (1 + scatter * N(0, 1)). The estimate is
    the mean commanded push over the trials after the first `discard`."""
    rng = np.random.default_rng(seed)
    level = np.full(repeats, 1.0-step)
    kept = []
    for t in range(trials):
        fell = level*(1+scatter*rng.standard_normal(repeats)) > 1.0
        if t >= discard: kept.append(level.copy())
        level = level+np.where(fell, -step, step)
    return float(np.std(np.mean(kept, axis=0)))


def spearman(x, y):
    def ranks(v):
        v = np.asarray(v, float); order = v.argsort(); r = np.empty(len(v))
        r[order] = np.arange(len(v))
        for value in np.unique(v): r[v == value] = r[v == value].mean()
        return r
    return float(np.corrcoef(ranks(x), ranks(y))[0, 1])


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    dis, noisy = load('mujoco_disagreement'), load('mujoco_noisy_payload')
    arrays = np.load(ROOT/'results/mujoco_disagreement/trajectories.npz')
    examples = np.load(ROOT/'results/mujoco_noisy_payload/examples.npz')
    protocol = dis['metadata']['protocol']
    step, fine = protocol['physics_step_s'], protocol['confirmation_step_s']
    manifest = {}

    def save(fig, name, caption):
        for ext in ['png', 'svg', 'pdf']:
            fig.savefig(FIG/f'{name}.{ext}', dpi=600, bbox_inches='tight', pad_inches=.1, facecolor='white')
        plt.close(fig); manifest[name] = {'name': name, 'caption': caption}

    # ---------------------------------------------------------------- disagreement diagnostics
    brackets = {b['variant']: b for b in dis['brackets']}
    labels = {(r['variant'], r['physics_step_s'], r['push_force_n']): r for r in dis['labels']}
    planar_mid, native_mid = dis['planar_midpoint_n'], dis['native_midpoint_n']
    gap = planar_mid-native_mid
    width = max(b['width_n'] for b in dis['brackets'] if b['transition_in_bracket'])
    variants = []
    for name, b in brackets.items():
        tags = {f'{"plus" if s > 0 else "minus"}_{"fine" if h == fine else "base"}': labels[name, h, s*18.2]['outcome'] for h in [step, fine] for s in [1, -1]}
        at_cell = labels[name, fine, 18.2]
        row = dict(name=name, label=display_label(b['label']), group=b['group'], engine=b['engine'], transition_in_bracket=b['transition_in_bracket'], **tags,
                   fall_time_s=at_cell['fall_time_s'], pulse_friction_use=at_cell['pulse_max_friction_utilisation'], pulse_slip_m_s=at_cell['pulse_max_slip_m_s'])
        if b['transition_in_bracket']:
            row.update(midpoint_n=b['midpoint_n'], last_not_fallen_n=b['last_not_fallen_n'], first_fallen_n=b['first_fallen_n'],
                       shift_from_planar_n=b['shift_from_planar_n'], shift_from_native_n=b['shift_from_native_n'],
                       fraction_of_gap_closed=b.get('fraction_of_gap_closed'))
        row['mirror_symmetric'] = tags['plus_fine'] == tags['minus_fine'] and tags['plus_base'] == tags['minus_base']
        row['step_consistent'] = tags['plus_fine'] == tags['plus_base'] and tags['minus_fine'] == tags['minus_base']
        variants.append(row)
    by_name = {v['name']: v for v in variants}
    addendum = json.loads((ROOT/'results/mujoco_disagreement/addendum.json').read_text())
    for f, digest in addendum['metadata']['source_sha256'].items():
        assert hashlib.sha256((ROOT/f).read_bytes()).hexdigest() == digest, f'{f} changed since the addendum was run'
    other = [{k: v for k, v in b.items() if k != 'checks'} for b in addendum['boundaries']]
    restores =[v['name'] for v in variants if v['engine'] == 'native' and v['name'] != 'native' and v['plus_fine'] != 'fallen' and v['minus_fine'] != 'fallen']
    planar_flips = [v['name'] for v in variants if v['engine'] == 'planar' and v['name'] != 'planar' and v['plus_fine'] == 'fallen' and v['minus_fine'] == 'fallen']

    # Figure 1: critical force by variant (chart plus the numbers it is drawn from).
    ordered = [(title, [v for v in variants if v['group'] == key]) for key, title in GROUPS]
    count = sum(len(rows)+1 for _, rows in ordered)
    fig, ax = plt.subplots(figsize=(7.3, .235*count+.9))
    fig.subplots_adjust(left=.40, right=.74, top=.93, bottom=.09)
    span = max(abs(gap)*2.6, .06)
    centre = (planar_mid+native_mid)/2
    lo, hi = centre-span, centre+span
    y, ticks = 0, []
    for title, rows in ordered:
        ax.text(-.02, y, title, transform=ax.get_yaxis_transform(), ha='right', va='center', fontsize=8.5, fontweight='bold', color=INK)
        y += 1
        for v in rows:
            ax.text(-.02, y, v['label'], transform=ax.get_yaxis_transform(), ha='right', va='center', fontsize=8, color=INK)
            if v['transition_in_bracket']:
                x = v['midpoint_n']
                if lo <= x <= hi:
                    ax.plot([x], [y], 'o', ms=7.5, mfc=PLANT_C[v['engine']], mec='white', mew=1.2, zorder=3, clip_on=False)
                else:
                    edge = lo if x < lo else hi
                    ax.annotate('', xy=(edge, y), xytext=(edge+(span*.12 if x < lo else -span*.12), y), arrowprops=dict(arrowstyle='-|>', color=PLANT_C[v['engine']], lw=1.4), zorder=3)
                    ax.text(edge+(span*.15 if x < lo else -span*.15), y, f'{x:.2f} N', ha='left' if x < lo else 'right', va='center', fontsize=7.5, color=MUTED)
                ax.text(1.04, y, f"{v['shift_from_planar_n']*1000:+.0f}", transform=ax.get_yaxis_transform(), ha='right', va='center', fontsize=8, color=INK, clip_on=False)
            else:
                trace = brackets[v['name']]['trace']
                assert all(outcome == 'fallen' for _, outcome in trace)
                ax.text(lo+span*.04, y, f'falls at every force tested ({trace[0][0]:g} to {trace[1][0]:g} N)', ha='left', va='center', fontsize=7.5, color=MUTED, bbox=dict(fc='white', ec='none', pad=1), zorder=4)
                ax.text(1.04, y, 'none', transform=ax.get_yaxis_transform(), ha='right', va='center', fontsize=8, color=MUTED)
            both = CODE[v['plus_fine']] if v['plus_fine'] == v['minus_fine'] else CODE[v['plus_fine']]+'/'+CODE[v['minus_fine']]
            ax.text(1.20, y, both, transform=ax.get_yaxis_transform(), ha='center', va='center', fontsize=8, color=INK, clip_on=False)
            ticks.append(y); y += 1
    ax.set_ylim(y-.4, -.8); ax.set_xlim(lo, hi); ax.set_yticks([])
    ax.spines['left'].set_visible(False)
    for value, name, colour in [(planar_mid, 'planar', PLANT_C['planar']), (native_mid, 'contact, as archived', PLANT_C['native'])]:
        ax.axvline(value, color=colour, lw=1, zorder=1)
    ax.axvline(18.2, color=INK, lw=.9, ls=(0, (4, 3)), zorder=0)
    ax.text(18.2, -.8, '18.2 N', ha='center', va='bottom', fontsize=7.5, color=INK)
    ax.grid(axis='x'); ax.set_xlabel('Critical force: midpoint of the fall bracket (N)')
    ax.text(1.04, -.75, 'Shift from\nplanar (mN)', transform=ax.get_yaxis_transform(), ha='right', va='bottom', fontsize=7.5, color=MUTED)
    ax.text(1.20, -.75, 'Label at\n±18.2 N', transform=ax.get_yaxis_transform(), ha='center', va='bottom', fontsize=7.5, color=MUTED)
    ax.legend(handles=[Line2D([], [], marker='o', ls='', ms=7, mfc=PLANT_C['planar'], mec='white', label='Planar equations'),
                       Line2D([], [], marker='o', ls='', ms=7, mfc=PLANT_C['native'], mec='white', label='Wheel-contact model')],
              loc='lower right', bbox_to_anchor=(-.02, 1.0), ncol=2, handletextpad=.3, columnspacing=1.2)
    save(fig, 'fu_fig01_boundary_shift', f'Critical push force for each of the {len(variants)} diagnostic models at a 0.5 N m torque limit, zero initial tilt and a {step*1000:g} ms physics step: the two unmodified references, {len(variants)-3} variants that each change one declared assumption, and one combined variant that changes three. Each dot is the midpoint of a bisection bracket {width*1000:.2f} mN wide; solid vertical lines mark the unmodified planar (dark grey) and wheel-contact (green) models, and the dashed line marks the disputed 18.2 N cell. The numeric columns give the shift from the planar bracket in millinewtons and the label at +18.2 and -18.2 N at the {fine*1000:g} ms confirmation step (R recovered, F fallen). Nothing is fitted, and no variant replaces the archived model.')

    # Figure 2: the label follows the pitch inertia, not the simulator; traction stays inside the friction limit.
    base = {**json.loads((ROOT/'config.json').read_text())['parameters'], 'torque_limit': protocol['torque_limit_nm']}
    def demand(a, inertia):
        p = Parameters(**{**base, 'body_inertia': inertia})
        out = []
        for row in a[:-1]:
            s, u, f = row[1:5], row[5], row[6]
            _, ax_, _, alpha = derivative(s, u, f, p)
            ml = p.body_mass*p.length
            horizontal = (p.base_mass+p.body_mass)*ax_+ml*(alpha*math.cos(s[2])-s[3]**2*math.sin(s[2]))-f+p.translation_drag*s[1]
            normal = (p.base_mass+p.body_mass)*p.gravity-ml*(alpha*math.sin(s[2])+s[3]**2*math.cos(s[2]))
            out.append([row[0], abs(horizontal)/normal])
        return np.array(out)
    key = lambda name: f'{name}__{fine:g}__+18.2'
    nominal_label, heavy_label = 'Pitch inertia as in the planar model', 'With 0.0002 kg m² chassis pitch inertia'
    SOLID, DASHED = '-', (0, (4, 2.5))
    panels = [('Planar equations', [('planar', SOLID, base['body_inertia']), ('planar_plus_chassis', DASHED, protocol['variants']['planar_plus_chassis']['planar_body_inertia'])]),
              ('Wheel-contact model', [('chassis_removed', SOLID, None), ('native', DASHED, None)])]
    fig, axs = plt.subplots(2, 2, figsize=(7.2, 4.9), sharex=True, sharey='row', layout='constrained')
    peak_use = {}
    for j, (title, runs) in enumerate(panels):
        colour = PLANT_C['planar' if j == 0 else 'native']
        for name, style, inertia in runs:
            a = arrays[key(name)]
            assert (by_name[name]['plus_fine'] == 'fallen') == (style == DASHED), name
            axs[0, j].plot(a[:, 0], a[:, 3]*180/math.pi, color=colour, ls=style)
            ratio = demand(a, inertia) if inertia else np.column_stack([a[:-1, 0], a[:-1, 15]])
            axs[1, j].plot(ratio[:, 0], ratio[:, 1], color=colour, ls=style)
            peak_use[name] = float(ratio[:, 1].max())
            end = a[-1]
            note = f'falls at {end[0]:.2f} s' if by_name[name]['plus_fine'] == 'fallen' else 'recovers'
            if by_name[name]['plus_fine'] == 'fallen':
                axs[0, j].annotate(note, (end[0], -45), xytext=(6, -3), textcoords='offset points', fontsize=7.5, color=MUTED, va='top', ha='left')
            else:
                axs[0, j].annotate(note, (3.3, float(np.interp(3.3, a[:, 0], a[:, 3]))*180/math.pi), xytext=(0, 6), textcoords='offset points', fontsize=7.5, color=MUTED, va='bottom', ha='center')
        axs[0, j].set_title(title, loc='left'); axs[1, j].set_xlabel('Time (s)')
        axs[0, j].axhline(-45, color=QUIET, lw=.8, zorder=1); axs[1, j].axhline(1, color=QUIET, lw=.8, zorder=1)
        for ax in axs[:, j]: ax.axvspan(2, 2.2, color='#efeeea', zorder=0); ax.grid(axis='y')
    axs[0, 1].text(4.47, -46, 'fall threshold', ha='right', va='top', fontsize=7.5, color=MUTED)
    axs[1, 1].text(4.47, 1.01, 'friction limit of the contact model', ha='right', va='bottom', fontsize=7.5, color=MUTED)
    axs[0, 0].set_ylabel('Body pitch (deg)'); axs[1, 0].set_ylabel('Horizontal / normal ground force')
    axs[1, 0].set_title('Traction the ideal-rolling model implies', loc='left', fontsize=8.5, fontweight='normal')
    axs[1, 1].set_title('Traction used (friction coefficient 1)', loc='left', fontsize=8.5, fontweight='normal')
    axs[0, 0].set_xlim(1.9, 4.5); axs[0, 0].set_ylim(-52, 26); axs[1, 0].set_ylim(0, 1.15)
    pitch_floor = min(float(np.min(arrays[key(n)][:, 3]))*180/math.pi for _, runs in panels for n, _, _ in runs)
    assert pitch_floor > -52
    largest_ratio = max(peak_use.values())
    assert largest_ratio < 1
    fig.legend(handles=[Line2D([], [], color=INK2, ls=SOLID, label=nominal_label), Line2D([], [], color=INK2, ls=DASHED, label=heavy_label)], loc='outside upper center', ncol=2)
    traction = dis['planar_traction_demand']['18.2']
    save(fig, 'fu_fig02_boundary_mechanism', f'The disputed +18.2 N cell at the {fine*1000:g} ms confirmation step; the push acts in the shaded interval. Left: planar equations (dark grey); right: wheel-contact model (green), the plant colours of the critical-force figure. Top: body pitch. In both simulators the run recovers with the planar pitch inertia (solid) and falls when 0.0002 kg m² of chassis pitch inertia is present (dashed): the label follows the inertia, not the simulator. The archived contact model is the dashed green curve. Bottom: ratio of horizontal to normal ground force. On the left it is the traction the ideal-rolling equations imply but never limit; on the right it is the traction the contact model uses. All four curves stay below the contact model\'s friction limit of 1 (largest value {largest_ratio:.2f}), so the wheels do not slide. The push tips the body backwards; a fall is declared at -45 deg. Curves end at a fall and are sampled every 5 ms.')

    # ---------------------------------------------------------------- noisy payload comparison
    summary_n = noisy['summary']
    archived = noisy['archived_planar']
    controls = json.loads((ROOT/'results/mujoco_noisy_payload/controls.json').read_text())
    for f, digest in controls['metadata']['source_sha256'].items():
        assert hashlib.sha256((ROOT/f).read_bytes()).hexdigest() == digest, f'{f} changed since the controls were run'
    plants = ['planar_port', 'native_0p25ms', 'native_0p5ms']
    PLANT = {'planar_port': 'Planar', 'native_0p25ms': 'Contact 0.25 ms', 'native_0p5ms': 'Contact 0.5 ms'}
    PROFILE = {'direct_0ms': 'Direct', 'kalman_0ms': 'Kalman', 'kalman_10ms': 'Kalman, 10 ms delay'}
    assert all(summary_n['planar_port'][k] == archived[k] for k in ['fixed_falls', 'exact_falls', 'prevented', 'introduced'])

    # Exact-feedback class of every load/scenario cell; identical in all three plants.
    exact_runs = {(r['engine_id'], r['mass_kg'], r['height_m'], r['scenario'], r['controller']): r for r in controls['ideal_control']['records']}
    def exact_class(e, m, h, s):
        f, a = (exact_runs[e, m, h, s, c]['outcome'] == 'fallen' for c in ['fixed', 'exact'])
        return {(False, False): 'both_survive', (True, False): 'only_fixed_falls', (True, True): 'both_fall', (False, True): 'only_aware_falls'}[f, a]
    classes = {}
    for m, h, s in {(r['mass_kg'], r['height_m'], r['scenario']) for r in noisy['records']}:
        found = {exact_class(e, m, h, s) for e in plants}
        assert len(found) == 1, (m, h, s, found)
        classes[m, h, s] = found.pop()
    class_counts = dict(Counter(classes.values()))
    paired = defaultdict(dict)
    for r in noisy['records']:
        paired[r['engine_id'], r['mass_kg'], r['height_m'], r['scenario'], r['profile'], r['seed']][r['controller']] = r['outcome'] == 'fallen'
    decomposition, by_class = {e: defaultdict(Counter) for e in plants}, {e: defaultdict(Counter) for e in plants}
    for (e, m, h, s, f, seed), v in paired.items():
        group = 'noise_induced' if classes[m, h, s] == 'both_survive' else 'falls_without_noise' if classes[m, h, s] in ('only_fixed_falls', 'both_fall') else 'other'
        for d in (decomposition[e][group], by_class[e][classes[m, h, s]]):
            d['pairs'] += 1; d['fixed_falls'] += v['fixed']; d['exact_falls'] += v['exact']
            d['prevented'] += v['fixed'] and not v['exact']; d['introduced'] += v['exact'] and not v['fixed']
    decomposition = {e: {k: dict(c) for k, c in d.items()} for e, d in decomposition.items()}
    by_class = {e: {k: dict(c) for k, c in d.items()} for e, d in by_class.items()}
    for e in plants:
        assert set(decomposition[e]) <= {'noise_induced', 'falls_without_noise'}
        for k in ['prevented', 'introduced', 'fixed_falls', 'exact_falls']:
            assert sum(c[k] for c in decomposition[e].values()) == summary_n[e][k]
    scenario_falls = {e: {s: dict(fixed=v['fixed_falls'], exact=v['exact_falls']) for s, v in summary_n[e]['by']['scenario'].items()} for e in plants}
    assert all(scenario_falls[e]['opposing_push'] == dict(fixed=0, exact=0) for e in plants)
    records_by = {(r['engine_id'], r['mass_kg'], r['height_m'], r['scenario'], r['profile'], r['seed'], r['controller']): r for r in noisy['records']}
    restricted_survivors = {}
    for e in plants:
        deltas = defaultdict(list)
        for (ee, m, h, s, f, seed), v in paired.items():
            if ee != e or s != 'restricted_torque' or classes[m, h, s] != 'both_survive' or v['fixed'] or v['exact']: continue
            a, b = records_by[e, m, h, s, f, seed, 'fixed'], records_by[e, m, h, s, f, seed, 'exact']
            for name in ['saturation_s', 'torque_squared_integral', 'peak_tilt_deg']: deltas[name].append(b[name]-a[name])
        restricted_survivors[e] = dict(pairs=len(deltas['saturation_s']), **{f'{k}_median': float(np.median(v)) for k, v in deltas.items()},
                                       **{f'{k}_iqr': [float(x) for x in np.percentile(v, [25, 75])] for k, v in deltas.items()})
    # Gradient with mounting height: noisy changes in the noise-induced cells, and the exact-feedback margin of the same cells.
    gradient = {e: {} for e in plants}
    for (e, m, h, s, f, seed), v in paired.items():
        if s != 'restricted_torque' or classes[m, h, s] != 'both_survive': continue
        g = gradient[e].setdefault(str(h), Counter())
        g['prevented'] += v['fixed'] and not v['exact']; g['introduced'] += v['exact'] and not v['fixed']
        g['fixed_falls'] += v['fixed']; g['exact_falls'] += v['exact']
    gradient = {e: {h: dict(c) for h, c in g.items()} for e, g in gradient.items()}
    exact_margin = [dict(mass_kg=m, height_m=h, exact_feedback_class=classes[m, h, 'restricted_torque'],
                         **{f'{c}_{k}': exact_runs[e, m, h, 'restricted_torque', c][k] for c in ['fixed', 'exact'] for k in ['outcome', 'saturation_s', 'peak_tilt_deg']}, engine_id=e)
                    for e in plants for m in [.35, .65, 1.15] for h in [.14, .26, .36]]
    saturation_share = {}
    for e in plants:
        groups = defaultdict(list)
        for r in noisy['records']:
            if r['engine_id'] == e and r['outcome'] != 'fallen': groups[r['scenario'], r['profile']].append(r['saturation_s']/r['duration_s'])
        saturation_share[e] = {f'{s}/{f}': float(np.median(v)) for (s, f), v in sorted(groups.items())}
    exact_saturation = {e: {s: float(np.median([r['saturation_s'] for r in controls['ideal_control']['records'] if r['engine_id'] == e and r['scenario'] == s and r['outcome'] != 'fallen']))
                            for s in ['opposing_push', 'restricted_torque']} for e in plants}
    noise_cells = sorted({(m, h) for (m, h, s), c in classes.items() if c == 'both_survive' and any(
        paired[e, m, h, s, f, seed]['fixed'] or paired[e, m, h, s, f, seed]['exact'] for (e, mm, hh, ss, f, seed) in paired if (mm, hh, ss) == (m, h, s))})

    fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.5), layout='constrained', gridspec_kw={'width_ratios': [1.1, 1]})
    ax = axs[0]
    ax.set_xlim(-.6, 2.6); ax.set_ylim(0, max(summary_n[k]['fixed_falls'] for k in plants)*1.12)
    ax.set_xticks(range(3)); ax.set_xticklabels([PLANT[k].replace(' ', '\n', 1) for k in plants]); ax.grid(axis='y'); ax.tick_params(axis='x', length=0, labelcolor=INK)
    ax.set_ylabel('Falls in 1,620 paired trials'); ax.set_title('Falls by controller', loc='left')
    rounded_bars(ax, [i-.15 for i in range(3)], [summary_n[k]['fixed_falls'] for k in plants], .26, ORANGE)
    rounded_bars(ax, [i+.15 for i in range(3)], [summary_n[k]['exact_falls'] for k in plants], .26, BLUE)
    for i, k in enumerate(plants):
        for dx, name in [(-.15, 'fixed_falls'), (.15, 'exact_falls')]:
            ax.text(i+dx, summary_n[k][name]+ax.get_ylim()[1]*.012, str(summary_n[k][name]), ha='center', va='bottom', fontsize=8, color=INK)
    ax = axs[1]
    rows3 = [('noise_induced', 'prevented', AQUA), ('falls_without_noise', 'prevented', YELLOW), ('noise_induced', 'introduced', INK2)]
    ax.set_ylim(2.65, -.65); ax.set_xlim(0, max(decomposition[k]['noise_induced']['prevented'] for k in plants)*1.22)
    ax.set_yticks(range(3)); ax.set_yticklabels([PLANT[k] for k in plants]); ax.grid(axis='x'); ax.tick_params(axis='y', length=0, labelcolor=INK)
    ax.set_xlabel('Paired trials whose fall label changes'); ax.set_title('What the redesign changes', loc='left')
    for offset, (group, key, colour) in zip([-.24, 0, .24], rows3):
        values = [decomposition[k][group].get(key, 0) for k in plants]
        rounded_bars(ax, [i+offset for i in range(3)], values, .2, colour, horizontal=True)
        for i, value in enumerate(values):
            ax.text(value+ax.get_xlim()[1]*.012, i+offset, str(value), ha='left', va='center', fontsize=7.5, color=INK)
    assert all(decomposition[k]['falls_without_noise'].get('introduced', 0) == 0 for k in plants)
    fig.legend(handles=[Patch(color=ORANGE, label='Fixed LQR'), Patch(color=BLUE, label='Payload-aware LQR'), Patch(color=AQUA, label='Prevented, both gains survive without noise'),
                        Patch(color=YELLOW, label='Prevented, fixed gain falls even without noise'), Patch(color=INK2, label='Introduced')],
               loc='outside lower center', ncol=2, fontsize=7.5)
    save(fig, 'fu_fig03_noisy_falls', 'Noisy fixed-versus-payload-aware comparison on the same 1,620 condition and seed pairs in each plant. Left: falls by controller. Right: paired trials whose fall label changes when the fixed gain is replaced by the exact-load gain. Prevented falls are split by what happens with exact state feedback at the same load and disturbance: both gains survive without noise, or the fixed gain falls even without noise. Every introduced fall is in conditions that both gains survive without noise. This split follows the fixed gain; the per-controller split is in the text. The planar counts equal the archived JavaScript results record for record. The contact runs use physics steps of 0.25 and 0.5 ms with 200 Hz control. Every fall in every plant occurs at the 0.3 N m torque limit. Counts describe this grid, not deployment probabilities.')

    cells = {}
    for engine in plants:
        for c in summary_n[engine]['by']['cell']:
            cells.setdefault((c['mass_kg'], c['height_m'], c['scenario'], c['profile']), {})[engine] = c
    diverging = LinearSegmentedColormap.from_list('fewer_more', [(0, '#104281'), (.25, '#3987e5'), (.42, '#9ec5f4'), (.5, '#f0efec'), (.58, '#f3b9b8'), (.75, '#e34948'), (1, '#9a2a29')])
    masses, heights = [.35, .65, 1.15], [.14, .26, .36]
    fig, axs = plt.subplots(2, 3, figsize=(7.2, 5.3), layout='constrained')
    for i, e in enumerate(['planar_port', 'native_0p25ms']):
        for j, f in enumerate(PROFILE):
            ax = axs[i, j]
            grid = np.array([[cells[m, h, 'restricted_torque', f][e]['exact_falls']-cells[m, h, 'restricted_torque', f][e]['fixed_falls'] for m in masses] for h in heights])
            image = ax.imshow(grid, origin='lower', cmap=diverging, vmin=-30, vmax=30, aspect='auto')
            for y, h in enumerate(heights):
                for x, m in enumerate(masses):
                    c = cells[m, h, 'restricted_torque', f][e]
                    r, g, bl, _ = diverging((grid[y, x]+30)/60)
                    light = .2126*r+.7152*g+.0722*bl > .5
                    ax.text(x, y+.08, f"{c['fixed_falls']} → {c['exact_falls']}", ha='center', va='center', fontsize=8, color=INK if light else 'white')
                    kind = classes[m, h, 'restricted_torque']
                    if kind != 'both_survive':
                        ax.text(x, y-.28, 'F | R' if kind == 'only_fixed_falls' else 'F | F', ha='center', va='center', fontsize=6.5, color=INK if light else 'white')
                        ax.add_patch(plt.Rectangle((x-.47, y-.47), .94, .94, fill=False, ec=INK if kind == 'only_fixed_falls' else MUTED, lw=1.4 if kind == 'only_fixed_falls' else 1.0, zorder=3))
            ax.set_xticks(range(3)); ax.set_xticklabels([f'{m:g}' for m in masses]); ax.set_yticks(range(3)); ax.set_yticklabels([f'{h:g}' for h in heights])
            ax.tick_params(length=0)
            for spine in ax.spines.values(): spine.set_visible(False)
            if i == 0: ax.set_title(PROFILE[f], loc='left')
            if i == 1: ax.set_xlabel('Payload mass (kg)')
            if j == 0: ax.set_ylabel(f'{PLANT[e]}\nPayload height (m)')
    bar = fig.colorbar(image, ax=axs, orientation='horizontal', shrink=.55, aspect=40, pad=.02)
    bar.set_label('Change in falls out of 30 seeds (payload-aware minus fixed)'); bar.outline.set_visible(False); bar.set_ticks([-30, -15, 0, 15, 30])
    save(fig, 'fu_fig04_noisy_map', 'Where the redesign changes falls: the 0.3 N m scenario (initial tilt -7 deg, +12 N push), for all nine loads and three sensing profiles, in the planar model (top) and the contact model at 0.25 ms (bottom). Each cell prints fixed-gain falls → payload-aware falls out of 30 paired seeds; colour is their difference, blue where the redesign removes falls. With exact state feedback both gains recover at every unmarked load. At the two loads marked F | R (0.65 and 1.15 kg at 0.36 m, three squares each) the fixed gain falls even with exact feedback and the payload-aware gain does not; at the load marked F | F (0.35 kg at 0.36 m) both fall. The 0.6 N m scenario is not drawn: no trial fell there under any gain, profile or plant. The 0.5 ms contact step check is in results/mujoco_followup/cells.csv.')

    fig, axs = plt.subplots(2, 2, figsize=(7.2, 4.5), sharey='row', layout='constrained')
    ex = {r['id']: r for r in noisy['examples']}
    for j, engine in enumerate(['planar_port', 'native_0p25ms']):
        for controller, colour, name in [('fixed', ORANGE, 'Fixed LQR'), ('exact', BLUE, 'Payload-aware LQR')]:
            a = examples[f'{engine}__{controller}']
            axs[0, j].plot(a[:, 0], a[:, 3]*180/math.pi, color=colour, label=name)
            axs[1, j].plot(a[:-1, 0], a[:-1, 9], color=colour, lw=.9, drawstyle='steps-post')
            assert ex[f'{engine}__{controller}']['outcome'] != 'fallen'
        axs[0, j].set_title('Planar equations' if j == 0 else 'Wheel-contact model, 0.25 ms', loc='left'); axs[0, j].set_xlabel('Time (s)')
        axs[1, j].set_xlim(1.9, 2.7); axs[1, j].set_xlabel('Time (s), around the push')
        for value in (-.6, .6): axs[1, j].axhline(value, color=QUIET, lw=.8, zorder=1)
        for ax in axs[:, j]: ax.axvspan(2, 2.2, color='#efeeea', zorder=0); ax.grid(axis='y')
    axs[0, 0].set_ylabel('True pitch (deg)'); axs[1, 0].set_ylabel('Motor torque (N m)')
    fig.legend(handles=[Line2D([], [], color=ORANGE, label='Fixed LQR'), Line2D([], [], color=BLUE, label='Payload-aware LQR')], loc='outside upper center', ncol=2)
    e = noisy['metadata']['protocol']['example']
    save(fig, 'fu_fig05_noisy_example', f'Protocol-selected noisy example, fixed before the runs: {e["mass_kg"]:g} kg at {e["height_m"]:g} m, 0.6 N m scenario (initial tilt +7 deg, -12 N push in the shaded interval), Kalman estimate with 10 ms packet delay, first seed. Top: true pitch over the whole run. Bottom: the torque command, held over each 5 ms interval, from 1.9 to 2.7 s; with this noise and delay it switches between the ±0.6 N m limits (grey lines) for much of the run, so a full-length trace is unreadable. Both controllers and both plants receive the same noise sequence, and all four runs stay upright, as does every trial at 0.6 N m.')

    ill = controls['illustration']
    ill_arrays = np.load(ROOT/'results/mujoco_noisy_payload/illustration.npz')
    shown = {r['id']: r for r in ill['runs']}
    limit = .3
    fig, axs = plt.subplots(2, 2, figsize=(7.2, 4.6), sharey='row', layout='constrained')
    for j, engine in enumerate(['planar_port', 'native_0p25ms']):
        # Fixed gain first as a wider underlay, so the payload-aware trace stays visible where both sit at the limit.
        for controller, colour, name, width in [('fixed', ORANGE, 'Fixed LQR', 2.6), ('exact', BLUE, 'Payload-aware LQR', 1.1)]:
            a = ill_arrays[f'{engine}__{controller}']
            r = shown[f'{engine}__{controller}']
            axs[0, j].plot(a[:, 0], a[:, 3]*180/math.pi, color=colour, label=name, lw=width*.8)
            axs[1, j].plot(a[:-1, 0], a[:-1, 9], color=colour, lw=width, drawstyle='steps-post')
            if r['outcome'] == 'fallen':
                axs[0, j].annotate(f"falls at {r['duration_s']:.2f} s", (a[-1, 0], -45), xytext=(6, 3), textcoords='offset points', fontsize=7.5, color=MUTED, va='bottom')
        axs[0, j].set_title('Planar equations' if j == 0 else 'Wheel-contact model, 0.25 ms', loc='left'); axs[0, j].set_xlabel('Time (s)')
        axs[1, j].set_xlim(1.8, 3.0); axs[1, j].set_xlabel('Time (s), around the push')
        for ax in axs[:, j]: ax.axvspan(2, 2.2, color='#efeeea', zorder=0); ax.grid(axis='y')
        for value in (-limit, limit): axs[1, j].axhline(value, color=QUIET, lw=.8, zorder=1)
        axs[0, j].axhline(45, color=QUIET, lw=.8, zorder=1); axs[0, j].axhline(-45, color=QUIET, lw=.8, zorder=1)
    assert all(shown[f'{e}__fixed']['outcome'] == 'fallen' and shown[f'{e}__exact']['outcome'] != 'fallen' for e in ['planar_port', 'native_0p25ms'])
    axs[0, 0].set_ylabel('True pitch (deg)'); axs[1, 0].set_ylabel('Motor torque (N m)')
    fig.legend(handles=[Line2D([], [], color=ORANGE, label='Fixed LQR'), Line2D([], [], color=BLUE, label='Payload-aware LQR')], loc='outside upper center', ncol=2)
    save(fig, 'fu_fig06_noise_induced_fall', f'A noisy fall that the redesign prevents. The selection rule was written after the noisy results were known and deliberately picks a favourable case: the condition that both gains survive without noise and in which the redesign prevents the most falls in the contact model ({ill["mass_kg"]:g} kg at {ill["height_m"]:g} m, 0.3 N m scenario, {PROFILE[ill["profile"]].lower()} profile; {ill["prevented_in_cell"]["native_0p25ms"]} of 30 seeds prevented), and the lowest seed at which the fixed gain falls and the payload-aware gain survives in both plants (seed index {ill["seed_index"]}). Top: true pitch over the whole run. Bottom: the torque command, held over each 5 ms interval, from 1.8 to 3.0 s; the fixed-gain trace is drawn wider underneath so both stay visible where they coincide at the limit. Grey lines mark the ±45 deg fall threshold and the ±0.3 N m torque limit. Both plants receive the same noise sequence. The payload-aware runs stay upright but, like every noisy survivor, do not meet the recovery dwell.')

    fig, axs = plt.subplots(1, 2, figsize=(7.2, 3.55), layout='constrained')
    def bubbles(ax, points, limits, title, xlabel, ylabel):
        groups = Counter(points)
        ax.plot(limits, limits, color=QUIET, lw=.8, zorder=1)
        for (x, yv), n in groups.items():
            ax.plot([x], [yv], 'o', ms=5+2.2*math.sqrt(n), mfc=INK2, mec='white', mew=1.2, alpha=.85, zorder=3)
        for (x, yv), n in groups.most_common(3):
            if n > 2: ax.annotate(f'{n} cells', (x, yv), xytext=(9+1.1*math.sqrt(n), -3), textcoords='offset points', fontsize=7.5, color=MUTED)
        ax.set(xlim=limits, ylim=limits, xlabel=xlabel, ylabel=ylabel); ax.set_title(title, loc='left'); ax.set_aspect('equal'); ax.grid()
    fixed_points = [(c['planar_port']['fixed_falls'], c['native_0p25ms']['fixed_falls']) for c in cells.values()]
    change_points = [(c['planar_port']['exact_falls']-c['planar_port']['fixed_falls'], c['native_0p25ms']['exact_falls']-c['native_0p25ms']['fixed_falls']) for c in cells.values()]
    bubbles(axs[0], fixed_points, (-2, 32), 'Falls with the fixed gain', 'Planar: falls in 30 seeds', 'Contact: falls in 30 seeds')
    low = min(min(p) for p in change_points)-2; high = max(max(p) for p in change_points)+2
    bubbles(axs[1], change_points, (low, high), 'Change from redesign', 'Planar: change in falls', 'Contact: change in falls')
    axs[1].axhline(0, color=QUIET, lw=.6, zorder=0); axs[1].axvline(0, color=QUIET, lw=.6, zorder=0)
    same_sign = sum((a < 0) == (b < 0) and (a > 0) == (b > 0) for a, b in change_points)
    save(fig, 'fu_fig07_plant_agreement', f'Agreement between plants, cell by cell. Each of the 54 cells is one load, scenario and sensing profile with 30 seeds; dot area grows with the number of coincident cells, and the 27 cells of the 0.6 N m scenario all sit at the origin. Left: falls under the fixed gain in the planar model against the contact model (0.25 ms). Right: net change in falls when the payload-aware gain replaces the fixed gain (payload-aware minus fixed); negative values are net reductions, prevented minus introduced. Points on the diagonal agree exactly. The direction of the change (fewer, equal or more falls) is the same in {same_sign} of 54 cells.')

    # ---------------------------------------------------------------- exact-feedback boundary of both gains (post hoc)
    eb = json.loads((ROOT/'results/mujoco_noisy_payload/exact_boundary.json').read_text())
    for f, digest in eb['metadata']['source_sha256'].items():
        assert hashlib.sha256((ROOT/f).read_bytes()).hexdigest() == digest, f'{f} changed since the boundary study was run'
    crit = {(r['engine_id'], r['mass_kg'], r['height_m'], r['scenario'], r['controller']): r for r in eb['records']}
    bwidth = eb['metadata']['width_n']
    for (m, h, s), kind in classes.items():
        for e in plants:
            for c, fell in zip(['fixed', 'exact'], {'both_survive': (False, False), 'only_fixed_falls': (True, False), 'both_fall': (True, True)}[kind]):
                r = crit[e, m, h, s, c]
                assert fell == (r['critical_push_n'] < 12) or abs(r['critical_push_n']-12) < bwidth, (e, m, h, s, c)
    shift = {(e, m, h, s): 100*(crit[e, m, h, s, 'exact']['critical_push_n']/crit[e, m, h, s, 'fixed']['critical_push_n']-1)
             for e, m, h, s, c in crit if c == 'fixed'}
    shift_by_height = {e: {str(h): [min(v for k, v in shift.items() if k[0] == e and k[2] == h), max(v for k, v in shift.items() if k[0] == e and k[2] == h)]
                           for h in [.14, .26, .36]} for e in plants}
    plant_gap = max(abs(crit[k]['critical_push_n']-crit[('planar_port',)+k[1:]]['critical_push_n']) for k in crit if k[0] != 'planar_port')
    shift_gap = max(abs(shift[k]-shift[('planar_port',)+k[1:]]) for k in shift if k[0] != 'planar_port')
    margin_points, rank = {e: [] for e in plants}, {e: {} for e in plants}
    for e in plants:
        for f in PROFILE:
            xs, ys = [], []
            for m, h in itertools.product(masses, heights):
                for c in ['fixed', 'exact']:
                    x, y = crit[e, m, h, 'restricted_torque', c]['critical_push_n']-12, cells[m, h, 'restricted_torque', f][e][f'{c}_falls']
                    margin_points[e].append(dict(mass_kg=m, height_m=h, profile=f, controller=c, margin_n=x, falls=y)); xs.append(x); ys.append(y)
            rank[e][f] = spearman(xs, ys)
    assert all(v < -.8 for e in plants for v in rank[e].values()), rank
    # Per controller: falls in conditions that the same controller survives without noise, and falls where it falls anyway.
    per_controller = {e: {c: Counter() for c in ['fixed', 'exact']} for e in plants}
    for (e, m, h, s, f, seed), v in paired.items():
        for c in ['fixed', 'exact']:
            per_controller[e][c]['noise_caused' if exact_runs[e, m, h, s, c]['outcome'] != 'fallen' else 'also_without_noise'] += v[c]
    per_controller = {e: {c: dict(x) for c, x in d.items()} for e, d in per_controller.items()}
    for e in plants:
        assert sum(per_controller[e]['fixed'].values()) == summary_n[e]['fixed_falls'] and sum(per_controller[e]['exact'].values()) == summary_n[e]['exact_falls']
        assert per_controller[e]['exact']['noise_caused'] >= per_controller[e]['fixed']['noise_caused']  # the redesign does not reduce noise-caused falls
    boundary = dict(width_n=bwidth, integrations=eb['metadata']['integrations'], shift_percent_by_height=shift_by_height, max_plant_gap_n=plant_gap,
                    max_shift_gap_points=shift_gap, max_shift_percent=max(shift.values()), min_shift_percent=min(shift.values()),
                    rank_correlation=rank, margin_points=margin_points, per_controller=per_controller,
                    cells=[dict(engine_id=e, mass_kg=m, height_m=h, scenario=s, fixed_n=crit[e, m, h, s, 'fixed']['critical_push_n'],
                                exact_n=crit[e, m, h, s, 'exact']['critical_push_n'], shift_percent=shift[e, m, h, s]) for e, m, h, s in sorted(shift)])

    fig = plt.figure(figsize=(7.2, 5.6), layout='constrained')
    top, bottom = fig.subfigures(2, 1, height_ratios=[1, 1.05])
    ax = top.subplots()
    e0 = 'native_0p25ms'
    xs = list(range(9))
    order = [(m, h) for h in heights for m in masses]
    for x, (m, h) in zip(xs, order):
        a, b2 = crit[e0, m, h, 'restricted_torque', 'fixed']['critical_push_n'], crit[e0, m, h, 'restricted_torque', 'exact']['critical_push_n']
        ax.plot([x-.09, x+.09], [a, b2], color=QUIET, lw=1.2, zorder=1)
        ax.plot([x-.09], [a], 'o', ms=7, mfc=ORANGE, mec='white', mew=1.2, zorder=3)
        ax.plot([x+.09], [b2], 'o', ms=7, mfc=BLUE, mec='white', mew=1.2, zorder=3)
        ax.text(x+.2, max(a, b2)+.12, f'{shift[e0, m, h, "restricted_torque"]:+.1f}%', ha='left', va='bottom', fontsize=7.5, color=INK)
    ax.axhline(12, color=INK, lw=.9, ls=(0, (4, 3)), zorder=0)
    ax.text(-.4, 11.95, 'push in the noisy trials', ha='left', va='top', fontsize=7.5, color=INK)
    ax.set_xticks(xs); ax.set_xticklabels([f'{m:g} kg\n{h:g} m' for m, h in order], fontsize=7.5); ax.tick_params(axis='x', length=0)
    ax.set_xlim(-.5, 8.7); ax.set_ylim(10.2, 15.6); ax.grid(axis='y'); ax.set_ylabel('Critical push (N)')
    ax.set_title('Exact-feedback fall boundary, 0.3 N m scenario, contact model 0.25 ms', loc='left')
    axs = bottom.subplots(1, 3, sharey=True)
    for j, f in enumerate(PROFILE):
        ax = axs[j]
        for c, colour in [('fixed', ORANGE), ('exact', BLUE)]:
            pts = [p for p in margin_points[e0] if p['profile'] == f and p['controller'] == c]
            ax.plot([p['margin_n'] for p in pts], [p['falls'] for p in pts], 'o', ms=6, mfc=colour, mec='white', mew=1, alpha=.9, zorder=3)
        ax.axvline(0, color=QUIET, lw=.8, zorder=0)
        ax.set_title(PROFILE[f], loc='left'); ax.grid(); ax.set_xlabel('Critical push minus 12 N (N)')
        ax.set_xlim(-1.6, 3.2); ax.set_ylim(-1.5, 31.5); ax.set_yticks([0, 10, 20, 30])
        ax.text(.97, .95, f'ρ = {rank[e0][f]:.2f}', transform=ax.transAxes, ha='right', va='top', fontsize=7.5, color=MUTED)
    axs[0].set_ylabel('Noisy falls in 30 seeds')
    fig.legend(handles=[Line2D([], [], marker='o', ls='', ms=7, mfc=ORANGE, mec='white', label='Fixed LQR'), Line2D([], [], marker='o', ls='', ms=7, mfc=BLUE, mec='white', label='Payload-aware LQR')],
               loc='outside upper center', ncol=2)
    save(fig, 'fu_fig08_exact_boundary', f'What the redesign changes without noise, and how that predicts the noisy falls. Top: the smallest push that makes each gain fall with exact state feedback, at the -7 deg tilt and 0.3 N m limit of the noisy scenario, found by bisection to {bwidth*1000:.1f} mN in the contact model at 0.25 ms (the planar model and the 0.5 ms step differ by at most {plant_gap:.2f} N). Labels give the payload-aware boundary relative to the fixed one. The dashed line is the 12 N push of the noisy trials: where both dots lie above it, both gains recover without noise. Bottom: noisy falls of each gain against the distance from 12 N to that gain\'s own boundary, one dot per load and gain, by sensing profile. Both gains follow one relationship; ρ is the Spearman rank correlation over all 18 dots. Post hoc analysis.')

    # ---------------------------------------------------------------- hardware planning numbers
    plan_cells = []
    for (m, h, s, f), c in cells.items():
        n = c['native_0p25ms']
        plan_cells.append(dict(mass_kg=m, height_m=h, scenario=s, profile=f, exact_feedback_class=classes[m, h, s], fixed_falls=n['fixed_falls'], exact_falls=n['exact_falls'],
                               prevented=n['prevented'], introduced=n['introduced'], planar_fixed_falls=c['planar_port']['fixed_falls'],
                               planar_exact_falls=c['planar_port']['exact_falls'], coarse_fixed_falls=c['native_0p5ms']['fixed_falls'], coarse_exact_falls=c['native_0p5ms']['exact_falls']))
    contrast = sorted([c for c in plan_cells if c['exact_feedback_class'] == 'both_survive' and c['profile'] != 'kalman_10ms' and c['prevented']-c['introduced'] >= 10
                       and c['planar_fixed_falls']-c['planar_exact_falls'] >= 10 and c['coarse_fixed_falls']-c['coarse_exact_falls'] >= 10],
                      key=lambda c: (-(c['prevented']-c['introduced']), c['mass_kg'], c['height_m'], c['profile']))
    chosen, seen = [], set()
    for c in contrast:
        if (c['mass_kg'], c['height_m']) in seen: continue
        chosen.append(c); seen.add((c['mass_kg'], c['height_m']))
        if len(chosen) == 3: break
    for c in chosen:
        c['pairs_for_90_percent_power_at_half_effect'] = pairs_needed(c['prevented']/30/2, c['introduced']/30+.05)
    # Boundary-shift stage: three loads chosen by rule from the contact-model shifts at 0.3 N m.
    loads = [(m, h) for m in masses for h in heights]
    s0 = lambda m, h: shift['native_0p25ms', m, h, 'restricted_torque']
    large = max(loads, key=lambda k: s0(*k))
    moderate = max([k for k in loads if k[1] == .26], key=lambda k: s0(*k))
    null = min(loads, key=lambda k: abs(s0(*k)))
    scatter, stair_step, margin = .03, .02, 3.0
    trials = next(n for n in range(10, 301) if (1.645+1.282)*math.sqrt(2)*100*staircase_sd(n, scatter, stair_step) <= margin)
    sd = 100*staircase_sd(trials, scatter, stair_step)
    boundary_cells = [dict(role=role, mass_kg=m, height_m=h, fixed_n=crit['native_0p25ms', m, h, 'restricted_torque', 'fixed']['critical_push_n'],
                           exact_n=crit['native_0p25ms', m, h, 'restricted_torque', 'exact']['critical_push_n'], shift_percent=s0(m, h),
                           power=None if role == 'null' else float(0.5*math.erfc(-(abs(s0(m, h))/(math.sqrt(2)*sd)-1.96)/math.sqrt(2))))
                      for role, (m, h) in [('large', large), ('moderate', moderate), ('null', null)]]
    hardware = dict(rule='Boundary-shift cells (H4), from the contact model at 0.25 ms, 0.3 N m, -7 deg: the load with the largest predicted shift, the load at 0.26 m with the largest shift, and the load with the smallest absolute shift (null). Staircase: one up, one down, step 2% of the predicted boundary, start one step below it, estimate = mean commanded push after the first six trials; realised push scatter assumed 3% (standard deviation) of the commanded push. Trials per gain and cell: smallest number for which a two one-sided test with a ±3% equivalence margin has 90% power at zero true shift (alpha 0.05 each side), from 4,000 simulated staircases. Contrast cells (H5): both gains survive with exact feedback in all three plants, not the delayed profile (in simulation it drives the torque into a sustained full-torque limit cycle that an ideal torque source can produce and a real motor driver may not), and a noisy net fall reduction of at least 10 of 30 seeds in the planar model and at both contact steps; at most one cell per load, largest contact-model reduction first. Pairs needed: exact two-sided sign test, alpha 0.05, power 0.9, assuming hardware prevents falls half as often as the contact simulation and introduces them 5 percentage points more often.',
                    boundary_cells=boundary_cells, staircase=dict(scatter=scatter, step=stair_step, margin_percent=margin, trials_per_gain=trials, estimate_sd_percent=sd),
                    contrast_cells=chosen, eligible_contrast_cells=len(contrast),
                    pairs_per_contrast_cell=max([c['pairs_for_90_percent_power_at_half_effect'] or 0 for c in chosen], default=None))

    summary = {
        'disagreement': dict(integrations=dis['metadata']['integrations']+addendum['metadata']['integrations'], main_integrations=dis['metadata']['integrations'],
                             addendum_integrations=addendum['metadata']['integrations'], other_boundaries=other, physics_step_s=step, confirmation_step_s=fine,
                             planar_midpoint_n=planar_mid, native_midpoint_n=native_mid, gap_n=gap, gap_percent=100*gap/planar_mid,
                             max_bracket_width_n=width, variants=variants, restores_planar_label=restores, planar_variants_that_fall=planar_flips,
                             startup=dis['startup'], traction=dis['planar_traction_demand'], monotonicity=dis['monotonicity'], regression=dis['regression']),
        'noisy': dict(trials=len(noisy['records']), engines={k: {x: y for x, y in v.items() if x != 'by'} for k, v in summary_n.items()},
                      by={k: {x: v['by'][x] for x in ['scenario', 'profile', 'mass_kg', 'height_m', 'load']} for k, v in summary_n.items()},
                      archived_planar=archived, port_verification={k: v for k, v in noisy['port_verification'].items() if k != 'outcome_mismatches'},
                      port_mismatches=len(noisy['port_verification']['outcome_mismatches']), persists=noisy['reduction_persists'],
                      fall_label_agreement=noisy['fall_label_agreement'], cells_same_direction=same_sign, cells=len(cells),
                      scenario_falls=scenario_falls, exact_feedback_classes=class_counts, decomposition=decomposition, decomposition_by_class=by_class, restricted_survivors=restricted_survivors,
                      height_gradient=gradient, exact_feedback_margin=exact_margin, saturation_share=saturation_share, exact_feedback_saturation_s=exact_saturation,
                      cell_classes=[dict(mass_kg=m, height_m=h, scenario=s, exact_feedback_class=c) for (m, h, s), c in sorted(classes.items())],
                      noise_induced_fall_cells=[list(x) for x in noise_cells],
                      exact_feedback_control={k: {x: y for x, y in v.items() if x != 'by_scenario'} for k, v in controls['ideal_control']['summary'].items()},
                      javascript_check=controls['ideal_control']['javascript_check'], repeatability=controls['repeatability'],
                      illustration={k: v for k, v in ill.items() if k not in ('runs', 'columns')},
                      illustration_runs={r['id']: dict(outcome=r['outcome'], duration_s=r['duration_s']) for r in ill['runs']},
                      protocol_sign_test_p={e: summary_n[e]['sign_test_p'] for e in plants}),
        'exact_boundary': boundary,
        'hardware_plan': hardware}
    (OUT/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    rows = ['mass_kg,height_m,scenario,profile,exact_feedback_class,'+','.join(f'{e}_{k}' for e in ['planar', 'contact_0p25ms', 'contact_0p5ms'] for k in ['fixed_falls', 'exact_falls', 'prevented', 'introduced'])]
    for (m, h, s, f), c in sorted(cells.items()):
        rows.append(','.join(map(str, [m, h, s, f, classes[m, h, s]]+[c[e][k] for e in plants for k in ['fixed_falls', 'exact_falls', 'prevented', 'introduced']])))
    (OUT/'cells.csv').write_text('\n'.join(rows)+'\n')
    (HERE/'followup_figure_manifest.json').write_text(json.dumps(list(manifest.values()), indent=2)+'\n')
    build_text(summary, manifest, cells, dis, noisy)


def build_text(summary, manifest, cells, dis, noisy):
    import followup_text
    title, blocks = followup_text.supplement(summary, cells, dis, noisy)
    setup_fonts()
    build_pdf(HERE/'output/pdf/mujoco_followup_report.pdf', title, blocks, manifest, 'Boundary diagnostics and noisy payload supplement | 13 November 2026')
    build_tex(HERE/'output/latex/mujoco_followup_report.tex', title, blocks, manifest)
    (OUT/'report_content.json').write_text(json.dumps(blocks, indent=2)+'\n')
    parts = []
    for block in blocks:
        if block[0] == 'heading': parts.append('<h2>'+html.escape(block[1])+'</h2>')
        elif block[0] == 'paragraph': parts.append('<p>'+html.escape(block[1])+'</p>')
        elif block[0] == 'figure':
            name = block[1]
            parts.append('<figure><img src="../../publication/figures/'+name+'.svg" alt="'+html.escape(manifest[name]['caption'], quote=True)+'"><figcaption>'+html.escape(manifest[name]['caption'])+'</figcaption></figure>')
        elif block[0] == 'table':
            parts.append('<div class="table-wrap"><table><caption>'+html.escape(block[1])+'</caption><tr>'+''.join('<th>'+html.escape(str(x))+'</th>' for x in block[2])+'</tr>'+''.join('<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in row)+'</tr>' for row in block[3])+'</table></div>')
        elif block[0] == 'reference': parts.append('<p>'+html.escape(block[1])+' <a href="'+block[2]+'">Source</a></p>')
    css = (ROOT/'results/mujoco/report.html').read_text(encoding='utf-8').split('<style>')[1].split('</style>')[0]
    nav = ''.join(f'<a href="{href}">{label}</a>' for href, label in [('../../publication/output/pdf/mujoco_followup_report.pdf', 'PDF report'), ('../../publication/output/pdf/manuscript_v2.pdf', 'Integrated manuscript'),
                                                                     ('../mujoco_disagreement/study.json', 'Diagnostics data'), ('../mujoco_noisy_payload/trials.csv', 'Noisy trials'), ('cells.csv', 'Per-cell table')])
    (OUT/'report.html').write_text('<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+html.escape(title)+'</title><style>'+css+'</style></head><body><main><nav>'+nav+'</nav><p>MICHAEL AGBO / BOUNDARY DIAGNOSTICS AND NOISY PAYLOAD COMPARISON</p><h1>'+html.escape(title)+'</h1>'+''.join(parts)+'</main></body></html>', encoding='utf-8')
    print(json.dumps({'disagreement': {k: v for k, v in summary['disagreement'].items() if k in ['integrations', 'planar_midpoint_n', 'native_midpoint_n', 'gap_n', 'restores_planar_label', 'planar_variants_that_fall']},
                      'noisy': summary['noisy']['engines'], 'persists': summary['noisy']['persists'], 'hardware_plan': summary['hardware_plan']}, indent=1, default=str))


if __name__ == '__main__':
    main()
