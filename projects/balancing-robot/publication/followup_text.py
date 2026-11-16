"""Prose for the follow-up supplement. Interpretive sentences are guarded by
assertions on the data, so a rerun that changes a finding fails loudly instead
of leaving stale text."""
PROFILE = {'direct_0ms': 'Direct', 'kalman_0ms': 'Kalman', 'kalman_10ms': 'Kalman, 10 ms delay'}
SCENARIO = {'opposing_push': '+7 deg, -12 N, 0.6 N m', 'restricted_torque': '-7 deg, +12 N, 0.3 N m'}
CODE = {'recovered': 'R', 'unsettled': 'U', 'fallen': 'F'}
TITLE = 'Boundary diagnostics, exact-feedback boundaries and the noisy payload comparison in MuJoCo'


def mn(value):
    return f'{value*1000:+.1f}'


def sci(value):
    """1.2 × 10⁻³⁹ instead of code notation."""
    mantissa, exponent = f'{value:.1e}'.split('e')
    return f"{mantissa} × 10{str(int(exponent)).translate(str.maketrans('-0123456789', '⁻⁰¹²³⁴⁵⁶⁷⁸⁹'))}"


def diagnosis_facts(d):
    """Numbers shared by the supplement and the integrated manuscript."""
    v = {x['name']: x for x in d['variants']}
    settling = ['height_radius', 'height_equilibrium', 'warmup_2s', 'reset_at_push']
    contact = [x for x in d['variants'] if x['group'] == 'contact' and x['transition_in_bracket']]
    assert set(d['restores_planar_label']) == {'chassis_removed', 'torso_compensated', 'idealised_combined'}
    assert d['planar_variants_that_fall'] == ['planar_plus_chassis']
    assert all(v[n]['shift_from_native_n'] == 0 and v[n]['plus_fine'] == 'fallen' for n in settling)
    assert v['planar_transplant']['shift_from_planar_n'] == 0 and v['planar_transplant']['plus_fine'] == 'recovered'
    assert not v['friction_0p5']['transition_in_bracket'] and v['friction_0p5']['plus_fine'] == 'fallen'
    assert all(x['plus_fine'] == x['minus_fine'] == 'fallen' for x in d['variants'] if x['group'] == 'contact')
    assert all(x['mirror_symmetric'] and x['step_consistent'] for x in d['variants'])
    assert all(v[n]['shift_from_native_n'] == 0 for n in ['friction_1p5', 'friction_2', 'friction_5', 'condim_3'])
    assert all(m['single_transition'] for m in d['monotonicity'].values())
    removed, doubled, plus = v['chassis_removed'], v['chassis_doubled'], v['planar_plus_chassis']
    assert v['torso_compensated']['midpoint_n'] == removed['midpoint_n'] == v['idealised_combined']['midpoint_n']
    axle = .012+1.2*.18**2
    # three non-overlapping increments: remove from contact, add again on top of contact, add to planar
    slopes = [(d['native_midpoint_n']-removed['midpoint_n'])/.0002, (doubled['midpoint_n']-d['native_midpoint_n'])/.0002, (plus['midpoint_n']-d['planar_midpoint_n'])/.0002]
    assert max(slopes)-min(slopes) < 5 and all(s < 0 for s in slopes)
    shifts = [x['shift_from_native_n'] for x in contact]
    chassis_effect = d['native_midpoint_n']-removed['midpoint_n']
    moved = sorted(abs(s/chassis_effect) for s in shifts if s)
    assert moved[-1] < .35 and moved[-2] < .1  # largest contact shift about a third of the inertia effect, the rest under a tenth
    assert v['native']['pulse_friction_use'] < .6 and v['native']['pulse_slip_m_s'] < .001 and v['friction_0p5']['pulse_slip_m_s'] > .15
    others = {b['torque_limit_nm']: b for b in d['other_boundaries']}
    assert all(b['archived_labels_reproduced'] == b['archived_zoom_cells'] for b in others.values())
    assert all(len(b['archived_grid_forces_inside_gap_n']) == b['archived_disagreements']/2 for b in others.values())
    startup = d['startup'][str(d['physics_step_s'])]
    assert startup['pulse_onset_state'] == [0, 0, 0, 0]
    traction = d['traction']
    assert max(t['peak_required_friction_coefficient'] for t in traction.values()) < 1
    assert .5 < traction['17.0']['peak_required_friction_coefficient'] and traction['18.2']['peak_required_friction_coefficient'] < .75
    return dict(v=v, axle_inertia=axle, inertia_percent=100*.0002/axle, slope_n_per_kg_m2=sum(slopes)/3, slopes=slopes, others=others,
                chassis_effect_n=chassis_effect, contact_variants=len([x for x in d['variants'] if x['group'] == 'contact']),
                residual_n=removed['shift_from_planar_n'], contact_low=min(shifts), contact_high=max(shifts),
                contact_low_name=min(contact, key=lambda x: x['shift_from_native_n'])['label'],
                contact_high_name=max(contact, key=lambda x: x['shift_from_native_n'])['label'], startup=startup, traction=traction)


def noisy_facts(n):
    e = n['engines']
    fine, coarse, port, old = e['native_0p25ms'], e['native_0p5ms'], e['planar_port'], n['archived_planar']
    assert n['port_mismatches'] == 0 and n['port_verification']['outcome_matches'] == 3240
    assert all(port[k] == old[k] for k in ['fixed_falls', 'exact_falls', 'prevented', 'introduced'])
    assert n['persists'] == all(x['exact_falls'] < x['fixed_falls'] and x['prevented'] > x['introduced'] for x in [fine, coarse])
    noise = {k: v['noise_induced'] for k, v in n['decomposition'].items()}
    det = {k: v['falls_without_noise'] for k, v in n['decomposition'].items()}
    assert all(det[k].get('introduced', 0) == 0 and noise[k]['introduced'] == e[k]['introduced'] for k in noise)
    assert all(v['opposing_push'] == dict(fixed=0, exact=0) for v in n['scenario_falls'].values())
    assert n['exact_feedback_classes'] == {'both_survive': 15, 'only_fixed_falls': 2, 'both_fall': 1}
    assert all(c['different_fall_labels'] == 2 and c['fixed_falls'] == 3 and c['exact_falls'] == 1 for c in n['exact_feedback_control'].values())
    assert n['javascript_check']['outcome_matches'] == 36 and n['repeatability']['identical'] == n['repeatability']['trials'] == 360
    cells = n['noise_induced_fall_cells']
    assert len(cells) == 6 and {h for _, h in cells} == {.14, .26}
    share = {k: noise[k]['prevented']/e[k]['prevented'] for k in noise}
    assert all(.55 < x < .65 for x in share.values())  # "about three fifths"
    assert all(n['restricted_survivors'][k]['saturation_s_median'] < 0 and n['restricted_survivors'][k]['torque_squared_integral_median'] < 0 for k in noise)
    reduction = lambda x: 100*(x['fixed_falls']-x['exact_falls'])/x['fixed_falls']
    return dict(fine=fine, coarse=coarse, port=port, old=old, persists=n['persists'], noise=noise, det=det, share=share,
                classes=n['exact_feedback_classes'], control=n['exact_feedback_control'],
                reduction={k: reduction(x) for k, x in [('planar', old), ('fine', fine), ('coarse', coarse)]},
                noise_reduction={k: reduction(x) for k, x in noise.items()})


def margin_facts(n, engine='native_0p25ms'):
    m = {(r['mass_kg'], r['height_m']): r for r in n['exact_feedback_margin'] if r['engine_id'] == engine}
    low, mid, high = ([m[x, h] for x in [.35, .65, 1.15]] for h in [.14, .26, .36])
    assert all(r['fixed_outcome'] == r['exact_outcome'] == 'recovered' for r in low+mid)
    low_peak = max(abs(r['fixed_peak_tilt_deg']-r['exact_peak_tilt_deg']) for r in low)
    low_sat = max(abs(r['fixed_saturation_s']-r['exact_saturation_s']) for r in low)
    mid_peak = [r['fixed_peak_tilt_deg']-r['exact_peak_tilt_deg'] for r in mid]
    sat = {x: (m[x, .26]['fixed_saturation_s'], m[x, .26]['exact_saturation_s']) for x in [.35, .65, 1.15]}
    assert low_peak < .3 and low_sat < .006 and min(mid_peak) > .5
    assert sat[.35][0]-sat[.35][1] > .1 and sat[.65][0]-sat[.65][1] > .1
    g = n['height_gradient']
    for e in g:
        assert g[e]['0.26']['introduced'] <= 1 and g[e]['0.26']['prevented'] > 5*g[e]['0.14']['prevented']
        assert abs(g[e]['0.14']['prevented']-g[e]['0.14']['introduced']) <= 4
    assert all(r['fixed_outcome'] == 'fallen' for r in high) and [r['exact_outcome'] == 'fallen' for r in high] == [True, False, False]
    return dict(low_peak=low_peak, mid_peak=(min(mid_peak), max(mid_peak)), sat=sat)


def boundary_facts(eb, n):
    """Exact-feedback boundary of both gains at the noisy conditions; guards every sentence built on it."""
    sh = eb['shift_percent_by_height']
    for e in sh:
        assert -1.5 < sh[e]['0.14'][0] and sh[e]['0.14'][1] < 0
        assert sh[e]['0.26'][0] > 3 and sh[e]['0.36'][1] > 15
    pc = eb['per_controller']
    rho = [v for e in eb['rank_correlation'] for v in eb['rank_correlation'][e].values()]
    assert eb['max_plant_gap_n'] < .1 and min(rho) > -1 and max(rho) < -.8
    fixed026 = [c['fixed_n'] for c in eb['cells'] if c['engine_id'] == 'native_0p25ms' and c['scenario'] == 'restricted_torque' and c['height_m'] == .26]
    assert all(12 < x < 13 for x in fixed026)
    opp = [min(c['fixed_n'], c['exact_n']) for c in eb['cells'] if c['scenario'] == 'opposing_push']
    assert min(opp) > 19
    return dict(sh=sh['native_0p25ms'], pc=pc, rho=(min(rho), max(rho)), fixed026=(min(fixed026), max(fixed026)), opp_min=min(opp),
                max_shift=eb['max_shift_percent'], integrations=eb['integrations'], width=eb['width_n'], plant_gap=eb['max_plant_gap_n'])


def exact_class(n, m, h, s):
    return next(c['exact_feedback_class'] for c in n['cell_classes'] if (c['mass_kg'], c['height_m'], c['scenario']) == (m, h, s))


def cell_list(n, kind):
    rows = [c for c in n['cell_classes'] if c['exact_feedback_class'] == kind]
    parts = []
    for s, label in [('opposing_push', '0.6 N m'), ('restricted_torque', '0.3 N m')]:
        chosen = [c for c in rows if c['scenario'] == s]
        if not chosen: continue
        parts.append(f'all nine loads at {label}' if len(chosen) == 9 else f'{label}: '+', '.join(f"{c['mass_kg']:g} kg at {c['height_m']:g} m" for c in chosen))
    return '; '.join(parts)


def fr_aware(n, engine):
    return n['decomposition_by_class'][engine]['only_fixed_falls']['exact_falls']


def by_class_prev(n, engine):
    """Prevented falls in the condition where both gains fall without noise."""
    return n['decomposition_by_class'][engine]['both_fall'].get('prevented', 0)


def number_blocks(blocks, prefix='S'):
    """Replace [[key]] table captions and {tab:key} / {fig:name} references with numbers in block order."""
    import re
    tables, figures = {}, {}
    for block in blocks:
        if block[0] == 'table':
            m = re.match(r'\[\[(\w+)\]\] ', block[1])
            assert m, block[1]
            tables[m.group(1)] = len(tables)+1
        elif block[0] == 'figure':
            figures[block[1]] = len(figures)+1
    out = []
    for block in blocks:
        if block[0] == 'table':
            key = re.match(r'\[\[(\w+)\]\] ', block[1]).group(1)
            block = (block[0], f'Table {prefix}{tables[key]}. '+block[1][len(key)+5:])+tuple(block[2:])
        elif block[0] == 'paragraph':
            text = re.sub(r'\{tab:(\w+)\}', lambda m: f'Table {prefix}{tables[m.group(1)]}', block[1])
            text = re.sub(r'\{fig:(\w+)\}', lambda m: str(figures[m.group(1)]), text)
            assert '{' not in text and '[[' not in text, text[:120]
            block = (block[0], text)
        out.append(block)
    return out


def supplement(summary, cells, dis, noisy):
    d, n, plan = summary['disagreement'], summary['noisy'], summary['hardware_plan']
    f, q = diagnosis_facts(d), noisy_facts(n)
    eb = boundary_facts(summary['exact_boundary'], n)
    assert f['residual_n'] > 0
    v = f['v']
    b = []
    def h(t): b.append(('heading', t))
    def p(t): b.append(('paragraph', t))
    def page(t, figure=None):
        b.append(('pagebreak',)); h(t)
        if figure: b.append(('figure', figure))
    step, fine = d['physics_step_s']*1000, d['confirmation_step_s']*1000

    h('Scope and principal findings')
    p('This supplement answers the two questions left open by the boundary and payload report. First, why do the planar model and the wheel-contact MuJoCo model disagree in exactly two fall-boundary cells? Second, does the reduction in noisy falls from payload-aware LQR redesign, seen so far only in the planar model, persist in the contact model? Both studies are simulations with illustrative parameters. Neither simulator is treated as ground truth, and nothing was fitted to make them agree.')
    p(f"The disagreement has a single sufficient cause. The contact model carries 0.0002 kg m² of chassis pitch inertia that the planar equations omit, {f['inertia_percent']:.2f}% of the pitch inertia about the axle. It lowers the contact model's critical push force by {abs(f['chassis_effect_n'])*1000:.1f} mN, from {v['chassis_removed']['midpoint_n']:.4f} N to {d['native_midpoint_n']:.4f} N. Everything else in the contact model raises its boundary {f['residual_n']*1000:.1f} mN above the planar {d['planar_midpoint_n']:.4f} N, so the net gap is {abs(d['gap_n'])*1000:.1f} mN ({abs(d['gap_percent']):.2f}%), and the 18.2 N grid cell happens to lie inside it. Removing that inertia from the contact model, or adding it to the planar model, exchanges their labels at +18.2 and -18.2 N. The unsettled contact start changes nothing in these cells. Contact and friction settings move the contact model's boundary by {mn(f['contact_low'])} to {mn(f['contact_high'])} mN, never enough to change the label, unless friction falls below the traction the manoeuvre needs.")
    verdict = 'persists' if q['persists'] else 'does not persist'
    p(f"The noisy fall reduction {verdict} in the contact model. With the same loads, disturbances, sensing profiles and noise seeds, fixed LQR falls in {q['fine']['fixed_falls']} of 1,620 trials and payload-aware LQR in {q['fine']['exact_falls']} at a 0.25 ms physics step ({q['fine']['prevented']} falls prevented, {q['fine']['introduced']} introduced), and in {q['coarse']['fixed_falls']} and {q['coarse']['exact_falls']} at 0.5 ms ({q['coarse']['prevented']} prevented, {q['coarse']['introduced']} introduced). The archived planar result is {q['old']['fixed_falls']} and {q['old']['exact_falls']} ({q['old']['prevented']} prevented, {q['old']['introduced']} introduced). A Python port of the sensing pipeline reproduces the outcome of all 3,240 archived planar records before any contact result is used. Every fall, in every plant, occurs at the tighter of the two torque limits, 0.3 N m.")
    pcn = eb['pc']['native_0p25ms']
    p(f"Two analyses added after these results show what the reduction is. With exact state feedback at the same conditions, the redesign changes the fall label in only 2 of the 18 load and scenario conditions. Measured as a boundary, its effect is larger and systematic: it raises the push the robot can withstand without falling by {eb['sh']['0.26'][0]:.0f}% to {eb['sh']['0.26'][1]:.0f}% at the 0.26 m mounting height and by {eb['sh']['0.36'][0]:.0f}% to {eb['sh']['0.36'][1]:.0f}% at 0.36 m, and lowers it by at most {abs(eb['sh']['0.14'][0]):.1f}% at 0.14 m, with the planar and contact models within {eb['plant_gap']:.2f} N of each other. At 0.26 m the 12 N push of the noisy scenario lies just below the fixed gain's boundary ({eb['fixed026'][0]:.2f} to {eb['fixed026'][1]:.2f} N), and the noisy fall count of each gain follows its distance to its own boundary, with both gains on one relationship (Spearman rank correlation {eb['rho'][0]:.2f} to {eb['rho'][1]:.2f} per plant and profile). Noise therefore makes a deterministic boundary shift visible as fewer falls; it does not add a separate benefit. Counted per controller, falls in conditions that the same controller survives without noise number {pcn['fixed']['noise_caused']} for the fixed gain and {pcn['exact']['noise_caused']} for the payload-aware gain in the contact model.")
    b.append(('table', '[[inventory]] Study inventory; integrations are not independent experimental samples', ['Study', 'Conditions', 'Integrations', 'Result'], [
        ['Boundary diagnostics and consistency check', f"{len(d['variants'])} variants, 3 limits", d['integrations'], 'Chassis inertia explains both cells'],
        ['Noisy payload, planar port', '1,620 pairs', '3,240', 'Matches archive in 3,240 of 3,240 outcomes'],
        ['Noisy payload, contact 0.25 ms', '1,620 pairs', '3,240', f"{q['fine']['fixed_falls']} to {q['fine']['exact_falls']} falls"],
        ['Noisy payload, contact 0.5 ms (step check)', '1,620 pairs', '3,240', f"{q['coarse']['fixed_falls']} to {q['coarse']['exact_falls']} falls"],
        ['Exact-feedback control (post hoc)', '18 conditions, 3 plants', '108', '2 of 18 fall labels changed, in every plant'],
        ['Exact-feedback boundary (post hoc)', '18 conditions, 3 plants', f"{eb['integrations']:,}", f"Boundary raised by up to {eb['max_shift']:.0f}%"],
        ['Repeatability check', '12 groups', '360', '360 of 360 records identical']], [.33, .19, .14, .34]))

    page('1 The two disagreements and the diagnostic protocol')
    p('The archived exploratory zoom compared both models at 0.05 N spacing around every coarse fall transition and found 86 agreements in 88 cells. The two exceptions are mirror images: an upright robot with the nominal fixed LQR and a 0.5 N m torque limit, pushed with +18.2 or -18.2 N for 0.2 s. The planar model recovers and the contact model falls, at every physics step down to 0.0625 ms. The earlier report listed three candidate causes without testing them: the additional chassis inertia, the start 0.2 mm above the resting height with no settling, and the contact model.')
    p(f"The diagnostic protocol was saved before any diagnostic run. It defines {len(d['variants'])} models in five groups: the two unmodified references, {len(d['variants'])-3} variants that each change one declared assumption, and one combined variant that changes three. The primary quantity is not the binary label but the critical force: for each variant, bisection between 17.0 and 19.5 N brackets the smallest force that produces a fall, to within {d['max_bracket_width_n']*1000:.2f} mN, at a {step:g} ms physics step. Labels at +18.2 and -18.2 N are then confirmed at {step:g} and {fine:g} ms. Control, sensing and fall detection stay at 5 ms. The main study uses {d['main_integrations']} integrations.")
    p(f"Three checks support the method. The unmodified planar and contact runs at +18.2 N reproduce the archived zoom trajectories bit for bit. Scans at 0.01 N spacing from 18.10 to 18.30 N show a single fall transition in each model (planar between {d['monotonicity']['planar']['last_not_fallen_n']:.2f} and {d['monotonicity']['planar']['first_fallen_n']:.2f} N, contact between {d['monotonicity']['native']['last_not_fallen_n']:.2f} and {d['monotonicity']['native']['first_fallen_n']:.2f} N), consistent with the bisection brackets. Every variant gives the same label for the two force signs and for the two physics steps. Because the target cells were chosen for their disagreement, this is an explanatory study of two cells, not a new test of simulator agreement.")

    page('2 Critical force by variant', 'fu_fig01_boundary_shift')
    rows = []
    for x in d['variants']:
        both = CODE[x['plus_fine']] if x['plus_fine'] == x['minus_fine'] else CODE[x['plus_fine']]+'/'+CODE[x['minus_fine']]
        if x['transition_in_bracket']:
            rows.append([x['label'], f"{x['last_not_fallen_n']:.4f} to {x['first_fallen_n']:.4f}", mn(x['shift_from_planar_n']), both])
        else:
            rows.append([x['label'], 'Falls at 17.0, ±18.2, 19.5', 'none', both])
    b.append(('table', f'[[brackets]] Fall brackets at {step:g} ms and labels at ±18.2 N at {fine:g} ms (R recovered, F fallen)', ['Variant', 'Bracket (N)', 'Shift from planar (mN)', 'Label'], rows, [.47, .25, .17, .11]))
    p('A bracket gives the largest tested force without a fall and the smallest with one. It is a property of this model, controller, torque limit and pulse; it is not a disturbance rating. Shifts smaller than the bracket width cannot be distinguished from zero.')

    page('3 Inertia: the label follows the chassis, not the simulator', 'fu_fig02_boundary_mechanism')
    p(f"The planar equations give the pitching assembly an inertia about the axle of I + m l² = {f['axle_inertia']:.5f} kg m². The contact model splits the 0.35 kg base into two wheels and a 0.11 kg chassis that is rigidly joined to the body and has its own inertia of 0.0002 kg m² about each axis. Its mass sits on the axle and is already counted in the base mass; its pitch inertia is additional, an increase of {f['inertia_percent']:.2f}%.")
    p(f"Four variants isolate this term. Removing the chassis inertia from the contact model moves its bracket from {mn(v['native']['shift_from_planar_n'])} to {mn(v['chassis_removed']['shift_from_planar_n'])} mN relative to the planar model, and the contact model then recovers at +18.2 and -18.2 N. Subtracting 0.0002 kg m² from the body pitch inertia instead gives the identical bracket. Adding 0.0002 kg m² to the planar body inertia moves the planar bracket by {mn(v['planar_plus_chassis']['shift_from_planar_n'])} mN, and the planar model then falls in both cells. Doubling the chassis inertia moves the contact bracket to {mn(v['chassis_doubled']['shift_from_planar_n'])} mN.")
    p(f"The response is linear over this range: {f['slopes'][0]:.0f}, {f['slopes'][1]:.0f} and {f['slopes'][2]:.0f} N per kg m² from three non-overlapping increments of 0.0002 kg m² (removing it from the contact model, adding a second one to the contact model, adding one to the planar model), a mean of {f['slope_n_per_kg_m2']:.0f} N per kg m², or about {f['slope_n_per_kg_m2']*.0002*1000:.0f} mN for the chassis term. Both simulators therefore agree on how the boundary depends on pitch inertia. With the inertia matched, the contact bracket lies {mn(f['residual_n'])} mN from the planar one ({abs(f['residual_n'])/d['planar_midpoint_n']*100:.3f}%), on the other side; that residual is the combined effect of everything else in the contact model. The inertia effect is therefore slightly larger than the gap it explains.")
    p('The direction is physically plausible but is reported as an observation. The fixed LQR gain was designed for the planar inertia, and under a saturated torque a body with more pitch inertia is both tipped and righted more slowly. In this cell the net effect is a smaller recoverable push. No claim is made that more inertia lowers the boundary in general.')

    page('4 Start-up and settling')
    s = f['startup']
    p(f"The contact model starts with the axle at 0.0452 m and comes to rest at {s['resting_axle_height_m']:.5f} m, a drop of {s['initial_drop_m']*1000:.2f} mm; first contact is recorded at {s['first_contact_time_s']*1000:.0f} ms. In the force slices the robot starts exactly upright, so this drop is symmetric: it excites vertical motion only. At the start of the pulse, 2 s later, the sampled position, speed, pitch and pitch rate are all exactly zero and no slip has been recorded.")
    p('Four contact variants test settling directly: starting at the wheel radius, starting at the measured resting height, running a 2 s controlled warm-up before time zero, and restoring an exactly upright zero-velocity state at the instant the pulse begins. All four give the same bracket and the same fall time at 18.2 N as the unmodified contact model. Injecting the contact model\'s pulse-onset state into the planar model also changes nothing, because that state is zero. Settling is therefore ruled out for these two cells. The test does not cover starts with an initial tilt, where a drop is not symmetric; the archived start-up slice showed no disagreement there.')

    page('5 Contact, friction and the traction the planar model assumes')
    p(f"{f['contact_variants']} variants change the contact model. Raising sliding friction to 1.5, 2 or 5, or removing torsional friction, leaves the bracket unchanged to the last bisection step. The other settings (friction 0.75, a 2 ms or 10 ms contact time constant, a stiffer impedance, a pyramidal friction cone, a no-slip solver pass, the RK4 integrator and rolling friction) move it by between {mn(f['contact_low'])} mN ({f['contact_low_name'].lower()}) and {mn(f['contact_high'])} mN ({f['contact_high_name'].lower()}) relative to the unmodified contact model. None changes the label at 18.2 N. The largest of these shifts is about a third of the {abs(f['chassis_effect_n'])*1000:.1f} mN inertia effect and the rest are under a tenth of it; that is the size of contact-model sensitivity at this boundary.")
    t = f['traction']
    p(f"Friction matters only when it is too low. The planar equations assume ideal rolling and never limit traction, but the ground force they imply can be computed from the trajectory. Its ratio to the normal force peaks at {t['17.0']['peak_required_friction_coefficient']:.2f} for a 17 N push, {t['18.0']['peak_required_friction_coefficient']:.2f} at 18 N and {t['18.2']['peak_required_friction_coefficient']:.2f} at 18.2 N, where the long recovery near the boundary unloads the wheels to {t['18.2']['min_normal_force_n']:.1f} N. The contact model with friction 1 uses at most {v['native']['pulse_friction_use']:.2f} of its limit during the pulse and its sampled slip stays below {v['native']['pulse_slip_m_s']*1000:.1f} mm/s. With friction 0.75 the bracket moves by {mn(v['friction_0p75']['shift_from_native_n'])} mN. With friction 0.5, below the implied demand, the wheels slide at about {v['friction_0p5']['pulse_slip_m_s']:.1f} m/s and the robot falls at all three forces tested (17.0, ±18.2 and 19.5 N), so no transition lies in the bracket.")
    p('This gives the planar model a stated domain of validity that it did not have before: near this boundary it describes a robot on a surface whose friction coefficient is at least about 0.75. That is a requirement to check on hardware, not a property the simulations establish.')

    page('6 What the diagnosis does and does not show')
    p(f"The two disagreements are a modelling difference of known size, not a numerical artefact and not a contact effect. Two simulators that agree on how the boundary depends on inertia were given inertias that differ by {f['inertia_percent']:.2f}%, and the 18.2 N grid force lies in the {abs(d['gap_n'])*1000:.1f} mN between their boundaries.")
    o = f['others']
    rows = [[f'{limit:g}', f"{b['brackets']['planar']['midpoint_n']:.4f}", f"{b['brackets']['native']['midpoint_n']:.4f}", mn(-b['gap_n']), mn(-b['gap_with_inertia_matched_n']),
             ', '.join(f'{x:g}' for x in b['archived_grid_forces_inside_gap_n']) or 'none', f"{b['archived_labels_reproduced']}/{b['archived_zoom_cells']}"] for limit, b in sorted(o.items())]
    b.append(('table', '[[consistency]] Post hoc consistency check at all three force boundaries (critical forces in N, shifts in mN relative to planar)', ['Limit (N m)', 'Planar', 'Contact', 'Contact shift', 'Inertia matched', 'Grid forces in gap', 'Archived labels'], rows, [.11, .13, .13, .14, .16, .17, .16]))
    p(f"If chassis inertia is the cause, it should act at every force boundary, and the archived zoom labels should follow from where the 0.05 N grid happens to fall. A post hoc check, designed after the main diagnostics, brackets the planar, contact, inertia-added and inertia-removed models at the 0.3 and 0.7 N m boundaries as well ({d['addendum_integrations']} further integrations). The contact boundary is lower than the planar one at all three limits, by {abs(o[.3]['gap_n'])*1000:.1f}, {abs(o[.5]['gap_n'])*1000:.1f} and {abs(o[.7]['gap_n'])*1000:.1f} mN. Matching the inertia reverses the sign at every limit, leaving the contact boundary {mn(-o[.3]['gap_with_inertia_matched_n'])}, {mn(-o[.5]['gap_with_inertia_matched_n'])} and {mn(-o[.7]['gap_with_inertia_matched_n'])} mN from the planar one: a small, consistent residual from the rest of the contact model. The brackets reproduce the archived label of both models in all {sum(b['archived_zoom_cells'] for b in o.values())} zoom cells of the three slices: the only grid forces that lie between the two boundaries are the two disputed ones. The archived agreement at 0.3 and 0.7 N m is therefore a matter of grid placement, and the critical-force difference is the better summary of simulator agreement than a count of matching cells.")
    p('The diagnosis does not make either model correct. Whether a real chassis contributes 0.0002 kg m² is a property of hardware that has not been built or measured. A shift of 0.2% in critical force is far below any plausible uncertainty in an identified pitch inertia or an applied push, so for hardware purposes the two models make the same prediction at this boundary, provided traction is adequate. The archived contact model is left unchanged, with its chassis inertia, and no variant is adopted as a correction.')

    page('7 Noisy payload comparison: protocol and port verification')
    p('The planar comparison of fixed and payload-aware LQR used nine reserved loads (0.35, 0.65 and 1.15 kg at 0.14, 0.26 and 0.36 m), two disturbance scenarios, three sensing profiles and 30 noise seeds: 1,620 paired trials per controller. Both scenarios pair a 7 degree initial tilt with an opposing 12 N push; they differ in torque limit, 0.6 N m in one and 0.3 N m in the other, with the signs mirrored. The profiles add Gaussian noise of 0.75 degrees to pitch and 3 deg/s to pitch rate and feed the controller the raw packet, a two-state Kalman estimate, or that estimate with a 10 ms packet delay. Both controllers use Q = diag(12,2,120,2) and R = 1; the payload-aware gain is designed for the true load.')
    p('The repeat changes one thing: the plant. The protocol, saved before any noisy contact run, copies the conditions, gains and seeds unchanged and declares in advance the paired analysis, a per-plant sign test, breakdowns by scenario, profile, mass and height, one example trajectory, and a persistence criterion: the reduction persists if payload-aware LQR has fewer falls than fixed LQR, and prevents more falls than it introduces, at both contact physics steps (0.25 and 0.5 ms). The criterion checks direction only, and the 0.5 ms run is a step-size check of the same deterministic model with the same seeds, not an independent replicate. The contact plant is the unmodified loaded model of the boundary and payload report, with friction 1, the archived chassis inertia and no warm-up. Measurements are the true position, speed, pitch and pitch rate of its base, with noise and delay applied by the same code path as for the planar plant.')
    ver = n['port_verification']
    p(f"The sensing pipeline exists in JavaScript. It was ported line for line to Python: the 32-bit linear congruential generator, the Box-Muller pairs, the packet queue and the scalar Kalman updates. Before any contact result was used, the port was run on the planar plant for every archived fixed and payload-aware record. All {ver['outcome_matches']:,} outcomes match the archive; the largest differences are {sci(ver['max_absolute_differences']['peak_tilt_deg'])} degrees in peak tilt and {sci(ver['max_absolute_differences']['torque_squared_integral'])} (N m)² s in the squared-torque integral. The contact plant therefore receives the same noise sequence as the planar plant, trial by trial.")

    page('8 Falls and paired transitions', 'fu_fig03_noisy_falls')
    rows = []
    for name, x in [('Planar', q['port']), ('Contact, 0.25 ms', q['fine']), ('Contact, 0.5 ms', q['coarse'])]:
        rows.append([name, x['fixed_falls'], x['exact_falls'], x['prevented'], x['introduced'], f"{100*(x['fixed_falls']-x['exact_falls'])/x['fixed_falls']:.0f}%"])
    b.append(('table', '[[falls]] Falls in 1,620 paired trials and paired transitions from fixed to payload-aware LQR', ['Plant', 'Fixed falls', 'Aware falls', 'Prevented', 'Introduced', 'Net reduction'], rows, [.26, .15, .15, .14, .14, .16]))
    if q['persists']:
        p(f"The declared criterion is met at both contact steps. At 0.25 ms the redesign removes {q['fine']['fixed_falls']-q['fine']['exact_falls']} of {q['fine']['fixed_falls']} falls ({q['reduction']['fine']:.0f}%); at 0.5 ms, {q['coarse']['fixed_falls']-q['coarse']['exact_falls']} of {q['coarse']['fixed_falls']} ({q['reduction']['coarse']:.0f}%). The planar reduction was {q['old']['fixed_falls']-q['old']['exact_falls']} of {q['old']['fixed_falls']} ({q['reduction']['planar']:.0f}%). As in the planar model, the net figure hides regressions: the redesign introduces {q['fine']['introduced']} falls at 0.25 ms and {q['coarse']['introduced']} at 0.5 ms.")
    else:
        p(f"The declared criterion is not met. At 0.25 ms fixed LQR falls in {q['fine']['fixed_falls']} trials and payload-aware LQR in {q['fine']['exact_falls']}; at 0.5 ms, {q['coarse']['fixed_falls']} and {q['coarse']['exact_falls']}. The planar reduction, {q['old']['fixed_falls']} to {q['old']['exact_falls']}, is not reproduced by the contact model under the rule fixed before the runs.")
    p(f"Every fall, in every plant and under every profile, occurs in the 0.3 N m scenario. No trial in the 0.6 N m scenario falls, under either gain. The comparison therefore rests on one disturbance at one torque limit, and the 810 pairs (1,620 trials) at 0.6 N m contribute only to the survivor statistics. Section 10 shows why: at 0.6 N m both gains' exact-feedback boundaries are at least {eb['opp_min']:.1f} N, far above the 12 N push. The declared exact two-sided sign test on the discordant pairs gives p = {sci(q['fine']['sign_test_p'])} at 0.25 ms, {sci(q['coarse']['sign_test_p'])} at 0.5 ms and {sci(n['protocol_sign_test_p']['planar_port'])} in the planar port.This summarises seed-level evidence inside one model. The 1,620 pairs share nine loads and two scenarios, so the test says nothing about a population of robots, loads or floors.")
    labels = {k: n['engines'][k]['outcomes'] for k in ['native_0p25ms', 'native_0p5ms']}
    recovered = sum(labels[k][c].get('recovered', 0) for k in labels for c in ['fixed', 'exact'])
    p(f"{'No' if recovered == 0 else recovered} noisy contact-model {'run meets' if recovered == 0 else 'runs meet'} the strict one-second recovery dwell, as in the planar model: persistent measurement noise keeps every survivor moving. Survivors are therefore labelled upright but unsettled, and the fall is the primary outcome.")

    page('9 Exact-feedback control at the same conditions', 'fu_fig04_noisy_map')
    c = q['control']['native_0p25ms']
    p(f"The archived ideal-sensing comparisons found no fall label changed by the redesign, in 216 planar and 36 contact conditions. Those conditions used different initial tilts and forces from the two noisy scenarios, at the same two torque limits. To separate the effect of the redesign from the effect of noise, the two noisy scenarios were run at all nine loads with exact state feedback (zero noise, zero delay) in all three plants: 108 deterministic runs. This control was designed after the noisy results were known. Its planar runs reproduce the archived JavaScript trial function in all {n['javascript_check']['outcome_matches']} records.")
    cls = {'both_survive': 'Both gains recover', 'only_fixed_falls': 'Only the fixed gain falls', 'both_fall': 'Both gains fall'}
    rows = [[cls[k], q['classes'][k], cell_list(n, k)] for k in ['both_survive', 'only_fixed_falls', 'both_fall']]
    b.append(('table', '[[outcome]] Exact-feedback outcome of the 18 load and scenario conditions, identical in all three plants', ['With exact feedback', 'Conditions', 'Which'], rows, [.27, .12, .61]))
    p(f"With exact feedback the redesign changes the fall label in two of the 18 conditions, both at 0.3 N m and 0.36 m: at 0.65 and 1.15 kg the fixed gain falls and the payload-aware gain recovers. It changes nothing in the other 16. The classification is the same in the planar model and at both contact steps. Taken with the earlier grids, the redesign changes the fall label with exact feedback in 2 of 234 distinct planar conditions and 2 of 54 contact-model conditions, always by preventing a fall.")
    rows = []
    for name, k in [('Planar', 'planar_port'), ('Contact, 0.25 ms', 'native_0p25ms'), ('Contact, 0.5 ms', 'native_0p5ms')]:
        a, d2 = q['noise'][k], q['det'][k]
        rows.append([name, f"{a['fixed_falls']} → {a['exact_falls']}", a['prevented'], a['introduced'], f"{d2['fixed_falls']} → {d2['exact_falls']}", d2['prevented']])
    b.append(('table', '[[split]] Noisy pairs split by the exact-feedback outcome of the same load and disturbance: falls (fixed → payload-aware) and falls prevented', ['Plant', 'Both gains survive without noise', 'Prevented', 'Introduced', 'Fixed gain falls without noise', 'Prevented'], rows, [.18, .20, .12, .12, .24, .14]))
    pr = []
    for name, k in [('Planar', 'planar_port'), ('Contact, 0.25 ms', 'native_0p25ms'), ('Contact, 0.5 ms', 'native_0p5ms')]:
        x = eb['pc'][k]
        pr.append([name, x['fixed']['noise_caused'], x['exact']['noise_caused'], x['fixed']['also_without_noise'], x['exact']['also_without_noise']])
    p(f"{{tab:split}} sorts every pair by what both gains do without noise. In the 15 conditions that both gains survive without noise, the redesign reduces falls from {q['noise']['native_0p25ms']['fixed_falls']} to {q['noise']['native_0p25ms']['exact_falls']} in the contact model at 0.25 ms (planar {q['noise']['planar_port']['fixed_falls']} to {q['noise']['planar_port']['exact_falls']}), preventing {q['noise']['native_0p25ms']['prevented']} and introducing {q['noise']['native_0p25ms']['introduced']}; all of these falls lie in the six 0.3 N m conditions at 0.14 and 0.26 m, and every introduced fall is in this group. The other {q['det']['native_0p25ms']['prevented']} prevented falls (planar {q['det']['planar_port']['prevented']}) are in the three conditions where the fixed gain falls without noise. Of these, the two conditions where the payload-aware gain recovers without noise contribute {q['det']['native_0p25ms']['prevented']-by_class_prev(n, 'native_0p25ms')} in every plant, and there noise makes the payload-aware gain fall in {fr_aware(n, 'native_0p25ms')} of 180 trials (planar {fr_aware(n, 'planar_port')}).")
    b.append(('table', '[[percontroller]] Noisy falls per controller, split by whether the same controller falls at the same load and disturbance without noise', ['Plant', 'Fixed: survives without noise', 'Aware: survives without noise', 'Fixed: falls without noise', 'Aware: falls without noise'], pr, [.20, .20, .20, .20, .20]))
    p(f"This split is not symmetric, because it classifies the payload-aware gain's falls by what the fixed gain does. {{tab:percontroller}} classifies each controller by its own exact-feedback outcome. Counted that way, the redesign does not reduce the falls that noise causes: in the contact model {eb['pc']['native_0p25ms']['fixed']['noise_caused']} fixed-gain and {eb['pc']['native_0p25ms']['exact']['noise_caused']} payload-aware falls occur in conditions that the same controller survives without noise. The whole net reduction comes from the conditions in which the fixed gain falls without noise and the payload-aware gain mostly does not. Neither split, on its own, says why; the boundary measurement of Section 10 does.")

    page('10 The exact-feedback boundary of both gains', 'fu_fig08_exact_boundary')
    cb = {(c['engine_id'], c['mass_kg'], c['height_m'], c['scenario']): c for c in summary['exact_boundary']['cells']}
    p(f"A fall label at one push says only on which side of the boundary that push lies. This post hoc analysis, added after an internal review of the integrated manuscript, measures the boundary itself. For each load, both noisy scenarios, both gains and all three plants, bisection over the push magnitude between 0 and 40 N finds, to {eb['width']*1000:.1f} mN, the smallest push that makes the robot fall with exact state feedback; tilt, torque limit, pulse, gains and fall rule are those of the noisy study ({eb['integrations']:,} integrations). Planar runs also scan 0 to 40 N in 1 N steps and show a single fall transition, and every boundary is consistent with the archived exact-feedback label at 12 N.")
    rows = []
    for m in [.35, .65, 1.15]:
        for hh in [.14, .26, .36]:
            c3, c6 = cb['native_0p25ms', m, hh, 'restricted_torque'], cb['native_0p25ms', m, hh, 'opposing_push']
            rows.append([f'{m:g} kg, {hh:g} m', f"{c3['fixed_n']:.2f}", f"{c3['exact_n']:.2f}", f"{c3['shift_percent']:+.1f}%", f"{c6['fixed_n']:.2f}", f"{c6['exact_n']:.2f}", f"{c6['shift_percent']:+.1f}%"])
    b.append(('table', '[[boundary]] Exact-feedback critical push (N) in the contact model at 0.25 ms; the noisy trials push with 12 N', ['Load', 'Fixed, 0.3 N m', 'Aware, 0.3 N m', 'Shift', 'Fixed, 0.6 N m', 'Aware, 0.6 N m', 'Shift'], rows, [.19, .135, .135, .11, .135, .135, .11]))
    p(f"The redesign moves the boundary by an amount that grows with mounting height and mass ({{tab:boundary}}, Figure {{fig:fu_fig08_exact_boundary}}). It raises it by {eb['sh']['0.26'][0]:.1f}% to {eb['sh']['0.26'][1]:.1f}% at 0.26 m and by {eb['sh']['0.36'][0]:.1f}% to {eb['sh']['0.36'][1]:.1f}% at 0.36 m, and lowers it by {abs(eb['sh']['0.14'][1]):.1f}% to {abs(eb['sh']['0.14'][0]):.1f}% at 0.14 m, at both torque limits. The planar model and both contact steps give the same boundaries to within {eb['plant_gap']:.2f} N. At the two upper heights these shifts are twenty to a hundred times the 0.2% that separates the two simulators at the boundary of Section 2, yet almost none of them changes a label on a fixed grid, because a label changes only when a test force happens to lie between the two boundaries. That happens at 0.65 and 1.15 kg at 0.36 m and 12 N, and nowhere on the 216-condition ideal grid.")
    p(f"The boundaries also account for the noisy results. At 0.6 N m both gains' boundaries are at least {eb['opp_min']:.1f} N, and no noisy trial falls. At 0.3 N m the 12 N push lies {eb['fixed026'][0]-12:.2f} to {eb['fixed026'][1]-12:.2f} N below the fixed gain's boundary at 0.26 m and further below the payload-aware one. Plotted against the distance from 12 N to its own boundary, each gain's noisy fall count falls on one decreasing relationship for both gains (Figure {{fig:fu_fig08_exact_boundary}}, bottom; Spearman rank correlation {eb['rho'][0]:.2f} to {eb['rho'][1]:.2f} over the 18 load and gain points, per plant and profile). For example, in the contact model with direct measurements the fixed gain at 1.15 kg and 0.26 m and the payload-aware gain at 1.15 kg and 0.36 m have nearly the same margin and nearly the same number of falls. The noisy fall reduction is therefore what a boundary shift looks like when noise spreads the effective disturbance around a push close to the boundary. This reading is post hoc and rests on 18 points per profile; it predicts that the noisy benefit would vanish for pushes far below both boundaries and turn into a deterministic difference for pushes between them.")

    page('11 A noisy fall that the redesign prevents', 'fu_fig06_noise_induced_fall')
    ill, runs = n['illustration'], n['illustration_runs']
    p(f"Figure {{fig:fu_fig06_noise_induced_fall}} shows one noisy fall that the redesign prevents. The selection rule was written after the noisy results were known and picks a favourable case: the condition that both gains survive without noise and in which the redesign prevents the most falls in the contact model ({ill['mass_kg']:g} kg at {ill['height_m']:g} m, 0.3 N m, {PROFILE[ill['profile']].lower()} profile; {ill['prevented_in_cell']['native_0p25ms']} of 30 seeds in the contact model and {ill['prevented_in_cell']['planar_port']} in the planar model), and within it the lowest seed at which the fixed gain falls and the payload-aware gain survives in both plants (seed index {ill['seed_index']}). The fixed gain falls at {runs['planar_port__fixed']['duration_s']:.2f} s in the planar model and {runs['native_0p25ms__fixed']['duration_s']:.2f} s in the contact model; the payload-aware gain stays upright in both. Without noise, both gains recover in this cell.")
    rs = n['restricted_survivors']
    def med(e, k, digits):
        return f"{rs[e][k+'_median']:+.{digits}f} (interquartile range {rs[e][k+'_iqr'][0]:+.{digits}f} to {rs[e][k+'_iqr'][1]:+.{digits}f})"
    g, mf = n['height_gradient'], margin_facts(n)
    p(f"The same ordering by mounting height appears in the noisy counts. In the conditions both gains survive without noise, the redesign prevents {g['native_0p25ms']['0.14']['prevented']} falls in the contact model at 0.14 m and introduces {g['native_0p25ms']['0.14']['introduced']}, little net effect where its boundary barely moves (planar {g['planar_port']['0.14']['prevented']} and {g['planar_port']['0.14']['introduced']}). At 0.26 m it prevents {g['native_0p25ms']['0.26']['prevented']} and introduces {g['native_0p25ms']['0.26']['introduced']} (planar {g['planar_port']['0.26']['prevented']} and {g['planar_port']['0.26']['introduced']}). The single exact-feedback runs at 12 N agree: at 0.14 m the two gains reach the same peak tilt within {mf['low_peak']:.2f} degrees and spend the same time at the torque limit to within one control interval; at 0.26 m the fixed gain recovers with {mf['mid_peak'][0]:.1f} to {mf['mid_peak'][1]:.1f} degrees more peak tilt. At 0.36 m the fixed gain falls at all three masses even without noise, and the payload-aware gain at one (0.35 kg).")
    sh, ex = n['saturation_share']['native_0p25ms'], n['exact_feedback_saturation_s']['native_0p25ms']
    p(f"Noise consumes much of the torque budget. With exact feedback, a surviving run at 0.3 N m spends a median of {ex['restricted_torque']:.2f} s at the torque limit. Under noise, surviving contact-model runs at 0.3 N m spend a median {100*sh['restricted_torque/direct_0ms']:.0f}% of the run at the limit with direct measurements, {100*sh['restricted_torque/kalman_0ms']:.0f}% with the Kalman estimate and {100*sh['restricted_torque/kalman_10ms']:.0f}% with the delayed estimate, where the command settles into a full-torque limit cycle (Figure {{fig:fu_fig05_noisy_example}} shows the same at 0.6 N m). Among the {rs['native_0p25ms']['pairs']} contact-model pairs in the six noise-affected cells in which both gains stay upright, the paired difference (payload-aware minus fixed) in time at the limit has a median of {med('native_0p25ms', 'saturation_s', 2)} s, and in the squared-torque integral {med('native_0p25ms', 'torque_squared_integral', 4)} (N m)² s; the planar values are {med('planar_port', 'saturation_s', 2)} s and {med('planar_port', 'torque_squared_integral', 4)}.")
    p('Survivor comparisons are conditional on both gains staying upright and describe nine loads, two disturbances and one noise level. They are consistent with the boundary reading of Section 10 but do not test it.')

    page('12 Protocol-selected example', 'fu_fig05_noisy_example')
    def metric(x, name, digits):
        z = lambda v: f'{v+0.0:+.{digits}f}'.replace(f'-{0:.{digits}f}', f'{0:.{digits}f}').replace(f'+{0:.{digits}f}', f'{0:.{digits}f}')
        return f"{z(x[f'survivor_{name}_median_delta'])} (interquartile range {z(x[f'survivor_{name}_iqr'][0])} to {z(x[f'survivor_{name}_iqr'][1])})"
    p(f"The protocol fixed one example before any noisy contact run (Figure {{fig:fu_fig05_noisy_example}}): 0.65 kg at 0.26 m, the 0.6 N m scenario, the Kalman profile with a 10 ms delay and the first seed. Like every trial at 0.6 N m, all four runs stay upright. The example shows that the two plants respond almost identically to the same noise sequence, and that the redesign changes the response without changing the outcome.")
    p(f"Across all {q['fine']['both_survive']:,} pairs in which both controllers survive in the contact model at 0.25 ms, most of them at 0.6 N m, the payload-aware design changes median peak tilt by {metric(q['fine'], 'peak_tilt_deg', 2)} degrees, the squared-torque integral by {metric(q['fine'], 'torque_squared_integral', 4)} (N m)² s and final-second tilt RMS by {metric(q['fine'], 'tail_tilt_rms_deg', 3)} degrees. In the planar port the {q['port']['both_survive']:,} mutual survivors give {metric(q['port'], 'peak_tilt_deg', 2)} degrees, {metric(q['port'], 'torque_squared_integral', 4)} (N m)² s and {metric(q['port'], 'tail_tilt_rms_deg', 3)} degrees. These are conditional on mutual survival and exclude every fall; they complement the fall counts and cannot replace them.")

    page('13 Agreement between plants, step sensitivity and repeatability', 'fu_fig07_plant_agreement')
    a = n['fall_label_agreement']
    p(f"The two plants agree on where falls occur and where the redesign helps (Figure {{fig:fu_fig07_plant_agreement}}). Here a cell is one load, scenario and sensing profile with 30 seeds. The direction of the change in falls (fewer, equal or more) is the same in the planar model and the contact model at 0.25 ms in {n['cells_same_direction']} of {n['cells']} cells. A noisy closed loop near a fall boundary amplifies small numerical differences, so an individual seed can fall in one plant or at one step and survive in another. Individual fall labels agree between the planar port and the contact model at 0.25 ms in {a['planar_port vs native_0p25ms']['same_fall_label']:,} of {a['planar_port vs native_0p25ms']['trials']:,} trials, between the two contact steps in {a['native_0p25ms vs native_0p5ms']['same_fall_label']:,}, and between the planar port and the contact model at 0.5 ms in {a['planar_port vs native_0p5ms']['same_fall_label']:,}. Aggregate and paired counts are the unit of comparison; they differ between the two contact steps by {abs(q['fine']['fixed_falls']-q['coarse']['fixed_falls'])} fixed-gain falls and {abs(q['fine']['exact_falls']-q['coarse']['exact_falls'])} payload-aware falls.")
    rep = n['repeatability']
    p(f"The contact runs were checkpointed by condition group and computed by parallel worker processes. To confirm that this cannot have changed a result, {rep['groups']} groups ({rep['trials']} trials, every 18th group in sorted order) were recomputed from scratch in fresh processes; all {rep['identical']} records are identical to the stored ones in all {rep['fields_compared']} fields. The two contact runs are complete repeats of the same model at two physics steps, not a selective audit, and do not amount to a convergence proof. The planar port uses one RK4 step per 5 ms control interval, as archived.")
    rows = [[f'{SCENARIO[s]}, all profiles']+[f"{n['by'][e]['scenario'][s]['fixed_falls']} to {n['by'][e]['scenario'][s]['exact_falls']}" for e in ['planar_port', 'native_0p25ms', 'native_0p5ms']] for s in SCENARIO]
    rows += [[f'0.3 N m, {label}']+[f"{n['by'][e]['profile'][prof]['fixed_falls']} to {n['by'][e]['profile'][prof]['exact_falls']}" for e in ['planar_port', 'native_0p25ms', 'native_0p5ms']] for prof, label in PROFILE.items()]
    b.append(('table', '[[byscenario]] Falls by scenario (810 pairs each) and, since every fall is at 0.3 N m, by sensing profile (540 pairs each, both scenarios): fixed to payload-aware', ['Group', 'Planar', 'Contact 0.25 ms', 'Contact 0.5 ms'], rows, [.37, .21, .21, .21]))

    page('14 Complete per-cell table')
    p('{tab:cells} gives every cell: one load, torque limit and sensing profile with 30 paired seeds. The exact-feedback column gives the outcome of the same load and disturbance without noise: R | R both gains recover, F | R only the fixed gain falls, F | F both fall, as in Figure {fig:fu_fig04_noisy_map}. The same table, with prevented and introduced counts, is in results/mujoco_followup/cells.csv.')
    tags = {'both_survive': 'R | R', 'only_fixed_falls': 'F | R', 'both_fall': 'F | F'}
    rows = []
    for (m, hgt, s, prof), cc in sorted(cells.items()):
        rows.append([f'{m:g} kg, {hgt:g} m', '0.6' if s == 'opposing_push' else '0.3', PROFILE[prof], tags[exact_class(n, m, hgt, s)]]+[f"{cc[e]['fixed_falls']} to {cc[e]['exact_falls']}" for e in ['planar_port', 'native_0p25ms', 'native_0p5ms']])
    b.append(('table', '[[cells]] Falls in 30 seeds, fixed to payload-aware, for every load, torque limit (N m) and sensing profile', ['Load', 'Limit', 'Profile', 'Exact', 'Planar', 'Contact 0.25', 'Contact 0.5'], rows, [.19, .08, .22, .08, .14, .145, .145]))

    page('14 Reproduction, limits and use')
    p(f"Engine versions: MuJoCo {dis['metadata']['mujoco']}, NumPy {dis['metadata']['numpy']}, Python {dis['metadata']['python']}. Run mujoco_lab/run_followup_study.ps1 from the project root to repeat the diagnostics, the consistency check, the noisy study, its controls, this report and the integrated manuscript. Protocols, source hashes, every trial metric and the diagnostic trajectories are stored in results/mujoco_disagreement and results/mujoco_noisy_payload; summary.json and cells.csv are in results/mujoco_followup. The report builder refuses to run if any hashed source has changed since the data were produced.")
    p('Limits. All parameters are illustrative and no hardware exists. The diagnostics explain two cells and are consistent at three torque limits, all from an upright start. The noisy comparison covers two disturbance scenarios, of which only the 0.3 N m one produces any fall, one noise level, exact load knowledge and a centred rigid payload; the four load-estimate-error variants were not repeated in the contact model. The exact-feedback control, both decompositions, the boundary measurement and the illustration were designed after the noisy results were known; the boundary measurement was added after an internal review. The two plants share the controller, sensing model, noise generator, ideal torque source, drag law and push convention; they differ in the rigid-body and contact description. Noise is white and Gaussian on pitch and pitch rate only; position and speed are noise-free. Forces act in the sagittal plane and the contact model has no steering or roll controller. The force acts at the combined centre of mass, which moves with the load.')
    p('Use. The diagnosis supports reporting simulator disagreement as a critical-force difference with an identified cause. The boundary measurement supports the same practice for controller comparisons: for this model family, exact-load redesign raises the push the robot survives by up to about a fifth at the higher mounting heights, a change that fixed-force grids almost never register and that noisy trials near the boundary register as fewer falls. This holds in both plants, which differ in rigid-body and contact modelling but share everything else. Hardware remains a separate source of evidence; the integrated manuscript gives the validation plan.')
    h('References')
    for number, label, url in [(1, 'MuJoCo XML Reference: inertial properties, contact parameters and solver options.', 'https://mujoco.readthedocs.io/en/stable/XMLreference.html'),
                               (2, 'MuJoCo Computation: soft contact model, friction cones and numerical integration.', 'https://mujoco.readthedocs.io/en/stable/computation/'),
                               (3, 'Q. McNemar. Note on the sampling error of the difference between correlated proportions or percentages. Psychometrika 12(2), 153-157, 1947.', 'https://doi.org/10.1007/BF02295996')]:
        b.append(('reference', f'[{number}] {label}', url))
    pf = {e: next(x for x in summary['exact_boundary']['margin_points'][e] if x['mass_kg'] == 1.15 and x['height_m'] == .26 and x['profile'] == 'direct_0ms' and x['controller'] == 'fixed') for e in ['native_0p25ms']}
    pa = {e: next(x for x in summary['exact_boundary']['margin_points'][e] if x['mass_kg'] == 1.15 and x['height_m'] == .36 and x['profile'] == 'direct_0ms' and x['controller'] == 'exact') for e in ['native_0p25ms']}
    assert abs(pf['native_0p25ms']['margin_n']-pa['native_0p25ms']['margin_n']) < .1 and abs(pf['native_0p25ms']['falls']-pa['native_0p25ms']['falls']) <= 2
    assert len({q['det'][k]['prevented']-by_class_prev(n, k) for k in q['det']}) == 1  # "in every plant"
    return TITLE, number_blocks(b)
