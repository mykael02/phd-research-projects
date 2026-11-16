"""Build manuscript version 2: the integrated, focused-contribution manuscript.

Version 1 (manuscript.pdf, manuscript_content.py) and every archived dataset
are left untouched. Each archived number quoted in the prose is asserted
against its source file here, so the text cannot drift from the data.
"""
from collections import Counter
import hashlib
import html
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
from build_reports import setup_fonts, build_pdf, build_tex
import integrated_manuscript_content as content

DATE = '13 November 2026'


def read(path):
    return json.loads((ROOT/path).read_text(encoding='utf-8'))


def archived():
    """Load archived evidence and assert every count the prose states."""
    analysis = read('publication/data/analysis.json')
    assert Counter((r['controller'], r['outcome']) for r in analysis['baseline']) == {
        ('pid', 'recovered'): 215, ('pid', 'unsettled'): 220, ('pid', 'fallen'): 384, ('lqr', 'recovered'): 357, ('lqr', 'fallen'): 462}
    assert Counter(r['outcome'] for r in analysis['ablation']) == {'recovered': 433, 'fallen': 386}
    assert Counter(r['outcome'] for r in analysis['extension']) == {'unsettled': 220}
    tolerance = {(r['controller'], r['position_tolerance_m']): r['recovered'] for r in analysis['tolerance_sensitivity']}
    assert [tolerance['pid', t] for t in (.01, .04, .05)] == [61, 381, 435] and all(tolerance['lqr', t] == 357 for t in (.01, .02, .03, .04, .05, .06))

    sensing = read('results/sensing/summary.json')
    push = Counter()
    for r in sensing:
        if r['scenario'] == 'large_push':
            push[r['controller'], r['observer'], 'n'] += r['n']; push[r['controller'], r['observer'], 'fallen'] += r['fallen']
    assert [push['lqr', o, k] for o in ('direct', 'kalman') for k in ('n', 'fallen')] == [695, 271, 695, 258]
    nominal = {(r['controller'], r['observer'], r['delay_ms']): r for r in sensing
               if r['scenario'] == 'nominal' and r['angle_sigma_deg'] == 1 and r['gyro_sigma_deg_s'] == 4}
    for controller in ('pid', 'lqr'):
        assert all(nominal[controller, o, d]['unsettled'] == 30 for o in ('direct', 'kalman') for d in (0, 40))
        assert round(nominal[controller, 'direct', 0]['median_current_rmse_deg'], 3) == 1.005
        assert round(nominal[controller, 'kalman', 0]['median_current_rmse_deg'], 3) == .146
        assert round(nominal[controller, 'kalman', 40]['median_source_rmse_deg'], 3) == .158
        assert 7.7 <= nominal[controller, 'kalman', 40]['median_current_rmse_deg'] <= 7.9
    baseline = read('results/robustness/results.json')['records']
    assert all(r['outcome'] == 'recovered' for r in baseline if r['controller'] == 'lqr' and r['initial_tilt_deg'] == 0
               and abs(r['push_force_n']) == 20 and r['torque_limit_nm'] == .7)

    test = read('results/comparison/test.json')['records']
    labels = Counter((r['phase'], r['variant'], r['outcome']) for r in test)
    assert [labels['ideal_test', v, o] for v in ('fixed', 'exact') for o in ('recovered', 'fallen')] == [166, 50, 166, 50]
    assert [labels['sensing_test', v, 'fallen'] for v in ('fixed', 'exact', 'mass_low', 'mass_high', 'height_low', 'height_high')] == [508, 334, 372, 304, 418, 259]
    assert not any(labels['sensing_test', v, 'recovered'] for v in ('fixed', 'exact'))
    paired = {(r['phase'], r['variant']): r for r in read('results/comparison/summary.json')['paired']}
    assert (paired['sensing_test', 'exact']['fixed_fall_to_upright'], paired['sensing_test', 'exact']['fixed_upright_to_fall']) == (186, 12)
    assert (paired['ideal_test', 'exact']['fixed_fall_to_upright'], paired['ideal_test', 'exact']['fixed_upright_to_fall']) == (0, 0)

    nominal_check = read('results/mujoco/summary.json')
    assert nominal_check['integrations'] == 150 and nominal_check['native_fine_outcome_agreement'] == 15 and nominal_check['reduced_trajectory_passes'] == 15
    assert round(nominal_check['native_fine_max_position_error_m']*1000, 2) == .89 and round(nominal_check['native_fine_max_pitch_error_deg'], 3) == .114
    extension = read('results/mujoco_extension/summary.json')
    assert (extension['coarse']['conditions'], extension['coarse']['outcome_agreement']) == (88, 88)
    assert (extension['zoom']['conditions'], extension['zoom']['outcome_agreement']) == (88, 86)
    assert extension['coarse']['integrations']+extension['zoom']['integrations'] == 708 and extension['payload']['integrations'] == 340
    assert extension['payload_primary_engine_agreement'] == {'fixed': 36, 'exact': 36}
    assert all(extension['payload_primary_controller_transitions'][e]['matrix'] == [[19, 0, 0], [0, 0, 0], [0, 0, 17]] for e in ('planar', 'native'))
    tilt = next(r for r in extension['paired_survivor_metrics'] if r['metric'] == 'peak_pitch_deg')
    assert tilt['n'] == 19 and round(tilt['planar_median_delta'], 2) == round(tilt['native_median_delta'], 2) == -.81
    payload_checks = read('results/mujoco_payload/study.json')
    assert sum(r['acceleration_checks'] for r in payload_checks['load_checks']) == 480 and len(payload_checks['reduced_checks']) == 20
    return dict(extension=extension, followup=read('results/mujoco_followup/summary.json'))


def write_html(blocks, manifest, title, target):
    parts = []
    for block in blocks:
        if block[0] == 'heading': parts.append('<h2>'+html.escape(block[1])+'</h2>')
        elif block[0] == 'paragraph': parts.append('<p>'+html.escape(block[1])+'</p>')
        elif block[0] == 'equation': parts.append('<p class="equation"><code>'+html.escape(block[1])+'</code></p>')
        elif block[0] == 'figure':
            name = block[1]
            parts.append('<figure><img src="../../publication/figures/'+name+'.svg" alt="'+html.escape(manifest[name]['caption'], quote=True)+'"><figcaption>'+html.escape(manifest[name]['caption'])+'</figcaption></figure>')
        elif block[0] == 'table':
            parts.append('<div class="table-wrap"><table><caption>'+html.escape(block[1])+'</caption><tr>'+''.join('<th>'+html.escape(str(x))+'</th>' for x in block[2])+'</tr>'+''.join('<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in row)+'</tr>' for row in block[3])+'</table></div>')
        elif block[0] == 'reference': parts.append('<p>'+html.escape(block[1])+' <a href="'+block[2]+'">Source</a></p>')
    css = (ROOT/'results/mujoco/report.html').read_text(encoding='utf-8').split('<style>')[1].split('</style>')[0]
    nav = ''.join(f'<a href="{href}">{label}</a>' for href, label in [('../../publication/output/pdf/manuscript_v2.pdf', 'PDF manuscript'), ('../mujoco_followup/report.html', 'Follow-up supplement'), ('../../publication/output/pdf/manuscript.pdf', 'Version 1 (archived)')])
    target.write_text('<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+html.escape(title)+'</title><style>'+css+'</style></head><body><main><nav>'+nav+'</nav><p>MICHAEL AGBO / INTEGRATED MANUSCRIPT, VERSION 2</p><h1>'+html.escape(title)+'</h1>'+''.join(parts)+'</main></body></html>', encoding='utf-8')


def main():
    data = archived()
    manifest = {}
    for name in ['figure_manifest', 'sensing_figure_manifest', 'comparison_figure_manifest', 'mujoco_extension_figure_manifest', 'followup_figure_manifest']:
        manifest.update({r['name']: r for r in read(f'publication/{name}.json')})
    # Reused archived figures keep their images; their captions get the manuscript's vocabulary (copies, the manifests are untouched).
    notes = {'ext_fig01_boundary_zoom': ('the follow-up uses', 'the zoom uses', ' Rows labelled Native are the MuJoCo wheel-contact model.'),
             'step3_fig04_delay_estimation_error': (None, None, ' The panel labelled PID shows the PID-type controller of Section 4.'),
             'compare_fig02_paired_outcomes': (None, None, ' The panel labelled perfect sensing is exact state feedback.')}
    for name, (old, new, extra) in notes.items():
        caption = manifest[name]['caption']
        if old: assert old in caption; caption = caption.replace(old, new)
        manifest[name] = dict(manifest[name], caption=caption+extra)
    blocks = content.manuscript(data)
    for kind, *rest in blocks:
        if kind == 'figure': assert rest[0] in manifest and (HERE/'figures'/f'{rest[0]}.png').is_file(), rest[0]
    setup_fonts()
    (HERE/'_qa').mkdir(exist_ok=True)
    build_pdf(HERE/'output/pdf/manuscript_v2.pdf', content.TITLE, blocks, manifest, f'Integrated simulation manuscript, version 2 | {DATE}')
    build_tex(HERE/'output/latex/manuscript_v2.tex', content.TITLE, blocks, manifest)
    tex = HERE/'output/latex/manuscript_v2.tex'
    tex.write_text(tex.read_text(encoding='utf-8').replace(r'\date{12 November 2026}', r'\date{'+DATE+'}'), encoding='utf-8')
    (HERE/'data/manuscript_v2_content.json').write_text(json.dumps(blocks, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    out = ROOT/'results/manuscript_v2'
    out.mkdir(parents=True, exist_ok=True)
    write_html(blocks, manifest, content.TITLE, out/'manuscript.html')
    words = sum(len(b[1].split()) for b in blocks if b[0] == 'paragraph')
    print(json.dumps({'paragraph_words': words, 'figures': sum(b[0] == 'figure' for b in blocks), 'tables': sum(b[0] == 'table' for b in blocks),
                      'pdf_sha256': hashlib.sha256((HERE/'output/pdf/manuscript_v2.pdf').read_bytes()).hexdigest()}, indent=1))


if __name__ == '__main__':
    main()
