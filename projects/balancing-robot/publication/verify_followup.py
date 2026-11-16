"""Integrity checks for the follow-up supplement and manuscript version 2.

Checks source hashes of the new datasets, figure files in all formats, local
links in the HTML versions, PDF text (key numbers, no missing glyphs, margins),
and renders contact sheets for visual review. Run from the project root.
"""
import hashlib
import json
from pathlib import Path
import re
from urllib.parse import unquote
import fitz
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
HERE = ROOT/'publication'


def main():
    report = {}
    for name in ['results/mujoco_disagreement/study.json', 'results/mujoco_disagreement/addendum.json',
                 'results/mujoco_noisy_payload/study.json', 'results/mujoco_noisy_payload/controls.json',
                 'results/mujoco_noisy_payload/exact_boundary.json']:
        data = json.loads((ROOT/name).read_text())
        for f, digest in data['metadata']['source_sha256'].items():
            assert hashlib.sha256((ROOT/f).read_bytes()).hexdigest() == digest, (name, f)
    summary = json.loads((ROOT/'results/mujoco_followup/summary.json').read_text())
    noisy = summary['noisy']['engines']
    assert (noisy['planar_port']['fixed_falls'], noisy['planar_port']['exact_falls']) == (508, 334)
    assert summary['noisy']['persists'] is True and summary['noisy']['port_mismatches'] == 0
    pc = summary['exact_boundary']['per_controller']['native_0p25ms']
    assert (pc['fixed']['noise_caused'], pc['exact']['noise_caused']) == (241, 254)

    manifest = json.loads((HERE/'followup_figure_manifest.json').read_text())
    for entry in manifest:
        for ext in ['png', 'svg', 'pdf']:
            assert (HERE/'figures'/f"{entry['name']}.{ext}").is_file(), (entry['name'], ext)
    report['figures'] = len(manifest)

    links = 0
    for page in [ROOT/'results/mujoco_followup/report.html', ROOT/'results/manuscript_v2/manuscript.html']:
        for link in re.findall(r'(?:href|src)="([^"]+)"', page.read_text(encoding='utf-8')):
            if link.startswith(('http:', 'https:', '#', 'data:')): continue
            assert (page.parent/unquote(link.split('?')[0].split('#')[0])).is_file(), (page, link)
            links += 1
    report['local_links'] = links

    expected = {'manuscript_v2': ['508', '334', '511', '343', '241', '254', '18.2271', '18.1887', '234', '44.6', '20.9%'],
                'mujoco_followup_report': ['508', '334', '511', '343', '3,240', '18.2271', '18.1887', '241', '254', '+20.9%']}
    for name, numbers in expected.items():
        doc = fitz.open(HERE/'output/pdf'/f'{name}.pdf')
        text = '\n'.join(p.get_text() for p in doc)
        assert 'Michael Agbo' in text and '�' not in text, name
        missing = [x for x in numbers if x not in text]
        flat = ' '.join(text.split())
        stale = [w for w in ['three fifths', 'two of three masses', 'independently assembled', 'falls at every force from', 'about two fifths'] if w in flat]
        assert not stale, (name, stale)
        assert not missing, (name, missing)
        qa = HERE/'_qa'/f'final-{name}'
        qa.mkdir(parents=True, exist_ok=True)
        for old in qa.glob('*.png'): old.unlink()
        for i, page in enumerate(doc):
            for w in page.get_text('words'):
                assert w[0] >= 30 and w[2] < page.rect.width-25 and w[1] > 25 and w[3] < page.rect.height-20, (name, i+1, w[4])
            page.get_pixmap(matrix=fitz.Matrix(1.5, 1.5)).save(qa/f'page-{i+1:02}.png')
        for start in range(0, len(doc), 4):
            sheet = Image.new('RGB', (1200, 1700), '#d9dde3'); draw = ImageDraw.Draw(sheet)
            for j in range(start, min(start+4, len(doc))):
                im = Image.open(qa/f'page-{j+1:02}.png'); im.thumbnail((575, 810))
                x, y = (j-start) % 2*600+10, (j-start)//2*850+25
                sheet.paste(im, (x, y)); draw.text((x, y-18), f'{name} p{j+1}', fill='black')
            sheet.save(qa/f'contact-{start//4+1}.jpg')
        report[name] = dict(pages=len(doc), words=len(text.split()),
                            sha256=hashlib.sha256((HERE/'output/pdf'/f'{name}.pdf').read_bytes()).hexdigest())
    (HERE/'data/followup_verification.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    print('Hashes, figures, links, PDF text and margins passed. Rendered pages still need visual inspection.')


if __name__ == '__main__':
    main()
