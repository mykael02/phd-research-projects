"""Check report/data/link integrity and render final PDFs for human visual QA."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
from urllib.parse import unquote
import fitz
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[1];HERE=ROOT/'publication'

def main():
    source=json.loads((ROOT/'results/robustness/results.json').read_text());data=json.loads((HERE/'data/analysis.json').read_text());audit=json.loads((HERE/'data/full_integration_audit.json').read_text())
    assert len(data['baseline'])==1638
    assert hashlib.sha256((ROOT/'balance.py').read_bytes()).hexdigest()==source['metadata']['source_sha256']['balance.py']
    assert Counter(r['outcome'] for r in data['extension'])=={'unsettled':220}
    assert Counter(r['outcome'] for r in data['ablation'])=={'recovered':433,'fallen':386}
    assert audit['audited']==audit['boundary_candidate_count']==705 and audit['disagreements']==0
    assert len(json.loads((HERE/'figure_manifest.json').read_text()))==16
    for file in [ROOT/'index.html',ROOT/'visualisation/index.html',ROOT/'visualisation/experiments.html']:
        for link in re.findall(r'(?:href|src)="([^"]+)"',file.read_text(encoding='utf-8')):
            if not link.startswith(('http:','https:','#','data:')):assert (file.parent/unquote(link.split('?')[0].split('#')[0])).is_file(),(file,link)
    summaries={}
    for name in ['manuscript','technical_report']:
        doc=fitz.open(HERE/'output/pdf'/f'{name}.pdf');qa=HERE/'_qa'/('final-'+name);qa.mkdir(exist_ok=True)
        text='\n'.join(p.get_text() for p in doc);assert 'Michael Agbo' in text;assert '\ufffd' not in text;assert all(x in text for x in ['215','220','384','357','462','433','386','705'])
        for i,page in enumerate(doc):
            page.get_pixmap(matrix=fitz.Matrix(1.5,1.5)).save(qa/f'page-{i+1:02}.png')
            for word in page.get_text('words'):assert word[0]>=30 and word[2]<page.rect.width-25 and word[1]>25 and word[3]<page.rect.height-20,(name,i,word)
        for start in range(0,len(doc),4):
            sheet=Image.new('RGB',(1200,1700),'#d9dde3');draw=ImageDraw.Draw(sheet)
            for j in range(start,min(start+4,len(doc))):
                im=Image.open(qa/f'page-{j+1:02}.png');im.thumbnail((575,810));x=(j-start)%2*600+10;y=(j-start)//2*850+25;sheet.paste(im,(x,y));draw.text((x,y-18),f'{name} p{j+1}',fill='black')
            sheet.save(qa/f'contact-{start//4+1}.jpg')
        summaries[name]={'pages':len(doc),'words':len(text.split()),'sha256':hashlib.sha256((HERE/'output/pdf'/f'{name}.pdf').read_bytes()).hexdigest()}
    gif=Image.open(ROOT/'visualisation/robot-3d-preview.gif');assert gif.n_frames==101
    for frame in [0,21,35,60,100]:gif.seek(frame);gif.convert('RGB').save(HERE/'_qa'/f'animation-{frame:03}.png')
    (HERE/'data/verification.json').write_text(json.dumps(summaries,indent=2)+'\n')
    print(json.dumps(summaries,indent=2));print('Data, hash, local-link, PDF text/margin and 101-frame animation checks passed. Rendered pages still require visual inspection.')

if __name__=='__main__':main()
