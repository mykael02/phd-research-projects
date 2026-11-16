"""Archive the complete local project, excluding environments and QA intermediates."""
import hashlib
import json
from pathlib import Path
import zipfile
ROOT=Path(__file__).resolve().parents[1];HERE=ROOT/'publication'

def main():
    destination=HERE/'output/Balancing_robot_Michael_Agbo_research.zip'
    files=[p for p in ROOT.rglob('*') if p.is_file() and not any(x in {'.venv','__pycache__','_qa','.git'} for x in p.relative_to(ROOT).parts) and p.suffix.lower()!='.zip' and p.name!='SHA256SUMS.json']
    runtime_files={'mujoco_lab/output/viewer.log','mujoco_lab/output/viewer-error.log','mujoco_lab/output/viewer_status.json'}
    files=[p for p in files if p.relative_to(ROOT).as_posix() not in runtime_files]
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
    manifest=HERE/'SHA256SUMS.json';manifest.write_text(json.dumps(hashes,indent=2)+'\n');files.append(manifest)
    with zipfile.ZipFile(destination,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
        for p in files:archive.write(p,Path('Balancing robot')/p.relative_to(ROOT))
    with zipfile.ZipFile(destination) as archive:assert archive.testzip() is None
    print(f'Archived {len(files)} files; {destination.stat().st_size/1024/1024:.1f} MiB; {destination}')

if __name__=='__main__':main()
