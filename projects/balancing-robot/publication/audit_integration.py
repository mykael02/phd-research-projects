"""Repeat every boundary-adjacent case with half the RK4 step, fixed control rate."""
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from run_robustness import audit_boundaries
if __name__=='__main__':
    source=json.loads((ROOT/'results/robustness/results.json').read_text())
    audit=audit_boundaries(source['metadata']['config'],source['records'],limit=len(source['records']))
    (ROOT/'publication/data/full_integration_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
    print(f"Audited {audit['audited']} boundary-adjacent cases; {audit['disagreements']} classification changes")
