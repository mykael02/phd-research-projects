"""Export payload-conditioned CARE gains without modifying the nominal config."""
import hashlib,json
from pathlib import Path
import numpy as np
import scipy
from scipy.linalg import solve_continuous_are,expm
ROOT=Path(__file__).resolve().parent
BASE=json.loads((ROOT/'config.json').read_text());PROTOCOL=json.loads((ROOT/'comparison_protocol.json').read_text())
def key(m,h):return f'{m:.4f}/{(.18 if m==0 else h):.4f}'
def parameters(m,h):
    p=dict(BASE['parameters']);m0,l0,I0=p['body_mass'],p['length'],p['body_inertia'];M=m0+m;L=l0 if m==0 else (m0*l0+m*h)/M
    p.update(body_mass=M,length=L,body_inertia=I0 if m==0 else I0+m0*(l0-L)**2+m*(.08**2+.06**2)/12+m*(h-L)**2)
    return p
def linear(p):
    m,l,I=p['body_mass'],p['length'],p['body_inertia'];a=p['base_mass']+m+2*p['wheel_inertia']/p['radius']**2;b=m*l;d=I+m*l*l
    mass=np.array([[a,b],[b,d]]);A=np.zeros((4,4));A[0,1]=A[2,3]=1
    A[[1,3],1]=np.linalg.solve(mass,[-p['translation_drag'],0]);A[[1,3],2]=np.linalg.solve(mass,[0,m*p['gravity']*l]);A[[1,3],3]=np.linalg.solve(mass,[0,-p['rotation_drag']]);B=np.zeros((4,1));B[[1,3],0]=np.linalg.solve(mass,[1/p['radius'],-1]);return A,B
def sampled_radius(p,K):
    A,B=linear(p);H=np.zeros((5,5));H[:4,:4]=A;H[:4,4:]=B;D=expm(H*.005);return float(max(abs(np.linalg.eigvals(D[:4,:4]-D[:4,4:]@np.array(K).reshape(1,4)))))
def design(m,h):
    p=parameters(m,h);A,B=linear(p);Q=np.diag(PROTOCOL['q']);R=np.array([[PROTOCOL['r']]]);P=solve_continuous_are(A,B,Q,R);K=np.linalg.solve(R,B.T@P)[0]
    residual=float(np.linalg.norm(A.T@P+P@A-P@B@np.linalg.solve(R,B.T)@P+Q));assert residual<1e-7
    if m==0:
        np.testing.assert_allclose(K,BASE['lqr_gain'],rtol=1e-10);K=np.array(BASE['lqr_gain'])
    rho=sampled_radius(p,K);assert rho<1
    return dict(gain=K.tolist(),care_residual=residual,sampled_radius=rho,parameters=p)
def assumed(variant,m,h):
    if variant=='mass_low':m*=.8
    elif variant=='mass_high':m*=1.2
    elif variant=='height_low':h-=.04
    elif variant=='height_high':h+=.04
    elif variant=='fixed':m,h=0,.18
    return round(m,8),round(h,8)
def main():
    pairs={(round(i*.05,8),round(.1+j*.01,8)) for i in range(31) for j in range(31)}
    for m in PROTOCOL['test_masses_kg']:
        for h in PROTOCOL['test_heights_m']:
            for v in PROTOCOL['variants']:pairs.add(assumed(v,m,h))
    bank={key(m,h):design(m,h) for m,h in sorted(pairs)}
    provenance={'protocol_sha256':hashlib.sha256((ROOT/'comparison_protocol.json').read_bytes()).hexdigest(),'design_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'numpy':np.__version__,'scipy':scipy.__version__,'q':PROTOCOL['q'],'r':PROTOCOL['r']}
    out=ROOT/'results/comparison';out.mkdir(parents=True,exist_ok=True)
    diagnostics=[]
    for m in PROTOCOL['test_masses_kg']:
        for h in PROTOCOL['test_heights_m']:
            for v in PROTOCOL['variants']:
                em,eh=assumed(v,m,h);diagnostics.append(dict(mass_kg=m,height_m=h,variant=v,estimated_mass_kg=em,estimated_height_m=eh,sampled_radius=sampled_radius(parameters(m,h),bank[key(em,eh)]['gain'])))
    (out/'gain_design.json').write_text(json.dumps({'metadata':provenance,'bank':bank,'test_linear_diagnostics':diagnostics},indent=2)+'\n')
    js='/* Precomputed CARE solutions: exact lookup at supported settings, no interpolation. */\nconst PAYLOAD_GAINS = '+json.dumps({k:v['gain'] for k,v in bank.items()},separators=(',',':'))+';\nif(typeof module!=="undefined"&&module.exports)module.exports=PAYLOAD_GAINS;\n'
    (ROOT/'visualisation/payload-gains.js').write_text(js)
    print(f'Exported {len(bank)} gain designs; Q/R unchanged; all matched unsaturated sampled linear models stable.')
if __name__=='__main__':main()
