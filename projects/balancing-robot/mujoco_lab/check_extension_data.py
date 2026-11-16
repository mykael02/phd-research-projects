"""Audit archived trajectory bookkeeping, repeated endpoints and prior examples."""
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent.parent


def main():
    datasets={}
    total=0
    for phase in ['boundary','boundary_zoom','payload']:
        folder=ROOT/f'results/mujoco_{phase}'
        data=json.loads((folder/'study.json').read_text())
        arrays=np.load(folder/'trajectories.npz')
        assert len(arrays.files)==len(data['records'])
        for name,digest in data['metadata']['source_sha256'].items():
            assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
        for r in data['records']:
            a=arrays[r['id']]
            assert a.shape[1]==14 and np.isfinite(a).all()
            np.testing.assert_allclose(np.diff(a[:,0]),.005,atol=1e-12,rtol=0)
            assert abs(a[-1,0]-r['duration_s'])<1e-12
            assert np.max(np.abs(a[:,5]))<=r['torque_limit_nm']+1e-12
            assert not any(r['warnings'])
            assert np.count_nonzero(a[:400,6])==0 and np.count_nonzero(a[440:,6])==0
            if r['duration_s']==10 and r['push_force_n']:
                assert np.count_nonzero(a[:,6])==40
                assert np.all(a[400:440,6]==r['push_force_n'])
        total+=len(data['records'])
        datasets[phase]=(data,arrays)
    def key(r):return tuple(r[x] for x in ['engine','controller','physics_step_s','initial_tilt_deg','push_force_n','torque_limit_nm','mass_kg','height_m'])
    coarse,ca=datasets['boundary'];zoom,za=datasets['boundary_zoom']
    index={key(r):r for r in coarse['records']}
    repeats=[]
    for r in zoom['records']:
        if key(r) not in index:continue
        original=index[key(r)]
        assert np.array_equal(ca[original['id']],za[r['id']])
        repeats.append(dict(coarse_id=original['id'],zoom_id=r['id'],result='bitwise identical'))
    old=json.loads((ROOT/'results/comparison/test.json').read_text())
    payload,pa=datasets['payload'];regressions=[]
    for controller in ['fixed','exact']:
        r=next(r for r in payload['records'] if r['engine']=='planar' and r['physics_step_s']==.0005 and r['mass_kg']==.65 and r['height_m']==.26 and r['scenario']=='opposing_standard' and r['controller']==controller)
        prior=old['examples'][controller]
        state=np.array([x['state'] for x in prior['rows']])
        error=np.max(np.abs(pa[r['id']][:,1:5]-state),axis=0)
        assert max(error)<1e-4
        assert r['outcome']==prior['metrics']['outcome']
        regressions.append(dict(controller=controller,max_state_errors_native_units=error.tolist(),same_outcome=True,
                                note='Prior JavaScript RK4 uses 5 ms physics; current Python uses 0.5 ms, both at 200 Hz control.'))
    result={'checked_integrations':total,'finite_states_timing_limits_pulses_warnings_and_hashes':'passed',
            'repeated_coarse_zoom_trajectories':repeats,'archived_payload_example_checks':regressions}
    target=ROOT/'results/mujoco_extension/data_checks.json'
    target.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'integrations':total,'bitwise_repeats':len(repeats),'archived_example_checks':regressions},indent=2))


if __name__=='__main__':main()
