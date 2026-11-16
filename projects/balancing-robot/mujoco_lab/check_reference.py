"""Check the comparison runner against the existing public simulation entry point."""
import json
import numpy as np
from validate import CONFIG, OUT, trial
from balance import simulate


def main():
    original, rows = simulate('lqr', CONFIG, initial_degrees=5, push_force=3,
                              dt=.005, physics_substeps=5)
    current, trajectory = trial('planar', .001, 5, 3)
    states = np.array([r['state'] for r in rows])
    errors = np.max(np.abs(trajectory[:, 1:5] - states), axis=0)
    assert np.max(errors) < 1e-10
    assert abs(original['recovery_s'] - current['recovery_s']) < 1e-6
    result = {'condition': {'tilt_deg': 5, 'push_n': 3},
              'max_state_errors_native_units': errors.tolist(),
              'existing_simulate_recovery_s': original['recovery_s'],
              'comparison_runner_recovery_s': current['recovery_s'], 'result': 'passed'}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT/'reference_check.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
