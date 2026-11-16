# Step 3: noisy sensing, packet delay and estimation

Implemented 12 November 2026. Author: Michael Agbo. Simulation-only research.

## Use the live lab

Open [visualisation/index.html](visualisation/index.html), keep **Live simulation**,
and choose a controller-input mode:

- **Perfect state:** original baseline; noise and delay settings are bypassed.
- **Noisy measurements:** controllers receive the delayed observation packet directly.
- **Kalman-filtered measurements:** packet-time tilt and angular rate are filtered;
  delayed ideal position and velocity complete the controller state vector.

Adjust angle noise, gyro noise, measurement delay and seed. Changes reset the run.
Click Run, then push left/right. A solid body shows the true state and the purple
outline shows the controller estimate. Select tilt, angular rate, position or speed
for the live traces. Export run JSON to preserve the settings, packet timestamps,
truth/measurement/estimate history and manual force events. Matching seeds produce
matching angle/gyro innovation sequences for both controllers, until either stops.

The offline study and the 3D lab share `visualisation/sensing.js`, `model.js` and
`config.js`. Rendering and plot updates never draw new random samples. The original
2D page and archived nominal results are unchanged.

## Sensor contract

State order is `[position_m, speed_m_s, tilt_rad, angular_rate_rad_s]`.
Samples arrive at 200 Hz. Independent Gaussian innovations are added to tilt and
angular rate; UI and protocol standard deviations are expressed in degrees and
degrees/second, then converted to radians. Position and speed are ideal encoder-like
channels. There is no accelerometer-force model, bias, drift, dropout or quantisation.

All four channels are delayed by an integer number D of control periods. At step k,
the delivered packet is `max(0,k-D)`. A measured initial packet is available at startup
and held while the delay pipeline fills. That convention is explicit; there is no
negative-time truth history and no future observation. Repeated startup packets do
not trigger repeated filter updates. A fallen robot stops acquisition and motion.

## Estimator contract

The two-state linear Kalman filter uses tilt and angular rate, with
`F=[[1,dt],[0,1]]`, `H=I`, and independent measurement covariance
`R=diag(angle_sigma_rad^2, gyro_sigma_rad_s^2)`.
Process covariance is `Q=sigma_a^2 G G^T`, where
`G=[dt^2/2,dt]^T` and `sigma_a=50 rad/s^2` is a fixed per-interval random-acceleration
standard deviation. It is not a continuous-time spectral-density parameter.

The first estimate is the first noisy observation, with initial covariance R.
Each new packet triggers one prediction and two scalar measurement corrections.
When both noise levels are zero, the estimator returns the exact packet.
No plant truth, external force or commanded torque enters this estimator. Its
constant-angular-rate model is deliberately simple; no nonlinear optimality or
closed-loop stability guarantee is claimed.

**The filter does not compensate delay.** Its estimate describes the packet time.
Current-state error and packet-time error are reported separately. This prevents
noise reduction from being mistaken for accurate knowledge of the present state.

## Experiments and outputs

The frozen [protocol](sensing_protocol.json) defines three scenarios, independent
angle-only and gyro-only sweeps, a combined noise/delay grid, fixed controller gains,
30 seeds per noisy setting and one run per deterministic setting. Overlapping cells
are deduplicated. The result is 8,340 unique sensing trials and six ideal baselines.
These conditions were chosen with knowledge of the earlier study, so this is an
exploratory study, not an independent held-out benchmark or a preregistered experiment.

- [HTML report](results/sensing/report.html)
- [PDF supplement](publication/output/pdf/sensing_report.pdf)
- [All trial records](results/sensing/results.json)
- [Per-cell summaries](results/sensing/summary.csv)
- Five new `publication/figures/step3_*` figures in PDF, SVG and 600 dpi PNG
- Editable LaTeX in `publication/output/latex/sensing_report.tex` (not compiled here)

The original strict final-dwell recovery criterion is retained. Persistent noise
can keep a robot upright while preventing recovery under that definition. Wilson
intervals apply only to the repeated seeds of a fixed noisy condition. No interval
is attached to deterministic cells. Cross-setting totals are descriptive inventories,
not deployment success probabilities. RMSE is computed up to termination, so compare
equal-horizon groups; the report explicitly identifies groups with complete 10 s runs.

## Reproduce and verify

```powershell
node run_sensing.js
publication/.venv/Scripts/python.exe publication/build_sensing_report.py
node tests/test_sensing.js
node tests/test_3d_ui.js
node tests/test_live_3d.js
python -m unittest discover -s tests -v
```

Dependencies use the existing publication environment. Source hashes are recorded
in the study snapshot. Validation covers no-noise parity with the old controllers,
selected archived Python outcomes, an independent dense-matrix Kalman reference,
causal delay, startup holding, seeded repeatability and covariance sanity. UI tests
use renderer mocks; interactive WebGL pixels could not be inspected because the
browser access host was unavailable.

Step 4 is now implemented in a separate [payload study](PAYLOAD.md), including
an exploratory sensing-plus-payload subset. A delay predictor, matched gain tuning,
held-out conditions and hardware validation remain future work. This Step 3 dataset
and report retain their original scope and results.

References: [Welch and Bishop, Kalman filter introduction (2004 version)](https://www.cs.utexas.edu/~pstone/Courses/393Rfall15/readings/Welch%2BBishop-TR-95.pdf),
[Music et al., networked balancing control](https://arxiv.org/abs/1812.03071v2).
