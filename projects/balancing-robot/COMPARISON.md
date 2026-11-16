# Fixed versus payload-aware LQR

Author: Michael Agbo. 12 November 2026. Simulation comparison; no hardware claims.

## Try it in 3D

Open [the live lab](visualisation/index.html) and refresh the page. Select
**Fixed vs payload-aware LQR** in the control panel. Orange uses the original
nominal gain; teal uses an LQR designed for the selected payload. At zero payload
the gains are identical. Add a payload, run, and apply forces to compare them.

Both controllers receive the same force and paired sensor noise. They have the
same true robot parameters and torque limit. Choose Perfect state to isolate
controller differences from sensing. Changing load, sensing or design settings
resets the run.

Expand **Test an incorrect payload estimate** and uncheck **Use actual payload
for design** to change the mass and height assumed by the teal controller.
The real load remains controlled by Carry a payload. This supplies assumed
parameters; there is no automatic payload estimator or online learning.
Export run JSON includes both labels and gain vectors as well as true load,
assumed load, sensing, force events and trajectories. The original PID/fixed-LQR
comparison and archived experiment replay remain available.

## Design contract

All payload-aware gains solve the continuous algebraic Riccati equation at the
upright equilibrium with **Q=diag(12,2,120,2), R=1**, exactly the original cost
weights. The tuning budget is zero performance-based parameter searches.
Gains are fixed during each run; control is sampled at 200 Hz and saturated.
The bank has 949 exact designs covering every UI setting plus the study's
off-grid mismatch settings. Lookup does not interpolate. Nominal parameters
and earlier datasets are unchanged.

The true loaded COM determines the force moment arm. An erroneous assumed COM
affects only the gain, never the true plant or force application point. Local
sampled linear stability does not guarantee stability with saturation or delay.

## Prospective comparison

The [protocol](comparison_protocol.json) was saved before the run, but was not
externally preregistered. Sixteen development checks use previously studied
loads/conditions. Test loads (0.35, 0.65, 1.15 kg at 0.14, 0.26, 0.36 m), force
conditions and noise seeds are separated from development and the earlier grid.
They are reserved conditions in the same known model, not independent hardware.

- 1,296 ideal test trials: 216 conditions x six controller variants.
- 9,720 noisy test trials: 1,620 paired conditions/seeds x six variants.
- Six variants: nominal fixed; exact load; mass -20% and +20%; height -4 and +4 cm.
- Noisy profiles: direct/0 ms, Kalman/0 ms, Kalman/10 ms; SDs 0.75 deg and 3 deg/s.
- All 1,296 ideal trials were repeated with half-sized RK4 steps at unchanged
  control frequency. No outcome labels changed.
- Six selected ideal trajectories agree with an independent component-matrix
  calculation integrated with DOP853. This is numerical cross-validation,
  not independent multibody or hardware validation.

Exact-load redesign leaves all ideal outcome labels unchanged (166 recoveries,
50 falls). Noisy falls decrease from 508 to 334, comprising 186 prevented falls
and 12 introduced falls. Every noisy survivor is unsettled under the strict
original recovery dwell. Continuous metric comparisons therefore retain both
the fall counts and explicit mutual-survivor conditioning.

The height-overestimate variant performs better on the aggregate grid than the
exact model. That observation is not used to retune the controller, and does not
justify deliberately biasing a physical robot's payload estimate. This test set
is now known; future tuned comparisons need a fresh test set.

## Outputs and reproduction

- [Report and graphs](results/comparison/report.html)
- [PDF supplement](publication/output/pdf/comparison_report.pdf)
- [All test trials](results/comparison/test.csv)
- [Per-condition sensing counts and Wilson intervals](results/comparison/sensing_summary.csv)
- [Paired transitions](results/comparison/paired_transitions.csv)
- [Conditional paired metric summaries](results/comparison/paired_metrics.csv)
- Six `publication/figures/compare_*` figures in PDF, SVG and 600 dpi PNG
- Editable LaTeX source in `publication/output/latex/comparison_report.tex`

```powershell
publication/.venv/Scripts/python.exe design_payload_lqr.py
node tests/test_comparison.js
node run_comparison.js --development
node run_comparison.js --test
node audit_comparison.js
publication/.venv/Scripts/python.exe validate_comparison.py
publication/.venv/Scripts/python.exe publication/build_comparison_report.py
node tests/test_3d_ui.js
```

Source hashes, gain residuals, sampled linear diagnostics and numerical validation
records are in `results/comparison`. No test outcomes were used to select Q/R.
The live UI is checked with renderer mocks; browser access was unavailable for
visual WebGL inspection. Static figures and PDF pages are reviewed separately.
LaTeX source is supplied but not compiled.
