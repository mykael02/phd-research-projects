# Balancing robot research package

Author: **Michael Agbo**. Journal-neutral simulation manuscript draft, 12 November 2026. No affiliation has been supplied. This is not a submitted or peer-reviewed paper.

## Read and explore

- **New: [Fixed versus payload-aware LQR](../results/comparison/report.html)** and
  [PDF supplement](output/pdf/comparison_report.pdf): 11,016 reserved test runs,
  six figures, same Q/R, paired outcomes and explicit load-estimate errors.
  See [comparison methods](../COMPARISON.md).


- **New: [Step 4 payload report](../results/payload/report.html)** and
  [PDF supplement](output/pdf/payload_report.pdf): 4,176 trials, seven figures,
  rigid-body mass/COM/inertia updates, frozen gains, sensing comparisons and
  a separate 2,016-trial integration audit. See [payload methods](../PAYLOAD.md).


- **New: [Step 3 sensing and delay report](../results/sensing/report.html)** and
  [PDF supplement](output/pdf/sensing_report.pdf): 8,340 unique trials, six ideal
  baselines, five new PDF/SVG/600 dpi figures, seeded noise, causal packet delay
  and a two-state Kalman estimator. See [sensing methods](../SENSING.md).

- [Manuscript PDF](output/pdf/manuscript.pdf): model, methods, paired results, integral-state mechanism, limitations and literature references.
- [Technical report](output/pdf/technical_report.pdf): detailed counts, numerical audit, 3D visualisations, complete seven-slice outcome atlas and proposed sensing/payload studies.
- [Live 3D simulation and replay](../visualisation/index.html): run, push left/right, adjust force (0-30 N), initial tilt and torque limit. Each click applies a 0.2 s pulse to both upright robots. Orbit, zoom, change speed and export PNG. Choose Recorded experiments for the original three cases and timeline scrubbing.
- [3D experiment map](../visualisation/experiments.html): all 1,638 original records, with controller, outcome and torque filters.
- [Animated 3D preview](../visualisation/robot-3d-preview.gif): nominal recorded trial, 10 Hz display, no WebGL required.
- [Original 2D study](../results/robustness/report.html): exact outcome maps and live simulation replays.

Open the HTML files directly in a browser. All libraries and data are local; no server or account is required. Interactive 3D requires WebGL. The browser access host was unavailable during creation, so interactive rendering was not visually verified. Data and UI-logic checks are separate from visual verification. PDF pages, static figures and selected animation frames were rendered and visually inspected.

The 3D robot displays existing planar dynamics. It does not add yaw, roll, lateral control, collision dynamics or hardware realism. Body-shell and track dimensions are illustrative. A fall freezes the last recorded pose. The experiment plot contains discrete points, with no interpolated stability boundary.

Live mode shares `model.js` and `config.js` with the 2D lab and runs at 200 Hz for up to 20 simulated seconds. Pause preserves the remaining pulse; another push replaces it. Changing initial tilt or torque resets the run. Force directions follow the ground X axis even when the camera is rotated. Manual runs do not alter the archived manuscript results. The PDFs describe the original study and recorded visualisations; live force interaction was added afterwards.

## Figures and editable source

`figures/` contains 34 figures, each in vector PDF, SVG and 600 dpi PNG. `figure_manifest.json` supplies captions for the 16 original figures. Eight manuscript figures cover outcome counts, maps, paired outcomes and metrics, trajectories, integral offsets, ablation and tolerance sensitivity. Seven supplementary figures cover every torque slice. The additional 3D figure displays selected state trajectories. The five Step 3 figures have captions in `sensing_figure_manifest.json`; the seven Step 4 figures use `payload_figure_manifest.json`. Six controller-comparison figures use `comparison_figure_manifest.json`.

`output/latex/` contains editable LaTeX with vector figures. The delivered PDFs are built using ReportLab; no TeX compiler was installed, so the LaTeX sources have not been compiled or layout-verified. Run a TeX compiler from that directory so relative figure paths resolve. Source prose also lives in `manuscript_content.py` and `data/*_content.json`.

## Evidence and interpretation

| Controller | Recovered | Upright, unsettled | Fallen |
|---|---:|---:|---:|
| Original PID-type | 215 | 220 | 384 |
| Original LQR | 357 | 0 | 462 |
| PID-type, integral removed | 433 | 0 | 386 |

Each row evaluates the same 819 deterministic conditions. The integral ablation is post hoc, with other gains unchanged. All 220 original unsettled cases remain unsettled at 30 seconds. All 705 boundary-adjacent cases retain their outcome when the RK4 step is halved while preserving the 200 Hz controller rate. These are numerical results, not hardware measurements or deployment success probabilities.

Before submission, develop a specific novelty claim, matched controller tuning,
independent model/hardware validation and a clearly defined contribution. Step 3
provides a sensing/delay study; Step 4 adds a separate frozen-gain payload study. The original
manuscript and its dataset remain frozen. The author must review claims, citations,
declarations and AI-assistance disclosure before integrating the new results.

## Reproduce

Use a compatible Python interpreter; the exact version is recorded in `data/analysis.json`. Installed build dependencies are pinned in `requirements.txt`. The report builder uses Windows Times New Roman and Arial fonts in `C:/Windows/Fonts`; adapt `setup_fonts()` to licensed equivalents on another platform. Figures use bundled DejaVu Sans.

From the **project root**:

```powershell
python -m venv publication/.venv
publication/.venv/Scripts/python.exe -m pip install -r publication/requirements.txt
publication/.venv/Scripts/python.exe publication/analyse_study.py
publication/.venv/Scripts/python.exe publication/audit_integration.py
publication/.venv/Scripts/python.exe publication/make_figures.py
publication/.venv/Scripts/python.exe publication/build_3d.py
publication/.venv/Scripts/python.exe publication/render_3d_preview.py
publication/.venv/Scripts/python.exe publication/build_reports.py
publication/.venv/Scripts/python.exe publication/verify_package.py
```

The first script checks the model source hash and exact original metric reproduction before extending trials. Preserve the archived baseline in `results/robustness/results.json`. The audit and full analyses can take several minutes. `make_figures.py` writes 15 main/atlas entries; run `build_3d.py` afterwards to append the 3D entry before building reports.

Run tests from the same project root:

```powershell
python -m unittest discover -s tests -v
node tests/test_ui.js
node tests/test_study_ui.js
node tests/test_3d.js
node tests/test_3d_ui.js
node tests/test_live_3d.js
node tests/test_sensing.js
```

UI tests use mocks and do not verify WebGL pixels. Original model/configuration hashes are preserved with the baseline. `SHA256SUMS.json` records package files at release. `package_release.py` refreshes the hashes and ZIP, excluding virtual environments, caches and intermediate QA images.

Three.js 0.160.1 and Plotly.js GL3D 3.3.0 are vendored under their included MIT licences. Sources: https://www.npmjs.com/package/three/v/0.160.1 and https://github.com/plotly/plotly.js/tree/v3.3.0. Download URLs and byte hashes are recorded in `../visualisation/vendor/DEPENDENCIES.json`.

## Follow-up supplement and manuscript version 2

```powershell
publication/.venv/Scripts/python.exe publication/build_followup_report.py
publication/.venv/Scripts/python.exe publication/build_integrated_manuscript.py
publication/.venv/Scripts/python.exe publication/verify_followup.py
```

`build_followup_report.py` writes eight `fu_fig*` figures, `results/mujoco_followup/summary.json`,
`cells.csv` and the supplement (prose in `followup_text.py`, guarded by assertions on the data;
tables and figure references are numbered automatically). `build_integrated_manuscript.py` asserts
every archived number and writes `output/pdf/manuscript_v2.pdf`, the LaTeX source and HTML; prose is in
`integrated_manuscript_content.py`. `verify_followup.py` checks source hashes, figures, links, key numbers,
stale phrases and margins, and renders contact sheets to `_qa/` for visual review. `manuscript.pdf`
(version 1) is left unchanged.
