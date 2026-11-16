# Sparse RGB human motion research starter

A small, CPU-runnable starting point for research on **dynamic human motion from sparse RGB observations in cluttered manufacturing environments**. The first experiment tests the data and evaluation pipeline: generate moving 3D landmarks, project them into two known cameras, corrupt the 2D observations, triangulate, and evaluate against separate ground truth.

**This pilot starts from 2D keypoints, not RGB pixels.** It does not detect humans, reconstruct a manufacturing scene, train a model, or establish research novelty. Independent random keypoint dropout is a controllable missing-data test, not a simulation of real clutter. No manufacturing dataset, GPU, weights, account or paid service is required.

## Run the first experiment

Use Python 3.11 or newer; the checked environment was Python 3.12.14 and NumPy 2.3.5 on CPU. NumPy is the only runtime dependency; tests use Python's `unittest`. Commands run from the repository root.

```bash
python -m venv .venv
# Linux/macOS:
source .venv/bin/activate
# Windows PowerShell instead:
# .venv\Scripts\Activate.ps1
python -m pip install -e .
python -m unittest discover -s tests -v
python -m sparse_rgb pilot --config configs/pilot.toml --output runs/pilot
python -m sparse_rgb validate runs/pilot/dataset/manifest.json
```

The initial package install needs access to a Python package index. In an environment where NumPy 2.3.5 is already installed, you can run without installation:

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
PYTHONPATH=src python -m sparse_rgb pilot --config configs/pilot.toml --output runs/pilot
```

For PowerShell, set `$env:PYTHONPATH = "src"` first, then use the same `python` commands without the `PYTHONPATH=src` prefix. Every run requires a **new output directory**. An existing directory is rejected rather than overwritten. A failed run can leave partial artifacts; use a new name after correcting its inputs.

## One pilot, two conditions

The committed configurations use the same 120-frame, 30 Hz, 15-joint synthetic reach and two calibrated 1280x720 pinhole cameras. The baseline performs confidence-weighted DLT triangulation independently at each time and joint. It rejects insufficient views, near-parallel rays, rank-deficient solutions and negative depth. It does not use ground truth, fill gaps, smooth motion or reject outliers robustly.

```bash
python -m sparse_rgb pilot --config configs/clean.toml --output runs/clean
python -m sparse_rgb pilot --config configs/pilot.toml --output runs/noisy
```

Reference results from the checked CPU environment (seed 42):

| Condition | MPJPE on reconstructed joints | Coverage | PCK at 50 mm, counting misses | Reprojection RMSE | Velocity error |
|---|---:|---:|---:|---:|---:|
| Clean, no dropout | <0.000001 mm | 100% | 100% | <0.000001 px | <0.000001 mm/s |
| 1 px Gaussian noise, 15% independent dropout per view/joint | 9.206 mm | 73.28% | 73.28% | 0.721 px | 390.10 mm/s |

These are pipeline sanity checks, not real-world accuracy claims. With two cameras, each joint needs both observations; independent 15% dropout predicts roughly `0.85² = 72.25%` coverage. A particular random draw differs. The large noisy velocity error demonstrates the temporal instability of an unsmoothed frame-wise estimator even when position error is small. Small floating-point differences across platforms are expected.

## Files and artifacts

| Path | Purpose |
|---|---|
| `configs/` | Explicit clean and noisy pilot settings; unknown keys fail |
| `schemas/manifest.schema.json` | Dataset and camera metadata contract |
| `src/sparse_rgb/data.py` | Metadata, shape, calibration, missingness and time validation |
| `src/sparse_rgb/synthetic.py` | Seeded landmark and camera observation generator |
| `src/sparse_rgb/baseline.py` | `Baseline.predict(Observations) -> T,J,3` interface and runnable DLT |
| `src/sparse_rgb/metrics.py` | Position, coverage, threshold, reprojection and velocity metrics |
| `src/sparse_rgb/cli.py` | `pilot`, `run` and `validate` commands; provenance and artifact hashes |
| `tests/` | Synthetic numerical, failure-case and end-to-end tests |
| `docs/dataset.md` | Array schema, camera conventions and real-data adapter requirements |
| `docs/research-plan.md` | Research questions, next experiments and unimplemented extensions |
| `.github/workflows/tests.yml` | CPU tests and both pilot commands on Python 3.11 and 3.12 |

Each pilot creates `dataset/manifest.json`, `dataset/observations.npz`, `dataset/ground_truth.npz`, `predictions.npz`, `metrics.json`, a copy of `config.toml`, and `run.json`. The run record includes resolved config, Python/NumPy versions, source hash, git revision/dirty state when available, and input/output SHA-256 hashes. Identical configs reproduce arrays and metrics in the same environment; timestamps, absolute paths and environment metadata in `run.json` vary. Synthetic datasets are generated on demand; no data or weights are committed.

`mpjpe_mm` and velocity error use only valid reconstructed ground-truth joints/pairs. Always read them alongside `coverage`. `pck_with_misses` uses **all ground-truth-valid joints** as its denominator and treats absent predictions as failures. `pck_on_reconstructed` is conditional on successful reconstruction. Errors are in the calibrated world frame with no root, rigid or scale alignment. Empty error sets are JSON `null`, never a misleading zero. [Full metric definitions](docs/dataset.md#evaluation).

## Use your own extracted observations

Follow [the dataset contract](docs/dataset.md), then:

```bash
python -m sparse_rgb validate data/my_sequence/manifest.json
python -m sparse_rgb run data/my_sequence/manifest.json --config configs/pilot.toml --output runs/my_sequence
```

The `run` command consumes the supplied arrays and does not generate synthetic data; its `[synthetic]` settings are retained in the config snapshot but unused. Set `ground_truth_file` to `null` when there is no reference 3D motion: the report then contains prediction fraction and reprojection diagnostics, while ground-truth metrics are `null`. Low reprojection error alone cannot establish 3D accuracy. RGB path metadata is optional and is not loaded or checked for image existence by this pilot.

## Runnable versus planned

**Runnable now:** deterministic synthetic data, calibrated sparse-view triangulation, input validation, optional GT evaluation, reproducible run artifacts and tests.

**Planned:** RGB pose detection, tracking/identity association, real clutter data, VGGT/MASt3R geometry adapters, semantic representations, temporal estimation, physics-grounded constraints, training and cross-environment evaluation. None of these are implemented or downloaded. The baseline interface and explicit coordinate contract give later adapters a stable evaluation target; [the research plan](docs/research-plan.md) specifies what each would need to demonstrate.
