# Research direction and next experiments

Working direction: sparse RGB observations and physics-grounded priors for dynamic human motion in cluttered manufacturing scenes. The scope here is intentionally one controlled experiment. It verifies camera conventions, triangulation, missingness, evaluation and repeatable artifacts before adding learned reconstruction or a continuous 4D representation.

## What the first pilot can answer

1. Does calibrated two-view triangulation recover noiseless landmarks in the correct units/frame? The clean configuration should reconstruct every joint to numerical precision.
2. What happens under pixel noise and missing observations? Run the noisy configuration and report MPJPE, coverage, PCK including misses and velocity error together.
3. Does temporal instability remain despite low position error? The velocity metric makes that limitation visible; temporal models have not yet been added.

For a first extension, copy the TOML config to test 0/1/3 px noise and 0/0.15/0.4 dropout over seeds 0–4. Keep one output directory per condition/seed and summarize mean and spread across seeds. This sweep and its aggregation are **proposed**, not an implemented command. Do not tune thresholds against held-out real test data.

## Where later components would connect

| Planned component | Proposed connection | Required controls before claiming benefit |
|---|---|---|
| RGB pose detector and identity tracking | Convert rectified images to `Observations` arrays with stable J ordering and one person identity | Freeze detector/version; record failures and confidence; evaluate detection separately from reconstruction |
| VGGT geometry | Adapter exports estimated cameras/geometry into explicit world-frame metadata; baseline/evaluator retain the same inputs/outputs | Verify camera transform direction, scale and units, coordinate handedness, calibration error, and dynamic-scene behavior against independent reference |
| MASt3R matching/geometry | Separate adapter for correspondences or geometric priors; joint identity still requires a human detector/tracker | Compare exact-camera baseline, estimated-camera baseline and optional refinement; record view selection, scale alignment and dynamic-scene failures |
| Semantic features (e.g. DINO-style representations) | A later feature extractor/cache plus a new estimator implementing `Baseline.predict`; design a versioned feature schema first | Use frozen feature/model revisions; ablate features while holding input frames, keypoints and evaluation split fixed |
| Temporal and physics-grounded constraints | A new estimator/refiner returns the same T,J,3 world-space predictions | Compare temporal-only, bone-length, joint-limit, floor/contact and collision constraints separately; estimate priors from training/calibration data, never test GT |
| Few-view/continuous 4D representation | Later model of time-varying motion/geometry with sampled joint predictions for the common evaluator | Test held-out times/views/persons/sites; separate interpolation from extrapolation; report motion failure and coverage |

These are integration proposals, not implemented APIs for those models. The only estimator implemented is `TriangulationBaseline`. Its `Baseline` protocol accepts observations and returns world-metre XYZ; the evaluator and artifact writer can be reused by future estimators after a deliberate CLI/config extension. No empty model classes or silent fallbacks claim model support.

Official upstream references for future investigation:

- [VGGT code and documentation](https://github.com/facebookresearch/vggt)
- [MASt3R code and documentation](https://github.com/naver/mast3r)

These projects provide geometry-related capabilities; neither link establishes that dynamic human motion in a cluttered factory is solved. Review upstream model terms, checkpoints, resource requirements and conventions at integration time. This scaffold makes no runtime imports from either project.

## A practical progression

1. **Pipeline sanity:** run the two committed configurations, inspect masks/denominators, and reproduce results on a second CPU environment.
2. **Failure characterization:** add structured occlusion blocks, camera perturbations and outlier observations; compare errors at matched coverage. Independent dropout alone is inadequate to study factory clutter.
3. **First real sequence:** select a permissioned public benchmark or small consented recording with calibration and independent 3D references. Add one tested conversion script, preserve subject/session/site splits, and establish the detector + triangulation reference.
4. **One research change:** add a temporal estimator or one constraint, then an ablation. Only after that baseline is credible should learned geometry/semantics be introduced one at a time.

No real dataset, GPU access, train/validation/test split, pretrained model, physics optimizer or training result is supplied. CPU pilot performance says nothing about the memory or accuracy of later learned methods.
