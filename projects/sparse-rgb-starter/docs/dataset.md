# Dataset and camera contract

One manifest describes **one synchronized sequence of one consistently identified person**. Multiple people, variable camera sets, asynchronous cameras, rolling shutter, distorted images and uncertain calibration require explicit adapters before using this baseline. No such adapters are included.

`schemas/manifest.schema.json` describes the JSON structure. The Python loader enforces the supported fields and adds numerical/semantic checks (array shapes, index bounds, unique camera IDs, finite calibration, rotation validity, timestamps and path containment). It is a purpose-built validator, not a general JSON Schema engine. Only NumPy is needed at runtime.

## Manifest

Run the pilot to obtain a complete manifest with numeric camera matrices. Required fields:

| Field | Meaning |
|---|---|
| `schema_version` | Exactly `"1.0"` |
| `sequence_id` | Nonempty sequence identifier |
| `source` | `"synthetic"` or `"real"` |
| `conventions` | Exact convention object specified below |
| `joint_names` | Ordered, unique names defining the J axis |
| `bones` | Pairs of zero-based indices into `joint_names`; metadata only |
| `cameras` | Ordered camera records defining the C axis |
| `observations_file` | Relative POSIX path to an NPZ file |
| `ground_truth_file` | Relative POSIX NPZ path, or `null` |

Unknown manifest/camera keys are rejected to expose typos. Paths are relative to the manifest directory, remain within it, and cannot contain `..`, drive letters or backslashes. Files are opened with `allow_pickle=False`; do not store Python objects.

```json
{
  "length_unit": "m",
  "world_axes": "x_right_y_forward_z_up",
  "camera_axes": "x_right_y_down_z_forward",
  "extrinsics": "world_to_camera",
  "image_coordinates": "uv_pixels",
  "distortion": "none_rectified"
}
```

World coordinates form a right-handed frame with Z up and floor Z=0. Each camera record has a unique `camera_id`, `image_size: [width, height]`, upper triangular 3x3 `intrinsics` K with positive focal lengths and last row `[0,0,1]`, and 4x4 `world_to_camera` E with last row `[0,0,0,1]`. E contains an orthonormal rotation R with determinant +1 and translation t in metres.

For column vectors: `X_camera = R @ X_world + t`; `p = K @ X_camera`; `u = p[0]/p[2]`, `v = p[1]/p[2]`. Camera depth must be positive. Pixel coordinates are continuous with the top-left pixel center `(0,0)` and valid bounds `0 <= u <= width-1`, `0 <= v <= height-1`. Images must already be rectified; do not insert camera-to-world matrices or normalized image coordinates. Camera K must match any crop/resize applied to the keypoints.

## Observations NPZ

T = frames, C = cameras, J = joints. Axis order must match the manifest; there is no automatic matching by name.

| Array | Shape | Type/units | Rules |
|---|---|---|---|
| `timestamps_s` | `(T,)` | float, seconds | Nonnegative, finite, strictly increasing; actual capture times |
| `keypoints_2d` | `(T,C,J,2)` | float, pixels | `[u,v]`; finite and in bounds when valid; both NaN when missing |
| `confidence` | `(T,C,J)` | float | Finite `[0,1]`; zero is recommended when missing |
| `valid` | `(T,C,J)` | bool | Whether a usable observation exists; dtype must be Boolean |
| `image_paths` (optional) | `(T,C)` | NumPy Unicode | Relative RGB paths; empty string for absent image |

The loader validates RGB reference syntax only. It does not require image files to exist, decode them, or verify synchronization against them. The estimator never reads images. Real data needs a separate detector/tracker producing matched joints and identities. The current baseline needs at least two confident views per joint; a one-camera manifest is valid but produces no reconstruction. Confidence is a heuristic weight, not a calibrated uncertainty model.

The synthetic generator has no RGB files and therefore omits `image_paths`. It supplies exact joint identities, timestamps and camera calibration, adds independent Gaussian pixel noise, and drops individual camera/joint observations independently. Its reaching landmarks can change bone lengths; they are not a biomechanical simulation.

## Optional ground truth and predictions

GT NPZ contains only `positions_3d` of shape `(T,J,3)` in world metres and `valid` of shape `(T,J)` with Boolean dtype. Coordinates must be finite where valid and all NaN where invalid. T/J ordering and times are inherited from observations. Reference 3D data should be independently measured; outputs of the same triangulation are not independent ground truth.

`predictions.npz` uses the same two array names/shapes. The baseline returns all-NaN triples when it abstains. `load_observations()` does not open GT; only the evaluator loads it after prediction. When there is no GT, use `ground_truth_file: null` rather than inventing reference coordinates.

## Evaluation

Let G be GT-valid joint/time slots and S be the subset with finite predicted XYZ. There is no coordinate alignment, scale fitting, root subtraction or interpolation.

| Metric | Definition |
|---|---|
| `prediction_fraction` | Finite predicted joint slots / all T×J slots |
| `coverage` | Number of slots in S / number in G |
| `mpjpe_mm` | Mean Euclidean XYZ error over S, multiplied by 1000 |
| `pck_with_misses` | Slots in S with error <= configured threshold / number in G; missing predictions fail |
| `pck_on_reconstructed` | Slots in S within threshold / number in S |
| `reprojection_rmse_px` | Square root of mean **squared 2D vector norm** between projected predictions and valid measured uv; positive-depth predictions only |
| `velocity_error_mm_s` | Mean norm of predicted minus GT forward-difference velocity, using actual delta time, over adjacent frames valid in both S masks |

No temporal gaps are bridged. Joint/time slots and temporal pair counts are reported explicitly. Reprojection uses all input-valid observations with reconstructed positive-depth points, including input-valid points below the estimator's confidence threshold. It measures fit to the observations, not independent 3D correctness. Empty denominators give `null`; entirely missing predictions with GT give zero coverage and zero PCK including misses. Without GT, all GT-dependent metrics remain `null`.

## Before a real-data experiment

Collect consented sequences with calibrated, synchronized cameras and document calibration error, dropped frames, lens rectification, units and annotation provenance. Split by person/session/site rather than nearby frames. Fix detector, keypoint mapping and settings before evaluating held-out sequences. Report both missing observations and missing predictions, and keep any independent reference system's uncertainty visible. These are experiment design requirements, not implemented data collection tooling.
