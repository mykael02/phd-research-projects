"""World-space, unaligned errors with explicit missing-prediction denominators."""

import numpy as np

from .data import Observations
from .geometry import project


def evaluate(predictions: np.ndarray, observations: Observations,
             ground_truth: tuple[np.ndarray, np.ndarray] | None = None,
             pck_threshold_m: float = 0.05) -> dict:
    shape = (len(observations.timestamps_s), len(observations.joint_names))
    if predictions.shape != shape + (3,):
        raise ValueError("predictions must have shape T,J,3")
    finite = np.isfinite(predictions)
    predicted = finite.all(axis=-1)
    if not np.all(predicted | np.isnan(predictions).all(axis=-1)):
        raise ValueError("predictions must contain finite or all-NaN xyz triples")
    if not np.isfinite(pck_threshold_m) or pck_threshold_m <= 0:
        raise ValueError("pck_threshold_m must be positive and finite")
    squared_reprojection = []
    for ci, cam in enumerate(observations.cameras):
        uv, front = project(cam, predictions)
        usable = predicted & front & observations.valid[:, ci]
        squared_reprojection.extend(np.sum((uv[usable] - observations.keypoints_2d[:, ci][usable]) ** 2,
                                            axis=-1).tolist())
    result = {
        "total_joint_slots": int(predicted.size),
        "predicted_joint_slots": int(predicted.sum()),
        "prediction_fraction": float(predicted.mean()),
        "reprojection_observations": len(squared_reprojection),
        "reprojection_rmse_px": (float(np.sqrt(np.mean(squared_reprojection)))
                                 if squared_reprojection else None),
        "has_ground_truth": ground_truth is not None,
        "pck_threshold_mm": float(pck_threshold_m * 1000),
        "ground_truth_joint_slots": None, "evaluated_joint_slots": None,
        "coverage": None, "mpjpe_mm": None, "pck_on_reconstructed": None,
        "pck_with_misses": None, "velocity_pairs": None, "velocity_error_mm_s": None,
    }
    if ground_truth is None:
        return result
    truth, gt_valid = ground_truth
    if truth.shape != predictions.shape or gt_valid.shape != shape or gt_valid.dtype != np.bool_:
        raise ValueError("ground truth shapes or mask are invalid")
    if not np.isfinite(truth[gt_valid]).all() or not np.isnan(truth[~gt_valid]).all():
        raise ValueError("ground truth coordinates must agree with validity")
    usable = gt_valid & predicted
    errors = np.linalg.norm(predictions[usable] - truth[usable], axis=-1)
    n_gt, n_eval = int(gt_valid.sum()), int(usable.sum())
    correct = int(np.sum(errors <= pck_threshold_m))
    pair_valid = usable[1:] & usable[:-1]
    dt = np.diff(observations.timestamps_s)[:, None, None]
    velocity_errors = np.linalg.norm(((np.diff(predictions, axis=0) - np.diff(truth, axis=0)) / dt)
                                     [pair_valid], axis=-1)
    result.update({
        "ground_truth_joint_slots": n_gt, "evaluated_joint_slots": n_eval,
        "coverage": n_eval / n_gt if n_gt else None,
        "mpjpe_mm": float(errors.mean() * 1000) if n_eval else None,
        "pck_on_reconstructed": correct / n_eval if n_eval else None,
        "pck_with_misses": correct / n_gt if n_gt else None,
        "velocity_pairs": int(pair_valid.sum()),
        "velocity_error_mm_s": float(velocity_errors.mean() * 1000) if len(velocity_errors) else None,
    })
    return result
