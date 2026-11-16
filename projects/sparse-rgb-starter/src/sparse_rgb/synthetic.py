"""Deterministic human-inspired landmarks; no images or realistic occluders are rendered."""

import json
from pathlib import Path

import numpy as np

from .data import CONVENTIONS, Observations
from .geometry import Camera, look_at, project

JOINT_NAMES = ("pelvis", "neck", "head", "left_shoulder", "left_elbow", "left_wrist",
               "right_shoulder", "right_elbow", "right_wrist", "left_hip", "left_knee",
               "left_ankle", "right_hip", "right_knee", "right_ankle")
BONES = ((0, 1), (1, 2), (1, 3), (3, 4), (4, 5), (1, 6), (6, 7), (7, 8),
         (0, 9), (9, 10), (10, 11), (0, 12), (12, 13), (13, 14))


def generate(seed: int = 42, frames: int = 120, fps: float = 30.0,
             noise_px: float = 1.0, dropout_probability: float = 0.15
             ) -> tuple[Observations, np.ndarray]:
    if type(seed) is not int or seed < 0 or type(frames) is not int or frames < 2:
        raise ValueError("seed must be nonnegative and frames must be an integer >=2")
    if (not np.isfinite([fps, noise_px, dropout_probability]).all() or fps <= 0 or noise_px < 0
            or not 0 <= dropout_probability <= 1):
        raise ValueError("fps>0, noise_px>=0 and dropout_probability in [0,1] are required")
    rng = np.random.default_rng(seed)
    timestamps = np.arange(frames, dtype=float) / fps
    rest = np.array([[0, 0, 1.0], [0, 0, 1.5], [0, 0, 1.78],
                     [-.22, 0, 1.48], [-.42, 0, 1.23], [-.46, 0, .98],
                     [.22, 0, 1.48], [.42, 0, 1.23], [.46, 0, .98],
                     [-.13, 0, .95], [-.14, 0, .52], [-.14, .04, .08],
                     [.13, 0, .95], [.14, 0, .52], [.14, .04, .08]])
    xyz = np.broadcast_to(rest, (frames,) + rest.shape).copy()
    phase = 2 * np.pi * .5 * timestamps
    xyz[:, :, 0] += .18 * np.sin(phase[:, None] * .5)
    xyz[:, :, 1] += .12 * np.cos(phase[:, None] * .5)
    # A simple reaching action, deliberately not a constant-bone-length physics model.
    xyz[:, 5, 2] += .30 * (1 + np.sin(phase))
    xyz[:, 5, 1] += .20 * np.cos(phase)
    xyz[:, 4, 2] += .12 * (1 + np.sin(phase))
    xyz[:, 8, 1] += .15 * np.sin(phase)
    k = np.array([[900., 0., 639.5], [0., 900., 359.5], [0., 0., 1.]])
    cameras = tuple(Camera(f"cam{i}", (1280, 720), k.copy(),
                           look_at(np.array(center), np.array([0., 0., .95])))
                    for i, center in enumerate(([-1.8, -4., 2.1], [1.8, -4., 2.1])))
    projections = [project(cam, xyz) for cam in cameras]
    uv = np.stack([p[0] for p in projections], axis=1)
    valid = np.stack([p[1] for p in projections], axis=1)
    uv += rng.normal(0, noise_px, uv.shape)
    valid &= (uv >= 0).all(axis=-1) & (uv <= [1279, 719]).all(axis=-1)
    valid &= rng.random(valid.shape) >= dropout_probability
    confidence = np.where(valid, 1.0, 0.0)
    uv[~valid] = np.nan
    return Observations(cameras, JOINT_NAMES, BONES, timestamps, uv, confidence, valid), xyz


def save_dataset(directory: Path, observations: Observations, xyz: np.ndarray) -> Path:
    directory.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(directory / "observations.npz", timestamps_s=observations.timestamps_s,
                        keypoints_2d=observations.keypoints_2d,
                        confidence=observations.confidence, valid=observations.valid)
    np.savez_compressed(directory / "ground_truth.npz", positions_3d=xyz,
                        valid=np.ones(xyz.shape[:2], dtype=bool))
    manifest = {
        "schema_version": "1.0", "sequence_id": "synthetic_reach", "source": "synthetic",
        "conventions": CONVENTIONS, "joint_names": list(observations.joint_names),
        "bones": observations.bones, "observations_file": "observations.npz",
        "ground_truth_file": "ground_truth.npz",
        "cameras": [{"camera_id": c.camera_id, "image_size": c.image_size,
                     "intrinsics": c.intrinsics.tolist(), "world_to_camera": c.world_to_camera.tolist()}
                    for c in observations.cameras],
    }
    path = directory / "manifest.json"
    path.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return path
