"""Confidence-weighted linear triangulation, independently per frame and joint."""

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .data import Observations


class Baseline(Protocol):
    def predict(self, observations: Observations) -> np.ndarray:
        """Return T,J,3 world coordinates in metres; all-NaN triples mean abstention."""
        ...


@dataclass(frozen=True)
class TriangulationBaseline:
    confidence_threshold: float = 0.2
    min_ray_angle_deg: float = 1.0

    def __post_init__(self):
        if not 0 < self.confidence_threshold <= 1:
            raise ValueError("confidence_threshold must be in (0,1]")
        if not 0 < self.min_ray_angle_deg < 90:
            raise ValueError("min_ray_angle_deg must be in (0,90)")

    def predict(self, observations: Observations) -> np.ndarray:
        obs = observations
        output = np.full((len(obs.timestamps_s), len(obs.joint_names), 3), np.nan)
        inverse_intrinsics = [np.linalg.inv(c.intrinsics) for c in obs.cameras]
        for ti in range(len(obs.timestamps_s)):
            for ji in range(len(obs.joint_names)):
                indices = np.flatnonzero(obs.valid[ti, :, ji] &
                                         (obs.confidence[ti, :, ji] >= self.confidence_threshold))
                if len(indices) < 2:
                    continue
                rows, directions = [], []
                for ci in indices:
                    cam = obs.cameras[ci]
                    uv1 = np.append(obs.keypoints_2d[ti, ci, ji], 1.0)
                    ray = inverse_intrinsics[ci] @ uv1
                    ray /= ray[2]
                    direction = cam.world_to_camera[:3, :3].T @ ray
                    directions.append(direction / np.linalg.norm(direction))
                    p = cam.world_to_camera[:3]
                    weight = np.sqrt(obs.confidence[ti, ci, ji])
                    rows.extend([weight * (ray[0] * p[2] - p[0]),
                                 weight * (ray[1] * p[2] - p[1])])
                # Near-parallel (including antiparallel) rays do not establish reliable depth.
                angles = [np.degrees(np.arccos(np.clip(abs(np.dot(a, b)), 0, 1)))
                          for i, a in enumerate(directions) for b in directions[i + 1:]]
                if max(angles) < self.min_ray_angle_deg:
                    continue
                _, singular, vh = np.linalg.svd(np.asarray(rows), full_matrices=False)
                if singular[-2] < 1e-10 or abs(vh[-1, 3]) < 1e-12:
                    continue
                xyz = vh[-1, :3] / vh[-1, 3]
                if all((obs.cameras[ci].world_to_camera[:3, :3] @ xyz +
                        obs.cameras[ci].world_to_camera[:3, 3])[2] > 1e-9 for ci in indices):
                    output[ti, ji] = xyz
        return output
