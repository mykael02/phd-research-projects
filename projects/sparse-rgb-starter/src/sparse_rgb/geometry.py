"""Pinhole projection. World: x right, y forward, z up. Camera: x right, y down, z forward."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Camera:
    camera_id: str
    image_size: tuple[int, int]  # width, height
    intrinsics: np.ndarray  # 3 x 3, pixel units
    world_to_camera: np.ndarray  # 4 x 4, translation in metres

    def __post_init__(self):
        if not isinstance(self.camera_id, str) or not self.camera_id or len(self.image_size) != 2:
            raise ValueError("camera id and image_size [width, height] are required")
        if any(type(v) is not int or v < 1 for v in self.image_size):
            raise ValueError("image_size must contain positive integers")
        k, e = self.intrinsics, self.world_to_camera
        if k.shape != (3, 3) or e.shape != (4, 4):
            raise ValueError("camera matrices must be 3x3 and 4x4")
        if not np.isfinite(k).all() or not np.isfinite(e).all():
            raise ValueError("camera matrices must be finite")
        if (k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1])
                or not np.isclose(k[1, 0], 0)):
            raise ValueError("intrinsics must be an upper triangular pinhole matrix")
        if not np.allclose(e[3], [0, 0, 0, 1]):
            raise ValueError("extrinsics must be a homogeneous world_to_camera transform")
        r = e[:3, :3]
        if not np.allclose(r @ r.T, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(r), 1):
            raise ValueError("camera rotation must be orthonormal with determinant +1")

    @property
    def center(self) -> np.ndarray:
        return -self.world_to_camera[:3, :3].T @ self.world_to_camera[:3, 3]


def look_at(center: np.ndarray, target: np.ndarray) -> np.ndarray:
    forward = target - center
    forward = forward / np.linalg.norm(forward)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right = right / np.linalg.norm(right)
    down = np.cross(forward, right)
    rotation = np.stack([right, down, forward])
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = rotation
    extrinsic[:3, 3] = -rotation @ center
    return extrinsic


def project(camera: Camera, xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return pixel coordinates and positive-depth validity, preserving leading dimensions."""
    local = xyz @ camera.world_to_camera[:3, :3].T + camera.world_to_camera[:3, 3]
    homogeneous = local @ camera.intrinsics.T
    depth = local[..., 2]
    valid = np.isfinite(local).all(axis=-1) & (depth > 1e-9)
    uv = np.full(xyz.shape[:-1] + (2,), np.nan)
    np.divide(homogeneous[..., :2], homogeneous[..., 2, None], out=uv, where=valid[..., None])
    return uv, valid
