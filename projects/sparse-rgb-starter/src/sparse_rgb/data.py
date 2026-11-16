"""Dataset I/O and semantic checks beyond the manifest's JSON Schema."""

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from .geometry import Camera

CONVENTIONS = {
    "length_unit": "m",
    "world_axes": "x_right_y_forward_z_up",
    "camera_axes": "x_right_y_down_z_forward",
    "extrinsics": "world_to_camera",
    "image_coordinates": "uv_pixels",
    "distortion": "none_rectified",
}


@dataclass(frozen=True)
class Observations:
    """Estimator inputs only. There is deliberately no ground-truth member."""

    cameras: tuple[Camera, ...]
    joint_names: tuple[str, ...]
    bones: tuple[tuple[int, int], ...]
    timestamps_s: np.ndarray  # T
    keypoints_2d: np.ndarray  # T,C,J,2; NaN when missing
    confidence: np.ndarray  # T,C,J in [0,1]
    valid: np.ndarray  # T,C,J bool

    def __post_init__(self):
        if self.timestamps_s.ndim != 1:
            raise ValueError("timestamps_s must be a one-dimensional float array")
        if any(not np.issubdtype(array.dtype, np.floating)
               for array in (self.timestamps_s, self.keypoints_2d, self.confidence)):
            raise ValueError("timestamps_s, keypoints_2d and confidence must be floating-point arrays")
        t, c, j = len(self.timestamps_s), len(self.cameras), len(self.joint_names)
        if min(t, c, j) < 1 or self.timestamps_s.shape != (t,):
            raise ValueError("nonempty time, camera and joint axes are required")
        if (not np.isfinite(self.timestamps_s).all() or np.any(self.timestamps_s < 0)
                or np.any(np.diff(self.timestamps_s) <= 0)):
            raise ValueError("timestamps_s must be finite, nonnegative and strictly increasing")
        if len(set(self.joint_names)) != j or any(not isinstance(n, str) or not n for n in self.joint_names):
            raise ValueError("joint_names must be unique nonempty strings")
        if len({camera.camera_id for camera in self.cameras}) != c:
            raise ValueError("camera ids must be unique")
        if (self.keypoints_2d.shape != (t, c, j, 2) or self.confidence.shape != (t, c, j)
                or self.valid.shape != (t, c, j) or self.valid.dtype != np.bool_):
            raise ValueError("observation shapes/dtypes must be T,C,J,2 / T,C,J / bool T,C,J")
        if not np.isfinite(self.confidence).all() or np.any((self.confidence < 0) | (self.confidence > 1)):
            raise ValueError("confidence must be finite and in [0,1]")
        if not np.isfinite(self.keypoints_2d[self.valid]).all():
            raise ValueError("valid keypoints must have finite uv coordinates")
        if not np.isnan(self.keypoints_2d[~self.valid]).all():
            raise ValueError("missing keypoints must use NaN coordinates")
        for ci, camera in enumerate(self.cameras):
            uv = self.keypoints_2d[:, ci][self.valid[:, ci]]
            if np.any(uv < 0) or np.any(uv > np.array(camera.image_size) - 1):
                raise ValueError("valid keypoints must be inside the image")
        for bone in self.bones:
            if (len(bone) != 2 or any(type(i) is not int or i < 0 or i >= j for i in bone)
                    or bone[0] == bone[1]):
                raise ValueError("bones must contain two different valid joint indices")


def relative_file(manifest_path: Path, value: str) -> Path:
    """All artifact/image references are relative to the manifest, without traversal."""
    if not isinstance(value, str) or not value or "\\" in value:
        raise ValueError("file references must be nonempty relative POSIX paths")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or ":" in value:
        raise ValueError("file references must stay inside the dataset directory")
    root = manifest_path.parent.resolve()
    resolved = (root / path).resolve()
    if not resolved.is_relative_to(root):
        raise ValueError("file references must stay inside the dataset directory")
    return resolved


def read_manifest(path: Path) -> dict:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    required = {"schema_version", "sequence_id", "source", "conventions", "joint_names", "bones",
                "cameras", "observations_file", "ground_truth_file"}
    if not isinstance(manifest, dict) or set(manifest) != required:
        raise ValueError("manifest keys must match schemas/manifest.schema.json")
    if manifest["schema_version"] != "1.0" or manifest["conventions"] != CONVENTIONS:
        raise ValueError("unsupported schema or coordinate conventions")
    if not isinstance(manifest["sequence_id"], str) or not manifest["sequence_id"]:
        raise ValueError("sequence_id must be a nonempty string")
    if manifest["source"] not in ("synthetic", "real"):
        raise ValueError("source must be synthetic or real")
    if not isinstance(manifest["joint_names"], list) or not isinstance(manifest["bones"], list):
        raise ValueError("joint_names and bones must be arrays")
    if not isinstance(manifest["cameras"], list) or not manifest["cameras"]:
        raise ValueError("at least one camera is required")
    relative_file(path, manifest["observations_file"])
    if manifest["ground_truth_file"] is not None:
        relative_file(path, manifest["ground_truth_file"])
    return manifest


def load_observations(path: Path) -> Observations:
    """Never opens the optional ground-truth file."""
    manifest = read_manifest(path)
    cameras = []
    for record in manifest["cameras"]:
        if not isinstance(record, dict) or set(record) != {"camera_id", "image_size", "intrinsics", "world_to_camera"}:
            raise ValueError("camera keys must match the manifest schema")
        cameras.append(Camera(record["camera_id"], tuple(record["image_size"]),
                              np.asarray(record["intrinsics"], dtype=float),
                              np.asarray(record["world_to_camera"], dtype=float)))
    with np.load(relative_file(path, manifest["observations_file"]), allow_pickle=False) as arrays:
        required = {"timestamps_s", "keypoints_2d", "confidence", "valid"}
        if not required.issubset(arrays.files) or set(arrays.files) - required - {"image_paths"}:
            raise ValueError("observations.npz keys must match docs/dataset.md")
        data = Observations(tuple(cameras), tuple(manifest["joint_names"]),
                            tuple(tuple(b) for b in manifest["bones"]),
                            *(arrays[k] for k in ("timestamps_s", "keypoints_2d", "confidence", "valid")))
        if "image_paths" in arrays.files:
            paths = arrays["image_paths"]
            if paths.shape != data.valid.shape[:2] or paths.dtype.kind != "U":
                raise ValueError("image_paths must be a Unicode T,C array")
            for value in paths.flat:
                if value:
                    relative_file(path, str(value))
    return data


def load_ground_truth(path: Path, shape: tuple[int, int]) -> tuple[np.ndarray, np.ndarray] | None:
    manifest = read_manifest(path)
    if manifest["ground_truth_file"] is None:
        return None
    with np.load(relative_file(path, manifest["ground_truth_file"]), allow_pickle=False) as arrays:
        if set(arrays.files) != {"positions_3d", "valid"}:
            raise ValueError("ground_truth.npz must contain positions_3d and valid")
        xyz, valid = arrays["positions_3d"], arrays["valid"]
    if xyz.shape != shape + (3,) or valid.shape != shape or valid.dtype != np.bool_:
        raise ValueError("ground truth must have shape T,J,3 and bool T,J")
    if not np.isfinite(xyz[valid]).all() or not np.isnan(xyz[~valid]).all():
        raise ValueError("ground truth must be finite when valid, NaN otherwise")
    return xyz, valid
