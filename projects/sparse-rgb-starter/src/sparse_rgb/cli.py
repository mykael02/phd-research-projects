"""Run a single synthetic pilot or reconstruct a supplied keypoint dataset."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import tomllib

import numpy as np

from . import __version__
from .baseline import TriangulationBaseline
from .data import load_ground_truth, load_observations, read_manifest, relative_file
from .metrics import evaluate
from .synthetic import generate, save_dataset


def load_config(path: Path) -> dict:
    config = tomllib.loads(path.read_text(encoding="utf-8"))
    expected = {"synthetic": {"seed", "frames", "fps", "noise_px", "dropout_probability"},
                "baseline": {"confidence_threshold", "min_ray_angle_deg"},
                "evaluation": {"pck_threshold_m"}}
    if set(config) != set(expected) or any(not isinstance(config[k], dict) or set(config[k]) != keys
                                          for k, keys in expected.items()):
        raise ValueError("config sections/keys must match configs/pilot.toml (unknown keys are rejected)")
    TriangulationBaseline(**config["baseline"])
    threshold = config["evaluation"]["pck_threshold_m"]
    if not np.isfinite(threshold) or threshold <= 0:
        raise ValueError("pck_threshold_m must be positive and finite")
    # Validate synthetic options without generating or allocating the requested sequence.
    synthetic = config["synthetic"]
    if (type(synthetic["seed"]) is not int or synthetic["seed"] < 0 or
            type(synthetic["frames"]) is not int or synthetic["frames"] < 2):
        raise ValueError("seed must be nonnegative and frames must be an integer >=2")
    numbers = [synthetic[k] for k in ("fps", "noise_px", "dropout_probability")]
    if (not np.isfinite(numbers).all() or numbers[0] <= 0 or numbers[1] < 0
            or not 0 <= numbers[2] <= 1):
        raise ValueError("invalid synthetic fps, noise or dropout")
    return config


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: dict):
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def provenance() -> dict:
    package_dir = Path(__file__).resolve().parent
    root = package_dir.parent.parent
    revision, dirty = None, None
    try:
        def git(*args):
            return subprocess.check_output(["git", "-C", str(root), *args], stderr=subprocess.DEVNULL,
                                           text=True, timeout=5).strip()
        if Path(git("rev-parse", "--show-toplevel")).resolve() == root:
            revision = git("rev-parse", "HEAD")
            dirty = bool(git("status", "--porcelain"))
    except (OSError, subprocess.SubprocessError):
        pass
    code_hash = hashlib.sha256()
    for path in sorted(package_dir.glob("*.py")):
        code_hash.update(path.name.encode())
        code_hash.update(path.read_bytes())
    return {"package_version": __version__, "python": platform.python_version(),
            "numpy": np.__version__, "platform": platform.platform(),
            "git_revision": revision, "git_dirty": dirty, "source_sha256": code_hash.hexdigest()}


def run(config_path: Path, output: Path, manifest_path: Path | None = None) -> dict:
    config = load_config(config_path)
    config_bytes = config_path.read_bytes()
    # A new directory for every run prevents mixing or overwriting experiments.
    output.mkdir(parents=True, exist_ok=False)
    (output / "config.toml").write_bytes(config_bytes)
    if manifest_path is None:
        obs, truth = generate(**config["synthetic"])
        manifest_path = save_dataset(output / "dataset", obs, truth)
    manifest_path = manifest_path.resolve()
    obs = load_observations(manifest_path)
    model = TriangulationBaseline(**config["baseline"])
    predictions = model.predict(obs)
    # Only the evaluator opens ground truth, after inference has completed.
    gt = load_ground_truth(manifest_path, predictions.shape[:2])
    metrics = evaluate(predictions, obs, gt, **config["evaluation"])
    np.savez_compressed(output / "predictions.npz", positions_3d=predictions,
                        valid=np.isfinite(predictions).all(axis=-1))
    write_json(output / "metrics.json", metrics)
    manifest = read_manifest(manifest_path)
    input_hashes = {"manifest": digest(manifest_path),
                    "observations": digest(relative_file(manifest_path, manifest["observations_file"]))}
    if manifest["ground_truth_file"] is not None:
        input_hashes["ground_truth"] = digest(relative_file(manifest_path, manifest["ground_truth_file"]))
    write_json(output / "run.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(), "environment": provenance(),
        "baseline": "confidence_weighted_dlt", "config": config,
        "config_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "dataset_manifest": str(manifest_path), "input_sha256": input_hashes,
        "output_sha256": {p.name: digest(p) for p in (output / "predictions.npz", output / "metrics.json")},
        "scope": "synthetic keypoint pilot" if manifest["source"] == "synthetic" else "supplied keypoint dataset",
    })
    return metrics


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("pilot", "run"):
        sub = commands.add_parser(name)
        if name == "run":
            sub.add_argument("manifest", type=Path)
        sub.add_argument("--config", type=Path, required=True)
        sub.add_argument("--output", type=Path, required=True)
    validate = commands.add_parser("validate", help="validate camera metadata, arrays and optional ground truth")
    validate.add_argument("manifest", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "validate":
            obs = load_observations(args.manifest)
            load_ground_truth(args.manifest, (len(obs.timestamps_s), len(obs.joint_names)))
            print("Dataset metadata and numeric arrays are valid. RGB files were not inspected.")
        else:
            metrics = run(args.config, args.output, getattr(args, "manifest", None))
            print(json.dumps(metrics, indent=2, allow_nan=False))
            print(f"Saved run to {args.output}")
    except (ValueError, TypeError, KeyError, OSError, np.linalg.LinAlgError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0
