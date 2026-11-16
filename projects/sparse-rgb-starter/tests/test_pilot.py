from dataclasses import replace
from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal

from sparse_rgb.baseline import TriangulationBaseline
from sparse_rgb.cli import load_config, main, run
from sparse_rgb.data import load_ground_truth, load_observations, relative_file
from sparse_rgb.geometry import Camera, project
from sparse_rgb.metrics import evaluate
from sparse_rgb.synthetic import generate, save_dataset

ROOT = Path(__file__).resolve().parents[1]


class GeometryAndBaselineTests(unittest.TestCase):
    def setUp(self):
        self.obs, self.truth = generate(frames=12, noise_px=0, dropout_probability=0)
        self.model = TriangulationBaseline()

    def test_noiseless_two_view_reconstruction(self):
        predicted = self.model.predict(self.obs)
        assert_allclose(predicted, self.truth, atol=1e-10, rtol=0)
        metrics = evaluate(predicted, self.obs, (self.truth, np.ones(self.truth.shape[:2], bool)))
        self.assertEqual(metrics["coverage"], 1)
        self.assertLess(metrics["reprojection_rmse_px"], 1e-9)

    def test_noisy_reconstruction_is_reasonable(self):
        obs, truth = generate(frames=30, noise_px=1, dropout_probability=.15)
        metrics = evaluate(self.model.predict(obs), obs, (truth, np.ones(truth.shape[:2], bool)))
        self.assertLess(metrics["mpjpe_mm"], 30)
        self.assertGreater(metrics["coverage"], .5)
        self.assertLess(metrics["coverage"], 1)

    def test_one_view_abstains(self):
        obs = replace(self.obs, cameras=self.obs.cameras[:1], keypoints_2d=self.obs.keypoints_2d[:, :1],
                      confidence=self.obs.confidence[:, :1], valid=self.obs.valid[:, :1])
        self.assertTrue(np.isnan(self.model.predict(obs)).all())

    def test_occlusion_and_low_confidence_abstain(self):
        uv, valid, confidence = self.obs.keypoints_2d.copy(), self.obs.valid.copy(), self.obs.confidence.copy()
        valid[:, 1, 0] = False
        uv[:, 1, 0] = np.nan
        confidence[:, 1, 1] = .1
        obs = replace(self.obs, keypoints_2d=uv, valid=valid, confidence=confidence)
        predicted = self.model.predict(obs)
        self.assertTrue(np.isnan(predicted[:, :2]).all())
        assert_allclose(predicted[:, 2:], self.truth[:, 2:], atol=1e-10)

    def test_coincident_cameras_abstain(self):
        cam0 = self.obs.cameras[0]
        cameras = (cam0, replace(cam0, camera_id="duplicate_position"))
        uv = np.repeat(self.obs.keypoints_2d[:, :1], 2, axis=1)
        obs = replace(self.obs, cameras=cameras, keypoints_2d=uv)
        self.assertTrue(np.isnan(self.model.predict(obs)).all())

    def test_near_parallel_rays_abstain(self):
        cam0 = self.obs.cameras[0]
        extrinsic = cam0.world_to_camera.copy()
        extrinsic[0, 3] += 1e-6
        camera1 = replace(cam0, camera_id="tiny_baseline", world_to_camera=extrinsic)
        uv1, _ = project(camera1, self.truth)
        obs = replace(self.obs, cameras=(cam0, camera1),
                      keypoints_2d=np.stack([self.obs.keypoints_2d[:, 0], uv1], axis=1))
        self.assertTrue(np.isnan(self.model.predict(obs)).all())

    def test_camera_rotation_validation(self):
        extrinsic = self.obs.cameras[0].world_to_camera.copy()
        extrinsic[0, :3] *= 2
        with self.assertRaises(ValueError):
            replace(self.obs.cameras[0], world_to_camera=extrinsic)

    def test_projection_rejects_points_behind_camera(self):
        camera = Camera("identity", (100, 100), np.eye(3), np.eye(4))
        uv, valid = project(camera, np.array([[1., 2., -1.], [1., 2., 1.]]))
        assert_array_equal(valid, [False, True])
        self.assertTrue(np.isnan(uv[0]).all())
        assert_allclose(uv[1], [1, 2])


class MetricsTests(unittest.TestCase):
    def setUp(self):
        self.obs, self.truth = generate(frames=3, fps=2, noise_px=0, dropout_probability=0)
        self.gt_valid = np.ones(self.truth.shape[:2], bool)

    def test_known_position_and_velocity_errors(self):
        predicted = self.truth.copy()
        predicted[:, :, 0] += np.arange(3)[:, None] * .01
        metrics = evaluate(predicted, self.obs, (self.truth, self.gt_valid), pck_threshold_m=.015)
        self.assertAlmostEqual(metrics["mpjpe_mm"], 10)
        self.assertAlmostEqual(metrics["velocity_error_mm_s"], 20)
        self.assertAlmostEqual(metrics["pck_with_misses"], 2 / 3)

    def test_velocity_uses_actual_irregular_timestamps(self):
        obs = replace(self.obs, timestamps_s=np.array([0., .25, 1.]))
        predicted = self.truth.copy()
        predicted[:, :, 0] += obs.timestamps_s[:, None] * .01
        metrics = evaluate(predicted, obs, (self.truth, self.gt_valid))
        self.assertAlmostEqual(metrics["velocity_error_mm_s"], 10)

    def test_missing_predictions_count_as_pck_failures(self):
        predicted = self.truth.copy()
        predicted[:, 0] = np.nan
        metrics = evaluate(predicted, self.obs, (self.truth, self.gt_valid))
        self.assertEqual(metrics["mpjpe_mm"], 0)
        self.assertEqual(metrics["pck_on_reconstructed"], 1)
        self.assertAlmostEqual(metrics["coverage"], 14 / 15)
        self.assertAlmostEqual(metrics["pck_with_misses"], 14 / 15)
        self.assertEqual(metrics["velocity_pairs"], 28)

    def test_all_missing_is_null_error_not_zero(self):
        metrics = evaluate(np.full_like(self.truth, np.nan), self.obs, (self.truth, self.gt_valid))
        self.assertEqual(metrics["coverage"], 0)
        self.assertEqual(metrics["pck_with_misses"], 0)
        self.assertIsNone(metrics["mpjpe_mm"])
        self.assertIsNone(metrics["velocity_error_mm_s"])
        self.assertIsNone(metrics["reprojection_rmse_px"])
        json.dumps(metrics, allow_nan=False)

    def test_no_ground_truth_and_empty_ground_truth(self):
        metrics = evaluate(self.truth, self.obs)
        self.assertFalse(metrics["has_ground_truth"])
        self.assertIsNone(metrics["mpjpe_mm"])
        empty = evaluate(self.truth, self.obs,
                         (np.full_like(self.truth, np.nan), np.zeros(self.truth.shape[:2], bool)))
        self.assertIsNone(empty["coverage"])
        self.assertIsNone(empty["pck_with_misses"])

    def test_partial_nan_or_infinity_is_invalid(self):
        for value in (np.nan, np.inf):
            predicted = self.truth.copy()
            predicted[0, 0, 0] = value
            with self.assertRaises(ValueError):
                evaluate(predicted, self.obs)

    def test_reprojection_uses_pixel_vector_norm(self):
        # Translate measured uv by [3,4]; reprojection vector RMSE must be 5 pixels.
        obs = replace(self.obs, keypoints_2d=self.obs.keypoints_2d + [3, 4])
        metrics = evaluate(self.truth, obs)
        self.assertAlmostEqual(metrics["reprojection_rmse_px"], 5)
        self.assertEqual(metrics["reprojection_observations"], 3 * 2 * 15)


class DatasetAndRunTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.obs, self.truth = generate(frames=10)
        self.manifest = save_dataset(self.root / "data", self.obs, self.truth)

    def edit_manifest(self, change):
        manifest = json.loads(self.manifest.read_text())
        change(manifest)
        self.manifest.write_text(json.dumps(manifest))

    def test_roundtrip_and_ground_truth_is_separate(self):
        obs = load_observations(self.manifest)
        assert_allclose(obs.keypoints_2d, self.obs.keypoints_2d, equal_nan=True)
        assert_allclose(load_ground_truth(self.manifest, self.truth.shape[:2])[0], self.truth)
        before = TriangulationBaseline().predict(obs)
        # Deleting GT cannot change inference or input validation.
        (self.manifest.parent / "ground_truth.npz").unlink()
        after = TriangulationBaseline().predict(load_observations(self.manifest))
        assert_allclose(before, after, equal_nan=True)

    def test_seed_reproducibility(self):
        obs2, truth2 = generate(frames=10)
        assert_array_equal(self.obs.keypoints_2d, obs2.keypoints_2d)
        assert_array_equal(self.truth, truth2)
        obs3, _ = generate(seed=43, frames=10)
        self.assertFalse(np.array_equal(self.obs.valid, obs3.valid))

    def test_invalid_times_confidence_and_missing_coordinates(self):
        with self.assertRaises(ValueError):
            replace(self.obs, timestamps_s=np.zeros(10))
        with self.assertRaises(ValueError):
            replace(self.obs, confidence=np.full_like(self.obs.confidence, 2))
        with self.assertRaises(ValueError):
            replace(self.obs, keypoints_2d=np.nan_to_num(self.obs.keypoints_2d))
        with self.assertRaises(ValueError):
            replace(self.obs, valid=self.obs.valid.astype(int))
        with self.assertRaises(ValueError):
            replace(self.obs, timestamps_s=np.arange(10, dtype=np.uint64)[::-1])
        with self.assertRaises(ValueError):
            replace(self.obs, timestamps_s=np.array(0.0))

    def test_manifest_rejects_wrong_conventions(self):
        self.edit_manifest(lambda m: m["conventions"].update(length_unit="mm"))
        with self.assertRaises(ValueError):
            load_observations(self.manifest)

    def test_manifest_rejects_unknown_keys(self):
        self.edit_manifest(lambda m: m.update(unused_typo=True))
        with self.assertRaises(ValueError):
            load_observations(self.manifest)

    def test_traversal_references_are_rejected(self):
        for name in ("../secret.npz", "/tmp/secret.npz", "C:/data.npz", "a\\b.npz"):
            with self.assertRaises(ValueError):
                relative_file(self.manifest, name)

    def test_object_arrays_are_rejected(self):
        np.savez(self.manifest.parent / "observations.npz", timestamps_s=np.array([{}], dtype=object),
                 keypoints_2d=self.obs.keypoints_2d, valid=self.obs.valid, confidence=self.obs.confidence)
        with self.assertRaises(ValueError):
            load_observations(self.manifest)

    def test_config_unknown_keys_fail(self):
        path = self.root / "bad.toml"
        path.write_text((ROOT / "configs/pilot.toml").read_text() + "\nunknown = 1\n")
        with self.assertRaises(ValueError):
            load_config(path)

    def test_repeat_runs_and_outputs(self):
        config = self.root / "short.toml"
        config.write_text((ROOT / "configs/pilot.toml").read_text().replace("frames = 120", "frames = 10"))
        metrics1 = run(config, self.root / "run1")
        metrics2 = run(config, self.root / "run2")
        self.assertEqual(metrics1, metrics2)
        record1 = json.loads((self.root / "run1/run.json").read_text())
        record2 = json.loads((self.root / "run2/run.json").read_text())
        self.assertEqual(record1["input_sha256"], record2["input_sha256"])
        self.assertEqual(record1["output_sha256"], record2["output_sha256"])
        self.assertEqual(len(record1["environment"]["source_sha256"]), 64)
        with self.assertRaises(FileExistsError):
            run(config, self.root / "run1")

    def test_supplied_dataset_without_ground_truth(self):
        self.edit_manifest(lambda m: m.update(ground_truth_file=None, source="real"))
        metrics = run(ROOT / "configs/pilot.toml", self.root / "real_run", self.manifest)
        self.assertFalse(metrics["has_ground_truth"])
        self.assertIsNone(metrics["coverage"])
        self.assertIsNone(metrics["mpjpe_mm"])

    def test_cli_pilot_validate_and_failed_overwrite(self):
        output = self.root / "cli"
        args = ["pilot", "--config", str(ROOT / "configs/clean.toml"), "--output", str(output)]
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(main(args), 0)
            self.assertEqual(main(["validate", str(output / "dataset/manifest.json")]), 0)
            self.assertEqual(main(args), 2)


if __name__ == "__main__":
    unittest.main()
