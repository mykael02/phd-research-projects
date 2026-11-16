import math
from pathlib import Path
import random
import unittest

from gridnav import load_grid, parse_grid
from rrtplan import cell_center, point_is_free, rrt, segment_is_free, validate_continuous_path

MAPS = Path(__file__).resolve().parents[1] / "maps"


class GeometryTests(unittest.TestCase):
    def test_cell_centres_and_shared_point_references(self):
        grid = load_grid(MAPS / "detour.txt")
        self.assertEqual(cell_center((2, 0)), (0.5, 2.5))
        for point, expected in (
            ((0.5, 2.5), True), ((1.0, 2.5), False),
            ((0.0, 2.5), False), ((3.0, 2.5), False), ((1.5, 4.5), True),
            ((0.5, 0.0), False), ((0.5, 5.0), False),
            ((1.5, 2.5), False), ((1.0, 4.0), False),
            ((float("nan"), 2.5), False), ((0.5, float("inf")), False),
        ):
            with self.subTest(point=point):
                self.assertEqual(point_is_free(grid, point), expected)
        for cell in ((-1, 0), (0, -1), (5, 0), (0, 3), (2, 1)):
            with self.subTest(cell=cell):
                self.assertFalse(point_is_free(grid, cell_center(cell)))

    def test_exact_segments_and_reversal(self):
        grid = parse_grid("...\n.#.\n...")
        for start, end, expected in (
            ((0.5, 1.5), (2.5, 1.5), False),  # free endpoints, through square
            ((0.5, 1.5), (1.5, 0.5), False),  # only touches corner (1,1)
            ((0.0, 0.5), (0.0, 2.5), False),  # outer edge
            ((0.5, 1.0), (2.5, 1.0), False),  # follows obstacle edge
            ((0.5, 0.5), (2.5, 0.5), True),
            ((0.5, 0.5), (0.5, 2.5), True),
            ((0.5, 1.25), (1.25, 0.5), True),  # bbox overlaps, segment misses
            ((0.5, 0.5), (0.5, 0.5), True),
            ((1.0, 1.0), (1.0, 1.0), False),
            ((0.5, 0.5), (-0.5, 0.5), False),
            ((0.5, 0.5), (float("nan"), 0.5), False),
            ((0.5, 1.5), (1.5, 0.5 - 2**-50), True),  # just misses corner
            ((0.5, 1.5), (1.5, 0.5 + 2**-50), False),  # just cuts corner
        ):
            with self.subTest(start=start, end=end):
                self.assertEqual(segment_is_free(grid, start, end), expected)
                self.assertEqual(segment_is_free(grid, end, start), expected)


class PlannerTests(unittest.TestCase):
    def test_endpoint_failures_and_priority(self):
        grid = parse_grid("#.")
        for start, goal, reason in (
            ((0, 0), (0, 1), "start_in_collision"),
            ((0, 0), (0, 0), "start_in_collision"),
            ((0, 0), (0, 2), "start_in_collision"),
            ((-1, 0), (0, 0), "start_in_collision"),
            ((-1, 0), (-1, 0), "start_in_collision"),
            ((1, 0), (0, 1), "start_in_collision"),
            ((0, -1), (0, 1), "start_in_collision"),
            ((0, 2), (0, 1), "start_in_collision"),
            ((0, 1), (0, 0), "goal_in_collision"),
            ((0, 1), (0, 2), "goal_in_collision"),
            ((0, 1), (-1, 0), "goal_in_collision"),
            ((0, 1), (1, 0), "goal_in_collision"),
            ((0, 1), (0, -1), "goal_in_collision"),
        ):
            with self.subTest(start=start, goal=goal):
                result = rrt(grid, start, goal)
                self.assertFalse(result.success)
                self.assertEqual(result.reason, reason)
                self.assertEqual(result.path, ())
                self.assertEqual(result.nodes, ())
                self.assertIsNone(result.length)
                self.assertEqual(result.iterations, 0)

    def test_equal_free_endpoints_and_zero_budget(self):
        grid = parse_grid("..")
        result = rrt(grid, (0, 0), (0, 0), max_iterations=0)
        self.assertTrue(result.success)
        self.assertEqual(result.path, ((0.5, 0.5),))
        self.assertEqual(result.length, 0)
        self.assertTrue(validate_continuous_path(grid, result.path, (0, 0), (0, 0)))
        failed = rrt(grid, (0, 0), (0, 1), max_iterations=0)
        self.assertFalse(failed.success)
        self.assertEqual(failed.reason, "budget_exhausted")
        self.assertEqual(failed.iterations, 0)
        self.assertIsNone(failed.length)

    def test_direct_goal_connection_and_length(self):
        grid = load_grid(MAPS / "open.txt")
        result = rrt(grid, (0, 0), (3, 4), goal_bias=1)
        self.assertEqual(result.iterations, 1)
        self.assertEqual(result.path, ((0.5, 0.5), (0.9, 0.8), (4.5, 3.5)))
        self.assertAlmostEqual(result.length, 5.0)
        self.assertTrue(validate_continuous_path(grid, result.path, (0, 0), (3, 4)))
        reached = rrt(grid, (0, 0), (0, 1), goal_bias=1, step=1)
        self.assertEqual(reached.path, ((0.5, 0.5), (1.5, 0.5)))

    def test_invalid_settings(self):
        for kwargs in (
            {"max_iterations": -1}, {"max_iterations": 1.5},
            {"step": 0}, {"step": -1}, {"step": float("inf")},
            {"goal_bias": -0.1}, {"goal_bias": 1.1}, {"goal_bias": float("nan")},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                rrt(parse_grid(".."), (0, 0), (0, 1), **kwargs)

    def test_default_seed_thresholds_validity_and_identical_trees(self):
        global_state = random.getstate()
        for name, goal, minimum, maximum in (
            ("open", (3, 4), 18, 20), ("detour", (0, 2), 18, 20),
            ("disconnected", (0, 4), 0, 0),
        ):
            grid = load_grid(MAPS / (name + ".txt"))
            successes = 0
            for seed in range(20):
                with self.subTest(map=name, seed=seed):
                    result = rrt(grid, (0, 0), goal, seed=seed)
                    self.assertEqual(result, rrt(grid, (0, 0), goal, seed=seed))
                    self.assertEqual(result.nodes[0], cell_center((0, 0)))
                    self.assertIsNone(result.parents[0])
                    self.assertEqual(len(result.nodes), len(result.parents))
                    for index, parent in enumerate(result.parents[1:], 1):
                        self.assertLess(parent, index)
                        self.assertTrue(segment_is_free(grid, result.nodes[parent], result.nodes[index]))
                        if not (result.success and index == len(result.nodes) - 1):
                            self.assertLessEqual(math.dist(result.nodes[parent], result.nodes[index]), 0.5 + 1e-14)
                    if result.success:
                        successes += 1
                        self.assertTrue(validate_continuous_path(grid, result.path, (0, 0), goal))
                        self.assertEqual(result.path[-1], cell_center(goal))
                        self.assertGreater(result.length, 0)
                    else:
                        self.assertEqual(result.reason, "budget_exhausted")
                        self.assertEqual(result.iterations, 2000)
                        self.assertEqual(result.path, ())
                        self.assertIsNone(result.length)
            with self.subTest(map=name):
                self.assertGreaterEqual(successes, minimum)
                self.assertLessEqual(successes, maximum)
        self.assertEqual(random.getstate(), global_state)


class ValidatorTests(unittest.TestCase):
    def test_reject_invalid_paths(self):
        grid = parse_grid("...\n.#.\n...")
        for path, start, goal in (
            ((), (0, 0), (0, 0)),
            (((0.5, 0.5), (2.5, 0.5)), (0, 1), (0, 2)),
            (((0.5, 0.5), (2.5, 0.5)), (0, 0), (2, 2)),
            (((0.5, 1.5), (2.5, 1.5)), (1, 0), (1, 2)),  # colliding segment
            (((0.5, 0.5), (1.5, 1.5), (2.5, 2.5)), (0, 0), (2, 2)),
            (((0.5, 0.5), (0.0, 0.5), (2.5, 0.5)), (0, 0), (0, 2)),
            (((1.5, 1.5),), (1, 1), (1, 1)),
            (((-0.5, 0.5),), (0, -1), (0, -1)),
        ):
            with self.subTest(path=path, start=start, goal=goal):
                self.assertFalse(validate_continuous_path(grid, path, start, goal))

    def test_valid_direct_and_detour_paths(self):
        grid = load_grid(MAPS / "detour.txt")
        path = ((0.5, 0.5), (0.5, 4.5), (2.5, 4.5), (2.5, 0.5))
        self.assertTrue(validate_continuous_path(grid, path, (0, 0), (0, 2)))
        self.assertTrue(validate_continuous_path(grid, ((0.5, 0.5),), (0, 0), (0, 0)))


if __name__ == "__main__":
    unittest.main()
