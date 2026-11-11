from pathlib import Path
import unittest

from gridnav import Grid, astar, bfs, load_grid, parse_grid, validate_path
from run_experiments import DEFAULT_SEEDS, seeded_case

ROOT = Path(__file__).resolve().parents[1]


class GridTests(unittest.TestCase):
    def test_load_grid_and_line_endings(self):
        grid = load_grid(ROOT / "maps" / "detour.txt")
        self.assertEqual((grid.height, grid.width), (5, 3))
        self.assertEqual(parse_grid(".#\r\n..\r\n"), Grid((".#", "..")))

    def test_malformed_maps(self):
        for text in ("", "\n", "...\n..", ".x", ". ", "..\n\n..", "..\n\n", ".\v.", ".\r."):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_grid(text)
        for rows in ((), ("",), (".", ".."), ("?",)):
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                Grid(rows)

    def test_neighbor_order_and_bounds(self):
        grid = parse_grid("...\n...\n...")
        self.assertEqual(list(grid.neighbors((1, 1))), [(0, 1), (1, 2), (2, 1), (1, 0)])
        self.assertEqual(list(grid.neighbors((0, 0))), [(0, 1), (1, 0)])
        self.assertFalse(grid.is_free((-1, 0)))


class PlannerTests(unittest.TestCase):
    def test_known_detour_optimum(self):
        # The vertical wall forces a trip along the bottom and back up:
        # (0,0) -> (0,2) has Manhattan distance 2 but shortest length 10.
        grid = load_grid(ROOT / "maps" / "detour.txt")
        for planner in (astar, bfs):
            with self.subTest(planner=planner.__name__):
                result = planner(grid, (0, 0), (0, 2))
                self.assertTrue(result.success)
                self.assertEqual(result.length, 10)
                self.assertEqual(result.length, len(result.path) - 1)
                self.assertTrue(validate_path(grid, result.path, (0, 0), (0, 2)))

    def test_seeded_astar_equals_bfs(self):
        for seed in DEFAULT_SEEDS + tuple(range(50)):
            case = seeded_case(seed)
            with self.subTest(seed=seed):
                actual = astar(case.grid, case.start, case.goal)
                reference = bfs(case.grid, case.start, case.goal)
                self.assertEqual(actual.success, reference.success)
                self.assertEqual(actual.reason, reference.reason)
                self.assertEqual(actual.length, reference.length)
                if actual.success:
                    self.assertTrue(validate_path(case.grid, actual.path, case.start, case.goal))

    def test_bounds_precede_blocked_and_equal_checks(self):
        grid = parse_grid("#.")
        cases = [
            ((-1, 0), (0, 0), "start"),  # OOB start, blocked goal
            ((0, 2), (0, 0), "start"),
            ((1, 0), (-1, 0), "start"),  # both OOB
            ((-1, 0), (-1, 0), "start"),  # equal OOB
            ((0, 0), (0, 2), "goal"),  # blocked start, OOB goal
            ((0, 1), (0, -1), "goal"),
            ((0, 1), (1, 0), "goal"),
        ]
        for planner in (astar, bfs):
            for start, goal, endpoint in cases:
                with self.subTest(planner=planner.__name__, start=start, goal=goal):
                    with self.assertRaisesRegex(ValueError, endpoint + " out of bounds"):
                        planner(grid, start, goal)

    def test_endpoint_results_and_no_path(self):
        cases = [
            ("#.", (0, 0), (0, 1), "start_blocked"),
            ("##", (0, 0), (0, 1), "start_blocked"),
            ("#", (0, 0), (0, 0), "start_blocked"),
            (".#", (0, 0), (0, 1), "goal_blocked"),
            (".", (0, 0), (0, 0), "ok"),
            (".#.", (0, 0), (0, 2), "no_path"),
        ]
        for planner in (astar, bfs):
            for text, start, goal, reason in cases:
                with self.subTest(planner=planner.__name__, text=text, reason=reason):
                    grid = parse_grid(text)
                    result = planner(grid, start, goal)
                    self.assertEqual(result.reason, reason)
                    self.assertEqual(result.success, reason == "ok")
                    self.assertEqual(result.path, [start] if result.success else [])
                    self.assertEqual(result.length, 0 if result.success else None)
                    self.assertEqual(result.nodes_expanded, 1 if reason == "no_path" else 0)

    def test_deterministic_ties(self):
        grid = parse_grid("..\n..")
        expected = [(0, 0), (0, 1), (1, 1)]
        for _ in range(5):
            result = astar(grid, (0, 0), (1, 1))
            self.assertEqual(result.path, expected)
            self.assertEqual(result.nodes_expanded, 4)


class ValidatorTests(unittest.TestCase):
    def test_valid_path(self):
        grid = parse_grid("..\n#.")
        self.assertTrue(validate_path(grid, [(0, 0), (0, 1), (1, 1)], (0, 0), (1, 1)))
        self.assertTrue(validate_path(grid, [(0, 0)], (0, 0), (0, 0)))

    def test_reject_invalid_paths(self):
        grid = parse_grid("..\n#.")
        cases = [
            ([], (0, 0), (1, 1)),
            ([(0, 0), (1, 1)], (0, 0), (1, 1)),  # diagonal
            ([(0, 0), (1, 0), (1, 1)], (0, 0), (1, 1)),  # obstacle
            ([(0, 0), (0, 1)], (0, 1), (0, 1)),  # wrong start
            ([(0, 0), (0, 1)], (0, 0), (1, 1)),  # wrong goal
            ([(0, 0), (-1, 0)], (0, 0), (-1, 0)),  # OOB
            ([(0, 0), (0, 0)], (0, 0), (0, 0)),  # repeated cell
            ([(1, 0)], (1, 0), (1, 0)),  # blocked equal endpoints
        ]
        for path, start, goal in cases:
            with self.subTest(path=path, start=start, goal=goal):
                self.assertFalse(validate_path(grid, path, start, goal))


if __name__ == "__main__":
    unittest.main()
