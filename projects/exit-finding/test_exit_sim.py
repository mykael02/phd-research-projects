import unittest
from exit_sim import Explorer, Grid, World, scenarios, simulate
from gridnav import validate_path


class ExitTests(unittest.TestCase):
    def test_hidden_exit_does_not_change_decision(self):
        grid = Grid((".....", ".....", "....."))
        worlds = [World(grid, (0, 0), target) for target in ((2, 4), (2, 2))]
        observations = [world.observe(world.start) for world in worlds]
        self.assertEqual(observations[0], observations[1])
        moves = []
        for observation in observations:
            explorer = Explorer(3, 5, "frontier", 7)
            explorer.update(observation)
            self.assertIsNone(explorer.exit)
            moves.append(explorer.choose((0, 0)))
        self.assertEqual(moves[0], moves[1])

    def test_frontier_reaches_exit_with_valid_moves(self):
        for name in ("branching", "open_room"):
            world = scenarios()[name]
            result = simulate(world)
            self.assertTrue(result["success"])
            self.assertTrue(validate_path(world.grid, result["path"], world.start, world.exit))
            self.assertEqual(result["collisions"], 0)
            self.assertGreaterEqual(result["steps"], result["oracle_shortest_steps"])

    def test_unreachable_exhausts_without_claiming_success(self):
        result = simulate(scenarios()["unreachable"])
        self.assertEqual(result["reason"], "exploration_exhausted")
        self.assertFalse(result["success"])
        self.assertIsNone(result["oracle_shortest_steps"])

    def test_zero_budget_and_start_at_exit(self):
        self.assertEqual(simulate(scenarios()["branching"], max_steps=0)["reason"], "step_limit")
        result = simulate(World(Grid((".",)), (0, 0), (0, 0)), max_steps=0)
        self.assertTrue(result["success"])
        self.assertEqual(result["steps"], 0)

    def test_random_is_reproducible(self):
        world = scenarios()["branching"]
        self.assertEqual(simulate(world, "random", 19), simulate(world, "random", 19))

    def test_invalid_endpoints_and_budget(self):
        with self.assertRaises(ValueError):
            simulate(World(Grid((".#",)), (0, 0), (0, 1)))
        with self.assertRaises(ValueError):
            simulate(scenarios()["branching"], max_steps=-1)


if __name__ == "__main__":
    unittest.main()
