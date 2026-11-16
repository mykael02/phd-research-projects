import unittest
from plant_sim import Grid, Plant, demo_plant, path_dose, route, simulate
from gridnav import validate_path


class PlantTests(unittest.TestCase):
    def test_integrated_dose_has_known_answer(self):
        plant = Plant(Grid(("...",)), (0, 0), ((0, 2),), {(0, 0): 0, (0, 1): 2, (0, 2): 4})
        self.assertEqual(path_dose(plant, [(0, 0), (0, 1), (0, 2)]), 4)
        result = simulate(plant, battery=5, dose_budget=12)
        self.assertEqual(result["completed"], 1)
        self.assertEqual(result["dose"], 12)  # round trip 8 + stationary inspection 4
        self.assertEqual(result["elapsed_steps"], 5)

    def test_reserves_inspection_and_return_cost(self):
        plant = Plant(Grid(("...",)), (0, 0), ((0, 2),), {(0, i): 1 for i in range(3)})
        for battery, dose in ((4, 10), (10, 4)):
            result = simulate(plant, battery=battery, dose_budget=dose)
            self.assertEqual(result["completed"], 0)
            self.assertEqual(result["path"], [plant.base])
        self.assertEqual(simulate(plant, battery=5, dose_budget=5)["completed"], 1)

    def test_dose_weight_avoids_hotspot(self):
        grid = Grid((".....", ".....", "....."))
        rates = {(r, c): 0 for r in range(3) for c in range(5)}
        rates[(1, 2)] = 100
        plant = Plant(grid, (1, 0), ((1, 4),), rates)
        shortest = route(plant, plant.base, (1, 4), 0)
        aware = route(plant, plant.base, (1, 4), 3)
        self.assertEqual(len(shortest)-1, 4)
        self.assertEqual(len(aware)-1, 6)
        self.assertEqual(path_dose(plant, aware), 0)
        self.assertTrue(validate_path(grid, aware, plant.base, (1, 4)))

    def test_unreachable_inspection(self):
        plant = Plant(Grid((".#.",)), (0, 0), ((0, 2),), {(0, 0): 0, (0, 2): 0})
        result = simulate(plant)
        self.assertEqual(result["completed"], 0)
        self.assertTrue(result["returned"])

    def test_budget_sweep_returns_valid_paths(self):
        plant = demo_plant()
        for policy in ("shortest", "dose_aware"):
            for battery in (0, 15, 30, 80):
                for budget in (0, 30, 80, 200):
                    result = simulate(plant, policy, battery, budget)
                    self.assertTrue(result["returned"])
                    self.assertTrue(result["within_budgets"])
                    self.assertTrue(validate_path(plant.grid, result["path"], plant.base, plant.base))
                    inspected = result["inspection_order"]
                    self.assertEqual(len(set(inspected)), result["completed"])
                    self.assertTrue(set(inspected) <= set(result["path"]) & set(plant.inspections))
                    expected_dose = path_dose(plant, result["path"]) + sum(plant.rates[p] for p in inspected)
                    self.assertAlmostEqual(result["dose"], expected_dose, places=5)
                    self.assertEqual(result["elapsed_steps"], result["movement_steps"] + result["completed"])

    def test_invalid_rates_and_budgets(self):
        for rate in (-1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                Plant(Grid((".",)), (0, 0), (), {(0, 0): rate})
        with self.assertRaises(ValueError):
            simulate(demo_plant(), battery=-1)


if __name__ == "__main__":
    unittest.main()
