"""Known-map inspection with synthetic dose and an explicit return reserve."""
from dataclasses import dataclass
from heapq import heappop, heappush
from itertools import count
from math import isfinite
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nav-baseline"))
from gridnav import Grid, astar


@dataclass
class Plant:
    grid: Grid
    base: tuple
    inspections: tuple
    rates: dict

    def __post_init__(self):
        if not self.grid.is_free(self.base) or any(not self.grid.is_free(p) for p in self.inspections):
            raise ValueError("base and inspection points must be free")
        if len(set(self.inspections)) != len(self.inspections) or self.base in self.inspections:
            raise ValueError("inspection points must be unique and separate from base")
        for r in range(self.grid.height):
            for c in range(self.grid.width):
                if self.grid.is_free((r, c)):
                    rate = self.rates.get((r, c), float("nan"))
                    if not isfinite(rate) or rate < 0:
                        raise ValueError("every free cell needs a finite, nonnegative dose rate")


def route(plant, start, goal, weight):
    if not isfinite(weight) or weight < 0:
        raise ValueError("weight must be finite and nonnegative")
    if weight == 0:
        return astar(plant.grid, start, goal).path
    order = count()
    queue, costs, parents = [(0.0, next(order), start)], {start: 0.0}, {}
    while queue:
        cost, _, cell = heappop(queue)
        if cost != costs[cell]:
            continue
        if cell == goal:
            path = [cell]
            while path[-1] in parents:
                path.append(parents[path[-1]])
            return list(reversed(path))
        for neighbor in plant.grid.neighbors(cell):
            # Each move lasts one time unit: integrate rate by endpoint average.
            dose = (plant.rates[cell] + plant.rates[neighbor]) / 2
            candidate = cost + 1 + weight * dose
            if candidate < costs.get(neighbor, float("inf")):
                costs[neighbor], parents[neighbor] = candidate, cell
                heappush(queue, (candidate, next(order), neighbor))
    return []


def path_dose(plant, path):
    return sum((plant.rates[a] + plant.rates[b]) / 2 for a, b in zip(path, path[1:]))


def simulate(plant, policy="dose_aware", battery=80, dose_budget=80.0, weight=3.0):
    if policy not in ("shortest", "dose_aware"):
        raise ValueError("unknown policy")
    if not isfinite(battery) or not isfinite(dose_budget) or battery < 0 or dose_budget < 0:
        raise ValueError("budgets must be finite and nonnegative")
    if not isfinite(weight) or weight < 0:
        raise ValueError("weight must be finite and nonnegative")
    planning_weight = 0 if policy == "shortest" else weight
    position, path = plant.base, [plant.base]
    remaining, completed = set(plant.inspections), []
    spent, dose = 0, 0.0
    reserve_path = [plant.base]
    while remaining:
        feasible = []
        for target in sorted(remaining):
            outward = route(plant, position, target, planning_weight)
            home = route(plant, target, plant.base, planning_weight)
            if not outward or not home:
                continue
            # One additional stationary time unit per completed inspection.
            outbound_steps = len(outward) - 1 + 1
            outbound_dose = path_dose(plant, outward) + plant.rates[target]
            if (spent + outbound_steps + len(home)-1 <= battery and
                    dose + outbound_dose + path_dose(plant, home) <= dose_budget + 1e-9):
                score = outbound_steps + planning_weight * outbound_dose
                feasible.append((score, target, outward, home, outbound_steps, outbound_dose))
        if not feasible:
            break
        _, target, outward, reserve_path, steps, added_dose = min(feasible)
        path.extend(outward[1:])
        spent += steps
        dose += added_dose
        position = target
        remaining.remove(target)
        completed.append(target)
    path.extend(reserve_path[1:])
    spent += len(reserve_path) - 1
    dose += path_dose(plant, reserve_path)
    return {"policy": policy, "completed": len(completed), "total_inspections": len(plant.inspections),
            "returned": path[-1] == plant.base, "reason": "all_inspected" if not remaining else "no_feasible_next_inspection",
            "movement_steps": len(path)-1, "elapsed_steps": spent, "dose": round(dose, 6),
            "battery_budget": battery, "dose_budget": dose_budget,
            "within_budgets": spent <= battery and dose <= dose_budget + 1e-9,
            "inspection_order": completed, "path": path}


def demo_plant():
    grid = Grid(tuple(["#############"] + ["#...........#"]*7 + ["#############"]))
    rates = {}
    for r in range(grid.height):
        for c in range(grid.width):
            if grid.is_free((r, c)):
                # Artificial field; no physical shielding or dose calibration.
                rates[(r, c)] = 0.2 + 18 / (1 + (r-4)**2 + (c-6)**2)
    return Plant(grid, (4, 1), ((4, 11), (1, 10), (7, 10)), rates)
