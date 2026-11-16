"""Unknown-map exit search. Policies receive observations, never the full map."""

from collections import deque
from dataclasses import dataclass
from pathlib import Path
import random
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nav-baseline"))
from gridnav import Grid, astar, bfs


def adjacent(cell, height, width):
    r, c = cell
    for dr, dc in ((-1, 0), (0, 1), (1, 0), (0, -1)):
        other = r + dr, c + dc
        if 0 <= other[0] < height and 0 <= other[1] < width:
            yield other


@dataclass(frozen=True)
class World:
    grid: Grid
    start: tuple
    exit: tuple

    def observe(self, position):
        # Perfect local sensing of the current cell and its four neighbours.
        visible = [position] + list(adjacent(position, self.grid.height, self.grid.width))
        return {cell: (self.grid.is_free(cell), cell == self.exit) for cell in visible}


class Explorer:
    def __init__(self, height, width, policy, seed):
        if policy not in ("frontier", "random"):
            raise ValueError("policy must be frontier or random")
        self.height, self.width, self.policy = height, width, policy
        self.known, self.exit = {}, None
        self.rng = random.Random(seed)

    def update(self, observation):
        for cell, (free, is_exit) in observation.items():
            self.known[cell] = free
            if is_exit:
                self.exit = cell

    def choose(self, position):
        known_grid = Grid(tuple("".join("." if self.known.get((r, c), False) else "#"
                                      for c in range(self.width)) for r in range(self.height)))
        if self.exit is not None:
            route = astar(known_grid, position, self.exit)
            return route.path[1] if route.success and len(route.path) > 1 else None
        if self.policy == "random":
            neighbors = list(known_grid.neighbors(position))
            return self.rng.choice(neighbors) if neighbors else None
        # Nearest reachable observed free cell bordering unobserved space.
        queue, parents = deque([position]), {position: None}
        while queue:
            cell = queue.popleft()
            if any(n not in self.known for n in adjacent(cell, self.height, self.width)):
                route = [cell]
                while parents[route[-1]] is not None:
                    route.append(parents[route[-1]])
                return route[-2] if len(route) > 1 else None
            for neighbor in known_grid.neighbors(cell):
                if neighbor not in parents:
                    parents[neighbor] = cell
                    queue.append(neighbor)
        return None


def simulate(world, policy="frontier", seed=0, max_steps=500):
    if max_steps < 0:
        raise ValueError("max_steps must be nonnegative")
    if not world.grid.is_free(world.start) or not world.grid.is_free(world.exit):
        raise ValueError("start and exit must be free cells")
    explorer = Explorer(world.grid.height, world.grid.width, policy, seed)
    path, collisions, reason = [world.start], 0, "step_limit"
    for step in range(max_steps + 1):
        position = path[-1]
        explorer.update(world.observe(position))
        if position == world.exit:
            reason = "exit_reached"
            break
        if step == max_steps:
            break
        next_cell = explorer.choose(position)
        if next_cell is None:
            reason = "exploration_exhausted" if policy == "frontier" else "trapped"
            break
        if next_cell not in list(world.grid.neighbors(position)):
            collisions += 1
            reason = "invalid_move"
            break
        path.append(next_cell)
    # Full-map reference is computed only after policy execution.
    optimum = bfs(world.grid, world.start, world.exit).length
    return {"policy": policy, "seed": seed, "success": reason == "exit_reached",
            "reason": reason, "steps": len(path)-1, "collisions": collisions,
            "observed_cells": len(explorer.known), "oracle_shortest_steps": optimum,
            "path": path}


def scenarios():
    return {
        "branching": World(Grid(("###########", "#.........#", "#.###.###.#",
                                  "#...#...#.#", "###.#.#.#.#", "#...#.#...#",
                                  "#.###.###.#", "#.........#", "###########")), (1, 1), (7, 9)),
        "open_room": World(Grid(tuple(["###########"] + ["#.........#"]*7 + ["###########"])), (1, 1), (7, 9)),
        "unreachable": World(Grid(("#######", "#..#..#", "#..#..#", "#######")), (1, 1), (1, 5)),
    }
