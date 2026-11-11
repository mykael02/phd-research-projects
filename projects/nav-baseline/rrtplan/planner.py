"""A deterministic seeded, continuous-space RRT for point robots."""

from dataclasses import dataclass
from math import hypot, isfinite
import random
from typing import List, Optional, Tuple

from gridnav import Cell, Grid
from .geometry import Point, cell_center, point_is_free, segment_is_free

DEFAULT_MAX_ITERATIONS = 2000
DEFAULT_STEP = 0.5
DEFAULT_GOAL_BIAS = 0.1


@dataclass(frozen=True)
class RRTResult:
    success: bool
    reason: str
    path: Tuple[Point, ...]
    nodes: Tuple[Point, ...]
    parents: Tuple[Optional[int], ...]
    iterations: int

    @property
    def length(self) -> Optional[float]:
        if not self.success:
            return None
        return sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.path, self.path[1:]))


def rrt(
    grid: Grid, start_cell: Cell, goal_cell: Cell, seed: int = 0,
    max_iterations: int = DEFAULT_MAX_ITERATIONS,
    step: float = DEFAULT_STEP, goal_bias: float = DEFAULT_GOAL_BIAS,
) -> RRTResult:
    """Return a path or an ordinary failure result; check start before goal.

    Invalid configuration raises ValueError; collision and budget failures do
    not. Each accepted extension attempts a free straight connection to goal.
    """
    start, goal = cell_center(start_cell), cell_center(goal_cell)
    if not point_is_free(grid, start):
        return RRTResult(False, "start_in_collision", (), (), (), 0)
    if not point_is_free(grid, goal):
        return RRTResult(False, "goal_in_collision", (), (), (), 0)
    if start == goal:
        return RRTResult(True, "ok", (start,), (start,), (None,), 0)
    if not isinstance(max_iterations, int) or max_iterations < 0:
        raise ValueError("max_iterations must be a nonnegative integer")
    if not isfinite(step) or step <= 0:
        raise ValueError("step must be finite and positive")
    if not isfinite(goal_bias) or not 0 <= goal_bias <= 1:
        raise ValueError("goal_bias must be between 0 and 1")

    rng = random.Random(seed)
    nodes = [start]
    parents = [None]  # type: List[Optional[int]]
    for iteration in range(1, max_iterations + 1):
        target = goal if rng.random() < goal_bias else (
            rng.uniform(0, grid.width), rng.uniform(0, grid.height)
        )
        nearest = min(range(len(nodes)), key=lambda i: (
            (nodes[i][0] - target[0]) ** 2 + (nodes[i][1] - target[1]) ** 2
        ))
        origin = nodes[nearest]
        dx, dy = target[0] - origin[0], target[1] - origin[1]
        distance = hypot(dx, dy)
        if distance == 0:
            continue
        candidate = target if distance <= step else (
            origin[0] + step * dx / distance, origin[1] + step * dy / distance
        )
        if candidate == origin or not segment_is_free(grid, origin, candidate):
            continue
        nodes.append(candidate)
        parents.append(nearest)
        if segment_is_free(grid, candidate, goal):
            if candidate != goal:
                parents.append(len(nodes) - 1)
                nodes.append(goal)
            path = []
            index = len(nodes) - 1
            while index is not None:
                path.append(nodes[index])
                index = parents[index]
            return RRTResult(True, "ok", tuple(reversed(path)), tuple(nodes), tuple(parents), iteration)
    return RRTResult(False, "budget_exhausted", (), tuple(nodes), tuple(parents), max_iterations)
