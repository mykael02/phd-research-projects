"""Deterministic unit-cost A* and a breadth-first reference search."""

from collections import deque
from dataclasses import dataclass
from heapq import heappop, heappush
from itertools import count
from typing import Dict, List, Optional, Sequence

from .grid import Cell, Grid


@dataclass
class SearchResult:
    success: bool
    path: List[Cell]
    reason: str
    nodes_expanded: int

    @property
    def length(self) -> Optional[int]:
        return len(self.path) - 1 if self.success else None


def validate_path(grid: Grid, path: Sequence[Cell], start: Cell, goal: Cell) -> bool:
    """Check planning validity, not feasibility of execution by a real robot."""
    if not path or path[0] != start or path[-1] != goal:
        return False
    if any(not grid.is_free(cell) for cell in path):
        return False
    return all(
        abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1
        for a, b in zip(path, path[1:])
    )


def _check_endpoints(grid: Grid, start: Cell, goal: Cell) -> Optional[SearchResult]:
    # This order is part of the public API, shared by both planners.
    if not grid.in_bounds(start):
        raise ValueError("start out of bounds")
    if not grid.in_bounds(goal):
        raise ValueError("goal out of bounds")
    if not grid.is_free(start):
        return SearchResult(False, [], "start_blocked", 0)
    if not grid.is_free(goal):
        return SearchResult(False, [], "goal_blocked", 0)
    if start == goal:
        return SearchResult(True, [start], "ok", 0)
    return None


def _success(parents: Dict[Cell, Cell], goal: Cell, expanded: int) -> SearchResult:
    path = [goal]
    while path[-1] in parents:
        path.append(parents[path[-1]])
    path.reverse()
    return SearchResult(True, path, "ok", expanded)


def _manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid: Grid, start: Cell, goal: Cell) -> SearchResult:
    early = _check_endpoints(grid, start, goal)
    if early is not None:
        return early
    order = count()
    frontier = [(_manhattan(start, goal), next(order), 0, start)]
    costs = {start: 0}
    parents = {}  # type: Dict[Cell, Cell]
    expanded = 0
    while frontier:
        _, _, cost, cell = heappop(frontier)
        if cost != costs[cell]:
            continue  # Ignore superseded queue entries.
        expanded += 1
        if cell == goal:
            return _success(parents, goal, expanded)
        for neighbor in grid.neighbors(cell):
            next_cost = cost + 1
            if neighbor not in costs or next_cost < costs[neighbor]:
                costs[neighbor] = next_cost
                parents[neighbor] = cell
                heappush(frontier, (
                    next_cost + _manhattan(neighbor, goal),
                    next(order), next_cost, neighbor,
                ))
    return SearchResult(False, [], "no_path", expanded)


def bfs(grid: Grid, start: Cell, goal: Cell) -> SearchResult:
    """FIFO reference; uses no heuristic or A* priority/cost logic."""
    early = _check_endpoints(grid, start, goal)
    if early is not None:
        return early
    frontier = deque([start])
    visited = {start}
    parents = {}  # type: Dict[Cell, Cell]
    expanded = 0
    while frontier:
        cell = frontier.popleft()
        expanded += 1
        if cell == goal:
            return _success(parents, goal, expanded)
        for neighbor in grid.neighbors(cell):
            if neighbor not in visited:
                visited.add(neighbor)
                parents[neighbor] = cell
                frontier.append(neighbor)
    return SearchResult(False, [], "no_path", expanded)
