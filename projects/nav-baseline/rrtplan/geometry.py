"""Shared rule G for point robots, with exact closed-square intersections."""

from fractions import Fraction
from math import isfinite
from typing import Iterator, Sequence, Tuple

from gridnav import Cell, Grid

Point = Tuple[float, float]


def cell_center(cell: Cell) -> Point:
    row, col = cell
    return col + 0.5, row + 0.5


def _blocked_squares(grid: Grid) -> Iterator[Tuple[int, int]]:
    for row, cells in enumerate(grid.rows):
        for col, value in enumerate(cells):
            if value == "#":
                yield col, row


def point_is_free(grid: Grid, point: Point) -> bool:
    x, y = point
    if not (isfinite(x) and isfinite(y) and 0 < x < grid.width and 0 < y < grid.height):
        return False
    return not any(
        col <= x <= col + 1 and row <= y <= row + 1
        for col, row in _blocked_squares(grid)
    )


def segment_is_free(grid: Grid, start: Point, end: Point) -> bool:
    """No sampling or tolerance: intersect the closed parameter intervals.

    Fractions make the predicate exact for the supplied binary float values.
    The open map rectangle is convex, so free endpoints exclude outer-edge
    contact anywhere along the segment.
    """
    if not point_is_free(grid, start) or not point_is_free(grid, end):
        return False
    exact_endpoints = None
    for col, row in _blocked_squares(grid):
        # Exact comparisons cheaply exclude squares outside the segment bounds.
        if (max(start[0], end[0]) < col or min(start[0], end[0]) > col + 1
                or max(start[1], end[1]) < row or min(start[1], end[1]) > row + 1):
            continue
        if exact_endpoints is None:
            exact_endpoints = tuple(
                (Fraction(a), Fraction(b)) for a, b in zip(start, end)
            )
        enter, leave = Fraction(0), Fraction(1)
        for (a, b), lower in zip(exact_endpoints, (col, row)):
            delta = b - a
            if delta == 0:
                if not lower <= a <= lower + 1:
                    break
            else:
                first, last = sorted(((lower - a) / delta, (lower + 1 - a) / delta))
                enter, leave = max(enter, first), min(leave, last)
                if enter > leave:
                    break
        else:
            return False  # A single shared parameter (enter == leave) collides.
    return True


def validate_continuous_path(
    grid: Grid, path: Sequence[Point], start_cell: Cell, goal_cell: Cell,
) -> bool:
    if not path or path[0] != cell_center(start_cell) or path[-1] != cell_center(goal_cell):
        return False
    return (
        all(point_is_free(grid, point) for point in path)
        and all(segment_is_free(grid, a, b) for a, b in zip(path, path[1:]))
    )
