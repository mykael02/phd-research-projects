"""Shared geometry rule G for continuous positions on a gridnav map.

G1  Grid cell (r, c) maps to the continuous point (x, y) = (c + 0.5, r + 0.5).
G2  The map area is [0, width] x [0, height]. Blocked cell (r, c) is the
    closed square [c, c + 1] x [r, r + 1].
G3  A point is free only if 0 < x < width, 0 < y < height and it lies in no
    closed blocked square. Touching an obstacle or the outer edge is a collision.
G4  A disc with centre p and radius rho >= 0 collides if the Euclidean distance
    from p to any blocked square is <= rho, or if x - rho <= 0,
    x + rho >= width, y - rho <= 0 or y + rho >= height. rho = 0 reduces to G3.
G5  Grid cells that are out of range or blocked map to points in collision.

The point rule (G3) and the disc rule (G4) are implemented separately so that
tests can check that rho = 0 gives the same answer as the point rule.
"""

import math
from typing import Iterator, Tuple

from gridnav import Cell, Grid

Point = Tuple[float, float]


def cell_center(cell: Cell) -> Point:
    """G1: the continuous centre of grid cell (row, col)."""
    row, col = cell
    return (col + 0.5, row + 0.5)


def _is_blocked(grid: Grid, row: int, col: int) -> bool:
    return grid.in_bounds((row, col)) and grid.rows[row][col] == "#"


def _touching_indices(value: float) -> Iterator[int]:
    """Indices k whose closed interval [k, k + 1] contains value."""
    base = int(math.floor(value))
    yield base
    if value == base:
        yield base - 1


def point_in_collision(grid: Grid, point: Point) -> bool:
    """G3: True unless the point is strictly inside the map and off all obstacles.

    Non-finite coordinates are treated as collisions.
    """
    x, y = point
    if not (math.isfinite(x) and math.isfinite(y)):
        return True
    if not (0 < x < grid.width and 0 < y < grid.height):
        return True
    for row in _touching_indices(y):
        for col in _touching_indices(x):
            if _is_blocked(grid, row, col):
                return True
    return False


def disc_in_collision(grid: Grid, center: Point, radius: float) -> bool:
    """G4: disc footprint check, using closed sets (touching is a collision).

    Raises ValueError for a negative or non-finite radius. Non-finite centre
    coordinates are treated as collisions.
    """
    if not math.isfinite(radius) or radius < 0:
        raise ValueError("radius must be finite and >= 0")
    x, y = center
    if not (math.isfinite(x) and math.isfinite(y)):
        return True
    if x - radius <= 0 or x + radius >= grid.width:
        return True
    if y - radius <= 0 or y + radius >= grid.height:
        return True
    # Only squares with col <= x + radius and col + 1 >= x - radius (and the
    # same for rows) can be within distance radius; scan a safe superset.
    row_lo = max(0, int(math.floor(y - radius)) - 1)
    row_hi = min(grid.height - 1, int(math.floor(y + radius)) + 1)
    col_lo = max(0, int(math.floor(x - radius)) - 1)
    col_hi = min(grid.width - 1, int(math.floor(x + radius)) + 1)
    limit = radius * radius
    for row in range(row_lo, row_hi + 1):
        for col in range(col_lo, col_hi + 1):
            if grid.rows[row][col] != "#":
                continue
            dx = max(col - x, 0.0, x - (col + 1))
            dy = max(row - y, 0.0, y - (row + 1))
            if dx * dx + dy * dy <= limit:
                return True
    return False


def cell_in_collision(grid: Grid, cell: Cell, radius: float = 0.0) -> bool:
    """G5: out-of-range or blocked cells are in collision; otherwise G4 at the centre."""
    if not grid.is_free(cell):
        return True
    return disc_in_collision(grid, cell_center(cell), radius)
