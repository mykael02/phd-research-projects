"""Strict ASCII grids for a point robot; coordinates are (row, column)."""

from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Tuple, Union

Cell = Tuple[int, int]


@dataclass(frozen=True)
class Grid:
    rows: Tuple[str, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "rows", tuple(self.rows))
        if not self.rows or not self.rows[0]:
            raise ValueError("map must contain at least one nonempty row")
        width = len(self.rows[0])
        if any(len(row) != width for row in self.rows):
            raise ValueError("map rows must have equal width")
        if any(cell not in ".#" for row in self.rows for cell in row):
            raise ValueError("map may contain only '.' and '#'")

    @property
    def height(self) -> int:
        return len(self.rows)

    @property
    def width(self) -> int:
        return len(self.rows[0])

    def in_bounds(self, cell: Cell) -> bool:
        row, col = cell
        return 0 <= row < self.height and 0 <= col < self.width

    def is_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and self.rows[cell[0]][cell[1]] == "."

    def neighbors(self, cell: Cell) -> Iterator[Cell]:
        """Yield free neighbors in fixed order: up, right, down, left."""
        row, col = cell
        for dr, dc in ((-1, 0), (0, 1), (1, 0), (0, -1)):
            neighbor = (row + dr, col + dc)
            if self.is_free(neighbor):
                yield neighbor


def parse_grid(text: str) -> Grid:
    """Parse without stripping spaces or blank rows; a final newline is OK."""
    normalized = text.replace("\r\n", "\n")
    if normalized.endswith("\n"):
        normalized = normalized[:-1]
    return Grid(tuple(normalized.split("\n")))


def load_grid(path: Union[str, Path]) -> Grid:
    return parse_grid(Path(path).read_text(encoding="utf-8"))
