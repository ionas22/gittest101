from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class Color(str, Enum):
    RED = "red"
    YELLOW = "yellow"
    BLUE = "blue"
    GREEN = "green"
    ORANGE = "orange"


@dataclass(frozen=True)
class GridCell:
    row: int
    col: int

    def __post_init__(self) -> None:
        if not 0 <= self.row < 3 or not 0 <= self.col < 3:
            raise ValueError(f"grid cell outside 3x3 board: {self.row},{self.col}")


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Cube:
    color: Color
    cell: GridCell
    center_px: tuple[int, int]
    top_angle_deg: float = 0.0
    image_confidence: float = 1.0

    def with_cell(self, cell: GridCell) -> "Cube":
        return Cube(
            color=self.color,
            cell=cell,
            center_px=self.center_px,
            top_angle_deg=self.top_angle_deg,
            image_confidence=self.image_confidence,
        )


@dataclass(frozen=True)
class Move:
    source: GridCell
    destination: GridCell
    color: Color
    preserve_orientation: bool = True
