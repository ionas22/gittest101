from collections.abc import Iterable

from .models import Color, Cube, GridCell, Move


def reproduction_moves(cubes: Iterable[Cube]) -> list[Move]:
    """Map each source cube to the same row/column on the destination grid."""
    return [
        Move(cube.cell, cube.cell, cube.color, preserve_orientation=True)
        for cube in sorted(cubes, key=lambda item: (item.cell.row, item.cell.col))
    ]


def tower_moves(cubes: Iterable[Cube], order: list[Color], target: GridCell) -> list[Move]:
    available = list(cubes)
    moves: list[Move] = []
    for color in order:
        candidates = [cube for cube in available if cube.color == color]
        if not candidates:
            raise ValueError(f"missing cube required by tower order: {color.value}")
        cube = candidates[0]
        available.remove(cube)
        moves.append(Move(cube.cell, target, cube.color, preserve_orientation=True))
    return moves
