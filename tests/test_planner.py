import unittest

from dofbot_robot.models import Color, Cube, GridCell
from dofbot_robot.planner import reproduction_moves, tower_moves


def cube(color: Color, row: int, col: int) -> Cube:
    return Cube(color, GridCell(row, col), (100, 100))


class PlannerTests(unittest.TestCase):
    def test_reproduction_preserves_cell_and_orientation(self) -> None:
        item = cube(Color.RED, 1, 2)
        move = reproduction_moves([item])[0]
        self.assertEqual(move.source, GridCell(1, 2))
        self.assertEqual(move.destination, GridCell(1, 2))
        self.assertTrue(move.preserve_orientation)

    def test_tower_uses_requested_bottom_to_top_order(self) -> None:
        cubes = [cube(Color.BLUE, 0, 0), cube(Color.RED, 1, 1)]
        moves = tower_moves(cubes, [Color.RED, Color.BLUE], GridCell(2, 2))
        self.assertEqual([move.color for move in moves], [Color.RED, Color.BLUE])

    def test_invalid_cell_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            GridCell(3, 0)
