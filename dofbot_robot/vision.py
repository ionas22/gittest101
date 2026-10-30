from __future__ import annotations

from dataclasses import dataclass
from math import atan2, degrees

import cv2
import numpy as np

from .config import RobotConfig
from .models import Color, Cube, GridCell


@dataclass(frozen=True)
class GridTransform:
    corners: np.ndarray
    size: int = 600

    def rectify(self, frame: np.ndarray) -> np.ndarray:
        target = np.float32([[0, 0], [self.size, 0], [self.size, self.size], [0, self.size]])
        matrix = cv2.getPerspectiveTransform(self.corners.astype(np.float32), target)
        return cv2.warpPerspective(frame, matrix, (self.size, self.size))


class CubeDetector:
    def __init__(self, config: RobotConfig, min_area: float = 900.0) -> None:
        self.config = config
        self.min_area = min_area

    def detect(self, frame: np.ndarray, corners: list[tuple[float, float]]) -> list[Cube]:
        warped = GridTransform(np.asarray(corners)).rectify(frame)
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        cubes: list[Cube] = []
        cell_size = warped.shape[0] / 3

        for color_name, (lower, upper) in self.config.colors.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue
                moments = cv2.moments(contour)
                if moments["m00"] == 0:
                    continue
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])
                row = min(2, max(0, int(y / cell_size)))
                col = min(2, max(0, int(x / cell_size)))
                rect = cv2.minAreaRect(contour)
                angle = degrees(rect[2])
                confidence = min(1.0, area / (cell_size * cell_size * 0.45))
                cubes.append(
                    Cube(Color(color_name), GridCell(row, col), (x, y), angle, confidence)
                )
        return self._deduplicate(cubes)

    @staticmethod
    def _deduplicate(cubes: list[Cube]) -> list[Cube]:
        by_cell: dict[GridCell, Cube] = {}
        for cube in cubes:
            current = by_cell.get(cube.cell)
            if current is None or cube.image_confidence > current.image_confidence:
                by_cell[cube.cell] = cube
        return list(by_cell.values())

    @staticmethod
    def orientation_marker_angle(contour: np.ndarray) -> float:
        rect = cv2.minAreaRect(contour)
        angle = float(rect[2])
        return degrees(atan2(rect[1][1], rect[1][0]))
