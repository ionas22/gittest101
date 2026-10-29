from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class RobotConfig:
    values: dict[str, Any]

    @classmethod
    def load(cls, path: str | Path) -> "RobotConfig":
        with Path(path).open(encoding="utf-8") as handle:
            values = json.load(handle)
        if "grid" not in values or "source" not in values["grid"]:
            raise ValueError("configuration must define source and destination grids")
        return cls(values)

    @property
    def camera_index(self) -> int:
        return int(self.values.get("camera_index", 0))

    @property
    def colors(self) -> dict[str, tuple[tuple[int, int, int], tuple[int, int, int]]]:
        return {
            name: (tuple(bounds[0]), tuple(bounds[1]))
            for name, bounds in self.values["colors"].items()
        }

    def grid_corners(self, name: str) -> list[tuple[float, float]]:
        return [tuple(point) for point in self.values["grid"][name]]

    def arm_value(self, name: str, default: float) -> float:
        return float(self.values.get("arm", {}).get(name, default))
