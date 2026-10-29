from __future__ import annotations

import argparse
from glob import glob
import re

import cv2

from .arm import DryRunArm, YahboomArm, cell_pose, execute_pick_place
from .config import RobotConfig
from .models import Color, GridCell
from .planner import reproduction_moves, tower_moves
from .vision import CubeDetector


def parse_order(value: str) -> list[Color]:
    return [Color(item.strip().lower()) for item in value.split(",") if item.strip()]


def capture(config: RobotConfig):
    configured = config.camera_device
    if configured != "auto":
        sources = [configured]
    else:
        sources = sorted(glob("/dev/video*"))
        if not sources:
            sources = list(range(0, 6))

    attempted = []
    for source in sources:
        # Some OpenCV builds on Raspberry Pi reject "/dev/videoN" as a
        # filename but accept the corresponding numeric V4L2 index.
        match = re.fullmatch(r"/dev/video(\d+)", str(source))
        capture_source = int(match.group(1)) if match else source
        camera = cv2.VideoCapture(capture_source, cv2.CAP_V4L2)
        attempted.append(str(source))
        if camera.isOpened():
            ok, frame = camera.read()
            camera.release()
            if ok:
                print(f"Camera: {source}")
                return frame
        else:
            camera.release()
    raise RuntimeError(
        "no capturable camera found. Tried: {}. Check /dev/video* and camera permissions."
        .format(", ".join(attempted))
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("calibrate", "reproduce", "tower"), required=True)
    parser.add_argument("--config", default="config/robot.json")
    parser.add_argument("--order", default="red,yellow,blue,green")
    parser.add_argument("--target", default="2,2")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    config = RobotConfig.load(args.config)
    if args.mode == "calibrate":
        print("Edit source/destination corner points in config/robot.json after calibration.")
        return

    frame = capture(config)
    detector = CubeDetector(config)
    cubes = detector.detect(frame, config.grid_corners("source"))
    if not cubes:
        raise RuntimeError("no cubes detected; check lighting, HSV ranges and calibration")
    print("Detected:", ", ".join(f"{c.color.value}@({c.cell.row},{c.cell.col})" for c in cubes))

    if args.mode == "reproduce":
        moves = reproduction_moves(cubes)
    else:
        row, col = (int(value) for value in args.target.split(",", 1))
        moves = tower_moves(cubes, parse_order(args.order), GridCell(row, col))

    arm = DryRunArm() if args.dry_run else YahboomArm(config)
    source_corners = config.grid_corners("source")
    destination_corners = config.grid_corners("destination")
    for move in moves:
        source = cell_pose(move.source, source_corners, config.arm_value("pick_height", 35))
        destination = cell_pose(move.destination, destination_corners, config.arm_value("place_height", 45))
        execute_pick_place(arm, source, destination, config)


if __name__ == "__main__":
    main()
