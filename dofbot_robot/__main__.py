from __future__ import annotations

import argparse
from glob import glob
from pathlib import Path
import re

import cv2
import numpy as np

from .arm import DryRunArm, YahboomArm, cell_pose, execute_pick_place
from .config import RobotConfig
from .models import Color, GridCell, Pose
from .planner import reproduction_moves, tower_moves
from .vision import CubeDetector, GridTransform


def parse_order(value: str) -> list[Color]:
    return [Color(item.strip().lower()) for item in value.split(",") if item.strip()]


def capture(config: RobotConfig) -> np.ndarray:
    configured = config.camera_device
    if configured != "auto":
        sources = [configured]
    else:
        sources = sorted(glob("/dev/video*"))
        if not sources:
            sources = list(range(0, 6))

    attempted = []
    width = int(config.values.get("image_width", 1280))
    height = int(config.values.get("image_height", 720))

    for source in sources:
        match = re.fullmatch(r"/dev/video(\d+)", str(source))
        capture_source = int(match.group(1)) if match else source
        camera = cv2.VideoCapture(capture_source, cv2.CAP_V4L2)
        if not camera.isOpened():
            camera.release()
            camera = cv2.VideoCapture(capture_source)

        attempted.append(str(source))
        if camera.isOpened():
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            # Citim cateva cadre pentru stabilizarea auto-expunerii camerei
            ok, frame = False, None
            for _ in range(5):
                ok, frame = camera.read()
            camera.release()
            if ok and frame is not None:
                print(f"Camera conectata: {source} ({frame.shape[1]}x{frame.shape[0]})")
                return frame
        else:
            camera.release()

    raise RuntimeError(
        "Nu s-a gasit nicio camera activa. Incercate: {}. "
        "Verifica /dev/video* sau foloseste --test-image / --image pentru testare."
        .format(", ".join(attempted))
    )


def generate_synthetic_frame(config: RobotConfig) -> np.ndarray:
    """Genereaza o imagine 1280x720 cu 4 cuburi colorate pe caroiajul sursa pentru teste."""
    width = int(config.values.get("image_width", 1280))
    height = int(config.values.get("image_height", 720))
    frame = np.full((height, width, 3), 235, dtype=np.uint8)

    corners = config.grid_corners("source")
    # Plasam 4 cuburi virtuale in celulele (0,0), (0,2), (1,1), (2,0)
    # Culori in format BGR compatibile cu pragurile HSV din robot.json
    test_cubes = [
        (GridCell(0, 0), (0, 0, 220)),     # red
        (GridCell(0, 2), (0, 230, 230)),   # yellow
        (GridCell(1, 1), (220, 90, 0)),    # blue
        (GridCell(2, 0), (0, 190, 0)),     # green
    ]

    for cell, bgr_color in test_cubes:
        center = cell_pose(cell, corners, 0.0)
        cx, cy = int(center.x), int(center.y)
        half_size = 45
        cv2.rectangle(
            frame,
            (cx - half_size, cy - half_size),
            (cx + half_size, cy + half_size),
            bgr_color,
            thickness=-1,
        )
    return frame


def save_calibration_preview(frame: np.ndarray, config: RobotConfig) -> None:
    preview = frame.copy()
    for name, color in (("source", (0, 255, 0)), ("destination", (255, 128, 0))):
        if name in config.values.get("grid", {}):
            pts = np.array(config.grid_corners(name), dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(preview, [pts], isClosed=True, color=color, thickness=3)
            cv2.putText(
                preview,
                name.upper(),
                tuple(pts[0][0]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                color,
                2,
            )

    warped = GridTransform(np.asarray(config.grid_corners("source"))).rectify(frame)
    # Desenam liniile caroiajului 3x3 pe imaginea rectificata
    cell_px = warped.shape[0] // 3
    for i in (1, 2):
        cv2.line(warped, (i * cell_px, 0), (i * cell_px, warped.shape[0]), (0, 255, 0), 2)
        cv2.line(warped, (0, i * cell_px), (warped.shape[1], i * cell_px), (0, 255, 0), 2)

    cv2.imwrite("calibration_preview.jpg", preview)
    cv2.imwrite("calibration_source_rectified.jpg", warped)
    print("Imagini salvate: 'calibration_preview.jpg' si 'calibration_source_rectified.jpg'.")
    print("Ajusteaza punctele din config/robot.json daca liniile nu incadreaza caroiajul 3x3.")


def get_world_corners(config: RobotConfig, grid_name: str) -> list[tuple[float, float]]:
    """Foloseste coordonatele fizice (*_world) daca exista in robot.json, altfel fallback la cele standard."""
    grid_dict = config.values.get("grid", {})
    world_key = f"{grid_name}_world"
    if world_key in grid_dict:
        return config.grid_corners(world_key)
    return config.grid_corners(grid_name)


def main() -> None:
    parser = argparse.ArgumentParser(description="Sistem autonom Yahboom DOFBOT - Concurs")
    parser.add_argument("--mode", choices=("calibrate", "reproduce", "tower"), required=True)
    parser.add_argument("--config", default="config/robot.json")
    parser.add_argument("--order", default="red,yellow,blue,green")
    parser.add_argument("--target", default="2,2")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--image", help="Cale catre o imagine locala pentru testare fara camera")
    parser.add_argument(
        "--test-image",
        action="store_true",
        help="Genereaza automat o imagine sintetica cu 4 cuburi pentru testare rapida",
    )
    args = parser.parse_args()

    config = RobotConfig.load(args.config)
    arm = DryRunArm() if args.dry_run else YahboomArm(config)

    # Daca bratul fizic are metoda de pozitionare pentru camera, o apelam inainte de captura
    if not args.dry_run and hasattr(arm, "move_to_look_pose"):
        arm.move_to_look_pose()

    # 1. Achizitia imaginii (fisier local, imagine sintetica de test sau camera reala)
    if args.image:
        if not Path(args.image).is_file():
            raise FileNotFoundError(f"Imaginea specificata nu exista: {args.image}")
        frame = cv2.imread(args.image)
        if frame is None:
            raise RuntimeError(f"Nu s-a putut citi imaginea: {args.image}")
        print(f"Imagine incarcata din fisier: {args.image}")
    elif args.test_image:
        frame = generate_synthetic_frame(config)
        print("Imagine sintetica generata in memorie (4 cuburi: red, yellow, blue, green).")
    else:
        frame = capture(config)

    # 2. Modul de calibrare vizuala
    if args.mode == "calibrate":
        save_calibration_preview(frame, config)
        return

    # 3. Detectia cuburilor cu OpenCV pe caroiajul sursa (in pixeli)
    detector = CubeDetector(config)
    source_px_corners = config.grid_corners("source")
    cubes = detector.detect(frame, source_px_corners)
    if not cubes:
        raise RuntimeError(
            "Niciun cub detectat! Verifica iluminarea, pragurile HSV din robot.json sau calibrarea."
        )

    print(
        "Cuburi detectate:",
        ", ".join(
            f"{c.color.value}@({c.cell.row},{c.cell.col}) [conf={c.image_confidence:.2f}]"
            for c in sorted(cubes, key=lambda item: (item.cell.row, item.cell.col))
        ),
    )

    # 4. Planificarea mutarilor pentru Sarcina 1 (reproduce) sau Sarcina 2 (tower)
    if args.mode == "reproduce":
        moves = reproduction_moves(cubes)
    else:
        row_str, col_str = args.target.split(",", 1)
        target_cell = GridCell(int(row_str.strip()), int(col_str.strip()))
        moves = tower_moves(cubes, parse_order(args.order), target_cell)

    # 5. Executia fizica / simulata a mutarilor
    source_world_corners = get_world_corners(config, "source")
    destination_world_corners = get_world_corners(config, "destination")

    pick_z = config.arm_value("pick_height", 35.0)
    base_place_z = config.arm_value("place_height", 45.0)
    cube_height = config.arm_value("cube_height", 30.0)

    print(f"Incepere executie ({len(moves)} mutari) in modul '{args.mode}'...")
    for index, move in enumerate(moves):
        # La Sarcina 2 (tower), fiecare cub nou se plaseaza mai sus cu inaltimea unui cub
        current_place_z = (
            base_place_z + (index * cube_height) if args.mode == "tower" else base_place_z
        )

        # Actualizam place_height in dictionarul de configurare pentru compatibilitate
        # directa cu functia execute_pick_place() din arm.py
        if "arm" in config.values and isinstance(config.values["arm"], dict):
            config.values["arm"]["place_height"] = current_place_z

        source_pose = cell_pose(move.source, source_world_corners, pick_z)
        destination_pose = cell_pose(move.destination, destination_world_corners, current_place_z)

        print(
            f"[{index + 1}/{len(moves)}] Mutare {move.color.value.upper()}: "
            f"({move.source.row},{move.source.col}) -> ({move.destination.row},{move.destination.col}) "
            f"| Z_pick={pick_z:.1f}, Z_place={current_place_z:.1f}"
        )
        execute_pick_place(arm, source_pose, destination_pose, config)


if __name__ == "__main__":
    main()