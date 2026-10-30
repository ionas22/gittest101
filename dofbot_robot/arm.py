from __future__ import annotations

from abc import ABC, abstractmethod
from time import sleep

from .config import RobotConfig
from .models import GridCell, Pose


class Arm(ABC):
    @abstractmethod
    def move_to(self, pose: Pose) -> None:
        raise NotImplementedError

    @abstractmethod
    def gripper(self, opening: float) -> None:
        raise NotImplementedError


class DryRunArm(Arm):
    def move_to(self, pose: Pose) -> None:
        print(f"[DRY-RUN] move x={pose.x:.1f} y={pose.y:.1f} z={pose.z:.1f}")

    def gripper(self, opening: float) -> None:
        print(f"[DRY-RUN] gripper={opening:.1f}")


class YahboomArm(Arm):
    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        try:
            from Arm_Lib import Arm_Device  # type: ignore
            from ArmIK.ArmMoveIK import ArmMoveIK  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "Yahboom SDK not found. Use --dry-run or install/configure Arm_Lib and ArmIK."
            ) from exc
        self.device = Arm_Device()
        self.kinematics = ArmMoveIK()

    def move_to_look_pose(self) -> None:
        # Pozitie fixa deasupra mesei pentru ca camera de pe brat sa vada caroiajul
        angles = self.config.values.get("arm", {}).get("look_servos", [90, 130, 0, 0, 90, 30])
        self.device.Arm_serial_servo_write6(*angles, 1000)
        sleep(1.2)

    def move_to(self, pose: Pose) -> None:
        move_time_ms = int(self.config.arm_value("move_time_ms", 1000))
        result = self.kinematics.setPitchRangeMoving(
            (pose.x, pose.y, pose.z), -90, -90, 0, move_time_ms
        )
        if result is False or result is None:
            raise RuntimeError(f"DOFBOT rejected unreachable pose: {pose}")
        # OBLIGATORIU: asteapta ca bratul sa termine miscarea fizica!
        sleep(move_time_ms / 1000.0 + 0.15)

    def gripper(self, opening: float) -> None:
        self.device.Arm_serial_servo_write(6, int(opening), 400)
        sleep(0.45)

def cell_pose(cell: GridCell, corners: list[tuple[float, float]], z: float) -> Pose:
    """Approximate XY mapping from grid cell to the calibrated board plane."""
    top_left, top_right, bottom_right, bottom_left = corners
    u = (cell.col + 0.5) / 3.0
    v = (cell.row + 0.5) / 3.0
    top = (top_left[0] * (1 - u) + top_right[0] * u, top_left[1] * (1 - u) + top_right[1] * u)
    bottom = (
        bottom_left[0] * (1 - u) + bottom_right[0] * u,
        bottom_left[1] * (1 - u) + bottom_right[1] * u,
    )
    return Pose(top[0] * (1 - v) + bottom[0] * v, top[1] * (1 - v) + bottom[1] * v, z)


def execute_pick_place(
    arm: Arm,
    source: Pose,
    destination: Pose,
    config: RobotConfig,
) -> None:
    safe = config.arm_value("safe_height", 140)
    arm.gripper(config.arm_value("grip_open", 40))
    arm.move_to(Pose(source.x, source.y, safe))
    arm.move_to(Pose(source.x, source.y, source.z))
    arm.gripper(config.arm_value("grip_closed", 138))
    sleep(config.arm_value("settle_seconds", 0.35))
    arm.move_to(Pose(source.x, source.y, safe))
    arm.move_to(Pose(destination.x, destination.y, safe))
    arm.move_to(Pose(destination.x, destination.y, destination.z))
    arm.gripper(config.arm_value("grip_open", 40))
    sleep(config.arm_value("settle_seconds", 0.35))
    arm.move_to(Pose(destination.x, destination.y, safe))
