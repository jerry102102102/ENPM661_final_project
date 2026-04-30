"""Motion primitive definitions and action-set helpers."""

from __future__ import annotations

from dataclasses import dataclass

from src.configs.defaults import VehicleParams


@dataclass(frozen=True)
class MotionPrimitive:
    """A discrete action command used to roll out a local trajectory."""

    command_a: float
    command_b: float
    label: str | None = None


def get_action_set(rpm1: float, rpm2: float, vehicle_params: VehicleParams) -> list[MotionPrimitive]:
    """Return the discrete primitive set for the selected vehicle model."""

    if vehicle_params.motion_model == "rear_drive_steered":
        normal = float(vehicle_params.normal_steering_deg)
        fast = float(vehicle_params.fast_steering_deg)
        return [
            MotionPrimitive(rpm2, 0.0, "straight_fast"),
            MotionPrimitive(rpm1, 0.0, "straight_slow"),
            MotionPrimitive(rpm1, -normal, "right"),
            MotionPrimitive(rpm2, -fast, "right_fast"),
            MotionPrimitive(rpm1, normal, "left"),
            MotionPrimitive(rpm2, fast, "left_fast"),
        ]

    return [
        MotionPrimitive(0.0, rpm1, "right_slow"),
        MotionPrimitive(rpm1, 0.0, "left_slow"),
        MotionPrimitive(rpm1, rpm1, "straight_slow"),
        MotionPrimitive(0.0, rpm2, "right_fast"),
        MotionPrimitive(rpm2, 0.0, "left_fast"),
        MotionPrimitive(rpm2, rpm2, "straight_fast"),
        MotionPrimitive(rpm1, rpm2, "arc_right"),
        MotionPrimitive(rpm2, rpm1, "arc_left"),
    ]
