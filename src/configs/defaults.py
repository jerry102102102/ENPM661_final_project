"""Default robot, collision, and planner configuration."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path


TEAM_CAR_PRIMITIVE_COMMAND_DURATION_S = 1.0

TEAM_CAR_CALIBRATED_PRIMITIVES: dict[str, dict[str, float]] = {
    "straight_fast": {
        "mean_delta_x_m": 0.11566845375400496,
        "mean_delta_y_m": 4.8189415766231035e-06,
        "mean_delta_theta_rad": 7.940642931636432e-06,
    },
    "straight_slow": {
        "mean_delta_x_m": 0.058423457978905954,
        "mean_delta_y_m": -1.560284794610567e-06,
        "mean_delta_theta_rad": 1.2025572487228687e-05,
    },
    "right": {
        "mean_delta_x_m": 0.05818592380158634,
        "mean_delta_y_m": -0.02401099194670022,
        "mean_delta_theta_rad": -0.23861846748918297,
    },
    "right_fast": {
        "mean_delta_x_m": 0.11065345033669746,
        "mean_delta_y_m": -0.035895947078411636,
        "mean_delta_theta_rad": -0.2809189009231968,
    },
    "left": {
        "mean_delta_x_m": 0.05485691565907379,
        "mean_delta_y_m": 0.019352051012276616,
        "mean_delta_theta_rad": 0.19489734473493786,
    },
    "left_fast": {
        "mean_delta_x_m": 0.10899701496843495,
        "mean_delta_y_m": 0.03147133789016641,
        "mean_delta_theta_rad": 0.24725686102420097,
    },
}


@dataclass(frozen=True)
class VehicleParams:
    """Minimal vehicle parameters needed for primitive rollout."""

    wheel_radius_m: float
    track_width_m: float
    wheelbase_m: float
    motion_model: str
    normal_steering_deg: float = 30.0
    fast_steering_deg: float = 20.0
    rear_drive_sign_left: float = 1.0
    rear_drive_sign_right: float = 1.0
    rear_to_reference_m: float = 0.0


@dataclass(frozen=True)
class CollisionParams:
    """Robot collision footprint."""

    radius_m: float
    source: str = "builtin"
    notes: tuple[str, ...] = ()
    footprint_type: str = "circle"
    length_m: float | None = None
    width_m: float | None = None
    center_offset_m: tuple[float, float] = (0.0, 0.0)


@dataclass(frozen=True)
class PlannerConfig:
    """Baseline planner and environment settings."""

    world_width_m: float = 4.0
    world_height_m: float = 2.0
    xy_resolution_m: float = 0.05
    theta_bins: int = 24
    action_duration_s: float = 1.0
    integration_dt_s: float = 0.05
    goal_tolerance_m: float = 0.15
    goal_heading_rad: float = 0.0
    goal_heading_tolerance_rad: float = math.radians(15.0)
    max_iterations: int = 50000
    output_dir: Path = Path("outputs")

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        return (0.0, self.world_width_m, 0.0, self.world_height_m)


def builtin_vehicle_params(profile_name: str) -> VehicleParams:
    """Return a built-in vehicle profile compatible with the old Part 1 planner."""

    if profile_name == "team_car":
        return VehicleParams(
            wheel_radius_m=0.028042,
            track_width_m=0.1367024,
            wheelbase_m=0.140208,
            motion_model="rear_drive_steered",
            normal_steering_deg=30.0,
            fast_steering_deg=20.0,
            rear_drive_sign_left=-1.0,
            rear_drive_sign_right=1.0,
            rear_to_reference_m=0.070104,
        )
    if profile_name == "turtlebot":
        return VehicleParams(
            wheel_radius_m=0.033,
            track_width_m=0.160,
            wheelbase_m=0.16,
            motion_model="differential_drive",
        )
    raise ValueError(f"Unknown motion profile: {profile_name}")


def builtin_collision_params(profile_name: str) -> CollisionParams:
    """Return a built-in robot footprint profile."""

    if profile_name == "team_car_circle":
        return CollisionParams(radius_m=0.17, source="builtin:team_car_circle")
    if profile_name == "turtlebot_circle":
        return CollisionParams(radius_m=0.17, source="builtin:turtlebot_circle")
    if profile_name == "team_car_box":
        return CollisionParams(
            radius_m=0.16,
            source="builtin:team_car_box",
            footprint_type="box",
            length_m=0.294437,
            width_m=0.140208,
            center_offset_m=(-0.014021, 0.0),
        )
    raise ValueError(f"Unknown collision profile: {profile_name}")
