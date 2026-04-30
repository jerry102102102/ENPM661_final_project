"""Motion primitive rollout for differential-drive and rear-drive-steered robots."""

from __future__ import annotations

import math

from src.configs.defaults import (
    TEAM_CAR_CALIBRATED_PRIMITIVES,
    TEAM_CAR_PRIMITIVE_COMMAND_DURATION_S,
    VehicleParams,
)
from src.core.angles import normalize_angle
from src.core.costs import arc_length_cost
from src.models.primitives import MotionPrimitive
from src.models.state import Pose2D, TrajectorySegment


def rpm_to_rad_per_sec(rpm: float) -> float:
    """Convert wheel RPM to rad/s."""

    return rpm * 2.0 * math.pi / 60.0


def _transform_local_pose_delta(start_pose: Pose2D, local_x: float, local_y: float, delta_theta: float) -> Pose2D:
    cos_theta = math.cos(start_pose.theta)
    sin_theta = math.sin(start_pose.theta)
    return Pose2D(
        x=start_pose.x + (local_x * cos_theta) - (local_y * sin_theta),
        y=start_pose.y + (local_x * sin_theta) + (local_y * cos_theta),
        theta=normalize_angle(start_pose.theta + delta_theta),
    )


def _simulate_calibrated_team_car_action(
    start_pose: Pose2D,
    primitive: MotionPrimitive,
    vehicle_params: VehicleParams,
    action_duration_s: float,
    integration_dt_s: float,
) -> TrajectorySegment | None:
    if primitive.label not in TEAM_CAR_CALIBRATED_PRIMITIVES:
        return None

    entry = TEAM_CAR_CALIBRATED_PRIMITIVES[primitive.label]
    duration_scale = action_duration_s / max(TEAM_CAR_PRIMITIVE_COMMAND_DURATION_S, 1e-9)
    delta_x = float(entry["mean_delta_x_m"]) * duration_scale
    delta_y = float(entry["mean_delta_y_m"]) * duration_scale
    delta_theta = float(entry["mean_delta_theta_rad"]) * duration_scale
    sample_count = max(1, int(math.ceil(action_duration_s / max(integration_dt_s, 1e-6))))

    samples = [start_pose]
    for index in range(1, sample_count + 1):
        alpha = index / sample_count
        samples.append(_transform_local_pose_delta(start_pose, delta_x * alpha, delta_y * alpha, delta_theta * alpha))

    drive_rpm = primitive.command_a
    return TrajectorySegment(
        start=start_pose,
        end=samples[-1],
        samples=samples,
        cost=arc_length_cost(samples),
        duration_s=action_duration_s,
        rpm_l=vehicle_params.rear_drive_sign_left * drive_rpm,
        rpm_r=vehicle_params.rear_drive_sign_right * drive_rpm,
        steering_angle_deg=primitive.command_b,
        drive_rpm=drive_rpm,
        action_name=primitive.label,
        command=(primitive.command_a, primitive.command_b),
    )


def simulate_primitive(
    start_pose: Pose2D,
    primitive: MotionPrimitive,
    vehicle_params: VehicleParams,
    action_duration_s: float,
    integration_dt_s: float,
) -> TrajectorySegment:
    """Integrate one motion primitive into sampled poses."""

    radius = vehicle_params.wheel_radius_m
    if vehicle_params.motion_model == "rear_drive_steered":
        calibrated = _simulate_calibrated_team_car_action(
            start_pose,
            primitive,
            vehicle_params,
            action_duration_s,
            integration_dt_s,
        )
        if calibrated is not None:
            return calibrated

        drive_rpm = primitive.command_a
        steering_angle_deg = primitive.command_b
        rear_wheel_speed = rpm_to_rad_per_sec(drive_rpm)
        steering_angle_rad = math.radians(steering_angle_deg)
        v = radius * rear_wheel_speed
        omega = 0.0 if abs(steering_angle_rad) < 1e-9 else v * math.tan(steering_angle_rad) / vehicle_params.wheelbase_m
        rpm_l = vehicle_params.rear_drive_sign_left * drive_rpm
        rpm_r = vehicle_params.rear_drive_sign_right * drive_rpm
        rear_to_reference = vehicle_params.rear_to_reference_m
    else:
        rpm_l = primitive.command_a
        rpm_r = primitive.command_b
        wheel_l = rpm_to_rad_per_sec(rpm_l)
        wheel_r = rpm_to_rad_per_sec(rpm_r)
        v = radius * (wheel_l + wheel_r) / 2.0
        omega = radius * (wheel_r - wheel_l) / vehicle_params.track_width_m
        drive_rpm = None
        steering_angle_deg = None
        rear_to_reference = 0.0

    pose = start_pose
    samples = [pose]
    elapsed = 0.0
    while elapsed < action_duration_s - 1e-12:
        dt = min(integration_dt_s, action_duration_s - elapsed)
        mid_theta = pose.theta + (omega * dt / 2.0)
        next_x = pose.x + ((v * math.cos(mid_theta) - rear_to_reference * omega * math.sin(mid_theta)) * dt)
        next_y = pose.y + ((v * math.sin(mid_theta) + rear_to_reference * omega * math.cos(mid_theta)) * dt)
        pose = Pose2D(
            x=next_x,
            y=next_y,
            theta=normalize_angle(pose.theta + (omega * dt)),
        )
        samples.append(pose)
        elapsed += dt

    return TrajectorySegment(
        start=start_pose,
        end=pose,
        samples=samples,
        cost=arc_length_cost(samples),
        duration_s=action_duration_s,
        rpm_l=rpm_l,
        rpm_r=rpm_r,
        steering_angle_deg=steering_angle_deg,
        drive_rpm=drive_rpm,
        action_name=primitive.label,
        command=(primitive.command_a, primitive.command_b),
    )
