"""State and trajectory data structures."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    """Continuous planar robot pose."""

    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class OrientedBox:
    """Oriented rectangle obstacle or robot footprint."""

    name: str
    center_x: float
    center_y: float
    width: float
    height: float
    angle_rad: float = 0.0


@dataclass(frozen=True)
class TimedPose2D:
    """Pose with an absolute timestamp."""

    pose: Pose2D
    time_s: float


@dataclass(frozen=True)
class TrajectorySegment:
    """Sampled rollout for one motion primitive."""

    start: Pose2D
    end: Pose2D
    samples: list[Pose2D]
    cost: float
    duration_s: float
    rpm_l: float | None = None
    rpm_r: float | None = None
    steering_angle_deg: float | None = None
    drive_rpm: float | None = None
    action_name: str | None = None
    command: tuple[float, float] | None = None
