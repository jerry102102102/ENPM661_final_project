"""Timestamp helpers for sampled primitive trajectories."""

from __future__ import annotations

from src.models.state import TimedPose2D, TrajectorySegment


def step_based_sample_times(sample_count: int, start_time_s: float, dt_s: float) -> list[float]:
    """Assign sample timestamps with t_i = t0 + i * dt."""

    return [start_time_s + (index * dt_s) for index in range(sample_count)]


def normalized_sample_times(sample_count: int, start_time_s: float, duration_s: float) -> list[float]:
    """Assign sample timestamps with t_i = t0 + alpha_i * T."""

    if sample_count <= 1:
        return [start_time_s]
    return [start_time_s + (duration_s * index / (sample_count - 1)) for index in range(sample_count)]


def annotate_trajectory_samples(
    trajectory: TrajectorySegment,
    start_time_s: float,
    *,
    mode: str = "normalized",
    dt_s: float | None = None,
) -> list[TimedPose2D]:
    """Return trajectory samples paired with absolute timestamps."""

    if mode == "step":
        if dt_s is None:
            if len(trajectory.samples) <= 1:
                dt_s = 0.0
            else:
                dt_s = trajectory.duration_s / (len(trajectory.samples) - 1)
        times = step_based_sample_times(len(trajectory.samples), start_time_s, dt_s)
    elif mode == "normalized":
        times = normalized_sample_times(len(trajectory.samples), start_time_s, trajectory.duration_s)
    else:
        raise ValueError(f"Unknown timestamp mode: {mode}")
    return [TimedPose2D(pose=pose, time_s=time_s) for pose, time_s in zip(trajectory.samples, times)]
