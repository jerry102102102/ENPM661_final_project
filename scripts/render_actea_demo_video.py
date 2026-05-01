#!/usr/bin/env python3
"""Render a short 2D ACTEA planning demo as GIF and MP4."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

REPO_ROOT = Path(__file__).resolve().parents[1]
if REPO_ROOT.as_posix() not in sys.path:
    sys.path.insert(0, REPO_ROOT.as_posix())

from PIL import Image, ImageDraw, ImageFont

from src.experiments.method_registry import build_sampled_temporal_planner
from src.experiments.scenarios import make_repeated_query_config, repeated_query_workload
from src.models.obstacles import DynamicCircleObstacle
from src.models.state import Pose2D, TrajectorySegment


def _project(
    x: float,
    y: float,
    *,
    width: int,
    height: int,
    margin: int,
    world_size: float,
) -> tuple[float, float]:
    px = margin + (x / world_size) * (width - 2 * margin)
    py = height - margin - (y / world_size) * (height - 2 * margin)
    return px, py


def _timed_path_samples(segments: list[TrajectorySegment]) -> list[tuple[float, Pose2D]]:
    timed: list[tuple[float, Pose2D]] = []
    current_time = 0.0
    for segment in segments:
        sample_count = max(len(segment.samples), 1)
        for index, pose in enumerate(segment.samples):
            if timed and index == 0:
                continue
            alpha = index / max(sample_count - 1, 1)
            timed.append((current_time + alpha * segment.duration_s, pose))
        current_time += segment.duration_s
    return timed


def _pose_at(timed_samples: list[tuple[float, Pose2D]], time_s: float) -> Pose2D:
    if not timed_samples:
        raise ValueError("Cannot interpolate an empty path.")
    if time_s <= timed_samples[0][0]:
        return timed_samples[0][1]
    for (t0, p0), (t1, p1) in zip(timed_samples, timed_samples[1:]):
        if time_s <= t1:
            alpha = (time_s - t0) / max(t1 - t0, 1e-9)
            return Pose2D(
                p0.x + alpha * (p1.x - p0.x),
                p0.y + alpha * (p1.y - p0.y),
                p0.theta + alpha * (p1.theta - p0.theta),
            )
    return timed_samples[-1][1]


def _obstacle_position(obstacle: DynamicCircleObstacle, time_s: float) -> tuple[float, float]:
    return obstacle.initial_x + obstacle.velocity_x * time_s, obstacle.initial_y + obstacle.velocity_y * time_s


def _demo_dynamic_obstacles() -> list[DynamicCircleObstacle]:
    """Dense moving-ball scene for the short visualization demo."""

    return [
        DynamicCircleObstacle(1.0, -0.5, 0.00, 0.48, 0.12, "north_1"),
        DynamicCircleObstacle(2.0, 8.5, 0.00, -0.46, 0.12, "south_1"),
        DynamicCircleObstacle(3.0, -0.6, 0.00, 0.44, 0.12, "north_2"),
        DynamicCircleObstacle(4.0, 8.6, 0.00, -0.42, 0.12, "south_2"),
        DynamicCircleObstacle(5.0, -0.5, 0.00, 0.46, 0.12, "north_3"),
        DynamicCircleObstacle(6.0, 8.5, 0.00, -0.44, 0.12, "south_3"),
        DynamicCircleObstacle(-0.5, 2.0, 0.46, 0.00, 0.12, "east_1"),
        DynamicCircleObstacle(8.5, 3.0, -0.44, 0.00, 0.12, "west_1"),
        DynamicCircleObstacle(-0.6, 4.2, 0.42, 0.00, 0.12, "east_2"),
        DynamicCircleObstacle(8.6, 5.3, -0.46, 0.00, 0.12, "west_2"),
        DynamicCircleObstacle(1.2, 8.5, 0.26, -0.36, 0.11, "diag_1"),
        DynamicCircleObstacle(8.4, 1.2, -0.36, 0.26, 0.11, "diag_2"),
        DynamicCircleObstacle(6.9, 8.4, -0.30, -0.34, 0.11, "diag_3"),
        DynamicCircleObstacle(0.8, 0.7, 0.36, 0.30, 0.11, "diag_4"),
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Render a 5-second ACTEA 2D demo video.")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/demo"))
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--world-size", type=float, default=8.0)
    parser.add_argument("--xy-samples", type=int, default=260)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    config = make_repeated_query_config(world_size_m=args.world_size, xy_sample_count=args.xy_samples)
    obstacles = _demo_dynamic_obstacles()
    start, goal = repeated_query_workload(args.world_size, 4)[3]
    planner, build_time, annotation_time = build_sampled_temporal_planner("actea", start, obstacles, config)
    result = planner.plan(start, goal, obstacles, clearance=config.clearance)
    if not result.success or result.path is None:
        raise RuntimeError(f"ACTEA demo planning failed: {result.message}")

    timed_samples = _timed_path_samples(result.path.segments)
    sim_duration = max(result.path.total_traversal_time, 1e-9)
    frame_count = max(int(args.duration_sec * args.fps), 1)
    width, height, margin = 960, 960, 70
    world_size = config.planner_config.world_width_m
    robot_radius_px = config.collision_params.radius_m / world_size * (width - 2 * margin)
    font = ImageFont.load_default()
    frames: list[Image.Image] = []

    path_points = [
        _project(pose.x, pose.y, width=width, height=height, margin=margin, world_size=world_size)
        for _, pose in timed_samples
    ]
    start_px = _project(start.x, start.y, width=width, height=height, margin=margin, world_size=world_size)
    goal_px = _project(goal.x, goal.y, width=width, height=height, margin=margin, world_size=world_size)

    for frame_index in range(frame_count):
        alpha = frame_index / max(frame_count - 1, 1)
        sim_time = alpha * sim_duration
        image = Image.new("RGB", (width, height), "white")
        draw = ImageDraw.Draw(image, "RGBA")
        draw.rectangle((margin, margin, width - margin, height - margin), fill=(248, 250, 252, 255), outline=(51, 65, 85, 255), width=2)

        for obstacle in config.static_world.obstacles:
            x0, y0 = _project(obstacle.center_x - obstacle.width / 2.0, obstacle.center_y + obstacle.height / 2.0, width=width, height=height, margin=margin, world_size=world_size)
            x1, y1 = _project(obstacle.center_x + obstacle.width / 2.0, obstacle.center_y - obstacle.height / 2.0, width=width, height=height, margin=margin, world_size=world_size)
            draw.rectangle((x0, y0, x1, y1), fill=(71, 85, 105, 120), outline=(30, 41, 59, 220))

        if len(path_points) > 1:
            draw.line(path_points, fill=(37, 99, 235, 90), width=4, joint="curve")

        draw.ellipse((start_px[0] - 8, start_px[1] - 8, start_px[0] + 8, start_px[1] + 8), fill=(22, 163, 74, 255))
        draw.text((start_px[0] + 10, start_px[1] - 12), "start", fill=(22, 101, 52, 255), font=font)
        draw.ellipse((goal_px[0] - 10, goal_px[1] - 10, goal_px[0] + 10, goal_px[1] + 10), fill=(124, 58, 237, 255))
        draw.text((goal_px[0] + 12, goal_px[1] - 12), "goal", fill=(88, 28, 135, 255), font=font)

        for obstacle in obstacles:
            trail: list[tuple[float, float]] = []
            for trail_step in range(14):
                trail_time = max(0.0, sim_time - trail_step * 0.35)
                ox, oy = _obstacle_position(obstacle, trail_time)
                trail.append(_project(ox, oy, width=width, height=height, margin=margin, world_size=world_size))
            if len(trail) > 1:
                draw.line(trail, fill=(239, 68, 68, 70), width=3)
            ox, oy = _obstacle_position(obstacle, sim_time)
            px, py = _project(ox, oy, width=width, height=height, margin=margin, world_size=world_size)
            radius_px = obstacle.radius / world_size * (width - 2 * margin)
            draw.ellipse((px - radius_px, py - radius_px, px + radius_px, py + radius_px), fill=(239, 68, 68, 170), outline=(127, 29, 29, 255), width=2)

        robot = _pose_at(timed_samples, sim_time)
        rx, ry = _project(robot.x, robot.y, width=width, height=height, margin=margin, world_size=world_size)
        draw.ellipse((rx - robot_radius_px, ry - robot_radius_px, rx + robot_radius_px, ry + robot_radius_px), fill=(37, 99, 235, 255), outline=(30, 64, 175, 255), width=2)
        hx = rx + 18.0 * math.cos(robot.theta)
        hy = ry - 18.0 * math.sin(robot.theta)
        draw.line((rx, ry, hx, hy), fill=(255, 255, 255, 255), width=3)
        draw.text((70, 25), "ACTEA 2D dynamic-obstacle demo", fill=(15, 23, 42, 255), font=font)
        draw.text((70, 45), f"animation: {args.duration_sec:.1f}s   simulated time: {sim_time:.1f}/{sim_duration:.1f}s", fill=(51, 65, 85, 255), font=font)
        draw.text((70, 65), f"build={build_time:.2f}s annotation={annotation_time:.2f}s query labels={result.expanded_labels}", fill=(51, 65, 85, 255), font=font)
        frames.append(image)

    gif_path = args.output_dir / "actea_2d_demo.gif"
    mp4_path = args.output_dir / "actea_2d_demo.mp4"
    frames[0].save(
        gif_path,
        save_all=True,
        append_images=frames[1:],
        duration=int(1000 / args.fps),
        loop=0,
    )

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            for index, frame in enumerate(frames):
                frame.save(tmp_path / f"frame_{index:04d}.png")
            subprocess.run(
                [
                    ffmpeg,
                    "-y",
                    "-framerate",
                    str(args.fps),
                    "-i",
                    str(tmp_path / "frame_%04d.png"),
                    "-pix_fmt",
                    "yuv420p",
                    "-movflags",
                    "+faststart",
                    str(mp4_path),
                ],
                check=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

    print(f"Wrote {gif_path}")
    if mp4_path.exists():
        print(f"Wrote {mp4_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
