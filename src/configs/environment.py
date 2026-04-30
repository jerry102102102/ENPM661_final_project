"""Static environment builders."""

from __future__ import annotations

import math

from src.models.obstacles import StaticWorld
from src.models.state import OrientedBox


WORLD_WIDTH_M = 4.0
WORLD_HEIGHT_M = 2.0
BAR_THICKNESS_M = 0.05
SQUARE_SIZE_M = 0.304


def build_project3_obstacles() -> list[OrientedBox]:
    """Build the Project 3 Part 1 course obstacles in the planner frame."""

    return [
        OrientedBox(
            "bottom_left_bar",
            center_x=0.7483,
            center_y=2.0 - 0.6187,
            width=1.40,
            height=BAR_THICKNESS_M,
            angle_rad=math.radians(-60.0),
        ),
        OrientedBox(
            "top_bar",
            center_x=4.0 - 1.05,
            center_y=2.0 - 0.7250,
            width=1.45,
            height=BAR_THICKNESS_M,
            angle_rad=math.radians(90.0),
        ),
        OrientedBox(
            "bottom_right_bar",
            center_x=1.6113,
            center_y=0.5971,
            width=1.35,
            height=BAR_THICKNESS_M,
            angle_rad=math.radians(60.0),
        ),
        OrientedBox("square_upper_left", 2.2, 2.0 - 0.26, SQUARE_SIZE_M, SQUARE_SIZE_M),
        OrientedBox("square_mid_left", 1.335, 2.0 - 0.45, SQUARE_SIZE_M, SQUARE_SIZE_M),
        OrientedBox("square_lower_right", 0.42, 0.45, SQUARE_SIZE_M, SQUARE_SIZE_M),
    ]


def build_project3_world() -> StaticWorld:
    """Build the original static Part 1 world."""

    return StaticWorld(
        bounds=(0.0, WORLD_WIDTH_M, 0.0, WORLD_HEIGHT_M),
        obstacles=build_project3_obstacles(),
    )
