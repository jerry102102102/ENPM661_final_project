"""Plan with the Phase 2 Part 1 A* code and execute the result in Gazebo."""

from __future__ import annotations

import csv
from datetime import datetime
import math
import traceback
from pathlib import Path
import threading
import time

from geometry_msgs.msg import Pose2D as Pose2DMsg
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path as PathMsg
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage

from team_car_interfaces.action import NavigateCourse

from .phase1_bridge import ensure_phase2_part1_on_path


ensure_phase2_part1_on_path()

from astar_planner import PlannerResult, plan  # type: ignore  # noqa: E402
from collision import is_pose_valid  # type: ignore  # noqa: E402
from config import (  # type: ignore  # noqa: E402
    FALLBACK_STL_PATHS,
    FALLBACK_URDF_PATHS,
    Part1Config,
    resolve_existing_path,
)
from geometry import Pose2D, TrajectorySegment  # type: ignore  # noqa: E402
from io_utils import write_json  # type: ignore  # noqa: E402
from map_spec import build_map_obstacles  # type: ignore  # noqa: E402
from motion_model import rpm_to_rad_per_sec  # type: ignore  # noqa: E402
from urdf_vehicle_model import (  # type: ignore  # noqa: E402
    build_collision_params,
    builtin_vehicle_params,
    parse_team_car_vehicle_params,
)


class NavigateActionServerNode(Node):
    """Expose the Part 1 planner as a ROS2 action server and execute in Gazebo."""

    def __init__(self) -> None:
        super().__init__("navigate_action_server")

        self.callback_group = ReentrantCallbackGroup()
        self.declare_parameter("world_name", "phase2_course")
        self.declare_parameter("model_name", "team_car")
        self.declare_parameter("reference_z", 0.0)
        self.declare_parameter("drive_command_topic", "/rear_wheel_velocity_controller/commands")
        self.declare_parameter("steering_command_topic", "/steering_position_controller/commands")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("steering_joint_names", ["fls_joint", "frs_joint"])
        self.declare_parameter("rear_wheel_joint_names", ["rld_joint", "rrd_joint"])
        self.declare_parameter("command_publish_period_sec", 0.05)
        self.declare_parameter("controller_ready_timeout_sec", 10.0)
        self.declare_parameter("joint_state_ready_timeout_sec", 10.0)
        self.declare_parameter("max_steering_command_rad", 0.7854)
        self.declare_parameter("pose_info_topic", "")
        self.declare_parameter("execution_log_path", "/tmp/team_car_execution_debug_{stamp}.csv")
        self.declare_parameter("planner_log_path", "/tmp/team_car_planner_debug_{stamp}.json")
        self.declare_parameter("tracking_lookahead_m", 0.26)
        self.declare_parameter("tracking_goal_tolerance_m", 0.12)
        self.declare_parameter("tracking_heading_tolerance_rad", 0.35)
        self.declare_parameter("tracking_progress_timeout_sec", 4.0)
        self.declare_parameter("tracking_min_progress_m", 0.04)
        self.declare_parameter("tracking_max_runtime_sec", 70.0)
        self.declare_parameter("tracking_fast_steering_threshold_rad", 0.34)
        self.declare_parameter("tracking_medium_steering_threshold_rad", 0.52)
        self.declare_parameter("tracking_medium_speed_scale", 0.95)
        self.declare_parameter("tracking_slow_speed_scale", 0.80)

        world_name = self.get_parameter("world_name").get_parameter_value().string_value
        self.model_name = self.get_parameter("model_name").get_parameter_value().string_value
        self.reference_z = self._float_parameter("reference_z")
        self.drive_command_topic = self.get_parameter("drive_command_topic").get_parameter_value().string_value
        self.steering_command_topic = (
            self.get_parameter("steering_command_topic").get_parameter_value().string_value
        )
        self.joint_states_topic = self.get_parameter("joint_states_topic").get_parameter_value().string_value
        self.steering_joint_names = list(
            self.get_parameter("steering_joint_names").get_parameter_value().string_array_value
        )
        self.rear_wheel_joint_names = list(
            self.get_parameter("rear_wheel_joint_names").get_parameter_value().string_array_value
        )
        self.command_publish_period_sec = self._float_parameter("command_publish_period_sec")
        self.controller_ready_timeout_sec = self._float_parameter("controller_ready_timeout_sec")
        self.joint_state_ready_timeout_sec = self._float_parameter("joint_state_ready_timeout_sec")
        self.max_steering_command_rad = self._float_parameter("max_steering_command_rad")
        self.pose_info_topic = self.get_parameter("pose_info_topic").get_parameter_value().string_value
        if not self.pose_info_topic:
            self.pose_info_topic = f"/world/{world_name}/pose/info"
        self.execution_log_path_template = (
            self.get_parameter("execution_log_path").get_parameter_value().string_value
        )
        self.planner_log_path_template = (
            self.get_parameter("planner_log_path").get_parameter_value().string_value
        )
        self.tracking_lookahead_m = self._float_parameter("tracking_lookahead_m")
        self.tracking_goal_tolerance_m = self._float_parameter("tracking_goal_tolerance_m")
        self.tracking_heading_tolerance_rad = self._float_parameter("tracking_heading_tolerance_rad")
        self.tracking_progress_timeout_sec = self._float_parameter("tracking_progress_timeout_sec")
        self.tracking_min_progress_m = self._float_parameter("tracking_min_progress_m")
        self.tracking_max_runtime_sec = self._float_parameter("tracking_max_runtime_sec")
        self.tracking_fast_steering_threshold_rad = self._float_parameter("tracking_fast_steering_threshold_rad")
        self.tracking_medium_steering_threshold_rad = self._float_parameter("tracking_medium_steering_threshold_rad")
        self.tracking_medium_speed_scale = self._float_parameter("tracking_medium_speed_scale")
        self.tracking_slow_speed_scale = self._float_parameter("tracking_slow_speed_scale")

        self._joint_lock = threading.Lock()
        self._joint_positions: dict[str, float] = {}
        self._pose_lock = threading.Lock()
        self._latest_model_base_pose: Pose2D | None = None

        self.drive_command_pub = self.create_publisher(Float64MultiArray, self.drive_command_topic, 10)
        self.steering_command_pub = self.create_publisher(Float64MultiArray, self.steering_command_topic, 10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            self.joint_states_topic,
            self._joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.pose_info_sub = self.create_subscription(
            TFMessage,
            self.pose_info_topic,
            self._pose_info_callback,
            10,
            callback_group=self.callback_group,
        )
        self.planned_path_pub = self.create_publisher(PathMsg, "planned_path", 10)
        self.executed_path_pub = self.create_publisher(PathMsg, "executed_path", 10)

        self._action_server = ActionServer(
            self,
            NavigateCourse,
            "navigate_course",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            "navigate_action_server ready. "
            f"model_name={self.model_name}, execution_mode=closed_loop_path, "
            f"drive_command_topic={self.drive_command_topic}, "
            f"steering_command_topic={self.steering_command_topic}, "
            f"joint_states_topic={self.joint_states_topic}, "
            f"pose_info_topic={self.pose_info_topic}, "
            f"execution_log_path={self.execution_log_path_template}, "
            f"planner_log_path={self.planner_log_path_template}, "
            f"tracking_lookahead_m={self.tracking_lookahead_m}, "
            f"tracking_goal_tolerance_m={self.tracking_goal_tolerance_m}"
        )

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        return super().destroy_node()

    def _float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _joint_state_callback(self, message: JointState) -> None:
        with self._joint_lock:
            for index, name in enumerate(message.name):
                if index < len(message.position):
                    self._joint_positions[name] = float(message.position[index])

    def _pose_info_callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if not self._is_model_transform(transform.child_frame_id):
                continue
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            pose = Pose2D(
                float(translation.x),
                float(translation.y),
                self._yaw_from_quaternion_msg(rotation),
            )
            with self._pose_lock:
                self._latest_model_base_pose = pose
            return

    def _is_model_transform(self, child_frame_id: str) -> bool:
        if child_frame_id == self.model_name:
            return True
        tokens = [token for token in child_frame_id.replace("::", "/").split("/") if token]
        return bool(tokens and tokens[-1] == self.model_name)

    def goal_callback(self, goal_request: NavigateCourse.Goal) -> GoalResponse:
        if goal_request.motion_profile and goal_request.motion_profile != "team_car":
            self.get_logger().error("Only motion_profile=team_car is supported.")
            return GoalResponse.REJECT
        if goal_request.collision_profile and goal_request.collision_profile != "team_car_circle":
            self.get_logger().error("Only collision_profile=team_car_circle is supported.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info("Received cancel request.")
        self._stop_joint_control()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> NavigateCourse.Result:
        try:
            return self._execute_callback_impl(goal_handle)
        except Exception as exc:
            self.get_logger().error("Unhandled exception in navigate action callback:\n" + traceback.format_exc())
            self._stop_joint_control()
            goal_handle.abort()
            result = NavigateCourse.Result()
            result.success = False
            result.message = f"Unhandled exception in navigate_action_server: {exc}"
            result.path_cost = float("inf")
            result.expanded_nodes = 0
            result.planning_time_sec = 0.0
            result.execution_time_sec = 0.0
            return result

    def _execute_callback_impl(self, goal_handle) -> NavigateCourse.Result:
        request = goal_handle.request
        goal_handle.publish_feedback(self._feedback("planning", 0, 0, Pose2D(0.0, 0.0, 0.0)))

        planning_started = time.perf_counter()
        plan_result, vehicle_params, config = self._run_plan(request)
        planning_time = time.perf_counter() - planning_started
        planner_log_path = self._write_planner_log(request, plan_result, config)
        if planner_log_path is not None:
            self.get_logger().info(f"Saved planner debug log to {planner_log_path}")

        if not plan_result.success:
            goal_handle.abort()
            return self._result(plan_result, planning_time, 0.0)

        path_samples = plan_result.path_samples
        self.planned_path_pub.publish(self._path_msg(path_samples))

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self._stop_joint_control()
            return self._result(plan_result, planning_time, 0.0, "Goal canceled before execution.")

        execution_time = 0.0
        if request.execute:
            execute_started = time.perf_counter()
            execution_ok, message = self._execute_plan(goal_handle, request, plan_result, vehicle_params, config)
            execution_time = time.perf_counter() - execute_started
            if not execution_ok:
                if message.startswith("Goal canceled"):
                    return self._result(plan_result, planning_time, execution_time, message)
                goal_handle.abort()
                return self._result(plan_result, planning_time, execution_time, message)

        goal_handle.succeed()
        return self._result(plan_result, planning_time, execution_time)

    def _run_plan(self, request: NavigateCourse.Goal) -> tuple[PlannerResult, object, Part1Config]:
        config = Part1Config()
        motion_profile = "team_car"
        collision_profile = "team_car_circle"

        urdf_path = resolve_existing_path(config.urdf_path, FALLBACK_URDF_PATHS)
        stl_path = resolve_existing_path(config.stl_path, FALLBACK_STL_PATHS)
        parsed_team_car_params = parse_team_car_vehicle_params(urdf_path, stl_path)
        vehicle_params = parsed_team_car_params if motion_profile == "team_car" else builtin_vehicle_params(motion_profile)
        collision_params, _ = build_collision_params(collision_profile, stl_path)

        obstacles = build_map_obstacles()
        start_pose = Pose2D(request.start_x, request.start_y, math.radians(request.start_theta_deg))
        requested_goal_xy = (request.goal_x, request.goal_y)
        goal_xy = self._planning_goal_xy(requested_goal_xy, collision_params, request.clearance, config)

        sanity_failures = self._sanity_checks(
            start_pose=start_pose,
            goal_xy=goal_xy,
            clearance=request.clearance,
            collision_params=collision_params,
            obstacles=obstacles,
            config=config,
        )
        if sanity_failures:
            return (
                PlannerResult(
                    success=False,
                    message="; ".join(sanity_failures),
                    runtime_sec=0.0,
                    expanded_nodes=0,
                    path_cost=math.inf,
                    path_segments=[],
                    explored_segments=[],
                    final_pose=None,
                    planner_log={
                        "uncertainty_aware_planning_enabled": False,
                        "primitive_covariance_table": {},
                        "path_nodes": [],
                    },
                ),
                vehicle_params,
                config,
            )

        result = plan(
            start_pose=start_pose,
            goal_xy=goal_xy,
            rpm1=request.rpm1,
            rpm2=request.rpm2,
            clearance=request.clearance,
            vehicle_params=vehicle_params,
            collision_params=collision_params,
            obstacles=obstacles,
            config=config,
        )
        return result, vehicle_params, config

    def _write_planner_log(
        self,
        request: NavigateCourse.Goal,
        plan_result: PlannerResult,
        config: Part1Config,
    ) -> Path | None:
        if not self.planner_log_path_template:
            return None
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = Path(self.planner_log_path_template.replace("{stamp}", stamp)).expanduser()
        payload = {
            "request": {
                "start": {
                    "x": request.start_x,
                    "y": request.start_y,
                    "theta_deg": request.start_theta_deg,
                },
                "goal": {"x": request.goal_x, "y": request.goal_y},
                "rpm1": request.rpm1,
                "rpm2": request.rpm2,
                "clearance": request.clearance,
                "motion_profile": request.motion_profile,
                "collision_profile": request.collision_profile,
                "execute": request.execute,
            },
            "planning_summary": {
                "success": plan_result.success,
                "message": plan_result.message,
                "runtime_sec": plan_result.runtime_sec,
                "expanded_nodes": plan_result.expanded_nodes,
                "path_cost": plan_result.path_cost,
            },
            "config": {
                "action_duration_s": config.action_duration_s,
                "integration_dt_s": config.integration_dt_s,
                "xy_resolution_m": config.xy_resolution_m,
                "theta_bins": config.theta_bins,
                "goal_tolerance_m": config.goal_tolerance_m,
                "enable_uncertainty_aware_planning": config.enable_uncertainty_aware_planning,
                "uncertainty_sigma_scale": config.uncertainty_sigma_scale,
                "uncertainty_heading_radius_scale": config.uncertainty_heading_radius_scale,
                "uncertainty_risk_weight": config.uncertainty_risk_weight,
            },
            "planner_log": plan_result.planner_log,
        }
        write_json(path, payload)
        return path

    def _sanity_checks(
        self,
        *,
        start_pose: Pose2D,
        goal_xy: tuple[float, float],
        clearance: float,
        collision_params,
        obstacles,
        config: Part1Config,
    ) -> list[str]:
        failures: list[str] = []
        min_x, max_x, min_y, max_y = config.bounds
        goal_pose = Pose2D(goal_xy[0], goal_xy[1], 0.0)

        if not (min_x <= start_pose.x <= max_x and min_y <= start_pose.y <= max_y):
            failures.append("Start is outside world bounds.")
        if not (min_x <= goal_xy[0] <= max_x and min_y <= goal_xy[1] <= max_y):
            failures.append("Goal is outside world bounds.")
        if clearance < 0.0:
            failures.append("Clearance must be non-negative.")
        if collision_params.radius_m <= 0.0:
            failures.append("Collision radius must be positive.")
        if not failures and not is_pose_valid(start_pose, collision_params, clearance, obstacles, config.bounds):
            failures.append("Start pose is not collision-free.")
        if not failures and not is_pose_valid(goal_pose, collision_params, clearance, obstacles, config.bounds):
            failures.append("Goal pose is not collision-free.")
        return failures

    @staticmethod
    def _planning_goal_xy(
        requested_goal_xy: tuple[float, float],
        collision_params,
        clearance: float,
        config: Part1Config,
    ) -> tuple[float, float]:
        """Map an end-line request onto a reachable center-point planning goal."""

        requested_x, requested_y = requested_goal_xy
        if requested_x < config.world_width_m:
            return requested_goal_xy

        effective_radius = float(getattr(collision_params, "radius_m", 0.17)) + clearance
        reachable_x = config.world_width_m - effective_radius - 0.01
        return (max(0.0, reachable_x), requested_y)

    def _execute_plan(
        self,
        goal_handle,
        request: NavigateCourse.Goal,
        plan_result: PlannerResult,
        vehicle_params,
        config: Part1Config,
    ) -> tuple[bool, str]:
        return self._execute_closed_loop_path(
            goal_handle,
            plan_result.path_samples,
            plan_result.path_segments,
            vehicle_params,
            plan_result.planner_log,
            request.goal_x if request.goal_x >= config.world_width_m else None,
        )

    def _execute_closed_loop_path(
        self,
        goal_handle,
        path_samples: list[Pose2D],
        path_segments: list[TrajectorySegment],
        vehicle_params,
        planner_log: dict | None,
        finish_line_x: float | None,
    ) -> tuple[bool, str]:
        if not path_samples:
            return True, "No execution needed for an empty path."
        if self.drive_command_pub.get_subscription_count() == 0:
            deadline = time.perf_counter() + self.controller_ready_timeout_sec
            while self.drive_command_pub.get_subscription_count() == 0 and time.perf_counter() < deadline:
                time.sleep(0.05)
            if self.drive_command_pub.get_subscription_count() == 0:
                return False, f"No subscribers on {self.drive_command_topic}; drive controller is not ready."
        if self.steering_command_pub.get_subscription_count() == 0:
            deadline = time.perf_counter() + self.controller_ready_timeout_sec
            while self.steering_command_pub.get_subscription_count() == 0 and time.perf_counter() < deadline:
                time.sleep(0.05)
            if self.steering_command_pub.get_subscription_count() == 0:
                return False, f"No subscribers on {self.steering_command_topic}; steering controller is not ready."

        required_joints = self.steering_joint_names + self.rear_wheel_joint_names
        if not self._wait_for_joint_states(required_joints, self.joint_state_ready_timeout_sec):
            return False, f"Timed out waiting for joint states: {required_joints}"

        base_link_offset = getattr(vehicle_params, "base_link_offset", None) or (0.0, 0.0)
        log_path = self._initialize_execution_log()
        planner_path_nodes = list((planner_log or {}).get("path_nodes", []))
        executed: list[Pose2D] = []
        nearest_index = 0
        start_wall = time.perf_counter()
        last_progress_wall = start_wall
        best_goal_distance = math.inf

        drive_rpms = sorted(
            {
                abs(float(segment.drive_rpm))
                for segment in path_segments
                if segment.drive_rpm is not None and abs(float(segment.drive_rpm)) > 1e-6
            }
        )
        slow_rpm = drive_rpms[0] if drive_rpms else 20.0
        fast_rpm = drive_rpms[-1] if drive_rpms else max(slow_rpm, 40.0)
        goal_pose = path_samples[-1]

        while True:
            if goal_handle.is_cancel_requested:
                self._stop_joint_control()
                goal_handle.canceled()
                return False, "Goal canceled during closed-loop execution."

            if time.perf_counter() - start_wall > self.tracking_max_runtime_sec:
                self._stop_joint_control()
                return False, (
                    "Closed-loop tracking exceeded tracking_max_runtime_sec="
                    f"{self.tracking_max_runtime_sec:.1f}."
                )

            actual_pose = self._latest_model_reference_pose(base_link_offset)
            if actual_pose is None:
                time.sleep(self.command_publish_period_sec)
                continue

            nearest_index = self._find_nearest_path_index(path_samples, actual_pose, nearest_index)
            lookahead_index = self._find_lookahead_index(path_samples, nearest_index, self.tracking_lookahead_m)
            target_pose = path_samples[lookahead_index]
            goal_distance = math.hypot(actual_pose.x - goal_pose.x, actual_pose.y - goal_pose.y)
            heading_to_goal = self._normalize_angle(goal_pose.theta - actual_pose.theta)
            if (
                goal_distance <= self.tracking_goal_tolerance_m
                and abs(heading_to_goal) <= self.tracking_heading_tolerance_rad
            ):
                if finish_line_x is not None and actual_pose.x < finish_line_x:
                    extension_ok, extension_message = self._drive_straight_to_finish_line(
                        goal_handle,
                        finish_line_x,
                        base_link_offset,
                        max(slow_rpm, min(fast_rpm, 30.0)),
                        vehicle_params.rear_drive_sign_left,
                        vehicle_params.rear_drive_sign_right,
                        log_path,
                        executed,
                    )
                    if not extension_ok:
                        return False, extension_message
                    actual_pose = self._latest_model_reference_pose(base_link_offset) or actual_pose
                self._stop_joint_control()
                executed.append(actual_pose)
                self.executed_path_pub.publish(self._path_msg(executed))
                self._append_execution_log_row(
                    log_path,
                    self._closed_loop_log_row(
                        status="goal_reached",
                        message="",
                        actual_pose=actual_pose,
                        target_pose=target_pose,
                        nearest_index=nearest_index,
                        lookahead_index=lookahead_index,
                        steering_target_rad=0.0,
                        drive_rpm=0.0,
                        goal_distance=goal_distance,
                        planner_node=(planner_path_nodes[min(len(planner_path_nodes) - 1, nearest_index)] if planner_path_nodes else None),
                    ),
                )
                return True, "Closed-loop path tracking complete."

            if goal_distance + self.tracking_min_progress_m < best_goal_distance:
                best_goal_distance = goal_distance
                last_progress_wall = time.perf_counter()
            elif time.perf_counter() - last_progress_wall > self.tracking_progress_timeout_sec:
                self._stop_joint_control()
                self._append_execution_log_row(
                    log_path,
                    self._closed_loop_log_row(
                        status="stuck",
                        message="Progress watchdog timeout.",
                        actual_pose=actual_pose,
                        target_pose=target_pose,
                        nearest_index=nearest_index,
                        lookahead_index=lookahead_index,
                        steering_target_rad=0.0,
                        drive_rpm=0.0,
                        goal_distance=goal_distance,
                        planner_node=(planner_path_nodes[min(len(planner_path_nodes) - 1, nearest_index)] if planner_path_nodes else None),
                    ),
                )
                return False, (
                    "Closed-loop tracking stuck: no progress for "
                    f"{self.tracking_progress_timeout_sec:.1f}s, goal_distance={goal_distance:.3f}."
                )

            steering_target_rad = self._pure_pursuit_steering(actual_pose, target_pose, vehicle_params.wheelbase_m)
            steering_target_rad = max(-self.max_steering_command_rad, min(self.max_steering_command_rad, steering_target_rad))
            drive_rpm = self._closed_loop_drive_rpm(abs(steering_target_rad), fast_rpm, slow_rpm, goal_distance)
            left_rpm = vehicle_params.rear_drive_sign_left * drive_rpm
            right_rpm = vehicle_params.rear_drive_sign_right * drive_rpm
            self._publish_steering_command_rad(steering_target_rad)
            self._publish_drive_command(left_rpm, right_rpm)

            executed.append(actual_pose)
            self.executed_path_pub.publish(self._path_msg(executed))
            goal_handle.publish_feedback(
                self._feedback("executing", nearest_index, len(path_samples) - 1, actual_pose)
            )
            self._append_execution_log_row(
                log_path,
                self._closed_loop_log_row(
                    status="tracking",
                    message="",
                    actual_pose=actual_pose,
                    target_pose=target_pose,
                    nearest_index=nearest_index,
                    lookahead_index=lookahead_index,
                    steering_target_rad=steering_target_rad,
                    drive_rpm=drive_rpm,
                    goal_distance=goal_distance,
                    planner_node=(planner_path_nodes[min(len(planner_path_nodes) - 1, nearest_index)] if planner_path_nodes else None),
                ),
            )
            time.sleep(self.command_publish_period_sec)

    def _drive_straight_to_finish_line(
        self,
        goal_handle,
        finish_line_x: float,
        base_link_offset: tuple[float, float],
        drive_rpm: float,
        rear_drive_sign_left: float,
        rear_drive_sign_right: float,
        log_path: Path | None,
        executed: list[Pose2D],
    ) -> tuple[bool, str]:
        """Continue straight until the reference point crosses the finish line."""

        start_wall = time.perf_counter()
        last_progress_wall = start_wall
        best_x = -math.inf
        timeout_sec = 10.0

        while True:
            if goal_handle.is_cancel_requested:
                self._stop_joint_control()
                goal_handle.canceled()
                return False, "Goal canceled during finish-line extension."

            if time.perf_counter() - start_wall > timeout_sec:
                self._stop_joint_control()
                return False, f"Finish-line extension timed out before x={finish_line_x:.3f}."

            actual_pose = self._latest_model_reference_pose(base_link_offset)
            if actual_pose is None:
                time.sleep(self.command_publish_period_sec)
                continue

            if actual_pose.x >= finish_line_x:
                self._stop_joint_control()
                target_pose = Pose2D(finish_line_x, actual_pose.y, 0.0)
                self._append_execution_log_row(
                    log_path,
                    self._closed_loop_log_row(
                        status="finish_line_crossed",
                        message="",
                        actual_pose=actual_pose,
                        target_pose=target_pose,
                        nearest_index=-1,
                        lookahead_index=-1,
                        steering_target_rad=0.0,
                        drive_rpm=0.0,
                        goal_distance=max(0.0, finish_line_x - actual_pose.x),
                        planner_node=None,
                    ),
                )
                return True, "Finish line crossed."

            if actual_pose.x > best_x + self.tracking_min_progress_m:
                best_x = actual_pose.x
                last_progress_wall = time.perf_counter()
            elif time.perf_counter() - last_progress_wall > self.tracking_progress_timeout_sec:
                self._stop_joint_control()
                return False, (
                    "Finish-line extension stuck: no x progress for "
                    f"{self.tracking_progress_timeout_sec:.1f}s, current_x={actual_pose.x:.3f}."
                )

            self._publish_steering_command_rad(0.0)
            self._publish_drive_command(
                rear_drive_sign_left * drive_rpm,
                rear_drive_sign_right * drive_rpm,
            )
            executed.append(actual_pose)
            self.executed_path_pub.publish(self._path_msg(executed))
            goal_handle.publish_feedback(
                self._feedback("finish_line", -1, -1, actual_pose)
            )
            target_pose = Pose2D(finish_line_x, actual_pose.y, 0.0)
            self._append_execution_log_row(
                log_path,
                self._closed_loop_log_row(
                    status="finish_line_extension",
                    message="",
                    actual_pose=actual_pose,
                    target_pose=target_pose,
                    nearest_index=-1,
                    lookahead_index=-1,
                    steering_target_rad=0.0,
                    drive_rpm=drive_rpm,
                    goal_distance=max(0.0, finish_line_x - actual_pose.x),
                    planner_node=None,
                ),
            )
            time.sleep(self.command_publish_period_sec)

    def _initialize_execution_log(self) -> Path | None:
        if not self.execution_log_path_template:
            return None
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = Path(self.execution_log_path_template.replace("{stamp}", stamp)).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as log_file:
            writer = csv.DictWriter(log_file, fieldnames=self._execution_log_fields())
            writer.writeheader()
        self.get_logger().info(f"Writing execution debug log to {path}")
        return path

    @staticmethod
    def _execution_log_fields() -> list[str]:
        return [
            "wall_time_sec",
            "status",
            "message",
            "steering_target_rad",
            "actual_end_x",
            "actual_end_y",
            "actual_end_theta",
            "error_x",
            "error_y",
            "error_theta",
            "error_dist",
            "planner_sigma_xy_m",
            "planner_sigma_theta_deg",
            "planner_uncertainty_margin_m",
            "planner_travel_cost_to_come",
            "planner_risk_cost_to_come",
            "planner_total_cost_to_come",
            "nearest_index",
            "lookahead_index",
            "target_x",
            "target_y",
            "target_theta",
            "goal_distance",
            "drive_rpm_cmd",
        ]

    def _append_execution_log_row(self, log_path: Path | None, row: dict[str, float | int | str | None]) -> None:
        if log_path is None:
            return
        with log_path.open("a", newline="", encoding="utf-8") as log_file:
            writer = csv.DictWriter(log_file, fieldnames=self._execution_log_fields())
            writer.writerow(row)

    def _closed_loop_log_row(
        self,
        *,
        status: str,
        message: str,
        actual_pose: Pose2D,
        target_pose: Pose2D,
        nearest_index: int,
        lookahead_index: int,
        steering_target_rad: float,
        drive_rpm: float,
        goal_distance: float,
        planner_node: dict | None,
    ) -> dict[str, float | int | str | None]:
        return {
            "wall_time_sec": time.time(),
            "status": status,
            "message": message,
            "steering_target_rad": steering_target_rad,
            "actual_end_x": actual_pose.x,
            "actual_end_y": actual_pose.y,
            "actual_end_theta": actual_pose.theta,
            "error_x": actual_pose.x - target_pose.x,
            "error_y": actual_pose.y - target_pose.y,
            "error_theta": self._normalize_angle(actual_pose.theta - target_pose.theta),
            "error_dist": math.hypot(actual_pose.x - target_pose.x, actual_pose.y - target_pose.y),
            "planner_sigma_xy_m": None if planner_node is None else planner_node.get("sigma_xy_m"),
            "planner_sigma_theta_deg": None if planner_node is None else planner_node.get("sigma_theta_deg"),
            "planner_uncertainty_margin_m": None if planner_node is None else planner_node.get("uncertainty_margin_m"),
            "planner_travel_cost_to_come": None if planner_node is None else planner_node.get("travel_cost_to_come"),
            "planner_risk_cost_to_come": None if planner_node is None else planner_node.get("risk_cost_to_come"),
            "planner_total_cost_to_come": None if planner_node is None else planner_node.get("total_cost_to_come"),
            "nearest_index": nearest_index,
            "lookahead_index": lookahead_index,
            "target_x": target_pose.x,
            "target_y": target_pose.y,
            "target_theta": target_pose.theta,
            "goal_distance": goal_distance,
            "drive_rpm_cmd": drive_rpm,
        }

    def _wait_for_joint_states(self, joint_names: list[str], timeout_sec: float) -> bool:
        deadline = time.perf_counter() + timeout_sec
        while rclpy.ok() and time.perf_counter() < deadline:
            positions = self._get_joint_positions(joint_names)
            if positions is not None:
                return True
            time.sleep(0.02)
        return False

    def _get_joint_positions(self, joint_names: list[str]) -> list[float] | None:
        with self._joint_lock:
            if any(name not in self._joint_positions for name in joint_names):
                return None
            return [self._joint_positions[name] for name in joint_names]

    @staticmethod
    def _find_nearest_path_index(path_samples: list[Pose2D], pose: Pose2D, start_index: int) -> int:
        best_index = start_index
        best_distance = math.inf
        search_end = min(len(path_samples), start_index + 40)
        for index in range(start_index, search_end):
            sample = path_samples[index]
            distance = math.hypot(sample.x - pose.x, sample.y - pose.y)
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index

    @staticmethod
    def _find_lookahead_index(path_samples: list[Pose2D], nearest_index: int, lookahead_m: float) -> int:
        accumulated = 0.0
        for index in range(nearest_index + 1, len(path_samples)):
            previous = path_samples[index - 1]
            current = path_samples[index]
            accumulated += math.hypot(current.x - previous.x, current.y - previous.y)
            if accumulated >= lookahead_m:
                return index
        return len(path_samples) - 1

    def _pure_pursuit_steering(self, actual_pose: Pose2D, target_pose: Pose2D, wheelbase_m: float) -> float:
        dx = target_pose.x - actual_pose.x
        dy = target_pose.y - actual_pose.y
        lookahead = max(math.hypot(dx, dy), 1e-6)
        alpha = self._normalize_angle(math.atan2(dy, dx) - actual_pose.theta)
        return math.atan2(2.0 * wheelbase_m * math.sin(alpha), lookahead)

    def _closed_loop_drive_rpm(
        self,
        steering_abs_rad: float,
        fast_rpm: float,
        slow_rpm: float,
        goal_distance: float,
    ) -> float:
        if goal_distance <= self.tracking_goal_tolerance_m * 1.35:
            return slow_rpm
        if steering_abs_rad <= self.tracking_fast_steering_threshold_rad:
            return fast_rpm
        if steering_abs_rad <= self.tracking_medium_steering_threshold_rad:
            return max(slow_rpm, self.tracking_medium_speed_scale * fast_rpm)
        return max(slow_rpm, self.tracking_slow_speed_scale * fast_rpm)

    def _publish_drive_command(self, rpm_l: float, rpm_r: float) -> None:
        left_rad_s = rpm_to_rad_per_sec(rpm_l)
        right_rad_s = rpm_to_rad_per_sec(rpm_r)
        command = Float64MultiArray()
        command.data = [left_rad_s, right_rad_s]
        self.drive_command_pub.publish(command)

    def _publish_steering_command(self, steering_angle_deg: float) -> None:
        self._publish_steering_command_rad(math.radians(steering_angle_deg))

    def _publish_steering_command_rad(self, steering_angle_rad: float) -> None:
        command = Float64MultiArray()
        command.data = [steering_angle_rad, steering_angle_rad]
        self.steering_command_pub.publish(command)

    def _stop_joint_control(self) -> None:
        self._publish_drive_command(0.0, 0.0)
        self._publish_steering_command(0.0)

    def _latest_model_reference_pose(self, base_link_offset: tuple[float, float]) -> Pose2D | None:
        with self._pose_lock:
            base_pose = self._latest_model_base_pose
        if base_pose is None:
            return None
        offset_x, offset_y = base_link_offset
        cos_theta = math.cos(base_pose.theta)
        sin_theta = math.sin(base_pose.theta)
        rotated_x = (offset_x * cos_theta) - (offset_y * sin_theta)
        rotated_y = (offset_x * sin_theta) + (offset_y * cos_theta)
        return Pose2D(
            base_pose.x + rotated_x,
            base_pose.y + rotated_y,
            base_pose.theta,
        )

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Quaternion:
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion

    @staticmethod
    def _yaw_from_quaternion_msg(quaternion: Quaternion) -> float:
        siny_cosp = 2.0 * ((quaternion.w * quaternion.z) + (quaternion.x * quaternion.y))
        cosy_cosp = 1.0 - (2.0 * ((quaternion.y * quaternion.y) + (quaternion.z * quaternion.z)))
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


    def _path_msg(self, samples: list[Pose2D]) -> PathMsg:
        message = PathMsg()
        message.header.frame_id = "world"
        message.header.stamp = self.get_clock().now().to_msg()
        for pose in samples:
            stamped = PoseStamped()
            stamped.header = message.header
            stamped.pose.position.x = pose.x
            stamped.pose.position.y = pose.y
            stamped.pose.position.z = self.reference_z
            stamped.pose.orientation = self._quaternion_from_yaw(pose.theta)
            message.poses.append(stamped)
        return message

    @staticmethod
    def _pose2d_msg(pose: Pose2D) -> Pose2DMsg:
        message = Pose2DMsg()
        message.x = pose.x
        message.y = pose.y
        message.theta = pose.theta
        return message

    def _feedback(self, phase: str, current_index: int, total_indices: int, pose: Pose2D) -> NavigateCourse.Feedback:
        feedback = NavigateCourse.Feedback()
        feedback.phase = phase
        feedback.current_index = current_index
        feedback.total_indices = total_indices
        feedback.current_pose = self._pose2d_msg(pose)
        return feedback

    def _result(
        self,
        plan_result: PlannerResult,
        planning_time: float,
        execution_time: float,
        override_message: str | None = None,
    ) -> NavigateCourse.Result:
        result = NavigateCourse.Result()
        result.success = plan_result.success and override_message is None
        result.message = override_message or plan_result.message
        result.path_cost = float(plan_result.path_cost)
        result.expanded_nodes = int(plan_result.expanded_nodes)
        result.planning_time_sec = planning_time
        result.execution_time_sec = execution_time
        if plan_result.final_pose is not None:
            result.final_pose = self._pose2d_msg(plan_result.final_pose)
        for segment in plan_result.path_segments:
            result.left_rpms.append(float(segment.rpm_l))
            result.right_rpms.append(float(segment.rpm_r))
        for pose in plan_result.path_samples:
            result.planned_path.append(self._pose2d_msg(pose))
        return result


def main() -> None:
    rclpy.init()
    node = NavigateActionServerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node._stop_joint_control()
        except Exception:
            pass
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
