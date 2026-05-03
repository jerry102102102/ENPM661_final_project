"""Closed-loop Gazebo route follower for ACTEA-generated route JSON files."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path as PathMsg
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage


@dataclass(frozen=True)
class RoutePose:
    x: float
    y: float
    theta: float


def rpm_to_rad_per_sec(rpm: float) -> float:
    return rpm * 2.0 * math.pi / 60.0


class ActeaRouteFollower(Node):
    """Track a preplanned ACTEA route with the team car Gazebo controllers."""

    def __init__(self) -> None:
        super().__init__("actea_route_follower")

        self.declare_parameter("route_file", "outputs/gazebo_integration/mbgazworld_route.json")
        self.declare_parameter("world_name", "competition_environment")
        self.declare_parameter("model_name", "team_car")
        self.declare_parameter("pose_info_topic", "")
        self.declare_parameter("drive_command_topic", "/rear_wheel_velocity_controller/commands")
        self.declare_parameter("steering_command_topic", "/steering_position_controller/commands")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("steering_joint_names", ["fls_joint", "frs_joint"])
        self.declare_parameter("rear_wheel_joint_names", ["rld_joint", "rrd_joint"])
        self.declare_parameter("control_period_sec", 0.05)
        self.declare_parameter("lookahead_m", 0.28)
        self.declare_parameter("goal_tolerance_m", 0.16)
        self.declare_parameter("heading_tolerance_rad", 0.45)
        self.declare_parameter("progress_timeout_sec", 5.0)
        self.declare_parameter("min_progress_m", 0.03)
        self.declare_parameter("max_runtime_sec", 90.0)
        self.declare_parameter("wheelbase_m", 0.140208)
        self.declare_parameter("max_steering_rad", 0.7854)
        self.declare_parameter("drive_rpm", 32.0)
        self.declare_parameter("slow_drive_rpm", 20.0)
        self.declare_parameter("rear_drive_sign_left", -1.0)
        self.declare_parameter("rear_drive_sign_right", 1.0)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("planned_start_time_s", -1.0)
        self.declare_parameter("start_time_late_tolerance_s", 0.75)
        self.declare_parameter("execution_log_path", "outputs/gazebo_integration/actea_gazebo_execution_{stamp}.csv")

        self.route_file = Path(str(self.get_parameter("route_file").value)).expanduser()
        self.world_name = str(self.get_parameter("world_name").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.pose_info_topic = str(self.get_parameter("pose_info_topic").value) or f"/world/{self.world_name}/pose/info"
        self.control_period_sec = float(self.get_parameter("control_period_sec").value)
        self.lookahead_m = float(self.get_parameter("lookahead_m").value)
        self.goal_tolerance_m = float(self.get_parameter("goal_tolerance_m").value)
        self.heading_tolerance_rad = float(self.get_parameter("heading_tolerance_rad").value)
        self.progress_timeout_sec = float(self.get_parameter("progress_timeout_sec").value)
        self.min_progress_m = float(self.get_parameter("min_progress_m").value)
        self.max_runtime_sec = float(self.get_parameter("max_runtime_sec").value)
        self.wheelbase_m = float(self.get_parameter("wheelbase_m").value)
        self.max_steering_rad = float(self.get_parameter("max_steering_rad").value)
        self.drive_rpm = float(self.get_parameter("drive_rpm").value)
        self.slow_drive_rpm = float(self.get_parameter("slow_drive_rpm").value)
        self.rear_drive_sign_left = float(self.get_parameter("rear_drive_sign_left").value)
        self.rear_drive_sign_right = float(self.get_parameter("rear_drive_sign_right").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)
        self.start_time_late_tolerance_s = float(self.get_parameter("start_time_late_tolerance_s").value)
        self.execution_log_path_template = str(self.get_parameter("execution_log_path").value)

        self.route_payload = self._load_route_payload(self.route_file)
        self.route = self._route_from_payload(self.route_payload, self.route_file)
        requested_start_time = float(self.get_parameter("planned_start_time_s").value)
        self.planned_start_time_s = (
            requested_start_time
            if requested_start_time >= 0.0
            else float(self.route_payload.get("planned_start_time_s", 0.0))
        )
        self.goal_pose = self.route[-1]
        self.nearest_index = 0
        self.started = False
        self.done = False
        self.start_wall = time.perf_counter()
        self.last_progress_wall = self.start_wall
        self.last_wait_log_wall = 0.0
        self.best_goal_distance = math.inf
        self.execution_log_path = self._initialize_execution_log()

        self._joint_lock = threading.Lock()
        self._joint_positions: dict[str, float] = {}
        self._pose_lock = threading.Lock()
        self._latest_pose: RoutePose | None = None

        self.drive_command_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("drive_command_topic").value),
            10,
        )
        self.steering_command_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("steering_command_topic").value),
            10,
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._joint_state_callback,
            10,
        )
        self.pose_info_sub = self.create_subscription(TFMessage, self.pose_info_topic, self._pose_info_callback, 10)
        self.planned_path_pub = self.create_publisher(PathMsg, "actea_planned_path", 10)
        self.executed_path_pub = self.create_publisher(PathMsg, "actea_executed_path", 10)
        self.executed: list[RoutePose] = []

        self.timer = self.create_timer(self.control_period_sec, self._control_tick)
        self.get_logger().info(
            f"ACTEA route follower loaded {len(self.route)} waypoints from {self.route_file}; "
            f"pose_info_topic={self.pose_info_topic}, auto_start={self.auto_start}, "
            f"planned_start_time_s={self.planned_start_time_s:.2f}"
        )

    def _load_route_payload(self, path: Path) -> dict:
        with path.open("r", encoding="utf-8") as handle:
            return json.load(handle)

    def _route_from_payload(self, payload: dict, path: Path) -> list[RoutePose]:
        if not payload.get("success", False):
            raise RuntimeError(f"Route file does not contain a successful plan: {payload.get('message')}")
        rows = payload.get("gazebo_waypoints") or payload.get("waypoints") or []
        route = [RoutePose(float(row["x"]), float(row["y"]), float(row["theta"])) for row in rows]
        if len(route) < 2:
            raise RuntimeError(f"Route file has too few waypoints: {path}")
        return route

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
            pose = RoutePose(float(translation.x), float(translation.y), self._yaw_from_quaternion_msg(rotation))
            with self._pose_lock:
                self._latest_pose = pose
            return

    def _is_model_transform(self, child_frame_id: str) -> bool:
        if child_frame_id == self.model_name:
            return True
        tokens = [token for token in child_frame_id.replace("::", "/").split("/") if token]
        return bool(tokens and tokens[-1] == self.model_name)

    def _control_tick(self) -> None:
        if self.done or not self.auto_start:
            return
        sim_time_s = self._sim_time_s()
        if sim_time_s < self.planned_start_time_s:
            self.planned_path_pub.publish(self._path_msg(self.route))
            if not self.started and time.perf_counter() - self.last_wait_log_wall >= 1.0:
                self.last_wait_log_wall = time.perf_counter()
                self.get_logger().info(
                    f"Waiting for planned_start_time_s={self.planned_start_time_s:.2f}; "
                    f"current sim time={sim_time_s:.2f}"
                )
            return
        actual_pose = self._latest_model_pose()
        if actual_pose is None:
            self.planned_path_pub.publish(self._path_msg(self.route))
            return
        if not self.started:
            self.started = True
            self.start_wall = time.perf_counter()
            self.last_progress_wall = self.start_wall
            if sim_time_s > self.planned_start_time_s + self.start_time_late_tolerance_s:
                self.get_logger().warn(
                    f"Starting at sim time {sim_time_s:.2f}s, but route was planned for "
                    f"{self.planned_start_time_s:.2f}s. Regenerate the route with "
                    "--start-time close to the intended controller start time for best temporal alignment."
                )
            self.get_logger().info("Starting ACTEA closed-loop route execution.")

        if time.perf_counter() - self.start_wall > self.max_runtime_sec:
            self._finish(False, "Tracking exceeded max runtime.")
            return

        self.nearest_index = self._find_nearest_path_index(self.route, actual_pose, self.nearest_index)
        lookahead_index = self._find_lookahead_index(self.route, self.nearest_index, self.lookahead_m)
        target_pose = self.route[lookahead_index]
        goal_distance = math.hypot(actual_pose.x - self.goal_pose.x, actual_pose.y - self.goal_pose.y)
        heading_error = self._normalize_angle(self.goal_pose.theta - actual_pose.theta)

        if goal_distance <= self.goal_tolerance_m and abs(heading_error) <= self.heading_tolerance_rad:
            self._finish(True, "ACTEA route reached.")
            return

        if goal_distance + self.min_progress_m < self.best_goal_distance:
            self.best_goal_distance = goal_distance
            self.last_progress_wall = time.perf_counter()
        elif time.perf_counter() - self.last_progress_wall > self.progress_timeout_sec:
            self._finish(False, f"Tracking stuck with goal_distance={goal_distance:.3f}.")
            return

        steering = self._pure_pursuit_steering(actual_pose, target_pose)
        steering = max(-self.max_steering_rad, min(self.max_steering_rad, steering))
        drive_rpm = self.slow_drive_rpm if abs(steering) > 0.55 or goal_distance < 0.5 else self.drive_rpm

        self._publish_steering_command_rad(steering)
        self._publish_drive_command(self.rear_drive_sign_left * drive_rpm, self.rear_drive_sign_right * drive_rpm)

        self.executed.append(actual_pose)
        self.planned_path_pub.publish(self._path_msg(self.route))
        self.executed_path_pub.publish(self._path_msg(self.executed))
        self._append_execution_log_row(
            {
                "wall_time_sec": time.time(),
                "sim_time_sec": sim_time_s,
                "status": "tracking",
                "actual_x": actual_pose.x,
                "actual_y": actual_pose.y,
                "actual_theta": actual_pose.theta,
                "target_x": target_pose.x,
                "target_y": target_pose.y,
                "target_theta": target_pose.theta,
                "nearest_index": self.nearest_index,
                "lookahead_index": lookahead_index,
                "goal_distance": goal_distance,
                "steering_target_rad": steering,
                "drive_rpm_cmd": drive_rpm,
                "message": "",
            }
        )

    def _finish(self, success: bool, message: str) -> None:
        self._stop_joint_control()
        self.done = True
        status = "goal_reached" if success else "failed"
        pose = self._latest_model_pose() or self.goal_pose
        self._append_execution_log_row(
            {
                "wall_time_sec": time.time(),
                "sim_time_sec": self._sim_time_s(),
                "status": status,
                "actual_x": pose.x,
                "actual_y": pose.y,
                "actual_theta": pose.theta,
                "target_x": self.goal_pose.x,
                "target_y": self.goal_pose.y,
                "target_theta": self.goal_pose.theta,
                "nearest_index": self.nearest_index,
                "lookahead_index": len(self.route) - 1,
                "goal_distance": math.hypot(pose.x - self.goal_pose.x, pose.y - self.goal_pose.y),
                "steering_target_rad": 0.0,
                "drive_rpm_cmd": 0.0,
                "message": message,
            }
        )
        log_fn = self.get_logger().info if success else self.get_logger().error
        log_fn(message)

    def _latest_model_pose(self) -> RoutePose | None:
        with self._pose_lock:
            return self._latest_pose

    def _sim_time_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    @staticmethod
    def _find_nearest_path_index(path: list[RoutePose], pose: RoutePose, start_index: int) -> int:
        best_index = start_index
        best_distance = math.inf
        search_end = min(len(path), start_index + 60)
        for index in range(start_index, search_end):
            sample = path[index]
            distance = math.hypot(sample.x - pose.x, sample.y - pose.y)
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index

    @staticmethod
    def _find_lookahead_index(path: list[RoutePose], nearest_index: int, lookahead_m: float) -> int:
        accumulated = 0.0
        for index in range(nearest_index + 1, len(path)):
            previous = path[index - 1]
            current = path[index]
            accumulated += math.hypot(current.x - previous.x, current.y - previous.y)
            if accumulated >= lookahead_m:
                return index
        return len(path) - 1

    def _pure_pursuit_steering(self, actual_pose: RoutePose, target_pose: RoutePose) -> float:
        dx = target_pose.x - actual_pose.x
        dy = target_pose.y - actual_pose.y
        lookahead = max(math.hypot(dx, dy), 1e-6)
        alpha = self._normalize_angle(math.atan2(dy, dx) - actual_pose.theta)
        return math.atan2(2.0 * self.wheelbase_m * math.sin(alpha), lookahead)

    def _publish_drive_command(self, rpm_l: float, rpm_r: float) -> None:
        command = Float64MultiArray()
        command.data = [rpm_to_rad_per_sec(rpm_l), rpm_to_rad_per_sec(rpm_r)]
        self.drive_command_pub.publish(command)

    def _publish_steering_command_rad(self, steering_angle_rad: float) -> None:
        command = Float64MultiArray()
        command.data = [steering_angle_rad, steering_angle_rad]
        self.steering_command_pub.publish(command)

    def _stop_joint_control(self) -> None:
        self._publish_drive_command(0.0, 0.0)
        self._publish_steering_command_rad(0.0)

    def _initialize_execution_log(self) -> Path | None:
        if not self.execution_log_path_template:
            return None
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = Path(self.execution_log_path_template.replace("{stamp}", stamp)).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as log_file:
            writer = csv.DictWriter(log_file, fieldnames=self._execution_log_fields())
            writer.writeheader()
        return path

    @staticmethod
    def _execution_log_fields() -> list[str]:
        return [
            "wall_time_sec",
            "sim_time_sec",
            "status",
            "actual_x",
            "actual_y",
            "actual_theta",
            "target_x",
            "target_y",
            "target_theta",
            "nearest_index",
            "lookahead_index",
            "goal_distance",
            "steering_target_rad",
            "drive_rpm_cmd",
            "message",
        ]

    def _append_execution_log_row(self, row: dict[str, float | int | str]) -> None:
        if self.execution_log_path is None:
            return
        with self.execution_log_path.open("a", newline="", encoding="utf-8") as log_file:
            writer = csv.DictWriter(log_file, fieldnames=self._execution_log_fields())
            writer.writerow(row)

    def _path_msg(self, samples: list[RoutePose]) -> PathMsg:
        message = PathMsg()
        message.header.frame_id = "world"
        message.header.stamp = self.get_clock().now().to_msg()
        for pose in samples:
            stamped = PoseStamped()
            stamped.header = message.header
            stamped.pose.position.x = pose.x
            stamped.pose.position.y = pose.y
            stamped.pose.position.z = 0.0
            stamped.pose.orientation = self._quaternion_from_yaw(pose.theta)
            message.poses.append(stamped)
        return message

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


def main() -> None:
    rclpy.init()
    node = ActeaRouteFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop_joint_control()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
