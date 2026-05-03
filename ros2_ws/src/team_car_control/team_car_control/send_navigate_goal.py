"""CLI action client for the team_car_control navigate action server."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from team_car_interfaces.action import NavigateCourse


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send a planning/execution goal to navigate_action_server.")
    parser.add_argument("--start", nargs=3, type=float, metavar=("x", "y", "theta_deg"), required=True)
    parser.add_argument("--goal", nargs=2, type=float, metavar=("x", "y"), required=True)
    parser.add_argument("--rpm1", type=float, required=True)
    parser.add_argument("--rpm2", type=float, required=True)
    parser.add_argument("--clearance", type=float, default=0.02)
    parser.add_argument("--execute", action="store_true")
    return parser.parse_args()


class NavigateGoalClient(Node):
    def __init__(self) -> None:
        super().__init__("send_navigate_goal")
        self.client = ActionClient(self, NavigateCourse, "navigate_course")

    def send_goal(self, args: argparse.Namespace) -> int:
        goal = NavigateCourse.Goal()
        goal.start_x = args.start[0]
        goal.start_y = args.start[1]
        goal.start_theta_deg = args.start[2]
        goal.goal_x = args.goal[0]
        goal.goal_y = args.goal[1]
        goal.rpm1 = args.rpm1
        goal.rpm2 = args.rpm2
        goal.clearance = args.clearance
        goal.motion_profile = "team_car"
        goal.collision_profile = "team_car_circle"
        goal.execute = args.execute

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_course action server is unavailable.")
            return 2

        self.get_logger().info("Sending navigate_course goal.")
        send_goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal was rejected.")
            return 2

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error("No action result received.")
            return 2

        result = wrapped_result.result
        self.get_logger().info(
            f"Result status={wrapped_result.status} success={result.success} message={result.message}"
        )
        self.get_logger().info(
            f"path_cost={result.path_cost:.3f}, expanded_nodes={result.expanded_nodes}, planning_time={result.planning_time_sec:.3f}s, execution_time={result.execution_time_sec:.3f}s"
        )
        return 0 if wrapped_result.status == 4 and result.success else 2

    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"feedback: phase={feedback.phase} step={feedback.current_index}/{feedback.total_indices} pose=({feedback.current_pose.x:.3f}, {feedback.current_pose.y:.3f}, {feedback.current_pose.theta:.3f})"
        )


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = NavigateGoalClient()
    try:
        raise SystemExit(node.send_goal(args))
    except KeyboardInterrupt:
        raise SystemExit(130)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
