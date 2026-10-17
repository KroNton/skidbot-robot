#!/usr/bin/env python3
"""
Passive benchmark recorder for Nav2 path planners (A* and Hybrid A*).

Works with any navigation method: RViz 2D Nav Goal, BasicNavigator.goToPose(),
followWaypoints(), followGpsWaypoints(), etc.

Triggers from /compute_path_to_pose/_action/status (planner-server action) so
it is not tied to /goal_pose. Records one CSV row per planned leg.

Usage:
    ros2 run skidbot_navigation planner_benchmark_recorder --ros-args \
        -p planner_name:=astar \
        -p scenario:=open_world \
        -p output_dir:=/home/kronton/planner_results
"""

import csv
import math
import os
from datetime import datetime
from enum import IntEnum

import rclpy
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.time import Time


class State(IntEnum):
    IDLE = 0
    PLANNING = 1    # compute_path_to_pose accepted/executing — timing started
    NAVIGATING = 2  # /plan received, robot heading to goal


class GoalStatus(IntEnum):
    ACCEPTED = 1
    EXECUTING = 2
    SUCCEEDED = 4
    CANCELED = 5
    ABORTED = 6


CSV_HEADER = [
    "timestamp",
    "planner",
    "scenario",
    "start_x",
    "start_y",
    "goal_x",
    "goal_y",
    "straight_line_dist_m",
    "planning_time_ms",
    "path_length_m",
    "path_efficiency",
    "num_waypoints",
    "avg_heading_change_rad",
    "max_heading_change_rad",
    "num_recoveries",
    "success",
    "total_nav_time_s",
]


class PlannerBenchmarkRecorder(Node):
    def __init__(self):
        super().__init__("planner_benchmark_recorder")

        self.declare_parameter("planner_name", "astar")
        self.declare_parameter("output_dir", os.path.expanduser("~/planner_results"))
        self.declare_parameter("csv_filename", "planner_results.csv")
        self.declare_parameter("scenario", "default")
        # use_sim_time is auto-declared by the Node base class

        self._planner = self.get_parameter("planner_name").get_parameter_value().string_value
        output_dir = os.path.expanduser(
            self.get_parameter("output_dir").get_parameter_value().string_value
        )
        csv_filename = self.get_parameter("csv_filename").get_parameter_value().string_value
        self._scenario = self.get_parameter("scenario").get_parameter_value().string_value

        os.makedirs(output_dir, exist_ok=True)
        self._csv_path = os.path.join(output_dir, csv_filename)
        self._init_csv()

        self._state = State.IDLE
        self._current_x = 0.0
        self._current_y = 0.0
        self._start_x = 0.0
        self._start_y = 0.0
        self._goal_x = 0.0
        self._goal_y = 0.0
        self._t_planning_start: Time | None = None
        self._t_goal_start: Time | None = None
        self._planning_time_ms = 0.0
        self._path_length_m = 0.0
        self._num_waypoints = 0
        self._avg_heading_change = 0.0
        self._max_heading_change = 0.0
        self._num_recoveries = 0

        # Track seen compute_path_to_pose goal UUIDs to detect each new planning request.
        self._seen_compute_ids: set[bytes] = set()
        # Previous navigate_to_pose status set — reset each new leg so SUCCEEDED
        # for goal N isn't confused with goal N+1.
        self._prev_nav_statuses: set[int] = set()

        self.create_subscription(Odometry, "/odometry/local", self._odom_cb, 10)
        # Primary trigger: planner_server starts a new computation
        self.create_subscription(
            GoalStatusArray,
            "/compute_path_to_pose/_action/status",
            self._compute_status_cb,
            10,
        )
        # Path metrics + planning_time finalisation
        self.create_subscription(Path, "/plan", self._plan_cb, 10)
        # Navigation completion
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._nav_status_cb,
            10,
        )
        # Recovery count — NavigateToPose feedback carries number_of_recoveries
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            "/navigate_to_pose/_action/feedback",
            self._nav_feedback_cb,
            10,
        )

        self.get_logger().info(
            f"Benchmark recorder ready — planner={self._planner}, "
            f"scenario={self._scenario}, csv={self._csv_path}"
        )

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y

    def _compute_status_cb(self, msg: GoalStatusArray):
        """Detect each new planning request and start the planning-time clock."""
        for s in msg.status_list:
            goal_key = bytes(s.goal_info.goal_id.uuid)
            if goal_key in self._seen_compute_ids:
                continue
            if s.status not in (GoalStatus.ACCEPTED, GoalStatus.EXECUTING):
                continue

            self._seen_compute_ids.add(goal_key)
            if len(self._seen_compute_ids) > 200:
                self._seen_compute_ids.clear()
                self._seen_compute_ids.add(goal_key)

            self._t_planning_start = self.get_clock().now()
            self._t_goal_start = self._t_planning_start
            self._start_x = self._current_x
            self._start_y = self._current_y
            self._num_recoveries = 0
            # Reset nav-status tracker so previous SUCCEEDED doesn't bleed into this leg
            self._prev_nav_statuses = set()
            self._state = State.PLANNING
            self.get_logger().info("Planning started — waiting for /plan")

    def _plan_cb(self, msg: Path):
        if self._state != State.PLANNING:
            return
        if not msg.poses:
            return

        t_now = self.get_clock().now()
        if self._t_planning_start is not None:
            self._planning_time_ms = (t_now - self._t_planning_start).nanoseconds / 1e6

        # Extract goal from the last pose in the path (works for any nav method)
        self._goal_x = msg.poses[-1].pose.position.x
        self._goal_y = msg.poses[-1].pose.position.y

        poses = msg.poses
        self._num_waypoints = len(poses)
        self._path_length_m = self._path_length(poses)
        self._avg_heading_change, self._max_heading_change = self._heading_change_stats(poses)

        self._state = State.NAVIGATING
        self.get_logger().info(
            f"Plan received — length={self._path_length_m:.2f} m, "
            f"waypoints={self._num_waypoints}, "
            f"planning_time={self._planning_time_ms:.1f} ms"
        )

    def _nav_feedback_cb(self, msg: NavigateToPose.Impl.FeedbackMessage):
        if self._state == State.NAVIGATING:
            self._num_recoveries = msg.feedback.number_of_recoveries

    def _nav_status_cb(self, msg: GoalStatusArray):
        if self._state != State.NAVIGATING:
            return

        current_statuses = {s.status for s in msg.status_list}
        new_statuses = current_statuses - self._prev_nav_statuses
        self._prev_nav_statuses = current_statuses

        if GoalStatus.SUCCEEDED in new_statuses:
            self._write_row(success=True)
            self._state = State.IDLE
        elif GoalStatus.ABORTED in new_statuses:
            self._write_row(success=False)
            self._state = State.IDLE

    # ── Metrics helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _path_length(poses) -> float:
        total = 0.0
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
            dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
            total += math.hypot(dx, dy)
        return total

    @staticmethod
    def _heading_change_stats(poses) -> tuple[float, float]:
        """Return (avg_heading_change_rad, max_heading_change_rad)."""
        if len(poses) < 3:
            return 0.0, 0.0
        headings = [
            math.atan2(
                poses[i + 1].pose.position.y - poses[i].pose.position.y,
                poses[i + 1].pose.position.x - poses[i].pose.position.x,
            )
            for i in range(len(poses) - 1)
        ]
        changes = []
        for i in range(1, len(headings)):
            delta = abs(headings[i] - headings[i - 1])
            if delta > math.pi:
                delta = 2 * math.pi - delta
            changes.append(delta)
        if not changes:
            return 0.0, 0.0
        return sum(changes) / len(changes), max(changes)

    # ── CSV helpers ───────────────────────────────────────────────────────────

    def _init_csv(self):
        if not os.path.exists(self._csv_path):
            with open(self._csv_path, "w", newline="") as f:
                csv.writer(f).writerow(CSV_HEADER)
            self.get_logger().info(f"Created CSV: {self._csv_path}")

    def _write_row(self, success: bool):
        t_now = self.get_clock().now()
        total_nav_s = (
            (t_now - self._t_goal_start).nanoseconds / 1e9
            if self._t_goal_start is not None
            else 0.0
        )
        straight_line = math.hypot(
            self._goal_x - self._start_x, self._goal_y - self._start_y
        )
        efficiency = (
            round(straight_line / self._path_length_m, 4)
            if self._path_length_m > 0
            else 0.0
        )
        row = [
            datetime.now().isoformat(timespec="seconds"),
            self._planner,
            self._scenario,
            round(self._start_x, 4),
            round(self._start_y, 4),
            round(self._goal_x, 4),
            round(self._goal_y, 4),
            round(straight_line, 4),
            round(self._planning_time_ms, 1),
            round(self._path_length_m, 4),
            efficiency,
            self._num_waypoints,
            round(self._avg_heading_change, 6),
            round(self._max_heading_change, 6),
            self._num_recoveries,
            success,
            round(total_nav_s, 2),
        ]
        with open(self._csv_path, "a", newline="") as f:
            csv.writer(f).writerow(row)

        self.get_logger().info(
            f"Row written — planner={self._planner}, success={success}, "
            f"efficiency={efficiency:.2f}, planning={self._planning_time_ms:.1f} ms, "
            f"recoveries={self._num_recoveries}, nav={total_nav_s:.1f} s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlannerBenchmarkRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
