#!/usr/bin/env python3
"""
Records planned path poses and actual robot trajectory to CSV for offline
visualization and plan-shape comparison between A* and Hybrid A*.

Two output files (appended across runs):
  plan_paths.csv         — one row per pose in each /plan message
  robot_trajectories.csv — one row per odometry sample while navigating

Both files share a run_id column so rows from the same leg can be joined
in pandas/matplotlib for overlaid plotting.

Triggers on /compute_path_to_pose/_action/status so it works with any
navigation method (RViz goal, followGpsWaypoints, goToPose, etc.).

Usage:
    ros2 run skidbot_navigation planner_path_recorder --ros-args \
        -p planner_name:=astar \
        -p scenario:=open_world \
        -p output_dir:=/home/kronton/planner_results

Quick plot (pandas + matplotlib):
    import pandas as pd, matplotlib.pyplot as plt
    plans = pd.read_csv("plan_paths.csv")
    trajs = pd.read_csv("robot_trajectories.csv")
    for rid, g in plans.groupby("run_id"):
        plt.plot(g.x, g.y, label=g.planner.iloc[0])
    plt.show()
"""

import csv
import math
import os
from datetime import datetime
from enum import IntEnum

import rclpy
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.time import Time


class State(IntEnum):
    IDLE = 0
    PLANNING = 1    # compute_path_to_pose detected — waiting for /plan
    NAVIGATING = 2  # plan saved, robot heading to goal — sampling odom


class GoalStatus(IntEnum):
    ACCEPTED = 1
    EXECUTING = 2
    SUCCEEDED = 4
    CANCELED = 5
    ABORTED = 6


PLAN_HEADER = [
    "run_id",
    "planner",
    "scenario",
    "pose_index",
    "x",
    "y",
    "yaw_rad",
]

TRAJ_HEADER = [
    "run_id",
    "planner",
    "scenario",
    "sample_index",
    "elapsed_s",
    "x",
    "y",
    "yaw_rad",
]

# Minimum seconds between trajectory samples to avoid huge files at 50 Hz odom.
TRAJ_SAMPLE_INTERVAL_S = 0.1


class PlannerPathRecorder(Node):
    def __init__(self):
        super().__init__("planner_path_recorder")

        self.declare_parameter("planner_name", "astar")
        self.declare_parameter("output_dir", os.path.expanduser("~/planner_results"))
        self.declare_parameter("scenario", "default")
        # use_sim_time is auto-declared by the Node base class

        self._planner = self.get_parameter("planner_name").get_parameter_value().string_value
        output_dir = os.path.expanduser(
            self.get_parameter("output_dir").get_parameter_value().string_value
        )
        self._scenario = self.get_parameter("scenario").get_parameter_value().string_value

        os.makedirs(output_dir, exist_ok=True)
        self._plan_csv = os.path.join(output_dir, "plan_paths.csv")
        self._traj_csv = os.path.join(output_dir, "robot_trajectories.csv")
        self._init_csv(self._plan_csv, PLAN_HEADER)
        self._init_csv(self._traj_csv, TRAJ_HEADER)

        self._state = State.IDLE
        self._run_id = ""
        self._t_nav_start: Time | None = None
        self._t_last_sample: Time | None = None
        self._sample_index = 0
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0

        self._seen_compute_ids: set[bytes] = set()
        self._prev_nav_statuses: set[int] = set()

        self.create_subscription(Odometry, "/odometry/local", self._odom_cb, 10)
        self.create_subscription(
            GoalStatusArray,
            "/compute_path_to_pose/_action/status",
            self._compute_status_cb,
            10,
        )
        self.create_subscription(Path, "/plan", self._plan_cb, 10)
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._nav_status_cb,
            10,
        )

        self.get_logger().info(
            f"Path recorder ready — planner={self._planner}, "
            f"scenario={self._scenario}\n"
            f"  plans  → {self._plan_csv}\n"
            f"  trajs  → {self._traj_csv}"
        )

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        self._current_yaw = self._quat_to_yaw(msg.pose.pose.orientation)

        if self._state != State.NAVIGATING:
            return

        now = self.get_clock().now()
        if self._t_last_sample is not None:
            dt = (now - self._t_last_sample).nanoseconds / 1e9
            if dt < TRAJ_SAMPLE_INTERVAL_S:
                return

        elapsed = (
            (now - self._t_nav_start).nanoseconds / 1e9
            if self._t_nav_start is not None
            else 0.0
        )
        self._write_traj_row(elapsed)
        self._t_last_sample = now
        self._sample_index += 1

    def _compute_status_cb(self, msg: GoalStatusArray):
        """Detect each new planning request and assign a fresh run_id."""
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

            self._run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            self._prev_nav_statuses = set()
            self._state = State.PLANNING
            self.get_logger().info(f"Planning detected — run_id={self._run_id}")

    def _plan_cb(self, msg: Path):
        if self._state != State.PLANNING:
            return
        if not msg.poses:
            return

        # Write every pose of the planned path
        with open(self._plan_csv, "a", newline="") as f:
            writer = csv.writer(f)
            for i, ps in enumerate(msg.poses):
                yaw = self._quat_to_yaw(ps.pose.orientation)
                writer.writerow([
                    self._run_id,
                    self._planner,
                    self._scenario,
                    i,
                    round(ps.pose.position.x, 4),
                    round(ps.pose.position.y, 4),
                    round(yaw, 5),
                ])

        self._t_nav_start = self.get_clock().now()
        self._t_last_sample = None
        self._sample_index = 0
        self._state = State.NAVIGATING
        self.get_logger().info(
            f"Plan saved — {len(msg.poses)} poses written to plan_paths.csv"
        )

    def _nav_status_cb(self, msg: GoalStatusArray):
        if self._state != State.NAVIGATING:
            return

        current_statuses = {s.status for s in msg.status_list}
        new_statuses = current_statuses - self._prev_nav_statuses
        self._prev_nav_statuses = current_statuses

        if GoalStatus.SUCCEEDED in new_statuses:
            self._finish_run(success=True)
        elif GoalStatus.ABORTED in new_statuses:
            self._finish_run(success=False)

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _quat_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _write_traj_row(self, elapsed_s: float):
        with open(self._traj_csv, "a", newline="") as f:
            csv.writer(f).writerow([
                self._run_id,
                self._planner,
                self._scenario,
                self._sample_index,
                round(elapsed_s, 3),
                round(self._current_x, 4),
                round(self._current_y, 4),
                round(self._current_yaw, 5),
            ])

    def _finish_run(self, success: bool):
        self._state = State.IDLE
        self.get_logger().info(
            f"Run {self._run_id} complete — success={success}, "
            f"trajectory samples={self._sample_index}"
        )

    @staticmethod
    def _init_csv(path: str, header: list):
        if not os.path.exists(path):
            with open(path, "w", newline="") as f:
                csv.writer(f).writerow(header)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerPathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
