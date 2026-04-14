#!/usr/bin/env python3
"""
Planner benchmarking script for skidbot_navigation.

Sends ComputePathToPose goals to the Nav2 planner server without moving the
robot, records metrics for each planner, and prints a comparison table.

Prerequisites:
  - Nav2 stack must be running (ros2 launch skidbot_navigation nav2.launch.py)
  - Robot must be localized (AMCL active with an initial pose set)

Usage:
  ros2 run skidbot_navigation planner_benchmark.py \\
      --goal_x 3.0 --goal_y 2.0 --goal_yaw 0.0 \\
      --planners GridBased HybridAStar \\
      --trials 5 \\
      --csv /tmp/benchmark.csv

Metrics collected per trial:
  planning_time_s  : time reported by the planner server (builtin_interfaces/Duration)
  wall_time_s      : wall-clock round-trip time for the action call
  path_length_m    : Euclidean arc length of the returned path
  path_points      : number of poses in the path
  mean_curvature   : mean absolute curvature (rad/m) — lower = smoother path
"""

import argparse
import csv
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


# ── Geometry helpers ──────────────────────────────────────────────────────────

def make_pose(x: float, y: float, yaw: float, frame: str = 'map') -> PoseStamped:
    """Build a PoseStamped from (x, y, yaw) in the given frame."""
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def path_length(path) -> float:
    """Euclidean arc length (m) of a nav_msgs/Path."""
    poses = path.poses
    if len(poses) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(poses)):
        dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
        dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
        total += math.hypot(dx, dy)
    return total


def mean_curvature(path) -> float:
    """
    Mean absolute curvature (rad/m) via the inscribed-circle method.

    For three consecutive points A, B, C the curvature at B is:
        k = 4 * area(ABC) / (|AB| * |BC| * |AC|)

    Lower values indicate smoother paths.
    Returns 0.0 for paths with fewer than 3 points.
    """
    poses = path.poses
    if len(poses) < 3:
        return 0.0
    curvatures = []
    for i in range(1, len(poses) - 1):
        ax = poses[i - 1].pose.position.x
        ay = poses[i - 1].pose.position.y
        bx = poses[i].pose.position.x
        by = poses[i].pose.position.y
        cx = poses[i + 1].pose.position.x
        cy = poses[i + 1].pose.position.y
        ab = math.hypot(bx - ax, by - ay)
        bc = math.hypot(cx - bx, cy - by)
        ac = math.hypot(cx - ax, cy - ay)
        area = abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay)) / 2.0
        denom = ab * bc * ac
        curvatures.append(4.0 * area / denom if denom > 1e-9 else 0.0)
    return sum(curvatures) / len(curvatures) if curvatures else 0.0


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class PlannerBenchmark(Node):
    def __init__(self):
        super().__init__('planner_benchmark')
        self._client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

    def wait_for_server(self, timeout: float = 15.0) -> bool:
        self.get_logger().info('Waiting for compute_path_to_pose action server...')
        return self._client.wait_for_server(timeout_sec=timeout)

    def request_path(self, goal_pose: PoseStamped, planner_id: str):
        """
        Request one path plan synchronously.

        Returns (result, wall_time_s) where result is None on failure.
        """
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal_pose
        goal_msg.planner_id = planner_id
        goal_msg.use_start = False  # use current robot TF pose as start

        t0 = time.monotonic()

        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f'[{planner_id}] Goal rejected by server.')
            return None, time.monotonic() - t0

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wall_elapsed = time.monotonic() - t0

        if result_future.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'[{planner_id}] Planning failed (status={result_future.result().status}).')
            return None, wall_elapsed

        return result_future.result().result, wall_elapsed


# ── Benchmark logic ───────────────────────────────────────────────────────────

def run_benchmark(node: PlannerBenchmark, planners: list,
                  goal_pose: PoseStamped, trials: int) -> list:
    """Run each planner for `trials` iterations. Returns a list of metric dicts."""
    rows = []
    for planner_id in planners:
        node.get_logger().info(
            f'\n{"─"*60}\n  Planner: {planner_id}  ({trials} trials)\n{"─"*60}')
        for trial in range(1, trials + 1):
            result, wall = node.request_path(goal_pose, planner_id)
            if result is None:
                rows.append({
                    'planner_id': planner_id,
                    'trial': trial,
                    'success': False,
                    'planning_time_s': float('nan'),
                    'wall_time_s': wall,
                    'path_length_m': float('nan'),
                    'path_points': 0,
                    'mean_curvature': float('nan'),
                })
                continue

            pt = result.planning_time.sec + result.planning_time.nanosec * 1e-9
            pl = path_length(result.path)
            pp = len(result.path.poses)
            mc = mean_curvature(result.path)

            node.get_logger().info(
                f'  Trial {trial:2d}: '
                f'plan={pt:.4f}s  wall={wall:.4f}s  '
                f'len={pl:.3f}m  pts={pp}  curv={mc:.4f} rad/m')

            rows.append({
                'planner_id': planner_id,
                'trial': trial,
                'success': True,
                'planning_time_s': pt,
                'wall_time_s': wall,
                'path_length_m': pl,
                'path_points': pp,
                'mean_curvature': mc,
            })
    return rows


# ── Output helpers ────────────────────────────────────────────────────────────

METRICS = ['planning_time_s', 'wall_time_s', 'path_length_m',
           'path_points', 'mean_curvature']

METRIC_LABELS = {
    'planning_time_s': 'Plan time (s)',
    'wall_time_s':     'Wall time (s)',
    'path_length_m':   'Path len (m)',
    'path_points':     'Path pts',
    'mean_curvature':  'Curvature (rad/m)',
}


def print_summary(rows: list, planners: list) -> None:
    col_w = 19
    print(f'\n{"═"*80}')
    print('  PLANNER COMPARISON SUMMARY')
    print(f'{"═"*80}')

    header = f'  {"Planner":<18} {"Trials":>6} {"Success":>8}'
    for m in METRICS:
        header += f'  {METRIC_LABELS[m]:>{col_w}}'
    print(header)
    print(f'  {"─"*78}')

    for pid in planners:
        all_rows = [r for r in rows if r['planner_id'] == pid]
        ok_rows  = [r for r in all_rows if r['success']]
        total    = len(all_rows)
        n        = len(ok_rows)
        line     = f'  {pid:<18} {total:>6} {n:>8}'
        for m in METRICS:
            vals = [r[m] for r in ok_rows]
            avg  = sum(vals) / len(vals) if vals else float('nan')
            line += f'  {avg:>{col_w}.4f}'
        print(line)

    print(f'{"═"*80}\n')


def write_csv(rows: list, path: str) -> None:
    if not rows:
        return
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f'Results written to: {path}')


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Benchmark Nav2 path planners for skidbot (no robot motion).')
    parser.add_argument('--goal_x', type=float, default=3.0,
                        help='Goal X in map frame (default: 3.0)')
    parser.add_argument('--goal_y', type=float, default=2.0,
                        help='Goal Y in map frame (default: 2.0)')
    parser.add_argument('--goal_yaw', type=float, default=0.0,
                        help='Goal yaw in radians (default: 0.0)')
    parser.add_argument('--trials', type=int, default=5,
                        help='Number of trials per planner (default: 5)')
    parser.add_argument('--planners', nargs='+',
                        default=['GridBased', 'HybridAStar'],
                        help='Planner IDs to test (default: GridBased HybridAStar)')
    parser.add_argument('--csv', type=str, default='',
                        help='Optional CSV output file path')

    # Strip ROS2 remapping arguments before argparse sees them
    clean_args = [a for a in sys.argv[1:] if not a.startswith('__')]
    args = parser.parse_args(clean_args)

    rclpy.init()
    node = PlannerBenchmark()

    if not node.wait_for_server(timeout=15.0):
        node.get_logger().error(
            'Action server not available. Is Nav2 running?')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    goal_pose = make_pose(args.goal_x, args.goal_y, args.goal_yaw)
    goal_pose.header.stamp = node.get_clock().now().to_msg()

    node.get_logger().info(
        f'Benchmarking planners {args.planners} | '
        f'goal=({args.goal_x}, {args.goal_y}, yaw={args.goal_yaw}) | '
        f'{args.trials} trials each')

    rows = run_benchmark(node, args.planners, goal_pose, args.trials)
    print_summary(rows, args.planners)

    if args.csv:
        write_csv(rows, args.csv)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
