import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('skidbot_navigation')

    # ── Configuration file paths ──────────────────────────────────────────────
    planner_yaml_astar        = os.path.join(pkg, 'config', 'planner_astar.yaml')
    planner_yaml_hybrid_astar = os.path.join(pkg, 'config', 'planner_hybrid_astar.yaml')
    controller_yaml   = os.path.join(pkg, 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(pkg, 'config', 'bt_navigator.yaml')
    recovery_yaml     = os.path.join(pkg, 'config', 'behavior_server.yaml')
    smoother_yaml     = os.path.join(pkg, 'config', 'smoother.yaml')

    # ── Behavior tree XML paths (one per planner) ─────────────────────────────
    bt_astar        = os.path.join(pkg, 'behavior_tree', 'simple_nav_w_replanning_astar.xml')
    bt_hybrid_astar = os.path.join(pkg, 'behavior_tree', 'simple_nav_w_replanning_hybrid_astar.xml')

    use_sim_time = True

    # ── Launch argument: choose active planner ────────────────────────────────
    declare_planner_arg = DeclareLaunchArgument(
        'planner',
        default_value='astar',
        description='Default path planner to use: "astar" (GridBased) or "hybrid_astar" (HybridAStar)',
        choices=['astar', 'hybrid_astar'],
    )
    planner_arg = LaunchConfiguration('planner')

    # Select both the planner config YAML and the BT XML together so they stay in sync.
    # "astar"        → planner_astar.yaml        (only GridBased loaded)  + astar BT
    # "hybrid_astar" → planner_hybrid_astar.yaml (only HybridAStar loaded) + hybrid BT
    planner_yaml = PythonExpression([
        '"', planner_yaml_astar, '" if "', planner_arg, '" == "astar" else "', planner_yaml_hybrid_astar, '"'
    ])
    bt_xml_file = PythonExpression([
        '"', bt_astar, '" if "', planner_arg, '" == "astar" else "', bt_hybrid_astar, '"'
    ])

    # ── Nodes ─────────────────────────────────────────────────────────────────

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml, {'use_sim_time': use_sim_time}],
        remappings=[('/odom', '/odometry/local')],
    )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml, {'use_sim_time': use_sim_time}],
    )

    nav2_behaviors_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[recovery_yaml, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    bt_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            bt_navigator_yaml,
            {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': bt_xml_file,
            },
        ],
        remappings=[('/odom', '/odometry/local')],
    )

    smoother_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[smoother_yaml, {'use_sim_time': use_sim_time}],
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[
            {'autostart': True},
            {'use_sim_time': use_sim_time},
            {'node_names': [
                'planner_server',
                'controller_server',
                'behavior_server',
                'smoother_server',
                'bt_navigator',
                'waypoint_follower',
            ]},
        ],
    )

    return LaunchDescription([
        declare_planner_arg,
        controller_node,
        planner_node,
        nav2_behaviors_node,
        bt_node,
        smoother_node,
        waypoint_follower_node,
        lifecycle_manager_node,
    ])
