import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('skidbot_navigation')

    # ── Configuration file paths ──────────────────────────────────────────────
    planner_yaml      = os.path.join(pkg, 'config', 'planner_server.yaml')
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

    # Resolve BT XML file at launch time based on the planner argument
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
        remappings=[('/odom', '/beetlebot_diff_drive_controller/odom')],
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
        remappings=[('/odom', '/beetlebot_diff_drive_controller/odom')],
    )

    smoother_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[smoother_yaml, {'use_sim_time': use_sim_time}],
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
        lifecycle_manager_node,
    ])
