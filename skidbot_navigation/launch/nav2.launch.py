from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Paths to configuration files
    planner_yaml = os.path.join(get_package_share_directory('beetlebot_navigation'), 'config', 'planner_server.yaml')
    controller_yaml = os.path.join(get_package_share_directory('beetlebot_navigation'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('beetlebot_navigation'), 'config', 'bt_navigator.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('beetlebot_navigation'), 'config', 'behavior_server.yaml')
    smoother_yaml = os.path.join(get_package_share_directory('beetlebot_navigation'), 'config', 'smoother.yaml')
    use_sim_time=True
    # Controller Server Node
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_yaml,{"use_sim_time": use_sim_time}],
        remappings=[('/odom','/beetlebot_diff_drive_controller/odom')])

    # Planner Server Node
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_yaml,{"use_sim_time": use_sim_time},]
        
        )
        
    # Behavior Server Node
    nav2_behaviors_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
       
        parameters=[recovery_yaml, {"use_sim_time": use_sim_time}],
        output='screen')

    # Behavior Tree Navigator Node
    bt_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_navigator_yaml,{"use_sim_time": use_sim_time}],
        remappings=[('/odom','/beetlebot_diff_drive_controller/odom')])

    # FIX 2: Added the Smoother Server Node
    smoother_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[smoother_yaml,{"use_sim_time": use_sim_time}])

    # Lifecycle Manager Node
    lifecycle_mange_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_pathplanner',
        output='screen',
        parameters=[{'autostart': True},{"use_sim_time": use_sim_time},
                    {'node_names': ['planner_server',
                                    'controller_server',
                                    'behavior_server',
                                    'smoother_server',
                                    'bt_navigator']}])
    


    return LaunchDescription([   
        controller_node,
        planner_node,
        nav2_behaviors_node,
        bt_node,
        smoother_node, # Added smoother_node to the launch list
        lifecycle_mange_node,
    ])