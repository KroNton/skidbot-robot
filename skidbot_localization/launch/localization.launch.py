from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
from ament_index_python.packages import get_package_share_directory

# Paths to configuration files
nav2_yaml = os.path.join(get_package_share_directory('beetlebot_localization'), 'config', 'beetlebot_amcl.yaml')
map_file = os.path.join(get_package_share_directory('beetlebot_localization'), 'map', 'home_map.yaml')
rviz_config = os.path.join(get_package_share_directory('beetlebot_localization'), 'config', 'beetlebot_amcl.rviz')

def generate_launch_description():
    # Define the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Define the map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                   {'yaml_filename': map_file}]
    )

    # Define the AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml]
    )

    # Define the lifecycle manager node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )


    # Add everything to the LaunchDescription
    return LaunchDescription([
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        # rviz_node, 
    ])