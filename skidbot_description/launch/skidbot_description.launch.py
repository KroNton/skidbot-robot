import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.substitutions import Command
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():

    package_description = 'skidbot_description'
    pkg_description = get_package_share_directory(package_description)

    install_dir_path = get_package_prefix(package_description) + '/share'
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        if install_dir_path not in os.environ['GZ_SIM_RESOURCE_PATH']:
            os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + install_dir_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = install_dir_path

    robot_desc_path = os.path.join(pkg_description, 'urdf', 'skidbot.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', robot_desc_path]),
        value_type=str
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_description, 'config', 'skidbot.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        robot_state_publisher,
        rviz
    ])
