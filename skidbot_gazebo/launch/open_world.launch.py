import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():

    package_description = 'skidbot_description'
    package_gazebo = 'skidbot_gazebo'

    pkg_gazebo = get_package_share_directory(package_gazebo)
    pkg_description = get_package_share_directory(package_description)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set resource paths so Gazebo can find meshes / models
    install_dir_path = get_package_prefix(package_description) + '/share'
    install_gazebo_dir_path = get_package_prefix(package_gazebo) + '/share'
    gazebo_resource_paths = [install_dir_path, install_gazebo_dir_path]

    for env_var in ['GZ_SIM_RESOURCE_PATH', 'GZ_SIM_MODEL_PATH', 'SDF_PATH']:
        if env_var in os.environ:
            for resource_path in gazebo_resource_paths:
                if resource_path not in os.environ[env_var]:
                    os.environ[env_var] += ':' + resource_path
        else:
            os.environ[env_var] = ':'.join(gazebo_resource_paths)

    robot_desc_path = os.path.join(pkg_description, 'urdf', 'skidbot.xacro')

    if not os.path.exists(robot_desc_path):
        raise FileNotFoundError(f'URDF file not found at: {robot_desc_path}')

    robot_description = ParameterValue(
        Command(['xacro ', robot_desc_path]),
        value_type=str
    )

    # Gazebo simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_gazebo,
            'worlds',
            'open_world.sdf'
        ])}.items(),
    )

    # Spawn arguments
    declare_spawn_x = DeclareLaunchArgument('x', default_value='3.3629',
                                            description='Model Spawn X Axis Value')
    declare_spawn_y = DeclareLaunchArgument('y', default_value='-1.1511',
                                            description='Model Spawn Y Axis Value')
    declare_spawn_z = DeclareLaunchArgument('z', default_value='1.5',
                                            description='Model Spawn Z Axis Value')
    declare_spawn_yaw = DeclareLaunchArgument('yaw', default_value='-2.155',
                                            description='Model Spawn yaw Value')
    # Spawn robot into Gazebo from robot_description topic
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='skidbot_spawn',
        arguments=[
            '-name', 'skidbot',
            '-allow_renaming', 'true',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-yaw',LaunchConfiguration('yaw')
        ],
        output='screen',
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
        ]
    )

    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_gazebo, 'config', 'skidbot_ros_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_gazebo, 'config', 'skidbot.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        gz_sim,
        gz_spawn_entity,
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        bridge,
        robot_state_publisher,
        rviz,
    ])
