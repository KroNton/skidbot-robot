from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ]
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'skidbot_controller',
            '--controller-manager',
            '/controller_manager'
        ]
    )

    twist_relay_node = Node(
        package='skidbot_controller',
        executable='twist_to_twist_stamped.py',
        name='twist_to_twist_stamped',
        output='screen',
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        twist_relay_node,
    ])
