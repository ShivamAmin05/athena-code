from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'jank_drive'

    # Create nodes
    joystick_node = Node(
        package=pkg_name,
        executable='jank_joystick',
        name='jank_joystick_node',
        output='screen',
    )

    jank_drive_node = Node(
        package=pkg_name,
        executable='jank_drive',
        name='jank_drive_node',
        output='screen',
    )

    return LaunchDescription([
        joystick_node,
        jank_drive_node
    ])
