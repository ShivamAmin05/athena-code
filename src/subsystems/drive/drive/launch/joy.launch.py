from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('drive'), 'config')
    
    # Declare launch arguments
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')
    
    declare_joy_dev = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path')
    
    declare_config_filepath = DeclareLaunchArgument(
        'config_filepath',
        default_value=os.path.join(config_dir, 'joystick_config.yaml'),
        description='Configuration file path')
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )
    
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_filepath],
        remappings=[('/cmd_vel', '/robot/cmd_vel')]
    )
    
    return LaunchDescription([
        declare_joy_dev,
        declare_config_filepath,
        joy_node,
        teleop_twist_joy_node,
    ])