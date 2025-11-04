from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rmd_thrust_control',
            executable='tm_node',
            name='tm_node',
            output='screen'
        ),
        Node(
            package='rmd_thrust_control',
            executable='tm_control_node',
            name='tm_control_node',
            output='screen'
        ),
        Node(
            package='rmd_thrust_control',
            executable='rover_node',
            name='rover_node',
            output='screen'
        ),
        Node(
            package='rmd_thrust_control',
            executable='motor_node',
            name='m1',
            output='screen',
            parameters=[{'id':'141'}]
        ),
        Node(
            package='rmd_thrust_control',
            executable='motor_node',
            name='m2',
            output='screen',
            parameters=[{'id':'143'}]
        ),
        Node(
            package='rmd_thrust_control',
            executable='motor_node',
            name='m3',
            output='screen',
            parameters=[{'id':'14A'}]
        ),
        Node(
            package='rmd_thrust_control',
            executable='motor_node',
            name='m4',
            output='screen',
            parameters=[{'id':'149'}]
        )
    ])
