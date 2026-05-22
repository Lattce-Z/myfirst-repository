from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_cpp',
            executable='unitree',
            name='unitree',
            output='screen'
        ),
        Node(
            package='ltyo_cpp',
            executable='ltyo',
            name='ltyo',
            output='screen'
        )
    ])