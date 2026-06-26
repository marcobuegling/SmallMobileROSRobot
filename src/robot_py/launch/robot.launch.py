from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = '/../config/robot_params.yaml'

    return LaunchDescription([
        Node(
            package='robot_py',
            executable='motor_controller',
            name='motor_controller',
            parameters=[params],
        ),
        Node(
            package='robot_py',
            executable='ultrasonic_sensor',
            name='ultrasonic_sensor',
            parameters=[params],
        ),
        Node(
            package='robot_py',
            executable='line_tracker',
            name='line_tracker',
            parameters=[params],
        ),
        Node(
            package='robot_py',
            executable='keyboard_controller',
            name='keyboard_controller',
            parameters=[params]
        )
    ])