from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = '/path/to/config/robot_params.yaml'

    return LaunchDescription([
        Node(
            package='my_robot',
            executable='motor_controller',
            name='motor_controller',
            parameters=[params],
        ),
        Node(
            package='my_robot',
            executable='ultrasonic_sensor',
            name='ultrasonic_sensor',
            parameters=[params],
        ),
        Node(
            package='my_robot',
            executable='line_tracker',
            name='line_tracker',
            parameters=[params],
        ),
    ])