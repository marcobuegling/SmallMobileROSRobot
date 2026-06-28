import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # load robot parameters from config file
    params = os.path.join(
        get_package_share_directory('robot_py'),
        'config',
        'robot_params.yaml'
    )

    # launch nodes with parameters
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