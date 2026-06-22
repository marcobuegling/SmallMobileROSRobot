import rclpy
from rclpy.node import Node
import pynput

class KeyboardControlNode(Node):
    """
    ROS node used for handling user input for mobile robots.
    Control using arrow keys: publishes on topic 'car_movement'
    Further actions: 'q': quit, 's': stop motors, 'd': start motors, 
    'l': toggle line tracking mode (deactivates user controls)
    """
    def __init__(self):
        
