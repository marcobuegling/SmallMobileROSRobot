import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from robot.hardware.sensors import BasicSensor

class LineTrackerNode(Node):
    def __init__(self):
        super().__init__('line_tracker')

        self.declare_parameter('enabled', False)
        self.declare_parameter('steering_step', 0.2)

        self._sensor = BasicSensor.from_config(...)
        self._steering_step = self.get_parameter('steering_step').value

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_line', 10)
        self._detected_pub = self.create_publisher(Bool, '/line_tracker/detected', 10)

        self.create_timer(0.05, self._timer_callback)

    def _timer_callback(self):
        detected = self._sensor.read_value()

        det_msg = Bool()
        det_msg.data = bool(detected)
        self._detected_pub.publish(det_msg)

        if self.get_parameter('enabled').value:
            twist = Twist()
            twist.linear.x = 0.5  # maintain forward speed
            # Detected = on line = steer left; not detected = steer right
            twist.angular.z = self._steering_step if detected else -self._steering_step
            self._cmd_pub.publish(twist)