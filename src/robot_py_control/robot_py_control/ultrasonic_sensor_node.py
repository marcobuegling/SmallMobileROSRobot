import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from robot.hardware.sensors import UltrasonicSensor

class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor')

        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('emergency_stop_distance', 15.0)  # cm

        self._sensor = UltrasonicSensor.from_config(...)  # pass config
        self._stop_distance = self.get_parameter('emergency_stop_distance').value

        self._range_pub = self.create_publisher(Range, '/ultrasonic/range', 10)
        self._estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate, self._timer_callback)

    def _timer_callback(self):
        distance = self._sensor.read_value()
        avg = self._sensor.get_buffer_avg()

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_front'
        msg.radiation_type = Range.ULTRASOUND
        msg.range = float(distance) / 100.0
        msg.min_range = 0.02
        msg.max_range = 4.0
        self._range_pub.publish(msg)

        # Emergency stop logic
        estop = Bool()
        estop.data = (avg / 100.0) < self._stop_distance / 100.0
        self._estop_pub.publish(estop)