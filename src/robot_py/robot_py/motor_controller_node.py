import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from robot.control.four_wheel_car_control import FourWheelCarControl
from robot.utils.config import RobotConfig

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Declare parameters 
        self.declare_parameter('base_speed', 100.0)
        self.declare_parameter('speed_step', 0.1)
        self.declare_parameter('steering_step', 0.2)
        self.declare_parameter('pwm_frequency', 1000.0)
        # ... declare pin parameters too

        base_speed = self.get_parameter('base_speed').value

        cfg = self._load_config()
        self._control = FourWheelCarControl.from_config(
            cfg.motors,
            pwm_frequency=cfg.control.pwm_frequency,
            base_speed=base_speed,
        )

        # Subscribe to standard velocity commands
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        # Subscribe to emergency stop signals (published by ultrasonic node)
        self.create_subscription(Bool, '/emergency_stop', self._estop_callback, 10)

        self.get_logger().info('Motor controller node started')

    def _cmd_vel_callback(self, msg: Twist):
        speed = msg.linear.x
        steering = msg.angular.z

        self._control.set_speed(speed)
        self._control.set_steering(steering)

    def _estop_callback(self, msg: Bool):
        self._control.allow_forward = not msg.data

    def destroy_node(self):
        self._control.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()