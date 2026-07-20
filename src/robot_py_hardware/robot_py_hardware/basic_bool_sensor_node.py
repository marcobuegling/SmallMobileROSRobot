#!/usr/bin/env python3
"""
ROS 2 node wrapping BasicSensor (simple high/low signal sensor, e.g. line tracker or PIR).

Publishes std_msgs/Bool at a configurable rate to a specified topic.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from robot.hardware.sensors import BasicSensor


class BasicBoolSensorNode(Node):
    """Publishes the boolean reading of a BasicSensor at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("basic_bool_sensor_node")

        # --- Parameters ---
        self.declare_parameter("signal_pin", 0)
        self.declare_parameter("buffer_size", 0)
        self.declare_parameter("frequency", 10.0)  # Hz
        self.declare_parameter("topic", "/basic_bool_sensor")

        signal_pin = self.get_parameter("signal_pin").get_parameter_value().integer_value
        buffer_size = self.get_parameter("buffer_size").get_parameter_value().integer_value
        frequency = self.get_parameter("frequency").get_parameter_value().double_value
        topic = self.get_parameter("topic").get_parameter_value().string_value

        if signal_pin <= 0:
            raise ValueError("Parameter 'signal_pin' must be set to a valid GPIO pin number")
        if frequency <= 0.0:
            raise ValueError("Parameter 'frequency' must be greater than 0")

        # --- Sensor ---
        self._sensor = BasicSensor(signal_pin, buffer_size)

        # --- Publisher ---
        self._publisher = self.create_publisher(Bool, topic, 10)

        # --- Timer ---
        period = 1.0 / frequency
        self._timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f"BasicBoolSensorNode started on pin {signal_pin}, "
            f"publishing to '{topic}' at {frequency} Hz"
        )

    def _timer_callback(self) -> None:
        detected = self._sensor.read_value()
        msg = Bool()
        msg.data = detected
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BasicBoolSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()