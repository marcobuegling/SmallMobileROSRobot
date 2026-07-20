#!/usr/bin/env python3
"""
ROS 2 node wrapping UltrasonicSensor (e.g. HC-SR04 or similar).

Publishes sensor_msgs/Range at a configurable rate. Range is the standard
ROS message type for single-beam range finders (ultrasonic, IR) and carries
min/max range and field of view alongside the measurement, which lets
downstream nodes (e.g. costmap layers, obstacle avoidance) interpret the
reading correctly.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

from robot.hardware.sensors import UltrasonicSensor


class UltrasonicSensorNode(Node):
    """Publishes the distance reading of an UltrasonicSensor at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("ultrasonic_sensor_node")

        # --- Parameters ---
        self.declare_parameter("trig_pin", 0)
        self.declare_parameter("echo_pin", 0)
        self.declare_parameter("buffer_size", 0)
        self.declare_parameter("frequency", 10.0)  # Hz
        self.declare_parameter("topic", "ultrasonic_sensor/range")
        self.declare_parameter("frame_id", "ultrasonic_sensor_link")
        # HC-SR04 typical specs, override via parameters if your module differs
        self.declare_parameter("field_of_view", 0.26)  # rad, ~15 degrees
        self.declare_parameter("min_range", 0.02)  # m
        self.declare_parameter("max_range", 4.0)  # m

        trig_pin = self.get_parameter("trig_pin").get_parameter_value().integer_value
        echo_pin = self.get_parameter("echo_pin").get_parameter_value().integer_value
        buffer_size = self.get_parameter("buffer_size").get_parameter_value().integer_value
        frequency = self.get_parameter("frequency").get_parameter_value().double_value
        topic = self.get_parameter("topic").get_parameter_value().string_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._field_of_view = self.get_parameter("field_of_view").get_parameter_value().double_value
        self._min_range = self.get_parameter("min_range").get_parameter_value().double_value
        self._max_range = self.get_parameter("max_range").get_parameter_value().double_value

        if trig_pin <= 0 or echo_pin <= 0:
            raise ValueError("Parameters 'trig_pin' and 'echo_pin' must be set to valid GPIO pin numbers")
        if frequency <= 0.0:
            raise ValueError("Parameter 'frequency' must be greater than 0")

        # --- Sensor ---
        self._sensor = UltrasonicSensor(trig_pin, echo_pin, buffer_size)

        # --- Publisher ---
        self._publisher = self.create_publisher(Range, topic, 10)

        # --- Timer ---
        period = 1.0 / frequency
        self._timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f"UltrasonicSensorNode started (trig={trig_pin}, echo={echo_pin}), "
            f"publishing to '{topic}' at {frequency} Hz"
        )

    def _timer_callback(self) -> None:
        distance_cm = self._sensor.read_value()
        distance_m = distance_cm / 100.0

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self._field_of_view
        msg.min_range = self._min_range
        msg.max_range = self._max_range
        msg.range = distance_m

        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()