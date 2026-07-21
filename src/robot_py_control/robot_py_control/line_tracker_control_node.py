#!/usr/bin/env python3
"""
ROS 2 node implementing single-sensor line-border following.

Subscribes to a std_msgs/Bool topic published by a line-tracking sensor
(e.g. basic_bool_sensor_node.py) and publishes geometry_msgs/Twist steering
commands to drive a mobile robot along the *edge* of a line.

Control strategy
-----------------
With only one boolean sensor, the robot cannot know how far it has drifted
from the line, only whether it is currently over the line or not. The
standard, minimal-complexity approach for this case is bang-bang
(on/off) edge following:

    - Sensor reads True  (over the line)  -> steer one fixed direction
    - Sensor reads False (off the line)   -> steer the other fixed direction

This makes the robot continuously "hunt" back and forth across the line
border, which on average keeps it tracking the edge. Forward speed is
constant while active. This is intentionally simple: no PID, no history
buffer, no debouncing beyond what the sensor node itself provides via its
'buffer_size' parameter. If more precise tracking is ever needed (e.g. to
reduce the wiggle), the natural upgrade path is a second sensor for a true
differential signal, not a heavier controller on top of one bit of input.

Only two Twist fields are used, per the robot's motion model:
    - linear.x  -> forward/backward speed   (-1.0 = full reverse, 1.0 = full forward)
    - angular.z -> steering                 (-1.0 = full right,   1.0 = full left)
All other Twist fields are left at zero.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


def _clamp(value: float, low: float = -1.0, high: float = 1.0) -> float:
    """Clamp a value to the inclusive range [low, high]."""
    return max(low, min(high, value))


class LineTrackerControlNode(Node):
    """Drives the robot along a line border using a single boolean sensor.

    Subscribes to a Bool topic (default '/line_tracker') and publishes a
    Twist to '/cmd/line_follower' on every received sensor reading, steering
    left or right depending on whether the line is currently detected.
    """

    def __init__(self) -> None:
        super().__init__("line_tracker_control_node")

        # --- Parameters ---
        # Kept intentionally minimal: just enough to retune behavior per
        # robot/surface without touching code.
        self.declare_parameter("sensor_topic", "/line_tracker")
        self.declare_parameter("cmd_topic", "/cmd/line_follower")
        self.declare_parameter("speed", 0.3)                # forward speed, -1..1
        self.declare_parameter("steering_intensity", 0.5)   # steering magnitude, 0..1
        self.declare_parameter("invert_steering", False)     # flip left/right mapping

        sensor_topic = self.get_parameter("sensor_topic").get_parameter_value().string_value
        cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value
        self._speed = _clamp(self.get_parameter("speed").get_parameter_value().double_value)
        self._steer_mag = abs(
            self.get_parameter("steering_intensity").get_parameter_value().double_value
        )
        self._steer_mag = _clamp(self._steer_mag, 0.0, 1.0)
        self._invert = self.get_parameter("invert_steering").get_parameter_value().bool_value

        # --- Publisher / Subscriber ---
        self._cmd_publisher = self.create_publisher(Twist, cmd_topic, 10)
        self._sensor_subscription = self.create_subscription(
            Bool, sensor_topic, self._sensor_callback, 10
        )

        self.get_logger().info(
            f"LineTrackerControlNode started: "
            f"listening on '{sensor_topic}', publishing to '{cmd_topic}' "
            f"(speed={self._speed}, steering_intensity={self._steer_mag}, "
            f"invert_steering={self._invert})"
        )

    def _sensor_callback(self, msg: Bool) -> None:
        """Compute and publish a steering command for one sensor reading.

        When the line is detected, steer toward one side; when it is not
        detected, steer toward the other. Which side is "toward" vs "away"
        depends on where the sensor is mounted relative to the line, so it
        is configurable via the 'invert_steering' parameter rather than
        hardcoded.
        """
        line_detected = msg.data

        # Base mapping: line detected -> steer left, not detected -> steer right.
        steer = self._steer_mag if line_detected else -self._steer_mag
        if self._invert:
            steer = -steer

        cmd = Twist()
        cmd.linear.x = self._speed
        cmd.angular.z = steer
        self._cmd_publisher.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineTrackerControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()