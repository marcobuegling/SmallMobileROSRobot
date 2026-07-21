#!/usr/bin/env python3
"""
ROS 2 node implementing a safety limiter for a small mobile robot 
based on ultrasonic distance measurement to avoid collisions with 
objects in front of the robot.

Subscribes to a geometry_msgs/Twist topic and republishes filtered 
messages of the same type on another topic. In case of an obstacle, 
the node limits forward movement, still allowing turning and 
backward movement. If no obstacle is detected, the messages are 
republished without any change.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class SafetyLimiterNode(Node):
    """
    Introduces a safety limiter to the command pipeline to perform 
    a safety stop in case an obstacle in front of the robot is 
    detected by an ultrasonic sensor
    """
    def __init__(self) -> None:
        super().__init__("safety_limiter_node")

        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("us_sensor_topic", "ultrasonic_sensor/range")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("stopping_distance", 0.15) # distance to obstacle in m at which a safety stop should be performed

        input_topic = self.get_parameter("input_topic")
        us_sensor_topic = self.get_parameter("us_sensor_topic")
        output_topic = self.get_parameter("output_topic")
        self._stopping_distance = self.get_parameter("stopping_distance")

        # --- Internal state ---
        self._obstacle_detected = False
        self._last_us_range = None # last ultrasonic measurement received in m

        # --- Subscribers / Publisher ---
        self._cmd_subscription = self.create_subscription(
            Twist, input_topic, self._cmd_input_callback, 10
        )
        self._us_sensor_subscription = self.create_subscription(
            Range, us_sensor_topic, self._us_sensor_callback, 10
        )
        self._cmd_publisher = self.create_publisher(
            Twist, output_topic, 10
        )

        # --- Logging ---
        self.get_logger().info(
            f"Safety limiter node started: subscribing to '{input_topic}' "
            f"and '{us_sensor_topic}', publishing filtered commands to "
            f"'{output_topic}'. Stopping distance: {self._stopping_distance:.1f} m."
        )

    def _us_sensor_callback(self, msg: Range) -> None:
        """
        Receives the incoming ultrasonic sensor measurement and calculates 
        whether a safety stop needs to be performed. Immediately publishes 
        a stop command when a new obstacle is detected to prevent robot 
        from driving further if no new command is received.
        """
        self._last_range_m = msg.range
        was_obstacle_detected = self._obstacle_detected
        self._obstacle_detected = msg.range <= self._stopping_distance
 
        if self._obstacle_detected and not was_obstacle_detected:

            # Send stop command
            safety_stop_cmd = Twist()
            safety_stop_cmd.linear.x = 0.0
            safety_stop_cmd.linear.y = 0.0
            safety_stop_cmd.linear.z = 0.0
            safety_stop_cmd.angular.x = 0.0
            safety_stop_cmd.angular.y = 0.0
            safety_stop_cmd.angular.z = 0.0
            self._cmd_publisher.publish()

            self.get_logger().warn(
                f"Obstacle detected at {msg.range * 100.0:.1f} cm "
                f"(threshold: {self._stopping_distance * 100.0:.1f} cm). "
                "Performing safety stop. Limiting forward movement."
            )
        elif was_obstacle_detected and not self._obstacle_detected:
            self.get_logger().info(
                f"Obstacle cleared (range: {msg.range * 100.0:.1f} cm). "
                "Forward movement re-enabled."
            )



    def _cmd_input_callback(self, msg: Twist) -> None:
        """
        Filters incoming velocity commands: if an obstacle is currently
        detected in front of the robot, forward motion (positive linear.x)
        is clamped to zero. Turning (angular.z) and backward motion
        (negative or zero linear.x) are always passed through unchanged.
        """
        filtered_msg = Twist()
        filtered_msg.linear.x = 0.0 if self._obstacle_detected else msg.linear.x
        filtered_msg.linear.y = msg.linear.y
        filtered_msg.linear.z = msg.linear.z
        filtered_msg.angular.x = msg.angular.x
        filtered_msg.angular.y = msg.angular.y
        filtered_msg.angular.z = msg.angular.z
 
        self._cmd_publisher.publish(filtered_msg)



def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyLimiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
