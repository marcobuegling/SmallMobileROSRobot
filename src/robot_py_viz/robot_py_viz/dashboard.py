#!/usr/bin/env python3
"""
dashboard.py

A ROS 2 Jazzy node for a small skid-steer mobile robot that subscribes to the
command topic and all optional sensor topics, then renders a clean, live
dashboard in the terminal.

Subscribed topics (all optional except /cmd_vel, which is expected on most
setups but is still treated as "may go quiet"):
    /cmd_vel                 (geometry_msgs/msg/Twist)
        Only linear.x (forward/backward speed) and angular.z (turning rate)
        are used, since this is a skid-steer robot and the rest of the
        Twist message is unused.
    /sensors/ultrasonic       (std_msgs/msg/Float32)
        Distance reading in centimeters.
    /sensors/line_tracker     (std_msgs/msg/Bool)
        True if a line is currently detected.
    /sensors/infrared         (std_msgs/msg/Bool)
        True if the IR sensor currently detects an obstacle/object.

Since not every sensor is always physically mounted, this node does not
assume any topic is guaranteed to publish. Each topic's row shows "NO DATA"
until at least one message arrives, and reverts to "STALE" if messages stop
arriving for longer than STALE_TIMEOUT_SEC (this usually means the sensor
was unplugged, its driver node died, or it was never mounted in the first
place).

Usage:
    ros2 run <your_package> robot_dashboard_node
    # or directly:
    python3 robot_dashboard_node.py
"""

import shutil
import time
from dataclasses import dataclass
from typing import Callable, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32

# Time after which a topic that has gone quiet is shown as "STALE" rather
# than displaying its last known value as if it were still current.
STALE_TIMEOUT_SEC = 2.0

# How often the dashboard redraws itself, independent of message arrival.
REFRESH_RATE_HZ = 10.0

# ANSI escape codes used to redraw the dashboard in place instead of
# spamming new lines to the terminal on every refresh.
_ANSI_CLEAR_SCREEN = "\033[2J"
_ANSI_CURSOR_HOME = "\033[H"
_ANSI_HIDE_CURSOR = "\033[?25l"
_ANSI_SHOW_CURSOR = "\033[?25h"


@dataclass
class TopicState:
    """
    Tracks the latest known value of a single topic and when it last arrived.

    Attributes:
        display_name: Human-readable label shown in the dashboard.
        formatter: Function that turns the stored raw value into a display
            string. Only called once a value has actually been received.
        value: The most recently received raw value (message-dependent
            type), or None if nothing has ever arrived.
        last_received: Monotonic timestamp (seconds) of the last message,
            or None if nothing has ever arrived.
    """

    display_name: str
    formatter: Callable[[object], str]
    value: Optional[object] = None
    last_received: Optional[float] = None

    def update(self, value: object) -> None:
        """Record a newly received value and stamp it with the current time."""
        self.value = value
        self.last_received = time.monotonic()

    def status_text(self) -> str:
        """
        Return the string to display for this topic right now.

        Returns:
            "NO DATA" if nothing has ever been received, "STALE (<n>s ago)"
            if the last message is older than STALE_TIMEOUT_SEC, or the
            formatted current value otherwise.
        """
        if self.last_received is None:
            return "NO DATA"

        age = time.monotonic() - self.last_received
        if age > STALE_TIMEOUT_SEC:
            return f"STALE ({age:.1f}s ago)"

        return self.formatter(self.value)


class RobotDashboardNode(Node):
    """
    Subscribes to the robot's command and sensor topics and periodically
    redraws a terminal dashboard summarizing their current state.
    """

    def __init__(self) -> None:
        """Set up subscriptions, internal state tracking, and the redraw timer."""
        super().__init__("robot_dashboard_node")

        # One TopicState per topic we care about. Using a dict keyed by
        # topic name keeps the subscription callbacks and the render loop
        # in sync without duplicating topic names as string literals.
        self._states: dict[str, TopicState] = {
            "/cmd_vel": TopicState(
                display_name="Cmd Vel",
                formatter=self._format_cmd_vel,
            ),
            "/sensors/ultrasonic": TopicState(
                display_name="Ultrasonic",
                formatter=lambda v: f"{v:.1f} cm",
            ),
            "/sensors/line_tracker": TopicState(
                display_name="Line Tracker",
                formatter=lambda v: "LINE DETECTED" if v else "no line",
            ),
            "/sensors/infrared": TopicState(
                display_name="Infrared",
                formatter=lambda v: "OBJECT DETECTED" if v else "clear",
            ),
        }

        # Sensor data on a small robot is typically fine to receive
        # best-effort; this also lets the node interoperate with sensor
        # drivers that publish with a sensor-data QoS profile.
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10
        )
        self.create_subscription(
            Float32, "/sensors/ultrasonic", self._on_ultrasonic, sensor_qos
        )
        self.create_subscription(
            Bool, "/sensors/line_tracker", self._on_line_tracker, sensor_qos
        )
        self.create_subscription(
            Bool, "/sensors/infrared", self._on_infrared, sensor_qos
        )

        # Redraw the dashboard on a fixed timer rather than only on message
        # arrival, so that stale/no-data topics are still updated on screen
        # even when nothing is being published.
        self.create_timer(1.0 / REFRESH_RATE_HZ, self._render)

        print(_ANSI_HIDE_CURSOR, end="")

    # ------------------------------------------------------------------ #
    # Subscription callbacks
    # ------------------------------------------------------------------ #

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Store the latest cmd_vel message (only linear.x/angular.z matter)."""
        self._states["/cmd_vel"].update((msg.linear.x, msg.angular.z))

    def _on_ultrasonic(self, msg: Float32) -> None:
        """Store the latest ultrasonic distance reading."""
        self._states["/sensors/ultrasonic"].update(msg.data)

    def _on_line_tracker(self, msg: Bool) -> None:
        """Store the latest line-tracker boolean state."""
        self._states["/sensors/line_tracker"].update(msg.data)

    def _on_infrared(self, msg: Bool) -> None:
        """Store the latest infrared boolean state."""
        self._states["/sensors/infrared"].update(msg.data)

    # ------------------------------------------------------------------ #
    # Formatting helpers
    # ------------------------------------------------------------------ #

    @staticmethod
    def _format_cmd_vel(value: tuple) -> str:
        """
        Format a (linear_x, angular_z) tuple for display.

        Only these two components are shown since the robot is skid-steer
        and the remaining four Twist fields are unused.
        """
        linear_x, angular_z = value
        return f"lin.x={linear_x:+.2f} m/s  ang.z={angular_z:+.2f} rad/s"

    # ------------------------------------------------------------------ #
    # Rendering
    # ------------------------------------------------------------------ #

    def _render(self) -> None:
        """Redraw the full dashboard in the terminal."""
        width = max(shutil.get_terminal_size((80, 20)).columns, 50)
        line = "-" * width

        rows = []
        rows.append(line)
        rows.append(" ROBOT DASHBOARD".ljust(width))
        rows.append(line)

        for state in self._states.values():
            label = f" {state.display_name:<14}"
            rows.append(f"{label}: {state.status_text()}".ljust(width))

        rows.append(line)
        rows.append(" Ctrl+C to exit".ljust(width))

        # Clear screen and redraw from the top so the dashboard updates in
        # place instead of scrolling.
        print(_ANSI_CLEAR_SCREEN + _ANSI_CURSOR_HOME + "\n".join(rows), end="\r")

    def destroy_node(self) -> bool:
        """Restore the terminal cursor before shutting down."""
        print(_ANSI_SHOW_CURSOR, end="")
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    """Initialize rclpy, spin the dashboard node, and clean up on exit."""
    rclpy.init(args=args)
    node = RobotDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()