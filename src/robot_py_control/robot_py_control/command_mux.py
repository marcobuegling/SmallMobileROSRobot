#!/usr/bin/env python3
"""Command velocity multiplexer node.

Subscribes to several ``geometry_msgs/Twist`` command sources (e.g. keyboard
teleop, line follower, target follower), selects exactly one of them
according to an explicit service request combined with a priority/timeout
fallback policy, and republishes the selection on ``/cmd_vel``. The
currently active source name is also published on a transient-local
("latched") topic so nodes that start later immediately learn the current
state without waiting for the next change.

Selection policy
-----------------
* A *requested* source is chosen either at startup (``default_source``
  parameter, or the highest-priority configured source if unset) or via the
  ``~/set_source`` service.
* At each control-loop tick, the requested source is used **if it has
  published a message within its configured ``timeout``**.
* If the requested source has gone stale (no message within its timeout),
  the node automatically fails over to the highest-``priority`` source
  that is currently alive, and reports that as the active source.
* If no source is alive, the node either publishes a zero ``Twist`` (stop
  command) or stops publishing altogether, depending on the
  ``stop_when_no_source`` parameter.
"""

from __future__ import annotations

import dataclasses
import functools
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.subscription import Subscription
from rclpy.time import Time
from std_msgs.msg import String

from robot_interfaces.srv import SetCommandSource


@dataclasses.dataclass
class CommandSource:
    """Book-keeping and configuration for a single candidate command source.

    Attributes:
        name: Logical name of the source, e.g. ``"keyboard"``. This is the
            identifier used by the ``SetCommandSource`` service.
        topic: Topic this source publishes ``Twist`` messages on.
        priority: Higher values win when more than one source is alive at
            the same time and no explicit selection overrides them.
        timeout: Seconds since the last received message after which the
            source is considered stale (no longer "alive").
        last_msg: Most recently received ``Twist`` message, if any.
        last_stamp: Node-clock time at which ``last_msg`` was received.
    """

    name: str
    topic: str
    priority: int
    timeout: float
    last_msg: Optional[Twist] = None
    last_stamp: Optional[Time] = None


class CommandMuxNode(Node):
    """ROS 2 node that multiplexes several Twist sources onto ``/cmd_vel``."""

    def __init__(self) -> None:
        """Initialize parameters, sources, publishers, subscribers and the service."""
        super().__init__(
            "command_mux",
            automatically_declare_parameters_from_overrides=True,
        )

        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("stop_when_no_source", True)
        self.declare_parameter("default_source", "")

        self._sources: Dict[str, CommandSource] = self._load_sources()
        if not self._sources:
            raise RuntimeError(
                "command_mux requires at least one entry under the "
                "'sources' parameter namespace (see config/command_mux.yaml)."
            )

        self._publish_rate: float = float(
            self.get_parameter("publish_rate").value
        )
        self._stop_when_no_source: bool = bool(
            self.get_parameter("stop_when_no_source").value
        )

        default_source = str(self.get_parameter("default_source").value)
        if default_source and default_source in self._sources:
            self._selected_source: str = default_source
        else:
            self._selected_source = max(
                self._sources.values(), key=lambda s: s.priority
            ).name

        self._active_source: Optional[str] = None

        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._state_pub = self.create_publisher(
            String, "/command_mux/active_source", latched_qos
        )

        self._subscriptions: List[Subscription] = []
        for source in self._sources.values():
            subscription = self.create_subscription(
                Twist,
                source.topic,
                functools.partial(self._on_source_msg, source.name),
                10,
            )
            self._subscriptions.append(subscription)

        self._service = self.create_service(
            SetCommandSource, "~/set_source", self._on_set_source
        )

        self._timer = self.create_timer(1.0 / self._publish_rate, self._on_timer)

        # Publish the initial state immediately so late/early joiners on the
        # transient-local topic see a sensible value right away.
        self._publish_state(self._selected_source, force=True)

        sources_summary = ", ".join(
            f"{s.name}(topic={s.topic}, priority={s.priority}, timeout={s.timeout}s)"
            for s in self._sources.values()
        )
        self.get_logger().info(
            f"command_mux started with sources: {sources_summary}. "
            f"Initial selection: '{self._selected_source}'."
        )

    def _load_sources(self) -> Dict[str, CommandSource]:
        """Parse the ``sources.*`` parameter group into `CommandSource` objects.

        Expects parameters declared like::

            sources.keyboard.topic: /cmd/keyboard
            sources.keyboard.priority: 100
            sources.keyboard.timeout: 0.5

        which is exactly what ``automatically_declare_parameters_from_overrides``
        produces when loading the accompanying YAML file.

        Returns:
            Mapping of source name to its parsed `CommandSource` config.

        Raises:
            RuntimeError: If a source is missing a required field
                (``topic``, ``priority`` or ``timeout``).
        """
        raw = self.get_parameters_by_prefix("sources")

        grouped: Dict[str, Dict[str, object]] = {}
        for dotted_key, parameter in raw.items():
            name, _, field = dotted_key.partition(".")
            grouped.setdefault(name, {})[field] = parameter.value

        sources: Dict[str, CommandSource] = {}
        for name, fields in grouped.items():
            try:
                sources[name] = CommandSource(
                    name=name,
                    topic=str(fields["topic"]),
                    priority=int(fields["priority"]),
                    timeout=float(fields["timeout"]),
                )
            except KeyError as exc:
                raise RuntimeError(
                    f"Source '{name}' is missing required field {exc}. "
                    "Each source needs 'topic', 'priority' and 'timeout'."
                ) from exc
        return sources

    def _on_source_msg(self, source_name: str, msg: Twist) -> None:
        """Record the latest ``Twist`` message received from a given source.

        Args:
            source_name: Name of the source that published this message.
            msg: The received command.
        """
        source = self._sources[source_name]
        source.last_msg = msg
        source.last_stamp = self.get_clock().now()

    def _on_set_source(
        self,
        request: SetCommandSource.Request,
        response: SetCommandSource.Response,
    ) -> SetCommandSource.Response:
        """Handle a `SetCommandSource` request to change the desired source.

        Args:
            request: Contains ``source_name``, the desired command source.
            response: Populated in-place with ``success`` and ``message``.

        Returns:
            The populated response.
        """
        name = request.source_name
        if name not in self._sources:
            response.success = False
            response.message = (
                f"Unknown source '{name}'. Valid sources: "
                f"{', '.join(sorted(self._sources))}"
            )
            self.get_logger().warning(response.message)
            return response

        self._selected_source = name
        response.success = True
        response.message = f"Command source set to '{name}'."
        self.get_logger().info(response.message)
        return response

    def _on_timer(self) -> None:
        """Resolve the currently active source and republish its command.

        Runs at ``publish_rate`` Hz. Applies the requested-source-with-
        priority-fallback policy described in the module docstring.
        """
        now = self.get_clock().now()
        alive = [s for s in self._sources.values() if self._is_alive(s, now)]

        active_name: Optional[str] = None
        selected = self._sources.get(self._selected_source)
        if selected is not None and self._is_alive(selected, now):
            active_name = selected.name
        elif alive:
            active_name = max(alive, key=lambda s: s.priority).name

        self._publish_state(active_name)

        if active_name is not None:
            last_msg = self._sources[active_name].last_msg
            if last_msg is not None:
                self._cmd_vel_pub.publish(last_msg)
        elif self._stop_when_no_source:
            self._cmd_vel_pub.publish(Twist())

    @staticmethod
    def _is_alive(source: CommandSource, now: Time) -> bool:
        """Check whether a source has published within its timeout window.

        Args:
            source: The source to check.
            now: Current node-clock time.

        Returns:
            ``True`` if the source has received at least one message and
            the most recent one is younger than its configured timeout.
        """
        if source.last_stamp is None:
            return False
        elapsed_seconds = (now - source.last_stamp).nanoseconds / 1e9
        return elapsed_seconds <= source.timeout

    def _publish_state(self, active_name: Optional[str], force: bool = False) -> None:
        """Publish the active source name on the transient-local state topic.

        Only publishes when the active source actually changed, unless
        ``force`` is set (used once at startup).

        Args:
            active_name: Name of the currently active source, or ``None``
                if no source is currently alive.
            force: Publish even if the value is unchanged.
        """
        if active_name == self._active_source and not force:
            return
        self._active_source = active_name

        msg = String()
        msg.data = active_name if active_name is not None else "none"
        self._state_pub.publish(msg)

        if not force:
            self.get_logger().info(f"Active command source changed to: '{msg.data}'.")


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for the ``command_mux`` executable.

    Args:
        args: Optional CLI arguments, forwarded to ``rclpy.init``.
    """
    rclpy.init(args=args)
    node = CommandMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
