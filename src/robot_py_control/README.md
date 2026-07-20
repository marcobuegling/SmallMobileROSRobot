# robot_py_control — command_mux

A ROS 2 Jazzy command-velocity multiplexer.

It subscribes to three `geometry_msgs/Twist` sources:

| source name       | default topic          | default priority | default timeout |
|--------------------|------------------------|-------------------|------------------|
| `keyboard`         | `/cmd/keyboard`         | 100               | 0.5 s            |
| `line_follower`    | `/cmd/line_follower`    | 50                | 0.5 s            |
| `target_follower`  | `/cmd/target_follower`  | 50                | 0.5 s            |

and republishes the currently active one on `/cmd_vel`.

## Packages

The `SetCommandSource` service is defined in `robot_interfaces`, since a pure 
`ament_python` package can't define services in ROS 2 Jazzy.

## How selection works

1. On startup, the node picks the `default_source` parameter if set,
   otherwise the source with the highest `priority`.
2. At every control tick (`publish_rate`, default 20 Hz):
   - If the requested source has published within its `timeout`, it is
     republished on `/cmd_vel`.
   - Otherwise, the node fails over to the highest-priority source that
     is currently alive (has published within its own `timeout`).
   - If nothing is alive, it publishes a zero `Twist` (or nothing, if
     `stop_when_no_source: false`).
3. Calling the `SetCommandSource` service changes the requested source.
   If that source later goes stale, the node still fails over
   automatically — the requested selection persists so it resumes as soon
   as the requested source starts publishing again.
4. The active source name is published on `/command_mux/active_source`
   (`std_msgs/String`) with QoS durability `TRANSIENT_LOCAL`, so any node
   that subscribes later immediately gets the current value without
   waiting for the next change.
