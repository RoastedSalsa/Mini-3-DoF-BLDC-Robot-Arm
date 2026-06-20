#!/usr/bin/env python3
"""Interactive PID-tuning console for the Mini_Ranka arm.

Reads SimpleFOC Commander commands from stdin (one per line) and publishes each
to ``<ns>/cmd``; the running ``telemetry_bridge`` forwards them to the firmware
over serial. Run the bridge first, watch the response live in PlotJuggler, then
type commands here, e.g.:

    MMG0            print all current gains / limits
    MAP12           angle PID P = 12
    MVP0.3          velocity PID P = 0.3
    L-0.15 0.15     square-wave step levels [rad]
    P1.0            step half-period [s]
    T               toggle the continuous step

Commander text replies are echoed by the bridge on ``<ns>/log``
(``ros2 topic echo <ns>/log``). Quit with Ctrl-D / Ctrl-C.

Usage:
    ros2 run mini_ranka_bridge tune
    ros2 run mini_ranka_bridge tune --ros-args -p namespace:=mini_ranka
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TuneConsole(Node):
    def __init__(self) -> None:
        super().__init__("tune_console")
        ns = str(self.declare_parameter("namespace", "mini_ranka").value).strip("/")
        self.topic = f"/{ns}/cmd" if ns else "/cmd"
        self.pub = self.create_publisher(String, self.topic, 10)

    def send(self, text: str) -> None:
        self.pub.publish(String(data=text))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TuneConsole()
    node.get_logger().info(
        f"tuning console -> {node.topic}; type Commander cmds, Ctrl-D to quit"
    )
    try:
        for raw in sys.stdin:
            line = raw.strip()
            if line:
                node.send(line)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
