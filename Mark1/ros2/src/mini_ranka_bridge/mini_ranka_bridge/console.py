#!/usr/bin/env python3
"""Read-only console showing what the MCU said, and what we said to it.

The other half of ``tune``. That node takes stdin and publishes to ``<ns>/cmd``;
this one subscribes to the same topic *and* to ``<ns>/log``, so sent commands and
the firmware's replies land interleaved in one stream:

    12:04:31.220  > DG
    12:04:31.244  < dynamics terms: gravity=on inertia=off centrifugal=off ...

``ros2 topic echo <ns>/log`` shows the replies but not what provoked them, which
makes a tuning session very hard to read back. Hence the pairing.

``/rosout`` is folded in as a third stream so bridge-level events ("opened
/dev/ttyACM0", "new signal -> ...", the stale-telemetry warning) appear in the
same timeline as the serial traffic. Without it a disconnected arm looks
identical to an idle one: both are simply silence.

Usage:
    ros2 run mini_ranka_bridge feedback
    ros2 run mini_ranka_bridge feedback --ros-args -p namespace:=mini_ranka
    ros2 run mini_ranka_bridge feedback --ros-args -p color:=false
"""

import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_rosout_default
from rcl_interfaces.msg import Log
from std_msgs.msg import String

# Same palette as the flow panel and the workspace-6 accent: cool white and
# steel, one pale-cyan accent, and a red family reserved for problems. Colour
# here has to survive a nearly transparent terminal sitting over a moving 3D
# scene, which is why nothing is saturated and nothing is warm.
INK = "\033[38;2;234;244;248m"       # you — sent commands, header
INK_SOFT = "\033[38;2;159;182;194m"  # the arm — ordinary replies
ACCENT = "\033[38;2;127;212;232m"    # something changed state
WARN = "\033[38;2;255;157;143m"      # warnings — a soft tint of ALERT
ALERT = "\033[38;2;255;107;94m"      # errors, rejected commands, corrupt lines
DIM = "\033[38;2;97;112;123m"        # timestamps and routine chatter
BOLD = "\033[1m"
OFF = "\033[0m"

# Firmware replies that report a state change rather than just chatter. Worth
# picking out of the scroll: these are the lines you go looking for after
# typing D / F / C / O / T / G.
STATE_REPLIES = (
    "dynamics terms:",
    "dynamics feedforward:",
    "cartesian measurement:",
    "homed offsets:",
    "trajectory resumed",
    "manual target",
    "step:",
    "step levels:",
)

# Nodes whose INFO-level rosout is worth showing. Everything else is only
# surfaced at WARN and above, otherwise rviz2 and rcl drown the serial traffic.
OWN_NODES = (
    "telemetry_bridge",
    "joint_state_bridge",
    "mock_arm",
    "tune_console",
    "flow_monitor",
    "robot_state_publisher",
)

LEVEL_NAMES = {Log.DEBUG: "DBG", Log.INFO: "INF", Log.WARN: "WRN",
               Log.ERROR: "ERR", Log.FATAL: "FTL"}


class FeedbackConsole(Node):
    def __init__(self) -> None:
        super().__init__("feedback_console")

        ns = str(self.declare_parameter("namespace", "mini_ranka").value).strip("/")
        # Honour NO_COLOR and non-tty output; the parameter is the manual override.
        want_color = bool(self.declare_parameter("color", True).value)
        self.color = (want_color and sys.stdout.isatty()
                      and not os.environ.get("NO_COLOR"))
        self.show_rosout = bool(self.declare_parameter("rosout", True).value)

        self.cmd_topic = f"/{ns}/cmd" if ns else "/cmd"
        self.log_topic = f"/{ns}/log" if ns else "/log"

        self.create_subscription(String, self.cmd_topic, self._on_cmd, 10)
        self.create_subscription(String, self.log_topic, self._on_log, 10)
        if self.show_rosout:
            # /rosout is transient-local depth 1000; a default profile silently
            # fails to match it and the subscription just never fires.
            self.create_subscription(
                Log, "/rosout", self._on_rosout, qos_profile_rosout_default)

        self._banner()

    # ------------------------------------------------------------------
    def _c(self, color: str, text: str) -> str:
        return f"{color}{text}{OFF}" if self.color else text

    def _banner(self) -> None:
        rule = "-" * 60
        print(self._c(DIM, rule))
        print(f"{self._c(BOLD + INK, 'mini_ranka feedback')}   "
              f"{self._c(INK, '>')} {self._c(DIM, self.cmd_topic)}   "
              f"{self._c(INK_SOFT, '<')} {self._c(DIM, self.log_topic)}")
        print(self._c(DIM, rule), flush=True)

    def _emit(self, marker: str, marker_color: str, text: str,
              text_color: str = "") -> None:
        stamp = time.strftime("%H:%M:%S") + f".{int(time.time() % 1 * 1000):03d}"
        body = self._c(text_color, text) if text_color else text
        print(f"{self._c(DIM, stamp)}  {self._c(marker_color, marker)} {body}",
              flush=True)

    # ------------------------------------------------------------------
    def _on_cmd(self, msg: String) -> None:
        self._emit(">", INK, msg.data, INK)

    def _on_log(self, msg: String) -> None:
        line = msg.data
        lowered = line.lower()

        # doHome (main.cpp:211) prints "home_angle<q><sensor>" with no newline and
        # no separator, so the telemetry line that follows is glued onto it, fails
        # to parse as JSON, and arrives here instead of becoming topics. Call that
        # out rather than rendering it as a normal reply.
        if '{"' in line:
            head, _, tail = line.partition('{"')
            self._emit("!", ALERT, f"malformed: {head or '(empty)'}"
                                   f"  + telemetry line lost ({len(tail) + 2}B)",
                       ALERT)
            return

        if lowered.startswith("usage:"):
            self._emit("<", INK_SOFT, line, ALERT)
        elif any(k in lowered for k in STATE_REPLIES):
            self._emit("<", INK_SOFT, line, ACCENT)
        else:
            self._emit("<", INK_SOFT, line)

    def _on_rosout(self, msg: Log) -> None:
        name = msg.name.rsplit(".", 1)[-1]
        if msg.level < Log.WARN and name not in OWN_NODES:
            return
        if msg.level < Log.INFO:
            return

        color = ALERT if msg.level >= Log.ERROR else WARN \
            if msg.level >= Log.WARN else DIM
        level = LEVEL_NAMES.get(msg.level, "???")
        self._emit("*", DIM, f"{name} [{level}] {msg.msg}", color)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FeedbackConsole()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
