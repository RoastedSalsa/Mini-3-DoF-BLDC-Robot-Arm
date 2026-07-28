#!/usr/bin/env python3
"""Turn the arm's telemetry into /joint_states, so RViz mirrors the real arm.

`telemetry_bridge` deliberately knows nothing about the arm's structure — it
fans every JSON key out to its own Float64 topic. That is right for plotting,
but robot_state_publisher needs one sensor_msgs/JointState carrying the URDF's
joint names. This node is that adapter, and it is the only piece that has to
know both vocabularies.

Two input modes, chosen automatically:

  q1/q2/q3   (preferred) IK-frame joint angles straight from the firmware. Use
             this. The firmware knows its own live home offsets, including any
             captured by the 'O' homing command since boot.

  meas1..3   (fallback) raw encoder angles, converted here using JOINT_DIR /
             JOINT_GEAR and the home offsets given as parameters. Works against
             firmware that predates the q* channels, but the offsets are a copy
             of config.h: re-home the arm and this node is silently wrong until
             you update `home_offsets` to match.

The node starts in fallback and upgrades itself the first time a q* message
arrives, so it does the right thing either way without a flag.

    ros2 launch mini_ranka_bridge twin.launch.py source:=live
"""

import rclpy
from std_msgs.msg import Float64

from .joint_state_source import JointStateSource
from .kinematics import JOINT_LIMITS, motor_to_joint


class JointStateBridge(JointStateSource):
    def __init__(self) -> None:
        super().__init__("joint_state_bridge")

        ns = str(self.get_parameter("namespace").value).strip("/")
        rate = float(self.declare_parameter("rate", 50.0).value)
        # Copy of HOME_OFFSET_1..3 in config.h. Only consulted in meas* fallback.
        self.home_offsets = [
            float(v) for v in self.declare_parameter(
                "home_offsets", [3.7675, -6.7613, 3.4452]).value
        ]
        # Warn if the wire goes quiet — otherwise RViz just freezes mid-pose and
        # looks like a rendering problem rather than a disconnected arm.
        self.stale_after = float(self.declare_parameter("stale_after", 1.0).value)

        self._q = [None, None, None]        # from q1/q2/q3
        self._meas = [None, None, None]     # from meas1/meas2/meas3
        self._direct = False                # upgraded on first q* message
        self._last_rx = None
        self._stale = False
        self._subs = []

        def topic(name: str) -> str:
            return f"/{ns}/{name}" if ns else f"/{name}"

        for i in range(3):
            self._subs.append(self.create_subscription(
                Float64, topic(f"q{i + 1}"), self._on_q(i), 10))
            self._subs.append(self.create_subscription(
                Float64, topic(f"meas{i + 1}"), self._on_meas(i), 10))

        self.create_timer(1.0 / rate if rate > 0 else 0.02, self._tick)
        self.get_logger().info(
            f"joint_state_bridge: {topic('q<n>')} (preferred) or "
            f"{topic('meas<n>')} -> /joint_states at {rate:.0f} Hz"
        )

    # ------------------------------------------------------------------
    def _on_q(self, i: int):
        def cb(msg: Float64) -> None:
            if not self._direct:
                self._direct = True
                self.get_logger().info(
                    "q1/q2/q3 present — using the firmware's joint angles "
                    "directly; home_offsets parameter ignored"
                )
            self._q[i] = msg.data
            self._mark_alive()
        return cb

    def _on_meas(self, i: int):
        def cb(msg: Float64) -> None:
            self._meas[i] = msg.data
            self._mark_alive()
        return cb

    def _mark_alive(self) -> None:
        self._last_rx = self.get_clock().now()
        if self._stale:
            self._stale = False
            self.get_logger().info("telemetry resumed")

    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if self._last_rx is None:
            return  # nothing has ever arrived; stay quiet rather than publish zeros

        age = (self.get_clock().now() - self._last_rx).nanoseconds * 1e-9
        if age > self.stale_after:
            if not self._stale:
                self._stale = True
                self.get_logger().warning(
                    f"no telemetry for {age:.1f}s — is the arm powered and "
                    "telemetry_bridge connected? Holding the last pose."
                )
            return

        if self._direct:
            if any(v is None for v in self._q):
                return
            q = tuple(self._q)
        else:
            if any(v is None for v in self._meas):
                return
            q = tuple(motor_to_joint(i, self._meas[i], self.home_offsets[i])
                      for i in range(3))

        for i, (v, (lo, hi)) in enumerate(zip(q, JOINT_LIMITS)):
            if not lo - 0.1 <= v <= hi + 0.1:
                # Not clamped: this is measurement, and hiding it would make a
                # miscalibrated home offset look like a healthy arm.
                self.get_logger().warning(
                    f"joint{i + 1} reads {v:.3f} rad, outside the config.h "
                    f"limits [{lo}, {hi}] — check the home offsets",
                    throttle_duration_sec=5.0,
                )

        self.publish_joints(q)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateBridge()
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
