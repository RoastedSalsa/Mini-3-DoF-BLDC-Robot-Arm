#!/usr/bin/env python3
"""Drive the twin with no hardware attached.

Runs the firmware's own trajectory + IK on the host and publishes the resulting
joint angles, so the whole visualisation stack — robot_state_publisher, RViz,
the tool trace — can be developed and demoed with the arm on the bench in
pieces. Same waypoints, same segment time, same elbow-down IK branch as
`main.cpp`, so what you see here is what the arm will do.

    ros2 run mini_ranka_bridge mock_arm
    ros2 launch mini_ranka_bridge twin.launch.py source:=mock

It is an ideal arm: no tracking error, no compliance, no gravity sag. It tells
you the model and the trajectory are right, not that the controller is.
"""

import rclpy

from .joint_state_source import JointStateSource
from .kinematics import (JOINT_LIMITS, TRAJ_SEGMENT_TIME_S, clamp_to_limits, ik,
                         trajectory_at)


class MockArm(JointStateSource):
    def __init__(self) -> None:
        super().__init__("mock_arm")

        rate = float(self.declare_parameter("rate", 50.0).value)
        self.segment_time = float(
            self.declare_parameter("segment_time", TRAJ_SEGMENT_TIME_S).value
        )
        # The firmware defaults to LINEAR easing; SMOOTH is the cubic smoothstep
        # (zero velocity at each waypoint) that lib/Trajectory also offers.
        self.smooth = bool(self.declare_parameter("smooth", False).value)
        # Flat [x,y,z,x,y,z,...]; empty means the config.h waypoints.
        raw = list(self.declare_parameter("waypoints", []).value or [])
        self.points = (tuple(tuple(raw[i:i + 3]) for i in range(0, len(raw), 3))
                       if len(raw) >= 6 else None)
        if raw and self.points is None:
            self.get_logger().warning(
                f"waypoints needs at least 2 xyz triples, got {len(raw)} values; "
                "falling back to the config.h trajectory"
            )

        self._t = 0.0
        self._dt = 1.0 / rate if rate > 0 else 0.02
        self._last_q = (0.0, 0.0, 0.0)
        self._warned_unreachable = False

        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f"mock_arm: {rate:.0f} Hz, {self.segment_time:.1f} s/segment, "
            f"{'smooth' if self.smooth else 'linear'} easing -> /joint_states"
        )

    def _tick(self) -> None:
        kwargs = {"segment_time_s": self.segment_time, "smooth": self.smooth}
        if self.points:
            kwargs["points"] = self.points
        x, y, z = trajectory_at(self._t, **kwargs)

        q = ik(x, y, z)
        if q is None:
            # Hold the last good pose rather than publishing NaN, which would
            # make the arm disappear from RViz with no obvious cause.
            if not self._warned_unreachable:
                self.get_logger().warning(
                    f"waypoint ({x:.3f}, {y:.3f}, {z:.3f}) is out of reach; "
                    "holding last pose"
                )
                self._warned_unreachable = True
            q = self._last_q
        else:
            self._warned_unreachable = False
            clamped = clamp_to_limits(q)
            if clamped != q:
                self.get_logger().warning(
                    f"IK solution {tuple(round(v, 3) for v in q)} exceeds the "
                    f"config.h travel limits {JOINT_LIMITS}; clamping",
                    throttle_duration_sec=5.0,
                )
            q = clamped
            self._last_q = q

        self.publish_joints(q)
        self._t += self._dt


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockArm()
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
