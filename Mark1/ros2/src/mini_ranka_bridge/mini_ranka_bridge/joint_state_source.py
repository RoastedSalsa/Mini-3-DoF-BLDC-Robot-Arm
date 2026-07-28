"""Shared base for anything that drives the twin's joints.

Both sources of /joint_states — the mock arm and the live telemetry bridge —
publish the same two things, so the plumbing lives here and they only have to
work out the joint angles:

  * ``/joint_states`` (sensor_msgs/JointState) for robot_state_publisher, which
    turns it into TF and hence into the arm you see in RViz.
  * ``<ns>/tool_path`` (nav_msgs/Path), a rolling trace of where the tool has
    been. The RViz config picks this up as the orange "Tool trace" line.

Only joint1/2/3 are published. joint2_belt is a `mimic` in the URDF, so
robot_state_publisher derives the pulley itself — publishing it here would be
redundant and could contradict the model.
"""

from collections import deque

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .kinematics import JOINT_NAMES, fk


class JointStateSource(Node):
    """Node base that publishes /joint_states and a tool trace."""

    def __init__(self, name: str) -> None:
        super().__init__(name)

        ns = str(self.declare_parameter("namespace", "mini_ranka").value).strip("/")
        self.frame_id = str(
            self.declare_parameter("base_frame", "base_link").value
        )
        # 0 disables the trace. 500 samples at 50 Hz is the last ~10 s of motion.
        self.path_length = int(self.declare_parameter("path_length", 500).value)

        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._path_pub = self.create_publisher(
            Path, f"/{ns}/tool_path" if ns else "/tool_path", 10
        )
        self._path: deque[PoseStamped] = deque(maxlen=max(self.path_length, 1))

    # ------------------------------------------------------------------
    def publish_joints(self, q, velocities=None) -> None:
        """Publish one joint-state sample and extend the tool trace.

        `q` is (q1, q2, q3) in the IK frame — the same convention as the
        firmware's q1/q2/q3 and as the URDF's joint1/joint2/joint3.
        """
        stamp = self.get_clock().now().to_msg()

        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in q]
        if velocities is not None:
            msg.velocity = [float(v) for v in velocities]
        self._joint_pub.publish(msg)

        if self.path_length <= 0:
            return

        x, y, z = fk(*q)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self._path.append(pose)

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id
        path.poses = list(self._path)
        self._path_pub.publish(path)
