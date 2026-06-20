"""Launch the Mini_Ranka telemetry bridge.

Examples:
    ros2 launch mini_ranka_bridge bridge.launch.py
    ros2 launch mini_ranka_bridge bridge.launch.py port:=/dev/ttyACM1 baud:=115200
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baud", default_value="921600"),
        DeclareLaunchArgument("namespace", default_value="mini_ranka"),
        DeclareLaunchArgument("reconnect_period", default_value="2.0"),
    ]

    bridge = Node(
        package="mini_ranka_bridge",
        executable="telemetry_bridge",
        name="telemetry_bridge",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "baud": LaunchConfiguration("baud"),
            "namespace": LaunchConfiguration("namespace"),
            "reconnect_period": LaunchConfiguration("reconnect_period"),
        }],
    )

    return LaunchDescription(args + [bridge])
