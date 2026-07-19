"""Launch the Mini_Ranka telemetry bridge.

Examples:
    ros2 launch mini_ranka_bridge bridge.launch.py
    ros2 launch mini_ranka_bridge bridge.launch.py port:=/dev/ttyACM1 baud:=115200

Selecting which dynamics-feedforward terms run, without reflashing (see
firmware/lib/Dynamics for what each term is):

    ros2 launch mini_ranka_bridge bridge.launch.py \\
        startup_commands:="['DN', 'DG', 'DI', 'F']"
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
        # Commander lines replayed after each (re)connect, e.g. "['DN','DG','F']".
        DeclareLaunchArgument("startup_commands", default_value="[]"),
        DeclareLaunchArgument("startup_delay", default_value="2.0"),
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
            "startup_commands": LaunchConfiguration("startup_commands"),
            "startup_delay": LaunchConfiguration("startup_delay"),
        }],
    )

    return LaunchDescription(args + [bridge])
