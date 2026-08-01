"""Bring up the digital twin: the model in RViz, driven by joints.

    ros2 launch mini_ranka_bridge twin.launch.py                  # mock, no hardware
    ros2 launch mini_ranka_bridge twin.launch.py source:=live     # the real arm
    ros2 launch mini_ranka_bridge twin.launch.py source:=live port:=/dev/ttyACM1

source:=mock  replays the config.h trajectory through the host-side IK. Nothing
              is connected; this is the mode to develop and demo in.
source:=live  opens the serial port, fans telemetry out to topics (so
              PlotJuggler still works as before), and adapts the joint angles
              into /joint_states.

This is the composition layer: mini_ranka_description knows nothing about
telemetry, and telemetry_bridge knows nothing about the robot model. Both are
wired together only here.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    source = LaunchConfiguration("source")
    namespace = LaunchConfiguration("namespace")

    is_mock = IfCondition(PythonExpression(["'", source, "' == 'mock'"]))
    is_live = IfCondition(PythonExpression(["'", source, "' == 'live'"]))

    args = [
        DeclareLaunchArgument(
            "source", default_value="mock", choices=["mock", "live"],
            description="Where joint angles come from: the host-side mock or "
                        "the real arm over serial.",
        ),
        DeclareLaunchArgument("namespace", default_value="mini_ranka"),
        DeclareLaunchArgument("rviz", default_value="true"),
        # Same file display.launch.py would have picked, spelled out here so the
        # session launcher can swap in the bare-viewport config instead.
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("mini_ranka_description"),
                "rviz", "mini_ranka.rviz",
            ]),
            description="RViz config to open. session.rviz drops the side docks "
                        "and leaves only the 3D view.",
        ),
        # --- live only ---
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baud", default_value="921600"),
        DeclareLaunchArgument("startup_commands", default_value="[]"),
        # --- mock only ---
        DeclareLaunchArgument("rate", default_value="50.0"),
        DeclareLaunchArgument(
            "smooth", default_value="false",
            description="Cubic smoothstep easing instead of the firmware's "
                        "default linear interpolation.",
        ),
    ]

    # jsp:=none — one of the nodes below owns /joint_states instead.
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("mini_ranka_description"), "launch", "display.launch.py",
        ])),
        launch_arguments={
            "jsp": "none",
            "rviz": LaunchConfiguration("rviz"),
            "rviz_config": LaunchConfiguration("rviz_config"),
        }.items(),
    )

    mock_arm = Node(
        package="mini_ranka_bridge",
        executable="mock_arm",
        name="mock_arm",
        output="screen",
        parameters=[{
            "namespace": namespace,
            "rate": LaunchConfiguration("rate"),
            "smooth": LaunchConfiguration("smooth"),
        }],
        condition=is_mock,
    )

    # Unchanged from bridge.launch.py — the twin reuses the same telemetry fan-out,
    # so PlotJuggler and ros2 bag keep working while RViz is up.
    telemetry_bridge = Node(
        package="mini_ranka_bridge",
        executable="telemetry_bridge",
        name="telemetry_bridge",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "baud": LaunchConfiguration("baud"),
            "namespace": namespace,
            "startup_commands": LaunchConfiguration("startup_commands"),
        }],
        condition=is_live,
    )

    joint_state_bridge = Node(
        package="mini_ranka_bridge",
        executable="joint_state_bridge",
        name="joint_state_bridge",
        output="screen",
        parameters=[{"namespace": namespace}],
        condition=is_live,
    )

    return LaunchDescription(
        args + [display, mock_arm, telemetry_bridge, joint_state_bridge]
    )
