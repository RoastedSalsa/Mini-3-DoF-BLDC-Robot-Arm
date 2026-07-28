"""Show the Mini_Ranka model in RViz.

    ros2 launch mini_ranka_description display.launch.py

Publishes /robot_description and TF, and opens RViz. By default the slider GUI
drives the joints, which is the standalone "look at the model" mode.

This launch deliberately knows nothing about the arm's telemetry — it only needs
*something* to publish /joint_states. To drive it from the real arm or from the
mock, use the bringup launch instead, which reuses this file with jsp:=none:

    ros2 launch mini_ranka_bridge twin.launch.py source:=mock
    ros2 launch mini_ranka_bridge twin.launch.py source:=live

jsp:=gui   slider GUI (default)
jsp:=idle  headless publisher, holds the zero pose
jsp:=none  publish nothing — for when another node owns /joint_states
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("mini_ranka_description"))
    urdf = share / "urdf" / "mini_ranka.urdf"

    jsp = LaunchConfiguration("jsp")
    rviz_config = LaunchConfiguration("rviz_config")

    args = [
        DeclareLaunchArgument(
            "jsp", default_value="gui",
            choices=["gui", "idle", "none"],
            description="What publishes /joint_states: slider GUI, a headless "
                        "publisher holding zero, or nothing (external source).",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Open RViz. false when you only want the model and TF.",
        ),
        DeclareLaunchArgument(
            "rviz_config", default_value=str(share / "rviz" / "mini_ranka.rviz"),
        ),
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        # Read at launch rather than passed as a path: robot_state_publisher
        # wants the URDF text, and RViz picks it up off /robot_description.
        parameters=[{"robot_description": urdf.read_text()}],
    )

    # Exactly one of these may run — two publishers would fight over /joint_states.
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(PythonExpression(["'", jsp, "' == 'gui'"])),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=IfCondition(PythonExpression(["'", jsp, "' == 'idle'"])),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        args + [robot_state_publisher, joint_state_publisher_gui,
                joint_state_publisher, rviz]
    )
