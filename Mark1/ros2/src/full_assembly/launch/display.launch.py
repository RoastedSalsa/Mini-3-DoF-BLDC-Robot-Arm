"""Show the Onshape CAD export of the arm in RViz.

    ros2 launch full_assembly display.launch.py

The joint_state_publisher_gui sliders drive the four continuous joints; drop it
with gui:=false if you only want the zero pose.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("full_assembly"))
    urdf = share / "urdf" / "full_assembly.urdf"
    rviz_config = share / "rviz" / "full_assembly.rviz"

    gui = LaunchConfiguration("gui")

    args = [
        DeclareLaunchArgument("gui", default_value="true"),
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf.read_text()}],
    )

    # Either the slider GUI or the headless publisher — never both, they would
    # fight over /joint_states.
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(gui),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(rviz_config)],
    )

    return LaunchDescription(
        args
        + [robot_state_publisher, joint_state_publisher_gui, joint_state_publisher, rviz]
    )
