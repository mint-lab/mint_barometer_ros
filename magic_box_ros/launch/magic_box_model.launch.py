import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro


def generate_launch_description():
    package_name = "magic_box_ros"
    xacro_file = os.path.join(
        get_package_share_directory(package_name), "config", "magic_box.urdf.xacro"
    )
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), "rviz", "test_urdf.rviz"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": Command(["xacro ", LaunchConfiguration("xacro")])}
        ],
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("gui")),
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz")],
    )

    gui_argument = DeclareLaunchArgument(
        name="gui",
        default_value="True",
        description="Flag to enable joint_state_publisher_gui",
    )
    xacro_argument = DeclareLaunchArgument(
        name="xacro",
        default_value=xacro_file,
        description="Xacro file to be loaded by robot_state_publisher",
    )
    rviz_argument = DeclareLaunchArgument(
        name="rviz",
        default_value=rviz_config_file,
        description="RViz config file",
    )

    return LaunchDescription(
        [
            gui_argument,
            xacro_argument,
            rviz_argument,
            robot_state_publisher_node,
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
