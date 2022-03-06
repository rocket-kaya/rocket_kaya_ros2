from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import os
import xacro
from symbol import parameters

def generate_launch_description():

    # arg_show_rviz = DeclareLaunchArgument(
        # "start_rviz",
        # default_value="false",
        # description="start RViz automatically with the launch file",
    # )

    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory("rocket_kaya_description"),
        "urdf",
        "rocket_kaya.urdf",
    )
    robot_description = {"robot_description": xacro.process_file(robot_description_path).toxml()}

    # robot_description_content = ""
    # robot_description = {"robot_description": robot_description_content}

    rocket_kaya_controller = PathJoinSubstitution(
        [
            FindPackageShare("rocket_kaya_controller"),
            "config",
            "rocket_kaya_controller.yaml",
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, rocket_kaya_controller],
        remappings=[("/rocket_kaya_controller/cmd_vel", "/cmd_vel")],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rocket_kaya_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )

    # spawn_jsb_controller = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["joint_state_broadcaster"],
    #     output="both",
    # )

    # rviz_config_file = PathJoinSubstitution(
        # [FindPackageShare("diffbot_description"), "config", "diffbot.rviz"]
    # )
    # rviz_node = Node(
        # package="rviz2",
        # executable="rviz2",
        # name="rviz2",
        # arguments=["-d", rviz_config_file],
        # condition=IfCondition(LaunchConfiguration("start_rviz")),
    # )

    return LaunchDescription(
        [
            # arg_show_rviz,
            control_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            robot_controller_spawner,
            # rviz_node,
        ]
    )
