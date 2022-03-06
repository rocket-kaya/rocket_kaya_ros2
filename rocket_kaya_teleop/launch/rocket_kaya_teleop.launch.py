import os

from symbol import parameters
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import Command
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    rocket_kaya_joy_config = PathJoinSubstitution(
        [
            FindPackageShare("rocket_kaya_teleop"),
            "config",
            "rocket_kaya_joy.yaml",
        ]
    )

    rocket_kaya_teleop_config = PathJoinSubstitution(
        [
            FindPackageShare("rocket_kaya_teleop"),
            "config",
            "rocket_kaya_teleop.yaml",
        ]
    )

    rocket_kaya_joy = Node(
        package="joy",
#        namespace="rocket_kaya",
        executable="joy_node",
        name="rocket_kaya_joy",
        parameters=[rocket_kaya_joy_config],
        output="both",
    )

    rocket_kaya_teleop = Node(
        package="joy_teleop",
#        namespace="rocket_kaya",
        executable="joy_teleop",
        name="rocket_kaya_teleop",
        parameters=[rocket_kaya_teleop_config],
        output="both",
    )

    return LaunchDescription([
        rocket_kaya_joy,
        rocket_kaya_teleop,
    ])
