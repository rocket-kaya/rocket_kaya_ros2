import os

from ament_index_python import get_package_share_directory

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
    
    log_level = 'info'

    cmd_vel_mux_config = PathJoinSubstitution(
        [
            FindPackageShare('rocket_kaya_bringup'),
            'config',
            'rocket_kaya_cmd_vel_mux.yaml',
        ]
    )
    
    base_launch_path = PathJoinSubstitution(
        [
            FindPackageShare('rocket_kaya_controller'),
            'launch',
            'rocket_kaya_controller.launch.py',
        ]
    )

    cmd_vel_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='cmd_vel_mux',
        # remappings=[('/cmd_vel_out', '/cmd_vel')],
        parameters=[cmd_vel_mux_config],
        output='both',
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True,
    )
    
    base_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(base_launch_path))
    
    return LaunchDescription([
        cmd_vel_mux,
        base_launch,
    ])
