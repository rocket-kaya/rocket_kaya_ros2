from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    log_level = 'info'

    rocket_kaya_vision_config = PathJoinSubstitution(
        [
            FindPackageShare("rocket_kaya_vision"),
            "config",
            "d435i.yaml",
        ]
    )

    rocket_kaya_camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        parameters=[rocket_kaya_vision_config],
        output="both",
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            rocket_kaya_camera,
        ]
    )
