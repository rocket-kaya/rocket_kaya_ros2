import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    log_level = 'info'

    camera_config = PathJoinSubstitution(
        [
            FindPackageShare("rocket_kaya_vision"),
            "config",
            "vision_pipeline_d435i.yaml",
        ]
    )

    depth2laser_config = PathJoinSubstitution(
        [
            FindPackageShare("rocket_kaya_vision"),
            "config",
            "vision_pipeline_depth2laser.yaml",
        ]
    )

    rocket_kaya_camera = ComposableNode(
        package="realsense2_camera",
        plugin='realsense2_camera::RealSenseNodeFactory',
        name="camera",
        namespace="vision",
        parameters=[camera_config],
    )

    depth2laser = ComposableNode(
        package="depthimage_to_laserscan",
        plugin='depthimage_to_laserscan::DepthImageToLaserScanROS',
        name="depth2laser",
        namespace="vision",
        parameters=[depth2laser_config],
        remappings=[("/vision/depth", "/vision/aligned_depth_to_color/image_raw"), ("/vision/depth_camera_info", "/vision/aligned_depth_to_color/camera_info")],
    )

    """Generate launch description with multiple components."""
    vision_pipeline = ComposableNodeContainer(
        name='vision_pipeline',
        namespace='vision',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            rocket_kaya_camera,
            depth2laser,
        ],
        output="both",
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True,
    )

    return launch.LaunchDescription([vision_pipeline])
