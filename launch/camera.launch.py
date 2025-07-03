from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    camera_manager = Node(
        package="bluerov2_camera",
        executable="camera_manager",
        name="camera_manager_node",
    )

    camera_viewer = Node(
        package="bluerov2_camera",
        executable="camera_viewer",
        name="camera_viewer_node",
    )

    return LaunchDescription([
        camera_manager,
        camera_viewer,
    ])
