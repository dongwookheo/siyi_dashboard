"""Launches the dashboard backend + image bridge.

External pieces (run separately):
  * MediaMTX:           scripts/run_mediamtx.sh
  * realsense2_camera:  ros2 launch realsense2_camera rs_launch.py \\
                          initial_reset:=true colorizer.enable:=true \\
                          pointcloud.enable:=true \\
                          rgb_camera.color_profile:=640,480,30 \\
                          depth_module.depth_profile:=640,480,30
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="siyi_dashboard",
                executable="dashboard_node",
                name="siyi_dashboard",
                output="screen",
                parameters=[
                    {
                        "pointcloud_topic": "/Laser_map",
                        "pointcloud_max_points": 24576,
                        "pointcloud_rate_hz": 6.0,
                    }
                ],
            ),
            Node(
                package="siyi_dashboard",
                executable="image_bridge",
                name="image_bridge",
                output="screen",
            ),
        ]
    )
