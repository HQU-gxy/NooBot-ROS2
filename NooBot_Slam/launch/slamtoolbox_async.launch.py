from ament_index_python.packages import get_package_share_directory
from os import path
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    noobot_link_dir = get_package_share_directory("noobot_link")
    slam_toolbox_dir = get_package_share_directory("noobot_slam")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    path.join(noobot_link_dir, "launch", "noobot_link.launch.py")
                )
            ),
            launch_ros.actions.Node(
                parameters=[
                    path.join(slam_toolbox_dir, "config", "toolbox_params.yaml")
                ],
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                remappings=[("odom", "odom")],
            ),
        ]
    )
