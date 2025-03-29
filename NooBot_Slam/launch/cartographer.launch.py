import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource


config_dir = os.path.join(get_package_share_directory("noobot_slam"), "config")
configuration_basename = "cartographer.lua"

resolution = "0.05"
publish_period_sec = "0.5"

bringup_dir = get_package_share_directory("noobot_link")
launch_dir = os.path.join(bringup_dir, "launch")


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "noobot_link.launch.py")
                )
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                parameters=[{"use_sim_time": False}],
                arguments=[
                    "-configuration_directory",
                    config_dir,
                    "-configuration_basename",
                    configuration_basename,
                ],
                remappings=[("odom", "odom"), ("imu", "/imu/data")],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="occupancy_grid_node",
                parameters=[{"use_sim_time": False}],
                arguments=[
                    "-resolution",
                    resolution,
                    "-publish_period_sec",
                    publish_period_sec,
                ],
            ),
        ]
    )
