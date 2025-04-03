import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    noobot_launch_dir = os.path.join(
        get_package_share_directory("noobot_link"), "launch"
    )
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    noobot_nav_dir = get_package_share_directory("noobot_nav2")
    param_dir = os.path.join(noobot_nav_dir, "config")
    param_file = LaunchConfiguration(
        "params", default=os.path.join(param_dir, "nav2_params.yaml")
    )

    noobot_slam_dir = get_package_share_directory("noobot_slam")
    map_file = LaunchConfiguration(
        "map", default=os.path.join(noobot_slam_dir, "map", "fucked_map.yaml")
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=map_file,
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "params",
                default_value=param_file,
                description="Full path to param file to load",
            ),
            Node(
                name="waypoint_cycle",
                package="nav2_waypoint_cycle",
                executable="nav2_waypoint_cycle",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [noobot_launch_dir, "/noobot_link.launch.py"]
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_bringup_launch_dir, "/bringup_launch.py"]
                ),
                launch_arguments={
                    "map": map_file,
                    "use_sim_time": "false",
                    "params_file": param_file,
                }.items(),
            ),
        ]
    )
