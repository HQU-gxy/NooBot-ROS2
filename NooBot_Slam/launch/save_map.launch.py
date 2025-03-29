import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions

share_dir = get_package_share_directory("noobot_sla")
map_dir = os.path.join(share_dir, "map", "fucked_map")


def generate_launch_description():

    map_saver = launch_ros.actions.Node(
        package="nav2_map_server",
        executable="map_saver_cli",
        output="screen",
        arguments=["-f", map_dir],
        parameters=[{"save_map_timeout": 20000.0}, {"free_thresh_default": 0.196}],
    )
    map_backup = launch_ros.actions.Node(
        package="nav2_map_server",
        executable="map_saver_cli",
        name="map_backup",
        output="screen",
        arguments=["-f", map_dir],
        parameters=[{"save_map_timeout": 20000.0}, {"free_thresh_default": 0.196}],
    )
    ld = LaunchDescription()

    ld.add_action(map_saver)
    ld.add_action(map_backup)
    return ld