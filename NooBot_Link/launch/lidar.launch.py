from os import path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    share_dir = get_package_share_directory("noobot_link")

    parameter_file = LaunchConfiguration("params_file")
    params_declare = DeclareLaunchArgument(
        "params_file",
        default_value=path.join(share_dir, "config", "lidar.yaml"),
        description="FPath to the ROS2 parameters file to use.",
    )

    ydlidar_node = LifecycleNode(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[parameter_file],
        namespace="/",
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(params_declare)
    ld.add_action(ydlidar_node)

    return ld
 