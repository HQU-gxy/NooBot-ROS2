import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("noobot_link")
    launch_dir = os.path.join(bringup_dir, "launch")

    imu_config = Path(get_package_share_directory("noobot_link"), "config", "imu.yaml")
    ekf_config = Path(get_package_share_directory("noobot_link"), "config", "ekf.yaml")

    imu_filter_node = launch_ros.actions.Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[imu_config],
    )

    ekf_node = launch_ros.actions.Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[ekf_config],
        remappings=[("/odometry/filtered", "odom_filtered")],
    )

    link_node = launch_ros.actions.Node(
        package="noobot_link",
        executable="noobot_node",
        output="screen",
        parameters=[
            {
                "serial_port": "/dev/ttyNooBot",
                "serial_baud_rate": 115200,
                "odom_topic": "odom",
                "odom_frame_id": "odom",
                "base_frame_id": "base_footprint",
                "gyro_frame_id": "gyro_link",
            }
        ],
    )

    base_to_gyro = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_gyro",
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "gyro_link"],
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "lidar.launch.py"))
    )

    bot_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "model_description.launch.py")
        )
    )

    ld = LaunchDescription()

    ld.add_action(bot_model)
    ld.add_action(link_node)
    ld.add_action(base_to_gyro)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(imu_filter_node)
    ld.add_action(ekf_node)
    ld.add_action(lidar_driver)

    return ld
