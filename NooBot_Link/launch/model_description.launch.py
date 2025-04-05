from os import path

from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription


def generate_launch_description():
    urdf_path = path.join(
        get_package_share_directory("noobot_link"), "urdf", "noobot.urdf"
    )

    robot_state_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_noobot",
        arguments=[urdf_path],
    )

    base_to_laser = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser",
        arguments=["0.15", "0", "0.14", "0", "0", "0", "base_footprint", "laser"],
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
        arguments=[urdf_path],
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(robot_state_node)
    ld.add_action(base_to_gyro)
    ld.add_action(base_to_laser)
    ld.add_action(joint_state_publisher_node)
    return ld
