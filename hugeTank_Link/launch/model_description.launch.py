import os

from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_robot_node(robot_urdf,child):
    return launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'robot_state_publisher_{child}',
        arguments=[os.path.join(get_package_share_directory('wheeltec_robot_urdf'), 'urdf', robot_urdf)],
    )

def generate_static_transform_publisher_node(translation, rotation, parent, child):
    return launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'base_to_{child}',
        arguments=[translation[0], translation[1], translation[2], rotation[0], rotation[1], rotation[2], parent, child],
    )
    
def generate_launch_description():
    hugeTank = LaunchConfiguration('hugeTank', default='true')
   
    hugeTank_Action = GroupAction(
        condition=IfCondition(hugeTank),
        actions=[
        generate_robot_node('flagship_four_wheel_diff_dl_robot.urdf','flagship_4wheel_diff_dl'),
        generate_static_transform_publisher_node(['0.272', '0', '0.257'], ['0', '0', '0'], 'base_footprint', 'laser'),
        generate_static_transform_publisher_node(['0.272', '0', '0.25'], ['0', '0', '0'], 'base_footprint', 'camera_link'),    
    ]) 

     
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(hugeTank_Action)
    return ld
