from launch import LaunchDescription
import launch_ros.actions

#def launch(launch_descriptor, argv):
def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='hugetank_link', 
            executable='wheeltec_robot_node', 
            output='screen',
            parameters=[{'usart_port_name': '/dev/ttyTank',
                'serial_baud_rate': 115200,
                'robot_frame_id': 'base_footprint',
                'odom_frame_id': 'odom_combined',
                'cmd_vel': 'cmd_vel',
                'akm_cmd_vel': 'none',
                'product_number': 0,}],
            )


  ])
