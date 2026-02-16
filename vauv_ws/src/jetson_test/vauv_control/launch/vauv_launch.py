from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # MAVROS Node (using config from your original utils.py/config.yaml)
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[{
                'fcu_url': 'serial:///dev/ttyACM0:115200', # Update based on your config.yaml
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
            }]
        ),
        # Mission Control Node
        Node(
            package='vauv_control',
            executable='mission_node',
            name='mission_manager',
            output='screen'
        ),
        # Logger Node
        Node(
            package='vauv_control',
            executable='logger_node',
            name='data_logger',
            output='screen'
        )
    ])