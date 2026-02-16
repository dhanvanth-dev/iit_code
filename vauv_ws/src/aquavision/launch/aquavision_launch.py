"""
Aquavision 2.0 Launch File.

Launches all three nodes (vision, nav_bridge, mission) with
configurable parameters for the autonomous gate traversal system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the full Aquavision stack."""

    # ── Vision Node ─────────────────────────────────────────────────
    vision_node = Node(
        package='aquavision',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{
            'model_path': 'best.pt',
            'target_gate_class': 'green_gate',
            'confidence_threshold': 0.5,
            'inference_rate': 10.0,
        }],
    )

    # ── Navigation Bridge Node ──────────────────────────────────────
    nav_bridge_node = Node(
        package='aquavision',
        executable='nav_bridge_node',
        name='nav_bridge_node',
        output='screen',
        parameters=[{
            'mavlink_connection': '/dev/ttyACM0',
            'mavlink_baud': 115200,
            'heartbeat_rate': 1.0,
            'telemetry_rate': 10.0,
            'control_rate': 20.0,
        }],
    )

    # ── Mission Node (State Machine) ────────────────────────────────
    mission_node = Node(
        package='aquavision',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[{
            'tick_rate': 20.0,
            'descend_duration': 5.0,
            'descend_thrust': 700,
            'stabilize_duration': 3.0,
            'search_yaw_speed': 200,
            'align_threshold_x': 0.05,
            'align_threshold_y': 0.05,
            'align_stable_count': 10,
            'thrust_forward_speed': 400,
            'thrust_width_threshold': 0.8,
            'blind_pass_duration': 5.0,
            'blind_pass_speed': 500,
            'pid_yaw_kp': 400.0,
            'pid_yaw_ki': 0.0,
            'pid_yaw_kd': 50.0,
            'pid_vertical_kp': 300.0,
            'pid_vertical_ki': 0.0,
            'pid_vertical_kd': 30.0,
            'neutral_z': 500,
        }],
    )

    return LaunchDescription([
        nav_bridge_node,
        vision_node,
        mission_node,
    ])
