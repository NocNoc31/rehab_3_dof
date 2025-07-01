from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odrive_can_control',  # Tên package
            executable='odrive_initializer',  # Tên executable
            name='odrive_initializer',  # Tên node
            output='screen',  # Hiển thị log ra terminal
            parameters=[
                {"control_mode_node_0": "velocity"},    # Node 0: TORQUE_CONTROL torque  velocity
                {"control_mode_node_1": "velocity"}     # Node 1: TORQUE_CONTROL
            ]
        )
    ])