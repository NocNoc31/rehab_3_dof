import launch
import launch_ros.actions
from launch import LaunchDescription

def generate_launch_description():
    # Danh sách các node
    nodes = [
        launch_ros.actions.Node(
            package='data_pub',
            executable='data_pub_node',
            name='data_pub_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='impedance_control',
            executable='impedance_control_node',
            name='impedance_control_node',
            output='screen',
        )
    ]

    # Trả về LaunchDescription chứa các node
    return LaunchDescription(nodes)