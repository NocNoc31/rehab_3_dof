from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Đường dẫn tới package rehab_arm_robot_moveit_config
    moveit_config_package = get_package_share_directory('rehab_arm_robot_moveit_config')

    return LaunchDescription([
        # Node position_node từ can_odrive_interface
        Node(
            package='can_odrive_interface',
            executable='position_node',
            name='position_node',
            output='screen',
            emulate_tty=True,
        ),

        # Node moveit_odrive_bridge từ rehab_arm_robot_control
        Node(
            package='rehab_arm_robot_control',
            executable='moveit_odrive_bridge',
            name='moveit_odrive_bridge',
            output='screen',
            emulate_tty=True,
        ),

        # Node planning từ rehab_arm_robot_control
        Node(
            package='rehab_arm_robot_control',
            executable='planning',
            name='planning',
            output='screen',
            emulate_tty=True,
        ),

        # Include file launch demo_with_controller.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_config_package, 'launch', 'demo_with_controller.launch.py')
            )
        ),
    ])