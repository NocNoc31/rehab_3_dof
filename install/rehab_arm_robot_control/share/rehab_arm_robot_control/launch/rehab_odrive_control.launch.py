from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to MoveIt2 demo launch file
    moveit_launch_file = os.path.join(
        get_package_share_directory('rehab_arm_robot_moveit_config'),
        'launch',
        'demo_with_controller.launch.py'
    )

    # Include MoveIt2 demo launch
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file)
    )

    # Arm ODrive bridge node
    arm_odrive_bridge = Node(
        package='rehab_arm_robot_control',
        executable='arm_odrive_bridge',
        name='arm_odrive_bridge',
        output='screen'
    )

    return LaunchDescription([
        moveit_launch,
        arm_odrive_bridge
    ])