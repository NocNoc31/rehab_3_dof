from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'max_iterations',
            default_value='1000',
            description='Maximum RRT iterations'
        ),
        
        DeclareLaunchArgument(
            'step_size',
            default_value='0.1',
            description='RRT step size'
        ),
        
        Node(
            package='rehab_arm_robot_control',
            executable='rrt_motion_planning',
            name='rrt_planner',
            output='screen',
            parameters=[{
                'max_iterations': LaunchConfiguration('max_iterations'),
                'step_size': LaunchConfiguration('step_size'),
                'goal_tolerance': 0.05
            }]
        )
    ])