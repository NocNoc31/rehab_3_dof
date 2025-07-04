


 
# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
# from launch.conditions import IfCondition
# from launch.event_handlers import OnProcessExit
# from launch.events import Shutdown
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from moveit_configs_utils import MoveItConfigsBuilder
# import xacro
 
 
# def generate_launch_description():
 
#     # Constants for paths to different files and folders
#     package_name_moveit_config = 'rehab_arm_robot_moveit_config'
 
#     # Set the path to different files and folders
#     pkg_share_moveit_config = FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config)
 
#     # Paths for various configuration files
#     urdf_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot/urdf/rehab_arm_robot.urdf.xacro'
#     srdf_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/rehab_arm_robot.srdf'
#     moveit_controllers_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/moveit_controllers.yaml'
#     joint_limits_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/joint_limits.yaml'
#     kinematics_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/kinematics.yaml'
#     pilz_cartesian_limits_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/pilz_cartesian_limits.yaml'
#     initial_positions_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/initial_positions.yaml'
#     rviz_config_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/moveit.rviz'
 
#     # Set the full paths
#     srdf_model_path = os.path.join(pkg_share_moveit_config, srdf_file_path)
#     moveit_controllers_file_path = os.path.join(pkg_share_moveit_config, moveit_controllers_file_path)
#     joint_limits_file_path = os.path.join(pkg_share_moveit_config, joint_limits_file_path)
#     kinematics_file_path = os.path.join(pkg_share_moveit_config, kinematics_file_path)
#     pilz_cartesian_limits_file_path = os.path.join(pkg_share_moveit_config, pilz_cartesian_limits_file_path)
#     initial_positions_file_path = os.path.join(pkg_share_moveit_config, initial_positions_file_path)
#     rviz_config_file = os.path.join(pkg_share_moveit_config, rviz_config_file_path)
 
#     # Launch configuration variables
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     use_rviz = LaunchConfiguration('use_rviz')
 
#     # Declare the launch arguments
#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         name='use_sim_time',
#         default_value='true',
#         description='Use simulation (Gazebo) clock if true')
 
#     declare_use_rviz_cmd = DeclareLaunchArgument(
#         name='use_rviz',
#         default_value='true',
#         description='Whether to start RViz')
 
#     # Load the robot configuration
#     # Typically, you would also have this line in here: .robot_description(file_path=urdf_model_path)
#     # Another launch file is launching the robot description.
#     moveit_config = (
#         MoveItConfigsBuilder("rehab_arm_robot", package_name=package_name_moveit_config)
#         .trajectory_execution(file_path=moveit_controllers_file_path)
#         .robot_description_semantic(file_path=srdf_model_path)
#         .joint_limits(file_path=joint_limits_file_path)
#         .robot_description_kinematics(file_path=kinematics_file_path)
#         .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
#         # .planning_scene_monitor(
#         #     publish_robot_description=False,
#         #     publish_robot_description_semantic=True,
#         #     publish_planning_scene=True,
#         # )
#         # .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
#         .to_moveit_configs()
#     )
     
#     # Start the actual move_group node/action server
#     start_move_group_node_cmd = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[
#             moveit_config.to_dict(),
#             {'use_sim_time': use_sim_time},
#             {'start_state': {'content': initial_positions_file_path}},
#         ],
#     )
 
#     # RViz
#     start_rviz_node_cmd = Node(
#         condition=IfCondition(use_rviz),
#         package="rviz2",
#         executable="rviz2",
#         arguments=["-d", rviz_config_file],
#         output="screen",
#         parameters=[
#             moveit_config.robot_description,
#             moveit_config.robot_description_semantic,
#             moveit_config.planning_pipelines,
#             moveit_config.robot_description_kinematics,
#             moveit_config.joint_limits,
#             {'use_sim_time': use_sim_time}
#         ],
#     )
     
#     exit_event_handler = RegisterEventHandler(
#         condition=IfCondition(use_rviz),
#         event_handler=OnProcessExit(
#             target_action=start_rviz_node_cmd,
#             on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
#         ),
#     )
    
 
    
#     # Create the launch description and populate
#     ld = LaunchDescription()
 
#     # Declare the launch options
#     ld.add_action(declare_use_sim_time_cmd)
#     ld.add_action(declare_use_rviz_cmd)
 
#     # Add any actions
#     ld.add_action(start_move_group_node_cmd)
#     ld.add_action(start_rviz_node_cmd)
     
#     # Clean shutdown of RViz
#     ld.add_action(exit_event_handler)
 
#     return ld








# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
# from launch.conditions import IfCondition
# from launch.event_handlers import OnProcessExit
# from launch.events import Shutdown
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from moveit_configs_utils import MoveItConfigsBuilder
# import xacro


# def generate_launch_description():

#     # Constants for paths to different files and folders
#     package_name_moveit_config = 'rehab_arm_robot_moveit_config'
#     package_name_odrive_publisher = 'odrive_joint_state_publisher' # <--- Đảm bảo tên package này chính xác

#     # Set the path to different files and folders
#     pkg_share_moveit_config = FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config)
#     pkg_share_odrive_publisher = FindPackageShare(package=package_name_odrive_publisher).find(package_name_odrive_publisher) # <--- Thêm dòng này

#     # Paths for various configuration files
#     # Cân nhắc sử dụng os.path.join với FindPackageShare cho các file này để có tính di động cao hơn
#     # Ví dụ: urdf_file_path = os.path.join(FindPackageShare('rehab_arm_robot').find('rehab_arm_robot'), 'urdf', 'rehab_arm_robot.urdf.xacro')
#     # Hiện tại, tôi sẽ giữ các đường dẫn tuyệt đối như bạn đã cung cấp, nhưng hãy lưu ý điểm này.
#     urdf_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot/urdf/rehab_arm_robot.urdf.xacro'
#     srdf_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/rehab_arm_robot.srdf'
#     moveit_controllers_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/moveit_controllers.yaml'
#     joint_limits_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/joint_limits.yaml'
#     kinematics_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/kinematics.yaml'
#     pilz_cartesian_limits_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/pilz_cartesian_limits.yaml'
#     initial_positions_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/initial_positions.yaml'
#     rviz_config_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/moveit.rviz'

#     # Set the full paths. Lưu ý: các dòng này có thể gây lỗi nếu bạn đã dùng đường dẫn tuyệt đối ở trên.
#     # Nếu urdf_file_path, srdf_file_path, v.v. đã là đường dẫn đầy đủ, không cần os.path.join với pkg_share_moveit_config nữa.
#     # Giả định bạn muốn các file cấu hình nằm trong thư mục share của package moveit_config.
#     srdf_model_path = os.path.join(pkg_share_moveit_config, 'config', 'rehab_arm_robot.srdf')
#     moveit_controllers_file_path = os.path.join(pkg_share_moveit_config, 'config', 'moveit_controllers.yaml')
#     joint_limits_file_path = os.path.join(pkg_share_moveit_config, 'config', 'joint_limits.yaml')
#     kinematics_file_path = os.path.join(pkg_share_moveit_config, 'config', 'kinematics.yaml')
#     pilz_cartesian_limits_file_path = os.path.join(pkg_share_moveit_config, 'config', 'pilz_cartesian_limits.yaml')
#     initial_positions_file_path = os.path.join(pkg_share_moveit_config, 'config', 'initial_positions.yaml')
#     rviz_config_file = os.path.join(pkg_share_moveit_config, 'config', 'moveit.rviz')

#     # Launch configuration variables
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     use_rviz = LaunchConfiguration('use_rviz')

#     # Declare the launch arguments
#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         name='use_sim_time',
#         default_value='true',
#         description='Use simulation (Gazebo) clock if true')

#     declare_use_rviz_cmd = DeclareLaunchArgument(
#         name='use_rviz',
#         default_value='true',
#         description='Whether to start RViz')

#     # Generate robot_description from xacro file
#     robot_description_content = xacro.process_file(urdf_file_path).toprettyxml(indent='  ')
#     robot_description = {'robot_description': robot_description_content}

#     # Load the robot configuration
#     moveit_config = (
#         MoveItConfigsBuilder("rehab_arm_robot", package_name=package_name_moveit_config)
#         .robot_description(file_path=urdf_file_path) # <--- Quan trọng: Đảm bảo robot_description được tải
#         .trajectory_execution(file_path=moveit_controllers_file_path)
#         .robot_description_semantic(file_path=srdf_model_path)
#         .joint_limits(file_path=joint_limits_file_path)
#         .robot_description_kinematics(file_path=kinematics_file_path)
#         .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
#         .to_moveit_configs()
#     )

#     # Start the actual move_group node/action server
#     start_move_group_node_cmd = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[
#             moveit_config.to_dict(),
#             robot_description, # <--- Pass the robot_description dictionary
#             {'use_sim_time': use_sim_time},
#             # {'start_state': {'content': initial_positions_file_path}}, # <--- Dòng này thường chỉ dùng cho fake controller hoặc initial pose trong MoveIt, không phải cho robot thật. Joint states thật sẽ do joint_state_publisher cung cấp.
#         ],
#     )

#     # RViz
#     start_rviz_node_cmd = Node(
#         condition=IfCondition(use_rviz),
#         package="rviz2",
#         executable="rviz2",
#         arguments=["-d", rviz_config_file],
#         output="screen",
#         parameters=[
#             moveit_config.robot_description, # <--- Thêm dòng này để RViz có robot_description
#             moveit_config.robot_description_semantic,
#             moveit_config.planning_pipelines,
#             moveit_config.robot_description_kinematics,
#             moveit_config.joint_limits,
#             {'use_sim_time': use_sim_time}
#         ],
#     )

#     # <--- THÊM CÁC NODE SAU ĐỂ HỆ THỐNG HOẠT ĐỘNG --->

#     # # Launch your ODrive Joint State Publisher Node
#     # start_odrive_joint_state_publisher_node = Node(
#     #     package=package_name_odrive_publisher,
#     #     executable="odrive_joint_state_publisher_node", # <--- Tên executable của bạn
#     #     output="screen",
#     #     parameters=[{'use_sim_time': use_sim_time}],
#     # )

#     # Robot State Publisher (quan trọng để xuất bản TF frames)
#     start_robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="screen",
#         parameters=[robot_description, {'use_sim_time': use_sim_time}],
#     )

#     exit_event_handler = RegisterEventHandler(
#         condition=IfCondition(use_rviz),
#         event_handler=OnProcessExit(
#             target_action=start_rviz_node_cmd,
#             on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
#         ),
#     )

#     # Create the launch description and populate
#     ld = LaunchDescription()

#     # Declare the launch options
#     ld.add_action(declare_use_sim_time_cmd)
#     ld.add_action(declare_use_rviz_cmd)

#     # Add any actions
#     ld.add_action(start_robot_state_publisher_node) # <--- Thêm node này
#     # ld.add_action(start_odrive_joint_state_publisher_node) # <--- Thêm node này
#     ld.add_action(start_move_group_node_cmd)
#     ld.add_action(start_rviz_node_cmd)

#     # Clean shutdown of RViz
#     ld.add_action(exit_event_handler)

#     return ld





import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def generate_launch_description():

    # Constants for paths to different files and folders
    package_name_moveit_config = 'rehab_arm_robot_moveit_config'
    package_name_odrive_publisher = 'odrive_joint_state_publisher'

    # Set the path to different files and folders
    pkg_share_moveit_config = FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config)

    # Paths for various configuration files
    urdf_file_path = '/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot/urdf/rehab_arm_robot.urdf.xacro'
    srdf_model_path = os.path.join(pkg_share_moveit_config, 'config', 'rehab_arm_robot.srdf')
    moveit_controllers_file_path = os.path.join(pkg_share_moveit_config, 'config', 'moveit_controllers.yaml')
    joint_limits_file_path = os.path.join(pkg_share_moveit_config, 'config', 'joint_limits.yaml')
    kinematics_file_path = os.path.join(pkg_share_moveit_config, 'config', 'kinematics.yaml')
    pilz_cartesian_limits_file_path = os.path.join(pkg_share_moveit_config, 'config', 'pilz_cartesian_limits.yaml')
    initial_positions_file_path = os.path.join(pkg_share_moveit_config, 'config', 'initial_positions.yaml')
    rviz_config_file = os.path.join(pkg_share_moveit_config, 'config', 'moveit.rviz')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',  # Đổi thành false cho robot thật
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz')

    # Generate robot_description from xacro file
    robot_description_content = xacro.process_file(urdf_file_path).toprettyxml(indent='  ')
    robot_description = {'robot_description': robot_description_content}

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder("rehab_arm_robot", package_name=package_name_moveit_config)
        .robot_description(file_path=urdf_file_path)
        .trajectory_execution(file_path=moveit_controllers_file_path)
        .robot_description_semantic(file_path=srdf_model_path)
        .joint_limits(file_path=joint_limits_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    # Robot State Publisher (quan trọng để xuất bản TF frames)
    start_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # Launch your ODrive Joint State Publisher Node
    start_odrive_joint_state_publisher_node = Node(
        package=package_name_odrive_publisher,
        executable="odrive_joint_state_publisher_node",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        # Thêm respawn để tự động khởi động lại nếu node bị crash
        respawn=True,
        respawn_delay=2.0,
    )

    # Start the actual move_group node/action server
    start_move_group_node_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            robot_description,
            {'use_sim_time': use_sim_time},
            # Thêm các tham số timeout để tránh lỗi
            {'trajectory_execution.allowed_execution_duration_scaling': 2.0},
            {'trajectory_execution.allowed_goal_duration_margin': 2.0},
        ],
    )

    # RViz
    start_rviz_node_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time}
        ],
    )

    exit_event_handler = RegisterEventHandler(
        condition=IfCondition(use_rviz),
        event_handler=OnProcessExit(
            target_action=start_rviz_node_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
        ),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Add nodes in correct order
    ld.add_action(start_robot_state_publisher_node)
    ld.add_action(start_odrive_joint_state_publisher_node)
    ld.add_action(start_move_group_node_cmd)
    ld.add_action(start_rviz_node_cmd)

    # Clean shutdown of RViz
    ld.add_action(exit_event_handler)

    return ld