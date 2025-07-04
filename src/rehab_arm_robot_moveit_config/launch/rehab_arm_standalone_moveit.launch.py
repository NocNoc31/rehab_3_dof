

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, Command 

# from launch_ros.actions import Node
# from launch_ros.parameter_descriptions import ParameterValue

# import os

# def generate_launch_description():
#     declared_arguments = []

#     declared_arguments.append(
#         DeclareLaunchArgument(
#             "rviz_config",
#             default_value="/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/moveit.rviz", 
#             description="Path to the RViz config file",
#         )
#     )

#     # --- BƯỚC QUAN TRỌNG: TIỀN XỬ LÝ XACRO THÀNH URDF ---
#     # Đường dẫn tuyệt đối đến file XACRO gốc của bạn
#     robot_description_xacro_file = "/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot/urdf/rehab_arm_robot.urdf.xacro"
    
#     robot_description_content = ParameterValue(
#         Command(['xacro ', robot_description_xacro_file]),
#         value_type=str 
#     )
#     # --- KẾT THÚC BƯỚC TIỀN XỬ LÝ XACRO ---

#     # --- BƯỚC QUAN TRỌNG: ĐỌC NỘI DUNG SRDF TRỰC TIẾP VÀ CHỈ ĐỊNH KIỂU CHUỖI ---
#     # Đường dẫn tuyệt đối đến file SRDF của bạn
#     robot_description_semantic_file = "/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/rehab_arm_robot.srdf"
    
#     # Đọc nội dung file SRDF trực tiếp và bọc nó trong ParameterValue để chỉ định kiểu là chuỗi
#     robot_description_semantic_content = ParameterValue(
#         Command(['cat ', robot_description_semantic_file]),
#         value_type=str 
#     )
#     # --- KẾT THÚC BƯỚC ĐỌC SRDF TRỰC TIẾP ---

#     # Các file cấu hình MoveIt khác (vẫn dùng đường dẫn tuyệt đối như bạn muốn)
#     kinematics_yaml = "/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/kinematics.yaml"
#     joint_limits_yaml = "/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/joint_limits.yaml"
#     chomp_planning_yaml = "/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/chomp_planning.yaml"
#     ompl_planning_yaml = "/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/ompl_planning.yaml"
    
#     # DÒNG NÀY ĐÃ ĐƯỢC ĐỔI TÊN ĐỂ TRÁNH NHẦM LẪN VỚI moveit_controllers YAML (mà không phải là một tham số trực tiếp)
#     # Tên tham số đúng là moveit_simple_controller_manager, như đã giải thích ở trên.
#     # Đảm bảo file ros2_controllers.yaml này chứa định nghĩa bộ điều khiển giả lập.
#     ros2_controllers_yaml = "/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/ros2_controllers.yaml"

#     # Các tham số cho move_group_node
#     move_group_params = {
#         "robot_description": robot_description_content, 
#         "robot_description_semantic": robot_description_semantic_content, 
#         "robot_description_kinematics": kinematics_yaml,
#         "joint_limits": joint_limits_yaml,
#         "chomp": chomp_planning_yaml,
#         "ompl": ompl_planning_yaml,
#         # >>> THAY ĐỔI QUAN TRỌNG NHẤT Ở ĐÂY <<<
#         # Đây là plugin MoveIt sẽ sử dụng để giao tiếp với bộ điều khiển của bạn
#         "rehab_arm_moveit_controller": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
#         # Đây là đường dẫn đến file YAML định nghĩa các bộ điều khiển cho MoveItSimpleControllerManager
#         "rehab_arm_moveit_controller": ros2_controllers_yaml, 
#         # >>> KẾT THÚC THAY ĐỔI QUAN TRỌNG <<<
#         # "use_sim_time": LaunchConfiguration("use_sim_time"),
#     }

#     # Node Move Group
#     move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[move_group_params],
#         arguments=["--ros-args", "--log-level", "info"],
#     )

#     # Node Robot State Publisher
#     robot_state_publisher_node = Node(
#         package="robot_state_publisher",
#         executable="robot_state_publisher",
#         output="screen",
#         parameters=[
#             {"robot_description": robot_description_content}, 
#             # {"use_sim_time": LaunchConfiguration("use_sim_time")}
#         ],
#     )

#     # Node RViz2
#     rviz_node = Node(
#         package="rviz2",
#         executable="rviz2",
#         name="rviz2",
#         output="log",
#         arguments=["-d", LaunchConfiguration("rviz_config")],
#         parameters=[
#             {"robot_description": robot_description_content}, 
#             {"robot_description_semantic": robot_description_semantic_content}, 
#             {"robot_description_kinematics": kinematics_yaml},
#             {"planning_pipelines": ["ompl"]}, # Hoặc "chomp" nếu bạn muốn dùng CHOMP planner mặc định
#         ],
#     )

#     return LaunchDescription(declared_arguments + [
#         robot_state_publisher_node,
#         move_group_node,
#         rviz_node,
#     ])





# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rehab_arm_robot"),
                    "urdf",
                    "rehab_arm_robot_real.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("rehab_arm_robot_moveit_config"),
            "config",
            "ros2_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("rehab_arm_robot_moveit_config"), "config", "moveit.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)