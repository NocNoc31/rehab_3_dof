from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Tìm kiếm các gói
    rehab_arm_robot_moveit_config_pkg = FindPackageShare('rehab_arm_robot')
    can_odrive_interface_pkg = FindPackageShare('can_odrive_interface')
    
    # Path tới file URDF/Xacro của robot
    robot_description_path = PathJoinSubstitution([
        rehab_arm_robot_moveit_config_pkg, "urdf", "rehab_arm_robot_real.urdf.xacro"
    ])
    
    # Path tới file SRDF của robot
    robot_description_semantic_path = PathJoinSubstitution([
        "rehab_arm_robot_moveit_config", "config", "rehab_arm_robot.srdf"
    ])

    # Path tới file cấu hình ros2_controllers.yaml
    ros2_controllers_path = PathJoinSubstitution([
        can_odrive_interface_pkg, "config", "rehab_arm_controllers.yaml"
    ])

    # Argument để tải robot description (nếu muốn)
    # Nếu bạn đã load robot_description qua ros2_control_node, thì MoveIt có thể lấy từ parameter server.
    # Tuy nhiên, vẫn nên load trong MoveIt configs để đảm bảo.
    load_robot_description = LaunchConfiguration("load_robot_description", default="true")

    # Node Robot State Publisher để xuất bản các transformation của robot
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {'robot_description': LaunchConfiguration('robot_description_content')} # Sẽ được nạp bởi ros2_control_node
        ],
        output="screen",
    )

    # Node ROS 2 Control (Controller Manager) để tải hardware_interface và các controller
    # quan trọng: 'rehab_arm_robot_hardware' là tên của hardware_interface được định nghĩa trong file YAML
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description_path, # Tải URDF vào robot_description parameter
            ros2_controllers_path   # Tải cấu hình controller vào parameter
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", "info"], # Tăng level log để debug
    )

    # Spawner để kích hoạt Joint State Broadcaster
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"], # -c chỉ định tên controller manager
        output="screen",
    )

    # Spawner để kích hoạt controller điều khiển quỹ đạo khớp của bạn
    load_rehab_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rehab_arm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Khởi chạy MoveIt 2 demo launch.py
    # Bạn cần đảm bảo demo.launch.py có thể đọc được robot_description và các controllers từ parameter server
    # Hoặc truyền chúng qua launch_arguments
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                rehab_arm_robot_moveit_config_pkg, "launch", "demo.launch.py"
            ])
        ]),
        launch_arguments={
            "load_robot_description": load_robot_description,
            # Nếu demo.launch.py yêu cầu đường dẫn trực tiếp, bạn có thể truyền nó
            # "robot_description_file": robot_description_path,
            # "robot_description_semantic_file": robot_description_semantic_path,
            "ros2_controllers_path": ros2_controllers_path, # Truyền đường dẫn cấu hình controller
            # Các tham số khác của demo.launch.py bạn muốn ghi đè
            "use_sim_time": "false", # Không dùng thời gian mô phỏng khi chạy trên phần cứng thật
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_description_content",
            default_value=[robot_description_path], # URDF được tải bởi ros2_control_node
            description="Path to robot URDF/xacro file",
        ),
        robot_state_publisher_node,
        ros2_control_node,
        load_joint_state_broadcaster,
        load_rehab_arm_controller,
        moveit_demo_launch,
    ])