
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("assem_tong", package_name="rehab_arm_robot_moveit_config")
        .robot_description(file_path="/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/rehab_arm_robot.urdf.xacro")  # tùy nếu bạn dùng xacro
        .robot_description_semantic(file_path="/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/rehab_arm_robot.srdf")
        .robot_description_kinematics(file_path="/home/nocc/Desktop/3dof_rehab_arm/src/rehab_arm_robot_moveit_config/config/kinematics.yaml")
        .planning_pipelines()  # nạp mặc định OMPL pipeline
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)









