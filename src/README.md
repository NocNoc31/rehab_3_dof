3DoF Rehabilitation Arm Robot
This repository contains the software stack for controlling a 3-degree-of-freedom (3DoF) rehabilitation robotic arm integrated with MoveIt2, RViz2, and ODrive motor controllers via CAN communication. The project is built for ROS 2 and provides tools for motion planning, obstacle visualization, and joint state monitoring.
Repository Structure

rehab_arm_robot_control: Contains nodes and launch files for controlling the robotic arm, including the custom RRT planner and ODrive CAN bridge.
rehab_arm_robot_colision: Configures and visualizes obstacles in RViz2 for collision-aware motion planning.
odrive_can_interface: Provides a CAN interface library for communicating with ODrive motor controllers.

Prerequisites

OS: Ubuntu 22.04 (Jammy) or later
ROS 2: Humble or Iron
Dependencies:
MoveIt2
ros2_control
can_msgs
ODrive CAN interface tools
RViz2


Hardware:
3DoF robotic arm with ODrive motor controllers
CAN interface (e.g., USB-CAN adapter like CANable)


CAN Setup:
sudo ip link set can0 up type can bitrate 1000000



Installation

Clone the Repository:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repository_url>


Build the Workspace:
cd ~/ros2_ws
colcon build
source install/setup.bash


Install Dependencies:Ensure all ROS 2 dependencies are installed:
rosdep install --from-paths src --ignore-src -r -y



ls /dev/ttyACM*
sudo slcan_attach -f -s6 -o /dev/ttyACM0
sudo slcand ttyACM0 can0
sudo ip link set can0 up
candump can0




Usage
1. Launch the Robot with MoveIt2
Start the robot with MoveIt2 for motion planning and visualization in RViz2:
ros2 launch rehab_arm_robot_moveit_config demo_with_controller.launch.py

This command:

Initializes the robot's MoveIt2 configuration.
Starts the ros2_control interface for joint trajectory control.
Launches RViz2 with the robot model and obstacle visualization.

2. Run the Custom RRT Planner
Launch the custom RRT (Rapidly-exploring Random Tree) planner for motion planning:
ros2 launch rehab_arm_robot_control rrt_planner.launch.py

This runs the RRT planner node within the rehab_arm_robot_control package.
3. Set a Specific Goal for the RRT Planner
To specify a goal for the RRT planner:
ros2 param set /rehab_arm_rrt_planner state_cmd 2

This sets the planner to target a predefined goal state (e.g., a specific joint configuration or end-effector pose).
4. Read Joint Positions and States
Monitor the robot's joint positions and states:
ros2 run rehab_arm_robot_control state_reader

This node subscribes to /joint_states and outputs the current positions and velocities of the robot's joints.
CAN Communication
The odrive_can_interface package enables communication with ODrive motor controllers over the CAN bus. Ensure the CAN interface (can0) is active before launching nodes:
candump can0  # Verify CAN bus activity

Troubleshooting

CAN Interface Issues:
Check if can0 is up: ip link show can0.
Ensure the correct bitrate (1 Mbps for ODrive).
Verify ODrive motor IDs match those in arm_odrive_bridge.cpp.


MoveIt2 Planning Failures:
Confirm the URDF and SRDF in rehab_arm_robot_moveit_config match your robot.
Check for obstacles in RViz2 (configured in rehab_arm_robot_colision).


Joint State Errors:
Ensure the state_reader node is receiving /joint_states messages.
Verify ODrive encoder feedback in arm_odrive_bridge.



Contributing
Contributions are welcome! Please submit pull requests or open issues for bugs, feature requests, or improvements.
License
This project is licensed under the MIT License. See the LICENSE file for details.
Contact
For questions or support, contact the project maintainer at nc3112@example.com.