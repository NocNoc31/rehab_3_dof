#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "odrive_can_interface/odrive_can_interface.hpp"
#include <memory>
#include <vector>

class ArmOdriveBridge : public rclcpp::Node {
public:
  ArmOdriveBridge() : Node("arm_odrive_bridge") {
    // Initialize ODrive CAN interfaces (one per motor/joint)
    odrives_ = {
      std::make_shared<ODriveCANInterface>("/odrive/velocity_0", "can0", 0), // Node ID 0 for joint1
      std::make_shared<ODriveCANInterface>("/odrive/velocity_1", "can0", 1)  // Node ID 1 for joint2
      // Add more for additional joints
    };

    // Configure ODrive motors
    for (auto& odrive : odrives_) {
      odrive->clear_errors();
      odrive->set_axis_state(8); // AXIS_STATE_CLOSED_LOOP_CONTROL
      odrive->set_controller_mode(3, 1); // POSITION_CONTROL, INPUT_MODE_PASSTHROUGH
      odrive->set_velocity_limit(10.0); // Adjust as needed
      odrive->set_torque_limit(1.0);    // Adjust as needed
    }

    // Subscriber for MoveIt2 joint trajectories
    trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/rehab_arm_moveit_controller/joint_trajectory", 1,
        std::bind(&ArmOdriveBridge::trajectory_callback, this, std::placeholders::_1));

    // Publisher for joint states
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Timer to publish joint states periodically
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                              std::bind(&ArmOdriveBridge::publish_joint_states, this));
  }

private:
  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    for (const auto& point : msg->points) {
      for (size_t i = 0; i < msg->joint_names.size() && i < odrives_.size(); ++i) {
        // Send position command to ODrive (position in turns)
        float position = point.positions[i]; // Convert radians to turns if needed
        odrives_[i]->set_input_position(position, 0, 0); // Adjust velocity/torque feedforward if needed
      }
    }
  }

  void publish_joint_states() {
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = this->get_clock()->now();
    joint_state.name = {"joint1", "joint2"}; // Match your joint names

    // Get encoder estimates from each ODrive
    for (const auto& odrive : odrives_) {
      auto estimates = odrive->get_encoder_estimates();
      joint_state.position.push_back(estimates.pos); // Position in turns (convert to radians if needed)
      joint_state.velocity.push_back(estimates.vel);
    }

    joint_state_pub_->publish(joint_state);
  }

  std::vector<std::shared_ptr<ODriveCANInterface>> odrives_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmOdriveBridge>());
  rclcpp::shutdown();
  return 0;
}




