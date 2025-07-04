
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float32.hpp>

class MoveItODriveBridge : public rclcpp::Node {
public:
    MoveItODriveBridge() : Node("moveit_odrive_bridge") {
        // Subscriber cho topic từ MoveIt2
        trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/rehab_arm_moveit_controller/joint_trajectory", 10,
            std::bind(&MoveItODriveBridge::trajectory_callback, this, std::placeholders::_1));

        // Publisher cho topic /desired_position
        position_pub_ = create_publisher<std_msgs::msg::Float32>("/desired_position", 10);

        RCLCPP_INFO(this->get_logger(), "MoveIt-ODrive Bridge Node started");
    }

private:
    void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        // Kiểm tra joint_names có chứa joint_1
        auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), "joint_1");
        if (it == msg->joint_names.end()) {
            RCLCPP_ERROR(this->get_logger(), "joint_1 not found in joint_trajectory");
            return;
        }
        size_t joint_index = std::distance(msg->joint_names.begin(), it);

        // Lấy vị trí của joint_1 từ điểm đầu tiên trong quỹ đạo
        if (!msg->points.empty()) {
            float joint_position_rad = msg->points[0].positions[joint_index];
            std_msgs::msg::Float32 pos_msg;
            pos_msg.data = joint_position_rad;
            position_pub_->publish(pos_msg);
            RCLCPP_INFO(this->get_logger(), "Published joint_1 position: %.2f rad to /desired_position", joint_position_rad);
        } else {
            RCLCPP_WARN(this->get_logger(), "Received empty joint_trajectory");
        }
    }

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveItODriveBridge>());
    rclcpp::shutdown();
    return 0;
}








